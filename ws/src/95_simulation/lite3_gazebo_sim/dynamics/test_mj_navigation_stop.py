"""Pure planner/controller tests. No MuJoCo solver, ONNX inference, ROS or hardware."""
import contextlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mj_navigation_stop as nav
import mj_step_baseline as baseline


def data(time=0., x=0., y=0., yaw=0.):
    return SimpleNamespace(time=time, qpos=np.array([x, y, .35, np.cos(yaw / 2),
                                                   0., 0., np.sin(yaw / 2)]))


def scan(stamp, points=()):
    return {'stamp': stamp, 'xyz': list(points), 'ok': True,
            'rays': [{'valid': True, 'hit': bool(points)} for _ in range(9)]}


def trace():
    return [{'time': t, 'xyz': [x, 0., .35], 'base_vxy': [v, 0.],
             'robot_aabb': [[x - .2, -.2, .05], [x + .2, .2, .5]]}
            for t, x, v in ((0., 0., .3), (1., .3, .3), (1.25, .34, .01),
                            (1.5, .342, 0.), (1.75, .343, 0.), (2., .343, 0.))]


class ControllerTests(unittest.TestCase):
    def test_clear_strip_real_planners_and_committed_segment(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        with patch.object(nav.local, 'plan', wraps=nav.local.plan) as planning:
            self.assertGreater(controller(None, data())[0], 0.)
            committed = controller.result
            self.assertGreater(controller(None, data(.2))[0], 0.)
            self.assertIs(controller.result, committed)
            self.assertEqual(planning.call_count, 1)
            for time in (.4, .6, .8):
                self.assertGreater(controller(None, data(time))[0], 0.)  # expiry replan, not a contact transfer
            self.assertEqual(planning.call_count, 2)
        self.assertEqual(len(controller.global_plans), 5)
        self.assertTrue(controller.local_plans[0]['trajectory'])
        self.assertTrue(controller.map['source']['uri'].startswith('fixture://declared_flat_floor'))

    def test_measured_blockage_real_no_route_revokes_and_latches(self):
        def sensor(model, d):
            return scan(d.time, [[1.9, y, .35] for y in np.linspace(-.4, .4, 9)] if d.time >= .2 else [])
        controller = nav.NavigationController(sensor=sensor)
        self.assertGreater(controller(None, data(x=1.37))[0], 0.)
        np.testing.assert_array_equal(controller(None, data(.2, x=1.37)), np.zeros(3))
        self.assertEqual(controller.latch['reason'], 'E_LOCAL_NO_ROUTE')
        self.assertIsNone(controller.result)
        self.assertEqual(controller.first_zero, {'time': .2, 'reason': 'E_LOCAL_NO_ROUTE'})
        self.assertTrue(controller.global_plans[-1]['path'])
        self.assertEqual(controller.local_plans[-1]['reason'], 'E_LOCAL_NO_ROUTE')
        np.testing.assert_array_equal(controller(None, data(.3, x=1.37)), np.zeros(3))

    def test_lateral_velocity_is_explicitly_preserved(self):
        vector = [.3, .01, 0.]
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        with patch.object(nav.local, 'track_segment', return_value={
                'velocity': vector, 'reason': 'track', 'stop': False}):
            np.testing.assert_allclose(controller(None, data()), vector)
        self.assertIsNone(controller.latch)
        self.assertEqual(controller.decisions[-1]['local_xyz'], vector)
        self.assertEqual(controller.decisions[-1]['body_command'], vector)
        self.assertEqual(controller.decisions[-1]['vx'], vector[0])

    def test_world_xy_rotates_by_observed_yaw_with_bounded_heading_feedback(self):
        vector = [.2, .1, 0.]
        for yaw in (.1, -.1, .15):
            with self.subTest(yaw=yaw):
                controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
                state = data(yaw=yaw)
                original_pose = state.qpos.copy()
                with patch.object(nav.local, 'track_segment', return_value={
                        'velocity': vector, 'reason': 'track', 'stop': False}):
                    command = controller(None, state)
                expected = [np.cos(yaw) * vector[0] + np.sin(yaw) * vector[1],
                            -np.sin(yaw) * vector[0] + np.cos(yaw) * vector[1], -yaw]
                np.testing.assert_allclose(command, expected)
                self.assertLessEqual(abs(command[2]), .15)
                np.testing.assert_array_equal(state.qpos, original_pose)
                self.assertIsNone(controller.latch)

    def test_nonzero_z_nonfinite_shape_and_speed_norm_fail_closed(self):
        for vector in ([.3, 0., .01], [.3, 0., 1e-12], [.3, 0., float('nan')],
                       [float('inf'), 0., 0.], [0., float('nan'), 0.], [.3, .3, 0.],
                       [0.], [.1, 0., 0., .1], [[.1, 0., 0.]]):
            with self.subTest(vector=vector):
                controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
                with patch.object(nav.local, 'track_segment', return_value={
                        'velocity': vector, 'reason': 'track', 'stop': False}):
                    np.testing.assert_array_equal(controller(None, data()), np.zeros(3))
                self.assertEqual(controller.latch['reason'], 'unsupported_axis_or_velocity')
                self.assertEqual(controller.decisions[-1]['local_xyz'], vector)

    def test_first_zero_requires_all_body_axes_to_withdraw(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        tracked = [{'velocity': v, 'reason': 'track', 'stop': False}
                   for v in ([0., 0., 0.], [.3, 0., 0.], [0., .1, 0.], [0., 0., 0.])]
        with patch.object(nav.local, 'track_segment', side_effect=tracked):
            np.testing.assert_array_equal(controller(None, data()), np.zeros(3))
            self.assertIsNone(controller.first_zero)
            self.assertGreater(controller(None, data(.02))[0], 0.)
            np.testing.assert_allclose(controller(None, data(.04)), [0., .1, 0.])
            self.assertIsNone(controller.first_zero)
            np.testing.assert_array_equal(controller.previous_command, [0., .1, 0.])
            np.testing.assert_array_equal(controller(None, data(.06)), np.zeros(3))
        self.assertEqual(controller.first_zero, {'time': .06, 'reason': 'track'})

    def test_heading_only_command_is_not_zero_and_latch_withdraws_it(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        with patch.object(nav.local, 'track_segment', return_value={
                'velocity': [0., 0., 0.], 'reason': 'hold', 'stop': True}):
            np.testing.assert_allclose(controller(None, data(yaw=.1)), [0., 0., -.1])
            self.assertIsNone(controller.first_zero)
            np.testing.assert_array_equal(controller.latch_zero(.02, 'E_LOCAL_NO_ROUTE'), np.zeros(3))
            np.testing.assert_array_equal(controller(None, data(.02, yaw=.1)), np.zeros(3))
        self.assertEqual(controller.first_zero, {'time': .02, 'reason': 'E_LOCAL_NO_ROUTE'})
        np.testing.assert_array_equal(controller.previous_command, np.zeros(3))

    def test_collision_checks_whole_remaining_segment_every_policy_tick(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        with patch.object(nav.local, 'body_collision', wraps=nav.local.body_collision) as collision:
            for time in (0., .02, .04):
                self.assertGreater(controller(None, data(time))[0], 0.)
            self.assertEqual(collision.call_count, 3)
            self.assertEqual(collision.call_args.args[1][3], .04)
            self.assertGreater(collision.call_args.args[2][0], 0.)
        with patch.object(nav.local, 'body_collision', return_value='dynamic:0'):
            np.testing.assert_array_equal(controller(None, data(.06)), np.zeros(3))
        self.assertEqual(controller.latch['reason'], 'E_LOCAL_COLLISION')

    def test_missing_stale_invalid_scan_and_watchdog_fail_closed(self):
        for sensing in (None, scan(-1.), dict(scan(0.), rays=[]),
                        scan(0., [[float('nan'), 0., .35]])):
            controller = nav.NavigationController(sensor=lambda m, d: sensing)
            np.testing.assert_array_equal(controller(None, data()), np.zeros(3))
            self.assertEqual(controller.latch['reason'], 'missing_sensing')
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        self.assertGreater(controller(None, data())[0], 0.)
        np.testing.assert_array_equal(controller(None, data(.5)), np.zeros(3))
        self.assertEqual(controller.latch['reason'], 'watchdog')

    def test_sensor_uses_nine_real_ray_calls_without_geom_id_filter(self):
        def ray(model, d, origin, direction, groups, static, exclude, geom):
            geom[0] = 123  # arbitrary object, not the owned barrier id
            return .7
        mocked = SimpleNamespace(mj_ray=Mock(side_effect=ray))
        with patch.dict(sys.modules, mujoco=mocked):
            sensed = nav.ray_scan(None, data(2.))
        self.assertEqual(mocked.mj_ray.call_count, 9)
        self.assertEqual(len(sensed['xyz']), 9)
        np.testing.assert_allclose(sensed['xyz'][0], [1.15, -.4, .35])
        bounds = nav.sensed_obstacles(sensed)[0]
        np.testing.assert_allclose(bounds['min'], [1.03, -.52, 0.])
        np.testing.assert_allclose(bounds['max'], [1.27, .52, .47])

    def test_injection_only_owned_geom_z_and_event(self):
        model = SimpleNamespace(geom_pos=np.array([[2., 0., .5]]))
        mj = SimpleNamespace(mj_name2id=Mock(return_value=0), mj_forward=Mock(),
                             mjtObj=SimpleNamespace(mjOBJ_GEOM=5))
        injection = nav.BarrierInjection(5.)
        with patch.dict(sys.modules, mujoco=mj):
            injection(model, data())
            np.testing.assert_allclose(model.geom_pos[0], [2., 0., -10.])
            injection(model, data(5.))
            injection(model, data(6.))
        np.testing.assert_allclose(model.geom_pos[0], [2., 0., .5])
        self.assertEqual(injection.spawn_time, 5.)
        self.assertEqual(mj.mj_name2id.call_count, 1)
        self.assertEqual([e['event'] for e in injection.events], ['barrier_hidden', 'barrier_spawn'])

    def test_wrapper_includes_original_and_preserves_asset_roots(self):
        with patch.object(ET, 'parse', return_value=ET.ElementTree(ET.fromstring(
                '<mujoco><compiler meshdir="../meshes"/></mujoco>'))):
            wrapped = ET.fromstring(nav.wrapper_xml('/tmp/vendor/mjcf/Lite3.xml'))
        self.assertIsNone(wrapped.find('include'))
        self.assertEqual(wrapped.find('compiler').get('meshdir'), '/tmp/vendor/meshes')
        self.assertEqual(wrapped.find('worldbody/geom').get('size'), '0.10 1.0 0.5')


class CallbackTests(unittest.TestCase):
    def stubbed_rollout(self, controller=None, *, seconds=.04, yaw=0., **kwargs):
        model = SimpleNamespace(opt=SimpleNamespace(timestep=.01), nq=19, nv=18, nu=0,
                                geom_friction=np.ones((1, 3)), geom_bodyid=np.zeros(1),
                                geom_contype=np.zeros(1), geom_conaffinity=np.zeros(1))
        d = SimpleNamespace(time=0., qpos=np.zeros(19), qvel=np.zeros(18),
                            qfrc_applied=np.zeros(18), ncon=0)
        d.qpos[3:7] = [np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)]
        session = SimpleNamespace(get_inputs=lambda: [SimpleNamespace(name='obs')],
                                  run=Mock(return_value=[np.ones((1, 12))]))
        def step(m, state):
            state.time += .01  # clock-only stub, NOT a physics simulation
        mj = SimpleNamespace(MjModel=SimpleNamespace(from_xml_path=lambda p: model),
                             MjData=lambda m: d, mj_forward=Mock(), mj_kinematics=Mock(),
                             mj_step=Mock(side_effect=step))
        ort = SimpleNamespace(set_seed=Mock(), InferenceSession=lambda *a, **k: session)
        with patch.dict(sys.modules, mujoco=mj, onnxruntime=ort), \
                patch.object(Path, 'read_bytes', return_value=b'pure-test-fixture'):
            run = baseline.rollout('unused.xml', 'unused.onnx', seconds=seconds,
                                   controller=controller, **kwargs)
        np.testing.assert_array_equal(d.qvel, np.zeros(18))
        np.testing.assert_array_equal(d.qfrc_applied[:6], np.zeros(6))
        return run, session, mj

    def test_callback_first_withdrawal_boundary_and_existing_torque_path(self):
        commands = iter([.3, 0.])
        run, session, mj = self.stubbed_rollout(lambda m, d: next(commands))
        self.assertAlmostEqual(run['trigger_time'], .02)
        self.assertEqual(run['trigger_boundary']['before']['command'], [.3, 0., 0.])
        self.assertEqual(run['trigger_boundary']['after']['command'], [0., 0., 0.])
        self.assertEqual(mj.mj_step.call_count, 4)
        self.assertEqual(session.run.call_count, 2)
        for call, vx in zip(session.run.call_args_list, (.3, 0.)):
            obs = call.args[1]['obs']
            self.assertEqual(obs.shape, (1, 45))
            self.assertAlmostEqual(float(obs[0, 6]), vx / .8)
        self.assertGreater(run['physics_trace'][-1]['tau_max'], 0.)
        self.assertLessEqual(run['physics_trace'][-1]['tau_max'], 30.)
        self.assertIsNotNone(run['braking'])

    def test_navigation_lateral_and_heading_commands_reach_policy_observation(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        world = [.2, .01, 0.]
        yaw = .1
        expected = [np.cos(yaw) * world[0] + np.sin(yaw) * world[1],
                    -np.sin(yaw) * world[0] + np.cos(yaw) * world[1], -yaw]
        with patch.object(nav.local, 'track_segment', return_value={
                'velocity': world, 'reason': 'track', 'stop': False}):
            run, session, _ = self.stubbed_rollout(controller, yaw=yaw)
        self.assertIsNone(run['trigger_time'])
        for call in session.run.call_args_list:
            np.testing.assert_allclose(call.args[1]['obs'][0, 6:9], np.array(expected) / .8,
                                       rtol=1e-6)
        for sample in run['physics_trace'][1:]:
            np.testing.assert_allclose(sample['command'], expected)
        for decision in controller.decisions:
            self.assertEqual(decision['local_xyz'], world)
            np.testing.assert_allclose(decision['body_command'], expected)

    def test_vector_withdrawal_waits_for_vy_and_wz_and_preserves_boundaries(self):
        vectors = [[.3, .02, .1], [0., .02, .1], [0., 0., 0.]]
        commands = iter(vectors)
        run, session, _ = self.stubbed_rollout(lambda m, d: next(commands), seconds=.06)
        self.assertAlmostEqual(run['trigger_time'], .04)
        self.assertEqual(run['physics_trace'][0]['command'], [0., 0., 0.])
        self.assertEqual(run['trigger_boundary']['before']['command'], vectors[1])
        self.assertEqual(run['trigger_boundary']['after']['command'], vectors[2])
        self.assertEqual(run['physics_trace'][1]['command'], vectors[0])
        self.assertEqual(run['physics_trace'][3]['command'], vectors[1])
        for call, vector in zip(session.run.call_args_list, vectors):
            np.testing.assert_allclose(call.args[1]['obs'][0, 6:9], np.array(vector) / .8,
                                       rtol=1e-6)

    def test_initial_zero_is_not_a_braking_trigger(self):
        for value in (0., [0., 0., 0.], np.zeros(3)):
            run, _, _ = self.stubbed_rollout(lambda m, d: value)
            self.assertIsNone(run['trigger_time'])

    def test_lateral_or_angular_only_command_can_trigger_full_withdrawal(self):
        for vector in ([0., .1, 0.], [0., 0., .1]):
            commands = iter([vector, np.zeros(3)])
            run, _, _ = self.stubbed_rollout(lambda m, d: next(commands))
            self.assertAlmostEqual(run['trigger_time'], .02)
            self.assertEqual(run['trigger_boundary']['before']['command'], vector)
            self.assertEqual(run['trigger_boundary']['after']['command'], [0., 0., 0.])

    def test_default_scalar_observation_scale_and_scheduled_zero_are_unchanged(self):
        for stop_at in (None, .02):
            run, session, _ = self.stubbed_rollout(stop_at=stop_at)
            self.assertEqual(run['physics_trace'][0]['command'], [.3, 0., 0.])
            for i, call in enumerate(session.run.call_args_list):
                expected = [.3 / .8 if stop_at is None or i == 0 else 0., 0., 0.]
                np.testing.assert_allclose(call.args[1]['obs'][0, 6:9], expected)
            if stop_at is None:
                self.assertIsNone(run['trigger_time'])
                self.assertIsNone(run['braking'])
            else:
                self.assertAlmostEqual(run['trigger_time'], .02)
                self.assertEqual(run['trigger_boundary']['before']['command'], [.3, 0., 0.])
                self.assertEqual(run['trigger_boundary']['after']['command'], [0., 0., 0.])

    def test_scalar_limits_and_mutual_exclusion(self):
        for value in (float('nan'), float('inf'), .41, -.41, [0.], True, np.bool_(False), '0', 1j, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.stubbed_rollout(lambda m, d: value)
        with self.assertRaises(ValueError):
            baseline.rollout('unused', 'unused', controller=lambda m, d: 0., stop_at=1.)
        for value in (True, .3, 'controller'):
            with self.assertRaises(ValueError):
                baseline.rollout('unused', 'unused', controller=value)

    def test_vector_types_shapes_nonfinite_bools_and_each_axis_limit_rejected(self):
        invalid = ([0., 0.], [0., 0., 0., 0.], [[0., 0., 0.]], [[0.], [0.], [0.]],
                   np.array(.3), [[], 0., 0.], [0., True, 0.], np.ones(3, dtype=bool),
                   [0., 0., '0'], [0., 0., 1j], {'vx': .3}, [0., object(), 0.])
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.stubbed_rollout(lambda m, d: value)
        for axis in range(3):
            for value in (.41, -.41, float('nan'), float('inf'), -float('inf')):
                vector = np.zeros(3)
                vector[axis] = value
                with self.subTest(axis=axis, value=value), self.assertRaises(ValueError):
                    self.stubbed_rollout(lambda m, d: vector)

    def test_scalar_and_vector_numeric_boundary_types_accepted(self):
        for value in (.4, -.4, 0, np.int64(0), np.float32(.3), [.4, -.4, .4],
                      (.1, -.1, .1), np.array([.4, -.4, .4])):
            run, session, _ = self.stubbed_rollout(lambda m, d: value)
            expected = [float(value), 0., 0.] if np.isscalar(value) else value
            np.testing.assert_allclose(run['physics_trace'][-1]['command'], expected)
            np.testing.assert_allclose(session.run.call_args.args[1]['obs'][0, 6:9],
                                       np.array(expected) / .8, rtol=1e-6)


class ReportTests(unittest.TestCase):
    def fixture(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        controller.injection = SimpleNamespace(spawn_time=.8, events=[{'time': .8, 'event': 'barrier_spawn'}])
        controller.first_zero = {'time': 1., 'reason': 'E_LOCAL_NO_ROUTE'}
        controller.latch = dict(controller.first_zero)
        controller.global_plans = [{'stamp': .2, 'ok': True, 'path': [[0., 0., 0.]]}]
        controller.local_plans = [{'stamp': .2, 'ok': True, 'trajectory': [[0., 0., 0., 0.]]},
                                  {'stamp': 1., 'ok': False, 'reason': 'E_LOCAL_NO_ROUTE'}]
        controller.clouds = [scan(.8, [[1.9, 0., .35]])]
        controller.decisions = [{'stamp': .2, 'vx': .3, 'body_command': [.3, 0., 0.], 'reason': 'track'},
                                {'stamp': 1., 'vx': 0., 'body_command': [0., 0., 0.],
                                 'reason': 'E_LOCAL_NO_ROUTE'}]
        run = {'physics_trace': trace(), 'fell': False}
        xml = '<mujoco><worldbody><geom name="navigation_pilot_barrier" pos="2 0 .5" size=".1 1 .5"/></worldbody></mujoco>'
        return run, controller, xml

    def test_report_separates_planner_and_spawn_origins_no_seed_claim(self):
        result = nav.report(*self.fixture())
        self.assertEqual(result['verdict'], 'PASS')  # synthetic report fixture only
        self.assertEqual(result['planner_braking']['trigger_time'], 1.)
        self.assertEqual(result['injection_to_stop']['trigger_time'], .8)
        self.assertGreater(result['injection_to_stop']['cumulative_xy_m'],
                           result['planner_braking']['cumulative_xy_m'])
        self.assertFalse(result['hardware'])
        self.assertFalse(result['budget_frozen'])
        self.assertTrue(result['no_ros'])
        self.assertNotIn('seed_statistics', result)

    def test_report_does_not_treat_lateral_or_heading_command_as_initial_zero(self):
        for command in ([0., .1, 0.], [0., 0., .1]):
            run, controller, xml = self.fixture()
            controller.decisions[0].update(vx=0., body_command=command)
            result = nav.report(run, controller, xml)
            self.assertEqual(result['verdict'], 'FAIL')
            self.assertFalse(result['checks']['forward_command_before'])

    def test_missing_sensor_watchdog_unsupported_or_no_route_cannot_green(self):
        for missing in ('cloud', 'watchdog', 'unsupported', 'route', 'fall', 'braking', 'late_hits', 'early_zero'):
            run, controller, xml = self.fixture()
            if missing == 'cloud':
                controller.clouds = []
            elif missing in ('watchdog', 'unsupported'):
                controller.decisions[-1]['reason'] = ('watchdog' if missing == 'watchdog'
                                                       else 'unsupported_axis_or_velocity')
            elif missing == 'route':
                controller.local_plans.pop()
            elif missing == 'fall':
                run['fell'] = True
            elif missing == 'late_hits':
                controller.clouds[0]['stamp'] = 1.2
            elif missing == 'early_zero':
                controller.first_zero['time'] = .6
            else:
                controller.first_zero = None
            self.assertNotEqual(nav.report(run, controller, xml)['verdict'], 'PASS', missing)

    def test_independent_truth_detects_crossing_between_samples(self):
        _, _, xml = self.fixture()
        samples = [{'time': 0., 'xyz': [1., 0., .35],
                    'robot_aabb': [[.8, -.2, 0.], [1.2, .2, .5]]},
                   {'time': 1., 'xyz': [3., 0., .35],
                    'robot_aabb': [[2.8, -.2, 0.], [3.2, .2, .5]]}]
        result = nav.collision_evaluation(samples, xml, .1)
        self.assertFalse(result['ok'])

    def test_latched_no_route_cannot_hide_subsequent_sensor_failure(self):
        run, controller, xml = self.fixture()
        controller.clouds.append({'stamp': 1.2, 'ok': False, 'error': 'missing'})
        self.assertEqual(nav.report(run, controller, xml)['verdict'], 'FAIL')

    def test_collision_geometry_can_extend_beyond_the_body_centre(self):
        _, _, xml = self.fixture()
        sample = {'time': 1., 'xyz': [1.51, 0., .35],
                  'robot_aabb': [[1.1, -.2, 0.], [1.93, .2, .5]]}
        later = dict(sample, time=1.1)
        self.assertFalse(nav.collision_evaluation([sample, later], xml, .9)['ok'])

    def test_yaw_mismatch_rejects_world_velocity(self):
        controller = nav.NavigationController(sensor=lambda m, d: scan(d.time))
        state = data()
        state.qpos[3:7] = [np.cos(np.pi / 4), 0., 0., np.sin(np.pi / 4)]
        np.testing.assert_array_equal(controller(None, state), np.zeros(3))
        self.assertEqual(controller.faults[0]['reason'], 'carrier_heading')

    def test_cli_invalid_parameters_never_runs_rollout(self):
        with patch.object(sys, 'argv', ['mj_navigation_stop.py', '--xml', 'unused',
                          '--policy', 'unused', '--out', '/tmp/unused', '--inject-at', '20']), \
                patch.object(nav, 'rollout') as run, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                nav.main()
        run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
