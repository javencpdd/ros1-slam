"""Counterexamples for native receding-horizon 3D planning and cutoff."""
import copy
import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'representation'))
import support_map
import local_native as native
import evaluate
import importlib.util
_ros_spec = importlib.util.spec_from_file_location('v8_native_local_ros', HERE / 'ros_node.py')
ros_node = importlib.util.module_from_spec(_ros_spec)
_ros_spec.loader.exec_module(ros_node)


def flat(ymin=-2.0, ymax=2.0):
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value['surfaces'][0]['footprint'].update(min_x=-2.0, max_x=3.0, min_y=ymin, max_y=ymax)
    return value


def layers():
    return support_map.build_support_map(
        'local_layers', .2, [-2., -2., 0.],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'fixture://ground',
          'footprint': {'min_x': -2., 'max_x': 0.2, 'min_y': -1., 'max_y': 1.},
          'edges': [{'to': 'deck', 'kind': 'step', 'max_step_height_m': .2, 'bidirectional': True}]},
         {'surface_id': 'deck', 'z_reference': .15, 'data_uri': 'fixture://deck',
          'footprint': {'min_x': 0., 'max_x': 3., 'min_y': -1., 'max_y': 1.}, 'edges': []}],
        None, {'uri': 'fixture://local_layers', 'sha256': 'a' * 64})


BOX = {'id': 'box', 'min': [.6, -.2, -.05], 'max': [1.0, .2, .8]}


class NativeTests(unittest.TestCase):
    def test_straight_three_dimensional_arrival(self):
        result = native.plan(flat(), (0., 0., 0.), (1., 0., 0.), now=10.)
        self.assertEqual(result['reason'], 'arrived')
        self.assertEqual(result['trajectory'][-1][:3], (1., 0., 0.))
        self.assertTrue(evaluate.evaluate(flat(), result, [], now=10.)['ok'])
        self.assertEqual(result['capability_domain'], 'experimental_kinematic_carrier')

    def test_new_obstacle_changes_motion_and_independent_evaluator_agrees(self):
        clear = native.plan(flat(), (0., 0., 0.), (1.6, 0., 0.))
        blocked = native.plan(flat(), (0., 0., 0.), (1.6, 0., 0.), [BOX])
        self.assertNotEqual(clear['trajectory'], blocked['trajectory'])
        self.assertGreater(max(abs(p[1]) for p in blocked['trajectory']), .5)
        self.assertTrue(evaluate.evaluate(flat(), blocked, [BOX], now=0.)['ok'])
        self.assertFalse(evaluate.evaluate(flat(), clear, [BOX], now=0.)['ok'])

    def test_moving_obstacle_predicted_in_time(self):
        moving = dict(BOX, velocity=[0., -.8, 0.], min=[.6, .8, -.05], max=[1., 1.2, .8])
        result = native.plan(flat(), (0., 0., 0.), (1.6, 0., 0.), [moving])
        self.assertTrue(evaluate.evaluate(flat(), result, [moving])['ok'])

    def test_swept_collision_detects_between_safe_endpoints(self):
        cfg = native.Config(radius_m=.01, clearance_m=.01, max_speed_mps=3.)
        thin = {'min': [.49, -.05, -.05], 'max': [.51, .05, .5]}
        box = native._obstacles([thin])[0]
        self.assertTrue(native._swept_hit((0., 0., 0., 0.), (1., 0., 0., 1.), box, cfg))
        # Fast transverse motion crosses the robot even though both endpoint poses are clear.
        moving = dict(thin, min=[-.05, -.05, -.05], max=[.05, .05, .5], velocity=[2., 0., 0.])
        box = native._obstacles([moving])[0]
        self.assertTrue(native._swept_hit((1., 0., 0., 0.), (1., 0., 0., 1.), box, cfg))

    def test_no_route_refuses_and_clears_command(self):
        wall = {'min': [-.15, -3., -.1], 'max': [.15, 3., 1.]}
        with self.assertRaises(native.LocalError) as caught:
            native.plan(flat(), (-.4, 0., 0.), (1.4, 0., 0.), [wall])
        self.assertEqual(caught.exception.code, 'E_LOCAL_NO_ROUTE')
        self.assertEqual(native.command(None, 0., 'single_floor', 2)['velocity'], [0., 0., 0.])

    def test_short_step_is_locally_searched_and_z_matters(self):
        value = layers()
        result = native.plan(value, (-.4, 0., 0.), (.6, 0., .15),
                             config=replace(native.Config(), tick_s=1.2, validity_s=2.0))
        self.assertEqual(result['reason'], 'arrived')
        self.assertIn('deck', result['surfaces'])
        self.assertEqual(result['trajectory'][-1][2], .15)
        self.assertTrue(evaluate.evaluate(value, result, [])['ok'])
        # Same XY, wrong layer is not accepted as arrival.
        with self.assertRaises(native.LocalError) as caught:
            native.plan(value, (-.4, 0., 0.), (.6, 0., 0.))
        self.assertEqual(caught.exception.code, 'E_LOCAL_SUPPORT')

    def test_same_xy_two_layers_remain_distinct(self):
        value = layers()
        value['surfaces'][0]['footprint']['max_x'] = 3.
        value['surfaces'][1]['z_reference'] = .6
        value['surfaces'][0]['edges'][0]['max_step_height_m'] = .7
        cfg = replace(native.Config(), tick_s=1., max_speed_mps=1., max_step_m=.7)
        low = native.plan(value, (.4, 0., 0.), (1.4, 0., 0.), config=cfg)
        high = native.plan(value, (.4, 0., .6), (1.4, 0., .6), config=cfg)
        self.assertEqual(set(low['surfaces']), {'ground'})
        self.assertEqual(set(high['surfaces']), {'deck'})
        self.assertNotEqual(low['trajectory'], high['trajectory'])

    def test_3d_obstacle_does_not_flatten_distinct_layers(self):
        value = layers()
        value['surfaces'][0]['footprint']['max_x'] = 3.
        value['surfaces'][1]['z_reference'] = .6
        value['surfaces'][0]['edges'][0]['max_step_height_m'] = .7
        low_box = dict(BOX, min=[.6, -.2, 0.], max=[1., .2, .2])
        result = native.plan(value, (.4, 0., .6), (1.4, 0., .6), [low_box])
        self.assertEqual(result['reason'], 'arrived')
        self.assertTrue(all(p[2] == .6 for p in result['trajectory']))
        self.assertTrue(evaluate.evaluate(value, result, [low_box])['ok'])

    def test_expiry_map_change_localization_stop_each_tick(self):
        value = flat()
        result = native.plan(value, (0., 0., 0.), (1., 0., 0.), now=5.)
        self.assertFalse(native.command(result, 5.1, value['map_id'], value['map_version'])['stop'])
        for stamp, map_id, version, local_ok, reason in [
            (6., value['map_id'], 2, True, 'E_LOCAL_EXPIRED'),
            (4., value['map_id'], 2, True, 'E_LOCAL_EXPIRED'),
            (5.1, value['map_id'], 3, True, 'E_LOCAL_MAP_STALE'),
            (5.1, 'other_map', 2, True, 'E_LOCAL_MAP_STALE'),
            (5.1, value['map_id'], 2, False, 'E_LOCAL_LOCALIZATION')]:
            out = native.command(result, stamp, map_id, version, local_ok)
            self.assertTrue(out['stop'])
            self.assertEqual(out['velocity'], [0., 0., 0.])
            self.assertEqual(out['reason'], reason)

    def test_budget_and_cancel_distinct_from_no_route(self):
        with self.assertRaises(native.LocalError) as caught:
            native.plan(flat(), (0., 0., 0.), (2., 0., 0.), config=replace(native.Config(), max_expansions=1))
        self.assertEqual(caught.exception.code, 'E_LOCAL_BUDGET')
        with self.assertRaises(native.LocalError) as caught:
            native.plan(flat(), (0., 0., 0.), (2., 0., 0.), cancel=lambda: True)
        self.assertEqual(caught.exception.code, 'E_LOCAL_CANCELLED')

    def test_prohibited_robot_profile_is_not_promoted(self):
        profile = json.loads((HERE.parent / 'contracts/fixtures/robot_lite3_design.json').read_text())
        with self.assertRaises(native.LocalError) as caught:
            native.plan(flat(), (0., 0., 0.), (1., 0., 0.), robot_profile=profile)
        self.assertEqual(caught.exception.code, 'E_TRAJ_CAPABILITY_UNVERIFIED')

    def test_nonfinite_and_malformed_inputs_refused(self):
        for kwargs in ({'start': (0., 0., float('nan'))}, {'now': float('inf')},
                       {'obstacles': [{'min': [0, 0, 0], 'max': [0, 1, 1]}]},
                       {'obstacles': None}, {'obstacles': ['bad_box']},
                       {'config': replace(native.Config(), max_expansions=1.5)}):
            args = {'map_value': flat(), 'start': (0., 0., 0.), 'goal': (1., 0., 0.)}
            args.update(kwargs)
            with self.assertRaises(native.LocalError):
                native.plan(**args)

    def test_horizon_has_finite_progress_and_terminal_hold(self):
        result = native.plan(flat(), (0., 0., 0.), (2.8, 0., 0.),
                             config=replace(native.Config(), horizon_s=2.))
        self.assertEqual(result['reason'], 'horizon')
        self.assertEqual(result['trajectory'][-1][:3], result['trajectory'][-2][:3])
        self.assertLessEqual(result['trajectory'][-1][3], 2.)
        self.assertLess(result['valid_until'], result['trajectory'][-1][3])

    def test_input_heartbeats_fail_closed(self):
        self.assertIsNone(ros_node.freshness(10., 9.5, 9.5, 9.5, True, 1.))
        self.assertEqual(ros_node.freshness(10., 9.5, 9.5, 9.5, False, 1.), 'E_LOCAL_LOCALIZATION')
        self.assertEqual(ros_node.freshness(10., 9.5, 9.5, 9., True, 1.), 'E_LOCAL_LOCALIZATION')
        self.assertEqual(ros_node.freshness(10., 9., 9.5, 9.5, True, 1.), 'E_LOCAL_POSE_STALE')
        self.assertEqual(ros_node.freshness(10., 9.5, 9., 9.5, True, 1.), 'E_LOCAL_OBSTACLES_STALE')

    def test_thin_static_roof_counterexample_is_refused_and_independently_red(self):
        value = layers()
        value['surfaces'][0]['footprint'] = {'min_x': -2., 'max_x': 3., 'min_y': -2., 'max_y': 2.}
        value['surfaces'][1]['footprint'] = {'min_x': .09, 'max_x': .11, 'min_y': -.05, 'max_y': .05}
        # Reviewer counterexample: lattice endpoints x=0,.2 are outside roof,
        # while the segment and even the starting robot envelope intersect it.
        with self.assertRaises(native.LocalError) as caught:
            native.plan(value, (0., 0., 0.), (.2, 0., 0.))
        self.assertEqual(caught.exception.code, 'E_LOCAL_CLEARANCE')
        wrong = {'map_id': value['map_id'], 'map_version': value['map_version'],
                 'config': native.asdict(native.Config()), 'trajectory': [(0., 0., 0., 0.), (.2, 0., 0., .5)]}
        verdict = evaluate.evaluate(value, wrong, [])
        self.assertFalse(verdict['ok'])
        self.assertIn('static_collision', verdict['failures'])
        self.assertGreater(len(verdict['static_collisions']), 0)

    def test_thin_roof_between_clear_body_endpoints_cannot_be_tunneled(self):
        value = layers()
        value['surfaces'][0]['footprint'] = {'min_x': -2., 'max_x': 3., 'min_y': -2., 'max_y': 2.}
        value['surfaces'][1]['footprint'] = {'min_x': .099, 'max_x': .101, 'min_y': -.01, 'max_y': .01}
        cfg = replace(native.Config(), radius_m=.001, clearance_m=.001)
        result = native.plan(value, (0., 0., 0.), (.2, 0., 0.), config=cfg)
        self.assertGreater(max(abs(p[1]) for p in result['trajectory']), 0.)
        self.assertTrue(evaluate.evaluate(value, result, [])['ok'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
