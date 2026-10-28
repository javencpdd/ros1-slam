"""Negative controls for step-face cases; these tests do not simulate MuJoCo."""
import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

import mj_step_baseline as baseline
import mj_step_obstacles as cases


def synthetic(case='clear'):
    geometry = {'deck': {'min': [-1., -1.6, 0.], 'max': [5., 1.6, .06]}, 'boxes': []}
    if case != 'clear':
        geometry['boxes'] = [{'min': [1.6, .1, .06], 'max': [2., .8, .66]}]
    controller = cases.RouteController({'ok': case != 'blocked_box', 'path': [[.05, .05, .06, 'deck'],
                                                                          [3.55, .05, .06, 'deck']]})
    controller.arrived = case != 'blocked_box'
    controller.decisions = [{'time': 0., 'command': [0., 0., 0.] if case == 'blocked_box' else [.2, 0., 0.]}]
    plan = {'ok': False, 'reason': 'E_PLAN_NO_ROUTE'} if case == 'blocked_box' else {'ok': True}
    samples = []
    for t, x in ((0., .05), (1., .05 if case == 'blocked_box' else 3.55)):
        feet = {leg: [x + .2 * (1 if leg[0] == 'F' else -1), .2, .082]
                for leg in ('FL', 'FR', 'HL', 'HR')}
        samples.append({'time': t, 'xyz': [x, .05, .4],
                        'base_vxy': [0., 0.] if case == 'blocked_box' else [.2, 0.],
                        'qpos': [x, .05, .4, 1., 0., 0., 0.] + [0., -.65, 1.3] * 4,
                        'command': [0., 0., 0.] if case == 'blocked_box' else [.2, 0., 0.],
                        'tau_max': 12.,
                        'robot_aabb': [[x - .4, -.25, .06], [x + .4, .35, .5]],
                        'observation': {'feet': feet, 'box_contacts': []}})
    return {'physics_trace': samples, 'fell': False}, plan, geometry, controller


class StepObstacleCaseTests(unittest.TestCase):
    def test_wrappers_preserve_input_and_use_same_deck(self):
        with tempfile.TemporaryDirectory() as directory:
            xml = Path(directory) / 'original.xml'
            original = '<mujoco><compiler/><worldbody><geom type="plane"/></worldbody></mujoco>'
            xml.write_text(original)
            decks = []
            for case in cases.CASES:
                root = ET.fromstring(cases.wrapper_xml(xml, case))
                decks.append(root.find('.//geom[@name="step_obstacles_deck"]').attrib)
                boxes = root.findall('.//geom[@name="step_obstacles_box"]')
                self.assertEqual(len(boxes), 0 if case == 'clear' else 1)
            self.assertEqual(decks[0], decks[1])
            self.assertEqual(decks[0], decks[2])
            self.assertEqual(xml.read_text(), original)

    def test_existing_step_world_rejected_instead_of_stacking_fixtures(self):
        with tempfile.TemporaryDirectory() as directory:
            xml = Path(directory) / 'existing_step.xml'
            xml.write_text('<mujoco><worldbody><geom name="step" type="box"/></worldbody></mujoco>')
            with self.assertRaises(ValueError):
                cases.wrapper_xml(xml, 'clear')

    def test_clear_success_is_explicitly_not_ascent_acceptance(self):
        args = synthetic()
        report = cases.evaluate_physics(*args, 'clear')
        self.assertEqual(report['verdict'], 'PASS')
        self.assertFalse(report['ascent_completed'])
        self.assertEqual(report['four_foot_ascent_acceptance'], 'NOT_TESTED')
        self.assertTrue(report['initialized_on_deck'])

    def test_declared_no_route_requires_zero_command_and_no_motion(self):
        run, plan, geometry, controller = synthetic('blocked_box')
        self.assertEqual(cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')['verdict'], 'PASS')
        controller.decisions[0]['command'][0] = .1
        self.assertEqual(cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')['verdict'], 'FAIL')
        controller.decisions[0]['command'][0] = 0.
        run['physics_trace'][-1]['xyz'][0] += .11
        self.assertEqual(cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')['verdict'], 'FAIL')

    def test_endpoint_success_does_not_hide_intermediate_collision(self):
        run, plan, geometry, controller = synthetic('offset_box')
        collision = copy.deepcopy(run['physics_trace'][0])
        collision.update(time=.5, xyz=[1.8, .05, .4], robot_aabb=[[1.4, -.25, .06], [2.2, .35, .5]])
        run['physics_trace'].insert(1, collision)
        result = cases.evaluate_physics(run, plan, geometry, controller, 'offset_box')
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertFalse(result['checks']['no_box_aabb_overlap'])

    def test_contacts_missing_geometry_and_wrong_envelope_cannot_inherit_green(self):
        for modification, failed in ((lambda s: s['observation']['box_contacts'].append({'geom1': 1, 'geom2': 2, 'distance': -.01}), 'no_box_contact'),
                                     (lambda s: s.pop('robot_aabb'), 'physics_trace_present'),
                                     (lambda s: s['robot_aabb'][1].__setitem__(2, .6), 'actual_body_within_planning_envelope'),
                                     (lambda s: s['robot_aabb'][0].__setitem__(2, .04), 'actual_body_within_planning_envelope'),
                                     (lambda s: s['observation']['feet']['HR'].__setitem__(2, .022), 'all_four_feet_end_on_deck')):
            run, plan, geometry, controller = synthetic()
            modification(run['physics_trace'][-1])
            result = cases.evaluate_physics(run, plan, geometry, controller, 'clear')
            self.assertEqual(result['verdict'], 'FAIL')
            self.assertFalse(result['checks'][failed])

    def test_wrong_rejection_reason_does_not_count_as_blockage(self):
        run, plan, geometry, controller = synthetic('blocked_box')
        plan['reason'] = 'E_PLAN_MAP_INVALID'
        result = cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')
        self.assertFalse(result['checks']['expected_planner_behavior'])

    def test_invalid_initialization_rejected_before_model_or_policy_import(self):
        for initial in ([0, 0], [0, 0, False], [0, 0, float('nan')], [0, 0, 0], 'bad'):
            with self.assertRaises(ValueError):
                baseline.rollout('unused.xml', 'unused.onnx', initial_xyz=initial)
        with self.assertRaises(ValueError):
            baseline.rollout('unused.xml', 'unused.onnx', physics_observer=1)

    def test_refused_plan_controller_returns_exact_zero(self):
        class Data:
            qpos = np.array([.05, .05, .4, 1., 0., 0., 0.])
            time = 0.
        controller = cases.RouteController({'ok': False, 'reason': 'E_PLAN_NO_ROUTE'})
        self.assertTrue(np.array_equal(controller(None, Data()), [0., 0., 0.]))

    def test_empty_physics_and_empty_successful_route_rejected(self):
        run, plan, geometry, controller = synthetic()
        run['physics_trace'] = []
        self.assertEqual(cases.evaluate_physics(run, plan, geometry, controller, 'clear')['verdict'], 'FAIL')
        with self.assertRaises(ValueError):
            cases.RouteController({'ok': True, 'path': []})

    def test_leaving_the_deck_and_returning_cannot_count_as_top_navigation(self):
        run, plan, geometry, controller = synthetic()
        departure = copy.deepcopy(run['physics_trace'][0])
        departure.update(time=.5, xyz=[.05, 1.7, .4], robot_aabb=[[-.35, 1.4, .06], [.45, 2., .5]])
        run['physics_trace'].insert(1, departure)
        result = cases.evaluate_physics(run, plan, geometry, controller, 'clear')
        self.assertFalse(result['checks']['actual_body_stays_over_deck'])
        self.assertEqual(result['verdict'], 'FAIL')

    def test_nonfinite_measurements_cannot_bypass_comparisons(self):
        for field in ('xyz', 'base_vxy', 'qpos', 'command', 'tau_max', 'robot_aabb'):
            for bad in (float('nan'), float('inf'), True, '0'):
                run, plan, geometry, controller = synthetic()
                sample = run['physics_trace'][0]
                if field == 'tau_max':
                    sample[field] = bad
                elif field == 'robot_aabb':
                    sample[field][0][0] = bad
                else:
                    sample[field][0] = bad
                result = cases.evaluate_physics(run, plan, geometry, controller, 'clear')
                self.assertEqual(result['verdict'], 'FAIL', (field, bad))
                self.assertFalse(result['checks']['physics_measurements_valid'])
        for field in ('feet', 'box_contacts'):
            run, plan, geometry, controller = synthetic()
            observation = run['physics_trace'][0]['observation']
            if field == 'feet':
                observation['feet']['HR'][2] = float('nan')
            else:
                observation['box_contacts'] = [{'geom1': 1, 'geom2': 2, 'distance': float('nan')}]
            self.assertEqual(cases.evaluate_physics(run, plan, geometry, controller, 'clear')['verdict'], 'FAIL')

    def test_nonfinite_geometry_decisions_and_time_are_rejected(self):
        for target in ('geometry', 'decisions', 'time', 'inverted_bounds'):
            run, plan, geometry, controller = synthetic()
            if target == 'geometry':
                geometry['deck']['max'][2] = float('nan')
            elif target == 'decisions':
                controller.decisions[0]['command'][0] = float('nan')
            elif target == 'inverted_bounds':
                run['physics_trace'][0]['robot_aabb'][0][0] = 100.
            else:
                run['physics_trace'][1]['time'] = run['physics_trace'][0]['time']
            result = cases.evaluate_physics(run, plan, geometry, controller, 'clear')
            self.assertEqual(result['verdict'], 'FAIL')
            self.assertFalse(result['checks']['physics_measurements_valid'])

    def test_blocked_case_detects_roundtrip_during_zero_command(self):
        run, plan, geometry, controller = synthetic('blocked_box')
        middle = copy.deepcopy(run['physics_trace'][0])
        middle.update(time=.5, xyz=[1., .05, .4], robot_aabb=[[.6, -.25, .06], [1.4, .35, .5]])
        run['physics_trace'].insert(1, middle)
        report = cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')
        self.assertEqual(report['verdict'], 'FAIL')
        self.assertEqual(report['base_displacement_m'], 0.)
        self.assertAlmostEqual(report['maximum_xy_excursion_m'], .95)
        self.assertFalse(report['checks']['motion_or_rejection_verified'])

    def test_actual_nonzero_physics_commands_cannot_be_hidden_by_zero_decisions(self):
        run, plan, geometry, controller = synthetic('blocked_box')
        for sample in run['physics_trace']:
            sample['command'] = [.2, 0., 0.]
        result = cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertTrue(result['physics_nonzero_command_observed'])
        self.assertFalse(result['checks']['motion_or_rejection_verified'])
        controller.decisions = []
        result = cases.evaluate_physics(run, plan, geometry, controller, 'blocked_box')
        self.assertFalse(result['checks']['physics_measurements_valid'])


if __name__ == '__main__':
    unittest.main()
