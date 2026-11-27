import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import route_contract as route  # noqa: E402


def base_route():
    return {
        'schema_version': 1,
        'backend': 'support',
        'map': {'map_id': 'm', 'map_version': 3},
        'capability': {'robot_id': 'lite3', 'capability_version': 'robot.v2:tested:1'},
        'path': [{'xyz': [0., 0., 0.], 'surface_id': 'ground'},
                 {'xyz': [1., 0., 0.], 'surface_id': 'ground'},
                 {'xyz': [2., 0., .2], 'surface_id': 'mezz'}],
        'transitions': [{'from_index': 1, 'to_index': 2, 'from_surface': 'ground',
                         'to_surface': 'mezz', 'kind': 'step'}],
        'cost_decomposition': {'distance_m': 2.1, 'time_s': 2.625,
                               'risk_expected_m': None, 'risk_known': False},
    }


def reason(value):
    try:
        route.validate_route(value)
    except route.RouteError as error:
        return error.code
    return 'OK'


class RouteContractTests(unittest.TestCase):
    def test_complete_support_route_is_valid(self):
        self.assertEqual(reason(base_route()), 'OK')

    def test_versions_are_required(self):
        value = base_route()
        value['capability'].pop('capability_version')
        self.assertEqual(reason(value), 'E_ROUTE_MISSING')
        value = base_route()
        value['map'].pop('map_version')
        self.assertEqual(reason(value), 'E_ROUTE_MISSING')

    def test_support_points_need_surface_id(self):
        value = base_route()
        value['path'][1].pop('surface_id')
        self.assertEqual(reason(value), 'E_ROUTE_SURFACE')

    def test_transition_kind_and_order_are_checked(self):
        value = base_route()
        value['transitions'][0]['kind'] = 'teleport'
        self.assertEqual(reason(value), 'E_ROUTE_TRANSITION')
        value = base_route()
        value['transitions'][0]['to_index'] = 0
        self.assertEqual(reason(value), 'E_ROUTE_TRANSITION')

    def test_flat_cannot_cross_and_step_must_cross(self):
        value = base_route()
        value['transitions'][0]['kind'] = 'flat'
        self.assertEqual(reason(value), 'E_ROUTE_TRANSITION')
        value = base_route()
        value['path'][2]['surface_id'] = 'ground'
        value['transitions'][0]['to_surface'] = 'ground'
        value['transitions'][0]['kind'] = 'step'
        self.assertEqual(reason(value), 'E_ROUTE_TRANSITION')

    def test_unknown_risk_must_stay_null(self):
        value = base_route()
        value['cost_decomposition']['risk_expected_m'] = 0.0
        self.assertEqual(reason(value), 'E_ROUTE_COST')
        value = base_route()
        value['cost_decomposition']['risk_known'] = True
        self.assertEqual(reason(value), 'E_ROUTE_COST')

    def test_far_route_may_omit_surface_ids(self):
        value = base_route()
        value['backend'] = 'far'
        value['path'] = [{'xyz': [0., 0., 0.]}, {'xyz': [1., 0., 0.]}]
        value['transitions'] = []
        self.assertEqual(reason(value), 'OK')

    def test_from_support_plan_marks_unknown_cost(self):
        plan = {'map_id': 'm', 'map_version': 3, 'length': 2.1,
                'path': [(0., 0., 0., 'ground'), (1., 0., 0., 'ground'), (2., 0., .2, 'mezz')],
                'transitions': [{'from': 'ground', 'to': 'mezz'}]}
        value = route.from_support_plan(plan, 'lite3', 'robot.v2:tested:1',
                                        {('ground', 'mezz'): 'step'})
        self.assertEqual(value['transitions'][0]['kind'], 'step')
        self.assertFalse(value['cost_decomposition']['risk_known'])
        self.assertIsNone(value['cost_decomposition']['risk_expected_m'])


    def test_from_support_plan_preserves_three_layer_transitions(self):
        plan = {
            'map_id': 'three_layers', 'map_version': 7, 'length': 3.4,
            'path': [(-2.0, 0.0, 0.0, 'ground'), (0.2, 0.0, 0.0, 'ground'),
                     (0.4, 0.0, 0.15, 'step1'), (0.8, 0.0, 0.30, 'step2')],
            'transitions': [{'from': 'ground', 'to': 'step1'},
                            {'from': 'step1', 'to': 'step2'}],
        }
        value = route.from_support_plan(
            plan, 'lite3', 'robot.v2:tested:1',
            {('ground', 'step1'): 'step', ('step1', 'step2'): 'step'})
        self.assertEqual([(t['from_surface'], t['to_surface'], t['kind'])
                          for t in value['transitions']],
                         [('ground', 'step1', 'step'), ('step1', 'step2', 'step')])
        self.assertEqual([p['surface_id'] for p in value['path']],
                         ['ground', 'ground', 'step1', 'step2'])
        self.assertEqual(value['map']['map_version'], 7)
        self.assertEqual(value['capability']['capability_version'], 'robot.v2:tested:1')
        self.assertFalse(value['cost_decomposition']['risk_known'])
        self.assertIsNone(value['cost_decomposition']['risk_expected_m'])
        self.assertEqual(reason(value), 'OK')


if __name__ == '__main__':
    unittest.main()