import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / '90_platform/v8_platform/contracts'))
sys.path.insert(0, str(ROOT / '80_algorithm/v8_algorithm/representation'))
import offline_capability_cost as cost  # noqa: E402
import support_map as rep  # noqa: E402

SRC = {'uri': 'design://two-routes', 'sha256': 'a' * 64}
TESTED = json.loads((ROOT / '90_platform/v8_platform/contracts/fixtures/robot_v2_tested_fixture.json').read_text())
DECLARED = json.loads((ROOT / '90_platform/v8_platform/contracts/fixtures/robot_lite3_v2_declared.json').read_text())


def two_routes():
    surfaces = [
        {'surface_id': 'left', 'z_reference': 0.0, 'data_uri': 'l',
         'footprint': {'min_x': -5, 'min_y': -1, 'max_x': -1, 'max_y': 1},
         'edges': [{'to': 'right', 'kind': 'step', 'max_step_height_m': 0.2, 'bidirectional': True},
                   {'to': 'ramp', 'kind': 'slope', 'max_slope_deg': 10.0, 'bidirectional': True}]},
        {'surface_id': 'right', 'z_reference': 0.2, 'data_uri': 'r',
         'footprint': {'min_x': 1, 'min_y': -1, 'max_x': 5, 'max_y': 1},
         'edges': [{'to': 'ramp', 'kind': 'slope', 'max_slope_deg': 10.0, 'bidirectional': True}]},
        {'surface_id': 'ramp', 'z_reference': 0.1, 'data_uri': 'm',
         'footprint': {'min_x': -1.5, 'min_y': -2, 'max_x': 1.5, 'max_y': 2},
         'edges': []},
    ]
    return rep.build_support_map('two_routes', 0.5, [-5, -5, 0], surfaces, None, SRC)


def profile(max_step):
    value = copy.deepcopy(TESTED)
    value['capability']['max_step_height_m']['value'] = max_step
    return value


class OfflineCostTests(unittest.TestCase):
    def test_capable_robot_compares_direct_and_detour(self):
        result = cost.evaluate(two_routes(), (-4, 0, 0), (4, 0, 0.2), profile(0.25))
        self.assertEqual(result['verdict'], 'FEASIBLE')
        self.assertEqual(result['removed_transitions'], [])
        routes = result['candidate_routes']
        self.assertGreaterEqual(len(routes), 2)
        baseline = routes[0]
        self.assertEqual(baseline['cost']['transitions'], [{'from': 'left', 'to': 'right'}])
        detour = next(r for r in routes if r is not baseline)
        self.assertIn('ramp', [item['to'] for item in detour['cost']['transitions']] +
                      [item['from'] for item in detour['cost']['transitions']])
        self.assertGreater(detour['cost']['distance_m'], baseline['cost']['distance_m'])
        self.assertTrue(baseline['cost']['risk_known'])
        self.assertIsNotNone(baseline['cost']['time_s'])
        self.assertIsNotNone(baseline['cost']['risk_expected_m'])

    def test_insufficient_step_filters_direct_edge_before_cost(self):
        result = cost.evaluate(two_routes(), (-4, 0, 0), (4, 0, 0.2), profile(0.1))
        self.assertEqual(result['verdict'], 'FEASIBLE')
        self.assertIn({'from': 'left', 'to': 'right', 'kind': 'step'},
                      result['removed_transitions'])
        transitions = result['candidate_routes'][0]['cost']['transitions']
        self.assertNotIn({'from': 'left', 'to': 'right'}, transitions)
        self.assertIn('ramp', [item['from'] for item in transitions] + [item['to'] for item in transitions])

    def test_safety_limit_tightens_before_plan(self):
        result = cost.evaluate(two_routes(), (-4, 0, 0), (4, 0, 0.2), profile(0.25),
                               {'max_step_height_m': 0.1})
        self.assertIn({'from': 'left', 'to': 'right', 'kind': 'step'},
                      result['removed_transitions'])

    def test_unknown_risk_is_not_zero(self):
        value = rep.build_support_map(
            'flat', 0.5, [-5, -5, 0],
            [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
              'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5}, 'edges': []}],
            None, SRC)
        result = cost.evaluate(value, (-4, 0, 0), (4, 0, 0), DECLARED)
        self.assertEqual(result['verdict'], 'FEASIBLE')
        self.assertFalse(result['risk_known'])
        self.assertIsNone(result['candidate_routes'][0]['cost']['risk_expected_m'])
        self.assertIsNone(result['candidate_routes'][0]['cost']['time_s'])


if __name__ == '__main__':
    unittest.main()