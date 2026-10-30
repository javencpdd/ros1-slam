"""NIST runner route derivation (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import nist_stairs_runtime as runner


def fixture():
    return {'map_id': 'fixture_stairs', 'map_version': 1, 'resolution': .15,
            'surfaces': [
                {'surface_id': 'ground', 'z_reference': 0., 'traversability': 'supported',
                 'footprint': {'min_x': -2., 'max_x': 2., 'min_y': -2., 'max_y': -1.},
                 'edges': [{'to': 'step1', 'kind': 'step'}]},
                {'surface_id': 'step1', 'z_reference': .2, 'traversability': 'supported',
                 'footprint': {'min_x': -1., 'max_x': 1., 'min_y': -1.1, 'max_y': 0.},
                 'edges': []},
                {'surface_id': 'ignored', 'z_reference': 5., 'traversability': 'blocked',
                 'footprint': {'min_x': 9., 'max_x': 9., 'min_y': 9., 'max_y': 9.},
                 'edges': []}]}


class NistRunnerTests(unittest.TestCase):
    def test_route_runs_ground_centre_to_highest_supported_centre(self):
        name, value, start, goal = runner.case_from_map(fixture())
        self.assertEqual(name, 'fixture_stairs')
        self.assertEqual(start, (0., -1.5, 0.))
        self.assertEqual(goal, (0., -.55, .2))
        self.assertIs(value, runner.case_from_map(value)[1])

    def test_experimental_config_uses_map_resolution_and_survives_plan_latency(self):
        config = runner.experimental_config(fixture())
        self.assertEqual(config['resolution'], .15)
        self.assertGreaterEqual(config['validity_s'], config['tick_s'] + 1.8)

    def test_single_surface_map_is_rejected(self):
        value = fixture()
        value['surfaces'] = value['surfaces'][:1]
        with self.assertRaises(ValueError):
            runner.case_from_map(value)


if __name__ == '__main__':
    unittest.main()
