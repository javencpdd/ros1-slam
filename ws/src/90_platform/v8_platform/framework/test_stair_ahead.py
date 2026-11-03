"""前方台阶探测单测（纯 Python，无 ROS）。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stair_ahead  # noqa: E402


def two_level_map():
    step = {'kind': 'step', 'max_step_height_m': 0.15, 'bidirectional': True}
    return {'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
            'resolution': 0.1, 'map_id': 'test', 'map_version': 1,
            'surfaces': [
                {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
                 'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 2.0, 'max_y': 4.0},
                 'edges': [dict(step, to='m1')]},
                {'surface_id': 'm1', 'z_reference': 0.15, 'traversability': 'supported',
                 'footprint': {'min_x': 2.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0},
                 'edges': [dict(step, to='m0')]},
            ]}


class StairAheadTests(unittest.TestCase):
    def test_detects_step_ahead(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         lookahead_m=2.0)
        self.assertTrue(result['ahead'])
        self.assertAlmostEqual(result['step_m'], 0.15, places=3)
        self.assertEqual(result['direction'], 'up')
        self.assertEqual(result['from_surface'], 'm0')
        self.assertEqual(result['to_surface'], 'm1')
        self.assertAlmostEqual(result['distance_m'], 1.0, delta=0.1)

    def test_lookahead_blocks_detection(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         lookahead_m=0.5)
        self.assertFalse(result['ahead'])

    def test_min_step_filters_small_ledges(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         min_step_m=0.2)
        self.assertFalse(result['ahead'])

    def test_pose_off_support(self):
        result = stair_ahead.stair_ahead(two_level_map(), (9.0, 9.0, 0.0),
                                         [(9.0, 9.0, 0.0), (3.0, 2.0, 0.0)])
        self.assertFalse(result['ahead'])
        self.assertEqual(result['reason'], 'pose_off_support')


if __name__ == '__main__':
    unittest.main(verbosity=2)
