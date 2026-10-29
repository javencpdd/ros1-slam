"""A5 kinematic staircase topology (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import a5_cross_layer_runtime as staircase_runtime


class StaircaseMapTests(unittest.TestCase):
    def test_five_levels_in_order(self):
        name, value, start, goal = staircase_runtime.case_args()
        self.assertEqual(name, 'staircase')
        levels = sorted((s['z_reference'], s['surface_id']) for s in value['surfaces'])
        self.assertEqual([sid for _z, sid in levels], ['ground', 'step1', 'step2', 'step3', 'deck'])
        self.assertEqual(levels[-1][0], .6)

    def test_declared_chain_and_endpoints(self):
        _name, value, start, goal = staircase_runtime.case_args()
        pairs = {(s['surface_id'], e['to']) for s in value['surfaces'] for e in s['edges']}
        self.assertIn(('ground', 'step1'), pairs)
        self.assertIn(('step1', 'step2'), pairs)
        self.assertIn(('step2', 'step3'), pairs)
        self.assertIn(('step3', 'deck'), pairs)
        self.assertEqual(start, (-4., 0., 0.))
        self.assertEqual(goal, (.8, 0., .6))


if __name__ == '__main__':
    unittest.main()
