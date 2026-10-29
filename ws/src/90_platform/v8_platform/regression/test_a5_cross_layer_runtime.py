"""A5 kinematic staircase topology (no ROS)."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'local_planner'))
import a5_cross_layer_runtime as staircase_runtime
import local_native as native


class StaircaseMapTests(unittest.TestCase):
    def test_replan_from_between_transfer_points_reaches_step1(self):
        # The ROS run stopped here under a pose-anchored lattice; a map-anchored
        # lattice puts -1.6 back inside the step1 candidate set.
        _name, value, _start, goal = staircase_runtime.case_args()
        config = replace(native.Config(), tick_s=1.2, validity_s=3.)
        result = native.plan(value, (-2.2757026168, 0., 0.), goal, [], now=100.,
                             config=config)
        self.assertIn('step1', result['surfaces'])
        self.assertIn('step2', result['surfaces'])
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

    def test_reverse_starts_on_deck(self):
        name, value, start, goal = staircase_runtime.case_args(reverse=True)
        self.assertEqual(name, 'staircase_down')
        self.assertEqual(start, (.8, 0., .6))
        self.assertEqual(goal, (-4., 0., 0.))
        self.assertEqual(len(value['surfaces']), 5)


if __name__ == '__main__':
    unittest.main()
