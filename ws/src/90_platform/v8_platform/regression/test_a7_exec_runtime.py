"""A7 execution runtime argument controls (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import a7_exec_runtime


class ExecRuntimeArgsTests(unittest.TestCase):
    def test_sc03_two_layer_reverse(self):
        name, value, start, goal = a7_exec_runtime.case_args('SC03')
        self.assertEqual(name, 'sc03')
        self.assertEqual(start, (3.0, 2.0, 0.6))
        self.assertEqual(goal, (-4.0, -4.0, 0.0))
        self.assertIn('mezz', {s['surface_id'] for s in value['surfaces']})

    def test_sc06_ground_path_under_mezz(self):
        name, value, start, goal = a7_exec_runtime.case_args('SC06')
        self.assertEqual(name, 'sc06')
        self.assertEqual(start, (-4.0, 0.0, 0.0))
        self.assertEqual(goal, (4.0, 0.0, 0.0))
        self.assertIn('mezz', {s['surface_id'] for s in value['surfaces']})


if __name__ == '__main__':
    unittest.main()
