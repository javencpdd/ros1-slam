"""A7 execution runtime argument controls (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import a7_exec_runtime
import runtime_probe


class ExecRuntimeArgsTests(unittest.TestCase):
    def test_long_path_uses_experimental_validity_explicitly(self):
        self.assertEqual(a7_exec_runtime.execution_config(),
                         {'tick_s': 1.2, 'validity_s': 2.0})
        self.assertIsNone(runtime_probe.experimental_config('sc06'))
        self.assertEqual(runtime_probe.experimental_config('sc06', a7_exec_runtime.execution_config()),
                         {'tick_s': 1.2, 'validity_s': 2.0})
        self.assertEqual(runtime_probe.experimental_config('step'),
                         {'tick_s': 1.2, 'validity_s': 2.0})

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

    def test_sc01_flat_diagonal_and_sc09_low_ceiling(self):
        name, value, start, goal = a7_exec_runtime.case_args('SC01')
        self.assertEqual((name, start, goal), ('sc01', (-4.0, -4.0, 0.0), (4.0, 4.0, 0.0)))
        name, value, start, goal = a7_exec_runtime.case_args('SC09')
        self.assertEqual((name, start, goal), ('sc09', (-4.0, 0.0, 0.0), (4.0, 0.0, 0.0)))
        self.assertGreater(len(value['surfaces']), 1)

    def test_sc02_slope_scene(self):
        name, value, start, goal = a7_exec_runtime.case_args('SC02')
        self.assertEqual((name, start, goal), ('sc02', (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6)))
        kinds = {edge['kind'] for surface in value['surfaces'] for edge in surface['edges']}
        self.assertIn('slope', kinds)


if __name__ == '__main__':
    unittest.main()
