#!/usr/bin/env python3
"""Smoke tests for the 3D algorithm matrix case construction and local cells."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'scenarios'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'local_planner'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import a3d_algorithm_matrix as matrix  # noqa: E402


class AlgorithmMatrixTests(unittest.TestCase):
    def test_global_and_local_cases_are_declared(self):
        global_names = [case['cell'] for case in matrix.global_cases()]
        local_names = [case['cell'] for case in matrix.local_cases()]
        self.assertEqual(len(global_names), 8)
        self.assertEqual(len(local_names), 9)
        self.assertIn('nist_top', global_names)
        self.assertIn('nist_unsupported', global_names)
        self.assertIn('nist_budget', global_names)
        self.assertIn('nist_down', local_names)
        self.assertIn('flat_moving_box', local_names)

    def test_local_receding_stairs_arrives(self):
        case = next(c for c in matrix.local_cases() if c['cell'] == 'nist_top')
        row = matrix.local_cell(case)
        self.assertTrue(row['ok'], row)
        self.assertEqual(row['reason'], 'arrived')
        self.assertTrue(row['evaluation_ok'])

    def test_local_no_route_reason_is_distinct(self):
        case = next(c for c in matrix.local_cases() if c['cell'] == 'nist_no_route')
        row = matrix.local_cell(case)
        self.assertTrue(row['ok'], row)
        self.assertEqual(row['error'], 'E_LOCAL_NO_ROUTE')


if __name__ == '__main__':
    unittest.main()
