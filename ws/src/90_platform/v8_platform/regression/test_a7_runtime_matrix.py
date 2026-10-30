"""A7 runtime matrix assembly (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import a7_runtime_matrix


class MatrixTests(unittest.TestCase):
    def test_grid_and_summary(self):
        cells = []
        for sid in ('SC01', 'SC07'):
            for track in ('truth', 'estimate'):
                cells.append({'scenario': sid, 'track': track,
                              'outcome': 'pass' if sid == 'SC01' else 'expected_failure',
                              'reason': 'ok' if sid == 'SC01' else 'E_PLAN_NO_ROUTE',
                              'points': 17, 'design_points': 17, 'same_as_design': True})
        result = a7_runtime_matrix.matrix(cells)
        self.assertEqual(result['summary']['cells'], 4)
        self.assertEqual(result['summary']['passed'], 2)
        self.assertEqual(result['summary']['expected_failures'], 2)
        self.assertEqual(result['summary']['unexpected_failures'], 0)
        self.assertEqual(set(result['grid']), {'SC01', 'SC07'})
        self.assertEqual(result['grid']['SC07']['truth']['outcome'], 'expected_failure')

    def test_unexpected_is_counted(self):
        result = a7_runtime_matrix.matrix([{'scenario': 'SC01', 'track': 'truth',
                                            'outcome': 'unexpected_failure',
                                            'same_as_design': False}])
        self.assertEqual(result['summary']['unexpected_failures'], 1)


if __name__ == '__main__':
    unittest.main()
