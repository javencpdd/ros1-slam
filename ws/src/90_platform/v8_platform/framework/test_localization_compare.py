import json
import sys
import unittest
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import localization_compare as lc


class CompareTests(unittest.TestCase):
    def test_error_stats_and_alignment(self):
        gt = [(0., 0., 0., 0., 0., 0., 1.), (1., 0., 0., 0., 0., 0., 1.)]
        est = [(5., 2., 0., 0., 0., 0., 1.), (6., 2., 0., 0., 0., 0., 1.)]
        result = lc.compare_samples(list(zip(gt, est)))
        self.assertEqual(result['origin_offset'], [5., 2., 0.])
        self.assertAlmostEqual(result['raw']['xy_m_mean'], 29 ** .5)
        self.assertAlmostEqual(result['translated']['distance_m_max'], 0.)
        self.assertAlmostEqual(result['rigid']['distance_m_max'], 0.)

    def test_empty(self):
        self.assertEqual(lc.compare_samples([])['raw'], None)


if __name__ == '__main__':
    unittest.main()
