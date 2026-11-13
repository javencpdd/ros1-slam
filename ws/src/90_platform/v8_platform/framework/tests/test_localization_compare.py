import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import json
import math
import sys
import unittest
from pathlib import Path
HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
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

    @staticmethod
    def _q(yaw):
        return (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))

    def test_rpe_zero_for_constant_offset(self):
        gt = [(float(i), 0., 0.) + self._q(0.) for i in range(3)]
        est = [(float(i) + 5., 2., 0.) + self._q(0.) for i in range(3)]
        result = lc.rpe_stats([(float(i), gt[i], est[i]) for i in range(3)], 1.)
        self.assertEqual(result['samples'], 2)
        self.assertAlmostEqual(result['translation_m']['max'], 0.)
        self.assertAlmostEqual(result['z_m']['max'], 0.)
        self.assertAlmostEqual(result['yaw_rad']['max'], 0.)

    def test_rpe_detects_relative_yaw(self):
        gt0 = (0., 0., 0.) + self._q(0.)
        gt1 = (1., 0., 0.) + self._q(0.)
        est0 = (0., 0., 0.) + self._q(0.)
        est1 = (1., 0., 0.) + self._q(0.2)
        result = lc.rpe_stats([(0., gt0, est0), (1., gt1, est1)], 1.)
        self.assertAlmostEqual(result['yaw_rad']['max'], 0.2, places=6)
        self.assertAlmostEqual(result['translation_m']['max'], 0., places=6)

    def test_tracking_stats_gaps_and_segments(self):
        pose = (0., 0., 0.) + self._q(0.)
        est = [(0., 0., pose), (0.1, 0., pose), (2.2, 0., pose)]
        pairs = [(pose, pose), ((0.8, 0., 0.) + self._q(0.), pose)]
        result = lc.tracking_stats(est, pairs, gap_s=1.0, xy_threshold_m=0.5)
        self.assertEqual(result['gaps_over_s'], 1)
        self.assertAlmostEqual(result['max_gap_s'], 2.1, places=6)
        self.assertEqual(result['xy_over_threshold_samples'], 1)
        self.assertEqual(result['xy_over_threshold_segments'], 1)

    def test_tracking_stats_removes_constant_frame_offset(self):
        pose = (0., 0., 0.) + self._q(0.)
        shifted = (5., 2., 0.) + self._q(0.)
        pairs = [(pose, shifted), (pose, shifted)]
        result = lc.tracking_stats([(0., 0., shifted), (0.1, 0., shifted)],
                                   pairs, xy_threshold_m=0.5)
        self.assertEqual(result['xy_over_threshold_samples'], 0)
        self.assertEqual(result['xy_over_threshold_segments'], 0)


if __name__ == '__main__':
    unittest.main()
