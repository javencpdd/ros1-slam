"""Bounds and determinism of the estimated-localization perturbation."""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import estimated_track as track


class EstimatedTrackTests(unittest.TestCase):
    def test_bounded_for_all_seeds(self):
        for seed in (0, 1, 2, 7):
            for step in range(0, 400):
                t = step * 0.05
                x, y, yaw = track.offset(t, seed)
                self.assertLessEqual(abs(x), track.AMP_XY_M + 1e-12)
                self.assertLessEqual(abs(y), track.AMP_XY_M + 1e-12)
                self.assertLessEqual(abs(yaw), track.AMP_YAW_RAD + 1e-12)

    def test_deterministic_and_seed_sensitive(self):
        for t in (0.0, 1.7, 9.3):
            self.assertEqual(track.offset(t, 1), track.offset(t, 1))
        samples = [track.offset(2.9, seed) for seed in (0, 1, 2)]
        self.assertGreater(len({tuple(round(v, 9) for v in s) for s in samples}), 1)

    def test_input_guard(self):
        for bad in ((-1.0, 0), (1.0, 1.5), (float('nan'), 0)):
            with self.assertRaises(ValueError):
                track.offset(*bad)
        with self.assertRaises(ValueError):
            track.offset(1.0, True)


if __name__ == '__main__':
    unittest.main()
