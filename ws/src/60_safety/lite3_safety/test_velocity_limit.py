"""速度钳位单测（纯 Python，无 ROS）。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / 'scripts'))
from velocity_limit import clamp  # noqa: E402


class VelocityLimitTests(unittest.TestCase):
    def test_scales_linear(self):
        out = clamp(0.3, 0.4, 0.0, 0.0, 0.0, 0.0, 0.5, None)
        self.assertAlmostEqual((out[0] ** 2 + out[1] ** 2) ** 0.5, 0.5, places=9)
        self.assertAlmostEqual(out[0] / out[1], 0.3 / 0.4, places=9)

    def test_under_limit_untouched(self):
        out = clamp(0.1, 0.2, 0.0, 0.0, 0.0, 0.3, 0.5, 1.0)
        self.assertEqual(out, [0.1, 0.2, 0.0, 0.0, 0.0, 0.3])

    def test_angular_scaled(self):
        out = clamp(0.0, 0.0, 0.0, 0.0, 0.0, 2.0, None, 1.0)
        self.assertAlmostEqual(out[5], 1.0, places=9)

    def test_zero_stays_zero(self):
        self.assertEqual(clamp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0),
                         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_none_means_unbounded(self):
        self.assertEqual(clamp(9.0, 0.0, 0.0, 9.0, 0.0, 0.0, None, None),
                         [9.0, 0.0, 0.0, 9.0, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
