"""前方超声限速决策单测（纯 Python，无 ROS）。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ultrasonic_guard import SpeedCapDecision  # noqa: E402


class SpeedCapTests(unittest.TestCase):
    def make(self, **kwargs):
        return SpeedCapDecision(cap_distance_m=0.8, clear_distance_m=1.2,
                                cap_speed_mps=0.1, clear_speed_mps=0.5, **kwargs)

    def test_clear(self):
        self.assertEqual(self.make().update(3.5), 0.5)

    def test_near_caps(self):
        self.assertEqual(self.make().update(0.5), 0.1)

    def test_hysteresis_keeps_cap(self):
        d = self.make()
        self.assertEqual(d.update(0.5), 0.1)
        self.assertEqual(d.update(0.9), 0.1)     # still inside the band
        self.assertEqual(d.update(1.3), 0.5)     # beyond clear distance

    def test_hysteresis_keeps_clear(self):
        d = self.make()
        self.assertEqual(d.update(2.0), 0.5)
        self.assertEqual(d.update(0.9), 0.5)     # inside the band, stays clear

    def test_stale_fail_open(self):
        self.assertEqual(self.make(fail_cap=False).update(0.5, fresh=False), 0.5)

    def test_stale_fail_closed(self):
        self.assertEqual(self.make(fail_cap=True).update(0.5, fresh=False), 0.1)

    def test_stop_tier(self):
        d = self.make()
        self.assertEqual(d.update(0.2), 0.0)
        self.assertEqual(d.reason, 'front_stop')
        self.assertEqual(d.update(1.3), 0.5)

    def test_invalid_range(self):
        self.assertEqual(self.make(fail_cap=True).update(0.0), 0.1)
        self.assertEqual(self.make(fail_cap=False).update(99.0), 0.5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
