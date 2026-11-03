"""卡尔曼多目标跟踪单测（纯 Python，无 ROS）。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from object_tracker import MultiObjectTracker  # noqa: E402


def box(cx, cy, size=0.4, zmin=0.2, zmax=1.6):
    return {'min': [cx - size / 2, cy - size / 2, zmin],
            'max': [cx + size / 2, cy + size / 2, zmax]}


class TrackerTests(unittest.TestCase):
    def make(self):
        return MultiObjectTracker(gate_m=0.8, max_miss=3, sigma_a=0.5,
                                   sigma_pos=0.02, max_speed_mps=2.0)

    def test_constant_velocity_estimated(self):
        tracker = self.make()
        x = 0.0
        for _ in range(12):
            x += 0.05                      # 0.5 m/s at dt=0.1
            tracks = tracker.update([box(x, 0.0)], 0.1)
        self.assertEqual(len(tracks), 1)
        vx, vy, _ = tracks[0].velocity()
        self.assertAlmostEqual(vx, 0.5, delta=0.15)
        self.assertAlmostEqual(vy, 0.0, delta=0.1)

    def test_static_object_velocity_near_zero(self):
        tracker = self.make()
        for _ in range(10):
            tracks = tracker.update([box(2.0, 1.0)], 0.1)
        vx, vy, _ = tracks[0].velocity()
        self.assertLess(abs(vx) + abs(vy), 0.15)

    def test_two_objects_keep_two_tracks(self):
        tracker = self.make()
        for _ in range(8):
            tracks = tracker.update([box(1.0, 0.0), box(4.0, 2.0)], 0.1)
        self.assertEqual(len(tracks), 2)
        xs = sorted(t.predicted_center()[0] for t in tracks)
        self.assertLess(abs(xs[0] - 1.0), 0.3)
        self.assertLess(abs(xs[1] - 4.0), 0.3)

    def test_coast_then_drop(self):
        tracker = self.make()
        for _ in range(3):
            tracker.update([box(1.0, 0.0)], 0.1)
        for _ in range(2):                 # missed frames -> coast
            tracks = tracker.update([], 0.1)
            self.assertEqual(len(tracks), 1)
        for _ in range(3):                 # beyond max_miss -> dropped
            tracks = tracker.update([], 0.1)
        self.assertEqual(len(tracks), 0)

    def test_new_object_gets_new_id(self):
        tracker = self.make()
        first = tracker.update([box(1.0, 0.0)], 0.1)[0]
        second = tracker.update([box(1.0, 0.0), box(3.0, 0.0)], 0.1)
        ids = sorted(t.id for t in second)
        self.assertIn(first.id, ids)
        self.assertEqual(len(ids), 2)

    def test_box_size_preserved(self):
        tracker = self.make()
        for _ in range(5):
            tracks = tracker.update([box(1.0, 0.0, size=0.6)], 0.1)
        b = tracks[0].box()
        self.assertAlmostEqual(b['max'][0] - b['min'][0], 0.6, places=6)
        self.assertAlmostEqual(b['min'][2], 0.2, places=6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
