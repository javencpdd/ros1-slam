"""动态障碍提取单测（纯 Python，无 ROS）。"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(_BASE))
import dynamic_obstacles as dyn  # noqa: E402


def cloud_with_two_objects():
    rng = np.random.default_rng(7)
    ground = np.stack([rng.uniform(-2, 6, 400), rng.uniform(-3, 3, 400),
                       np.zeros(400) + 0.02], axis=1)
    person_a = np.stack([rng.uniform(1.8, 2.2, 90), rng.uniform(-0.3, 0.3, 90),
                         rng.uniform(0.2, 1.6, 90)], axis=1)
    person_b = np.stack([rng.uniform(4.0, 4.6, 70), rng.uniform(1.5, 2.2, 70),
                         rng.uniform(0.2, 1.4, 70)], axis=1)
    return np.vstack([ground, person_a, person_b])


class DynamicObstacleTests(unittest.TestCase):
    def test_roi_and_ground_removed(self):
        points = cloud_with_two_objects()
        kept = dyn.roi_filter(points, (0.0, 0.0), 0.0, 6.0, 3.0, 0.15, 1.5)
        self.assertTrue((kept[:, 2] >= 0.15).all())
        self.assertTrue((np.abs(kept[:, 1]) <= 3.0).all())

    def test_self_filter(self):
        points = np.array([[0.1, 0.0, 0.3], [1.0, 0.0, 0.3]])
        kept = dyn.self_filter(points, (0.0, 0.0), 0.45)
        self.assertEqual(len(kept), 1)

    def test_voxel_downsample(self):
        points = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 1.0, 1.0]])
        centers, keys = dyn.voxel_downsample(points, 0.1)
        self.assertEqual(len(centers), 2)
        self.assertEqual(len(keys), 2)

    def test_two_objects_two_boxes(self):
        points = cloud_with_two_objects()
        kept = dyn.roi_filter(points, (0.0, 0.0), 0.0, 6.0, 3.0, 0.15, 1.5)
        kept = dyn.self_filter(kept, (0.0, 0.0), 0.45)
        centers, keys = dyn.voxel_downsample(kept, 0.1)
        boxes = dyn.cluster_boxes(centers, keys, min_points=5)
        self.assertGreaterEqual(len(boxes), 2)
        spans = sorted(round(b['max'][0] - b['min'][0], 1) for b in boxes)[:2]
        self.assertTrue(all(span < 1.5 for span in spans))

    def test_velocity_from_motion(self):
        before = [{'min': [1.0, 0.0, 0.2], 'max': [1.4, 0.4, 1.4], 'points': 20}]
        after = [{'min': [1.5, 0.0, 0.2], 'max': [1.9, 0.4, 1.4], 'points': 20}]
        velocities = dyn.associate(before, after, dt=0.1, gate_m=0.8, max_speed_mps=10.0)
        self.assertAlmostEqual(velocities[0][0], 5.0, delta=1.0)

    def test_velocity_clamped(self):
        before = [{'min': [0.0, 0.0, 0.2], 'max': [0.4, 0.4, 1.4], 'points': 20}]
        after = [{'min': [10.0, 0.0, 0.2], 'max': [10.4, 0.4, 1.4], 'points': 20}]
        velocities = dyn.associate(before, after, dt=0.1, gate_m=0.8, max_speed_mps=0.5)
        self.assertEqual(velocities[0], (0.0, 0.0, 0.0))  # beyond gate -> unmatched

    def test_payload_schema(self):
        box = {'min': [0.0, 0.0, 0.2], 'max': [1.0, 1.0, 1.5], 'points': 9}
        payload = dyn.to_payload([box], [(0.2, 0.1, 0.0)], stamp=100.0, ttl_s=1.0)
        self.assertEqual(len(payload), 1)
        self.assertEqual(payload[0]['active_from'], 99.0)
        self.assertEqual(payload[0]['active_until'], 101.0)
        self.assertEqual(payload[0]['velocity'], [0.2, 0.1, 0.0])

    def test_empty_input(self):
        empty = np.empty((0, 3))
        self.assertEqual(len(dyn.roi_filter(empty, (0, 0), 0, 6, 3, 0.1, 1.5)), 0)
        centers, keys = dyn.voxel_downsample(empty, 0.1)
        self.assertEqual(len(dyn.cluster_boxes(centers, keys)), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
