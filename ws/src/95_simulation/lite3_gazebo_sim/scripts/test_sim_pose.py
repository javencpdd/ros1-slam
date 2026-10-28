"""sim_pose 纯函数测试（宿主无 ROS 可跑）。"""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_pose  # noqa: E402


class QuaternionTests(unittest.TestCase):
    def test_yaw_degrees(self):
        self.assertEqual(sim_pose.quaternion_from_yaw_deg(0), (0.0, 0.0, 0.0, 1.0))
        x, y, z, w = sim_pose.quaternion_from_yaw_deg(90)
        self.assertEqual((x, y), (0.0, 0.0))
        self.assertAlmostEqual(z, math.sqrt(0.5))
        self.assertAlmostEqual(w, math.sqrt(0.5))
        x, y, z, w = sim_pose.quaternion_from_yaw_deg(180)
        self.assertAlmostEqual(z, 1.0)
        self.assertAlmostEqual(abs(w), 0.0, places=9)

    def test_invalid_yaw(self):
        for value in (float('nan'), float('inf'), True, '30'):
            with self.assertRaises(ValueError):
                sim_pose.quaternion_from_yaw_deg(value)

    def test_pose_xyz(self):
        self.assertEqual(sim_pose.pose_xyz([1, -2.5], 'start'), [1.0, -2.5])
        for value in ([1], [1, 2, 3], [1, float('nan')], [True, 0]):
            with self.assertRaises(ValueError):
                sim_pose.pose_xyz(value, 'start')


if __name__ == '__main__':
    unittest.main()
