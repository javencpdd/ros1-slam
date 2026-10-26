"""闭环收敛测试：用简化运动学把 follower 的 (v,w) 积分成轨迹。

不是 Gazebo 仿真，只证明控制律在闭环里能收敛、能按序过点、取消即零速。
"""
import math, os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower

DT = 0.1


def run(waypoints, cancel_at=None, max_steps=2000):
    pose = [0.0, 0.0, 0.0]
    index, trail, steps = 0, [], 0
    while index < len(waypoints) and steps < max_steps:
        if cancel_at is not None and steps >= cancel_at:
            v, w = follower.stop_twist()
            trail.append((pose[0], pose[1], pose[2], v, w, 'cancel'))
            break
        v, w, arrived = follower.twist_for(tuple(pose), waypoints[index])
        if arrived:
            index += 1
            continue
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
        pose[2] = follower.wrap(pose[2] + w * DT)
        trail.append((pose[0], pose[1], pose[2], v, w, 'move'))
        steps += 1
    return pose, index, trail, steps


class ClosedLoopTests(unittest.TestCase):
    def test_reaches_all_waypoints_in_order(self):
        waypoints = [(1.0, 0.0), (1.0, 1.0), (2.0, 1.0)]
        pose, done, trail, steps = run(waypoints)
        self.assertEqual(done, len(waypoints))
        self.assertLess(steps, 1500)
        for wx, wy in waypoints:
            self.assertLess(min(math.hypot(t[0]-wx, t[1]-wy) for t in trail), 0.15)

    def test_cancel_gives_zero_twist(self):
        pose, done, trail, _ = run([(3.0, 0.0)], cancel_at=5)
        self.assertEqual(trail[-1][5], 'cancel')
        self.assertEqual((trail[-1][3], trail[-1][4]), (0.0, 0.0))
        self.assertLess(done, 1)

    def test_never_exceeds_speed_limits(self):
        _, _, trail, _ = run([(2.0, 0.5)])
        self.assertTrue(all(t[3] <= 0.25 + 1e-9 and abs(t[4]) <= 0.8 + 1e-9 for t in trail))


if __name__ == '__main__':
    unittest.main(verbosity=2)
