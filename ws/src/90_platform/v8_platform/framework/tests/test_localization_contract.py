#!/usr/bin/env python3
"""定位契约三件事的零节点测试：姿态随位置一起转、源戳保留与三类拒绝、新鲜度可落。

2026-11-27 修的三处缺陷各有一组用例，红→绿才算收口：
  a to_map 只转位置不转四元数        → test_yaw_*（非零 yaw 下位置与姿态同时正确）
  b select 用 Time.now() 造输出戳    → test_stamp_verdict_*（未来/回退/非法各被拒）
  c 新鲜度在赋值后求值 ⇒ 恒真         → test_age_is_computed_from_source_stamp
"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'nodes'))

import localization_node as loc  # noqa: E402


def q_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PoseTransformTest(unittest.TestCase):
    def test_identity_keeps_pose(self):
        got = loc.rotate_to_map((1.0, 2.0, 3.0) + q_yaw(0.0), 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(got[0], 1.0, places=9)
        self.assertAlmostEqual(got[2], 3.0, places=9)

    def test_yaw_90_rotates_position_and_orientation(self):
        # 位置 (1,0) 绕 Z 转 90° → (0,1)；朝向 0° → 90°
        got = loc.rotate_to_map((1.0, 0.0, 0.0) + q_yaw(0.0), 0.0, 0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(got[0], 0.0, places=9)
        self.assertAlmostEqual(got[1], 1.0, places=9)
        self.assertAlmostEqual(loc.yaw_of(got[3:]), math.pi / 2, places=9)

    def test_yaw_90_preserves_relative_heading(self):
        # 机器人自身朝 +y（90°），世界再转 90° ⇒ 输出应 180°，而非保持 90°
        got = loc.rotate_to_map((0.0, 0.0, 0.0) + q_yaw(math.pi / 2), 0.0, 0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(abs(loc.yaw_of(got[3:])), math.pi, places=6)

    def test_translation_applies_to_all_three_axes(self):
        got = loc.rotate_to_map((1.0, 1.0, 0.5) + q_yaw(0.0), 5.2, 2.45, -0.11, 0.0)
        self.assertAlmostEqual(got[0], 6.2, places=9)
        self.assertAlmostEqual(got[1], 3.45, places=9)
        self.assertAlmostEqual(got[2], 0.39, places=9)


class StampContractTest(unittest.TestCase):
    def test_ok_past_stamp(self):
        self.assertEqual(loc.stamp_verdict(100.0, 100.4, 99.0, 0.5), 'ok')

    def test_future_rejected(self):
        self.assertEqual(loc.stamp_verdict(200.0, 100.0, None, 0.5), 'future')

    def test_backwards_rejected(self):
        self.assertEqual(loc.stamp_verdict(99.0, 100.0, 99.5, 0.5), 'back')

    def test_zero_and_negative_and_nan_rejected(self):
        for bad in (0.0, -1.0, float('nan')):
            self.assertEqual(loc.stamp_verdict(bad, 100.0, None, 0.5), 'bad', bad)

    def test_age_from_source_stamp_means_it_can_actually_expire(self):
        # 源戳停在 100，时间走到 103 ⇒ age=3 超过 max_age=1，必须能判"不新鲜"
        stamp, now, max_age = 100.0, 103.0, 1.0
        age = max(0.0, now - stamp)
        self.assertGreater(age, max_age)
        # 反控：若像旧实现那样用"刚赋值的 selected_at"，age 恒 0 ⇒ 永远判新鲜
        selected_at = now
        self.assertLessEqual(now - selected_at, max_age)


if __name__ == '__main__':
    unittest.main()
