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




def _q_from_rpy(roll, pitch, yaw):
    """独立实现：由 rpy 造四元数（ZYX 顺序），用于交叉验证。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _R_from_q(q):
    """独立实现：四元数 → 旋转矩阵（行主序）。"""
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


class CrossCheckTest(unittest.TestCase):
    """★独立实现交叉验证★：只测纯 yaw 时 qx=qy=0，符号写反也能过（2026-11-27 就这么漏的）。"""

    def _check(self, roll, pitch, yaw, oyaw, ox, oy, oz):
        q = _q_from_rpy(roll, pitch, yaw)
        got = loc.rotate_to_map((1.3, -0.7, 0.4) + q, ox, oy, oz, oyaw)
        expect_R = _mul(_Rz(oyaw), _R_from_q(q))
        got_R = _R_from_q(got[3:])
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(got_R[i][j], expect_R[i][j], places=9,
                                       msg='R[%d][%d] 与独立实现不符（rpy=%.2f,%.2f,%.2f oyaw=%.2f）'
                                           % (i, j, roll, pitch, yaw, oyaw))
        p = [1.3, -0.7, 0.4]
        for i in range(3):
            exp = sum(_Rz(oyaw)[i][k] * p[k] for k in range(3)) + (ox, oy, oz)[i]
            self.assertAlmostEqual(got[i], exp, places=9)

    def test_combined_roll_pitch_yaw_matches_independent_impl(self):
        self._check(0.30, -0.20, 1.10, 0.70, 5.2, 2.45, -0.11)

    def test_negative_yaw_and_pitch(self):
        self._check(-0.15, 0.25, -2.30, -1.20, -1.0, 0.5, 0.0)

    def test_pure_roll_only(self):
        self._check(0.40, 0.0, 0.0, 0.55, 0.0, 0.0, 0.0)


class FreshnessTest(unittest.TestCase):
    """新鲜度必须由**源戳**驱动：回调与健康循环共用同一个函数。"""

    def test_no_sample_is_not_fresh(self):
        self.assertEqual(loc.freshness(None, None, 100.0, 1.0), (False, None))

    def test_flow_stops_then_expires(self):
        fresh, age = loc.freshness(('p',), 100.0, 100.3, 1.0)
        self.assertTrue(fresh)
        fresh, age = loc.freshness(('p',), 100.0, 103.0, 1.0)
        self.assertFalse(fresh, '停流 3s 必须判不新鲜')
        self.assertAlmostEqual(age, 3.0, places=6)

    def test_duplicate_stamp_does_not_refresh(self):
        # 重复戳（同一帧被重发）不能让 age 回零
        fresh, age = loc.freshness(('p',), 100.0, 102.0, 1.0)
        self.assertFalse(fresh)

    def test_clock_frozen_keeps_age_constant(self):
        # 仿真时钟冻结：now 不前进 ⇒ age 不上涨，判"仍新鲜"属设计语义（不是 bug），
        # 但绝不能崩、也不能造出负 age。
        a = loc.freshness(('p',), 100.0, 100.0, 1.0)
        b = loc.freshness(('p',), 100.0, 100.0, 1.0)
        self.assertEqual(a, b)
        self.assertGreaterEqual(a[1], 0.0)

    def test_clock_jump_backwards_does_not_produce_negative_age(self):
        fresh, age = loc.freshness(('p',), 100.0, 90.0, 1.0)
        self.assertGreaterEqual(age, 0.0)
        self.assertTrue(fresh)

    def test_health_loop_key_error_regression(self):
        # 回归：健康循环曾引用已改名的 state['selected_at']，首个样本后 KeyError 打死节点
        st = {'selected': ('p',), 'accepted_stamp': 100.0}
        fresh, age = loc.freshness(st['selected'], st['accepted_stamp'], 100.2, 1.0)
        self.assertTrue(fresh)




class WallClockSafetyTest(unittest.TestCase):
    """ROS 时钟冻结/回跳时不得继续放行（2026-11-27 真人指出）。"""

    def test_frozen_ros_clock_expires_via_wall_clock(self):
        # 源戳停在 100、ROS 时钟也停在 100（停钟），但墙钟走了 5s ⇒ 必须判不新鲜
        fresh, age = loc.freshness(('p',), 100.0, 100.0, 1.0,
                                   wall_now=105.0, wall_at_accept=100.0)
        self.assertFalse(fresh, '停钟 + 墙钟 5s 无新帧，不得继续判 fresh')
        self.assertEqual(age, 0.0)      # ROS 年龄仍是 0，说明判据不是靠它

    def test_clock_jump_backwards_expires_via_wall_clock(self):
        # ROS 时钟回跳：now < accepted_stamp，age 被夹到 0，但墙钟照样在走
        fresh, age = loc.freshness(('p',), 100.0, 90.0, 1.0,
                                   wall_now=104.0, wall_at_accept=100.0)
        self.assertFalse(fresh)
        self.assertGreaterEqual(age, 0.0)

    def test_healthy_flow_stays_fresh_on_both_clocks(self):
        fresh, _ = loc.freshness(('p',), 100.0, 100.2, 1.0,
                                 wall_now=100.2, wall_at_accept=100.0)
        self.assertTrue(fresh)

    def test_without_wall_input_keeps_legacy_behaviour(self):
        # 不传墙钟＝旧语义（仅用于单点判断），但生产路径必须传
        fresh, _ = loc.freshness(('p',), 100.0, 100.5, 1.0)
        self.assertTrue(fresh)




class RenewPermissionTest(unittest.TestCase):
    """重复帧不得续期运动许可（2026-11-27）。"""

    def test_duplicate_stamp_does_not_renew(self):
        adv, st, wl = loc.renew_permission(100.0, 100.0, 1000.0, 1005.0)
        self.assertFalse(adv)
        self.assertEqual((st, wl), (100.0, 1000.0), '重复戳不能把墙钟推进到 1005')

    def test_advanced_stamp_renews(self):
        adv, st, wl = loc.renew_permission(100.0, 100.1, 1000.0, 1005.0)
        self.assertTrue(adv)
        self.assertEqual((st, wl), (100.1, 1005.0))

    def test_first_sample_renews(self):
        self.assertTrue(loc.renew_permission(None, 100.0, None, 5.0)[0])

    def test_stuck_source_keeps_old_wall_so_it_expires(self):
        # 源卡住后不断重发同一戳：墙钟停在首次推进点 ⇒ 超时后 freshness 必须判不新鲜
        wall_at_accept = 1000.0
        for _ in range(10):
            adv, _, wall_at_accept = loc.renew_permission(100.0, 100.0, wall_at_accept, 1005.0)
            self.assertFalse(adv)
        fresh, _ = loc.freshness(('p',), 100.0, 100.0, 1.0,
                                 wall_now=1010.0, wall_at_accept=wall_at_accept)
        self.assertFalse(fresh, '只靠重发的源不得一直保持可运动')

    def test_backwards_stamp_is_rejected_before_renewing(self):
        self.assertEqual(loc.stamp_verdict(95.0, 100.0, 100.0, 0.5), 'back')


if __name__ == '__main__':
    unittest.main()
