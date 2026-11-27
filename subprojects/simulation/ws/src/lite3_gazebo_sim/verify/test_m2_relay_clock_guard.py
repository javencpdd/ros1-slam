#!/usr/bin/env python3
"""零节点负控：证明 m2_cmd_relay 的心跳守卫在**时钟回跳**时不会退出。

为什么要这一条：ROSTimeMovedBackwardsException 是 ROSInterruptException 的**子类**
（MRO: [ROSTimeMovedBackwardsException, ROSInterruptException, ROSException, Exception]），
把父类的 handler 写在前面会让"回跳"走进"正常关机"分支并 break —— 实测就这么被杀过一次。

三连：
  A 正控   —— 正常 tick            ⇒ 返回 True（继续转发）
  B 负控   —— 抛 ROSTimeMovedBackwardsException ⇒ 必须返回 True 且告警被调用
  C 反控   —— 抛普通 ROSInterruptException       ⇒ 必须返回 False（真的是一次关机请求）
若 B 返回 False，说明异常顺序又写反了。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))

try:
    import rospy  # noqa: E402
    import m2_cmd_relay as relay  # noqa: E402
except ImportError as _e:      # 无 ROS 环境（如干净 CI）跳过，不伪造通过
    rospy = None
    relay = None
    _SKIP = '需要 ROS 环境（source /opt/ros/noetic/setup.bash）：%s' % _e
else:
    _SKIP = None


class FakeRate:
    def __init__(self, exc=None):
        self.exc = exc
        self.calls = 0

    def sleep(self):
        self.calls += 1
        if self.exc is not None:
            raise self.exc


@unittest.skipIf(_SKIP is not None, _SKIP or '')
class ClockGuardTest(unittest.TestCase):
    def test_a_normal_tick_continues(self):
        r = FakeRate()
        self.assertTrue(relay._sleep_guarded(r))
        self.assertEqual(r.calls, 1)

    def test_b_backwards_jump_does_not_exit(self):
        warns = []
        r = FakeRate(rospy.ROSTimeMovedBackwardsException(5.0))
        self.assertTrue(relay._sleep_guarded(r, warns.append),
                        '时钟回跳被当成了关机请求：异常捕获顺序又写反了')
        self.assertEqual(len(warns), 1, '回跳必须留下告警，便于现场定位')

    def test_c_real_shutdown_still_exits(self):
        r = FakeRate(rospy.ROSInterruptException('shutdown'))
        self.assertFalse(relay._sleep_guarded(r), '正常关机请求必须能退出，否则节点无法停止')




class _Q:
    """最小四元数桩：绕 Y 转 theta。"""
    def __init__(self, theta):
        import math
        self.x, self.y = 0.0, math.sin(theta / 2.0)
        self.z, self.w = 0.0, math.cos(theta / 2.0)


class _Tr:
    class _T:
        def __init__(self, q):
            self.rotation = q
    def __init__(self, q):
        self.transform = self._T(q)


class TfRotationTest(unittest.TestCase):
    """★真实函数调用测试★：py_compile 抓不到 NameError。

    2026-11-27 就是这么漏的：_tf_rotation 里用了 time.time() 但模块没 import time，
    py_compile 全过，真跑到 tf 模式时 NameError 当场把 relay 打死。
    这组用例必须真的**调用**该函数。
    """

    def test_d_success_returns_rotation_matrix(self):
        import math
        got, err = relay._tf_rotation('base_link', 'rslidar', timeout=1.0,
                                      lookup=lambda: _Tr(_Q(math.radians(15.0))),
                                      sleep=lambda s: None,
                                      is_shutdown=lambda: False)
        self.assertIsNone(err)
        # 绕 Y +15°：把 (0,0,1) 旋过去应得 (sin15, 0, cos15)
        v = relay._mat_vec(got, (0.0, 0.0, 1.0))
        self.assertAlmostEqual(v[0], math.sin(math.radians(15.0)), places=6)
        self.assertAlmostEqual(v[2], math.cos(math.radians(15.0)), places=6)

    def test_e_timeout_returns_reason(self):
        import time as _t
        def boom():
            raise RuntimeError('no tf yet')
        got, err = relay._tf_rotation('base_link', 'rslidar', timeout=0.1,
                                      lookup=boom, sleep=lambda s: _t.sleep(0.01),
                                      is_shutdown=lambda: False)
        self.assertIsNone(got)
        self.assertIn('不可得', err or '')

    def test_f_shutdown_aborts_quickly(self):
        got, err = relay._tf_rotation('base_link', 'rslidar', timeout=30.0,
                                      lookup=lambda: (_ for _ in ()).throw(RuntimeError('x')),
                                      sleep=lambda s: None, is_shutdown=lambda: True)
        self.assertIsNone(got)
        self.assertIn('shutdown', err or '')

    def test_g_stationary_gravity_becomes_vertical_after_rotation(self):
        """静止时 rslidar 系看到的是"倾斜的重力"，旋到 base_link 应回到近似 (0,0,9.8)。"""
        import math
        q = _Q(math.radians(15.0))
        R = relay._quat_to_matrix(_Q(-math.radians(15.0)))   # 逆变换：把倾斜量转回来
        tilted = relay._mat_vec(R, (0.0, 0.0, 9.81))
        back = relay._mat_vec(relay._quat_to_matrix(q), tilted)
        self.assertAlmostEqual(back[0], 0.0, places=6)
        self.assertAlmostEqual(back[2], 9.81, places=6)




class FramePolicyTest(unittest.TestCase):
    """输入 frame 的三态：转 / 跳过（防重复旋转）/ 丢弃（未知系）。"""

    def test_h_source_frame_rotates(self):
        self.assertEqual(relay._frame_action('rslidar', 'rslidar', 'base_link'), 'rotate')

    def test_i_already_output_frame_skips(self):
        # 上游已经转过：再转一次＝30°，必须跳过
        self.assertEqual(relay._frame_action('base_link', 'rslidar', 'base_link'), 'skip')

    def test_j_unknown_frame_is_dropped(self):
        self.assertEqual(relay._frame_action('map', 'rslidar', 'base_link'), 'drop')

    def test_k_no_src_declared_still_guards_double_rotation(self):
        self.assertEqual(relay._frame_action('anything', '', 'base_link'), 'rotate')
        self.assertEqual(relay._frame_action('base_link', '', 'base_link'), 'skip')


class WallClockWaitTest(unittest.TestCase):
    """时钟冻结/回跳对照：截止只认 time.monotonic，坏时钟注不进去。"""

    def test_l_frozen_lookup_returns_by_wall_deadline(self):
        import time as _t
        n = [0]
        def lookup():
            n[0] += 1
            raise RuntimeError('frozen clock, no tf')
        t0 = _t.monotonic()
        got, err = relay._tf_rotation('a', 'b', timeout=0.2, lookup=lookup,
                                      sleep=lambda s: _t.sleep(0.01),
                                      is_shutdown=lambda: False)
        self.assertIsNone(got)
        self.assertIn('不可得', err or '')
        self.assertLess(_t.monotonic() - t0, 2.0, '必须按墙钟 0.2s 截止，不能死等')

    def test_m_clock_cannot_be_injected_to_bend_deadline(self):
        import time as _t
        # 即便调用方塞一个"永远不动"的 clock，也不该影响截止（实现里已固定用 monotonic）
        t0 = _t.monotonic()
        got, err = relay._tf_rotation('a', 'b', timeout=0.15,
                                      lookup=lambda: (_ for _ in ()).throw(RuntimeError('x')),
                                      clock=lambda: 0.0, sleep=lambda s: _t.sleep(0.01),
                                      is_shutdown=lambda: False)
        self.assertIsNone(got)
        self.assertLess(_t.monotonic() - t0, 2.0)

if __name__ == '__main__':
    unittest.main()
