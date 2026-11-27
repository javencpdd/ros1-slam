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
                                      clock=lambda: 0.0, sleep=lambda s: None,
                                      is_shutdown=lambda: False)
        self.assertIsNone(err)
        # 绕 Y +15°：把 (0,0,1) 旋过去应得 (sin15, 0, cos15)
        v = relay._mat_vec(got, (0.0, 0.0, 1.0))
        self.assertAlmostEqual(v[0], math.sin(math.radians(15.0)), places=6)
        self.assertAlmostEqual(v[2], math.cos(math.radians(15.0)), places=6)

    def test_e_timeout_uses_wall_clock_and_returns_reason(self):
        ticks = [0.0]
        def clock():
            ticks[0] += 10.0      # 每次读表都推进，模拟墙钟流逝
            return ticks[0]
        def boom():
            raise RuntimeError('no tf yet')
        got, err = relay._tf_rotation('base_link', 'rslidar', timeout=15.0,
                                      lookup=boom, clock=clock, sleep=lambda s: None,
                                      is_shutdown=lambda: False)
        self.assertIsNone(got)
        self.assertIn('不可得', err or '')

    def test_f_shutdown_aborts_quickly(self):
        got, err = relay._tf_rotation('base_link', 'rslidar', timeout=30.0,
                                      lookup=lambda: (_ for _ in ()).throw(RuntimeError('x')),
                                      clock=lambda: 0.0, sleep=lambda s: None,
                                      is_shutdown=lambda: True)
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


if __name__ == '__main__':
    unittest.main()
