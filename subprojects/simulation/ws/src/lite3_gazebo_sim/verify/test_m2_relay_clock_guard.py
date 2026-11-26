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


if __name__ == '__main__':
    unittest.main()
