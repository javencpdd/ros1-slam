#!/usr/bin/env python3
"""path_goal 航点推进（tick）的语义测试（2026-11-26）。

真实缺陷（靠 `nav_timeline.tsv` 才看清）：路径点间距 0.1 m，而 local 每段只走 0.30 m、
给太近的目标根本不产生速度 —— 旧实现"每 tick 前进一格"，于是大量目标落在 0.1/0.2 m 处，
`mux reason=track` 而 `cmd` 恒零；同时兜底超时用 `rospy.Time.now()`，该节点没有 `/clock`
时恒为 0 ⇒ 超时永不成立 ⇒ index 冻结（实测 139 s 停在 index=5/14）。

本测试锁定三条：
  ① 推进必须跳过"太近"的点（< min_goal_m），直接给够远的一段；
  ② 当前目标还够远且未超时 ⇒ 不推进、不重发；
  ③ 超时用**墙钟**：published_at 设在过去 → 必须推进（用 rospy 时间则永远 0 时不成立）。
"""
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock

for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
              'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(_name, MagicMock())


class _String(object):
    def __init__(self, data=''):
        self.data = data


sys.modules['std_msgs.msg'].String = _String

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import path_goal_node as pg  # noqa: E402


def _node(points, pose=(0.0, 0.0), index=0, published_at=None):
    o = pg.PathGoal.__new__(pg.PathGoal)
    o.poses = []
    for i, (x, y) in enumerate(points):
        ps = types.SimpleNamespace()
        ps.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0))
        o.poses.append(ps)
    o.index = index
    o.pose = types.SimpleNamespace(x=pose[0], y=pose[1], z=0.0)
    o.arrive_m = 0.30
    o.min_goal_m = 0.30
    o.advance_timeout_s = 8.0
    o.published_at = published_at
    o.cancelled_at = None
    o.published = []
    o.publish_current = lambda: o.published.append(o.index)
    return o


class TickAdvance(unittest.TestCase):
    def test_skips_too_close_points(self):
        """点间距 0.1 m：从原点起，第一个"够远"的点是 0.3 m 处（下标 2：0.1/0.2 太近，跳过）。"""
        o = _node([(0.1 * i, 0.0) for i in range(1, 11)], pose=(0.0, 0.0))
        o.tick(None)
        self.assertEqual(o.index, 2, '必须跳过 0.1/0.2 两个太近的点')
        self.assertEqual(o.published, [2], '一次 tick 只发一个目标')

    def test_far_enough_target_not_advanced(self):
        o = _node([(0.1 * i, 0.0) for i in range(1, 11)], pose=(0.0, 0.0), index=5)
        import time as _t
        o.published_at = _t.monotonic()          # 刚发过，未超时
        o.tick(None)
        self.assertEqual(o.index, 5)
        self.assertEqual(o.published, [], '够远且未超时 ⇒ 不重发')

    def test_wallclock_timeout_advances(self):
        """反控：把 published_at 放到 9 s 前 ⇒ 必须推进（用 rospy 时间的话这里永远是 0）。"""
        import time as _t
        o = _node([(0.1 * i, 0.0) for i in range(1, 11)], pose=(0.0, 0.0), index=5)
        o.published_at = _t.monotonic() - 9.0
        o.tick(None)
        self.assertGreater(o.index, 5, '超时必须推进')
        self.assertEqual(len(o.published), 1)

    def test_tail_falls_back_to_last_point(self):
        """末段剩余不足 min_goal_m 时必须兜底发最后一点，否则永远到不了终点。"""
        o = _node([(0.1 * i, 0.0) for i in range(1, 11)], pose=(0.95, 0.0), index=9)
        import time as _t
        o.published_at = _t.monotonic() - 9.0
        o.tick(None)
        self.assertEqual(o.index, 9)             # 已是最后一点，保持
        self.assertEqual(o.published, [9])


if __name__ == '__main__':
    unittest.main()
