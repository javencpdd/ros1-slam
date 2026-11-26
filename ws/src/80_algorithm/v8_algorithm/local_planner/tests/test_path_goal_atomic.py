#!/usr/bin/env python3
"""path_goal 原子控制通道的语义测试（真人 2026-11-26 第 3/4 条）。

覆盖：
  ① 完全重复（identity+内容相同）⇒ 不重置、不重发；
  ② 同一 mission 的 revision **回退** ⇒ 拒绝；
  ③ 撤销过的 mission ⇒ 拒绝（任何 revision 都不复活）；
  ④ 畸形/缺字段 ⇒ 拒绝；
  ⑤ 新 revision 的进度**按几何重算**（不沿用旧 index）。
"""
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock

for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
              'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(_name, MagicMock())

# String 必须是"能存 data 的类"，否则 String(data=..) 得到 MagicMock，读 .data 拿不到字符串
class _String(object):
    def __init__(self, data=''):
        self.data = data


sys.modules['std_msgs.msg'].String = _String

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import path_goal_node as pg  # noqa: E402


def _node():
    o = pg.PathGoal.__new__(pg.PathGoal)      # 绕过 __init__ 的 rospy 依赖
    o.poses = []
    o.index = 0
    o.task_meta = None
    o.revoked_mission = -1
    o.pending_meta = None
    o.last_goal_key = None
    o.accepted_generation = 0
    o.status_pub = MagicMock()
    o.goal_pub = MagicMock()
    o.goal_meta_pub = MagicMock()
    o.goal_task_pub = MagicMock()
    o.pose = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    o.published = []
    o.publish_current = lambda: o.published.append(o.index)
    o._first_unreached = lambda: 2 if len(o.poses) > 2 else 0
    return o


def _msg(payload):
    m = MagicMock()
    m.data = json.dumps(payload)
    return m


def _task(mission=1, rev=0, poses=((0, 0, 0), (1, 0, 0), (2, 0, 0))):
    return {'mission_id': mission, 'path_revision': rev, 'waypoint_index': 0,
            'execution_generation': 0, 'poses': [list(p) for p in poses]}


class PathGoalAtomic(unittest.TestCase):
    def test_duplicate_is_noop(self):
        o = _node()
        o.on_path_task(_msg(_task()))
        first = len(o.published)
        o.on_path_task(_msg(_task()))     # 完全重复
        self.assertEqual(len(o.published), first)

    def test_revision_rollback_rejected(self):
        o = _node()
        o.on_path_task(_msg(_task(rev=3)))
        before = len(o.published)
        o.on_path_task(_msg(_task(rev=2, poses=((0, 0, 0), (9, 9, 9)))))
        self.assertEqual(len(o.published), before)
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'stale_revision')

    def test_revoked_mission_rejected(self):
        o = _node()
        o.revoked_mission = 5
        o.on_path_task(_msg(_task(mission=5, rev=9)))
        self.assertEqual(len(o.published), 0)
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'task_revoked')

    def test_malformed_rejected(self):
        o = _node()
        bad = MagicMock()
        bad.data = 'not-json'
        o.on_path_task(bad)
        self.assertEqual(len(o.published), 0)
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'path_task_malformed')

    def test_new_revision_recomputes_progress(self):
        o = _node()
        o.on_path_task(_msg(_task(rev=1)))
        o.index = 0
        o.on_path_task(_msg(_task(rev=2, poses=((0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)))))
        self.assertEqual(o.index, 2)      # 几何重算（这里用桩返回 2），不沿用旧 index

    def test_late_mission_rejected(self):
        """晚到：旧 mission 在新 mission 之后到达必须拒（跟撤销线无关，纯编号顺序）。"""
        o = _node()
        o.on_path_task(_msg(_task(mission=3, rev=0)))
        o.on_path_task(_msg(_task(mission=1, rev=9)))
        self.assertEqual(len(o.published), 1, '旧 mission 晚到不许再驱动 local')
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'stale_mission')

    def test_cancel_revokes_mission(self):
        """取消＝撤销：旧任务带着"更新的时间戳"重放也必须被拒（判据是 mission，不是 stamp）。"""
        o = _node()
        o.on_path_task(_msg(_task(mission=3, rev=0)))
        self.assertEqual(len(o.published), 1)
        o.on_cancel(MagicMock(data='7:current:user'))
        self.assertEqual(o.revoked_mission, 3)
        # 旧任务重放：内容一模一样（新的 timestamp 在原子消息里根本没用），必须拒
        o.on_path_task(_msg(_task(mission=3, rev=0)))
        self.assertEqual(len(o.published), 1, '取消后的旧 mission 不许复活')
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'task_revoked')

    def test_cancel_allows_genuinely_new_mission(self):
        """反控：取消只封旧 mission，新 mission（编号更大）必须照常放行。"""
        o = _node()
        o.on_path_task(_msg(_task(mission=3, rev=0)))
        o.on_cancel(MagicMock(data='7:current:user'))
        o.on_path_task(_msg(_task(mission=4, rev=0)))
        self.assertEqual(len(o.published), 2)


if __name__ == '__main__':
    unittest.main()
