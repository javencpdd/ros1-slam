#!/usr/bin/env python3
"""path_goal 的**版本仲裁**离线正反控（2026-11-26 真人裁决③/④/⑤）。

FAR 的两点路径首点随位姿 10 Hz 变，如果每一帧都当新任务：
  · `path_revision` 天天变 ⇒ 每帧重置执行；
  · local 每 100 ms 收一次新目标 ⇒ 在飞动作被打断，车几乎不动。

所以钉住四条：
  ① 同一 (mission, path_revision)、只有点列变 ⇒ **只刷新几何**，不发目标、不改 index/代次；
  ② revision 前进 ⇒ 正常接受并下发目标（有效航点变化要能生效）；
  ③ 带 `invalidated` ⇒ 清空活动路径且**不发目标**（失效优先于去重）；
  ④ 10 Hz 刷新 100 帧 ⇒ 目标只发 1 次（执行重置计数必须保持 1）。
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


class _String(object):
    def __init__(self, data=''):
        self.data = data


sys.modules['std_msgs.msg'].String = _String

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import path_goal_node as pg  # noqa: E402


# ★不能直接吃 MagicMock 的 `PoseStamped`★：`MagicMock` 的 `return_value` 是**同一个对象**，
# 每次"新建"其实都拿到它 ⇒ 所有航点会变成最后一次赋值的值（实测两个点都成了 (1.0,0)）。
class _Point(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Pose(object):
    def __init__(self):
        self.position = _Point()
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _PoseStamped(object):
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=None)
        self.pose = _Pose()


pg.PoseStamped = _PoseStamped


def _node(mission=1, revision=1):
    o = pg.PathGoal.__new__(pg.PathGoal)
    o.poses = []
    o.index = 0
    o.task_meta = {'mission_id': mission, 'path_revision': revision,
                   'waypoint_index': 0, 'execution_generation': 0}
    o.revoked_mission = -1
    o.pending_meta = None
    o.last_goal_key = None
    o.accepted_generation = 0
    o.atomic_mode = True
    o.status_pub = MagicMock()
    o.goal_pub = MagicMock()
    o.goal_meta_pub = MagicMock()
    o.goal_task_pub = MagicMock()
    o.pose = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    o.arrive_m = 0.30
    o.min_goal_m = 0.30
    o.published = []
    o.publish_current = lambda: o.published.append(o.index)
    return o


def _msg(payload):
    m = MagicMock()
    m.data = json.dumps(payload)
    return m


def _task(mission=1, revision=1, poses=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))):
    return {'mission_id': mission, 'path_revision': revision, 'waypoint_index': 0,
            'execution_generation': 0, 'poses': [list(p) for p in poses]}


class TaskRevision(unittest.TestCase):
    def test_same_revision_refreshes_without_reset(self):
        o = _node()
        o.on_path_task(_msg(_task()))                      # 首次接受
        first = len(o.published)
        o.index = 0
        gen = o.accepted_generation
        # 位姿动了：首点变，但 mission/revision 没变
        o.on_path_task(_msg(_task(poses=((0.15, 0.0, 0.0), (1.0, 0.0, 0.0)))))
        self.assertEqual(len(o.published), first, '同版本刷新不得重发目标')
        self.assertEqual(o.accepted_generation, gen, '同版本刷新不得推执行代次')
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertTrue(last.get('refreshed'), '必须回执 refreshed，便于读数')
        self.assertEqual(o.poses[0].pose.position.x, 0.15, '几何必须真的刷新了')

    def test_new_revision_is_executed(self):
        o = _node()
        o.on_path_task(_msg(_task()))
        first = len(o.published)
        o.on_path_task(_msg(_task(revision=2, poses=((0.0, 0.0, 0.0), (0.6, 0.0, 0.0)))))
        self.assertGreater(len(o.published), first, '有效航点变化必须能下发')
        self.assertEqual(o.accepted_generation, 2, '两次接受（首次 + 换版本）各计一次')

    def test_invalidation_clears_and_does_not_publish(self):
        o = _node()
        o.on_path_task(_msg(_task()))
        before = len(o.published)
        payload = _task(revision=2)
        payload['poses'] = []
        payload['invalidated'] = 'far_waypoint_stale'
        o.on_path_task(_msg(payload))
        self.assertEqual(o.poses, [], '失效必须清空活动路径')
        self.assertEqual(len(o.published), before, '失效帧不得再下发目标')
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'path_invalidated')

    def test_pose_stream_refresh_is_idempotent(self):
        """反控：10 Hz 位姿流刷 100 帧，目标只能发 1 次（执行重置 0 次）。"""
        o = _node()
        o.on_path_task(_msg(_task()))
        self.assertEqual(len(o.published), 1)
        for i in range(100):
            o.on_path_task(_msg(_task(poses=((0.001 * i, 0.0, 0.0), (1.0, 0.0, 0.0)))))
        self.assertEqual(len(o.published), 1, '刷新帧一律不得触发新目标')

    def test_stale_mission_still_rejected_under_refresh(self):
        o = _node(mission=5, revision=1)
        o.on_path_task(_msg(_task()))
        before = len(o.published)
        o.on_path_task(_msg(_task(mission=4, revision=9)))
        self.assertEqual(len(o.published), before, '旧 mission 晚到仍必须拒')
        last = json.loads(o.status_pub.publish.call_args[0][0].data)
        self.assertEqual(last.get('reason'), 'stale_mission')


if __name__ == '__main__':
    unittest.main()
