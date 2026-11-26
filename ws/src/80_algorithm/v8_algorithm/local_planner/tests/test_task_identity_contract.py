#!/usr/bin/env python3
"""任务身份契约的离线测试（真人裁决 2026-11-26 第 2/3/4/6 条）。

背景（实测）：ROS1 的 publisher 会把 `header.seq` 改写成"该 publisher 的消息计数"
（rospy 发 seq=100/101/102，订阅端收到 1/2/3）⇒ **header.seq 绝不能承载任务身份**。
因此身份走独立 meta：`(mission_id, path_revision, waypoint_index, execution_generation)`。

本文件覆盖：
  ① 同任务多航点推进（waypoint 0→1→2 连续接受）
  ② 滚动重规划（path_revision 前进接受）
  ③ 重复消息不得重置规划（同四元组 → 忽略，generation 与 stop 锁存都不动）
  ④ 旧任务用**新时间戳**重发 → 仍拒绝（身份与时间无关）
  ⑤ 故障撤销后，旧 mission 的任何版本都不得复活
  ⑥ 异步结果晚到 → 代次不符即丢弃
"""
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# 本文件**不依赖 ROS 运行时**：用 SimpleNamespace 造消息，避免污染 sys.modules 影响同目录其它测试。
for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
              'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(_name, MagicMock())

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from ros_node import (accept_goal, accept_task_json,  # noqa: E402
                      accept_worker_result)


def _goal(x=1.0, y=0.0, frame='map', stamp_sec=0.0):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame, stamp=MagicMock(secs=stamp_sec), seq=0),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0)))


def _meta(mission=1, rev=0, wp=0, gen=0):
    return {'mission_id': mission, 'path_revision': rev,
            'waypoint_index': wp, 'execution_generation': gen}


def _shared():
    return {'goal': None, 'result': None, 'generation': 0, 'stop_latched': False,
            'stop_reason': None, 'last_error': 'ok', 'cancel_requested': False,
            'cancel_seq': 0, 'cancel_reason': None, 'require_goal_seq': True,
            'last_goal_seq': 0, 'result_generation': -1, 'pose_error': None,
            'pose_invalid_since': None, 'mission_id': 0, 'path_revision': 0,
            'waypoint_index': 0, 'exec_gen': 0, 'revoked_mission': -1}


class TaskIdentity(unittest.TestCase):
    def test_same_mission_waypoints_advance(self):
        sh = _shared()
        self.assertTrue(accept_goal(sh, _goal(1.0), _meta(mission=1, wp=0)))
        g0 = sh['generation']
        self.assertTrue(accept_goal(sh, _goal(2.0), _meta(mission=1, wp=1)))
        self.assertTrue(accept_goal(sh, _goal(3.0), _meta(mission=1, wp=2)))
        self.assertEqual(sh['goal'], (3.0, 0.0, 0.0))
        self.assertGreater(sh['generation'], g0)

    def test_rolling_replan_same_waypoint_new_revision(self):
        sh = _shared()
        self.assertTrue(accept_goal(sh, _goal(1.0), _meta(mission=1, rev=0, wp=0)))
        self.assertTrue(accept_goal(sh, _goal(1.2), _meta(mission=1, rev=1, wp=0)))
        self.assertEqual(sh['goal'], (1.2, 0.0, 0.0))
        self.assertEqual(sh['path_revision'], 1)

    def test_duplicate_does_not_reset(self):
        """③ 重复消息：不接受、且**不得**改动 generation / stop 锁存。"""
        sh = _shared()
        self.assertTrue(accept_goal(sh, _goal(1.0), _meta(mission=1, rev=0, wp=0)))
        gen_after_first = sh['generation']
        self.assertFalse(accept_goal(sh, _goal(1.0), _meta(mission=1, rev=0, wp=0)))
        self.assertEqual(sh['generation'], gen_after_first)   # 没有被重置/推进
        self.assertEqual(sh['goal'], (1.0, 0.0, 0.0))

    def test_old_mission_with_new_stamp_rejected(self):
        """④ 旧任务刷新时间戳：身份看 mission_id，不看时间。"""
        sh = _shared()
        self.assertTrue(accept_goal(sh, _goal(1.0), _meta(mission=5, wp=0)))
        # mission 4 带着"更新的"时间戳回来
        self.assertFalse(accept_goal(sh, _goal(9.0, stamp_sec=1e9), _meta(mission=4, wp=0)))
        self.assertEqual(sh['last_error'], 'E_LOCAL_STALE_MISSION')
        self.assertEqual(sh['goal'], (1.0, 0.0, 0.0))

    def test_revoked_mission_never_revives(self):
        """⑤ 撤销后任何版本都拒绝（含新时间戳、更高 waypoint/revision）。"""
        sh = _shared()
        self.assertTrue(accept_goal(sh, _goal(1.0), _meta(mission=3, wp=0)))
        sh['revoked_mission'] = 3          # 故障撤销
        self.assertFalse(accept_goal(sh, _goal(5.0, stamp_sec=1e9),
                                     _meta(mission=3, rev=9, wp=9)))
        self.assertEqual(sh['last_error'], 'E_LOCAL_TASK_REVOKED')
        # 新任务（mission 更大）可以重新武装
        self.assertTrue(accept_goal(sh, _goal(6.0), _meta(mission=4, wp=0)))

    def test_atomic_task_missing_identity_rejected(self):
        """★原子消息缺身份必须拒★（禁止回退 header.seq）。"""
        sh = _shared()
        self.assertFalse(accept_task_json(sh, {'x': 1.0, 'y': 0.0}))
        self.assertEqual(sh['last_error'], 'E_LOCAL_NO_TASK_IDENTITY')
        self.assertIsNone(sh['goal'])

    def test_atomic_task_malformed_rejected(self):
        sh = _shared()
        self.assertFalse(accept_task_json(sh, {'mission_id': 1, 'x': 'oops'}))
        self.assertEqual(sh['last_error'], 'E_LOCAL_TASK_MALFORMED')

    def test_atomic_task_out_of_order_rejected(self):
        """乱序：older mission 晚到必须拒（不因后到而覆盖当前任务）。"""
        sh = _shared()
        self.assertTrue(accept_task_json(sh, {'mission_id': 7, 'path_revision': 0,
                                              'waypoint_index': 0, 'execution_generation': 0,
                                              'x': 1.0, 'y': 0.0}))
        self.assertFalse(accept_task_json(sh, {'mission_id': 6, 'path_revision': 9,
                                               'waypoint_index': 9, 'execution_generation': 0,
                                               'x': 9.0, 'y': 9.0}))
        self.assertEqual(sh['last_error'], 'E_LOCAL_STALE_MISSION')
        self.assertEqual(sh['goal'], (1.0, 0.0, 0.0))

    def test_atomic_task_revoked_rejected(self):
        sh = _shared()
        sh['revoked_mission'] = 5
        self.assertFalse(accept_task_json(sh, {'mission_id': 5, 'x': 2.0, 'y': 2.0}))
        self.assertEqual(sh['last_error'], 'E_LOCAL_TASK_REVOKED')

    def test_late_async_result_dropped(self):
        """⑥ 异步结果晚到：代次不符即丢弃，不得覆盖当前结果。"""
        sh = _shared()
        accept_goal(sh, _goal(1.0), _meta(mission=1, wp=0))
        good = {'generation': sh['generation'], 'ok': True,
                'result': {'valid_until': 1e9}, 'plan_ms': 1.0}
        self.assertTrue(accept_worker_result(sh, good, 1.0))
        stale = dict(good, generation=sh['generation'] - 1)
        self.assertFalse(accept_worker_result(sh, stale, 2.0))


if __name__ == '__main__':
    unittest.main()
