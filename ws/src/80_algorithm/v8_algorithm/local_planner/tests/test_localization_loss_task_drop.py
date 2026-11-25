#!/usr/bin/env python3
"""定位失效 ⇒ 任务作废：三连自测（真人裁决 2026-11-26 的语义）。

语义：定位失效后当前任务作废，**定位恢复不等于运动授权恢复**；
恢复后必须收到代次更高的合法新任务才能重新武装。

三连：
  ① 主测：位姿持续失效超过阈值 ⇒ goal 清空、停止锁存、代次 +1；
  ② 反控（防止主测是空控）：失效时长**不足**阈值 ⇒ 不得作废（否则正常连续步进会被毁）；
  ③ 任务身份：作废后同代次 goal 必须被拒，代次更高的才接受。
"""
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped

from ros_node import accept_goal, drop_task_on_pose_invalid


def _goal(seq=1, x=1.0, y=0.0, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.seq = seq
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


def _shared(**over):
    base = {'goal': None, 'result': None, 'generation': 0, 'stop_latched': False,
            'stop_reason': None, 'last_error': 'ok', 'cancel_requested': False,
            'cancel_seq': 0, 'cancel_reason': None, 'require_goal_seq': True,
            'last_goal_seq': 0, 'result_generation': -1, 'pose_error': None,
            'pose_invalid_since': None}
    base.update(over)
    return base


class DropTaskOnPoseInvalid(unittest.TestCase):
    def test_persistent_loss_drops_task(self):
        now = time.monotonic()
        sh = _shared(goal=(1.0, 0.0, 0.0), pose_error='E_LOCAL_NOT_READY',
                     pose_invalid_since=now - 2.0)
        self.assertTrue(drop_task_on_pose_invalid(sh, now, 0.8))
        self.assertIsNone(sh['goal'])
        self.assertTrue(sh['stop_latched'])
        self.assertEqual(sh['stop_reason'], 'E_LOCAL_LOCALIZATION_LOST')
        self.assertEqual(sh['generation'], 1)

    def test_brief_loss_does_not_drop(self):
        """反控：短暂失效（<阈值）不得作废，否则正常步进会被打断。"""
        now = time.monotonic()
        sh = _shared(goal=(1.0, 0.0, 0.0), pose_error='E_LOCAL_POSE_STALE',
                     pose_invalid_since=now - 0.2)
        self.assertFalse(drop_task_on_pose_invalid(sh, now, 0.8))
        self.assertEqual(sh['goal'], (1.0, 0.0, 0.0))
        self.assertFalse(sh['stop_latched'])

    def test_old_task_rejected_new_task_arms(self):
        """作废后：同代次 goal 被拒（哪怕时间戳新），代次更高的才接受。"""
        now = time.monotonic()
        sh = _shared(goal=(1.0, 0.0, 0.0), pose_error='E_LOCAL_NOT_READY',
                     pose_invalid_since=now - 2.0)
        self.assertTrue(accept_goal(sh, _goal(seq=3)))
        self.assertTrue(drop_task_on_pose_invalid(sh, now, 0.8))
        # 旧任务用同一代次重发（时间戳更新也无效：身份看 seq）
        self.assertFalse(accept_goal(sh, _goal(seq=3)))
        self.assertEqual(sh['last_error'], 'E_LOCAL_STALE_GOAL')
        # 代次更高的新任务才重新武装
        self.assertTrue(accept_goal(sh, _goal(seq=4)))
        self.assertFalse(sh['stop_latched'])
        self.assertIsNotNone(sh['goal'])


if __name__ == '__main__':
    rospy.init_node('test_drop_task', anonymous=True)
    unittest.main()
