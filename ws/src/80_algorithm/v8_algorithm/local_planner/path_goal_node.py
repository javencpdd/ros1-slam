#!/usr/bin/env python3
"""Publish 3D global Path waypoints to the local planner, one at a time.

为什么不只发终点：local_native 是**步进式**的 —— 收到一个 goal 走 max_step_m（默认 0.30 m）
就置 segment_complete（reason='segment_complete'），之后要等下一个目标才继续。
原实现只把 Path 的 `poses[-1]` 发出去，于是车走完一步 0.30 m 就停住，
BT 的 wait_arrival 一直空等到超时 —— 实测 travel 恰好 0.308 m / 0.307 m（两路一致）。

本节点按路径顺序逐点下发：当前 waypoint 到达（或超时）后推进到下一个。
不重复重发同一点，因为 local 的 accept_goal 每次都把 generation +1，会让在飞结果作废。
"""
import json
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def accepts_path_stamp(cancelled_at, path_stamp):
    """取消之后是否接受这条路径。两个参数都是秒（float 或 None）。

    返回 (accepted, reason)。判定只看时间序，不依赖 ROS 类型，便于离线单测。
    fail-closed：调用方拿不到 stamp 时按拒绝处理 —— 无法区分新旧就不能重放。
    """
    if cancelled_at is None:
        return True, 'no_cancel'
    if not path_stamp:
        return False, 'no_stamp_fail_closed'
    if path_stamp <= cancelled_at:
        return False, 'stale_after_cancel'
    return True, 'newer_than_cancel'


def make_goal(pose, generation=None):
    """把代次写进 header.seq —— 这是**任务身份**，不是本地计数器。

    只靠本地 generation 计数无法防住"旧任务带着更新 timestamp 重新下发"：
    下游看不出两者区别。把代次编码进消息后，接收端可以按身份拒绝更老的代次。
    """
    goal = PoseStamped()
    goal.header = pose.header
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = goal.header.frame_id or 'map'
    if generation is not None:
        goal.header.seq = int(generation)
    goal.pose = pose.pose
    return goal


class PathGoal(object):
    def __init__(self):
        self.path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
        self.goal_topic = rospy.get_param('~goal_topic', '/v8/local/goal')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/path_goal/status')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        # 取消必须贯穿到这一层：本节点是 local 目标的直接下发者，BT 收到 cancel 后
        # 若这里不停手，latched 的全局路径会让它继续逐点下发，local 被重新驱动。
        self.cancel_topic = rospy.get_param('~cancel_topic', '/v8/local/cancel')
        self.arrive_m = float(rospy.get_param('~arrive_m', 0.30))
        self.advance_timeout_s = float(rospy.get_param('~advance_timeout_s', 8.0))
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.poses = []
        self.index = 0
        self.published_at = None
        self.pose = None
        # 取消时刻：此后只接受 stamp 更新的 path，杜绝旧路径重放
        self.cancelled_at = None
        # 任务代次：每接受一条路径 +1（供审计"是否重放"）；被拒的旧路径单独计数
        self.accepted_generation = 0
        self.rejected_stale = 0
        self.last_reject_reason = None

        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber(self.cancel_topic, String, self.on_cancel, queue_size=10)
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def on_pose(self, message):
        self.pose = message.pose.position

    def on_cancel(self, message):
        """立即清空活动路径；此后只有 stamp 更新的 path 才能重新武装。"""
        dropped = len(self.poses)
        self.cancelled_at = rospy.Time.now()
        self.poses = []
        self.index = 0
        self.published_at = None
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'cancelled': True, 'dropped_poses': dropped,
            'reason': message.data[:80]})))

    def on_path(self, message):
        if not message.poses:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'empty_path'})))
            return
        stamp_obj = message.header.stamp
        stamp = stamp_obj.to_sec() if stamp_obj else 0.0
        cancelled = self.cancelled_at.to_sec() if self.cancelled_at is not None else None
        accepted, why = accepts_path_stamp(cancelled, stamp)
        if not accepted:
            self.rejected_stale += 1
            self.last_reject_reason = why
            self.status_pub.publish(String(data=json.dumps({
                'ok': False, 'reason': why, 'path_stamp': round(stamp, 3),
                'cancelled_at': None if cancelled is None else round(cancelled, 3),
                'rejected_stale': self.rejected_stale})))
            return
        if self.cancelled_at is not None:
            self.cancelled_at = None
        self.accepted_generation += 1
        self.poses = list(message.poses)
        self.index = 0
        self.publish_current()

    def remaining(self):
        if not self.poses or self.pose is None:
            return None
        p = self.poses[self.index].pose.position
        return math.hypot(p.x - self.pose.x, p.y - self.pose.y)

    def publish_current(self):
        goal = make_goal(self.poses[self.index], self.accepted_generation)
        self.goal_pub.publish(goal)
        self.published_at = rospy.Time.now()
        rest = self.remaining()
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'generation': self.accepted_generation,
            'rejected_stale': self.rejected_stale,
            'poses': len(self.poses), 'index': self.index + 1,
            'remaining_m': None if rest is None else round(rest, 3),
            'goal': [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3),
                     round(goal.pose.position.z, 3)],
            'frame': goal.header.frame_id})))

    def tick(self, _event):
        if self.cancelled_at is not None:
            return
        if not self.poses or self.index >= len(self.poses) - 1:
            return
        rest = self.remaining()
        stale = (self.published_at is not None and
                 (rospy.Time.now() - self.published_at).to_sec() > self.advance_timeout_s)
        if (rest is not None and rest <= self.arrive_m) or stale:
            self.index += 1
            self.publish_current()


def main():
    rospy.init_node('path_goal3d', anonymous=False, disable_signals=True)
    PathGoal()
    rospy.spin()


if __name__ == '__main__':
    main()
