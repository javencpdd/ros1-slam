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


def make_goal(pose):
    goal = PoseStamped()
    goal.header = pose.header
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = goal.header.frame_id or 'map'
    goal.pose = pose.pose
    return goal


class PathGoal(object):
    def __init__(self):
        self.path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
        self.goal_topic = rospy.get_param('~goal_topic', '/v8/local/goal')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/path_goal/status')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.arrive_m = float(rospy.get_param('~arrive_m', 0.30))
        self.advance_timeout_s = float(rospy.get_param('~advance_timeout_s', 8.0))
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.poses = []
        self.index = 0
        self.published_at = None
        self.pose = None

        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=10)
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def on_pose(self, message):
        self.pose = message.pose.position

    def on_path(self, message):
        if not message.poses:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'empty_path'})))
            return
        self.poses = list(message.poses)
        self.index = 0
        self.publish_current()

    def remaining(self):
        if not self.poses or self.pose is None:
            return None
        p = self.poses[self.index].pose.position
        return math.hypot(p.x - self.pose.x, p.y - self.pose.y)

    def publish_current(self):
        goal = make_goal(self.poses[self.index])
        self.goal_pub.publish(goal)
        self.published_at = rospy.Time.now()
        rest = self.remaining()
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'poses': len(self.poses), 'index': self.index + 1,
            'remaining_m': None if rest is None else round(rest, 3),
            'goal': [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3),
                     round(goal.pose.position.z, 3)],
            'frame': goal.header.frame_id})))

    def tick(self, _event):
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
