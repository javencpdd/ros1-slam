"""mode3 中继的 ROS 侧最小节点：位姿+路点 → Twist，取消/hold 即零速。"""
import json
import math

import rospy
from geometry_msgs.msg import Twist
import follower


class FollowerNode(object):
    def __init__(self):
        self.path = [tuple(p) for p in rospy.get_param('~path')]
        self.pose_topic = rospy.get_param('~pose_topic', '/test/pose')
        self.out_topic = rospy.get_param('~out_topic', '/follow/cmd_vel')
        self.max_v = rospy.get_param('~max_v', 0.25)
        self.max_w = rospy.get_param('~max_w', 0.8)
        self.index = 0
        self.stopped = False
        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=1)
        from geometry_msgs.msg import PoseStamped
        self.pose = None
        self._sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._tick)

    def _pose_cb(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def stop(self):
        self.stopped = True
        self.pub.publish(Twist())

    def _tick(self, _event):
        if self.stopped or self.pose is None:
            return
        if self.index >= len(self.path):
            self.stop()
            rospy.loginfo('[follower] path complete')
            return
        v, w, arrived = follower.twist_for(self.pose, self.path[self.index],
                                           max_v=self.max_v, max_w=self.max_w)
        if arrived:
            self.index += 1
            if self.index >= len(self.path):
                self.stop()
                rospy.loginfo('[follower] arrived (%d/%d)', self.index, len(self.path))
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)
