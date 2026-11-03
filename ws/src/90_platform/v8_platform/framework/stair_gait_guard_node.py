#!/usr/bin/env python3
"""Gait decision node: stair look-ahead -> /v8/chassis/gait.

Pure decision logic lives in GaitDecision (unit-tested); this wrapper only moves
ROS messages. It never publishes a velocity: the chassis bridge is the only
consumer of the gait topic and the safety mux stays the only /cmd_vel writer.
"""
import json

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

import stair_ahead


from gait_decision import GaitDecision


class GaitGuardNode(object):
    def __init__(self):
        self.enable = bool(rospy.get_param('~enable', True))
        self.enter_m = float(rospy.get_param('~enter_m', 1.0))
        self.exit_m = float(rospy.get_param('~exit_m', 1.5))
        self.min_step_m = float(rospy.get_param('~min_step_m', 0.05))
        self.max_step_m = float(rospy.get_param('~max_step_m', 0.15))
        self.lookahead_m = float(rospy.get_param('~lookahead_m', 1.5))
        self.rate_hz = float(rospy.get_param('~rate_hz', 5.0))
        self.map_topic = rospy.get_param('~map_topic', '/v8_support_map')
        self.path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.gait_topic = rospy.get_param('~gait_topic', '/v8/chassis/gait')
        self.status_topic = rospy.get_param('~status_topic', '/v8/gait_guard/status')

        self.decision = GaitDecision(self.enter_m, self.exit_m, self.min_step_m,
                                     self.max_step_m,
                                     rospy.get_param('~flat_gait', 'flat_mid'),
                                     rospy.get_param('~step_gait', 'high_step'))
        self.map_value = None
        self.path = None
        self.pose = None
        self.published = None

        self.gait_pub = rospy.Publisher(self.gait_topic, String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.map_topic, String, self.on_map, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=1)

    def on_map(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        self.map_value = value.get('map', value)

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                     for p in msg.poses]

    def on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def step(self):
        if not self.enable or self.map_value is None or self.pose is None:
            return
        result = stair_ahead.stair_ahead(self.map_value, self.pose, self.path,
                                         lookahead_m=self.lookahead_m,
                                         min_step_m=self.min_step_m)
        gait = self.decision.update(result)
        if gait != self.published:
            self.published = gait
            self.gait_pub.publish(String(data=gait))
        self.status_pub.publish(String(data=json.dumps({
            'gait': gait,
            'climbing': self.decision.climbing,
            'refusal': self.decision.refusal,
            'ahead': result.get('ahead', False),
            'step_m': result.get('step_m'),
            'distance_m': result.get('distance_m')})))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('v8_gait_guard')
    GaitGuardNode().spin()
