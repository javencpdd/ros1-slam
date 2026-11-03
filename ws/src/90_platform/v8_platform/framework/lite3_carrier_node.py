#!/usr/bin/env python3
"""Lite3 pose carrier: map->base_link (TF) or an odometry topic -> the 3D app pose contract.

This node never publishes a command topic. On the Lite3 the only /cmd_vel writer is
safe_cmd_vel_mux and the chassis bridge is ros2qnx, which is allowed only inside the
real-robot navigation mode. The carrier therefore supplies pose + localization health
and nothing else.
"""
import json
import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _yaw(orientation):
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose_from_odom(message, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
    pose.pose = message.pose.pose
    return pose


def _pose_from_transform(transform, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = transform.header.stamp if not transform.header.stamp.is_zero() else rospy.Time.now()
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


class Lite3Carrier(object):
    def __init__(self):
        self.pose_source = rospy.get_param('~pose_source', 'tf')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.localization_topic = rospy.get_param('~localization_topic',
                                                  '/v8/local/localization_ok')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/carrier/status')
        self.input_pose_topic = rospy.get_param('~input_pose_topic', '/odom')
        self.rate_hz = float(rospy.get_param('~rate_hz', 20.0))
        self.stale_s = float(rospy.get_param('~stale_s', 0.5))

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=20)
        self.loc_pub = rospy.Publisher(self.localization_topic, Bool, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5)

        self.latest = None
        self.latest_wall = None
        if self.pose_source == 'tf':
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer)
        elif self.pose_source == 'topic':
            rospy.Subscriber(self.input_pose_topic, Odometry, self.on_odom, queue_size=20)
        else:
            raise SystemExit('unknown ~pose_source: %s' % self.pose_source)

    def on_odom(self, message):
        self.latest = _pose_from_odom(message, self.map_frame)
        self.latest_wall = rospy.get_time()

    def step(self):
        if self.pose_source == 'tf':
            try:
                transform = self.buffer.lookup_transform(
                    self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                transform = None
            if transform is not None:
                self.latest = _pose_from_transform(transform, self.map_frame)
                self.latest_wall = rospy.get_time()
        if self.latest is None:
            self.loc_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data=json.dumps({
                'source': self.pose_source, 'ok': False, 'reason': 'no_pose_yet'})))
            return
        age = rospy.get_time() - self.latest_wall
        healthy = age <= self.stale_s
        self.pose_pub.publish(self.latest)
        self.loc_pub.publish(Bool(data=healthy))
        self.status_pub.publish(String(data=json.dumps({
            'source': self.pose_source, 'ok': healthy, 'age_s': round(age, 3),
            'frame': self.map_frame, 'base_frame': self.base_frame})))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('v8_lite3_carrier')
    Lite3Carrier().spin()
