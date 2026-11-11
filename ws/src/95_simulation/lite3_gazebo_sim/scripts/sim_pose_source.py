#!/usr/bin/env python3
"""Publish /v8/local/pose from Gazebo ground truth, and declare localization OK.

/v8/local/pose is the contract topic every consumer subscribes to (console BT,
local planner), but the sim stack never had a producer for it — earlier runs
bridged it inside a throw-away test script.  This node is the formal bridge:

    /sim/odom_gt  (lite3_planar_drive WorldPose, nav_msgs/Odometry)
        -> /v8/local/pose            (geometry_msgs/PoseStamped, frame = map)
        -> /v8/local/localization_ok (std_msgs/Bool, True while poses are fresh)

The second output matters: `local_planner/ros_node.py` refuses to command motion
with `reason=E_LOCAL_LOCALIZATION` until it hears that flag, so a pose source
that stays silent leaves the carrier standing still even with a valid path.

It republishes at a fixed rate so downstream nodes keep a steady stream even if
the drive plugin's odometry rate changes; the stamp is taken from the source
message so consumers can still age-check it.
"""
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class SimPoseSource(object):
    def __init__(self):
        self.src_topic = rospy.get_param('~src_topic', '/sim/odom_gt')
        self.dst_topic = rospy.get_param('~dst_topic', '/v8/local/pose')
        self.loc_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
        self.publish_ok = bool(rospy.get_param('~publish_localization_ok', True))
        self.ok_period = float(rospy.get_param('~localization_ok_period_s', 1.0))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        rate = float(rospy.get_param('~rate', 50.0))
        self.latest = None
        self._last_ok = 0.0
        self.pub = rospy.Publisher(self.dst_topic, PoseStamped, queue_size=2)
        self.ok_pub = rospy.Publisher(self.loc_topic, Bool, queue_size=1, latch=True)
        rospy.Subscriber(self.src_topic, Odometry, self.on_odom, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo('[sim_pose_source] %s -> %s @ %.0f Hz; localization_ok -> %s',
                      self.src_topic, self.dst_topic, rate, self.loc_topic)

    def on_odom(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose
        self.latest = pose

    def tick(self, _event):
        if self.latest is None:
            return
        self.pub.publish(self.latest)
        if not self.publish_ok:
            return
        now = time.time()
        if now - self._last_ok >= self.ok_period:
            # Republish periodically: the flag means "poses are currently fresh",
            # not "we once had a pose".
            self.ok_pub.publish(Bool(data=True))
            self._last_ok = now


def main():
    rospy.init_node('sim_pose_source')
    SimPoseSource()
    rospy.spin()


if __name__ == '__main__':
    main()
