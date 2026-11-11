#!/usr/bin/env python3
"""Publish /v8/local/pose from Gazebo ground truth.

/v8/local/pose is the contract topic every consumer subscribes to (console BT,
local planner), but the sim stack never had a producer for it — earlier runs
bridged it inside a throw-away test script.  This node is the formal bridge:

    /sim/odom_gt  (lite3_planar_drive WorldPose, nav_msgs/Odometry)
        -> /v8/local/pose  (geometry_msgs/PoseStamped, frame = map)

It republishes at a fixed rate so downstream nodes keep a steady stream even if
the drive plugin's odometry rate changes; the stamp is taken from the source
message so consumers can still age-check it.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SimPoseSource(object):
    def __init__(self):
        self.src_topic = rospy.get_param('~src_topic', '/sim/odom_gt')
        self.dst_topic = rospy.get_param('~dst_topic', '/v8/local/pose')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        rate = float(rospy.get_param('~rate', 50.0))
        self.latest = None
        self.pub = rospy.Publisher(self.dst_topic, PoseStamped, queue_size=2)
        rospy.Subscriber(self.src_topic, Odometry, self.on_odom, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo('[sim_pose_source] %s -> %s @ %.0f Hz frame=%s',
                      self.src_topic, self.dst_topic, rate, self.frame_id)

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


def main():
    rospy.init_node('sim_pose_source')
    SimPoseSource()
    rospy.spin()


if __name__ == '__main__':
    main()
