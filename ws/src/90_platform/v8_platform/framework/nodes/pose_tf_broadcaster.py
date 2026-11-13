#!/usr/bin/env python3
"""Broadcast a display-only TF from the 3D local pose to the robot URDF root."""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


def main():
    rospy.init_node('pose_tf_broadcaster')
    parent = rospy.get_param('~parent_frame', 'map')
    child = rospy.get_param('~child_frame', 'TORSO')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    z_offset = float(rospy.get_param('~z_offset', 0.18))
    broadcaster = tf2_ros.TransformBroadcaster()

    def on_pose(message):
        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = message.pose.position.x
        transform.transform.translation.y = message.pose.position.y
        transform.transform.translation.z = message.pose.position.z + z_offset
        transform.transform.rotation = message.pose.orientation
        broadcaster.sendTransform(transform)

    rospy.Subscriber(pose_topic, PoseStamped, on_pose, queue_size=20)
    rospy.loginfo('pose_tf_broadcaster ready: %s -> %s (%s)',
                  parent, child, pose_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
