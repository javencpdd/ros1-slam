#!/usr/bin/env python3
"""Kinematic 3D carrier inside Gazebo for the A5 local planner.

Applies the map-frame 3D velocity directly to a static Gazebo model with
SetModelState and returns the same pose to the planner. This removes the planar
carrier's vertical rejection by construction; it is a kinematic pose sink, not
Gazebo contact dynamics and not a four-leg climb.
"""
import argparse
import json
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


def advance(position, velocity, dt):
    return tuple(position[i] + velocity[i] * dt for i in range(3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--start', default='0,-2.175,0')
    parser.add_argument('--center-z', type=float, default=0.1)
    parser.add_argument('--cmd-timeout', type=float, default=0.5)
    args = parser.parse_args()
    start = tuple(float(v) for v in args.start.split(','))
    rospy.init_node('gazebo_3d_carrier', anonymous=True, disable_signals=True)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    shared = {'velocity': (0., 0., 0.), 'at': -1e9}

    def on_cmd(message):
        shared['velocity'] = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        shared['at'] = time.monotonic()

    rospy.Subscriber(args.prefix + '/cmd', TwistStamped, on_cmd, queue_size=50)
    pose_pub = rospy.Publisher(args.prefix + '/pose', PoseStamped, queue_size=50)
    body_pub = rospy.Publisher(args.prefix + '/body', PoseStamped, queue_size=50)
    status_pub = rospy.Publisher(args.prefix + '/carrier', String, queue_size=20)

    position = start
    last = time.monotonic()
    tick = 0
    while not rospy.is_shutdown():
        now = time.monotonic()
        velocity = shared['velocity'] if now - shared['at'] <= args.cmd_timeout else (0., 0., 0.)
        position = advance(position, velocity, max(0., now - last))
        last = now
        stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.w = 1.
        pose_pub.publish(pose)
        body = PoseStamped()
        body.header.frame_id, body.header.stamp = 'map', stamp
        body.pose.position.x, body.pose.position.y = position[0], position[1]
        body.pose.position.z = position[2]
        body.pose.orientation.w = 1.
        body_pub.publish(body)
        state = ModelState()
        state.model_name, state.reference_frame = args.model, 'world'
        state.pose.position.x, state.pose.position.y = position[0], position[1]
        state.pose.position.z = position[2] + args.center_z
        state.pose.orientation.w = 1.
        try:
            set_state(state)
        except rospy.ServiceException:
            pass
        if tick % 5 == 0:
            status_pub.publish(String(data=json.dumps({
                'position': position, 'velocity': velocity,
                'model_center_z': position[2] + args.center_z,
                'velocity_age_s': now - shared['at']})))
        tick += 1
        time.sleep(.02)


if __name__ == '__main__':
    main()
