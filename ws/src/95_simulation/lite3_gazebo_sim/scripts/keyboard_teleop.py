#!/usr/bin/env python3
"""Minimal keyboard teleop for the Gazebo Lite3 planar drive.

Run in its own terminal (the launch itself must not read stdin):
  rosrun lite3_gazebo_sim keyboard_teleop.py

Keys: w/s forward/back, a/d yaw, q/e strafe, space/k stop, Ctrl+C quit.
"""
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

KEYS = {
    'w': (0.30, 0.0, 0.0),
    's': (-0.20, 0.0, 0.0),
    'a': (0.0, 0.0, 0.60),
    'd': (0.0, 0.0, -0.60),
    'q': (0.0, 0.20, 0.0),
    'e': (0.0, -0.20, 0.0),
    ' ': (0.0, 0.0, 0.0),
    'k': (0.0, 0.0, 0.0),
}


def main():
    rospy.init_node('keyboard_teleop')
    topic = rospy.get_param('~topic', '/cmd_vel')
    publisher = rospy.Publisher(topic, Twist, queue_size=1)
    command = Twist()

    def stop():
        publisher.publish(Twist())

    rospy.on_shutdown(stop)
    print('keyboard teleop -> %s' % topic)
    print('w/s forward/back  a/d yaw  q/e strafe  space stop  Ctrl+C quit')
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    rate = rospy.Rate(10)
    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                key = sys.stdin.read(1)
                if key in KEYS:
                    command.linear.x, command.linear.y, command.angular.z = KEYS[key]
            publisher.publish(command)
            rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop()


if __name__ == '__main__':
    main()
