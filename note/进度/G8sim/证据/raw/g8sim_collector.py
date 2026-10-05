#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# g8sim_collector.py —— G8-sim 返工R1 全程采集（/Odometry + /quad_0/body_pose GT）
# 用法: python3 g8sim_collector.py <输出前缀路径>   → 前缀.odom.csv / 前缀.gt.csv
import math
import sys

import rospy
from nav_msgs.msg import Odometry


def yaw_from(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    prefix = sys.argv[1]
    rospy.init_node('g8sim_collector', disable_signals=True)
    fo = open(prefix + '.odom.csv', 'w')
    fg = open(prefix + '.gt.csv', 'w')
    fo.write('wall,stamp,x,y,z,qx,qy,qz,qw,yaw\n')
    fg.write('wall,stamp,x,y,z,qx,qy,qz,qw,yaw\n')

    def row(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return '%.3f,%.3f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (
            rospy.get_time(), msg.header.stamp.to_sec(), p.x, p.y, p.z,
            q.x, q.y, q.z, q.w, yaw_from(q))

    def cb_odom(msg):
        fo.write(row(msg))
        fo.flush()

    def cb_gt(msg):
        fg.write(row(msg))
        fg.flush()

    rospy.Subscriber('/quad_0/body_pose', Odometry, cb_gt, queue_size=50)
    rospy.Subscriber('/Odometry', Odometry, cb_odom, queue_size=10)
    rospy.loginfo('[collector] up: prefix=%s', prefix)
    rospy.spin()
    fo.close()
    fg.close()


if __name__ == '__main__':
    main()
