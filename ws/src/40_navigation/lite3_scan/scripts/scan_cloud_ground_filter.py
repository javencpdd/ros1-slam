#!/usr/bin/env python3
"""scan_cloud_ground_filter.py — 壳侧地面/自车点过滤（上游 SCAN 零改动）。

仅用于仿真斜坡：SCAN 的 2.5D 栅格会把机器人下方/前方的坡面点记成占据，
A* 起点不可用（2026-11-12 实测 a star error 11410 条）。本中继把
`/LIO/clouds_lidar`（机体系）里 z < min_z 的地面点与机体圆柱内(r < self_radius)
的点滤掉，再发 `~dst_topic`；70_scan.launch 的 cloud_topic 指向过滤后话题。
阈值按机体系给：平地/坡面均可；障碍（0.8m 箱、墙）在阈值上方仍保留。

只做点云过滤；不写任何上游话题，不替代感知，也不改变 SCAN 算法。
"""
import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def main():
    rospy.init_node('scan_cloud_ground_filter')
    src = rospy.get_param('~src_topic', '/LIO/clouds_lidar')
    dst = rospy.get_param('~dst_topic', '/LIO/clouds_lidar_ground_filtered')
    min_z = float(rospy.get_param('~min_z', -0.12))
    self_radius = float(rospy.get_param('~self_radius', 0.35))
    pub = rospy.Publisher(dst, PointCloud2, queue_size=2)

    def callback(message):
        points = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
        if points.size:
            keep = (points[:, 2] >= min_z) & (np.hypot(points[:, 0], points[:, 1]) >= self_radius)
            points = points[keep]
        pub.publish(point_cloud2.create_cloud_xyz32(message.header, points.tolist()))

    rospy.Subscriber(src, PointCloud2, callback, queue_size=2, buff_size=2 ** 24)
    rospy.loginfo('ground filter up: %s -> %s (min_z=%.2f self_r=%.2f)',
                  src, dst, min_z, self_radius)
    rospy.spin()


if __name__ == '__main__':
    main()
