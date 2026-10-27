#!/usr/bin/env python
"""把 /rslidar_points 重发为 base_link 系的 /scan_points。

R770 实测：模拟雷达点数据已是水平/世界对齐（URDF 的 15° 俯仰不体现在点里），
而 URDF 的 rslidar 帧带 15° 俯仰且无 TF 发布者；直接给 costmap 用会按 15° 变换。
故只改 frame_id、不动坐标，并由 costmap 走 base_link→odom→map 的既有 TF。
"""
import rospy
from sensor_msgs.msg import PointCloud2


def main():
    rospy.init_node('scan_relay')
    pub = rospy.Publisher('/scan_points', PointCloud2, queue_size=2)

    def cb(msg):
        msg.header.frame_id = 'base_link'
        pub.publish(msg)

    rospy.Subscriber('/rslidar_points', PointCloud2, cb, queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    main()
