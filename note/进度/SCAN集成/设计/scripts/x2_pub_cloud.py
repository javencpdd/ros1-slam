#!/usr/bin/env python3
# x2_pub_cloud.py —— 格③ 用的**合成点云发布器**（R332 建；＃166 工单格③ 甲支＝纯合成点云、不接真 sim）
# 被测面＝现役 stvl 的参数块（在册 costmap_common.yaml 的 stvl_obstacle_layer 逐字拷）：observation_sources=rslidar_points、
#   topic=/rslidar_points、sensor_frame=rslidar、data_type=PointCloud2、min/max_obstacle_height=0.15/1.5、min_z/max_z=-0.30/3.0、
#   filter=voxel、voxel_min_points=3 ⇒ 本件每体素投 **6** 点（>3）保证能过体素过滤。
# ★本件只测"stvl 进不进层／costmap 是否还发得出来"，★不据它判感知性能（那是真 sim 那一支＝勾复① 走的是合成支）。
import os
import math
import struct
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import MultiArrayDimension

FRAME = "rslidar"
TOPIC = "/rslidar_points"
TAG = os.environ.get("X2_CLOUD_TAG", "g6")


def cloud(pts):
    m = PointCloud2()
    m.header.frame_id = FRAME
    m.height = 1
    m.width = len(pts)
    m.fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
    ]
    m.is_dense = True
    buf = b"".join(struct.pack("<fff", x, y, z) for x, y, z in pts)
    m.row_step = len(buf)
    m.point_step = 12
    m.data = buf
    return m


def build():
    pts = []
    for i in range(6):
        for j in range(6):
            pts.append((1.00 + 0.02 * i, -0.05 + 0.02 * j, 0.50))
    for k in range(6):
        pts.append((2.50, -0.40 + 0.16 * k, 0.50))
    return pts


def main():
    rospy.init_node("x2_cloud_%s" % TAG)
    pub = rospy.Publisher(TOPIC, PointCloud2, queue_size=2)
    rospy.sleep(0.5)
    pts = build()
    r = rospy.Rate(2)
    n = 0
    while not rospy.is_shutdown():
        m = cloud(pts)
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        n += 1
        if n == 1:
            rospy.loginfo("X2_CLOUD first sent pts=%d frame=%s topic=%s tag=%s", len(pts), FRAME, TOPIC, TAG)
        r.sleep()


if __name__ == "__main__":
    main()
