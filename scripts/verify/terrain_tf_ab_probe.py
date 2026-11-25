#!/usr/bin/env python3
"""同一帧点云的 A/B：TF 直接变换 vs far_node 产出的 /terrain_cloud，比较 map 系 z 分布。

目的：`/scan_cloud`（雷达系）中位数 z=0.00、88.7% 落在 ±0.1；但 `/terrain_cloud`（map 系）
中位数 1.32、z<0.1 占 0%。要么变换把点抬了，要么地形编码改写了 z。本探针把两者放在**同一时刻**
对同一份点云比较，直接指出是哪一步抬高的。
"""
import argparse
import math

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def apply_tf(pts, t, q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    R = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])
    return pts @ R.T + np.array(t)


def read_xyz(msg):
    return np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                                 skip_nans=True)], dtype=np.float64)


def stats(z, label):
    print('%-22s n=%5d  z1%%=%6.2f med=%6.2f p99=%6.2f  z<0.1=%5.1f%%  z<0.3=%5.1f%%'
          % (label, len(z), np.percentile(z, 1), np.median(z), np.percentile(z, 99),
             100 * (z < 0.1).mean(), 100 * (z < 0.3).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default='map')
    ap.add_argument('--source', default='rslidar')
    args = ap.parse_args()

    rospy.init_node('terrain_ab_probe', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    raw = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=15)
    terrain = rospy.wait_for_message('/terrain_cloud', PointCloud2, timeout=15)

    pts = read_xyz(raw)
    stats(pts[:, 2], 'raw(%s)' % raw.header.frame_id)

    tr = buf.lookup_transform(args.target, args.source, rospy.Time(0), rospy.Duration(2.0))
    t = (tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z)
    q = (tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w)
    mapped = apply_tf(pts, t, q)
    stats(mapped[:, 2], 'TF->%s (直接变换)' % args.target)
    print('  TF: t=(%.2f,%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)' % (t + q))

    tp = read_xyz(terrain)
    stats(tp[:, 2], 'terrain_cloud(%s)' % terrain.header.frame_id)


if __name__ == '__main__':
    main()
