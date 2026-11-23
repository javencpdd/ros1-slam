#!/usr/bin/env python3
"""去掉点云中的 (0,0,0) 填充点，再转发给下游（仿真适配器，不改算法）。

背景（2026-11-24）：livox 仿真输出定长 20000 点，未命中的射线以 (0,0,0) 补齐，
实测 `NaN=0 / 全零=72.8%`。hdl_localization 的 NDT 目标/源里塞进上万条原点会退化，
表现在 Gazebo 里就是 nodelet manager SIGSEGV（与 M2-R2C 记录的现象一致）。

本节点只做「按点拷贝 + 丢弃全零/非有限点」，保持原始字段布局，不改任何算法参数。
用法：python3 cloud_denull.py [--in /rslidar_points] [--out /velodyne_points]
"""
import argparse
import math
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField


def _xyzi_offsets(msg):
    """返回 (x_off, y_off, z_off)，按 fields 名字取；缺失则视为不支持的布局。"""
    by_name = {f.name: f.offset for f in msg.fields}
    if not all(k in by_name for k in ('x', 'y', 'z')):
        return None
    return by_name['x'], by_name['y'], by_name['z']


def filter_cloud(msg):
    offs = _xyzi_offsets(msg)
    if offs is None:
        return None
    xo, yo, zo = offs
    step = msg.point_step
    total = msg.width * msg.height
    out = bytearray()
    kept = 0
    raw = msg.data
    for i in range(total):
        base = i * step
        try:
            x, y, z = struct.unpack_from('<fff', raw, base + xo)[0], \
                struct.unpack_from('<fff', raw, base + yo)[0], \
                struct.unpack_from('<fff', raw, base + zo)[0]
        except struct.error:
            break
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        out += raw[base:base + step]
        kept += 1
    if kept == 0:
        return None
    out_msg = PointCloud2()
    out_msg.header = msg.header
    out_msg.height = 1
    out_msg.width = kept
    out_msg.fields = list(msg.fields)
    out_msg.is_bigendian = msg.is_bigendian
    out_msg.point_step = step
    out_msg.row_step = step * kept
    out_msg.is_dense = True
    out_msg.data = bytes(out)
    return out_msg, kept, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='in_topic', default='/rslidar_points')
    ap.add_argument('--out', dest='out_topic', default='/velodyne_points')
    ap.add_argument('--rate-log', type=float, default=10.0)
    args, _ = ap.parse_known_args()

    rospy.init_node('cloud_denull', anonymous=True)
    pub = rospy.Publisher(args.out_topic, PointCloud2, queue_size=2)
    stat = {'in': 0, 'out': 0, 'kept': 0, 'total': 0}

    def on_cloud(msg):
        res = filter_cloud(msg)
        stat['in'] += 1
        if res is None:
            rospy.logwarn_throttle(5.0, '[cloud_denull] 无有效点或被丢弃')
            return
        out_msg, kept, total = res
        pub.publish(out_msg)
        stat['out'] += 1
        stat['kept'], stat['total'] = kept, total
        rospy.loginfo_throttle(args.rate_log * 6, '[cloud_denull] kept %d/%d (%.1f%%)',
                                kept, total, 100.0 * kept / max(1, total))

    rospy.Subscriber(args.in_topic, PointCloud2, on_cloud, queue_size=2)
    rospy.loginfo('[cloud_denull] %s -> %s（丢弃全零/非有限点）', args.in_topic, args.out_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
