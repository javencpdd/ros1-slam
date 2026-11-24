#!/usr/bin/env python3
"""量化 dynamic_obstacles 的 AABB 过度包围：用真实点云算每个框的空洞率与膨胀增益。

判据定义（都只用点云本身，不引入外部真值）：
  投影面积   = (max_x-min_x)*(max_y-min_y)
  占用面积   = 框内非空 xy 栅格数 * voxel^2      ← 传感器实际看到的占据
  空洞率     = 1 - 占用面积/投影面积             ← 过度包围的直接度量
  膨胀增量   = 膨胀后面积/投影面积                ← 碰撞判定白付出去的空间

膨胀距离与线上一致：d = radius_m + clearance_m（local_native.Config 的两个字段）。

用法：
  obstacle_aabb_audit.py --topic /rslidar_points --out audit.tsv
  obstacle_aabb_audit.py --npy frame.npy --out audit.tsv        # 离线复算
"""
import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FRAMEWORK = os.path.join(REPO, 'ws/src/90_platform/v8_platform/framework')
sys.path.insert(0, FRAMEWORK)
import dynamic_obstacles as do  # noqa: E402


def cloud_from_ros(topic, timeout):
    import rospy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2
    rospy.init_node('aabb_audit', anonymous=True)
    msg = rospy.wait_for_message(topic, PointCloud2, timeout=timeout)
    rows = [[p[0], p[1], p[2]] for p in pc2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)]
    return np.array(rows, dtype=float) if rows else np.empty((0, 3))


def pose_from_ros(topic, timeout):
    import rospy
    from nav_msgs.msg import Odometry
    msg = rospy.wait_for_message(topic, Odometry, timeout=timeout)
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.x, p.y, p.z, yaw)


def audit(points, center, params):
    """返回 (rows, summary)。rows 每项一个框。"""
    roi = do.roi_filter(points, center[:2], params['floor_z'],
                        params['roi_forward_m'], params['roi_lateral_m'],
                        params['min_height_m'], params['max_height_m'])
    roi = do.self_filter(roi, center[:2], params['self_radius_m'])
    if not len(roi):
        return [], {'boxes': 0, 'reason': 'empty_after_roi'}
    centers, keys = do.voxel_downsample(roi, params['voxel_m'])
    boxes = do.cluster_boxes(centers, keys, params['min_points'], params['max_objects'],
                             min_fill=params['min_fill'], min_extent_m=params['min_extent_m'],
                             max_depth=params['max_depth'], voxel_m=params['voxel_m'])

    voxel = params['voxel_m']
    inflate = params['radius_m'] + params['clearance_m']
    rows = []
    for index, box in enumerate(boxes):
        lo, hi = box['min'], box['max']
        w, d = hi[0] - lo[0], hi[1] - lo[1]
        area = max(w * d, 1e-9)
        inside = centers[(centers[:, 0] >= lo[0]) & (centers[:, 0] <= hi[0])
                         & (centers[:, 1] >= lo[1]) & (centers[:, 1] <= hi[1])]
        cells = len({(int(math.floor(p[0] / voxel)), int(math.floor(p[1] / voxel)))
                     for p in inside})
        occl = cells * voxel * voxel
        inflated = (w + 2 * inflate) * (d + 2 * inflate)
        rows.append({
            'box': index,
            'min': [round(v, 2) for v in lo],
            'max': [round(v, 2) for v in hi],
            'size': [round(w, 2), round(d, 2), round(hi[2] - lo[2], 2)],
            'points': box['points'],
            'area_m2': round(area, 3),
            'occupied_m2': round(occl, 3),
            'hollowness': round(max(0.0, 1.0 - occl / area), 3),
            'inflated_area_m2': round(inflated, 3),
            'inflate_gain': round(inflated / area, 2),
        })

    # ★覆盖性断言★：分裂只允许收紧，不允许漏点。任何点必须仍落在某个框里，
    # 否则等于把障碍挖掉一块 —— 那是降低碰撞保护，不是优化。
    uncovered = 0
    if len(centers):
        covered = np.zeros(len(centers), dtype=bool)
        for box in boxes:
            lo, hi = box['min'], box['max']
            covered |= ((centers[:, 0] >= lo[0]) & (centers[:, 0] <= hi[0])
                        & (centers[:, 1] >= lo[1]) & (centers[:, 1] <= hi[1])
                        & (centers[:, 2] >= lo[2]) & (centers[:, 2] <= hi[2]))
        uncovered = int((~covered).sum())

    total_area = sum(r['area_m2'] for r in rows)
    total_occl = sum(r['occupied_m2'] for r in rows)
    total_inf = sum(r['inflated_area_m2'] for r in rows)
    worst = max(rows, key=lambda r: r['hollowness']) if rows else None
    summary = {
        'boxes': len(rows),
        'roi_points': int(len(roi)),
        'total_area_m2': round(total_area, 3),
        'total_occupied_m2': round(total_occl, 3),
        'total_hollowness': round(max(0.0, 1.0 - total_occl / total_area), 3) if total_area else None,
        'total_inflated_area_m2': round(total_inf, 3),
        'inflate_gain': round(total_inf / total_area, 2) if total_area else None,
        'worst_box': None if worst is None else {
            'box': worst['box'], 'hollowness': worst['hollowness'], 'size': worst['size']},
        'inflate_m': round(inflate, 3),
        'uncovered_points': uncovered,
        'covered_all': uncovered == 0,
    }
    return rows, summary


DEFAULTS = {
    'floor_z': 0.0, 'roi_forward_m': 6.0, 'roi_lateral_m': 3.0,
    'min_height_m': 0.15, 'max_height_m': 1.6, 'voxel_m': 0.10,
    'min_points': 8, 'max_objects': 32, 'self_radius_m': 0.45,
    'radius_m': 0.15, 'clearance_m': 0.10,
    'min_fill': 1.0, 'min_extent_m': 0.45, 'max_depth': 4,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/rslidar_points')
    ap.add_argument('--npy', default='')
    ap.add_argument('--pose-topic', default='/sim/odom_gt')
    ap.add_argument('--center', default='', help='x,y[,yaw]; 空则取 pose-topic 的真值位置')
    ap.add_argument('--out', default='')
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--min-fill', type=float, default=1.0,
                    help='1.0=一个连通分量一个 AABB（历史行为）；0.45 启用递归二分')
    ap.add_argument('--min-extent', type=float, default=0.45)
    ap.add_argument('--max-depth', type=int, default=4)
    ap.add_argument('--min-points', type=int, default=8,
                    help='簇的最小点数；提高可滤掉点云噪声形成的小簇（实测拐点 16）')
    ap.add_argument('--save-npy', default='', help='把这一帧点云存下来，便于同一帧做前后对比')
    args = ap.parse_args()

    params = dict(DEFAULTS)
    params['min_fill'] = args.min_fill
    params['min_extent_m'] = args.min_extent
    params['max_depth'] = args.max_depth
    params['min_points'] = args.min_points
    if args.npy:
        points = np.load(args.npy)
        center_xy = [float(v) for v in args.center.split(',')][:2] if args.center else [0.0, 0.0]
        center = (center_xy[0], center_xy[1], 0.0, 0.0)
    else:
        points = cloud_from_ros(args.topic, args.timeout)
        if args.center:
            vals = [float(v) for v in args.center.split(',')]
            center = (vals[0], vals[1], 0.0, vals[2] if len(vals) > 2 else 0.0)
        else:
            center = pose_from_ros(args.pose_topic, args.timeout)

    if args.save_npy:
        np.save(args.save_npy, points)

    rows, summary = audit(points, center, params)
    summary['min_fill'] = params['min_fill']

    # ★不漏点判据：与「一个分量一个 AABB」的基线比，未覆盖点数不得增加★
    # 绝对值为 0 做不到也不是本优化的责任：零星小簇（< min_points）历来不报障碍，
    # 那是既有行为。这里要证明的是「分裂没有让任何新点掉出去」。
    baseline = dict(params)
    baseline['min_fill'] = 1.0
    _, raw_summary = audit(points, center, baseline)
    summary['uncovered_baseline'] = raw_summary.get('uncovered_points')
    summary['coverage_ok'] = (summary.get('uncovered_points', 0)
                              <= (raw_summary.get('uncovered_points') or 0))
    summary['center'] = [round(v, 3) for v in center]
    summary['cloud_points'] = int(len(points))
    text = json.dumps({'summary': summary, 'boxes': rows}, ensure_ascii=False)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    if args.out:
        with open(args.out, 'w') as stream:
            stream.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
