#!/usr/bin/env python3
"""Read-only contact residual probe for T1/T2/T3.

Records per-frame source stamps, frames, base/contact z, support z_reference and
the raw pairing delta. It reports statistics only: the 0.08 m tolerance is not an
approved gate.
"""
import argparse
import csv
import json
import math
import os
import statistics
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def xyzq(message):
    pose = getattr(message, 'pose', message)
    pose = getattr(pose, 'pose', pose)
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def load_surface(path, frame):
    with open(path, encoding='utf-8') as stream:
        value = json.load(stream)
    surfaces = [s for s in value.get('surfaces', []) if s.get('traversability') == 'supported']
    if not surfaces:
        raise SystemExit('no supported surface in %s' % path)
    chosen = min(surfaces, key=lambda s: float(s['z_reference']))
    if frame and value.get('frame') != frame:
        raise SystemExit('map frame %r != requested %r' % (value.get('frame'), frame))
    return value, chosen


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--pair-tol', type=float, default=0.06)
    parser.add_argument('--out-json', required=True)
    parser.add_argument('--out-csv', required=True)
    args = parser.parse_args()

    map_value, surface = load_surface(args.map, args.frame)
    z_ref = float(surface['z_reference'])
    rospy.init_node('contact_residual_probe', anonymous=True, disable_signals=True)
    poses, plans, odoms, gts = [], [], [], []

    def pose_cb(message, bucket):
        stamp = message.header.stamp.to_sec()
        bucket.append((stamp, message.header.frame_id, xyzq(message)))

    def odom_cb(message):
        odoms.append((message.header.stamp.to_sec(), message.header.frame_id, message.child_frame_id, xyzq(message)))

    def gt_cb(message):
        gts.append((message.header.stamp.to_sec(), xyzq(message)))

    rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: pose_cb(m, poses), queue_size=50)
    rospy.Subscriber('/v8/local/pose_plan', PoseStamped, lambda m: pose_cb(m, plans), queue_size=50)
    rospy.Subscriber('/Odometry', Odometry, odom_cb, queue_size=50)
    rospy.Subscriber('/sim/odom_gt', Odometry, gt_cb, queue_size=50)
    wall_start = time.monotonic()
    rospy.sleep(max(1.0, args.seconds))
    wall_duration = time.monotonic() - wall_start

    def nearest(items, stamp):
        if not items:
            return None
        candidate = min(items, key=lambda item: abs(item[0] - stamp))
        return candidate if abs(candidate[0] - stamp) <= args.pair_tol else None

    rows = []
    residuals = []
    pair_deltas = []
    for stamp, frame, plan_pose in plans:
        pose = nearest(poses, stamp)
        odom = nearest(odoms, stamp)
        gt = nearest(gts, stamp)
        if pose is None or odom is None:
            continue
        base = pose[2]
        residual = plan_pose[2] - z_ref
        pair_dt = max(abs(pose[0] - stamp), abs(odom[0] - stamp))
        pair_deltas.append(pair_dt)
        residuals.append(residual)
        rows.append({
            'plan_stamp': '%.9f' % stamp,
            'pose_stamp': '%.9f' % pose[0],
            'odom_stamp': '%.9f' % odom[0],
            'gt_stamp': '%.9f' % gt[0] if gt else '',
            'pair_dt_s': '%.9f' % pair_dt,
            'plan_frame': frame,
            'pose_frame': pose[1],
            'odom_frame': odom[1],
            'odom_child': odom[2],
            'base_x': '%.9f' % base[0],
            'base_y': '%.9f' % base[1],
            'base_z': '%.9f' % base[2],
            'plan_x': '%.9f' % plan_pose[0],
            'plan_y': '%.9f' % plan_pose[1],
            'plan_z': '%.9f' % plan_pose[2],
            'z_reference': '%.9f' % z_ref,
            'residual_m': '%.9f' % residual,
            'q_x': '%.9f' % plan_pose[3],
            'q_y': '%.9f' % plan_pose[4],
            'q_z': '%.9f' % plan_pose[5],
            'q_w': '%.9f' % plan_pose[6],
            'gt_x': '%.9f' % gt[1][0] if gt else '',
            'gt_y': '%.9f' % gt[1][1] if gt else '',
            'gt_z': '%.9f' % gt[1][2] if gt else '',
        })

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    with open(args.out_csv, 'w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()) if rows else ['plan_stamp'])
        writer.writeheader()
        writer.writerows(rows)

    if residuals:
        ordered = sorted(residuals)
        summary = {
            'samples': len(rows),
            'wall_duration_s': round(wall_duration, 3),
            'map_id': map_value.get('map_id'),
            'map_version': map_value.get('map_version'),
            'map_sha256_source': os.path.abspath(args.map),
            'surface_id': surface.get('surface_id'),
            'z_reference': z_ref,
            'residual_m': {
                'min': min(residuals), 'max': max(residuals),
                'mean': statistics.mean(residuals),
                'std': statistics.pstdev(residuals),
                'p50': statistics.median(residuals),
                'p95': ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))],
                'abs_max': max(abs(v) for v in residuals),
            },
            'pair_dt_max_s': max(pair_deltas),
            'report_only_thresholds': {
                'abs_max_within_0_05': max(abs(v) for v in residuals) <= 0.05,
                'abs_max_within_0_08': max(abs(v) for v in residuals) <= 0.08,
                'note': '0.08 m is not an approved formal tolerance; this is a reading only',
            },
            'verdict': 'MEASURED',
        }
    else:
        summary = {'samples': 0, 'verdict': 'NO_DATA'}
    with open(args.out_json, 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if rows else 2)


if __name__ == '__main__':
    main()