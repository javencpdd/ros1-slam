#!/usr/bin/env python3
"""Read-only runtime check for the T1/T2/T3 localization contract."""
import argparse
import json
import math
import os
import statistics
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../ws/src/90_platform/v8_platform/framework'))
import pose_contract as pc  # noqa: E402


def pose_of(message):
    pose = getattr(message, 'pose', message)
    pose = getattr(pose, 'pose', pose)
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--contact-dz', type=float, default=-0.11)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    rospy.init_node('loc_tf_runtime_probe', anonymous=True, disable_signals=True)
    state = {'pose': [], 'plan': [], 'gt': [], 'ok': []}
    rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: state['pose'].append(pose_of(m)), queue_size=20)
    rospy.Subscriber('/v8/local/pose_plan', PoseStamped, lambda m: state['plan'].append(pose_of(m)), queue_size=20)
    rospy.Subscriber('/sim/odom_gt', Odometry, lambda m: state['gt'].append(pose_of(m)), queue_size=20)
    rospy.Subscriber('/v8/local/localization_ok', Bool, lambda m: state['ok'].append(bool(m.data)), queue_size=5)

    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)
    start = time.monotonic()
    rospy.sleep(max(1.0, args.seconds))
    wall = time.monotonic() - start

    tf_pose = None
    tf_error = ''
    try:
        transform = buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
        tr, rot = transform.transform.translation, transform.transform.rotation
        tf_pose = (tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w)
    except Exception as error:
        tf_error = str(error)[:200]

    pose = state['pose'][-1] if state['pose'] else None
    plan = state['plan'][-1] if state['plan'] else None
    gt = state['gt'][-1] if state['gt'] else None
    offsets = [a[2] - b[2] for a, b in zip(state['pose'], state['plan'])]
    result = {
        'wall_duration_s': round(wall, 3),
        'pose_samples': len(state['pose']),
        'plan_samples': len(state['plan']),
        'gt_samples': len(state['gt']),
        'navigation_ok_samples': len(state['ok']),
        'navigation_ok_last': state['ok'][-1] if state['ok'] else None,
        'contact_offset_median_m': round(statistics.median(offsets), 9) if offsets else None,
        'contact_offset_error_m': round(abs(statistics.median(offsets) - (-args.contact_dz)), 9) if offsets else None,
        'pose_last': pose,
        'plan_last': plan,
        'gt_last': gt,
        'loc_minus_gt_m': round(distance(pose, gt), 6) if pose and gt else None,
        'loc_minus_tf_m': round(distance(pose, tf_pose), 6) if pose and tf_pose else None,
        'loc_minus_tf_rpy': list(pc.rpy_from_quat(pose[3:])) if pose else None,
        'gt_rpy': list(pc.rpy_from_quat(gt[3:])) if gt else None,
        'tf_map_base': tf_pose,
        'tf_error': tf_error,
    }
    checks = {
        'localization_fresh': bool(state['ok'] and state['ok'][-1]),
        'pose_and_plan_present': len(state['pose']) >= 5 and len(state['plan']) >= 5,
        't3_contact_offset': result['contact_offset_error_m'] is not None and result['contact_offset_error_m'] <= 1e-6,
        'plan_orientation_matches_base': bool(pose and plan and abs(sum(a * b for a, b in zip(pose[3:], plan[3:]))) > 0.999999),
        'loc_matches_tf': result['loc_minus_tf_m'] is not None and result['loc_minus_tf_m'] <= 0.05,
        'gt_available': result['gt_samples'] > 0,
    }
    failed = [name for name, ok in checks.items() if not ok]
    result.update({'checks': checks, 'failed': failed, 'verdict': 'PASS' if not failed else 'FAIL'})
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as stream:
            stream.write(rendered + '\n')
    raise SystemExit(0 if result['verdict'] == 'PASS' else 1)


if __name__ == '__main__':
    main()