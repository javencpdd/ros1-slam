#!/usr/bin/env python3
"""Record NDT monitor outputs while a bag replays one frozen sensor stream.

The bag holds /rslidar_points, /livox/imu, /sim/odom_gt and /clock; this node
only listens, so the same stream can be replayed under different NDT settings.
No navigation backend is switched.
"""
import argparse
import json
import math
from pathlib import Path
import time

from a6_backend_compare import metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--quiet-after', type=float, default=2.0)
    args = parser.parse_args()
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32, Float32MultiArray

    rospy.init_node('a6_ndt_replay', anonymous=True, disable_signals=True)
    records = {'odom': [], 'health': [], 'score': [], 'gt': [], 'debug': []}

    def record(store):
        def callback(message):
            pose = message.pose.pose
            q = pose.orientation
            store.append({'t': message.header.stamp.to_sec(), 'x': pose.position.x,
                          'y': pose.position.y,
                          'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y),
                                            1 - 2 * (q.y * q.y + q.z * q.z))})
        return callback

    subscriptions = [
        rospy.Subscriber('/a6_ndt/odom', Odometry, record(records['odom']), queue_size=500),
        rospy.Subscriber('/sim/odom_gt', Odometry, record(records['gt']), queue_size=500),
        rospy.Subscriber('/a6_ndt/healthy', Bool, lambda m: records['health'].append(
            {'t': rospy.Time.now().to_sec(), 'healthy': bool(m.data)}), queue_size=500),
        rospy.Subscriber('/a6_ndt/score', Float32, lambda m: records['score'].append(
            {'t': rospy.Time.now().to_sec(), 'score': float(m.data)}), queue_size=500),
        rospy.Subscriber('/a6_ndt/debug', Float32MultiArray, lambda m: records['debug'].append(
            {'t': rospy.Time.now().to_sec(),
             'accepted': bool(m.data[0]) if len(m.data) > 0 else None,
             'score': float(m.data[1]) if len(m.data) > 1 else None,
             'converged': bool(m.data[2]) if len(m.data) > 2 else None}), queue_size=500),
    ]
    deadline = time.monotonic() + args.timeout
    last_gt_time = None
    seen = 0
    while time.monotonic() < deadline:
        if len(records['gt']) != seen:
            seen = len(records['gt'])
            last_gt_time = time.monotonic()
        elif last_gt_time is not None and time.monotonic() - last_gt_time > args.quiet_after:
            break
        time.sleep(0.05)
    gt = records['gt']
    result = {
        'domain': 'a6_ndt_replay_same_bag',
        'hardware': False,
        'accuracy_backend_switch_tested': False,
        'samples': {'gt': len(gt), 'ndt': len(records['odom'])},
        'ndt': metrics(gt, records['odom']) if len(gt) >= 2 else {'matched': 0},
        'ndt_health': {'messages': len(records['health']),
                       'healthy_true': sum(1 for h in records['health'] if h['healthy']),
                       'healthy_false': sum(1 for h in records['health'] if not h['healthy'])},
        'ndt_score': {'messages': len(records['score']),
                      'mean': (sum(s['score'] for s in records['score']) / len(records['score']))
                              if records['score'] else None,
                      'max': max((s['score'] for s in records['score']), default=None)},
        'ndt_debug': {'messages': len(records['debug']),
                      'converged_true': sum(1 for d in records['debug'] if d['converged']),
                      'converged_false': sum(1 for d in records['debug'] if d['converged'] is False),
                      'accepted_true': sum(1 for d in records['debug'] if d['accepted'])},
        'debug_raw': records['debug'],
    }
    Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
