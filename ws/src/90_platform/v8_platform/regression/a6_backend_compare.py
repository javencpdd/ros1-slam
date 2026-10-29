#!/usr/bin/env python3
"""A6 accuracy comparison on one live sensor stream.

Both backends consume the same /rslidar_points (+ /livox/imu): faster_lio
publishes /Odometry and the NDT monitor publishes ~odom. Evaluation always uses
/sim/odom_gt truth. This compares accuracy; it does not promote NDT to a
continuous navigation backend and does not switch navigation odometry.
"""
import argparse
import json
import math
from pathlib import Path
import time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def nearest(samples, stamp, limit=0.1):
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s['t'] - stamp))
    return best if abs(best['t'] - stamp) <= limit else None


def metrics(gt, est):
    pairs = []
    for g in gt:
        e = nearest(est, g['t'])
        if e is not None:
            pairs.append((g, e))
    if len(pairs) < 2:
        return {'matched': len(pairs), 'covered': 0.0}
    origin_g, origin_e = pairs[0][0], pairs[0][1]
    errors, yaw_errors = [], []
    for g, e in pairs:
        errors.append(math.hypot((e['x'] - origin_e['x']) - (g['x'] - origin_g['x']),
                                 (e['y'] - origin_e['y']) - (g['y'] - origin_g['y'])))
        yaw_errors.append(wrap((e['yaw'] - origin_e['yaw']) - (g['yaw'] - origin_g['yaw'])))
    relative = []
    for i in range(len(pairs)):
        for j in range(i + 1, len(pairs)):
            g0, e0 = pairs[i]
            g1, e1 = pairs[j]
            dg = math.hypot(g1['x'] - g0['x'], g1['y'] - g0['y'])
            if dg < 0.9 or dg > 1.1:
                continue
            de = math.hypot((e1['x'] - e0['x']) - (g1['x'] - g0['x']),
                            (e1['y'] - e0['y']) - (g1['y'] - g0['y']))
            relative.append(de / dg)
    return {'matched': len(pairs), 'covered': len(pairs) / len(gt),
            'ate_rmse_m': math.sqrt(sum(e * e for e in errors) / len(errors)),
            'ate_max_m': max(errors), 'ate_final_m': errors[-1],
            'yaw_rmse_rad': math.sqrt(sum(e * e for e in yaw_errors) / len(yaw_errors)),
            'rpe_1m_mean': sum(relative) / len(relative) if relative else None,
            'rpe_1m_max': max(relative) if relative else None,
            'rpe_pairs': len(relative)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--drive-seconds', type=float, default=12.0)
    parser.add_argument('--speed', type=float, default=0.25)
    parser.add_argument('--settle-seconds', type=float, default=3.0)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--lio-topic', default='/Odometry')
    parser.add_argument('--ndt-topic', default='/a6_ndt/odom')
    parser.add_argument('--gt-topic', default='/sim/odom_gt')
    parser.add_argument('--no-lio', action='store_true')
    args = parser.parse_args()
    if not (args.drive_seconds > 0 and args.speed > 0 and args.settle_seconds >= 0 and args.timeout > 0):
        raise SystemExit('drive, speed and timeout must be positive')
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32

    rospy.init_node('a6_backend_compare', anonymous=True, disable_signals=True)
    records = {'lio': [], 'ndt': [], 'gt': [], 'health': [], 'score': []}

    def odom(key):
        def callback(message):
            pose = message.pose.pose
            records[key].append({'t': message.header.stamp.to_sec(),
                                 'x': pose.position.x, 'y': pose.position.y,
                                 'yaw': yaw_of(pose.orientation)})
        return callback

    subscriptions = [
        rospy.Subscriber(args.lio_topic, Odometry, odom('lio'), queue_size=500),
        rospy.Subscriber(args.ndt_topic, Odometry, odom('ndt'), queue_size=500),
        rospy.Subscriber(args.gt_topic, Odometry, odom('gt'), queue_size=500),
        rospy.Subscriber('/a6_ndt/healthy', Bool, lambda m: records['health'].append(
            {'t': rospy.Time.now().to_sec(), 'healthy': bool(m.data)}), queue_size=200),
        rospy.Subscriber('/a6_ndt/score', Float32, lambda m: records['score'].append(
            {'t': rospy.Time.now().to_sec(), 'score': float(m.data)}), queue_size=200),
    ]
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        ready = records['gt'] and records['ndt'] and (args.no_lio or records['lio'])
        if ready:
            break
        rospy.sleep(0.1)
    if not (records['gt'] and records['ndt'] and (args.no_lio or records['lio'])):
        data = {'ready': False, 'topics': {k: len(records[k]) for k in ('gt', 'lio', 'ndt')}}
        Path(args.out).write_text(json.dumps(data, indent=2) + '\n')
        return 2

    rate = rospy.Rate(20)
    start = rospy.Time.now().to_sec()
    command = Twist()
    command.linear.x = args.speed
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start < args.drive_seconds:
        publisher.publish(command)
        rate.sleep()
    publisher.publish(Twist())
    stop = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - stop < args.settle_seconds:
        publisher.publish(Twist())
        rate.sleep()

    t0 = start - 0.2
    t1 = stop + args.settle_seconds
    gt = [s for s in records['gt'] if t0 <= s['t'] <= t1]
    lio = [s for s in records['lio'] if t0 - 0.2 <= s['t'] <= t1 + 0.2]
    ndt = [s for s in records['ndt'] if t0 - 0.2 <= s['t'] <= t1 + 0.2]
    health = records['health']
    scores = records['score']
    result = {
        'domain': 'a6_same_stream_lio_vs_ndt_monitor',
        'hardware': False,
        'accuracy_backend_switch_tested': False,
        'ndt_role': 'monitor',
        'topics': {'lio': args.lio_topic, 'ndt': args.ndt_topic, 'gt': args.gt_topic},
        'window': {'t0': t0, 't1': t1, 'drive_seconds': args.drive_seconds, 'speed_m_s': args.speed},
        'samples': {'gt': len(gt), 'lio': len(lio), 'ndt': len(ndt)},
        'lio': {'matched': 0, 'note': 'not run'} if args.no_lio else metrics(gt, lio),
        'ndt': metrics(gt, ndt),
        'ndt_health': {'messages': len(health),
                       'healthy_true': sum(1 for h in health if h['healthy']),
                       'healthy_false': sum(1 for h in health if not h['healthy'])},
        'ndt_score': {'messages': len(scores),
                      'mean': (sum(s['score'] for s in scores) / len(scores)) if scores else None,
                      'max': max((s['score'] for s in scores), default=None)},
        'samples_raw': {'gt': gt, 'lio': lio, 'ndt': ndt},
    }
    Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'lio': result['lio'], 'ndt': result['ndt'],
                      'health': result['ndt_health'], 'samples': result['samples']}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
