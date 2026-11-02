#!/usr/bin/env python3
"""Demo-line sensor survey: move a Gazebo model along a GT path and compare LIO.

This is intentionally SLAM-only: it uses /gazebo/set_model_state (no /cmd_vel,
no carrier gait/navigation) and starts 10_slam_mapping_sim.launch for faster-lio.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from collections import deque
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry

import localization_compare


def load_waypoints(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    points = value['waypoints']
    if len(points) < 2:
        raise ValueError('path needs at least two waypoints')
    return points, float(value.get('speed_mps', 0.5))


def interpolate(points, distance):
    """Return (x, y, z, yaw) at arc length distance; clamp at the end."""
    remaining = max(0.0, distance)
    for first, second in zip(points, points[1:]):
        dx, dy, dz = second['x'] - first['x'], second['y'] - first['y'], second['z'] - first['z']
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1e-9:
            continue
        if remaining <= length:
            ratio = remaining / length
            yaw = second.get('yaw', math.atan2(dy, dx))
            return (first['x'] + ratio * dx, first['y'] + ratio * dy,
                    first['z'] + ratio * dz, yaw)
        remaining -= length
    last = points[-1]
    return (last['x'], last['y'], last['z'], last.get('yaw', 0.0))


def _terminate(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=8.)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', required=True)
    parser.add_argument('--out', default='/tmp/demo_slam_survey.json')
    parser.add_argument('--duration-s', type=float, default=40.)
    parser.add_argument('--rate-hz', type=float, default=20.)
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--settle-s', type=float, default=3.)
    parser.add_argument('--max-age-s', type=float, default=.15)
    args = parser.parse_args()
    points, path_speed = load_waypoints(args.path)
    rospy.init_node('demo_slam_survey', anonymous=True, disable_signals=True)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    start = points[0]
    state = ModelState()
    state.model_name, state.reference_frame = args.model, 'world'
    state.pose.position.x, state.pose.position.y, state.pose.position.z = (
        start['x'], start['y'], start['z'])
    state.pose.orientation.z, state.pose.orientation.w = (
        math.sin(start.get('yaw', 0.) / 2.), math.cos(start.get('yaw', 0.) / 2.))
    set_state(state)
    time.sleep(args.settle_s)
    lio = (subprocess.Popen(['roslaunch', 'lite3_slam', '10_slam_mapping_sim.launch'],
                            start_new_session=True, stdout=open('/tmp/demo_slam_lio.log', 'ab'),
                            stderr=subprocess.STDOUT, env=dict(os.environ))
           if args.launch else None)
    gt_buffer, est_buffer = deque(maxlen=120), deque(maxlen=120)
    lock = __import__('threading').Lock()

    def on_gt(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        with lock:
            gt_buffer.append((message.header.stamp.to_sec(), time.monotonic(),
                              (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))

    def on_est(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        with lock:
            est_buffer.append((message.header.stamp.to_sec(), time.monotonic(),
                               (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))

    rospy.Subscriber('/sim/odom_gt', Odometry, on_gt, queue_size=20)
    rospy.Subscriber('/Odometry', Odometry, on_est, queue_size=20)
    deadline = time.monotonic() + 40.
    while time.monotonic() < deadline:
        with lock:
            if gt_buffer and est_buffer:
                break
        time.sleep(.2)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_s and not rospy.is_shutdown():
            elapsed = time.monotonic() - started
            x, y, z, yaw = interpolate(points, path_speed * elapsed)
            state = ModelState()
            state.model_name, state.reference_frame = args.model, 'world'
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
            state.pose.orientation.z, state.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
            try:
                set_state(state)
            except Exception:
                pass
            time.sleep(1. / args.rate_hz)
    finally:
        _terminate(lio)
    with lock:
        gt = list(gt_buffer)
        est = list(est_buffer)
    pairs = []
    for stamp, received, pose in est:
        nearest = min(gt, key=lambda item: (abs(item[0] - stamp)
                                            if stamp > 0 and item[0] > 0
                                            else abs(item[1] - received))) if gt else None
        if nearest is not None:
            delta = (abs(nearest[0] - stamp) if stamp > 0 and nearest[0] > 0
                     else abs(nearest[1] - received))
            if delta <= args.max_age_s:
                pairs.append((nearest[2], pose))
    result = {
        'scope': 'demo_two_floor_survey',
        'ok': bool(pairs),
        'launch': args.launch, 'path': str(Path(args.path).resolve()),
        'duration_s': args.duration_s, 'rate_hz': args.rate_hz,
        'path_speed_mps': path_speed, 'pairs': len(pairs),
        'gt_samples': len(gt), 'lio_samples': len(est),
        'compare': localization_compare.compare_samples(pairs),
        'gt_tail': [list(item[2][:3]) for item in gt[-3:]],
        'lio_tail': [list(item[2][:3]) for item in est[-3:]],
        'note': 'GT survey via set_model_state; no /cmd_vel, no navigation, no gait',
        'boundary': 'demo/SLAM only; faster-lio starts its own map frame; not formal'
    }
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({k: result[k] for k in ('ok', 'pairs', 'gt_samples', 'lio_samples',
                                             'compare')}, ensure_ascii=False))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
