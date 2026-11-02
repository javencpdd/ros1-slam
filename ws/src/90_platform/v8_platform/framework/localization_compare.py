#!/usr/bin/env python3
"""GT vs faster-lio localization comparison (NDT monitor deferred).

Records /sim/odom_gt and /Odometry, optionally starts the simulation LIO launch
and drives lite3_planar with /cmd_vel. Because faster-lio starts its own map at
its first pose, this reports both raw frame offset and initial-pose-aligned
drift; the latter is the fair localization accuracy reading.
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

HERE = Path(__file__).resolve().parent


def yaw_of(q):
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


def error_between(gt, estimate):
    dx, dy, dz = (estimate[i] - gt[i] for i in range(3))
    dyaw = math.atan2(math.sin(yaw_of(estimate[3:]) - yaw_of(gt[3:])),
                      math.cos(yaw_of(estimate[3:]) - yaw_of(gt[3:])))
    return {'xy_m': math.hypot(dx, dy), 'z_m': abs(dz), 'yaw_rad': abs(dyaw),
            'distance_m': math.sqrt(dx * dx + dy * dy + dz * dz)}


def error_stats(errors):
    if not errors:
        return None
    keys = ('xy_m', 'z_m', 'yaw_rad', 'distance_m')
    result = {'samples': len(errors)}
    for key in keys:
        values = sorted(float(item[key]) for item in errors)
        result[key + '_mean'] = sum(values) / len(values)
        result[key + '_rms'] = math.sqrt(sum(v * v for v in values) / len(values))
        result[key + '_max'] = values[-1]
    return result


def aligned(gt, estimate, offset):
    return (estimate[0] - offset[0], estimate[1] - offset[1], estimate[2] - offset[2],
            estimate[3], estimate[4], estimate[5], estimate[6])


def rigid(gt0, est0, estimate):
    """First-pose rigid alignment (yaw + translation), preserving scale."""
    dyaw = yaw_of(est0[3:]) - yaw_of(gt0[3:])
    cos_d, sin_d = math.cos(-dyaw), math.sin(-dyaw)
    dx, dy = estimate[0] - est0[0], estimate[1] - est0[1]
    x = cos_d * dx - sin_d * dy + gt0[0]
    y = sin_d * dx + cos_d * dy + gt0[1]
    z = estimate[2] - est0[2] + gt0[2]
    yaw = yaw_of(estimate[3:]) - dyaw
    return (x, y, z, 0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))


def compare_samples(pairs):
    """pairs = [(gt_pose, estimate_pose), ...]; raw/translated/rigid stats."""
    raw = [error_between(gt, est) for gt, est in pairs]
    if not pairs:
        return {'raw': None, 'translated': None, 'rigid': None, 'origin_offset': None}
    offset = tuple(pairs[0][1][i] - pairs[0][0][i] for i in range(3))
    moved = [error_between(gt, aligned(gt, est, offset)) for gt, est in pairs]
    rigid_errors = [error_between(gt, rigid(pairs[0][0], pairs[0][1], est))
                    for gt, est in pairs]
    dyaw = yaw_of(pairs[0][1][3:]) - yaw_of(pairs[0][0][3:])
    return {'raw': error_stats(raw), 'translated': error_stats(moved),
            'rigid': error_stats(rigid_errors),
            'origin_offset': list(offset), 'origin_yaw_rad': dyaw}


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _metric_stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'mean': sum(ordered) / len(ordered),
            'rms': math.sqrt(sum(v * v for v in ordered) / len(ordered)),
            'max': ordered[-1]}


def _relative_xy(first, second):
    """second position in first pose's yaw frame."""
    yaw = yaw_of(first[3:])
    dx, dy = second[0] - first[0], second[1] - first[1]
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    return cos_y * dx - sin_y * dy, sin_y * dx + cos_y * dy


def rpe_stats(timed_pairs, window_s=1.0):
    """Relative pose error over a fixed time window.

    timed_pairs = [(t, gt_pose, estimate_pose), ...]. Translation is measured in
    the start pose's yaw frame; yaw error is the relative yaw difference.
    """
    if not timed_pairs:
        return {'samples': 0, 'window_s': window_s}
    samples, next_index = [], 1
    for index, (stamp, gt0, est0) in enumerate(timed_pairs):
        if next_index <= index:
            next_index = index + 1
        while (next_index < len(timed_pairs)
               and timed_pairs[next_index][0] - stamp < window_s):
            next_index += 1
        if next_index >= len(timed_pairs):
            break
        end_stamp, gt1, est1 = timed_pairs[next_index]
        gt_dx, gt_dy = _relative_xy(gt0, gt1)
        est_dx, est_dy = _relative_xy(est0, est1)
        gt_dyaw = _wrap_angle(yaw_of(gt1[3:]) - yaw_of(gt0[3:]))
        est_dyaw = _wrap_angle(yaw_of(est1[3:]) - yaw_of(est0[3:]))
        samples.append({
            'dt_s': end_stamp - stamp,
            'translation_m': math.hypot(gt_dx - est_dx, gt_dy - est_dy),
            'z_m': abs((gt1[2] - gt0[2]) - (est1[2] - est0[2])),
            'yaw_rad': abs(_wrap_angle(est_dyaw - gt_dyaw)),
        })
    if not samples:
        return {'samples': 0, 'window_s': window_s}
    return {
        'samples': len(samples), 'window_s': window_s,
        'dt_s_mean': sum(item['dt_s'] for item in samples) / len(samples),
        'translation_m': _metric_stats([item['translation_m'] for item in samples]),
        'z_m': _metric_stats([item['z_m'] for item in samples]),
        'yaw_rad': _metric_stats([item['yaw_rad'] for item in samples]),
    }


def tracking_stats(est_samples, pairs, gap_s=1.0, xy_threshold_m=0.5):
    """Descriptive LIO sample gaps and contiguous xy-error exceedances."""
    stamps = sorted({float(item[0]) for item in est_samples if item[0] > 0})
    gaps = [second - first for first, second in zip(stamps, stamps[1:]) if second > first]
    errors = []
    if pairs:
        offset = tuple(pairs[0][1][i] - pairs[0][0][i] for i in range(3))
        errors = [error_between(gt, aligned(gt, est, offset))['xy_m']
                  for gt, est in pairs]
    segments = 0
    previous = False
    for error in errors:
        over = error > xy_threshold_m
        if over and not previous:
            segments += 1
        previous = over
    return {
        'est_samples': len(stamps),
        'max_gap_s': max(gaps) if gaps else None,
        'mean_gap_s': (sum(gaps) / len(gaps)) if gaps else None,
        'gaps_over_s': sum(1 for gap in gaps if gap > gap_s),
        'gap_threshold_s': gap_s,
        'max_xy_m': max(errors) if errors else None,
        'xy_over_threshold_samples': sum(1 for error in errors if error > xy_threshold_m),
        'xy_over_threshold_segments': segments,
        'xy_threshold_m': xy_threshold_m,
    }


def _terminate(proc, group=False):
    if proc is None or proc.poll() is not None:
        return
    try:
        if group:
            os.killpg(proc.pid, signal.SIGTERM)
        else:
            proc.terminate()
        proc.wait(timeout=8.)
    except Exception:
        try:
            if group:
                os.killpg(proc.pid, signal.SIGKILL)
            else:
                proc.kill()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/tmp/localization_compare.json')
    parser.add_argument('--duration-s', type=float, default=20.)
    parser.add_argument('--launch', action='store_true', help='start 10_slam_mapping_sim.launch')
    parser.add_argument('--no-move', action='store_true')
    parser.add_argument('--speed', type=float, default=.25)
    parser.add_argument('--drive-mode', choices=['cmd_vel', 'set_state'], default='cmd_vel')
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--place-model', default='', help='x,y,z[,yaw] place before LIO starts')
    parser.add_argument('--place-settle-s', type=float, default=.8)
    parser.add_argument('--max-age-s', type=float, default=.15)
    args = parser.parse_args()
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rospy.init_node('localization_compare', anonymous=True, disable_signals=True)
    if args.place_model:
        values = [float(v) for v in args.place_model.split(',')]
        if len(values) not in (3, 4):
            raise SystemExit('--place-model must be x,y,z[,yaw]')
        yaw = values[3] if len(values) == 4 else 0.
        state = ModelState()
        state.model_name, state.reference_frame = args.model, 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = values[:3]
        state.pose.orientation.z, state.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
        rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
        time.sleep(args.place_settle_s)
    lio = subprocess.Popen(['roslaunch', 'lite3_slam', '10_slam_mapping_sim.launch'],
                           start_new_session=True, stdout=open('/tmp/r1103_lio.log', 'ab'),
                           stderr=subprocess.STDOUT, env=dict(os.environ)) if args.launch else None
    gt_buffer, est_buffer = deque(maxlen=60), deque(maxlen=60)
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
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    set_state = None
    if args.drive_mode == 'set_state':
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    deadline = time.monotonic() + 35.0
    while time.monotonic() < deadline:
        with lock:
            if gt_buffer and est_buffer:
                break
        time.sleep(.2)
    started = time.monotonic()
    start_pose = None
    with lock:
        if gt_buffer:
            start_pose = gt_buffer[-1][2]
    try:
        while time.monotonic() - started < args.duration_s and not rospy.is_shutdown():
            if not args.no_move:
                if args.drive_mode == 'cmd_vel':
                    command = Twist()
                    command.linear.x = args.speed
                    command.angular.z = .08 * math.sin((time.monotonic() - started) * .8)
                    cmd_pub.publish(command)
                elif set_state is not None and start_pose is not None:
                    state = ModelState()
                    state.model_name, state.reference_frame = args.model, 'world'
                    state.pose.position.x = start_pose[0] + args.speed * (time.monotonic() - started)
                    state.pose.position.y = start_pose[1]
                    state.pose.position.z = start_pose[2]
                    state.pose.orientation.w = 1.
                    try:
                        set_state(state)
                    except Exception:
                        pass
            time.sleep(.05)
        if args.drive_mode == 'cmd_vel':
            cmd_pub.publish(Twist())
    finally:
        _terminate(lio, group=True)
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
    result = {'scope': 'gt_vs_faster_lio_localization', 'ok': bool(pairs),
              'launch': args.launch, 'place_model': args.place_model, 'duration_s': args.duration_s,
              'speed_mps': args.speed, 'pairs': len(pairs),
              'gt_samples': len(gt), 'lio_samples': len(est),
              'compare': compare_samples(pairs),
              'gt_tail': [list(item[2][:3]) for item in gt[-3:]],
              'lio_tail': [list(item[2][:3]) for item in est[-3:]],
              'note': 'translated removes first translation; rigid removes first yaw+translation',
              'boundary': 'simulation only; faster-lio starts its own map frame; NDT monitor deferred'}
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({k: result[k] for k in ('ok', 'pairs', 'gt_samples', 'lio_samples',
                                             'compare')}, ensure_ascii=False))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
