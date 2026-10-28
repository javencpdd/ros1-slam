"""Actual ROS messages through native local node and experimental contact carrier.

Requires an existing dedicated roscore. Does not start Gazebo or claim dynamics.
The carrier integrates every observed 3D command at its receipt time, feeds back
actual xyz continuously, and never snaps a wrong result onto a support. Contact
events and the complete execution stream are recorded as separate channels.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import evaluate
import local_native as native
from test_local_native import BOX, flat, layers


def integrate_received(position, stamp, velocity, events, until):
    """Zero-order hold using actual receipt times, including all stationary gaps.

    No trajectory point, support projection, phase-duration clipping or inferred
    velocity is used. Returned samples retain each command change and final time.
    """
    samples = []
    for changed_at, next_velocity in list(events) + [(until, None)]:
        if changed_at < stamp - 1e-9 or changed_at > until + 1e-9:
            raise ValueError('command receipt time outside integration interval')
        duration = max(0., changed_at - stamp)
        position = tuple(position[i] + duration * velocity[i] for i in range(3))
        if duration > 0:
            samples.append((*position, changed_at))
        stamp = changed_at
        if next_velocity is not None:
            velocity = tuple(next_velocity)
    return position, stamp, velocity, samples


def start_node(argv, stream):
    """Child must stay in its caller's process group for lab-wide cleanup."""
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT)


def run_case(name, value, start, goal, obstacles, behavior, deadline_s):
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String

    node_name = 'a5_%s_%d' % (name, os.getpid())
    prefix = '/' + node_name
    shared = {'command': None, 'status': None, 'trajectory': None}
    event_lock = threading.Lock()
    command_events, command_log = [], []

    def on_command(message):
        velocity = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        with event_lock:
            received_at = rospy.Time.now().to_sec()
            command_events.append((received_at, velocity))
            command_log.append({'received_at': received_at, 'source_stamp': message.header.stamp.to_sec(),
                                'velocity': list(velocity)})
            shared['command'] = velocity
    observations = []
    callbacks = [
        rospy.Subscriber(prefix + '/cmd', TwistStamped, on_command, queue_size=100),
        rospy.Subscriber(prefix + '/status', String,
                         lambda m: shared.update(status=json.loads(m.data)), queue_size=1),
        rospy.Subscriber(prefix + '/trajectory', String,
                         lambda m: shared.update(trajectory=json.loads(m.data)), queue_size=1)]
    pose_pub = rospy.Publisher(prefix + '/pose', PoseStamped, queue_size=1)
    goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1, latch=True)
    obstacle_pub = rospy.Publisher(prefix + '/obstacles', String, queue_size=1)
    loc_pub = rospy.Publisher(prefix + '/localization', Bool, queue_size=1)
    pubs = [pose_pub, goal_pub, obstacle_pub, loc_pub]
    with tempfile.TemporaryDirectory(prefix='a5_runtime_') as directory:
        map_path, log_path = Path(directory) / 'map.json', Path(directory) / 'node.log'
        map_path.write_text(json.dumps(value), encoding='utf-8')
        argv = [sys.executable, '-B', str(Path(__file__).with_name('ros_node.py')),
                '__name:=' + node_name, '_map_json:=' + str(map_path),
                '_pose_topic:=' + prefix + '/pose', '_goal_topic:=' + prefix + '/goal',
                '_obstacles_topic:=' + prefix + '/obstacles', '_localization_topic:=' + prefix + '/localization',
                '_command_topic:=' + prefix + '/cmd', '_status_topic:=' + prefix + '/status',
                '_trajectory_topic:=' + prefix + '/trajectory']
        if name == 'step':
            argv.append('_experimental_config:=' + json.dumps({'tick_s': 1.2, 'validity_s': 1.5}))
        with log_path.open('w') as stream:
            # Inherit probe PGID so a runtime-lab group timeout also kills this
            # node. Normal cleanup targets this child PID, never our own group.
            proc = start_node(argv, stream)
        clean = False
        try:
            deadline = time.monotonic() + deadline_s
            while time.monotonic() < deadline and not all(p.get_num_connections() for p in pubs):
                if proc.poll() is not None:
                    raise RuntimeError('node exited: ' + log_path.read_text())
                time.sleep(.02)
            if not all(p.get_num_connections() for p in pubs):
                raise RuntimeError('subscription readiness timeout')
            epoch = rospy.Time.now().to_sec()
            if epoch <= 0:
                raise RuntimeError('ROS clock is not ready')
            truth = [dict(o, stamp=epoch) for o in obstacles]
            goal_msg = PoseStamped()
            goal_msg.header.frame_id, goal_msg.pose.orientation.w = 'map', 1.
            goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = goal
            goal_pub.publish(goal_msg)
            actual = tuple(start)
            execution_trace = [(*actual, epoch)]
            trace = [(*actual, epoch)]
            last_clock, integrated_velocity = epoch, (0., 0., 0.)
            with event_lock:
                command_events[:] = [(t, v) for t, v in command_events if t >= epoch]
            fault_at, saw_motion, passed, stopped_samples = None, False, False, 0
            while time.monotonic() < deadline and not rospy.is_shutdown():
                with event_lock:
                    stamp = rospy.Time.now().to_sec()
                    ready = [(t, v) for t, v in command_events if t <= stamp]
                    command_events[:] = [(t, v) for t, v in command_events if t > stamp]
                if stamp < last_clock:
                    raise RuntimeError('ROS clock went backwards during execution')
                actual, last_clock, integrated_velocity, samples = integrate_received(
                    actual, last_clock, integrated_velocity, ready, stamp)
                execution_trace.extend(samples)
                # The contact-state channel is separate from the complete
                # measured execution stream. Unsupported intermediate z is
                # retained in execution_trace and independently checked against
                # the declared edge between actual contact events.
                for sample in samples:
                    if any(abs(sample[2] - s['z_reference']) <= 1e-6 for s in value['surfaces']):
                        trace.append(sample)
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp, pose.pose.orientation.w = 'map', rospy.Time.now(), 1.
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = actual
                pose_pub.publish(pose)
                obstacle_pub.publish(String(data=json.dumps({'obstacles': truth})))
                loc_pub.publish(Bool(data=not (behavior == 'localization' and fault_at is not None)))
                velocity = shared['command']
                if velocity is not None:
                    moving = any(abs(v) > 1e-8 for v in velocity)
                    if moving and not saw_motion:
                        saw_motion = True
                        if behavior in ('localization', 'expiry'):
                            fault_at = time.monotonic()
                            if behavior == 'expiry':
                                rospy.set_param(prefix + '/pause_planning', True)
                    status = shared['status'] or {}
                    if behavior == 'blocked':
                        stopped_samples = stopped_samples + 1 if not moving and status.get('reason') == 'E_LOCAL_NO_ROUTE' else 0
                        if stopped_samples >= 5:
                            passed = True
                            break
                    elif fault_at is not None:
                        wanted = ('E_LOCAL_LOCALIZATION',) if behavior == 'localization' else ('E_LOCAL_EXPIRED', 'E_LOCAL_WALL_EXPIRED')
                        if not moving and status.get('reason') in wanted:
                            passed = True
                            observations.append({'zero_command': list(velocity), 'reason': status['reason'],
                                                 'fault_to_zero_wall_s': time.monotonic() - fault_at})
                            break
                    elif behavior == 'arrival':
                        if (math.sqrt(sum((actual[i] - goal[i]) ** 2 for i in range(3))) <= .08
                                and any(abs(actual[2] - s['z_reference']) <= 1e-6 for s in value['surfaces'])):
                            passed = True
                            break
                time.sleep(.01)
            last_result = shared['trajectory']
            independent = None
            if behavior == 'arrival' and last_result is not None and len(trace) > 1:
                record = dict(last_result, trajectory=trace, execution_trace=execution_trace)
                record['config'] = dict(last_result['config'], horizon_s=trace[-1][3] - trace[0][3] + 1.)
                independent = evaluate.evaluate(value, record, truth)
                passed = passed and independent['ok']
            latency_limit = native.Config().command_latency_bound_s
            latencies = [q['received_at'] - q['source_stamp'] for q in command_log if q['source_stamp'] > 0]
            latency_violations = [latency for latency in latencies if latency < 0. or latency > latency_limit]
            passed = passed and not latency_violations
            return_value = {'name': name, 'ok': passed, 'behavior': behavior, 'saw_nonzero_command': saw_motion,
                            'final_contact_xyz': actual, 'goal_xyz': goal, 'trajectory_points': len(trace),
                            'trace': trace, 'execution_trace': execution_trace, 'command_receipts': command_log,
                            'observations': observations, 'last_status': shared['status'],
                            'nonempty_trajectory': bool(last_result and last_result.get('trajectory')),
                            'command_transport': {'limit_s': latency_limit, 'sample_n': len(latencies),
                                                  'max_s': max(latencies) if latencies else None,
                                                  'violations': latency_violations},
                            'independent_evaluation': independent,
                            'domain': 'ROS_messages_with_continuous_command_receipt_integrator',
                            'gazebo': False, 'dynamics': False}
        finally:
            try:
                proc.terminate()
                proc.wait(timeout=5.)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.)
            clean = proc.poll() is not None
            for obj in callbacks + pubs:
                obj.unregister()
            try:
                rospy.delete_param(prefix)
            except KeyError:
                pass
        return_value['clean_exit'] = clean
        return_value['node_log'] = log_path.read_text()
        return return_value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=40.)
    parser.add_argument('--case', action='append', choices=['dynamic', 'step', 'blocked', 'localization', 'expiry'],
                        help='run only selected cases; omitted means all five')
    args = parser.parse_args()
    import rospy
    rospy.init_node('a5_runtime_probe', anonymous=True, disable_signals=True)
    cases = []
    definitions = [
        ('dynamic', flat(), (0., 0., 0.), (1.2, 0., 0.),
         [dict(BOX, min=[.6, .8, -.05], max=[1., 1.2, .8], velocity=[0., -.4, 0.])], 'arrival'),
        ('step', layers(), (-.4, 0., 0.), (.6, 0., .15), [], 'arrival'),
        ('blocked', flat(), (-.4, 0., 0.), (1.4, 0., 0.),
         [{'min': [-.15, -3., -.1], 'max': [.15, 3., 1.]}], 'blocked'),
        ('localization', flat(), (0., 0., 0.), (1., 0., 0.), [], 'localization'),
        ('expiry', flat(), (0., 0., 0.), (1., 0., 0.), [], 'expiry')]
    for definition in definitions:
        if args.case and definition[0] not in args.case:
            continue
        try:
            cases.append(run_case(*definition, deadline_s=args.wall_deadline_s))
        except Exception as error:
            cases.append({'name': definition[0], 'ok': False, 'error': repr(error)})
    result = {'ok': all(c['ok'] for c in cases), 'cases': cases, 'complete_suite': len(cases) == len(definitions),
              'gazebo': False, 'dynamics': False}
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'ok': result['ok'], 'cases': [(c['name'], c['ok']) for c in cases]}))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
