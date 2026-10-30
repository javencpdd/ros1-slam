#!/usr/bin/env python3
"""A5 ROS local node -> V8-X1 MuJoCo policy chain on a NIST-profile stair world.

The local node plans 3D support trajectories; the bridge feeds map XY to the
policy and returns the highest declared support under the body. Contact
climbing supplies Z. Pilot only: no four-leg or real-robot acceptance.
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

HERE = Path(__file__).resolve().parent
LOCAL = HERE.parent / 'local_planner'
DYNAMICS = HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'dynamics'
sys.path.insert(0, str(DYNAMICS))
import mj_dreamwaq_walk as walk  # noqa: E402

EXPERIMENTAL = {'tick_s': 1.2, 'horizon_s': 12.0, 'validity_s': 12.0, 'clearance_m': .1}


def start(argv, stream):
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT)


def support_marks(applied, levels):
    """First applied record on each declared support, in declared order."""
    out, order = {}, []
    for level in levels:
        for record in applied:
            if abs(record.get('support_z', -1.) - level) <= 1e-6:
                out['%.4f' % level] = {'world_xyz': record.get('world_xyz'),
                                       'snapped': record.get('snapped'),
                                       'stamp': record.get('received_at')}
                order.append(level)
                break
    return out, order


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--xml', default='/tmp/mjcf/Lite3.xml')
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=120.)
    parser.add_argument('--step-h', type=float, default=.1988)
    parser.add_argument('--steps', type=int, default=5)
    parser.add_argument('--step-depth', type=float, default=.72)
    parser.add_argument('--landing-depth', type=float, default=2.0)
    parser.add_argument('--goal-map', default='0,1.34,0.890358')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--perturb', action='store_true')
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text(encoding='utf-8'))
    goal = tuple(float(v) for v in args.goal_map.split(','))
    stairs_xml = walk.make_stairs_xml(args.xml, args.step_h, args.steps, args.step_depth,
                                      3.0, 1.0, args.landing_depth)
    boundaries = [1.0 + args.step_depth * i for i in range(1, args.steps + 1)]

    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    rospy.init_node('a5_v8x1_runtime', anonymous=True)

    node_name = 'a5v8x1_%d' % os.getpid()
    prefix = '/' + node_name
    shared = {'body': None, 'map_pose': None, 'status': None, 'trajectories': 0, 'last_plan': None,
              'applied': 0, 'fault': None}
    body_trace, map_trace, status_log, applied_log = [], [], [], []
    lock = threading.Lock()

    def on_body(message):
        p = message.pose.position
        with lock:
            shared['body'] = (p.x, p.y, p.z)
            body_trace.append([message.header.stamp.to_sec(), round(p.x, 4), round(p.y, 4),
                               round(p.z, 4)])

    def on_map(message):
        p = message.pose.position
        with lock:
            shared['map_pose'] = (p.x, p.y, p.z)
            map_trace.append([message.header.stamp.to_sec(), round(p.x, 4), round(p.y, 4),
                              round(p.z, 4)])

    def on_status(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['status'] = payload
            status_log.append([rospy.Time.now().to_sec(), payload.get('reason'),
                               payload.get('complete'), payload.get('velocity')])

    def on_trajectory(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['trajectories'] += 1
            shared['last_plan'] = payload

    def on_applied(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['applied'] += 1
            payload['received_at'] = rospy.Time.now().to_sec()
            applied_log.append(payload)
            if payload.get('fault'):
                shared['fault'] = payload['fault']

    callbacks = [
        rospy.Subscriber(prefix + '/body', PoseStamped, on_body, queue_size=200),
        rospy.Subscriber(prefix + '/pose', PoseStamped, on_map, queue_size=100),
        rospy.Subscriber(prefix + '/status', String, on_status, queue_size=100),
        rospy.Subscriber(prefix + '/trajectory', String, on_trajectory, queue_size=10),
        rospy.Subscriber(prefix + '/applied', String, on_applied, queue_size=100)]
    goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1, latch=True)
    obstacle_pub = rospy.Publisher(prefix + '/obstacles', String, queue_size=1)
    loc_pub = rospy.Publisher(prefix + '/localization', Bool, queue_size=1)
    pubs = [goal_pub, obstacle_pub, loc_pub]

    config = dict(EXPERIMENTAL, resolution=value['resolution'])
    with tempfile.TemporaryDirectory(prefix='a5v8x1_') as directory:
        map_path, node_log, bridge_log = (Path(directory) / 'map.json',
                                          Path(directory) / 'node.log',
                                          Path(directory) / 'bridge.log')
        map_path.write_text(json.dumps(value), encoding='utf-8')
        node_argv = [sys.executable, '-B', str(LOCAL / 'ros_node.py'),
                     '__name:=' + node_name, '_map_json:=' + str(map_path),
                     '_pose_topic:=' + prefix + '/pose', '_goal_topic:=' + prefix + '/goal',
                     '_obstacles_topic:=' + prefix + '/obstacles',
                     '_localization_topic:=' + prefix + '/localization',
                     '_command_topic:=' + prefix + '/cmd', '_status_topic:=' + prefix + '/status',
                     '_trajectory_topic:=' + prefix + '/trajectory',
                     '_experimental_config:=' + json.dumps(config)]
        bridge_argv = [sys.executable, '-B', str(DYNAMICS / 'a5_v8x1_bridge.py'),
                       '--map', str(map_path), '--xml', stairs_xml, '--policy', args.policy,
                       '--prefix', prefix, '--seed', str(args.seed)]
        if args.perturb:
            bridge_argv.append('--perturb')
        node_stream, bridge_stream = node_log.open('w'), bridge_log.open('w')
        node = start(node_argv, node_stream)
        bridge = start(bridge_argv, bridge_stream)
        try:
            deadline = time.monotonic() + args.wall_deadline_s
            while time.monotonic() < deadline and not all(p.get_num_connections() for p in pubs):
                if node.poll() is not None or bridge.poll() is not None:
                    raise RuntimeError('child exited: ' + node_log.read_text() +
                                       bridge_log.read_text())
                time.sleep(.05)
            if not all(p.get_num_connections() for p in pubs):
                raise RuntimeError('subscription readiness timeout')
            message = PoseStamped()
            message.header.frame_id, message.pose.orientation.w = 'map', 1.
            message.pose.position.x, message.pose.position.y, message.pose.position.z = goal
            goal_pub.publish(message)
            arrived = fell = False
            while time.monotonic() < deadline and not rospy.is_shutdown():
                obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
                loc_pub.publish(Bool(data=True))
                time.sleep(.05)
                with lock:
                    body, pose = shared['body'], shared['map_pose']
                if body is None or pose is None:
                    continue
                if body[2] < .12:
                    fell = True
                    break
                with lock:
                    fault = shared['fault']
                if fault:
                    break
                distance = math.sqrt((pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2)
                if distance <= .10 and body[2] > args.step_h * args.steps + .1:
                    arrived = True
                    break
        finally:
            for proc in (bridge, node):
                if proc.poll() is None:
                    proc.send_signal(signal.SIGTERM)
            for proc in (bridge, node):
                try:
                    proc.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.)
            node_stream.close()
            bridge_stream.close()
            for obj in callbacks + pubs:
                obj.unregister()
            try:
                rospy.delete_param(prefix)
            except KeyError:
                pass
        node_text, bridge_text = node_log.read_text(), bridge_log.read_text()

    with lock:
        body_view, map_view = list(body_trace), list(map_trace)
        status_view = list(status_log)
        applied_view = [a for i, a in enumerate(applied_log) if i % 10 == 0]
        last_plan = shared['last_plan']
        plans = shared['trajectories']
        applied_n = shared['applied']
        fault = shared['fault']
    levels = sorted({surface['z_reference'] for surface in value['surfaces']
                     if surface.get('traversability') == 'supported'})
    step_marks, order = support_marks(applied_log, levels)
    times = [step_marks['%.4f' % level]['stamp'] for level in order]
    climbed = bool(len(step_marks) == len(levels) and all(t is not None for t in times)
                   and all(a < b for a, b in zip(times, times[1:])))
    final = body_view[-1] if body_view else None
    final_on_top = bool(final and final[1] >= boundaries[-2]
                        and final[3] > args.step_h * args.steps + .1)
    goal_map = bool(map_view and math.sqrt((map_view[-1][1] - goal[0]) ** 2 +
                                           (map_view[-1][2] - goal[1]) ** 2) <= .10)
    no_fall = bool(body_view and min(sample[3] for sample in body_view) > .12)
    active = bool(plans and applied_n and any(
        any(abs(v) > 1e-8 for v in record.get('body_command', ())) for record in applied_log))
    ok = bool(climbed and final_on_top and goal_map and no_fall and active and not fell
              and fault is None)
    payload = {
        'scenario': 'a5_ros_local_to_v8x1_nist_profile',
        'config': config, 'seed': args.seed, 'perturb': bool(args.perturb),
        'stair': {'steps': args.steps, 'step_h': args.step_h,
                                    'depth': args.step_depth, 'front': 1.0,
                                    'landing_depth': args.landing_depth},
        'scope': 'ROS local node + MuJoCo 270 policy; contact climbing; no Gazebo/no real robot/#151',
        'goal_map': list(goal), 'boundaries_world_x': boundaries,
        'ok': ok, 'arrived': arrived, 'fell': fell, 'bridge_fault': fault,
        'checks': {'climbed': climbed, 'final_on_top': final_on_top, 'goal_map': goal_map,
                   'no_fall': no_fall, 'a5_active': active},
        'plans': plans, 'applied_samples': applied_n,
        'step_marks': step_marks, 'final_body': final, 'final_map': map_view[-1] if map_view else None,
        'last_status': shared['status'], 'last_plan': last_plan,
        'body_trace': body_view[::5], 'map_trace': map_view[::5],
        'status_log': status_view[::5], 'applied': applied_view,
        'node_log': node_text, 'bridge_log': bridge_text,
    }
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'ok': ok, 'checks': payload['checks'], 'final_body': final,
                      'final_map': payload['final_map'], 'plans': plans,
                      'last_reason': (shared['status'] or {}).get('reason')}, ensure_ascii=False))
    return 0 if ok else 6


if __name__ == '__main__':
    raise SystemExit(main())
