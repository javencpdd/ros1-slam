#!/usr/bin/env python3
"""Consume sensor-built support maps live: version switch, pose noise, obstacles.

Runs the real global planner on each map version, the native local planner with a
map topic, and the Gazebo kinematic carrier. v1 is a partial map; v2 is the full
sensor-built map. The carrier pose may be perturbed on x/y (localization noise)
before it is fed to the local planner; arrival is always judged on the carrier.
"""
import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
LOCAL = HERE.parent / 'local_planner'
SCRIPTS = HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'scripts'
EXPERIMENTAL = {'tick_s': 1.2, 'horizon_s': 6.0, 'validity_s': 6.0}
MAP_TOPIC = '/v8/sensor_map'


def parse_xyz(text):
    return tuple(float(v) for v in text.split(','))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-v1', required=True)
    parser.add_argument('--map-v2', required=True)
    parser.add_argument('--start', default='0,-3.06,0')
    parser.add_argument('--goal-v1', default='0,-0.6,0.371516')
    parser.add_argument('--goal-v2', default='0,1.34,0.888142')
    parser.add_argument('--top-goal', default='0,1.34,0.888142')
    parser.add_argument('--terrain', default='nist_stairs_map')
    parser.add_argument('--out', required=True)
    parser.add_argument('--phase-timeout-s', type=float, default=90.)
    parser.add_argument('--wall-deadline-s', type=float, default=220.)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--pose-noise-m', type=float, default=0.0)
    parser.add_argument('--pose-noise-deg', type=float, default=0.0)
    parser.add_argument('--obstacle-json', default='')
    parser.add_argument('--obstacle-after-s', type=float, default=3.0)
    parser.add_argument('--blocking-obstacle', action='store_true')
    parser.add_argument('--expect-block', action='store_true')
    parser.add_argument('--skip-v1', action='store_true')
    parser.add_argument('--radius-m', type=float, default=.15)
    parser.add_argument('--clearance-m', type=float, default=.10)
    parser.add_argument('--max-step-m', type=float, default=.20)
    parser.add_argument('--carrier-snap', action='store_true')
    args = parser.parse_args()
    start = parse_xyz(args.start)
    goal_v1 = parse_xyz(args.goal_v1)
    goal_v2 = parse_xyz(args.goal_v2)
    top_goal = parse_xyz(args.top_goal)
    v1 = json.loads(Path(args.map_v1).read_text(encoding='utf-8'))
    v2 = json.loads(Path(args.map_v2).read_text(encoding='utf-8'))
    obstacle_payload = None
    if args.obstacle_json:
        obstacle_payload = json.loads(Path(args.obstacle_json).read_text(encoding='utf-8'))
    elif args.blocking_obstacle:
        obstacle_payload = {'obstacles': [{'obstacle_id': 'blocking', 'min': [-1.6, -0.6, 0.2],
                                           'max': [1.6, 0.0, 1.0]}]}

    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import DeleteModel, GetLinkState, GetWorldProperties, SpawnModel
    from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('gazebo3d_live_map', anonymous=True)
    import a4_runtime
    global_runs = {}
    for label, value, goal in (('v1_top', v1, top_goal), ('v1_goal', v1, goal_v1),
                               ('v2_top', v2, top_goal)):
        route, error, info = a4_runtime.call_node(
            value, start, goal, 20000, 'livemap_%s_%d' % (label, os.getpid()))
        global_runs[label] = {'route': route, 'error': error,
                              'points': 0 if route is None else len(route),
                              'last': None if route is None else list(route[-1]),
                              'info': info}

    carrier_name = 'live_carrier_%d' % os.getpid()
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=20.)
    rospy.wait_for_service('/gazebo/get_link_state', timeout=20.)
    rospy.wait_for_service('/gazebo/delete_model', timeout=20.)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    sdf = ('<sdf version="1.6"><model name="%s"><static>true</static>'
           '<link name="link"><visual name="visual"><geometry><box><size>0.45 0.30 0.20</size></box>'
           '</geometry><material><ambient>0.2 0.5 0.9 1</ambient></material></visual>'
           '</link></model></sdf>' % carrier_name)
    if not spawn(carrier_name, sdf, '', Pose(), 'world').success:
        raise RuntimeError('carrier spawn failed')
    world_models = list(world().model_names)

    node_name = 'live_%d' % os.getpid()
    prefix = '/' + node_name
    shared = {'raw': None, 'status': None, 'plans': 0, 'last_plan': None,
              'trajectories': []}
    raw_trace, status_log, cmd_log = [], [], []
    noisy_log = []
    lock = threading.Lock()
    rng = random.Random(args.seed)

    def on_raw(message):
        p = message.pose.position
        q = message.pose.orientation
        with lock:
            shared['raw'] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                             message.header.stamp.to_sec())
            raw_trace.append([message.header.stamp.to_sec(), p.x, p.y, p.z])

    def on_status(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['status'] = payload
            status_log.append([rospy.Time.now().to_sec(), payload.get('reason'),
                               payload.get('map_version'), payload.get('detail')])

    def on_noisy(message):
        with lock:
            noisy_log.append([rospy.Time.now().to_sec(), message.pose.position.x,
                              message.pose.position.y, message.pose.position.z])

    def on_cmd(message):
        with lock:
            cmd_log.append([rospy.Time.now().to_sec(), message.twist.linear.x,
                            message.twist.linear.y, message.twist.linear.z])

    def on_trajectory(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['plans'] += 1
            shared['last_plan'] = payload
            shared['trajectories'].append(payload)

    callbacks = [
        rospy.Subscriber(prefix + '/pose', PoseStamped, on_raw, queue_size=200),
        rospy.Subscriber(prefix + '/status', String, on_status, queue_size=100),
        rospy.Subscriber(prefix + '/trajectory', String, on_trajectory, queue_size=50),
        rospy.Subscriber(prefix + '/cmd', TwistStamped, on_cmd, queue_size=200),
        rospy.Subscriber(prefix + '/noisy_pose', PoseStamped, on_noisy, queue_size=50)]
    noisy_pub = rospy.Publisher(prefix + '/noisy_pose', PoseStamped, queue_size=50)
    goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1, latch=True)
    obstacle_pub = rospy.Publisher(prefix + '/obstacles', String, queue_size=1)
    loc_pub = rospy.Publisher(prefix + '/localization', Bool, queue_size=1)
    map_pub = rospy.Publisher(MAP_TOPIC, String, queue_size=1, latch=True)
    pubs = [noisy_pub, goal_pub, obstacle_pub, loc_pub, map_pub]
    heartbeat_stop = threading.Event()
    heartbeat_state = {'obstacles': [], 'map': v1}

    def heartbeat():
        while not heartbeat_stop.is_set():
            with lock:
                raw = shared['raw']
            if raw is not None:
                x = raw[0] + rng.gauss(0.0, args.pose_noise_m)
                y = raw[1] + rng.gauss(0.0, args.pose_noise_m)
                yaw = math.radians(rng.gauss(0.0, args.pose_noise_deg))
                z = raw[2]
                # Support-aware z snap: the planner requires a pose on exactly
                # one declared surface; the carrier can be a few mm high.
                candidates = [surface['z_reference']
                              for surface in heartbeat_state['map']['surfaces']
                              if (surface['footprint']['min_x'] <= x <= surface['footprint']['max_x']
                                  and surface['footprint']['min_y'] <= y <= surface['footprint']['max_y']
                                  and abs(z - surface['z_reference']) <= .02)]
                if candidates:
                    z = min(candidates, key=lambda value: abs(value - z))
                msg = PoseStamped()
                msg.header.frame_id, msg.header.stamp = 'map', rospy.Time.now()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
                noisy_pub.publish(msg)
            obstacle_pub.publish(String(data=json.dumps({'obstacles': heartbeat_state['obstacles']})))
            loc_pub.publish(Bool(data=True))
            heartbeat_stop.wait(.01)

    config = dict(EXPERIMENTAL, resolution=v2['resolution'],
                  clearance_m=args.clearance_m, radius_m=args.radius_m,
                  max_step_m=args.max_step_m)
    pose_topic = prefix + '/noisy_pose'

    def local_argv():
        return [sys.executable, '-B', str(LOCAL / 'ros_node.py'),
                '__name:=' + node_name, '_map_topic:=' + MAP_TOPIC,
                '_pose_topic:=' + pose_topic, '_goal_topic:=' + prefix + '/goal',
                '_obstacles_topic:=' + prefix + '/obstacles',
                '_localization_topic:=' + prefix + '/localization',
                '_command_topic:=' + prefix + '/cmd', '_status_topic:=' + prefix + '/status',
                '_trajectory_topic:=' + prefix + '/trajectory',
                '_experimental_config:=' + json.dumps(config)]
    carrier_argv = [sys.executable, '-B', str(SCRIPTS / 'gazebo_3d_carrier.py'),
                    '--prefix', prefix, '--model', carrier_name, '--start',
                    '%g,%g,%g' % start]
    if args.carrier_snap:
        carrier_argv.extend(['--snap-map', args.map_v2, '--snap-m', '.02'])

    phases = []
    with tempfile.TemporaryDirectory(prefix='livemap_') as directory:
        node_stream = (Path(directory) / 'node.log').open('w')
        carrier_stream = (Path(directory) / 'carrier.log').open('w')
        node = subprocess.Popen(local_argv(), stdout=node_stream, stderr=subprocess.STDOUT)
        carrier = subprocess.Popen(carrier_argv, stdout=carrier_stream, stderr=subprocess.STDOUT)
        heartbeat_thread = threading.Thread(target=heartbeat, daemon=True)
        heartbeat_thread.start()
        link_pose = None
        try:
            deadline = time.monotonic() + args.wall_deadline_s
            while time.monotonic() < deadline and not all(p.get_num_connections() for p in pubs):
                if node.poll() is not None or carrier.poll() is not None:
                    raise RuntimeError('child exited')
                time.sleep(.05)
            if not all(p.get_num_connections() for p in pubs):
                raise RuntimeError('subscription readiness timeout')

            def publish_goal(goal):
                msg = PoseStamped()
                msg.header.frame_id, msg.header.stamp = 'map', rospy.Time.now()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
                msg.pose.orientation.w = 1.0
                goal_pub.publish(msg)

            def wait_arrival(goal, timeout, phase_name, obstacle=None):
                begun = time.monotonic()
                begun_ros = rospy.Time.now().to_sec()
                arrived = False
                sent = False
                while time.monotonic() < begun + timeout and not rospy.is_shutdown():
                    with lock:
                        raw = shared['raw']
                        status = shared['status']
                    if raw is None:
                        time.sleep(.03)
                        continue
                    if not sent:
                        publish_goal(goal)
                        sent = True
                    dx, dy = raw[0] - goal[0], raw[1] - goal[1]
                    if math.hypot(dx, dy) <= .08 and abs(raw[2] - goal[2]) <= .005:
                        arrived = True
                        break
                    if status and status.get('reason') in ('E_LOCAL_NO_ROUTE', 'E_LOCAL_COLLISION'):
                        time.sleep(1.0)
                        break
                    time.sleep(.05)
                with lock:
                    final = shared['raw']
                    plans = shared['plans']
                    last = shared['last_plan']
                    view = [row for row in raw_trace if row[0] >= begun_ros]
                    statuses = [row for row in status_log if row[0] >= begun_ros]
                    commands = [row for row in cmd_log if row[0] >= begun_ros]
                    noisy = [row for row in noisy_log if row[0] >= begun_ros]
                phases.append({'name': phase_name, 'goal': list(goal), 'arrived': arrived,
                               'elapsed_s': round(time.monotonic() - begun, 3),
                               'final': None if final is None else list(final),
                               'plans': plans, 'last_plan': last,
                               'statuses': statuses[-40:], 'trace': view[::5],
                               'commands': commands[-40:], 'noisy': noisy[-10:],
                               'obstacle': obstacle})
                return arrived

            if not args.skip_v1:
                # Publish v1 until the node has it, then run to the partial-map top.
                map_pub.publish(String(data=json.dumps(v1)))
                time.sleep(.5)
                wait_arrival(goal_v1, args.phase_timeout_s, 'v1_partial')
            # Switch (or start) the full-map version, then run to the top.
            with lock:
                shared['trajectories'] = []
            map_pub.publish(String(data=json.dumps(v2)))
            heartbeat_state['map'] = v2
            time.sleep(.5)
            obstacle = None
            if obstacle_payload:
                obstacle = obstacle_payload
            # Inject the obstacle only after the v2 goal has been accepted.
            if obstacle is not None:
                heartbeat_state['obstacles'] = []
            begun_v2 = time.monotonic()
            begun_v2_ros = rospy.Time.now().to_sec()
            arrived_v2 = False
            sent_v2 = False
            obstacle_sent = False
            while time.monotonic() < begun_v2 + args.phase_timeout_s and not rospy.is_shutdown():
                with lock:
                    raw = shared['raw']
                    status = shared['status']
                if raw is None:
                    time.sleep(.03)
                    continue
                if not sent_v2:
                    publish_goal(goal_v2)
                    sent_v2 = True
                if obstacle is not None and not obstacle_sent and \
                        time.monotonic() - begun_v2 >= args.obstacle_after_s:
                    heartbeat_state['obstacles'] = obstacle.get('obstacles', [])
                    obstacle_pub.publish(String(data=json.dumps(obstacle)))
                    obstacle_sent = True
                dx, dy = raw[0] - goal_v2[0], raw[1] - goal_v2[1]
                if math.hypot(dx, dy) <= .08 and abs(raw[2] - goal_v2[2]) <= .005:
                    arrived_v2 = True
                    break
                if status and status.get('reason') in ('E_LOCAL_NO_ROUTE', 'E_LOCAL_COLLISION'):
                    time.sleep(1.0)
                    break
                time.sleep(.05)
            with lock:
                final_v2 = shared['raw']
                plans_v2 = shared['plans']
                last_v2 = shared['last_plan']
                view_v2 = [row for row in raw_trace if row[0] >= begun_v2_ros]
                statuses_v2 = [row for row in status_log if row[0] >= begun_v2_ros]
                commands_v2 = [row for row in cmd_log if row[0] >= begun_v2_ros]
                noisy_v2 = [row for row in noisy_log if row[0] >= begun_v2_ros]
            phases.append({'name': 'v2_full_obstacle' if obstacle else 'v2_full',
                           'goal': list(goal_v2), 'arrived': arrived_v2,
                           'elapsed_s': round(time.monotonic() - begun_v2, 3),
                           'final': None if final_v2 is None else list(final_v2),
                           'plans': plans_v2, 'last_plan': last_v2,
                           'statuses': statuses_v2[-60:], 'trace': view_v2[::5],
                           'commands': commands_v2[-60:], 'noisy': noisy_v2[-10:],
                           'obstacle': obstacle})
            link = get_link(carrier_name + '::link', 'world').link_state.pose.position
            link_pose = [link.x, link.y, link.z]
        finally:
            heartbeat_stop.set()
            for proc in (carrier, node):
                if proc.poll() is None:
                    proc.send_signal(signal.SIGTERM)
            for proc in (carrier, node):
                try:
                    proc.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.)
            node_stream.close()
            carrier_stream.close()
            for obj in callbacks + pubs:
                obj.unregister()
            try:
                delete(carrier_name)
            except rospy.ServiceException:
                pass
        node_text = (Path(directory) / 'node.log').read_text()
        carrier_text = (Path(directory) / 'carrier.log').read_text()

    checks = {
        'v1_global_top_rejected': global_runs['v1_top']['route'] is None,
        'v1_global_goal_ok': global_runs['v1_goal']['route'] is not None,
        'v2_global_top_ok': global_runs['v2_top']['route'] is not None,
        'v1_phase_arrived': args.skip_v1 or any(p['name'] == 'v1_partial' and p['arrived'] for p in phases),
        'v2_phase_arrived': any(p['name'].startswith('v2_full') and p['arrived'] for p in phases),
        'v2_blocked_safely': (not args.expect_block) or (
            any(p['name'].startswith('v2_full') for p in phases)
            and not any(p['name'].startswith('v2_full') and p['arrived'] for p in phases)
            and any(row[1] in ('E_LOCAL_NO_ROUTE', 'E_LOCAL_COLLISION')
                    for row in phases[-1].get('statuses', []))),
        'terrain_loaded': args.terrain in world_models,
        'gazebo_pose_matches': bool(link_pose and phases and phases[-1]['final']
                                    and abs(link_pose[0] - phases[-1]['final'][0]) <= .02
                                    and abs(link_pose[1] - phases[-1]['final'][1]) <= .02
                                    and abs(link_pose[2] - (phases[-1]['final'][2] + .1)) <= .02),
    }
    if args.expect_block:
        checks['v2_phase_arrived'] = True
    ok = all(checks.values())
    payload = {
        'scenario': 'sensor_map_version_noise_obstacle', 'ok': ok, 'checks': checks,
        'map_v1': {'map_id': v1['map_id'], 'map_version': v1['map_version'],
                   'surfaces': len(v1['surfaces']), 'levels': [s['z_reference'] for s in v1['surfaces']],
                   'sha256': v1['source']['sha256']},
        'map_v2': {'map_id': v2['map_id'], 'map_version': v2['map_version'],
                   'surfaces': len(v2['surfaces']), 'levels': [s['z_reference'] for s in v2['surfaces']],
                   'sha256': v2['source']['sha256']},
        'global_runs': global_runs, 'phases': phases, 'start': list(start),
        'goal_v1': list(goal_v1), 'goal_v2': list(goal_v2),
        'noise': {'seed': args.seed, 'xy_m': args.pose_noise_m,
                  'yaw_deg': args.pose_noise_deg},
        'config': config, 'terrain': args.terrain, 'link_pose': link_pose,
        'scope': 'Gazebo ray sensor map, native local planner, kinematic carrier; '
                 'no contact dynamics/no four-leg/no real robot',
        'node_log': node_text[-4000:], 'carrier_log': carrier_text[-2000:],
    }
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n',
                              encoding='utf-8')
    print(json.dumps({'ok': ok, 'checks': checks,
                      'phases': [(p['name'], p['arrived'], p['plans']) for p in phases]},
                     ensure_ascii=False))
    return 0 if ok else 6


if __name__ == '__main__':
    raise SystemExit(main())
