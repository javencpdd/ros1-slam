#!/usr/bin/env python3
"""ROS local node -> V8-X1 270 policy on a flat strip, with a sensed blocked goal.

The real local_node plans in the map frame; the bridge maps map velocity to the
MuJoCo body and returns the highest declared support.  A static obstacle is
injected at the task goal while the robot is moving; the node must return
E_LOCAL_NO_ROUTE, the bridge must stop commanding velocity, and the physical
body must come to rest without collision.  This is a full-chain (ROS) stop
pilot, not a formal #151 acceptance.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOCAL = HERE.parent / 'local_planner'
DYNAMICS = HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'dynamics'
sys.path.insert(0, str(DYNAMICS))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'local_planner'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import support_planner  # noqa: E402

PORT = 11533
SCOPE = 'ros_local_node_45policy_mujoco_flat_sensor_path_or_goal_stop_pilot'


def map_value(goal_y):
    surface = {'surface_id': 'floor', 'z_reference': 0., 'data_uri': 'fixture://flat_stop_strip',
               'footprint': {'min_x': -.5, 'max_x': .5, 'min_y': -3., 'max_y': 3.},
               'thickness_m': .05, 'edges': [], 'representation': 'elevation_grid',
               'traversability': 'supported', 'clearance_min_m': .4}
    source = {'uri': 'fixture://flat_stop_map',
              'sha256': hashlib.sha256(b'flat stop map x[-.5,.5] y[-3,3]').hexdigest()}
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value.update(map_id='mj_flat_stop_ros', resolution=.1, origin=[-.5, -3., 0.],
                 extent={'width': 10, 'height': 60}, surfaces=[surface], source=source,
                 multiple_surfaces_per_xy=False)
    support_planner.contract.validate('map', value)
    return value


def obstacle():
    return {'min': [-.4, .8, 0.], 'max': [.4, 1.2, .5],
            'stamp': 0., 'velocity': [0., 0., 0.]}

def sensor_xml(xml):
    root = ET.parse(Path(xml).resolve()).getroot()
    root.set('model', 'stop_sensor_world')
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    for key in ('meshdir', 'texturedir'):
        compiler.set(key, str((Path(xml).resolve().parent / compiler.get(key, '')).resolve()))
    world = root.find('worldbody')
    if world is None:
        world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='perception_barrier', type='box',
                  pos='3.2 0 0.5', size='0.05 1 0.5', rgba='0.8 0.2 0.1 1')
    return ET.tostring(root, encoding='unicode')


def path_obstacle():
    return {'min': [-.4, .45, 0.], 'max': [.4, .55, .5],
            'stamp': 0., 'velocity': [0., 0., 0.]}


def start(argv, stream, cwd=None, env=None):
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                            start_new_session=True, cwd=cwd, env=env)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.)


def interpolate(samples, stamp, key):
    if not samples:
        return None
    if stamp <= samples[0]['sim_time']:
        return list(samples[0][key])
    if stamp >= samples[-1]['sim_time']:
        return list(samples[-1][key])
    for a, b in zip(samples, samples[1:]):
        if a['sim_time'] <= stamp <= b['sim_time']:
            span = b['sim_time'] - a['sim_time']
            u = 0. if span <= 0 else (stamp - a['sim_time']) / span
            return [a[key][i] + u * (b[key][i] - a[key][i]) for i in range(len(a[key]))]
    return list(samples[-1][key])


def measure(samples, trigger, hold_s=.5, speed_limit=.2, xy_limit=.05):
    """Position/hold stop measurement on sim-time applied records."""
    result = {'trigger_sim': trigger, 'completed': False, 'status': 'INVALID',
              'reason': 'trigger_not_observed', 'stop_time_s': None,
              'peak_forward_m': None, 'path_m': None, 'first_hold_sim': None,
              'confirmation_sim': None}
    if trigger is None or len(samples) < 2:
        return result
    xy = [s['world_xyz'][:2] for s in samples]
    times = [s['sim_time'] for s in samples]
    if not times[0] < trigger <= times[-1]:
        return result
    origin = interpolate(samples, trigger, 'world_xyz')[:2]
    result['reason'] = 'stop_not_completed'
    result['status'] = 'FAIL'
    for index, sample in enumerate(samples):
        start_sim = sample['sim_time']
        if start_sim < trigger or start_sim + hold_s > times[-1]:
            continue
        end_sim = start_sim + hold_s
        inside = [(t, p) for t, p in zip(times, xy) if start_sim <= t <= end_sim]
        end_xy = interpolate(samples, end_sim, 'world_xyz')[:2]
        inside.append((end_sim, end_xy))
        span_x = max(p[0] for _, p in inside) - min(p[0] for _, p in inside)
        span_y = max(p[1] for _, p in inside) - min(p[1] for _, p in inside)
        velocities = [math.hypot(*s.get('base_vxy', [0., 0.]))
                      for s in samples if start_sim <= s['sim_time'] <= end_sim]
        if span_x <= xy_limit and span_y <= xy_limit and max(velocities or [0.]) <= speed_limit:
            after = [s for s in samples if s['sim_time'] > end_sim]
            if any(math.hypot(*s.get('base_vxy', [0., 0.])) > speed_limit for s in after):
                continue
            if any(math.hypot(s['world_xyz'][0] - end_xy[0],
                              s['world_xyz'][1] - end_xy[1]) > 2. * xy_limit for s in after):
                continue
            result.update(completed=True, status='PASS', reason='stationary_window_observed',
                          first_hold_sim=start_sim, confirmation_sim=end_sim)
            break
    if result['completed']:
        end_sim = result['confirmation_sim']
        end_xy = interpolate(samples, end_sim, 'world_xyz')[:2]
        measured = [origin] + [s['world_xyz'][:2] for s in samples
                               if trigger < s['sim_time'] <= end_sim]
        if not measured or measured[-1] != end_xy:
            measured.append(end_xy)
        peak = max([p[0] - origin[0] for p in measured] or [0.])
        steps = [math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(measured, measured[1:])]
        result.update(peak_forward_m=float(max(0., peak)),
                      path_m=float(sum(steps)),
                      stop_time_s=float(end_sim - trigger))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', default='/tmp/mjcf/Lite3.xml')
    parser.add_argument('--policy', default='/tmp/new_policy.onnx')
    parser.add_argument('--bridge45', action='store_true')
    parser.add_argument('--bridge-kin', action='store_true')
    parser.add_argument('--sensor-bridge', action='store_true')
    parser.add_argument('--perturb', action='store_true')
    parser.add_argument('--out', required=True)
    parser.add_argument('--max-speed', type=float, default=.3)
    parser.add_argument('--resolution', type=float, default=.05)
    parser.add_argument('--tick-s', type=float, default=1.2)
    parser.add_argument('--path-obstacle', action='store_true')
    parser.add_argument('--sensor-range', type=float, default=1.5)
    parser.add_argument('--start-map-y', type=float, default=-2.175)
    parser.add_argument('--inject-map-y', type=float, default=-.2)
    parser.add_argument('--goal-y', type=float, default=1.0)
    parser.add_argument('--speed-scale', type=float, default=1.0)
    parser.add_argument('--wall-deadline-s', type=float, default=60.)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--hold-s', type=float, default=.5)
    parser.add_argument('--speed-limit', type=float, default=.2)
    parser.add_argument('--xy-limit', type=float, default=.05)
    parser.add_argument('--out-dir', default='')
    parser.add_argument('--port', type=int, default=0)
    args = parser.parse_args()
    if not (0 < args.max_speed <= .5):
        parser.error('max-speed must be in (0,.5]')
    if not args.start_map_y < args.inject_map_y < args.goal_y:
        parser.error('require start-map-y < inject-map-y < goal-y')

    port = args.port or (PORT + os.getpid() % 1000)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1046_ros',
                      ROS_LOG_DIR='/tmp/r1046_logs', PYTHONDONTWRITEBYTECODE='1')
    Path('/tmp/r1046_ros').mkdir(exist_ok=True)
    Path('/tmp/r1046_logs').mkdir(exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='a5stop_', dir=args.out_dir or '/tmp'))
    map_path = work / 'map.json'
    map_path.write_text(json.dumps(map_value(args.goal_y)))
    bridge_xml = args.xml
    if args.sensor_bridge:
        sensor_path = work / 'sensor.xml'
        sensor_path.write_text(sensor_xml(args.xml))
        bridge_xml = str(sensor_path)
    master_log = work / 'master.log'
    node_log = work / 'node.log'
    bridge_log = work / 'bridge.log'

    master = start(['roscore', '-p', str(port)], master_log.open('ab'))
    node = bridge = None
    body, applied, statuses, obstacle_events = [], [], [], []
    pose_seen = {'count': 0, 'last': None, 'frame': None}
    result = {'scope': SCOPE, 'no_ros': False, 'hardware': False,
              'formal_151_acceptance': False, 'verdict': 'INVALID', 'checks': {},
              'run_config': {'max_speed_mps': args.max_speed,
                             'start_map_y': args.start_map_y,
                             'inject_map_y': args.inject_map_y,
                             'goal_y': args.goal_y,
                             'speed_scale': args.speed_scale,
              'bridge45': bool(args.bridge45),
              'bridge_kin': bool(args.bridge_kin),
              'resolution_m': args.resolution,
              'seed': args.seed,
              'perturb': bool(args.perturb),
                             'stop_hold_s': args.hold_s,
                             'stop_speed_limit_mps': args.speed_limit,
                             'stop_xy_limit_m': args.xy_limit,
              'path_obstacle': bool(args.path_obstacle),
              'sensor_range_m': args.sensor_range,
              'sensor_bridge': bool(args.sensor_bridge),
                             'policy': args.policy, 'xml': args.xml}}
    try:
        time.sleep(3.)
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String
        rospy.init_node('a5_v8x1_stop_smoke', anonymous=True, disable_signals=True)
        node_name = 'a5stop_%d' % os.getpid()
        prefix = '/' + node_name
        config = {'tick_s': args.tick_s, 'horizon_s': 12., 'validity_s': 12.,
                  'clearance_m': .1, 'max_speed_mps': args.max_speed, 'resolution': args.resolution}
        node_argv = [sys.executable, '-u', '-B', str(LOCAL / 'ros_node.py'),
                     '__name:=' + node_name,
                     '_map_json:=' + str(map_path),
                     '_pose_topic:=' + prefix + '/pose',
                     '_goal_topic:=' + prefix + '/goal',
                     '_obstacles_topic:=' + prefix + '/stop_obstacles',
                     '_localization_topic:=' + prefix + '/localization',
                     '_command_topic:=' + prefix + '/cmd',
                     '_status_topic:=' + prefix + '/status',
                     '_trajectory_topic:=' + prefix + '/trajectory',
                     '_experimental_config:=' + json.dumps(config)]
        if args.bridge_kin:
            bridge_argv = [sys.executable, '-u', '-B',
                           str(DYNAMICS / 'a5_kinematic_stop_bridge.py'),
                           '--map', str(map_path), '--prefix', prefix,
                           '--start-map-y', str(args.start_map_y),
                           '--cmd-timeout', '.5']
        elif args.bridge45:
            bridge_argv = [sys.executable, '-u', '-B', str(DYNAMICS / 'a5_v8x1_stop_bridge45.py'),
                           '--map', str(map_path), '--xml', bridge_xml, '--policy', args.policy,
                           '--prefix', prefix, '--start-map-y', str(args.start_map_y),
                           '--speed-scale', str(args.speed_scale), '--max-command', '.5',
                           '--cmd-timeout', '.5', '--initial-z', '.35', '--seed', str(args.seed)]
            if args.perturb:
                bridge_argv.append('--perturb')
            if args.sensor_bridge:
                bridge_argv.extend(['--sense', '--sensor-range', str(args.sensor_range)])
        else:
            bridge_argv = [sys.executable, '-u', '-B', str(DYNAMICS / 'a5_v8x1_bridge.py'),
                           '--map', str(map_path), '--xml', args.xml, '--policy', args.policy,
                           '--prefix', prefix, '--start-map-y', str(args.start_map_y),
                           '--speed-scale', str(args.speed_scale), '--max-command', '.45',
                           '--cmd-timeout', '.5', '--seed', str(args.seed)]
        node = start(node_argv, node_log.open('ab'), cwd=str(LOCAL),
                     env=dict(os.environ, PYTHONPATH=str(DYNAMICS) + ':' +
                              str(HERE.parent / 'planner') + ':' +
                              str(HERE.parent / 'local_planner') + ':' +
                              str(HERE.parent / 'contracts') + ':' +
                              os.environ.get('PYTHONPATH', '')))
        bridge = start(bridge_argv, bridge_log.open('ab'), cwd=str(DYNAMICS))
        goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1, latch=True)
        obstacle_pub = rospy.Publisher(prefix + '/stop_obstacles', String, queue_size=1)
        loc_pub = rospy.Publisher(prefix + '/localization', Bool, queue_size=1)
        injected = {'wall': None, 'sim': None}
        lock = __import__('threading').Lock()

        def on_body(message):
            with lock:
                p = message.pose.position
                body.append({'wall': message.header.stamp.to_sec(), 'xyz': [p.x, p.y, p.z]})

        def on_applied(message):
            try:
                payload = json.loads(message.data)
            except ValueError:
                return
            with lock:
                payload['wall'] = time.monotonic()
                applied.append(payload)

        def on_status(message):
            try:
                payload = json.loads(message.data)
            except ValueError:
                return
            with lock:
                statuses.append({'wall': rospy.Time.now().to_sec(), **payload})

        def on_pose(message):
            pose_seen['count'] += 1
            pose_seen['last'] = [message.pose.position.x, message.pose.position.y,
                                 message.pose.position.z]
            pose_seen['frame'] = message.header.frame_id

        rospy.Subscriber(prefix + '/pose', PoseStamped, on_pose, queue_size=20)
        rospy.Subscriber(prefix + '/body', PoseStamped, on_body, queue_size=200)
        rospy.Subscriber(prefix + '/applied', String, on_applied, queue_size=200)
        rospy.Subscriber(prefix + '/status', String, on_status, queue_size=100)
        deadline = time.monotonic() + 30.
        while time.monotonic() < deadline and not all(
                p.get_num_connections() for p in (goal_pub, obstacle_pub, loc_pub)):
            if node.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('child exited: ' + node_log.read_text() +
                                   bridge_log.read_text())
            time.sleep(.05)
        if not all(p.get_num_connections() for p in (goal_pub, obstacle_pub, loc_pub)):
            raise RuntimeError('subscription readiness timeout')
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 0., args.goal_y, 0.
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
        loc_pub.publish(Bool(data=True))
        end = time.monotonic() + args.wall_deadline_s
        latched = None
        while time.monotonic() < end and not rospy.is_shutdown():
            with lock:
                pose = None
                if applied:
                    pose = applied[-1].get('map_xyz')
                moving = applied[-1].get('base_vxy') if applied else None
            if args.sensor_bridge:
                pass
            elif args.path_obstacle:
                if injected['wall'] is None and pose is not None:
                    # Forward ray from the body: the obstacle is published only
                    # after the sensor range reaches it, not at a scheduled time.
                    origin_y = pose[1] + .45
                    distance = .45 - origin_y
                    if 0. <= distance <= args.sensor_range:
                        injected['wall'] = time.monotonic()
                        obstacle_events.append({
                            'event': 'perception_hit', 'wall': injected['wall'],
                            'map_y': pose[1], 'ray_distance_m': distance,
                            'sensor_range_m': args.sensor_range})
                if injected['wall'] is None:
                    obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
                else:
                    obstacle_pub.publish(String(data=json.dumps(
                        {'obstacles': [path_obstacle()]})))
            else:
                if injected['wall'] is None and pose is not None and pose[1] >= args.inject_map_y:
                    injected['wall'] = time.monotonic()
                    obstacle_events.append({'event': 'obstacle_publish', 'wall': injected['wall'],
                                            'map_y': pose[1]})
                if injected['wall'] is None:
                    obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
                else:
                    obstacle_pub.publish(String(data=json.dumps({'obstacles': [obstacle()]})))
            loc_pub.publish(Bool(data=True))
            with lock:
                if statuses:
                    last = statuses[-1]
                    if last.get('latched') or last.get('reason') == 'E_LOCAL_NO_ROUTE':
                        latched = last
                        break
            time.sleep(.05)
        if latched is not None:
            time.sleep(2.5)
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        for process in (bridge, node):
            if process is not None:
                stop(process)
        stop(master)
        time.sleep(.3)
    with lock if 'lock' in locals() else __import__('contextlib').nullcontext():
        body_view = list(body)
        applied_view = list(applied)
        status_view = list(statuses)
    if args.sensor_bridge and injected['wall'] is None:
        hit = next((s for s in applied_view if s.get('sensor_hit')), None)
        if hit is not None:
            injected['wall'] = hit.get('wall')
            obstacle_events.append({'event': 'sensor_hit', 'wall': hit.get('wall'),
                                    'sim_time': hit.get('sim_time')})
    result['events'] = obstacle_events
    result['pose_count'] = pose_seen['count']
    result['pose_last'] = pose_seen['last']
    result['pose_frame'] = pose_seen['frame']
    result['statuses'] = status_view[:200]
    result['body_trace'] = body_view[::5]
    result['applied'] = applied_view
    result['node_log'] = node_log.read_text() if node_log.exists() else ''
    result['bridge_log'] = bridge_log.read_text() if bridge_log.exists() else ''
    # Measurement: first applied record after the injection wall time.
    trigger = None
    if injected['wall'] is not None and applied_view:
        for sample in applied_view:
            if sample.get('wall', 0.) >= injected['wall']:
                trigger = sample.get('sim_time')
                break
    if trigger is None and injected['wall'] is not None and applied_view:
        trigger = applied_view[-1].get('sim_time')
    measurement = measure(applied_view, trigger, args.hold_s, args.speed_limit, args.xy_limit)
    result['measurement'] = measurement
    result['first_zero_status'] = next(
        (s for s in status_view if s.get('reason') == 'E_LOCAL_NO_ROUTE'), None)
    final = applied_view[-1] if applied_view else None
    final_xyz = final.get('world_xyz') if final else None
    # Barrier world x range: map y .8—1.2 + y_offset 2.7.
    barrier_face = 3.15 if args.path_obstacle else 3.5
    gap = None if final_xyz is None else barrier_face - (final_xyz[0] + .25)
    zmin = min([s['xyz'][2] for s in body_view] or [0.])
    checks = {
        'moving_observed': bool(applied_view and max(
            [math.hypot(*s.get('base_vxy', [0., 0.])) for s in applied_view
             if trigger is None or s.get('sim_time', 1e9) <= trigger] or [0.]) > .02),
        'obstacle_injected_while_moving': bool(injected['wall'] is not None and
                                               trigger is not None),
        'planner_no_route': bool(result['first_zero_status']),
        'stop_completed': bool(measurement['completed']),
        'no_fall': bool(body_view and zmin > .12),
        'collision_free': bool(gap is not None and gap > 0.),
        'sensing_watchdog_ok': bool(applied_view and status_view),
    }
    result['checks'] = checks
    result['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    result['gap_m'] = gap
    result['zmin'] = zmin
    result['trigger_sim'] = trigger
    result['final_world_xyz'] = final_xyz
    result['scope_note'] = ('ROS local node + MuJoCo/45 policy; flat support map; '
                            + ('sensor-triggered path obstacle' if args.path_obstacle else 'static obstacle at goal')
                            + '; full-chain pilot, not formal #151')
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'verdict': result['verdict'], 'checks': checks,
                      'measurement': measurement, 'gap_m': gap}, ensure_ascii=False))
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
