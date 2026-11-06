#!/usr/bin/env python3
"""Assembly closed loop: 3D global path -> path goal -> local -> 3D mux -> carrier.

This is a deterministic kinematic 3D closed loop, not Gazebo physics or a robot.
The map/odom inputs are the same declared fixtures used by R1040; the new part
is that Assembly starts the whole graph with a real 3D command arbiter.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import safety  # noqa: E402
import ros_map_codec  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/tmp/r1097_3d_assembly.json')
    parser.add_argument('--port', type=int, default=11527)
    parser.add_argument('--state', default='/tmp/r1097_3d_state.json')
    parser.add_argument('--map', default=str(ROOT / 'contracts' / 'fixtures'
                                            / 'map_ml_two_surfaces.json'))
    parser.add_argument('--graph', default=str(HERE / 'fixtures' / 'graphs'
                                              / 'graph_3d_real.yaml'))
    parser.add_argument('--start', default='3.0,2.0,0.6')
    parser.add_argument('--goal', default='3.5,2.0,0.6')
    parser.add_argument('--timeout-s', type=float, default=25.)
    args = parser.parse_args()
    map_path = Path(args.map).resolve()
    start = tuple(float(v) for v in args.start.split(','))
    goal = tuple(float(v) for v in args.goal.split(','))
    output = Path(args.out).resolve()
    state_path = Path(args.state).resolve()
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(Path(args.graph).resolve())
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    provider = next(item for item in plan['instances'] if item['id'] == 'provider')
    provider['params']['map_json'] = str(map_path)
    provider['params']['state_path'] = str(state_path)
    carrier = next(item for item in plan['instances'] if item['id'] == 'carrier')
    carrier['params'].update(start_x=start[0], start_y=start[1], start_z=start[2])
    runtime.execution.check_plan(plan, catalog, config.load_yaml)
    if state_path.exists():
        state_path.unlink()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                      ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1097_ros',
                      ROS_LOG_DIR='/tmp/r1097_logs', PYTHONDONTWRITEBYTECODE='1')
    Path('/tmp/r1097_ros').mkdir(exist_ok=True)
    Path('/tmp/r1097_logs').mkdir(exist_ok=True)
    result = {'scope': 'assembly_3d_global_local_kinematic_closed_loop',
              'map': str(map_path), 'start': list(start), 'goal': list(goal),
              'safety_gate': None, 'command_writers': report['command_writers'],
              'startup_order': report['startup_order'], 'path': None,
              'local_trajectory': None, 'carrier_final': None, 'mux': {},
              'error': None, 'assembly_health': None, 'stopped': False}
    try:
        safety.check_plan(plan, catalog)
        result['safety_gate'] = 'ok'
    except config.FrameworkError as error:
        result['safety_gate'] = error.code
        output.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 6
    master = subprocess.Popen(['roscore', '-p', str(args.port)],
                              stdout=open('/tmp/r1097_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        from v8_msgs.msg import SupportMap
        rospy.init_node('r1097_assembly_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=30.0, run_timeout=120.0)
        assembly.start()
        result['assembly_ready'] = assembly.ready()
        pub_a = rospy.Publisher('/v8/3d/base_a', SupportMap, queue_size=1, latch=True)
        pub_b = rospy.Publisher('/v8/3d/base_b', SupportMap, queue_size=1, latch=True)
        value = json.loads(map_path.read_text())
        support = ros_map_codec.support_map_msg(value)
        time.sleep(.8)
        pub_a.publish(support)
        pub_b.publish(support)
        time.sleep(2.0)
        for name in ('/v8/3d/cache_a/v8_map_cache/flush',
                     '/v8/3d/cache_b/v8_map_cache/flush'):
            rospy.wait_for_service(name, timeout=20)
            result[name] = rospy.ServiceProxy(name, Trigger)().success
        status = None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            raw = rospy.wait_for_message('/v8_map_status', String, timeout=5)
            status = json.loads(raw.data)
            if status.get('map_version') == 3:
                break
        result['provider_status'] = None if status is None else {
            'map_version': status.get('map_version'),
            'deltas': status.get('applied_deltas')}
        start_pub = rospy.Publisher('/v8/3d/global/start', PoseStamped, queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/v8/3d/global/goal', PoseStamped, queue_size=1, latch=True)
        obstacle_pub = rospy.Publisher('/v8/local/obstacles', String, queue_size=2)
        hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=2, latch=True)
        path = {'value': None}
        trajectory = {'value': None}
        status_log = []
        mux_log = []
        pose = {'value': None}
        rospy.Subscriber('/v8/3d/global/path', RosPath,
                         lambda message: path.update(value=message), queue_size=2)
        rospy.Subscriber('/v8/local/trajectory', String,
                         lambda message: trajectory.update(value=json.loads(message.data)), queue_size=5)
        rospy.Subscriber('/v8/3d/mux/status', String,
                         lambda message: mux_log.append(json.loads(message.data)), queue_size=200)
        rospy.Subscriber('/v8/local/pose', PoseStamped,
                         lambda message: pose.update(value=[message.pose.position.x,
                                                            message.pose.position.y,
                                                            message.pose.position.z]), queue_size=50)
        rospy.Subscriber('/v8/local/status', String,
                         lambda message: status_log.append(json.loads(message.data)), queue_size=200)

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        time.sleep(.5)
        hold_pub.publish(Bool(data=False))
        start_pub.publish(signed(*start))
        goal_pub.publish(signed(*goal))
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            obstacle_pub.publish(String(data='[]'))
            hold_pub.publish(Bool(data=False))
            if trajectory['value'] and trajectory['value'].get('reason') == 'arrived':
                if pose['value'] and math.dist(pose['value'], goal) <= .10:
                    break
            if assembly.health():
                result['assembly_health'] = assembly.health()
                break
            time.sleep(.1)
        result['path'] = None if path['value'] is None else {
            'poses': len(path['value'].poses),
            'last': [path['value'].poses[-1].pose.position.x,
                     path['value'].poses[-1].pose.position.y,
                     path['value'].poses[-1].pose.position.z]}
        result['local_trajectory'] = None if trajectory['value'] is None else {
            'map_version': trajectory['value'].get('map_version'),
            'reason': trajectory['value'].get('reason'),
            'last': trajectory['value'].get('trajectory', [None])[-1]}
        result['carrier_final'] = pose['value']
        result['carrier_distance_to_goal_m'] = (None if pose['value'] is None else
                                                math.dist(pose['value'], goal))
        result['mux'] = {'samples': len(mux_log),
                         'track_samples': sum(1 for row in mux_log if row.get('reason') == 'track'),
                         'stale_samples': sum(1 for row in mux_log if row.get('reason') == 'E_CMD3D_STALE'),
                         'last': mux_log[-1] if mux_log else None}
        result['local_status_samples'] = len(status_log)
        result['local_status_tail'] = status_log[-12:]
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        if assembly is not None:
            try:
                assembly.stop()
            except Exception as error:
                result['stop_error'] = repr(error)
            result['stopped'] = not assembly.started
            result['stop_record'] = assembly.stop_record
            result['assembly_health'] = result['assembly_health'] or assembly.health()
        try:
            os.killpg(master.pid, signal.SIGTERM)
            master.wait(timeout=10)
        except Exception:
            pass
    result['ok'] = (result.get('safety_gate') == 'ok'
                    and (result.get('provider_status') or {}).get('map_version') is not None
                    and result.get('path') is not None
                    and (result.get('local_trajectory') or {}).get('reason') == 'arrived'
                    and result.get('carrier_distance_to_goal_m') is not None
                    and result['carrier_distance_to_goal_m'] <= .10
                    and (result.get('mux') or {}).get('track_samples', 0) > 0
                    and result.get('error') is None
                    and result.get('stopped'))
    output.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({k: result[k] for k in ('ok', 'safety_gate', 'path',
                                             'local_trajectory', 'carrier_final',
                                             'carrier_distance_to_goal_m', 'mux',
                                             'error')}, ensure_ascii=False))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
