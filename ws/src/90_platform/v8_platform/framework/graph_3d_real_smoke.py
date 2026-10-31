#!/usr/bin/env python3
"""Real ROS smoke: real caches -> merge -> relay -> map provider -> 3D planners."""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = Path('/tmp/r1040_smoke.json')
STATE = Path('/tmp/r1040_state.json')
PORT = 11526
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1040_ros',
                  ROS_LOG_DIR='/tmp/r1040_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1040_ros').mkdir(exist_ok=True)
Path('/tmp/r1040_logs').mkdir(exist_ok=True)
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
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from v8_msgs.msg import SupportMap
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_3d_real.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    provider = next(item for item in plan['instances'] if item['id'] == 'provider')
    provider['params']['map_json'] = str(ROOT / 'contracts' / 'fixtures'
                                         / 'map_ml_two_surfaces.json')
    provider['params']['state_path'] = str(STATE)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers']}
    try:
        safety.check_plan(plan, catalog)
        result['safety_gate'] = 'ok'
    except config.FrameworkError as error:
        result['safety_gate'] = error.code
    if STATE.exists():
        STATE.unlink()
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1040_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runners, specs = {}, {}
    running = threading.Event()
    try:
        time.sleep(3)
        rospy.init_node('r1040_smoke', anonymous=True, disable_signals=True)
        specs = {item.id: item for item in runtime.build_specs(plan, catalog)}
        commands = {}
        for instance in plan['instances']:
            iid = instance['id']
            command, marker = runtime.default_command(
                instance, catalog['components'][instance['component']])
            commands[iid] = command[:2]
            runner = runtime.ProcessRunner(specs[iid], command, ready_marker=marker)
            runner.start()
            runners[iid] = runner
            time.sleep(.4)
        result['commands'] = commands
        value = json.loads((ROOT / 'contracts' / 'fixtures'
                            / 'map_ml_two_surfaces.json').read_text())
        support = ros_map_codec.support_map_msg(value)
        pub_a = rospy.Publisher('/v8/3d/base_a', SupportMap, queue_size=1, latch=True)
        pub_b = rospy.Publisher('/v8/3d/base_b', SupportMap, queue_size=1, latch=True)
        time.sleep(1.0)
        pub_a.publish(support)
        pub_b.publish(support)
        time.sleep(1.0)
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
        start_pub = rospy.Publisher('/v8/3d/global/start', PoseStamped,
                                    queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/v8/3d/global/goal', PoseStamped,
                                   queue_size=1, latch=True)
        local_pose = rospy.Publisher('/v8/local/pose', PoseStamped,
                                     queue_size=2, latch=True)
        local_goal = rospy.Publisher('/v8/local/goal', PoseStamped,
                                     queue_size=2, latch=True)
        obstacles = rospy.Publisher('/v8/local/obstacles', String,
                                    queue_size=2, latch=True)
        localization = rospy.Publisher('/v8/local/localization_ok', Bool,
                                       queue_size=2, latch=True)
        path = {'value': None}
        trajectory = {'value': None}
        rospy.Subscriber('/v8/3d/global/path', Path,
                         lambda message: path.update(value=message), queue_size=2)
        rospy.Subscriber('/v8/local/trajectory', String,
                         lambda message: trajectory.update(
                             value=json.loads(message.data)), queue_size=2)

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        time.sleep(.5)
        start_pub.publish(signed(3.0, 2.0, .6))
        goal_pub.publish(signed(3.5, 2.0, .6))
        local_pose.publish(signed(3.0, 2.0, .6))
        local_goal.publish(signed(3.5, 2.0, .6))
        obstacles.publish(String(data='[]'))
        localization.publish(Bool(data=True))
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and path['value'] is None:
            time.sleep(.05)
        result['path'] = None if path['value'] is None else {
            'poses': len(path['value'].poses),
            'last': [path['value'].poses[-1].pose.position.x,
                     path['value'].poses[-1].pose.position.y,
                     path['value'].poses[-1].pose.position.z]}
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and trajectory['value'] is None:
            time.sleep(.05)
        result['trajectory'] = None if trajectory['value'] is None else {
            'map_version': trajectory['value'].get('map_version'),
            'reason': trajectory['value'].get('reason'),
            'last': trajectory['value']['trajectory'][-1]}
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        running.clear()
        for iid in reversed(plan['startup_order']):
            runner = runners.get(iid)
            if runner is None:
                continue
            runner.stop(specs[iid].stop_method)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and not runner.is_stopped():
                time.sleep(.1)
            if not runner.is_stopped():
                runner.force()
        result['stopped'] = bool(runners) and all(item.is_stopped()
                                                  for item in runners.values())
        os.killpg(master.pid, signal.SIGTERM)
        master.wait(timeout=10)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        names = ('map_cache_node.py', 'map_merge_node.py', 'map_delta_relay.py',
                 'delta_map_node.py', 'topic_node.py', 'ros_node.py')
        leftover = [row[2] for row in rows
                    if any(name in row[2] for name in names)
                    and 'r1040_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('safety_gate') == 'E_SAFETY_ADVICE'
                        and result.get('provider_status', {}).get('map_version') == 3
                        and result.get('path') is not None
                        and result.get('trajectory', {}).get('map_version') == 3
                        and result['trajectory'].get('reason') == 'arrived'
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
