#!/usr/bin/env python3
"""Real ROS smoke: semantic -> support -> planner through the v2 graph.

The graph selects the semantic provider, the bridge and the native planner; the
smoke starts every instance through ProcessRunner with its rendered command.
A semantic version bump must reach the planner as a new support-map version and
change the planned height.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent / 'framework'
FIXTURE = FRAMEWORK / 'fixtures' / 'maps' / 'sem_support.json'
OUT = Path('/tmp/r1026_smoke.json')
PORT = 11505
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1026_ros',
                  ROS_LOG_DIR='/tmp/r1026_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1026_ros').mkdir(exist_ok=True)
Path('/tmp/r1026_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import safety  # noqa: E402


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    graph = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs'
                              / 'graph_semantic_planning.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    provider = next(item for item in plan['instances'] if item['id'] == 'semantic')
    provider['params']['semantic_files'] = str(FIXTURE)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers'],
              'config_sha256': report['configs']['local']['sha256']}
    try:
        safety.check_plan(plan, catalog)
        result['safety_gate'] = 'ok'
    except config.FrameworkError as error:
        result['safety_gate'] = error.code
    base = json.loads(FIXTURE.read_text())
    raised = json.loads(json.dumps(base))
    raised['map_version'] = 2
    mezz = next(item for item in raised['features'] if item['id'] == 'mezz')
    mezz['geometry']['min'][2] = 0.75
    mezz['geometry']['max'][2] = 0.8
    for feature in raised['features']:
        for edge in (feature.get('attributes') or {}).get('edges', []):
            edge['max_step_height_m'] = 1.0
    result['fixture_map_version'] = base['map_version']
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1026_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runners = {}
    specs = {}
    running = threading.Event()
    try:
        time.sleep(3)
        rospy.init_node('r1026_smoke', anonymous=True, disable_signals=True)
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
            time.sleep(.5)
        result['commands'] = commands
        result['local_uses_param_exec'] = commands['local'][1].endswith('param_exec.py')
        pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
        goal_pub = rospy.Publisher('/v8/local/goal', PoseStamped, queue_size=2, latch=True)
        obstacles_pub = rospy.Publisher('/v8/local/obstacles', String, queue_size=2)
        localization_pub = rospy.Publisher('/v8/local/localization_ok', Bool, queue_size=2)
        semantic_pub = rospy.Publisher('/v8/semantics/sem_support', String,
                                       queue_size=1, latch=True)
        feed = {'pose': (3.0, 2.0, .6), 'goal': (3.5, 2.0, .6)}
        lock = threading.Lock()
        running.set()

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        def feed_loop():
            sent_goal = None
            obstacles = String(data='[]')
            localization = Bool(data=True)
            while running.is_set() and not rospy.is_shutdown():
                with lock:
                    pose, goal = feed['pose'], feed['goal']
                pose_pub.publish(signed(*pose))
                if goal != sent_goal:
                    goal_pub.publish(signed(*goal))
                    sent_goal = goal
                obstacles_pub.publish(obstacles)
                localization_pub.publish(localization)
                time.sleep(.1)

        thread = threading.Thread(target=feed_loop, daemon=True)
        thread.start()
        bridge_status = {'value': None}
        rospy.Subscriber('/v8/support_map_sem/status', String,
                         lambda message: bridge_status.update(
                             value=json.loads(message.data)), queue_size=1)

        def wait_trajectory(version, z, timeout=25):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    payload = json.loads(rospy.wait_for_message('/v8/local/trajectory',
                                                                String, timeout=2).data)
                except Exception:
                    continue
                if payload.get('map_version') == version and \
                        abs(payload['trajectory'][-1][2] - z) < 1e-6:
                    return payload
            return None

        first = wait_trajectory(1, .6)
        result['first'] = None if first is None else {
            'map_version': first['map_version'], 'reason': first['reason'],
            'last': first['trajectory'][-1], 'surfaces': sorted(set(first['surfaces']))}
        result['bridge_initial'] = bridge_status['value']
        time.sleep(.5)
        semantic_pub.publish(String(data=json.dumps(raised, ensure_ascii=False)))
        with lock:
            feed.update(pose=(3.0, 2.0, .8), goal=(3.5, 2.0, .8))
        second = wait_trajectory(2, .8)
        result['second'] = None if second is None else {
            'map_version': second['map_version'], 'reason': second['reason'],
            'last': second['trajectory'][-1], 'surfaces': sorted(set(second['surfaces']))}
        result['bridge_after'] = bridge_status['value']
        result['changed'] = bool(first is not None and second is not None and
                                 first['trajectory'] != second['trajectory'])
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
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('semantic_map_node.py' in row[2]
                        or 'semantic_support_bridge_node.py' in row[2]
                        or 'ros_node.py' in row[2])
                    and 'r1026_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('safety_gate') == 'E_SAFETY_ADVICE'
                        and result.get('first') is not None
                        and result['first']['reason'] == 'arrived'
                        and result.get('second') is not None
                        and result['second']['reason'] == 'arrived'
                        and result.get('changed')
                        and result.get('bridge_after', {}).get('map_version') == 2
                        and result.get('local_uses_param_exec')
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
