#!/usr/bin/env python3
"""Real ROS smoke: semantic obstacles -> planner detour -> removal replan.

The graph feeds semantic support surfaces to the planner and semantic obstacle
boxes to the same planner. The first plan must detour around the crate; a
semantic version bump that removes the crate must produce a straight plan.
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
FIXTURE = FRAMEWORK / 'fixtures' / 'maps' / 'sem_obstacle.json'
OUT = Path('/tmp/r1032_smoke.json')
PORT = 11518
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1032_ros',
                  ROS_LOG_DIR='/tmp/r1032_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1032_ros').mkdir(exist_ok=True)
Path('/tmp/r1032_logs').mkdir(exist_ok=True)
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
                              / 'graph_semantic_obstacle_planning.yaml')
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
    removed = json.loads(json.dumps(base))
    removed['map_version'] = 2
    removed['features'] = [item for item in removed['features']
                           if (item.get('attributes') or {}).get('role') != 'obstacle']
    result['fixture_map_version'] = base['map_version']
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1032_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runners = {}
    specs = {}
    running = threading.Event()
    try:
        time.sleep(3)
        rospy.init_node('r1032_smoke', anonymous=True, disable_signals=True)
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
        pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
        goal_pub = rospy.Publisher('/v8/local/goal', PoseStamped, queue_size=2, latch=True)
        localization_pub = rospy.Publisher('/v8/local/localization_ok', Bool, queue_size=2)
        semantic_pub = rospy.Publisher('/v8/semantics/sem_support', String,
                                       queue_size=1, latch=True)
        feed = {'pose': (2.6, 2.0, .6), 'goal': (3.8, 2.8, .6)}
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
            localization = Bool(data=True)
            while running.is_set() and not rospy.is_shutdown():
                with lock:
                    pose, goal = feed['pose'], feed['goal']
                pose_pub.publish(signed(*pose))
                if goal != sent_goal:
                    goal_pub.publish(signed(*goal))
                    sent_goal = goal
                localization_pub.publish(localization)
                time.sleep(.1)

        thread = threading.Thread(target=feed_loop, daemon=True)
        thread.start()
        obstacle_status = {'value': None}
        rospy.Subscriber('/v8/local/obstacles/status', String,
                         lambda message: obstacle_status.update(
                             value=json.loads(message.data)), queue_size=1)

        def band_min_y(payload):
            values = [point[1] for point in payload['trajectory']
                      if 3.0 <= point[0] <= 3.4]
            return min(values) if values else None

        def wait_trajectory(version, predicate, timeout=30):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    payload = json.loads(rospy.wait_for_message('/v8/local/trajectory',
                                                                String, timeout=2).data)
                except Exception:
                    continue
                if payload.get('map_version') == version and predicate(payload):
                    return payload
            return None

        detour = wait_trajectory(1, lambda p: p['reason'] == 'arrived'
                          and (band_min_y(p) or 0) > 2.55)
        result['detour'] = None if detour is None else {
            'map_version': detour['map_version'], 'reason': detour['reason'],
            'last': detour['trajectory'][-1], 'band_min_y': round(band_min_y(detour), 4),
            'surfaces': sorted(set(detour['surfaces']))}
        result['obstacles_initial'] = obstacle_status['value']
        time.sleep(.5)
        semantic_pub.publish(String(data=json.dumps(removed, ensure_ascii=False)))
        straight = wait_trajectory(2, lambda p: p['reason'] == 'arrived'
                            and (band_min_y(p) or 99) < 2.55)
        result['straight'] = None if straight is None else {
            'map_version': straight['map_version'], 'reason': straight['reason'],
            'last': straight['trajectory'][-1], 'band_min_y': round(band_min_y(straight), 4)}
        result['obstacles_after'] = obstacle_status['value']
        result['changed'] = bool(detour is not None and straight is not None
                                 and detour['trajectory'] != straight['trajectory'])
        result['ok'] = (result.get('safety_gate') == 'E_SAFETY_ADVICE'
                        and result.get('detour') is not None
                        and result.get('straight') is not None
                        and result.get('changed')
                        and result.get('obstacles_initial', {}).get('count') == 1
                        and result.get('obstacles_after', {}).get('count') == 0)
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
                        or 'semantic_obstacle_bridge_node.py' in row[2]
                        or 'ros_node.py' in row[2])
                    and 'r1032_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = bool(result.get('ok')) and result.get('stopped') and not leftover
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
