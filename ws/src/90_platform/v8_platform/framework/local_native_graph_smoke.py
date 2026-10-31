#!/usr/bin/env python3
"""Real ROS smoke: native 3D planner registered in v2 and started by Assembly.

The graph selects v8.fw.local_native and points at its per-instance YAML. The
runtime must launch it through a generated roslaunch so the config file actually
reaches the node; planning then proves the loaded config took effect.
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
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import safety  # noqa: E402

OUT = Path('/tmp/r1025_smoke.json')
STATE = Path('/tmp/r1025_state.json')
PORT = 11504
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1025_ros',
                  ROS_LOG_DIR='/tmp/r1025_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1025_ros').mkdir(exist_ok=True)
Path('/tmp/r1025_logs').mkdir(exist_ok=True)


def start(argv, log):
    stream = open(log, 'ab')
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_local_native.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers'],
              'config_sha256': plan['instances'][0]['config_sha256'],
              'config_keys': report['configs']['local']['keys']}
    try:
        safety.check_plan(plan, catalog)
        result['safety_gate'] = 'ok'
    except config.FrameworkError as error:
        # No 3D arbiter owns /cmd_vel for a TwistStamped command yet; the gate
        # refuses a full assembly, so this smoke starts the runner directly.
        result['safety_gate'] = error.code
    if STATE.exists():
        STATE.unlink()
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1025_master.log')
    delta_node = None
    runner = None
    spec = None
    running = threading.Event()
    try:
        time.sleep(3)
        import rospy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, String
        rospy.init_node('r1025_smoke', anonymous=True, disable_signals=True)
        delta_node = start([sys.executable, '-B',
                            str(HERE.parent / 'representation' / 'delta_map_node.py'),
                            '_map_json:=' + str(HERE.parent / 'contracts' / 'fixtures'
                                                / 'map_ml_two_surfaces.json'),
                            '_state_path:=' + str(STATE),
                            '_delta_topic:=/map_delta',
                            '_map_topic:=/v8_support_map',
                            '_status_topic:=/v8_map_status'],
                           '/tmp/r1025_delta.log')
        time.sleep(1.)
        spec = runtime.build_specs(plan, catalog)[0]
        command, marker = runtime.default_command(plan['instances'][0],
                                                  catalog['components'][plan['instances'][0]['component']])
        result['command'] = command[:2]
        result['uses_param_exec'] = command[1].endswith('param_exec.py')
        result['ready_marker'] = marker
        runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
        runner.start()
        result['started'] = runner.is_alive()
        pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
        goal_pub = rospy.Publisher('/v8/local/goal', PoseStamped, queue_size=2, latch=True)
        obstacles_pub = rospy.Publisher('/v8/local/obstacles', String, queue_size=2)
        localization_pub = rospy.Publisher('/v8/local/localization_ok', Bool, queue_size=2)
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
                pose_pub.publish(signed(3.0, 2.0, 0.6))
                if sent_goal != (3.5, 2.0, 0.6):
                    goal_pub.publish(signed(3.5, 2.0, 0.6))
                    sent_goal = (3.5, 2.0, 0.6)
                obstacles_pub.publish(obstacles)
                localization_pub.publish(localization)
                time.sleep(.1)

        thread = threading.Thread(target=feed_loop, daemon=True)
        thread.start()
        latest = {'status': None}
        rospy.Subscriber('/v8/local/status', String,
                         lambda message: latest.update(status=json.loads(message.data)),
                         queue_size=1)
        trajectory = None
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and trajectory is None:
            try:
                payload = json.loads(rospy.wait_for_message('/v8/local/trajectory', String,
                                                            timeout=2).data)
            except Exception:
                continue
            if payload.get('map_version') == 2 and abs(payload['trajectory'][-1][2] - .6) < 1e-6:
                trajectory = payload
        result['trajectory'] = None if trajectory is None else {
            'map_version': trajectory['map_version'], 'reason': trajectory['reason'],
            'last': trajectory['trajectory'][-1],
            'surfaces': sorted(set(trajectory['surfaces']))}
        status = latest['status']
        result['status'] = None if status is None else {
            'map_version': status.get('map_version'),
            'wall_watchdog_s': status.get('wall_watchdog_s'),
            'plan_ms': status.get('plan_ms'),
            'capability_domain': status.get('capability_domain')}
        result['config_applied'] = bool(status and status.get('wall_watchdog_s') == 7.5)
        runner.stop(spec.stop_method)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not runner.is_stopped():
            time.sleep(.1)
        if not runner.is_stopped():
            runner.force()
            time.sleep(.5)
        result['stopped'] = runner.is_stopped()
        stop(delta_node)
        delta_node = None
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('ros_node.py' in row[2] or 'delta_map_node.py' in row[2]
                        or 'v8_param_exec_' in row[2])
                    and 'r1025_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['safety_gate'] == 'E_SAFETY_ADVICE'
                        and result['started'] and result['uses_param_exec']
                        and result['trajectory'] is not None
                        and result['trajectory']['reason'] == 'arrived'
                        and result['trajectory']['map_version'] == 2
                        and result['config_applied']
                        and result['stopped'] and not leftover)
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        running.clear()
        if runner is not None and spec is not None and not runner.is_stopped():
            runner.stop(spec.stop_method)
        if delta_node is not None:
            stop(delta_node)
        stop(master)
        time.sleep(.3)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
