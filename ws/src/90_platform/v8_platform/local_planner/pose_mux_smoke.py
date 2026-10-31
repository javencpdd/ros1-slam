#!/usr/bin/env python3
"""Real ROS smoke: pose-source handover, seamless on agreement, reject on jump."""
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
OUT = Path('/tmp/r1033_smoke.json')
PORT = 11519
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1033_ros',
                  ROS_LOG_DIR='/tmp/r1033_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1033_ros').mkdir(exist_ok=True)
Path('/tmp/r1033_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    graph = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs' / 'graph_pose_mux.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1033_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runner = None
    running = threading.Event()
    try:
        time.sleep(3)
        rospy.init_node('r1033_smoke', anonymous=True, disable_signals=True)
        instance = plan['instances'][0]
        command, marker = runtime.default_command(
            instance, catalog['components'][instance['component']])
        spec = runtime.build_specs(plan, catalog)[0]
        runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
        runner.start()
        result['command'] = command[:2]
        pose_a = rospy.Publisher('/v8/local/pose_in_a', PoseStamped, queue_size=2)
        pose_b = rospy.Publisher('/v8/local/pose_in_b', PoseStamped, queue_size=2)
        select = rospy.Publisher('/v8/local/pose_select', String, queue_size=2)
        outputs, statuses = [], []
        rospy.Subscriber('/v8/local/pose', PoseStamped,
                         lambda message: outputs.append(
                             (time.monotonic(), message.pose.position.x)), queue_size=20)
        rospy.Subscriber('/v8/local/pose_mux/status', String,
                         lambda message: statuses.append(json.loads(message.data)),
                         queue_size=20)
        feed = {'a': (0.0, 0.0, 0.0), 'b': (0.05, 0.0, 0.0)}
        lock = threading.Lock()
        running.set()

        def signed(values):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = values
            message.pose.orientation.w = 1.
            return message

        def feed_loop():
            while running.is_set() and not rospy.is_shutdown():
                with lock:
                    a, b = feed['a'], feed['b']
                pose_a.publish(signed(a))
                pose_b.publish(signed(b))
                time.sleep(.05)

        threading.Thread(target=feed_loop, daemon=True).start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and len(outputs) < 20:
            time.sleep(.05)
        result['initial_outputs'] = len(outputs)
        time.sleep(1.0)
        before = len(outputs)
        select.publish(String(data='b'))
        deadline = time.monotonic() + 5
        accepted = False
        while time.monotonic() < deadline:
            accepted = any(item.get('action') == 'select' and item.get('ok')
                           for item in statuses)
            if accepted and len(outputs) > before + 10:
                break
            time.sleep(.05)
        result['switch_accepted'] = accepted
        result['max_gap_s'] = round(max((b[0] - a[0] for a, b in zip(
            outputs, outputs[1:])), default=0.0), 4)
        with lock:
            feed['b'] = (5.0, 0.0, 0.0)
        time.sleep(.2)
        statuses.clear()
        select.publish(String(data='b'))
        deadline = time.monotonic() + 3
        rejected = None
        while time.monotonic() < deadline:
            rejected = next((item for item in statuses
                             if item.get('action') == 'select'
                             and item.get('reason') == 'E_POSE_MUX_JUMP'), None)
            if rejected:
                break
            time.sleep(.05)
        result['jump_rejected'] = rejected is not None
        time.sleep(.5)
        result['last_x'] = round(outputs[-1][1], 4) if outputs else None
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        running.clear()
        if runner is not None and not runner.is_stopped():
            runner.stop('process_group_signal')
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and not runner.is_stopped():
                time.sleep(.1)
            if not runner.is_stopped():
                runner.force()
        result['stopped'] = bool(runner and runner.is_stopped())
        os.killpg(master.pid, signal.SIGTERM)
        master.wait(timeout=10)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'pose_mux_node.py' in row[2]
                    and 'r1033_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('initial_outputs', 0) > 0
                        and result.get('switch_accepted')
                        and result.get('max_gap_s', 1) < 0.12
                        and result.get('jump_rejected')
                        and result.get('last_x') is not None and result['last_x'] < 1.
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
