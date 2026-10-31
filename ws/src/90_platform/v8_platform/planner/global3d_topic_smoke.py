#!/usr/bin/env python3
"""Real ROS smoke: real support planner through graph with map/start/goal topics."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent / 'framework'
FIXTURE = HERE.parent / 'contracts' / 'fixtures' / 'map_ml_two_surfaces.json'
OUT = Path('/tmp/r1035_smoke.json')
PORT = 11521
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1035_ros',
                  ROS_LOG_DIR='/tmp/r1035_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1035_ros').mkdir(exist_ok=True)
Path('/tmp/r1035_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def distance(a, b):
    return sum((a[i] - b[i]) ** 2 for i in range(3)) ** .5


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    graph = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs'
                              / 'graph_global3d_topic.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1035_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runner = None
    try:
        time.sleep(3)
        rospy.init_node('r1035_smoke', anonymous=True, disable_signals=True)
        instance = plan['instances'][0]
        command, marker = runtime.default_command(
            instance, catalog['components'][instance['component']])
        spec = runtime.build_specs(plan, catalog)[0]
        runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
        runner.start()
        result['command'] = command[:2]
        map_value = json.loads(FIXTURE.read_text())
        last_path, last_status = {'value': None}, {'value': None}

        def on_path(message):
            if message.poses:
                pose = message.poses[-1].pose.position
                last_path['value'] = {'poses': len(message.poses),
                                      'last': [pose.x, pose.y, pose.z],
                                      'frame': message.header.frame_id}

        def on_status(message):
            last_status['value'] = json.loads(message.data)

        rospy.Subscriber('/global_path', Path, on_path, queue_size=2)
        rospy.Subscriber('/global_path/status', String, on_status, queue_size=5)
        map_pub = rospy.Publisher('/v8_support_map', String, queue_size=1, latch=True)
        start_pub = rospy.Publisher('/v8/global/start', PoseStamped, queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/v8/global/goal', PoseStamped, queue_size=1, latch=True)

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        time.sleep(.5)
        map_pub.publish(String(data=json.dumps(map_value)))
        start_pub.publish(signed(3.0, 2.0, .6))
        goal_pub.publish(signed(3.5, 2.0, .6))
        import threading
        running = threading.Event()
        running.set()

        def start_loop():
            while running.is_set() and not rospy.is_shutdown():
                start_pub.publish(signed(3.0, 2.0, .6))
                time.sleep(.1)
        threading.Thread(target=start_loop, daemon=True).start()

        def wait_path(target, timeout=15):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                value = last_path['value']
                if value and distance(value['last'], target) < .1:
                    return value
                time.sleep(.05)
            return None

        first = wait_path((3.5, 2.0, .6))
        result['first'] = first
        result['first_status'] = last_status['value']
        goal_pub.publish(signed(3.8, 2.0, .6))
        second = wait_path((3.8, 2.0, .6))
        result['second'] = second
        updated = json.loads(json.dumps(map_value))
        updated['map_version'] = 3
        updated['source']['uri'] = 'smoke://r1035/v3'
        map_pub.publish(String(data=json.dumps(updated)))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if (last_status['value'] or {}).get('map_version') == 3:
                break
            time.sleep(.05)
        result['version_status'] = last_status['value']
        running.clear()
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
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
                    if 'planner/topic_node.py' in row[2]
                    and 'r1035_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('first') is not None
                        and result.get('second') is not None
                        and (result.get('version_status') or {}).get('map_version') == 3
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
