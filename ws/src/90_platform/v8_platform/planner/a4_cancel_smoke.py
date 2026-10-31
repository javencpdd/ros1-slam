#!/usr/bin/env python3
"""A4 cancel smoke: cancel a long no-route search through the real adapter."""
import copy
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
OUT = Path('/tmp/r1037_smoke.json')
PORT = 11523
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1037_ros',
                  ROS_LOG_DIR='/tmp/r1037_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1037_ros').mkdir(exist_ok=True)
Path('/tmp/r1037_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def blocked_map():
    value = json.loads(FIXTURE.read_text())
    value['schema_version'] = 2
    value['map_version'] = 3
    value['multiple_surfaces_per_xy'] = True
    value['obstacles'] = [{'obstacle_id': 'wall', 'min': [3.2, -5.0, -1.0],
                           'max': [3.4, 5.0, 1.5]}]
    return value


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    graph = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs'
                              / 'graph_global3d_topic.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    plan['instances'][0]['params'].update(
        robot_radius_m=.15, robot_height_m=.35, min_clearance_m=.2)
    result = {'cancellation': json.loads(
        (FRAMEWORK / 'fixtures' / 'manifests' / 'manifest_global3d_topic.json').read_text()
    )['capability']['cancellation']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1037_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runner = None
    try:
        time.sleep(3)
        rospy.init_node('r1037_smoke', anonymous=True, disable_signals=True)
        instance = plan['instances'][0]
        command, marker = runtime.default_command(
            instance, catalog['components'][instance['component']])
        spec = runtime.build_specs(plan, catalog)[0]
        runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
        runner.start()
        statuses, paths = [], []
        rospy.Subscriber('/global_path/status', String,
                         lambda m: statuses.append(json.loads(m.data)), queue_size=10)
        rospy.Subscriber('/global_path', Path, lambda m: paths.append(m), queue_size=2)
        map_pub = rospy.Publisher('/v8_support_map', String, queue_size=1, latch=True)
        start_pub = rospy.Publisher('/v8/global/start', PoseStamped,
                                    queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/v8/global/goal', PoseStamped,
                                   queue_size=1, latch=True)

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        time.sleep(.5)
        map_pub.publish(String(data=json.dumps(blocked_map())))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if any(item.get('map_version') == 3 for item in statuses):
                break
            time.sleep(.05)
        statuses.clear()
        paths.clear()
        start_pub.publish(signed(3.0, 2.0, .6))
        goal_pub.publish(signed(3.8, 2.0, .6))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if any(item.get('reason') == 'E_PLAN_RUNNING' for item in statuses):
                break
            time.sleep(.05)
        result['running_seen'] = any(item.get('reason') == 'E_PLAN_RUNNING'
                                     for item in statuses)
        rospy.wait_for_service('/v8_support_planner_topic/cancel', timeout=10)
        started = time.monotonic()
        response = rospy.ServiceProxy('/v8_support_planner_topic/cancel', Trigger)()
        result['cancel_service'] = response.success
        deadline = time.monotonic() + 15
        cancelled = None
        while time.monotonic() < deadline:
            cancelled = next((item for item in statuses
                              if item.get('reason') == 'E_PLAN_CANCELLED'), None)
            if cancelled:
                break
            time.sleep(.05)
        result['cancelled'] = cancelled
        result['cancel_latency_s'] = round(time.monotonic() - started, 3)
        result['paths_after_cancel'] = len(paths)
        result['nonzero_paths_after_cancel'] = sum(1 for item in paths if item.poses)
        result['statuses'] = statuses[-8:]
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
        result['stopped'] = runner is None or runner.is_stopped()
        os.killpg(master.pid, signal.SIGTERM)
        master.wait(timeout=10)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'planner/topic_node.py' in row[2]
                    and 'r1037_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('cancellation') == 'cooperative'
                        and result.get('cancel_service')
                        and result.get('cancelled') is not None
                        and result.get('cancel_latency_s', 99) < 3.
                        and result.get('nonzero_paths_after_cancel') == 0
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
