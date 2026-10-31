#!/usr/bin/env python3
"""A4 runtime smoke: real 3D global planner path, no-route and budget refusals."""
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
OUT = Path('/tmp/r1036_smoke.json')
PORT = 11522
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1036_ros',
                  ROS_LOG_DIR='/tmp/r1036_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1036_ros').mkdir(exist_ok=True)
Path('/tmp/r1036_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'validator'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import trajectory_checker  # noqa: E402


def blocked_map():
    value = json.loads(FIXTURE.read_text())
    value['schema_version'] = 2
    value['map_version'] = 3
    value['multiple_surfaces_per_xy'] = True
    value['obstacles'] = [{'obstacle_id': 'wall', 'min': [3.2, -5.0, -1.0],
                       'max': [3.4, 5.0, 1.5]}]
    return value


def start_node(plan, catalog, port):
    instance = plan['instances'][0]
    command, marker = runtime.default_command(
        instance, catalog['components'][instance['component']])
    spec = runtime.build_specs(plan, catalog)[0]
    runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
    runner.start()
    return runner, command


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
    plan['instances'][0]['params'].update(
        robot_radius_m=.15, robot_height_m=.35, min_clearance_m=.2)
    result = {'startup_order': report['startup_order']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1036_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runner = None
    try:
        time.sleep(3)
        rospy.init_node('r1036_smoke', anonymous=True, disable_signals=True)
        paths, statuses = [], []
        rospy.Subscriber('/global_path', Path, lambda m: paths.append(m), queue_size=2)
        rospy.Subscriber('/global_path/status', String,
                         lambda m: statuses.append(json.loads(m.data)), queue_size=10)
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

        runner, command = start_node(plan, catalog, PORT)
        result['command'] = command[:2]
        value = json.loads(FIXTURE.read_text())
        time.sleep(.5)
        map_pub.publish(String(data=json.dumps(value)))
        start_pub.publish(signed(3.0, 2.0, .6))
        goal_pub.publish(signed(3.5, 2.0, .6))

        def wait_path(timeout=15):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if paths:
                    return paths[-1]
                time.sleep(.05)
            return None

        path = wait_path()
        result['path'] = None
        if path is not None:
            points = [[pose.pose.position.x, pose.pose.position.y,
                       pose.pose.position.z, index * .5]
                      for index, pose in enumerate(path.poses)]
            checked = trajectory_checker.check(value, points, min_clearance=.2,
                                               expected_map_version=value['map_version'])
            result['path'] = {'poses': len(path.poses), 'checker_ok': checked['ok'],
                              'violations': checked['violations'],
                              'checked_points': checked['checked_points']}
        wall = blocked_map()
        statuses.clear()
        map_pub.publish(String(data=json.dumps(wall)))
        goal_pub.publish(signed(3.8, 2.0, .6))
        deadline = time.monotonic() + 25
        no_route = None
        while time.monotonic() < deadline:
            no_route = next((item for item in statuses
                             if item.get('reason') == 'E_PLAN_NO_ROUTE'), None)
            if no_route:
                break
            time.sleep(.05)
        result['no_route'] = no_route
        result['no_route_statuses'] = statuses[-6:]
        runner.stop('process_group_signal')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not runner.is_stopped():
            time.sleep(.1)
        runner = None
        plan_budget = copy.deepcopy(plan)
        plan_budget['instances'][0]['params']['budget'] = 1
        statuses.clear()
        runner, command = start_node(plan_budget, catalog, PORT)
        result['budget_command'] = command[:2]
        time.sleep(.5)
        map_pub.publish(String(data=json.dumps(value)))
        start_pub.publish(signed(3.0, 2.0, .6))
        goal_pub.publish(signed(3.5, 2.0, .6))
        deadline = time.monotonic() + 10
        budget = None
        while time.monotonic() < deadline:
            budget = next((item for item in statuses
                           if item.get('reason') == 'E_PLAN_BUDGET_EXCEEDED'), None)
            if budget:
                break
            time.sleep(.05)
        result['budget'] = budget
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
                    and 'r1036_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('path') is not None
                        and result['path']['checker_ok']
                        and result.get('no_route') is not None
                        and result.get('budget') is not None
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
