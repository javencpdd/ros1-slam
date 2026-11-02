#!/usr/bin/env python3
"""One-command 3D navigation app runner.

Dry-run resolves app/map/task/graph and prints the filled 3D task. Execute
starts roscore + Gazebo (optional), launches the graph Assembly, publishes the
task start/goal, monitors the global path/local trajectory/carrier and writes a
JSON record. The support-map marker node is started when app.visualization.markers.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import app_config  # noqa: E402
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def dry_run(app_path):
    resolved = app_config.load_app(app_path)
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(resolved['graph_path'])
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    return {
        'scope': 'navigation_app_resolution',
        'ok': True,
        'app': str(resolved['app_path']),
        'app_name': resolved['app']['name'],
        'map': {'entry': resolved['map_entry'], 'path': str(resolved['map_path']),
                'sha256': resolved['map_sha256']},
        'task_resolved': resolved['task_resolved'],
        'graph': {'path': str(resolved['graph_path']), 'digest': report['digest'],
                  'startup_order': report['startup_order'],
                  'command_writers': report['command_writers']},
        'plan': plan,
    }


def _terminate(proc, group=False):
    if proc is None or proc.poll() is not None:
        return
    try:
        if group:
            os.killpg(proc.pid, signal.SIGTERM)
        else:
            proc.terminate()
        proc.wait(timeout=8.)
    except Exception:
        try:
            if group:
                os.killpg(proc.pid, signal.SIGKILL)
            else:
                proc.kill()
        except Exception:
            pass


def execute(app_path, out_path, port, with_gazebo, gui, timeout_s,
            interactive=False, rviz=False):
    resolved = app_config.load_app(app_path)
    task = resolved['task_resolved']
    app = resolved['app']
    start = tuple(float(v) for v in task['start']['xyz'])
    goal = tuple(float(v) for v in task['goal']['xyz'])
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_HOME='/tmp/r1103_ros', ROS_LOG_DIR='/tmp/r1103_logs',
                      GAZEBO_MASTER_URI='http://127.0.0.1:%d' % (port + 1),
                      PYTHONDONTWRITEBYTECODE='1')
    Path('/tmp/r1103_ros').mkdir(exist_ok=True)
    Path('/tmp/r1103_logs').mkdir(exist_ok=True)
    master = subprocess.Popen(['roscore', '-p', str(port)], start_new_session=True,
                              stdout=open('/tmp/r1103_master.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
    gazebo = None
    viz = None
    picker = None
    rviz_proc = None
    tf_static = None
    robot_state = None
    joint_state = None
    pose_tf = None
    assembly = None
    old_handlers = {}
    stop_requested = {'value': False}
    result = {'scope': 'navigation_app_execute', 'app': str(resolved['app_path']),
              'ok': False, 'map': {'id': resolved['map_entry']['map_id'],
                                   'version': resolved['map_entry']['version'],
                                   'path': str(resolved['map_path']),
                                   'sha256': resolved['map_sha256']},
              'task_resolved': task, 'start': list(start), 'goal': list(goal),
              'interactive': bool(interactive), 'rviz': bool(rviz),
              'checks': {}, 'path': None, 'local_trajectory': None,
              'carrier_final': None, 'markers': 0, 'error': None,
              'logs': {}}
    try:
        time.sleep(3.0)
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from std_msgs.msg import Bool, String
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('navigation_app_runner', anonymous=True, disable_signals=True)
        if with_gazebo and app['carrier']['backend'] == 'gazebo':
            world_launch = app['carrier'].get('world_launch')
            if world_launch:
                launch_path = Path(world_launch)
                if not launch_path.is_absolute():
                    launch_path = resolved['root'] / launch_path
                launch_args = [str(launch_path)]
            else:
                launch_args = ['lite3_gazebo_sim', 'nist_stairs_120_s020x170.launch']
            launch_args.append('gui:=%s' % ('true' if gui else 'false'))
            world = app['carrier'].get('world')
            if world:
                world_path = Path(world)
                if not world_path.is_absolute():
                    world_path = resolved['root'] / world_path
                launch_args.append('world:=%s' % world_path)
            gazebo = subprocess.Popen(
                ['roslaunch'] + launch_args,
                start_new_session=True, stdout=open('/tmp/r1103_gazebo.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30.)
        catalog = config.load_components(HERE / 'fixtures' / 'manifests')
        graph_path = resolved['graph_path']
        graph = config.load_graph(graph_path)
        report = graph_mod.check(graph, catalog)
        plan = render.render_plan(graph, catalog, report)
        for instance in plan['instances']:
            if instance['id'] == 'map':
                instance['params']['map_json'] = str(resolved['map_path'])
                instance['params']['map_topic'] = '/v8_support_map'
            if instance['id'] == 'carrier':
                instance['params'].update(start_x=start[0], start_y=start[1], start_z=start[2])
                if app['carrier'].get('backend') == 'gazebo':
                    instance['params'].update(
                        model_name=app['carrier'].get('model_name', 'app_carrier'),
                        center_z=app['carrier'].get('center_z', .1),
                        snap_m=app['carrier'].get('snap_m', .03), spawn=True)
            if instance['id'] == 'loc':
                instance['params']['source'] = app['localization']['source']
        assembly = runtime.Assembly(plan, catalog, ready_timeout=45.,
                                    run_timeout=timeout_s + 40.)
        assembly.start()
        result['graph'] = {'path': str(graph_path), 'digest': report['digest'],
                           'startup_order': report['startup_order'],
                           'command_writers': report['command_writers']}
        result['checks']['assembly_ready'] = assembly.ready()
        if not assembly.ready():
            raise RuntimeError('assembly not ready')
        if app.get('visualization', {}).get('markers', True):
            viz = subprocess.Popen(
                ['rosrun', 'v8_platform', 'framework/nav3d_viz_node.py'],
                start_new_session=True, stdout=open('/tmp/r1103_viz.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        if interactive:
            picker = subprocess.Popen(
                ['rosrun', 'v8_platform', 'framework/operator_surface_picker.py'],
                start_new_session=True, stdout=open('/tmp/r1103_picker.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        if rviz:
            # 3D app markers/paths use frame_id=map, but the graph has no TF edge.
            # Publish an identity world->map so RViz Fixed Frame=map and Publish
            # Point both resolve; map is otherwise a floating frame.
            tf_static = subprocess.Popen(
                ['rosrun', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'world', 'map'],
                start_new_session=True, stdout=open('/tmp/r1103_tf_static.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
            urdf = None
            for package_name, relative in (
                    ('lite3_description', 'urdf/Lite3.urdf'),
                    ('lite3_bringup', 'urdf/Lite3_prim.urdf')):
                try:
                    package = subprocess.check_output(
                        ['rospack', 'find', package_name],
                        env=dict(os.environ)).decode().strip()
                    candidate = Path(package) / relative
                    if candidate.is_file():
                        urdf = candidate
                        break
                except Exception:
                    continue
            if urdf is not None and urdf.is_file():
                rospy.set_param('/robot_description', urdf.read_text())
                robot_state = subprocess.Popen(
                    ['rosrun', 'robot_state_publisher', 'robot_state_publisher'],
                    start_new_session=True,
                    stdout=open('/tmp/r1103_robot_state.log', 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
                joint_state = subprocess.Popen(
                    ['rosrun', 'joint_state_publisher', 'joint_state_publisher'],
                    start_new_session=True,
                    stdout=open('/tmp/r1103_joint_state.log', 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
                pose_tf = subprocess.Popen(
                    ['rosrun', 'v8_platform', 'framework/pose_tf_broadcaster.py',
                     '_z_offset:=0.18'],
                    start_new_session=True,
                    stdout=open('/tmp/r1103_pose_tf.log', 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
            rviz_proc = subprocess.Popen(
                ['rviz', '-d', str(HERE / 'fixtures' / 'rviz' / 'nav3d.rviz')],
                start_new_session=True, stdout=open('/tmp/r1103_rviz.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        state = {'path': None, 'trajectory': None, 'pose': None, 'markers': 0,
                 'statuses': [], 'carrier_status': None}
        rospy.Subscriber('/v8/3d/global/path', RosPath,
                         lambda m: state.update(path=m), queue_size=2)
        rospy.Subscriber('/v8/local/trajectory', String,
                         lambda m: state.update(trajectory=json.loads(m.data)), queue_size=5)
        rospy.Subscriber('/v8/local/pose', PoseStamped,
                         lambda m: state.update(pose=(m.pose.position.x, m.pose.position.y,
                                                      m.pose.position.z)), queue_size=20)
        rospy.Subscriber('/v8/local/status', String,
                         lambda m: state['statuses'].append(json.loads(m.data)), queue_size=50)
        rospy.Subscriber('/v8/3d/carrier/status', String,
                         lambda m: state.update(carrier_status=json.loads(m.data)), queue_size=20)
        rospy.Subscriber('/v8/3d/viz/markers', MarkerArray,
                         lambda m: state.update(markers=len(m.markers)), queue_size=2)
        start_pub = rospy.Publisher('/v8/3d/global/start', PoseStamped, queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/v8/3d/global/goal', PoseStamped, queue_size=1, latch=True)
        obstacle_pub = rospy.Publisher('/v8/local/obstacles', String, queue_size=2)
        hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=2, latch=True)

        def signed(xyz):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = xyz
            message.pose.orientation.w = 1.
            return message
        time.sleep(1.0)
        start_pub.publish(signed(start))
        if not interactive:
            goal_pub.publish(signed(goal))
        deadline = time.monotonic() + timeout_s
        arrived = False
        if interactive:
            def request_stop(_signum, _frame):
                stop_requested['value'] = True
            for sig in (signal.SIGINT, signal.SIGTERM):
                old_handlers[sig] = signal.signal(sig, request_stop)
            result['checks']['assembly_ready'] = assembly.ready()
            result['checks']['markers_published'] = state['markers'] > 0
            print('INTERACTIVE_READY start=%s; RViz Publish Point 选 3D 目标' % (start,),
                  flush=True)
            try:
                while not rospy.is_shutdown() and not stop_requested['value']:
                    obstacle_pub.publish(String(data='[]'))
                    hold_pub.publish(Bool(data=False))
                    time.sleep(.2)
            except KeyboardInterrupt:
                result['interrupted'] = True
            if stop_requested['value']:
                result['interrupted'] = True
        while not interactive and time.monotonic() < deadline and not rospy.is_shutdown():
            obstacle_pub.publish(String(data='[]'))
            hold_pub.publish(Bool(data=False))
            trajectory = state['trajectory']
            pose = state['pose']
            path_now = state['path']
            local_goal_now = (goal if path_now is None or not path_now.poses else
                              (path_now.poses[-1].pose.position.x,
                               path_now.poses[-1].pose.position.y,
                               path_now.poses[-1].pose.position.z))
            if trajectory and trajectory.get('reason') == 'arrived' and pose is not None:
                distance = ((pose[0] - local_goal_now[0]) ** 2 +
                            (pose[1] - local_goal_now[1]) ** 2) ** .5
                if distance <= .20 and abs(pose[2] - local_goal_now[2]) <= .02:
                    arrived = True
                    break
            if assembly.health():
                result['assembly_health'] = assembly.health()
                break
            time.sleep(.1)
        path = state['path']
        result['path'] = None if path is None else {
            'poses': len(path.poses),
            'last': [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y,
                     path.poses[-1].pose.position.z]}
        result['local_trajectory'] = None if state['trajectory'] is None else {
            'map_version': state['trajectory'].get('map_version'),
            'reason': state['trajectory'].get('reason'),
            'last': (state['trajectory'].get('trajectory') or [None])[-1]}
        result['carrier_final'] = state['pose']
        result['carrier_status'] = state['carrier_status']
        result['markers'] = state['markers']
        result['start_status'] = rospy.get_param('/v8/3d/global/status', None)
        local_goal = None if result['path'] is None else result['path']['last']
        result['local_goal'] = local_goal
        distance = (None if state['pose'] is None or local_goal is None else
                    ((state['pose'][0] - local_goal[0]) ** 2 +
                     (state['pose'][1] - local_goal[1]) ** 2) ** .5)
        task_distance = (None if state['pose'] is None else
                         ((state['pose'][0] - goal[0]) ** 2 +
                          (state['pose'][1] - goal[1]) ** 2) ** .5)
        result['carrier_distance_to_local_goal_m'] = distance
        result['carrier_distance_to_task_goal_m'] = task_distance
        z_error = (None if state['pose'] is None or local_goal is None
                   else abs(state['pose'][2] - local_goal[2]))
        result['carrier_z_error_m'] = z_error
        result['local_status_tail'] = state['statuses'][-5:]
        local_arrived = bool(state['trajectory'] and state['trajectory'].get('reason') == 'arrived')
        at_local_goal = distance is not None and distance <= .20 and z_error is not None and z_error <= .02
        result['checks'].update({
            'map_configured': True,
            'global_path': result['path'] is not None and result['path']['poses'] > 0,
            'local_arrived': local_arrived,
            'carrier_at_local_goal': at_local_goal,
            'markers_published': state['markers'] > 0,
        })
        if interactive:
            result['ok'] = bool(assembly.ready() and state['markers'] > 0)
        else:
            result['ok'] = all(result['checks'].values())
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
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        _terminate(picker, group=True)
        _terminate(rviz_proc, group=True)
        _terminate(pose_tf, group=True)
        _terminate(joint_state, group=True)
        _terminate(robot_state, group=True)
        _terminate(tf_static, group=True)
        _terminate(viz, group=True)
        _terminate(gazebo, group=True)
        _terminate(master, group=True)
        result['logs'] = {
            'master': '/tmp/r1103_master.log', 'gazebo': '/tmp/r1103_gazebo.log',
            'viz': '/tmp/r1103_viz.log', 'picker': '/tmp/r1103_picker.log',
            'rviz': '/tmp/r1103_rviz.log', 'tf_static': '/tmp/r1103_tf_static.log',
            'robot_state': '/tmp/r1103_robot_state.log',
            'joint_state': '/tmp/r1103_joint_state.log',
            'pose_tf': '/tmp/r1103_pose_tf.log'}
    Path(out_path).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({k: result[k] for k in ('ok', 'checks', 'path', 'local_trajectory',
                                             'carrier_final', 'markers', 'error')},
                     ensure_ascii=False))
    return 0 if result['ok'] else 6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', required=True)
    parser.add_argument('--out', default='/tmp/navigation_app.json')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-gazebo', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--port', type=int, default=11537)
    parser.add_argument('--timeout-s', type=float, default=90.)
    parser.add_argument('--interactive', action='store_true',
                        help='keep the 3D stack alive; RViz Publish Point selects goals')
    parser.add_argument('--rviz', action='store_true',
                        help='start RViz with fixtures/rviz/nav3d.rviz (implies interactive)')
    args = parser.parse_args()
    if not args.execute:
        result = dry_run(args.app)
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps({'ok': result['ok'], 'task_resolved': result['task_resolved'],
                          'graph': result['graph'], 'map': result['map']},
                         ensure_ascii=False))
        return 0
    return execute(args.app, args.out, args.port, not args.no_gazebo, args.gui,
                   args.timeout_s, interactive=args.interactive or args.rviz,
                   rviz=args.rviz)


if __name__ == '__main__':
    raise SystemExit(main())
