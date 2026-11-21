#!/usr/bin/env python3
"""app_runner execute module — façade split (M9); external imports unchanged."""
import argparse
import copy
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../../..' / '90_platform/v8_platform/contracts').resolve()))
import app_config  # noqa: E402
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import execution  # noqa: E402
import mission  # noqa: E402
import safety  # noqa: E402


from app_runner_prepare import _prepare_app, resolve_sim_launch, prepare_execution


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
        # 宽泛静默（2026-11-18 标注）：此处吞掉任何异常且不留痕。
        # 排查「静默失败」时优先看这一行——日志缺失往往从这里开始。
            pass


def initialize_ros_node(rospy, gazebo_clock=False):
    """Select the owned Gazebo master's clock before rospy subscribes to time."""
    if gazebo_clock:
        rospy.set_param('/use_sim_time', True)
    rospy.init_node('navigation_app_runner', anonymous=True, disable_signals=True)


def path_summary(path):
    if path is None or not path.poses:
        return None
    return {'poses': len(path.poses),
            'last': [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y,
                     path.poses[-1].pose.position.z]}


def quiesce_before_stop(hold_pub, bool_type, real, sleep=time.sleep):
    if not real and hold_pub is not None:
        hold_pub.publish(bool_type(data=True))
        sleep(.15)


def trace_observer(trace, kind, convert, ros_clock, wall_clock=time.monotonic, period_s=0.):
    """Observe current messages at a bounded rate, without queuing old samples."""
    last_at = [-1e100]
    def callback(message):
        try:
            at = wall_clock()
            if at - last_at[0] < period_s:
                return
            last_at[0] = at
            trace.record(kind, convert(message), ros_clock(), at)
        except Exception as error:
            trace.error(kind, error)
    return callback


def assembly_failed(assembly, result):
    """Retain the first health result; polling can transition the task state."""
    dead = assembly.health()
    if dead:
        result.update(assembly_health=dead, error='assembly component failure: %s' % dead)
    return bool(dead)


def execute(app_path, out_path, port, with_gazebo, gui, timeout_s,
            interactive=False, rviz=False, real=False, trace_path=None):
    prepared = prepare_execution(app_path, real=real, with_gazebo=with_gazebo)
    resolved, catalog, report, plan = (prepared[key] for key in ('resolved', 'catalog', 'report', 'plan'))
    sim_launch = prepared.get('sim_launch')
    log_paths = {name: '/tmp/r1103_' + name + '.log' for name in
                 ('master', 'gazebo', 'viz', 'picker', 'rviz', 'tf_static', 'robot_state', 'joint_state', 'pose_tf')}
    ros_home, ros_logs = '/tmp/r1103_ros', '/tmp/r1103_logs'
    trace = None
    if trace_path is not None:
        from app_trace import Trace, prepare_logs
        log_paths, ros_home, ros_logs = prepare_logs(trace_path)
        trace = Trace({'app': str(resolved['app_path']),
                       'map_id': resolved['map_entry']['map_id'],
                       'map_sha256': resolved['map_sha256'],
                       'backend': resolved['app']['carrier']['backend'],
                       'model_name': resolved['app']['carrier'].get('model_name', 'app_carrier'),
                       'carrier_center_z': resolved['app']['carrier'].get('center_z', .1),
                       'execution_mode': plan.get('execution_mode'), 'port': port,
                       'gazebo_model_min_period_s': .02})
    graph_path = resolved['graph_path']
    task = resolved['task_resolved']
    app = resolved['app']
    mission_goals = mission.goals(task)
    start = tuple(float(v) for v in task['start']['xyz'])
    goal = tuple(float(v) for v in mission_goals[0]['xyz'])
    master = None
    if real:
        # 真机窗口：沿用现场已在位的 master 与传感器；不得另起 roscore。
        os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    else:
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                          ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                          ROS_HOME=ros_home, ROS_LOG_DIR=ros_logs,
                          GAZEBO_MASTER_URI='http://127.0.0.1:%d' % (port + 1),
                          PYTHONDONTWRITEBYTECODE='1')
        Path(ros_home).mkdir(exist_ok=True)
        Path(ros_logs).mkdir(exist_ok=True)
        master = subprocess.Popen(['roscore', '-p', str(port)], start_new_session=True,
                                  stdout=open(log_paths['master'], 'ab'),
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
    hold_pub = None
    old_handlers = {}
    stop_requested = {'value': False}
    result = {'scope': 'navigation_app_execute', 'app': str(resolved['app_path']),
              'ok': False, 'map': {'id': resolved['map_entry']['map_id'],
                                   'version': resolved['map_entry']['version'],
                                   'path': str(resolved['map_path']),
                                   'sha256': resolved['map_sha256']},
              'task_resolved': task, 'start': list(start), 'goal': list(goal),
              'goals': [list(float(v) for v in item['xyz']) for item in mission_goals],
              'mission': len(mission_goals) > 1,
              'interactive': bool(interactive), 'rviz': bool(rviz),
              'checks': {}, 'path': None, 'local_trajectory': None,
              'carrier_final': None, 'markers': 0, 'error': None,
              'logs': {}}
    try:
        time.sleep(3.0)
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Path as RosPath
        from std_msgs.msg import Bool, String
        from visualization_msgs.msg import MarkerArray
        initialize_ros_node(rospy, not real and with_gazebo and app['carrier']['backend'] == 'gazebo')
        if not real and with_gazebo and app['carrier']['backend'] == 'gazebo':
            if sim_launch is None:
                raise config.FrameworkError('E_EXEC_BACKEND', 'gazebo carrier without a simulation launch')
            launch_args = list(sim_launch['argv'])
            launch_args.append('gui:=%s' % ('true' if gui else 'false'))
            world = app['carrier'].get('world')
            if world:
                world_path = Path(world)
                if not world_path.is_absolute():
                    world_path = resolved['root'] / world_path
                launch_args.append('world:=%s' % world_path)
            model_path = app['carrier'].get('model_path')
            if model_path and app['carrier'].get('world_launch'):
                model_dir = Path(model_path)
                if not model_dir.is_absolute():
                    model_dir = resolved['root'] / model_dir
                launch_args.append('model_path:=%s' % model_dir)
            gazebo = subprocess.Popen(
                ['roslaunch'] + launch_args,
                start_new_session=True, stdout=open(log_paths['gazebo'], 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30.)
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
                ['rosrun', 'v8_platform', 'framework/nodes/nav3d_viz_node.py'],
                start_new_session=True, stdout=open(log_paths['viz'], 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        if interactive:
            picker = subprocess.Popen(
                ['rosrun', 'v8_platform', 'framework/nodes/operator_surface_picker.py'],
                start_new_session=True, stdout=open(log_paths['picker'], 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        if rviz:
            # 3D app markers/paths use frame_id=map, but the graph has no TF edge.
            # Publish an identity world->map so RViz Fixed Frame=map and Publish
            # Point both resolve; map is otherwise a floating frame.
            tf_static = subprocess.Popen(
                ['rosrun', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'world', 'map'],
                start_new_session=True, stdout=open(log_paths['tf_static'], 'ab'),
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
                    stdout=open(log_paths['robot_state'], 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
                joint_state = subprocess.Popen(
                    ['rosrun', 'joint_state_publisher', 'joint_state_publisher'],
                    start_new_session=True,
                    stdout=open(log_paths['joint_state'], 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
                pose_tf = subprocess.Popen(
                    ['rosrun', 'v8_platform', 'framework/nodes/pose_tf_broadcaster.py',
                     '_z_offset:=0.18'],
                    start_new_session=True,
                    stdout=open(log_paths['pose_tf'], 'ab'),
                    stderr=subprocess.STDOUT, env=dict(os.environ))
            rviz_proc = subprocess.Popen(
                ['rviz', '-d', str(FRAMEWORK / 'fixtures' / 'rviz' / 'nav3d.rviz')],
                start_new_session=True, stdout=open(log_paths['rviz'], 'ab'),
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
        if trace is not None:
            def observe(kind, convert, period_s=0.):
                return trace_observer(trace, kind, convert, lambda: rospy.Time.now().to_sec(),
                                      period_s=period_s)
            def pose_value(message):
                return {'xyz': [message.pose.position.x, message.pose.position.y, message.pose.position.z],
                        'frame': message.header.frame_id, 'stamp': message.header.stamp.to_sec()}
            def path_value(message):
                return {'points': [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                   for p in message.poses],
                        'frame': message.header.frame_id, 'stamp': message.header.stamp.to_sec()}
            def twist_value(message):
                return {'linear': [message.linear.x, message.linear.y, message.linear.z],
                        'angular': [message.angular.x, message.angular.y, message.angular.z]}
            rospy.Subscriber('/v8/3d/global/path', RosPath, observe('global_path', path_value), queue_size=5)
            rospy.Subscriber('/v8/3d/global/status', String,
                             observe('global_status', lambda m: json.loads(m.data)), queue_size=50)
            rospy.Subscriber('/v8/local/trajectory', String,
                             observe('local_trajectory', lambda m: json.loads(m.data)), queue_size=100)
            rospy.Subscriber('/v8/local/status', String,
                             observe('local_status', lambda m: json.loads(m.data)), queue_size=100)
            rospy.Subscriber('/v8/local/pose', PoseStamped, observe('pose', pose_value), queue_size=200)
            rospy.Subscriber('/cmd_vel', Twist, observe('command', twist_value), queue_size=200)
            if with_gazebo and app['carrier']['backend'] == 'gazebo':
                from gazebo_msgs.msg import ModelStates
                from rosgraph_msgs.msg import Clock
                clock_last = {'at': -1e9}
                def on_clock(message):
                    at = time.monotonic()
                    if at - clock_last['at'] >= .05:
                        trace.record('clock', {'stamp': message.clock.to_sec()}, rospy.Time.now().to_sec(), at)
                        clock_last['at'] = at
                rospy.Subscriber('/clock', Clock, on_clock, queue_size=100)
                def model_value(message):
                    name = app['carrier'].get('model_name', 'app_carrier')
                    if name not in message.name:
                        return {'present': False, 'model': name}
                    i = message.name.index(name)
                    pose = message.pose[i]
                    return {'present': True, 'model': name,
                            'xyz': [pose.position.x, pose.position.y, pose.position.z],
                            'linear': [message.twist[i].linear.x, message.twist[i].linear.y,
                                       message.twist[i].linear.z]}
                rospy.Subscriber('/gazebo/model_states', ModelStates,
                                 observe('gazebo_model', model_value, period_s=.02), queue_size=1)
        start_pub = rospy.Publisher('/v8/3d/global/start', PoseStamped, queue_size=1, latch=True)
        goal_topic = app.get('goal_topic', '/v8/3d/global/goal')
        goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1, latch=True)
        mission_topic = app.get('mission_topic', '/v8/bt/goal_mission')
        mission_pub = rospy.Publisher(mission_topic, String, queue_size=16, latch=True)
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
            if len(mission_goals) > 1:
                for index, item in enumerate(mission_goals, 1):
                    mission_pub.publish(String(data=mission.goal_wire(
                        item, index, len(mission_goals))))
                    time.sleep(0.05)
            else:
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
                    if not real:
                        obstacle_pub.publish(String(data='[]'))
                        hold_pub.publish(Bool(data=False))
                    time.sleep(.2)
            except KeyboardInterrupt:
                result['interrupted'] = True
            if stop_requested['value']:
                result['interrupted'] = True
        while not interactive and time.monotonic() < deadline and not rospy.is_shutdown():
            if not real:
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
            if assembly_failed(assembly, result):
                break
            time.sleep(.1)
        path = state['path']
        result['path'] = path_summary(path)
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
            result['ok'] = not result['error'] and all(result['checks'].values())
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        try:
            quiesce_before_stop(hold_pub, Bool, real)
        except Exception as error:
            result['stop_error'] = repr(error)
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
        result['logs'] = dict(log_paths)
    if trace is not None:
        try:
            trace_value = trace.write(trace_path)
            result['trace'] = {'path': str(trace_path), 'events': len(trace_value['events']),
                               'capture_ok': trace_value['capture_ok']}
            if not trace_value['capture_ok']:
                result['ok'] = False
                result['trace_error'] = 'observation overflow or callback error'
        except Exception as error:
            result['ok'] = False
            result['trace_error'] = repr(error)
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
    parser.add_argument('--trace-out', default=None, help='opt-in full observation JSON; capture failure fails this run')
    parser.add_argument('--real', action='store_true',
                        help='use the on-robot ROS master and live sensors; '
                             'no roscore, no gazebo, no harness obstacle/hold publishers')
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
    return execute(args.app, args.out, args.port, not args.real and not args.no_gazebo, args.gui,
                   args.timeout_s, interactive=args.interactive or args.rviz,
                   rviz=args.rviz, real=args.real, trace_path=args.trace_out)

