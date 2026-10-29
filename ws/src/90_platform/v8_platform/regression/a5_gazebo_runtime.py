#!/usr/bin/env python3
"""Native local planner → explicitly gated planar carrier → Gazebo physics.

Requires an existing isolated master, m1 Gazebo, /rslidar_points and safe mux.
Creates only owned planner/adapter processes and one temporary obstacle model.
Obstacle input comes from measured lidar ROI points, never model-state truth.
The floor is an explicit test support-map fixture, not sensor-built mapping.
No four-legged contact/step capability or #151 budget is established here.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

BASE = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
sys.path.insert(0, str(BASE / 'orchestrator'))
import orchestrator as orch


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def gated_planar_velocity(velocity, yaw, frame, max_vz=1e-6):
    """Reject unsupported vertical motion before rotating map XY into body XY."""
    if frame != 'map' or len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
        return [0., 0., 0.], 'E_CARRIER_FRAME_OR_FINITE'
    if abs(velocity[2]) > max_vz:
        return [0., 0., 0.], 'E_CARRIER_VERTICAL_UNSUPPORTED'
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * velocity[0] + s * velocity[1], -s * velocity[0] + c * velocity[1], 0.], 'flat_supported'


def sensed_box(points):
    """Fixed experiment ROI, measured points only; padding represents uncertainty.

    This detects a newly introduced stationary object. It does not infer unseen
    box volume from simulator state or claim general object tracking.
    """
    selected = [p for p in points if .55 <= p[0] <= 1.45 and abs(p[1]) <= .65 and .05 <= p[2] <= .95]
    if len(selected) < 4:
        return None, selected
    lo = [math.floor(min(p[i] for p in selected) * 10) / 10 - .12 for i in range(3)]
    hi = [math.ceil(max(p[i] for p in selected) * 10) / 10 + .12 for i in range(3)]
    lo[2] = 0.0  # supported object on the declared floor; no GT dimensions are read
    return {'min': lo, 'max': hi, 'velocity': [0., 0., 0.], 'stamp': 0.}, selected


def classify_measurement(case, wanted, first_zero, target, sense, status_rows=None, rest_wall=None):
    """Refusal chain must be causal and latched: an earlier zero, a missing cause,
    a non-target cause, or a nonzero command before rest is INVALID, not PASS."""
    if first_zero is None:
        return 'INVALID_NO_ZERO'
    if target is None:
        return 'INVALID_NO_TARGET_REASON'
    if first_zero.get('reason') != wanted:
        return 'INVALID_PREEMPTED_' + str(first_zero.get('reason'))
    if status_rows is not None and rest_wall is not None and any(
            r.get('stop') is False and first_zero.get('wall', 0.) < r.get('wall', 0.) <= rest_wall
            for r in status_rows):
        return 'INVALID_UNLATCHED'
    if case == 'sensed_no_route' and (sense is None or
                                      first_zero.get('wall', 0.) < sense.get('wall', 0.)):
        return 'INVALID_BEFORE_SENSING'
    return 'VALID'


TRANSIENT_ZEROS = {'E_LOCAL_NO_TRAJECTORY', 'E_LOCAL_EXPIRED', 'E_LOCAL_NOT_READY',
                   'E_LOCAL_INPUT', 'segment_complete', 'ok', 'hold'}


def first_terminal_zero(status_rows):
    """Skip replanning gaps; a terminal stop is the first non-transient zero."""
    return next((r for r in status_rows
                 if r.get('stop') and r.get('reason') not in TRANSIENT_ZEROS), None)


def adapter(prefix, source_topic):
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    rospy.init_node('a5_planar_adapter', anonymous=True, disable_signals=True)
    shared = {'command': None, 'command_at': -1e100, 'yaw': 0., 'pose_at': -1e100}
    lock = threading.Lock()
    out = rospy.Publisher(source_topic, Twist, queue_size=1)
    status = rospy.Publisher(prefix + '/adapter_status', String, queue_size=100)

    def command(msg):
        with lock:
            shared.update(command=msg, command_at=time.monotonic())

    def odometry(msg):
        with lock:
            shared.update(yaw=yaw_of(msg.pose.pose.orientation), pose_at=time.monotonic())

    rospy.Subscriber(prefix + '/cmd3d', TwistStamped, command, queue_size=1)
    rospy.Subscriber('/sim/odom_gt', Odometry, odometry, queue_size=1)
    try:
        deadline = time.monotonic() + 20
        while not out.get_num_connections():
            if time.monotonic() >= deadline:
                raise RuntimeError('planar adapter has no mux subscriber')
            time.sleep(.02)
        print('READY', flush=True)
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                state = dict(shared)
            msg = state['command']
            velocity, reason = [0., 0., 0.], 'E_CARRIER_INPUT_STALE'
            incoming = None
            if msg is not None and now - state['command_at'] < .4 and now - state['pose_at'] < 2.:
                incoming = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
                velocity, reason = gated_planar_velocity(incoming, state['yaw'], msg.header.frame_id)
            value = Twist()
            value.linear.x, value.linear.y, value.angular.z = velocity
            out.publish(value)
            status.publish(String(data=json.dumps({'wall': now, 'sim': rospy.Time.now().to_sec(),
                                                   'velocity_body': velocity, 'incoming_xyz': incoming,
                                                   'reason': reason})))
            time.sleep(.05)
    finally:
        out.publish(Twist())


class OwnedChild:
    """Inherit the probe's process group so outer lab timeout owns the whole tree.

    Local cleanup targets only this direct child. Sending a group signal here
    would also signal the probe; only the outer runtime_lab owns that group.
    Both commands launched below are Python nodes with threads, no descendants.
    """
    def __init__(self, spec, command, ready_marker=None, stderr=None):
        self.id, self.command = spec.id, command
        self.marker, self.stderr = ready_marker, stderr
        self.proc = None
        self.seen = ready_marker is None
        self.reader = None

    def start(self):
        self.proc = subprocess.Popen(self.command, stdout=subprocess.PIPE,
                                     stderr=self.stderr or subprocess.DEVNULL,
                                     start_new_session=False)
        def drain():
            pending = b''
            while True:
                chunk = self.proc.stdout.read1(4096)
                if not chunk:
                    return
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if self.marker and self.marker.encode() in line:
                        self.seen = True
                pending = pending[-65536:]
        self.reader = threading.Thread(target=drain, daemon=True)
        self.reader.start()
        return self

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def ready(self):
        return self.is_alive() and self.seen

    def stop(self, _method):
        if self.is_alive():
            self.proc.terminate()

    def reap(self):
        if self.proc is None:
            return
        if self.is_alive():
            self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=3)
        if self.reader is not None:
            self.reader.join(timeout=1)
        self.proc.stdout.close()


class Probe:
    def __init__(self, args):
        import rospy
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool, String
        self.rospy, self.args = rospy, args
        self.Pose, self.Bool, self.String = PoseStamped, Bool, String
        self.prefix = '/a5_gazebo_%d' % os.getpid()
        self.model_name = 'a5_sensor_box_%d' % os.getpid()
        self.lock = threading.Lock()
        self.odom, self.cloud = None, None
        self.cloud_seq = 0
        self.phase = 'initializing'
        self.detect_enabled = False
        self.obstacle = None
        self.last_cloud_published = -1
        self.latest_status = {}
        self.events, self.cases = [], []
        self.raw = {key: deque(maxlen=60000) for key in ('physics', 'cmd3d', 'cmd_mux', 'status', 'adapter', 'clouds', 'plans', 'inputs')}
        self.pose_pub = rospy.Publisher(self.prefix + '/pose', PoseStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(self.prefix + '/goal', PoseStamped, queue_size=1, latch=True)
        self.obstacle_pub = rospy.Publisher(self.prefix + '/obstacles', String, queue_size=1)
        self.loc_pub = rospy.Publisher(self.prefix + '/localization', Bool, queue_size=1)
        self.hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)
        self.subs = [
            rospy.Subscriber('/sim/odom_gt', Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber('/rslidar_points', PointCloud2, self.on_cloud, queue_size=1),
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_models, queue_size=10),
            rospy.Subscriber('/cmd_vel', Twist, lambda m: self.capture('cmd_mux', {'xyz': [m.linear.x, m.linear.y, m.linear.z], 'wz': m.angular.z}), queue_size=100),
            rospy.Subscriber(self.prefix + '/cmd3d', TwistStamped,
                             lambda m: self.capture('cmd3d', {'xyz': [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z], 'frame': m.header.frame_id}), queue_size=100),
            rospy.Subscriber(self.prefix + '/status', String, self.on_status, queue_size=100),
            rospy.Subscriber(self.prefix + '/adapter_status', String,
                             lambda m: self.capture('adapter', json.loads(m.data)), queue_size=100),
            rospy.Subscriber(self.prefix + '/trajectory', String,
                             lambda m: self.capture('plans', json.loads(m.data)), queue_size=10)]
        self.runners = []
        self.node_streams = []
        self.node_logs = []
        self.hold_active = False

    def stamp(self):
        return {'wall': time.monotonic(), 'sim': self.rospy.Time.now().to_sec(), 'phase': self.phase}

    def capture(self, key, data):
        with self.lock:
            self.raw[key].append(dict(data, **self.stamp()))

    def rows(self, key, after=-1e100):
        with self.lock:
            return [r for r in self.raw[key] if r['wall'] >= after]

    def event(self, name, **data):
        self.events.append(dict(event=name, **self.stamp(), **data))
        return self.events[-1]

    def on_odom(self, message):
        with self.lock:
            self.odom = (message, time.monotonic())

    def on_status(self, message):
        value = json.loads(message.data)
        self.latest_status = value
        self.capture('status', value)

    def on_models(self, message):
        if 'lite3_planar' not in message.name:
            return
        index = message.name.index('lite3_planar')
        p, t = message.pose[index].position, message.twist[index]
        # Decimate solely for artifact size; never infer physical velocity from cmd.
        with self.lock:
            recent = [self.raw['physics'][-1]] if self.raw['physics'] else []
        now = self.stamp()
        if recent and now['sim'] - recent[0]['sim'] < .02 and now['wall'] - recent[0]['wall'] < .1:
            return
        self.capture('physics', {'xyz': [p.x, p.y, p.z],
                                'linear': [t.linear.x, t.linear.y, t.linear.z],
                                'angular': [t.angular.x, t.angular.y, t.angular.z]})

    def on_cloud(self, message):
        from sensor_msgs import point_cloud2 as pc2
        points = list(pc2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True))
        observed, roi = sensed_box(points) if self.detect_enabled else (None, [])
        received = self.stamp()
        with self.lock:
            self.cloud_seq += 1
            if observed is not None and self.obstacle is None:
                self.obstacle = observed
                self.events.append(dict(event='lidar_obstacle_detected', prediction=observed,
                                        sensor_stamp=message.header.stamp.to_sec(), **received))
            self.cloud = {'sequence': self.cloud_seq, 'received': received['wall'],
                          'sensor_stamp': message.header.stamp.to_sec(), 'obstacles': [self.obstacle] if self.obstacle else []}
            self.raw['clouds'].append(dict(sequence=self.cloud_seq, sensor_stamp=message.header.stamp.to_sec(),
                                           header_frame=message.header.frame_id, point_count=len(points),
                                           roi_point_count=len(roi), roi_points=[list(p) for p in roi[:300]],
                                           assumed_numeric_frame='map/world per fixed m1 R788', **received))

    def pump(self, localization=True, sensor_forward=True):
        with self.lock:
            odom, cloud = self.odom, dict(self.cloud) if self.cloud else None
        if odom is not None and time.monotonic() - odom[1] < 2.:
            source = odom[0]
            # Explicit flat-ground support adapter: this is a declared planar
            # contact plane, not a measured four-foot contact-state estimator.
            if abs(source.pose.pose.position.z - .112) > .065:
                raise RuntimeError('body height incompatible with declared flat support plane')
            pose = self.Pose()
            pose.header.frame_id, pose.header.stamp = 'map', source.header.stamp
            pose.pose.position.x, pose.pose.position.y = source.pose.pose.position.x, source.pose.pose.position.y
            pose.pose.position.z = 0.
            pose.pose.orientation = source.pose.pose.orientation
            self.pose_pub.publish(pose)
        self.loc_pub.publish(self.Bool(data=localization))
        if sensor_forward and cloud and cloud['sequence'] != self.last_cloud_published:
            payload = {'obstacles': cloud['obstacles']}
            self.obstacle_pub.publish(self.String(data=json.dumps(payload)))
            self.capture('inputs', {'kind': 'measured_lidar_obstacles', 'sensor_sequence': cloud['sequence'],
                                    'sensor_stamp': cloud['sensor_stamp'], 'payload': payload})
            self.last_cloud_published = cloud['sequence']

    def wait(self, predicate, label, timeout=None, localization=True, sensor_forward=True):
        deadline = time.monotonic() + (timeout or self.args.case_timeout)
        while time.monotonic() < deadline:
            self.pump(localization, sensor_forward)
            value = predicate()
            if value:
                return value
            if any(not r.is_alive() for r in self.runners):
                raise RuntimeError('owned planner/adapter process exited: ' + label)
            time.sleep(.02)
        raise RuntimeError('observation timeout: %s; status=%r' % (label, self.latest_status))

    def hold(self, value):
        self.hold_pub.publish(self.Bool(data=value))
        self.hold_active = value

    def reset(self):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import SetModelState
        self.hold(True)
        start = time.monotonic()
        self.wait(lambda: any(sum(abs(v) for v in r['xyz']) < 1e-6 for r in self.rows('cmd_mux', start)),
                  'mux zero before reset', timeout=15)
        state = ModelState(model_name='lite3_planar', reference_frame='world')
        state.pose.position.z, state.pose.orientation.w = .112, 1.
        result = self.rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
        if not result.success:
            raise RuntimeError(result.status_message)
        self.wait(lambda: self.rows('physics') and math.hypot(*self.rows('physics')[-1]['xyz'][:2]) < .025,
                  'physical pose reset', timeout=15)

    def start_owned(self, directory, case):
        value = json.loads((BASE / 'contracts/fixtures/map_single_floor.json').read_text())
        value.update(map_id='a5_gazebo_floor', resolution=.1, origin=[-1., -.65, 0.], extent={'width': 40, 'height': 13})
        value['surfaces'][0]['footprint'] = {'min_x': -1., 'max_x': 3., 'min_y': -.65, 'max_y': .65}
        map_path = directory / 'floor.json'
        map_path.write_text(json.dumps(value))
        self.map_fixture = value
        config = {'resolution': .1, 'tick_s': .5, 'max_speed_mps': .4, 'radius_m': .37,
                  'height_m': .25, 'clearance_m': .20, 'horizon_s': 6., 'validity_s': .9}
        self.execution_config = config
        node_args = [sys.executable, '-B', str(BASE / 'local_planner/ros_node.py'),
                     '__name:=a5_owned_local', '_map_json:=' + str(map_path), '_input_max_age_s:=2.0',
                     '_experimental_config:=' + json.dumps(config)]
        for parameter, topic in [('pose', 'pose'), ('goal', 'goal'), ('obstacles', 'obstacles'),
                                 ('localization', 'localization'), ('command', 'cmd3d'),
                                 ('trajectory', 'trajectory'), ('status', 'status')]:
            node_args.append('_%s_topic:=%s/%s' % (parameter, self.prefix, topic))
        stream = open(directory / (case + '.node.log'), 'wb')
        self.node_streams.append(stream)
        self.node_logs.append(str(directory / (case + '.node.log')))

        node_spec = orch.ComponentSpec('local', ['local_planner'], 'process_group_signal')
        node = OwnedChild(node_spec, node_args, stderr=stream)
        node.start()
        self.runners.append(node)
        adapter_spec = orch.ComponentSpec('adapter', ['controller'], 'process_group_signal')
        bridge = OwnedChild(adapter_spec, [sys.executable, '-B', str(SCRIPT), '--adapter',
                                          '--prefix', self.prefix, '--source-topic', '/follow/cmd_vel'], ready_marker='READY')
        bridge.start()
        self.runners.append(bridge)
        self.wait(lambda: all(pub.get_num_connections() for pub in
                             [self.pose_pub, self.goal_pub, self.obstacle_pub, self.loc_pub]) and bridge.ready(),
                  'owned node subscriptions and adapter readiness', timeout=30)
        self.global_route = None
        self.route_goals = [[2., 0., 0.]]
        self.route_goal_index = 0
        if self.args.global_route and case == 'arrival':
            sys.path.insert(0, str(BASE / 'regression'))
            from a4_runtime import call_node
            route, error, info = call_node(value, (0., 0., 0.), (2., 0., 0.), 2000,
                                           'a4_gazebo_global_%d' % os.getpid(),
                                           min_clearance_m=.20)
            if route is None:
                raise RuntimeError('global ROS planner refused: %s / %r' % (error, info))
            self.global_route = {'points': route, 'status': info,
                                 'source': 'real ROS GetPlan response'}
            self.route_goals = list(route[4::4])
            if not self.route_goals or self.route_goals[-1] != route[-1]:
                self.route_goals.append(route[-1])
        self.publish_route_goal()

    def publish_route_goal(self):
        goal = self.Pose()
        goal.header.frame_id, goal.pose.orientation.w = 'map', 1.
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.route_goals[self.route_goal_index]
        self.goal_pub.publish(goal)
        self.event('global_route_goal_submitted' if self.global_route else 'local_goal_submitted',
                   xyz=list(self.route_goals[self.route_goal_index]))

    def arrived_along_route(self):
        rows = self.rows('physics')
        if not rows:
            return False
        target = self.route_goals[self.route_goal_index]
        if math.dist(rows[-1]['xyz'][:2], target[:2]) > .09:
            return False
        if self.route_goal_index < len(self.route_goals) - 1:
            self.route_goal_index += 1
            self.publish_route_goal()
            return False
        return True

    def moving(self, after):
        rows = self.rows('physics', after)
        return rows[-1] if rows and math.hypot(*rows[-1]['linear'][:2]) > .03 and rows[-1]['xyz'][0] > .025 else None

    def stable_stop(self, after):
        rows = self.rows('physics', after)
        if len(rows) < 4:
            return None
        tail = [r for r in rows if rows[-1]['sim'] - r['sim'] <= .3]
        if len(tail) < 4 or tail[-1]['sim'] - tail[0]['sim'] < .2:
            return None
        if any(math.hypot(*r['linear'][:2]) > .01 or abs(r['angular'][2]) > .01 for r in tail):
            return None
        if max(math.dist(r['xyz'][:2], tail[0]['xyz'][:2]) for r in tail) > .005:
            return None
        return tail[0]

    def first_stable_stop(self, after, status_after=None):
        """First completed 0.3 s sim rest window; later nonzero commands disqualify it."""
        rows = self.rows('physics', after)
        status = self.rows('status', status_after if status_after is not None else after)
        for i, first in enumerate(rows):
            tail = [r for r in rows[i:] if r['sim'] - first['sim'] <= .3]
            if len(tail) < 4 or tail[-1]['sim'] - first['sim'] < .2:
                continue
            if any(math.hypot(*r['linear'][:2]) > .01 or abs(r['angular'][2]) > .01 for r in tail):
                continue
            if max(math.dist(r['xyz'][:2], first['xyz'][:2]) for r in tail) > .005:
                continue
            if any(r.get('stop') is False and first['wall'] < r['wall'] <= tail[-1]['wall']
                   for r in status):
                continue
            return first
        return None

    def spawn(self):
        from gazebo_msgs.srv import SpawnModel, GetModelState
        from geometry_msgs.msg import Pose
        # Only the evaluator knows this geometry; perception receives lidar points.
        xml = ('<sdf version="1.6"><model name="%s"><static>true</static><link name="body">'
               '<collision name="collision"><geometry><box><size>.4 .4 .6</size></box></geometry></collision>'
               '<visual name="visual"><geometry><box><size>.4 .4 .6</size></box></geometry></visual>'
               '</link></model></sdf>') % self.model_name
        pose = Pose()
        pose.position.x, pose.position.z, pose.orientation.w = 1., .3, 1.
        self.detect_enabled = True
        event = self.event('spawn_requested', model=self.model_name)
        response = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(self.model_name, xml, '', pose, 'world')
        if not response.success:
            raise RuntimeError(response.status_message)
        truth = self.rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(self.model_name, 'world')
        if not truth.success or abs(truth.pose.position.x - 1.) > .01:
            raise RuntimeError('spawn initial_pose was not honored')
        self.event('spawn_confirmed_GT_evaluation_only', xyz=[truth.pose.position.x, truth.pose.position.y, truth.pose.position.z])
        return event

    def delete_box(self):
        from gazebo_msgs.srv import DeleteModel
        self.rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(self.model_name)
        self.detect_enabled = False
        self.obstacle = None

    def stop_owned(self):
        self.hold(True)
        for runner in reversed(self.runners):
            runner.stop('process_group_signal')
            runner.reap()
        self.runners = []
        # Do not release hold while the mux may still cache a nonzero candidate.
        expiry_start = self.rospy.Time.now().to_sec()
        self.wait(lambda: self.rospy.Time.now().to_sec() - expiry_start > .6,
                  'mux candidate cache expiry', timeout=30)

    def run_case(self, case, directory):
        self.phase = case
        self.latest_status, self.detect_enabled, self.obstacle = {}, False, None
        self.reset()
        self.start_owned(directory, case)
        began = self.event('case_ready', case=case)
        self.hold(False)
        moving = self.wait(lambda: self.moving(began['wall']), 'physical motion before ' + case)
        if case == 'arrival':
            self.wait(self.arrived_along_route,
                      'physical arrival at map/(2,0,0)')
            trigger = self.event('arrival_geometric_tolerance')
            stopped = self.wait(lambda: self.stable_stop(trigger['wall']), 'arrival physical stop')
            last = self.rows('physics')[-1]
            self.cases.append({'case': case, 'ok': math.dist(last['xyz'][:2], (2., 0.)) <= .10,
                               'measurement': 'VALID',
                               'actual_final_body_xyz': last['xyz'], 'goal_support_xyz': [2., 0., 0.],
                               'global_route': self.global_route,
                               'global_waypoints_consumed': self.route_goal_index + 1 if self.global_route else None,
                               'support_adapter': 'measured body XY + declared flat support z=0',
                               'stop_sim': stopped['sim']})
        else:
            if case == 'sensed_no_route':
                trigger = self.spawn()
                self.wait(lambda: self.obstacle, 'real lidar ROI detection')
                wanted, localization, sensor_forward = 'E_LOCAL_NO_ROUTE', True, True
            else:
                trigger = self.event('localization_false' if case == 'localization' else 'drop_obstacle_forwarding')
                wanted = 'E_LOCAL_LOCALIZATION' if case == 'localization' else 'E_LOCAL_OBSTACLES_STALE'
                localization, sensor_forward = case != 'localization', case != 'sensor_freshness'
            first_zero = self.wait(
                lambda: first_terminal_zero(self.rows('status', trigger['wall'])),
                'first terminal zero after trigger', localization=localization, sensor_forward=sensor_forward)
            transient_zeros = [r for r in self.rows('status', trigger['wall'])
                               if r.get('stop') and r.get('reason') in TRANSIENT_ZEROS
                               and r['wall'] <= first_zero['wall']]
            mux = self.wait(
                lambda: next((r for r in self.rows('cmd_mux', first_zero['wall'])
                              if max(map(abs, r['xyz'])) <= 1e-6), None),
                'mux zero after first zero', localization=localization, sensor_forward=sensor_forward)
            target = None
            target_deadline = time.monotonic() + 2.
            while time.monotonic() < target_deadline:
                self.pump(localization, sensor_forward)
                target = next((r for r in self.rows('status', trigger['wall'])
                               if r.get('reason') == wanted), None)
                if target is not None:
                    break
                time.sleep(.02)
            status = self.rows('status', trigger['wall'])
            latch_end = target['wall'] if target is not None else first_zero['wall'] + 1.
            nonzero = [r for r in status
                       if first_zero['wall'] <= r['wall'] <= latch_end and r.get('stop') is False]
            latch_wall = max([r['wall'] for r in nonzero], default=first_zero['wall'])
            while True:
                stopped = self.wait(
                    lambda: self.first_stable_stop(latch_wall, status_after=latch_wall),
                    'first physical rest after latch', localization=localization,
                    sensor_forward=sensor_forward)
                status = self.rows('status', trigger['wall'])
                late = [r for r in status
                        if first_zero['wall'] <= r['wall'] <= latch_end and r.get('stop') is False]
                if not late:
                    break
                new_latch = max(r['wall'] for r in late)
                if new_latch <= latch_wall:
                    break
                latch_wall = new_latch
            sense = next((r for r in self.rows('inputs', trigger['wall'])
                          if r.get('kind') == 'measured_lidar_obstacles'
                          and r.get('payload', {}).get('obstacles')), None)
            measurement = classify_measurement(case, wanted, first_zero, target, sense,
                                               status_rows=status, rest_wall=stopped['wall'])

            def path_between(start_wall, end_wall):
                seg = [r for r in self.rows('physics', start_wall) if r['wall'] <= end_wall]
                return sum(math.dist(a['xyz'][:2], b['xyz'][:2]) for a, b in zip(seg, seg[1:]))

            truth = self.rows('physics', trigger['wall'])
            min_box_gap = None
            if case == 'sensed_no_route':
                # Independent known-spawn-geometry metric; never fed to planner.
                min_box_gap = min(math.hypot(max(abs(r['xyz'][0] - 1.) - .2, 0.),
                                             max(abs(r['xyz'][1]) - .2, 0.)) - self.execution_config['radius_m'] for r in truth)
            self.cases.append({'case': case,
                               'ok': (min_box_gap is None or min_box_gap >= self.execution_config['clearance_m'])
                                     and measurement == 'VALID',
                               'measurement': measurement,
                               'required_body_clearance_m': self.execution_config['clearance_m'],
                               'reason': wanted, 'speed_before_m_s': math.hypot(*moving['linear'][:2]),
                               'trigger': trigger, 'refusal': first_zero,
                               'target_reason': target, 'first_obstacle_input': sense,
                               'transient_zeros_before_stop': transient_zeros,
                               'latch_wall_s': latch_wall - trigger['wall'],
                               'trigger_to_first_zero_wall_s': first_zero['wall'] - trigger['wall'],
                               'trigger_to_first_zero_sim_s': first_zero['sim'] - trigger['sim'],
                               'trigger_to_target_reason_wall_s':
                                   (target['wall'] - trigger['wall']) if target else None,
                               'trigger_to_3d_zero_wall_s': first_zero['wall'] - trigger['wall'],
                               'trigger_to_mux_zero_wall_s': mux['wall'] - trigger['wall'],
                               'trigger_to_physical_stop_wall_s': stopped['wall'] - trigger['wall'],
                               'trigger_to_physical_stop_sim_s': stopped['sim'] - trigger['sim'],
                               'stop_from_sense_wall_s':
                                   (stopped['wall'] - sense['wall']) if sense else None,
                               'stop_from_sense_sim_s':
                                   (stopped['sim'] - sense['sim']) if sense else None,
                               'stop_distance_from_sense_m':
                                   path_between(sense['wall'], stopped['wall']) if sense else None,
                               'stop_distance_from_trigger_m':
                                   path_between(trigger['wall'], stopped['wall']),
                               'GT_min_radial_box_clearance_m': min_box_gap})
        self.stop_owned()
        if case == 'sensed_no_route':
            self.delete_box()
        # A new goal is published per case, so old state is never reused as progress.
        self.event('case_clean', case=case)

    def run(self, directory):
        import rosgraph
        import tf2_ros
        publishers = dict(rosgraph.Master(self.prefix).getSystemState()[0])
        if publishers.get('/cmd_vel') != ['/mux'] or publishers.get('/follow/cmd_vel'):
            raise RuntimeError('command/source ownership precondition failed')
        self.wait(lambda: self.odom and self.cloud and self.rows('physics') and self.hold_pub.get_num_connections(),
                  'real odometry, lidar, model states and mux', timeout=40)
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        def identity_frame():
            try:
                tr = buffer.lookup_transform('map', self.odom[0].header.frame_id,
                                             self.rospy.Time(0), self.rospy.Duration(0))
            except Exception:
                return False
            t, q = tr.transform.translation, tr.transform.rotation
            if math.sqrt(t.x*t.x + t.y*t.y + t.z*t.z) > 1e-6 or max(abs(q.x), abs(q.y), abs(q.z)) > 1e-6:
                raise RuntimeError('expected m1 identity map←odom bridge is not identity')
            return True
        self.wait(identity_frame, 'verified identity map←odometry frame', timeout=15)
        self.event('frame_verified', odometry_frame=self.odom[0].header.frame_id, map_identity=True)
        for service in ('/gazebo/set_model_state', '/gazebo/spawn_sdf_model', '/gazebo/delete_model'):
            self.rospy.wait_for_service(service, timeout=15)
        for case in self.args.cases.split(','):
            self.run_case(case, directory)

    def close(self):
        self.stop_owned()
        if self.detect_enabled:
            self.delete_box()
        self.hold(False)
        for stream in self.node_streams:
            stream.close()
        for subscriber in self.subs:
            subscriber.unregister()

    def record(self, error=None):
        return {'ok': error is None and bool(self.cases) and all(c['ok'] for c in self.cases),
                'measurement_valid': error is None and all(
                    c.get('measurement') == 'VALID' for c in self.cases),
                'measurement_rule': 'first zero/cause after trigger; sensed_no_route requires '
                                    'first zero after first obstacle input; preemption is INVALID',
                'error': error, 'cases': self.cases, 'events': self.events,
                'domain': 'native_3D_planner_flat_explicitly_gated_planar_Gazebo_dynamics',
                'hardware': False, 'four_foot_contact_or_cross_layer': False, 'budget_151_frozen': False,
                'perception': 'actual /rslidar_points numeric world coordinates, fixed ROI, measured AABB + 0.12m uncertainty',
                'known_floor': getattr(self, 'map_fixture', None),
                'execution_config': getattr(self, 'execution_config', None),
                'node_stderr': {Path(p).name: Path(p).read_text(errors='replace')[-12000:]
                                for p in self.node_logs if Path(p).exists()},
                'timebase': 'wall=time.monotonic receipt/trigger; sim=current /clock receipt; sensor_stamp from PointCloud2 header',
                'raw': {key: self.rows(key) for key in self.raw}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out')
    parser.add_argument('--adapter', action='store_true')
    parser.add_argument('--prefix', default='/a5_gazebo')
    parser.add_argument('--source-topic', default='/follow/cmd_vel')
    parser.add_argument('--cases', default='arrival,sensed_no_route,localization,sensor_freshness')
    parser.add_argument('--case-timeout', type=float, default=100.)
    parser.add_argument('--global-route', action='store_true')
    args = parser.parse_args()
    allowed = {'arrival', 'sensed_no_route', 'localization', 'sensor_freshness'}
    if any(case not in allowed for case in args.cases.split(',')):
        parser.error('--cases contains an unsupported scenario')
    if args.adapter:
        adapter(args.prefix, args.source_topic)
        return 0
    import rospy
    rospy.init_node('a5_gazebo_probe', anonymous=True, disable_signals=True)
    probe = Probe(args)
    error = None
    with tempfile.TemporaryDirectory(prefix='v8_a5_gazebo_') as work:
        try:
            probe.run(Path(work))
        except Exception as exc:
            error = repr(exc)
        finally:
            try:
                probe.close()
            except Exception as exc:
                error = '%s; cleanup=%r' % (error, exc)
        result = probe.record(error)
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.out:
        Path(args.out).write_text(encoded)
    print(json.dumps({k: result[k] for k in ('ok', 'error', 'cases', 'domain')}))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    sys.exit(main())
