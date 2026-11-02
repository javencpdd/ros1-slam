#!/usr/bin/env python3
"""Live A6 NDT recovery: loss -> bounded NDT relocalize -> candidate -> seed
-> new lease, with the localization lease gate rejecting commands throughout.

The NDT monitor is fed through a pausable relay so loss is a real absence of
observations, not an injected flag.  Backend application is the NDT native
`~initialpose` seed; the A6 supervisor never receives Gazebo truth.  Gazebo
truth is used only for the post-recovery XY gate and to report the candidate's
distance from truth.  The certified slice is the static-loss case; `--move`
is exploratory because a nearest-prior candidate may stay at the prior.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rospy
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from candidate_consensus import CandidateConsensus  # noqa: E402

SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
PCD = '/opt/ros1-slam/data/maps/m1_world_synth_v1.pcd'
BBS_RESOLUTIONS = ((0.5, 'res05'), (0.2, 'res02'), (0.1, 'res01'))
PORT, GAZEBO_PORT = 11950, 11951
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/a6_ndt_ros',
                  ROS_LOG_DIR='/tmp/a6_ndt_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/a6_ndt_ros').mkdir(exist_ok=True)
Path('/tmp/a6_ndt_logs').mkdir(exist_ok=True)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def wait_until(predicate, timeout, label, diagnostic=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    if diagnostic is not None:
        print('DIAG ' + json.dumps(diagnostic(), ensure_ascii=False, default=str)[:6000],
              flush=True)
    raise RuntimeError('timeout waiting for ' + label)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def read_pcd(path):
    with open(path, 'rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header incomplete')
            header.append(line)
            if line.startswith(b'DATA'):
                break
        fields = [line.split()[1:] for line in header if line.startswith(b'FIELDS')][0]
        sizes = [int(x) for x in [line.split() for line in header
                                  if line.startswith(b'SIZE')][0][1:]]
        points = int([line.split()[1] for line in header if line.startswith(b'POINTS')][0])
        data = stream.read()
    dtype = np.dtype([(fields[i].decode(), 'f%d' % sizes[i]) for i in range(len(fields))])
    return np.frombuffer(data, dtype=dtype, count=points)


def voxel_downsample(points, leaf):
    if len(points) == 0:
        return points
    cells = np.floor(points / leaf).astype(np.int64)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=float)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse)
    return sums / counts[:, None]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spawn', default='1.5,0')
    parser.add_argument('--move', default='0,0')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--gate-m', type=float, default=0.3)
    parser.add_argument('--deadline-s', type=float, default=25.0)
    parser.add_argument('--relocalize-budget-s', type=float, default=8.0)
    parser.add_argument('--search-resume-delay-s', type=float, default=.5)
    parser.add_argument('--negative', action='store_true')
    parser.add_argument('--consensus', action='store_true',
                        help='require independent 3-resolution BBS consensus before lease restore')
    parser.add_argument('--consensus-gate-m', type=float, default=.5)
    parser.add_argument('--consensus-radius', type=float, default=.6)
    parser.add_argument('--consensus-scan-queries', type=int, default=1)
    parser.add_argument('--consensus-scan-downsample', type=float, default=.3)
    parser.add_argument('--expect-fail-closed', action='store_true',
                        help='the cycle must end FAILED without a new lease/reset')
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
    from hdl_global_localization.srv import QueryGlobalLocalization, SetGlobalMap
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String, UInt64

    spawn_x, spawn_y = (float(v) for v in args.spawn.split(','))
    move_x, move_y = (float(v) for v in args.move.split(','))
    master = gazebo = ndt = bridge = a6 = gate = None
    consensus_gate_proc = None
    bbs_nodes = []
    relay = {'paused': False, 'clouds': 0}
    status = {'phase': None, 'allowed': False, 'lease': None, 'backend': None,
              'calls': 0, 'report': None}
    observations = {'odom': None, 'truth': None, 'gate_status': None,
                    'gate_cmd': None, 'gate_cmds': 0, 'health': None,
                    'relocalized': 0, 'backend': None, 'resets': [],
                    'recent': deque(maxlen=max(1, args.consensus_scan_queries)),
                    'consensus': None, 'consensus_events': 0}
    lock = threading.RLock()
    callbacks = []

    def diagnostic():
        return {'status': status, 'observations': observations,
                'relay': relay}

    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], check=True,
                       env=dict(os.environ))
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('a6_ndt_recovery_live_smoke', anonymous=True, disable_signals=True)
        rospy.wait_for_message('/clock', Clock, timeout=90)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=90)
        rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        def place(x, y):
            state = ModelState(model_name='lite3_planar', reference_frame='world')
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, .112
            state.pose.orientation.w = 1.
            set_state(state)

        place(spawn_x, spawn_y)
        time.sleep(2.0)

        if args.consensus:
            cloud_points = read_pcd(PCD)
            xyz = np.stack([cloud_points['x'], cloud_points['y'],
                            cloud_points['z']], axis=1).astype(np.float32)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            header = rospy.Header()
            header.frame_id, header.stamp = 'map', rospy.Time.now()
            global_cloud = pc2.create_cloud_xyz32(header, xyz)
            config_dir = (Path('/opt/ros1-slam/ws/src/20_localization/'
                               'hdl_global_localization/config'))
            for resolution, name in BBS_RESOLUTIONS:
                for config_name in ('general_config.yaml', 'bbs_config.yaml'):
                    for key, value in yaml.safe_load(
                            (config_dir / config_name).read_text()).items():
                        rospy.set_param('/%s/%s' % (name, key), value)
                rospy.set_param('/%s/bbs/map_min_z' % name, .05)
                rospy.set_param('/%s/bbs/map_max_z' % name, 1.0)
                rospy.set_param('/%s/bbs/scan_min_z' % name, -.5)
                rospy.set_param('/%s/bbs/scan_max_z' % name, .6)
                rospy.set_param('/%s/bbs/map_resolution' % name, resolution)
                rospy.set_param('/%s/globalmap_downsample_resolution' % name, .2)
                rospy.set_param('/%s/query_downsample_resolution' % name, .2)
                node = subprocess.Popen(['rosrun', 'hdl_global_localization',
                                         'hdl_global_localization_node', '__name:=' + name],
                                        start_new_session=True,
                                        stdout=open('/tmp/a6_ndt_bbs_%s.log' % name, 'ab'),
                                        stderr=subprocess.STDOUT, env=dict(os.environ))
                bbs_nodes.append(node)
                for service in ('/%s/set_global_map' % name, '/%s/query' % name):
                    rospy.wait_for_service(service, timeout=30)
                rospy.ServiceProxy('/%s/set_global_map' % name, SetGlobalMap)(global_cloud)

        ndt = subprocess.Popen(['rosrun', 'v8_ndt_odom', 'ndt_odom_node', '__name:=a6_ndt',
                                '_map_path:=' + PCD, '_cloud_topic:=/a6_ndt/cloud',
                                '_resolution:=0.5', '_score_max:=30.0',
                                '_ext_z:=0.34', '_ext_pitch:=0.2617993878',
                                '_init_x:=%s' % spawn_x, '_init_y:=%s' % spawn_y,
                                '_init_yaw_deg:=0', '_global_search:=false',
                                '_relocalize_radius_m:=0.4', '_relocalize_xy_step_m:=0.2',
                                '_relocalize_yaw_span_deg:=10',
                                '_relocalize_yaw_step_deg:=5',
                                '_relocalize_budget_s:=%s' % args.relocalize_budget_s,
                                '_relocalize_score_max:=15.0',
                                '_relocalize_max_displacement_m:=0.6'],
                               start_new_session=True,
                               stdout=open('/tmp/a6_ndt_node.log', 'ab'),
                               stderr=subprocess.STDOUT, env=dict(os.environ))
        def service_ready(name):
            try:
                rospy.wait_for_service(name, timeout=.5)
                return True
            except rospy.ROSException:
                return False

        wait_until(lambda: service_ready('/a6_ndt/relocalize'), 30, 'NDT relocalize service')
        wait_until(lambda: service_ready('/a6_ndt/reset'), 30, 'NDT reset service')

        candidate_topic = ('/a6/recovery/candidate_raw' if args.consensus
                           else '/a6/recovery/candidate')
        bridge = subprocess.Popen([sys.executable, '-B', str(HERE / 'ndt_recovery_bridge.py'),
                                   '--odom-topic', '/a6_ndt/odom',
                                   '--health-topic', '/a6_ndt/healthy',
                                   '--score-topic', '/a6_ndt/score',
                                   '--relocalized-topic', '/a6_ndt/relocalized',
                                   '--candidate-topic', candidate_topic,
                                   '--fitness-max', '15.0'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_bridge.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        if args.consensus:
            consensus_gate_proc = subprocess.Popen(
                [sys.executable, '-B', str(HERE / 'consensus_gate.py'),
                 '--input-topic', '/a6/recovery/candidate_raw',
                 '--consensus-topic', '/a6/consensus/candidate',
                 '--output-topic', '/a6/recovery/candidate',
                 '--radius', str(args.consensus_gate_m),
                 '--max-age-s', '8.0'],
                start_new_session=True,
                stdout=open('/tmp/a6_ndt_consensus_gate.log', 'ab'),
                stderr=subprocess.STDOUT, env=dict(os.environ))
        a6_argv = [sys.executable, '-B', str(HERE / 'a6_recovery_node.py'),
                   '--namespace', '/a6/recovery',
                   '--odom-topic', '/a6_ndt/odom',
                   '--candidate-topic', '/a6/recovery/candidate',
                   '--relocalize-service', '/a6_ndt/relocalize',
                   '--relocalize-service-type', 'trigger',
                   '--backend-name', 'ndt',
                   '--initial-lease', '1',
                   '--deadline-s', str(args.deadline_s),
                   '--timeout-s', '1.0',
                   '--score-min', '0.5',
                   '--bounds=-3,8,-5,5']
        if args.consensus:
            a6_argv.extend(['--require-candidate-source', 'bbs_consensus',
                            '--max-agreement-m', str(args.consensus_gate_m)])
        a6 = subprocess.Popen(a6_argv,
                              start_new_session=True,
                              stdout=open('/tmp/a6_ndt_recovery.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
        gate = subprocess.Popen([sys.executable, '-B', str(HERE / 'localization_state.py'),
                                 '--namespace', '/a6/gate',
                                 '--odom-topic', '/a6_ndt/odom',
                                 '--lease-topic', '/a6/recovery/lease',
                                 '--candidate-topic', '/a6/gate/candidate',
                                 '--output-topic', '/a6/gate/cmd_vel',
                                 '--timeout-s', '1.0',
                                 '--command-timeout-s', '0.3',
                                 '--keep-alive-s', '180'],
                                start_new_session=True,
                                stdout=open('/tmp/a6_ndt_gate.log', 'ab'),
                                stderr=subprocess.STDOUT, env=dict(os.environ))

        consensus_busy = {'busy': False}
        consensus_pub = rospy.Publisher('/a6/consensus/candidate', String, queue_size=5)

        def query_consensus():
            try:
                scans = list(observations['recent'])[-args.consensus_scan_queries:]
                samples, errors = [], []
                for scan_index, points_now in enumerate(scans):
                    header_now = rospy.Header(stamp=rospy.Time.now(), frame_id='base_link')
                    query_cloud = pc2.create_cloud_xyz32(
                        header_now, voxel_downsample(points_now,
                                                     args.consensus_scan_downsample))
                    for resolution, name in BBS_RESOLUTIONS:
                        try:
                            response = rospy.ServiceProxy(
                                '/%s/query' % name,
                                QueryGlobalLocalization)(5, query_cloud)
                            if response.poses:
                                pose = response.poses[0]
                                q = pose.orientation
                                samples.append({
                                    'source': resolution, 'scan': scan_index,
                                    'pose': (pose.position.x, pose.position.y,
                                             pose.position.z),
                                    'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                      1 - 2 * (q.y * q.y + q.z * q.z))})
                        except Exception as error:
                            errors.append('%s:%d:%r' % (name, scan_index, error))
                accepted = CandidateConsensus(
                    radius=args.consensus_radius, min_support=2).select(samples)
                yaw = None
                if accepted is not None:
                    yaw = next((item['yaw'] for item in samples
                                if item['source'] == accepted['source']), 0.0)
                    score = accepted['support'] / float(len(BBS_RESOLUTIONS))
                    consensus_pub.publish(String(data=json.dumps({
                        'pose': [accepted['pose'][0], accepted['pose'][1], 0.0],
                        'yaw': yaw, 'score': score, 'source': 'bbs_consensus'})))
                with lock:
                    observations['consensus'] = {
                        'accepted': accepted, 'yaw': yaw, 'samples': samples,
                        'errors': errors, 'scans': len(scans)}
            finally:
                consensus_busy['busy'] = False

        def on_cloud(message):
            if args.consensus:
                points_now = np.asarray(list(pc2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
                if len(points_now):
                    with lock:
                        observations['recent'].append(points_now)
            if not relay['paused']:
                relay['clouds'] += 1
                cloud_pub.publish(message)

        def on_truth(message):
            names = list(message.name)
            if 'lite3_planar' not in names:
                return
            pose = message.pose[names.index('lite3_planar')]
            q = pose.orientation
            with lock:
                observations['truth'] = [pose.position.x, pose.position.y, pose.position.z,
                                         math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                    1 - 2 * (q.y * q.y + q.z * q.z))]

        def on_odom(message):
            q = message.pose.pose.orientation
            with lock:
                observations['odom'] = [message.pose.pose.position.x,
                                        message.pose.pose.position.y,
                                        message.pose.pose.position.z,
                                        math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                   1 - 2 * (q.y * q.y + q.z * q.z))]

        def on_health(message):
            with lock:
                observations['health'] = bool(message.data)

        def on_relocalized(message):
            with lock:
                if message.data:
                    observations['relocalized'] += 1
                    if args.consensus and not consensus_busy['busy']:
                        consensus_busy['busy'] = True
                        threading.Thread(target=query_consensus, daemon=True).start()

        def on_gate_status(message):
            with lock:
                observations['gate_status'] = json.loads(message.data)

        def on_gate_cmd(message):
            with lock:
                observations['gate_cmd'] = [message.linear.x, message.angular.z]
                observations['gate_cmds'] += 1

        def on_a6_status(message):
            report = json.loads(message.data)
            status.update(report=report, phase=report.get('phase'),
                          calls=report.get('relocalize_calls', 0))

        def on_a6_allowed(message):
            status['allowed'] = bool(message.data)

        def on_a6_lease(message):
            status['lease'] = int(message.data)

        def on_backend(message):
            payload = json.loads(message.data)
            with lock:
                observations['backend'] = payload
            if payload.get('lease') in [item['lease'] for item in observations['resets']]:
                return
            pose = payload['pose']
            yaw = payload.get('yaw')
            yaw = 0.0 if yaw is None else float(yaw)
            reset = PoseWithCovarianceStamped()
            reset.header.stamp, reset.header.frame_id = rospy.Time.now(), 'map'
            reset.pose.pose.position.x, reset.pose.pose.position.y = float(pose[0]), float(pose[1])
            reset.pose.pose.position.z = 0.0
            reset.pose.pose.orientation.z = math.sin(yaw / 2.0)
            reset.pose.pose.orientation.w = math.cos(yaw / 2.0)
            initialpose_pub.publish(reset)
            with lock:
                observations['resets'].append({'lease': int(payload['lease']),
                                               'pose': list(pose), 'yaw': yaw})

        cloud_pub = rospy.Publisher('/a6_ndt/cloud', PointCloud2, queue_size=2)
        initialpose_pub = rospy.Publisher('/a6_ndt/initialpose',
                                          PoseWithCovarianceStamped, queue_size=5)
        gate_candidate = rospy.Publisher('/a6/gate/candidate', Twist, queue_size=10)
        callbacks.extend([
            rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=2),
            rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2),
            rospy.Subscriber('/a6_ndt/odom', Odometry, on_odom, queue_size=50),
            rospy.Subscriber('/a6_ndt/healthy', Bool, on_health, queue_size=20),
            rospy.Subscriber('/a6_ndt/relocalized', Bool, on_relocalized, queue_size=20),
            rospy.Subscriber('/a6/recovery/status', String, on_a6_status, queue_size=20),
            rospy.Subscriber('/a6/recovery/command_allowed', Bool, on_a6_allowed, queue_size=20),
            rospy.Subscriber('/a6/recovery/lease', UInt64, on_a6_lease, queue_size=10),
            rospy.Subscriber('/a6/recovery/backend', String, on_backend, queue_size=10),
            rospy.Subscriber('/a6/gate/status', String, on_gate_status, queue_size=20),
            rospy.Subscriber('/a6/gate/cmd_vel', Twist, on_gate_cmd, queue_size=20),
        ])
        stop_feed = threading.Event()

        def feed_candidate():
            # Keep a non-zero command offered even while lost: the gate, not the
            # command timeout, must be the reason the output drops to zero.
            while not stop_feed.is_set():
                gate_candidate.publish(Twist(linear=Vector3(x=.1)))
                time.sleep(.05)

        threading.Thread(target=feed_candidate, daemon=True).start()

        wait_until(lambda: observations['health'] is True and observations['odom'] is not None,
                   90, 'NDT initial tracking', diagnostic)
        wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                   and status['lease'] == 1, 30, 'A6 initial lease 1', diagnostic)
        wait_until(lambda: observations['gate_status'] is not None
                   and observations['gate_status'].get('active_lease') == 1,
                   20, 'gate lease 1', diagnostic)
        with lock:
            initial = {'odom': list(observations['odom']), 'truth': list(observations['truth']),
                       'health': observations['health'], 'lease': status['lease'],
                       'phase': status['phase'], 'allowed': status['allowed'],
                       'gate': dict(observations['gate_status'])}
        # The gate must pass the offered non-zero command only while healthy + leased.
        time.sleep(.4)
        with lock:
            gate_open = observations['gate_cmd']
        initial['gate_command'] = gate_open
        initial['ok'] = (gate_open is not None and abs(gate_open[0] - .1) < .01)

        cycles = []
        for index in range(args.cycles):
            expected = index + 2
            with lock:
                relay['paused'] = True
                calls_before = status['calls']
                resets_before = len(observations['resets'])
                gate_before = observations['gate_cmd']
            wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'RELOCALIZING', diagnostic)
            wait_until(lambda: status['calls'] > calls_before, 6, 'relocalize service call',
                       diagnostic)
            time.sleep(.7)
            with lock:
                gate_during_loss = observations['gate_cmd']
            time.sleep(max(0.0, args.search_resume_delay_s - .7))
            if move_x or move_y:
                place(spawn_x + move_x, spawn_y + move_y)
            with lock:
                relay['paused'] = False
            if args.expect_fail_closed:
                wait_until(lambda: status['phase'] == 'FAILED',
                           args.deadline_s + 5, 'expected fail-closed', diagnostic)
                time.sleep(2.0)
                with lock:
                    truth = list(observations['truth'])
                    odom = list(observations['odom'])
                    gate_now = observations['gate_status']
                    consensus = observations['consensus']
                    new_reset = len(observations['resets']) > resets_before
                cycles.append({
                    'expected_fail_closed': True,
                    'lease': status['lease'], 'lease_after': status['lease'],
                    'truth_xy': truth[:2], 'odom_xy': odom[:2],
                    'error_m': math.hypot(odom[0] - truth[0], odom[1] - truth[1]),
                    'new_reset': new_reset, 'consensus': consensus,
                    'gate': gate_now, 'gate_before_loss': gate_before,
                    'gate_during_loss': gate_during_loss,
                    'phase': status['phase'], 'allowed': status['allowed']})
                continue
            wait_until(lambda: len(observations['resets']) > resets_before,
                       args.deadline_s + 5, 'backend seed lease %d' % expected, diagnostic)
            wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                       and status['lease'] == expected, 20, 'TRACKING lease %d' % expected,
                       diagnostic)
            wait_until(lambda: observations['gate_status'] is not None
                       and observations['gate_status'].get('active_lease') == expected,
                       20, 'gate lease %d' % expected)
            time.sleep(1.5)
            with lock:
                odom = list(observations['odom'])
                truth = list(observations['truth'])
                reset = observations['resets'][-1]
                backend = observations['backend']
                relocalized = observations['relocalized']
                gate_now = observations['gate_status']
                consensus = observations['consensus']
            error = math.hypot(odom[0] - truth[0], odom[1] - truth[1])
            candidate_error = math.hypot(reset['pose'][0] - truth[0],
                                         reset['pose'][1] - truth[1])
            yaw_error = wrap(odom[3] - truth[3])
            cycles.append({'lease': expected, 'truth_xy': truth[:2], 'odom_xy': odom[:2],
                           'error_m': error, 'candidate_error_m': candidate_error,
                           'yaw_error_rad': yaw_error, 'reset': reset,
                           'backend': backend, 'relocalized_events': relocalized,
                           'gate': gate_now, 'gate_before_loss': gate_before,
                           'gate_during_loss': gate_during_loss,
                           'phase': status['phase'], 'allowed': status['allowed'],
                           'consensus': consensus})

        negative = None
        if args.negative:
            with lock:
                relay['paused'] = True
                calls_before = status['calls']
                lease_before = status['lease']
                resets_before = len(observations['resets'])
            wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'negative RELOCALIZING')
            wait_until(lambda: status['calls'] > calls_before, 6, 'negative relocalize call')
            wait_until(lambda: status['phase'] == 'FAILED', args.deadline_s + 3, 'negative FAILED')
            time.sleep(1.0)
            with lock:
                negative = {'phase': status['phase'], 'allowed': status['allowed'],
                            'lease_before': lease_before, 'lease_after': status['lease'],
                            'new_reset': len(observations['resets']) > resets_before,
                            'gate': observations['gate_status'],
                            'gate_cmd': observations['gate_cmd']}

        result = {
            'domain': 'a6_ndt_recovery_live_gazebo',
            'hardware': False,
            'spawn_xy': [spawn_x, spawn_y], 'move_xy': [move_x, move_y],
            'initial': initial, 'cycles': cycles, 'negative': negative,
            'relocalize_budget_s': args.relocalize_budget_s,
            'deadline_s': args.deadline_s, 'gate_m': args.gate_m,
            'consensus': args.consensus,
            'consensus_gate_m': args.consensus_gate_m if args.consensus else None,
            'expect_fail_closed': args.expect_fail_closed,
            'gate_rejection': {'gate_before_loss': initial.get('gate_command'),
                               'gate_during_loss': ([cycle['gate_during_loss']
                                                    for cycle in cycles]
                                                   if cycles else None)},
            'input_semantics': 'live Gazebo /rslidar_points through a pausable relay; '
                               'Gazebo truth used only for the post-recovery XY gate',
            'reset_path': 'A6 backend message -> NDT native /initialpose',
            'relocalize_path': 'A6 /relocalize trigger -> NDT bounded local search',
        }
        def cycle_ok(item):
            if item.get('expected_fail_closed'):
                return (item['phase'] == 'FAILED' and not item['allowed']
                        and not item['new_reset']
                        and item['lease_after'] == item['lease']
                        and item['gate_during_loss'] is not None
                        and abs(item['gate_during_loss'][0]) < 1e-9)
            return (item['error_m'] <= args.gate_m and item['phase'] == 'TRACKING'
                    and item['allowed'] and item['gate_during_loss'] is not None
                    and abs(item['gate_during_loss'][0]) < 1e-9)

        # The callbacks keep the latest gate output; sample it after loss above.
        result['ok'] = bool(
            initial['ok'] and all(cycle_ok(item) for item in cycles)
            and (negative is None or (negative['phase'] == 'FAILED'
                                      and not negative['allowed']
                                      and negative['lease_after'] == negative['lease_before']
                                      and not negative['new_reset'])))
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        stop_feed.set() if 'stop_feed' in locals() else None
        for subscription in callbacks:
            subscription.unregister()
        for process in ([gate, a6, consensus_gate_proc, bridge, ndt] + bbs_nodes
                        + [gazebo, master]):
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
