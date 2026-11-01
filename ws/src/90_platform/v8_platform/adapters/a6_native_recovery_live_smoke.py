#!/usr/bin/env python3
"""Live A6 native recovery: loss -> 3-resolution BBS consensus -> hdl /initialpose
-> new lease -> TRACKING, plus a fail-closed rejection control.

The localization input is the sensor-local m1 scan relabelled base_link (mount
x/y offset zero, pitch z-only), voxel-downsampled at the same 0.3 m leaf as the
hdl nodelet.  Gazebo truth is used only to score the final /hdl_odom; it never
enters the candidate or reset path.

The smoke supplies the A6 node's `/relocalize` candidate source (three real BBS
instances + CandidateConsensus) and the backend switch adapter: on the A6 node's
`/a6/recovery/backend` message it applies the accepted pose to the real hdl
nodelet through the native `/initialpose` topic.  It does not call the hdl
`/relocalize` service; that single-candidate service is covered by R1078.
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'regression'))
from candidate_consensus import CandidateConsensus  # noqa: E402
from hdl_global_consensus_smoke import RESOLUTIONS, read_pcd  # noqa: E402

SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
TRACK_LAUNCH = Path('/opt/ros1-slam/ws/src/20_localization/lite3_localization/launch/hdl_m1_tracking_smoke.launch')
PCD = '/opt/ros1-slam/data/maps/m1_world_synth_v1.pcd'
PORT, GAZEBO_PORT = 11670, 11671
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/a6_native_ros',
                  ROS_LOG_DIR='/tmp/a6_native_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/a6_native_ros').mkdir(exist_ok=True)
Path('/tmp/a6_native_logs').mkdir(exist_ok=True)
DIAG = None


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def voxel_downsample(points, leaf):
    if len(points) == 0:
        return points
    cells = np.floor(points / leaf).astype(np.int64)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=float)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse)
    return sums / counts[:, None]


def wait_until(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.05)
    if DIAG is not None:
        print('DIAG ' + json.dumps(DIAG(), ensure_ascii=False, default=str)[:6000], flush=True)
    raise RuntimeError('timeout waiting for ' + label)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--spawn', default='1.5,0')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--gate-m', type=float, default=0.5)
    parser.add_argument('--deadline-s', type=float, default=20.0)
    parser.add_argument('--scan-downsample', type=float, default=0.3)
    parser.add_argument('--candidate-mode', choices=('fine', 'consensus'), default='fine')
    parser.add_argument('--scan-queries', type=int, default=2)
    parser.add_argument('--prior-radius', type=float, default=1.5)
    parser.add_argument('--settle-s', type=float, default=2.0)
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    import rospy
    import yaml
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from hdl_global_localization.srv import QueryGlobalLocalization, SetGlobalMap
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String, UInt64
    from std_srvs.srv import Empty, EmptyResponse

    spawn_x, spawn_y = (float(v) for v in args.spawn.split(','))
    master = gazebo = hdl = a6 = None
    bbs_nodes = []
    stop_feed = threading.Event()
    callbacks = []
    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/a6_native_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], check=True,
                       env=dict(os.environ))
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_native_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('a6_native_recovery_live_smoke', anonymous=True, disable_signals=True)
        rospy.wait_for_message('/clock', Clock, timeout=90)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=90)
        rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState(model_name='lite3_planar', reference_frame='world')
        state.pose.position.x, state.pose.position.y, state.pose.position.z = spawn_x, spawn_y, .112
        state.pose.orientation.w = 1.
        set_state(state)
        time.sleep(2.0)

        points = read_pcd(PCD)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        header = rospy.Header()
        header.frame_id, header.stamp = 'map', rospy.Time.now()
        global_cloud = pc2.create_cloud_xyz32(header, xyz)
        config_dir = HERE.parents[2] / '20_localization' / 'hdl_global_localization' / 'config'
        for resolution, name in RESOLUTIONS:
            for config_name in ('general_config.yaml', 'bbs_config.yaml'):
                values = yaml.safe_load((config_dir / config_name).read_text())
                for key, value in values.items():
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
                                    stdout=open('/tmp/a6_native_%s.log' % name, 'ab'),
                                    stderr=subprocess.STDOUT, env=dict(os.environ))
            bbs_nodes.append(node)
            for service in ('/%s/set_global_map' % name, '/%s/query' % name):
                rospy.wait_for_service(service, timeout=30)
            rospy.ServiceProxy('/%s/set_global_map' % name, SetGlobalMap)(global_cloud)

        hdl = subprocess.Popen(['roslaunch', str(TRACK_LAUNCH),
                                'init_x:=%s' % spawn_x, 'init_y:=%s' % spawn_y],
                               start_new_session=True,
                               stdout=open('/tmp/a6_native_hdl.log', 'ab'),
                               stderr=subprocess.STDOUT, env=dict(os.environ))
        shared = {'raw': None, 'local': None, 'truth': None, 'odom': None,
                  'paused': False, 'reject': False, 'calls': 0, 'last_query': None,
                  'resets': [], 'sent': 0, 'last_reset_lease': 0,
                  'recent': deque(maxlen=max(1, args.scan_queries))}
        lock = threading.RLock()
        status = {'phase': None, 'allowed': False, 'lease': None, 'backend': None, 'report': None}
        global DIAG
        DIAG = lambda: {'status': status, 'calls': shared['calls'], 'query': shared['last_query'],
                        'resets': shared['resets'], 'paused': shared['paused'],
                        'reject': shared['reject'], 'sent': shared['sent'],
                        'odom': shared['odom'], 'truth': shared['truth']}

        def on_raw(message):
            with lock:
                shared['raw'] = message

        def on_truth(message):
            names = list(message.name)
            if 'lite3_planar' not in names:
                return
            pose = message.pose[names.index('lite3_planar')]
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            with lock:
                shared['truth'] = [pose.position.x, pose.position.y, pose.position.z, yaw]

        def on_odom(message):
            q = message.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            with lock:
                shared['odom'] = [message.pose.pose.position.x, message.pose.pose.position.y,
                                  message.pose.pose.position.z, yaw]

        callbacks.extend([
            rospy.Subscriber('/rslidar_points', PointCloud2, on_raw, queue_size=2),
            rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2),
            rospy.Subscriber('/hdl_odom', Odometry, on_odom, queue_size=50),
            rospy.Subscriber('/a6/recovery/status', String,
                             lambda m: status.update(report=json.loads(m.data),
                                                     phase=json.loads(m.data).get('phase')),
                             queue_size=20),
            rospy.Subscriber('/a6/recovery/command_allowed', Bool,
                             lambda m: status.update(allowed=m.data), queue_size=20),
            rospy.Subscriber('/a6/recovery/lease', UInt64,
                             lambda m: status.update(lease=m.data), queue_size=10),
            rospy.Subscriber('/a6/recovery/backend', String,
                             lambda m: status.update(backend=json.loads(m.data)), queue_size=10)])
        cloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=2)
        leg_pub = rospy.Publisher('/leg_odom2', Odometry, queue_size=2, latch=True)
        initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                          queue_size=5, latch=True)
        candidate_pub = rospy.Publisher('/a6/recovery/candidate', String, queue_size=5)
        fields = [pc2.PointField(name=n, offset=4 * i, datatype=pc2.PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]

        def feed_loop():
            while not stop_feed.is_set() and not rospy.is_shutdown():
                leg = Odometry()
                leg.header.stamp = rospy.Time.now()
                leg.header.frame_id, leg.child_frame_id = 'map', 'base_link'
                leg.pose.pose.orientation.w = 1.
                leg_pub.publish(leg)
                with lock:
                    raw, paused = shared['raw'], shared['paused']
                if raw is not None and not paused and cloud_pub.get_num_connections():
                    points_now = np.asarray(list(pc2.read_points(
                        raw, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
                    if len(points_now):
                        message = pc2.create_cloud(
                            rospy.Header(stamp=rospy.Time.now(), frame_id='base_link'),
                            fields, points_now)
                        cloud_pub.publish(message)
                        with lock:
                            shared['local'] = message
                            shared['recent'].append(points_now.copy())
                            shared['sent'] += 1
                time.sleep(.2)

        def relocalize_cb(_request):
            with lock:
                shared['calls'] += 1
                cloud, reject = shared['local'], shared['reject']
                recent = list(shared['recent'])
                prior = None if shared['odom'] is None else [shared['odom'][0],
                                                             shared['odom'][1]]
                call_number = shared['calls']
            print('ADAPTER_CALL %d reject=%s' % (call_number, reject), flush=True)
            finest = min(resolution for resolution, _name in RESOLUTIONS)
            samples, errors, scans = [], [], []
            if cloud is not None:
                if reject:
                    scans = [np.zeros((5, 3))]
                elif recent:
                    scans = recent[-max(1, args.scan_queries):]
                else:
                    scans = [np.asarray(list(pc2.read_points(
                        cloud, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)]
                for scan_index, points_now in enumerate(scans):
                    header_now = rospy.Header(stamp=rospy.Time.now(), frame_id='base_link')
                    query_cloud = pc2.create_cloud_xyz32(
                        header_now, voxel_downsample(points_now, args.scan_downsample))
                    for resolution, name in RESOLUTIONS:
                        if args.candidate_mode == 'fine' and resolution != finest:
                            continue
                        try:
                            response = rospy.ServiceProxy(
                                '/%s/query' % name, QueryGlobalLocalization)(5, query_cloud)
                            if response.poses:
                                p = response.poses[0].position
                                q = response.poses[0].orientation
                                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                 1 - 2 * (q.y * q.y + q.z * q.z))
                                samples.append({'source': resolution, 'scan': scan_index,
                                                'pose': (p.x, p.y, p.z), 'yaw': yaw})
                        except Exception as error:
                            errors.append('%s:%d:%r' % (name, scan_index, error))
            accepted = None
            accepted_yaw = None
            if args.candidate_mode == 'fine':
                eligible = samples
                if prior is not None:
                    eligible = [sample for sample in samples
                                if math.hypot(sample['pose'][0] - prior[0],
                                              sample['pose'][1] - prior[1])
                                <= args.prior_radius]
                if eligible:
                    pick = (min(eligible, key=lambda sample: math.hypot(
                        sample['pose'][0] - prior[0], sample['pose'][1] - prior[1]))
                        if prior is not None else eligible[0])
                    distance = (math.hypot(pick['pose'][0] - prior[0],
                                           pick['pose'][1] - prior[1])
                                if prior is not None else None)
                    accepted = {'pose': list(pick['pose'][:2]), 'source': pick['source'],
                                'support': len(eligible), 'scan': pick['scan'],
                                'prior_distance_m': distance, 'members': [dict(pick)]}
                    accepted_yaw = pick['yaw']
            else:
                accepted = CandidateConsensus(radius=.6, min_support=2).select(samples)
                if accepted is not None:
                    accepted_yaw = next((sample['yaw'] for sample in samples
                                         if sample['source'] == accepted['source']), 0.0)
            with lock:
                shared['last_query'] = {'samples': samples, 'accepted': accepted,
                                        'accepted_yaw': accepted_yaw,
                                        'prior_xy': prior,
                                        'prior_radius_m': args.prior_radius,
                                        'scan_queries': len(scans) if not reject else 0,
                                        'reject': reject, 'errors': errors}
            print('ADAPTER_QUERY %d %s' % (call_number, json.dumps(
                shared['last_query'], ensure_ascii=False, default=str)), flush=True)
            if accepted is not None and not reject:
                score = (1.0 if args.candidate_mode == 'fine'
                         else accepted['support'] / float(len(RESOLUTIONS)))
                candidate_pub.publish(String(data=json.dumps({
                    'pose': [accepted['pose'][0], accepted['pose'][1], 0.0],
                    'yaw': accepted_yaw,
                    'score': score})))
            return EmptyResponse()

        rospy.Service('/relocalize', Empty, relocalize_cb)

        def on_backend(message):
            try:
                payload = json.loads(message.data)
                pose = payload['pose']
                lease = int(payload['lease'])
                yaw = payload.get('yaw')
                yaw = 0.0 if yaw is None else float(yaw)
            except (ValueError, KeyError, TypeError, IndexError):
                return
            with lock:
                if lease <= shared['last_reset_lease'] or len(pose) < 2:
                    return
                shared['last_reset_lease'] = lease
                shared['resets'].append({'lease': lease, 'pose': list(pose), 'yaw': yaw})
            reset = PoseWithCovarianceStamped()
            reset.header.stamp, reset.header.frame_id = rospy.Time.now(), 'map'
            reset.pose.pose.position.x, reset.pose.pose.position.y = float(pose[0]), float(pose[1])
            reset.pose.pose.position.z = 0.0
            reset.pose.pose.orientation.z = math.sin(yaw / 2.0)
            reset.pose.pose.orientation.w = math.cos(yaw / 2.0)
            initialpose_pub.publish(reset)

        callbacks.append(rospy.Subscriber('/a6/recovery/backend', String, on_backend,
                                          queue_size=10))
        threading.Thread(target=feed_loop, daemon=True).start()
        wait_until(lambda: shared['odom'] is not None, 90, 'hdl odom start')
        a6 = subprocess.Popen([sys.executable, '-B', str(HERE / 'a6_recovery_node.py'),
                               '--namespace', '/a6/recovery',
                               '--odom-topic', '/hdl_odom',
                               '--candidate-topic', '/a6/recovery/candidate',
                               '--initial-lease', '1',
                               '--relocalize-service', '/relocalize',
                               '--deadline-s', str(args.deadline_s),
                               '--timeout-s', '0.5',
                               '--score-min', '0.5',
                               '--bounds=-3,8,-5,5'],
                              start_new_session=True,
                              stdout=open('/tmp/a6_native_node.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
        wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                   and status['lease'] == 1, 30, 'initial TRACKING lease 1')
        time.sleep(1.0)
        with lock:
            initial_calls = shared['calls']
            initial_truth = list(shared['truth'])
            initial_odom = list(shared['odom'])
        initial_tracking = True
        healthy_calls = initial_calls
        cycles = []
        for index in range(args.cycles):
            expected = index + 2
            with lock:
                calls_before = shared['calls']
                shared['paused'] = True
            wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'RELOCALIZING')
            wait_until(lambda: shared['calls'] > calls_before, 6, 'relocalize call')
            wait_until(lambda: any(item['lease'] == expected for item in shared['resets']),
                       args.deadline_s + 2, 'backend reset lease %d' % expected)
            reset = next(item for item in shared['resets'] if item['lease'] == expected)
            with lock:
                shared['paused'] = False
            wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                       and status['lease'] == expected, 15,
                       'TRACKING lease %d' % expected)
            time.sleep(args.settle_s)
            with lock:
                odom = list(shared['odom'])
                truth = list(shared['truth'])
                query = shared['last_query']
            error = math.hypot(odom[0] - truth[0], odom[1] - truth[1])
            yaw_error = math.atan2(math.sin(odom[3] - truth[3]),
                                   math.cos(odom[3] - truth[3]))
            cycles.append({'lease': expected, 'truth_xy': truth[:2], 'odom_xy': odom[:2],
                           'error_m': error, 'truth_yaw': truth[3], 'odom_yaw': odom[3],
                           'yaw_error_rad': yaw_error,
                           'reset': reset, 'query': query,
                           'phase': status['phase'], 'allowed': status['allowed']})
        with lock:
            reject_calls_before = shared['calls']
            shared['reject'] = True
            shared['paused'] = True
            resets_before_reject = len(shared['resets'])
        wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'reject RELOCALIZING')
        wait_until(lambda: shared['calls'] > reject_calls_before, 6, 'reject relocalize call')
        wait_until(lambda: status['phase'] == 'FAILED', args.deadline_s + 3, 'FAILED')
        time.sleep(.5)
        with lock:
            reject_query = shared['last_query']
            resets_after = len(shared['resets'])
        negative = {'phase': status['phase'], 'allowed': status['allowed'],
                    'query': reject_query, 'new_reset': resets_after > resets_before_reject}
        result = {'ok': bool(initial_tracking and healthy_calls == 0
                             and all(item['phase'] == 'TRACKING' and item['allowed']
                                     and item['error_m'] <= args.gate_m for item in cycles)
                             and negative['phase'] == 'FAILED'
                             and not negative['allowed'] and not negative['new_reset']
                             and negative['query'] is not None
                             and negative['query']['accepted'] is None),
                  'spawn_xy': [spawn_x, spawn_y],
                  'initial_truth_xy': initial_truth[:2], 'initial_odom_xy': initial_odom[:2],
                  'initial_phase': 'TRACKING', 'initial_lease': 1,
                  'healthy_relocalize_calls': healthy_calls,
                  'cycles': cycles, 'negative': negative,
                  'gate_m': args.gate_m, 'deadline_s': args.deadline_s,
                  'scan_downsample_m': args.scan_downsample,
                  'candidate_mode': args.candidate_mode,
                  'scan_queries': args.scan_queries,
                  'prior_radius_m': args.prior_radius,
                  'input_semantics': 'sensor-local scan relabelled base_link, voxel .3; '
                                     'Gazebo truth scoring only',
                  'candidate_source': (('finest BBS .1 over %d recent scans, nearest to last '
                                        'odom within %.1f m') % (args.scan_queries,
                                                                 args.prior_radius)
                                       if args.candidate_mode == 'fine' else
                                       'three real BBS instances (.5/.2/.1) + CandidateConsensus'),
                  'reset_path': 'A6 backend message -> native hdl /initialpose',
                  'native_relocalize_service': 'not used (single-candidate service covered by R1078)'}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        stop_feed.set()
        for subscription in callbacks:
            subscription.unregister()
        for process in bbs_nodes + [a6, hdl, gazebo, master]:
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
