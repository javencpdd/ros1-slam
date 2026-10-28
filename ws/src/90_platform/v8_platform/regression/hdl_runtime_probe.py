#!/usr/bin/env python3
"""Measure the existing HDL nodelet with an isolated, truth-adapted sensor fixture.

No third-party source, frozen limit, manifest or continuous localization route is
changed.  Gazebo's /rslidar_points has WORLD-valued XYZ despite its sensor frame;
this adapter applies the inverse GT base pose at the cloud timestamp.  It does
not merely rename frame_id.  This dependency is recorded as GT-assisted input
semantics, not evidence of an independent sensor/localization implementation.

The stock nodelet dereferences /leg_odom2 before checking the pointer.  A declared
zero-twist message is delivered before enabling clouds.  It is not leg odometry
or a prediction source.  All HDL outputs, including TF, are privately remapped.
"""
import argparse
from collections import deque
import hashlib
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
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def rotation_matrix(quaternion):
    """Quaternion order XYZW, as ROS geometry messages."""
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if q.shape != (4,) or not np.all(np.isfinite(q)) or norm < 1e-9:
        raise ValueError('finite nonzero XYZW quaternion required')
    x, y, z, w = q / norm
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def world_to_base(points, position, quaternion):
    points = np.asarray(points, dtype=float)
    position = np.asarray(position, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or position.shape != (3,) \
            or not np.all(np.isfinite(points)) or not np.all(np.isfinite(position)):
        raise ValueError('finite Nx3 points and XYZ translation required')
    return (points - position) @ rotation_matrix(quaternion)


def interpolate_truth(samples, stamp, max_gap=.05):
    """Bracketed pose interpolation, shortest quaternion arc; bounded nearest fallback."""
    if not samples:
        return None
    before = next((sample for sample in reversed(samples) if sample['stamp'] <= stamp), None)
    after = next((sample for sample in samples if sample['stamp'] >= stamp), None)
    if before and after and max(stamp - before['stamp'], after['stamp'] - stamp) <= max_gap:
        duration = after['stamp'] - before['stamp']
        ratio = (stamp - before['stamp']) / duration if duration > 1e-12 else 0.
        first = np.asarray(before['quaternion'], dtype=float)
        second = np.asarray(after['quaternion'], dtype=float)
        first, second = first / np.linalg.norm(first), second / np.linalg.norm(second)
        dot = float(first @ second)
        if dot < 0:
            second, dot = -second, -dot
        if dot > .9995:
            quaternion = first + ratio * (second - first)
        else:
            angle = math.acos(max(-1., min(1., dot)))
            quaternion = (math.sin((1 - ratio) * angle) * first
                          + math.sin(ratio * angle) * second) / math.sin(angle)
        quaternion /= np.linalg.norm(quaternion)
        position = np.asarray(before['position']) * (1 - ratio) + np.asarray(after['position']) * ratio
        return {'stamp': stamp, 'position': position.tolist(), 'quaternion': quaternion.tolist(),
                'truth_bracket': [before['stamp'], after['stamp']], 'nearest_offset_s': 0.}
    nearest = min(samples, key=lambda sample: abs(sample['stamp'] - stamp))
    if abs(nearest['stamp'] - stamp) <= max_gap:
        return dict(nearest, truth_bracket=[nearest['stamp'], nearest['stamp']],
                    nearest_offset_s=nearest['stamp'] - stamp)
    return None


def yaw_error(estimate, truth):
    def yaw(quaternion):
        matrix = rotation_matrix(quaternion)
        return math.atan2(matrix[1, 0], matrix[0, 0])
    delta = yaw(estimate) - yaw(truth)
    return math.atan2(math.sin(delta), math.cos(delta))


def make_launch(map_path, prefix='/hdlcandidate', initial=(0., 0., .11, 0.), ndt_resolution=1.):
    """Temporary wrapper around existing nodelets; immutable upstream launch is unused."""
    if not prefix.startswith('/') or prefix == '/' or '..' in prefix:
        raise ValueError('private absolute namespace required')
    prefix = prefix.rstrip('/')
    root = ET.Element('launch')
    manager = ET.SubElement(root, 'node', pkg='nodelet', type='nodelet',
                            name='manager', ns=prefix, args='manager', output='screen')
    for source, target in (('/tf', prefix + '/tf'), ('/tf_static', prefix + '/tf_static')):
        ET.SubElement(manager, 'remap', {'from': source, 'to': target})
    ET.SubElement(manager, 'param', name='num_worker_threads', value='2')
    map_node = ET.SubElement(root, 'node', pkg='nodelet', type='nodelet', name='map_server',
        ns=prefix, args='load hdl_localization/GlobalmapServerNodelet ' + prefix + '/manager', output='screen')
    localizer = ET.SubElement(root, 'node', pkg='nodelet', type='nodelet', name='localizer',
        ns=prefix, args='load hdl_localization/HdlLocalizationNodelet ' + prefix + '/manager', output='screen')
    remappings = {'/globalmap': prefix + '/globalmap', '/map_request/pcd': prefix + '/map_request/pcd',
                  '/velodyne_points': prefix + '/cloud', '/initialpose': prefix + '/initialpose',
                  '/leg_odom2': prefix + '/leg_odom2', '/odom': prefix + '/odom',
                  '/status': prefix + '/status', '/aligned_points': prefix + '/aligned_points',
                  '/relocalize': prefix + '/relocalize', '/tf': prefix + '/tf',
                  '/tf_static': prefix + '/tf_static'}
    for node in (map_node, localizer):
        for source, target in remappings.items():
            ET.SubElement(node, 'remap', {'from': source, 'to': target})
    for name, value in {'globalmap_pcd': str(map_path), 'convert_utm_to_local': False,
                         'downsample_resolution': .1}.items():
        ET.SubElement(map_node, 'param', name=name, value=str(value).lower() if isinstance(value, bool) else str(value))
    x, y, z, yaw = initial
    parameters = {'odom_child_frame_id': 'base_link', 'robot_odom_frame_id': prefix[1:] + '_robot_odom',
                  'use_imu': False, 'use_global_localization': False,
                  'enable_robot_odometry_prediction': False, 'specify_init_pose': True,
                  'init_pos_x': x, 'init_pos_y': y, 'init_pos_z': z,
                  'init_ori_x': 0., 'init_ori_y': 0., 'init_ori_z': math.sin(yaw / 2),
                  'init_ori_w': math.cos(yaw / 2), 'reg_method': 'NDT_OMP',
                  'ndt_neighbor_search_method': 'DIRECT7', 'ndt_resolution': ndt_resolution,
                  'downsample_resolution': .1, 'cool_time_duration': .5,
                  'lidar_timeout_ms': 200}
    for name, value in parameters.items():
        ET.SubElement(localizer, 'param', name=name, value=str(value).lower() if isinstance(value, bool) else str(value))
    return ET.tostring(root, encoding='unicode')


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def measure(args):
    import rospy
    from geometry_msgs.msg import Twist
    from hdl_localization.msg import ScanMatchingStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Bool, Header, String, UInt64

    rospy.init_node('hdl_runtime_probe', anonymous=True, disable_signals=True)
    prefix = args.namespace.rstrip('/')
    start_wall = time.monotonic()
    now = lambda: time.monotonic() - start_wall
    lock = threading.RLock()
    shared = {'cloud': None, 'map_ready': False, 'truth': deque(maxlen=4000),
              'sent_truth': {}, 'sent': [], 'pairs': [], 'statuses': [], 'drops': {},
              'guard_outputs': [], 'guard_states': []}
    output_cloud = rospy.Publisher(prefix + '/cloud', PointCloud2, queue_size=1)
    leg_twist = rospy.Publisher(prefix + '/leg_odom2', Odometry, queue_size=1, latch=True)
    health = rospy.Publisher(prefix + '/healthy', Bool, queue_size=20, latch=True)
    candidate = rospy.Publisher(prefix + '/guard/candidate', Twist, queue_size=20)
    leases = rospy.Publisher(prefix + '/guard/lease', UInt64, queue_size=20)
    health.publish(Bool(data=False))

    def ground_truth(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        sample = {'stamp': message.header.stamp.to_sec(), 'position': [p.x, p.y, p.z],
                  'quaternion': [q.x, q.y, q.z, q.w]}
        with lock:
            if not shared['truth'] or sample['stamp'] > shared['truth'][-1]['stamp']:
                shared['truth'].append(sample)

    def cloud(message):
        with lock:
            shared['cloud'] = message  # One pending frame; no unbounded queue of old scans.

    def map_ready(_message):
        shared['map_ready'] = True

    def odometry(message):
        stamp = message.header.stamp.to_sec()
        p, q = message.pose.pose.position, message.pose.pose.orientation
        with lock:
            truth = shared['sent_truth'].get(stamp) or interpolate_truth(list(shared['truth']), stamp)
            row = {'wall': now(), 'stamp': stamp, 'frame': message.header.frame_id,
                   'child': message.child_frame_id, 'position': [p.x, p.y, p.z],
                   'quaternion': [q.x, q.y, q.z, q.w], 'truth': truth}
            if truth is not None:
                row.update(xy_error_m=math.hypot(p.x - truth['position'][0], p.y - truth['position'][1]),
                           z_error_m=p.z - truth['position'][2],
                           yaw_error_rad=yaw_error(row['quaternion'], truth['quaternion']))
            shared['pairs'].append(row)

    def status(message):
        row = {'wall': now(), 'stamp': message.header.stamp.to_sec(),
               'has_converged': message.has_converged,
               'matching_error': finite_or_none(message.matching_error),
               'inlier_fraction': finite_or_none(message.inlier_fraction)}
        with lock:
            shared['statuses'].append(row)
        # Explicitly only a scan-consistency signal, not an accuracy certificate.
        health.publish(Bool(data=bool(row['has_converged'] and row['matching_error'] is not None
                                      and row['inlier_fraction'] is not None and row['inlier_fraction'] > 0)))

    def guard_output(message):
        shared['guard_outputs'].append({'wall': now(), 'vx': message.linear.x,
                                         'vy': message.linear.y, 'wz': message.angular.z})

    def guard_status(message):
        shared['guard_states'].append(dict(wall=now(), **json.loads(message.data)))

    subscriptions = [rospy.Subscriber(args.truth_topic, Odometry, ground_truth, queue_size=200),
                     rospy.Subscriber(args.points_topic, PointCloud2, cloud, queue_size=1),
                     rospy.Subscriber(prefix + '/globalmap', PointCloud2, map_ready, queue_size=1),
                     rospy.Subscriber(prefix + '/odom', Odometry, odometry, queue_size=100),
                     rospy.Subscriber(prefix + '/status', ScanMatchingStatus, status, queue_size=100)]
    if args.with_lease_gate:
        subscriptions.extend([rospy.Subscriber(prefix + '/guard/cmd_vel', Twist, guard_output, queue_size=100),
                              rospy.Subscriber(prefix + '/guard/status', String, guard_status, queue_size=100)])
    fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
              for index, name in enumerate(('x', 'y', 'z', 'intensity'))]
    last_sent = [-1e100]

    def tick(enabled=True):
        leg_message = Odometry()
        leg_message.header.stamp = rospy.Time.now()
        leg_message.header.frame_id = 'map'
        leg_message.child_frame_id = 'base_link'
        leg_message.pose.pose.orientation.w = 1.
        leg_twist.publish(leg_message)
        if args.with_lease_gate:
            command = Twist()
            command.linear.x = .10  # Private test candidate, never connected to the robot mux.
            candidate.publish(command)
        with lock:
            message = shared['cloud']
            shared['cloud'] = None
            samples = list(shared['truth'])
        if not enabled or message is None or now() - last_sent[0] < args.cloud_period:
            return
        stamp = message.header.stamp.to_sec()
        truth = interpolate_truth(samples, stamp)
        def drop(reason):
            shared['drops'][reason] = shared['drops'].get(reason, 0) + 1
        if truth is None:
            drop('no_timestamp_matched_truth')
            return
        age = rospy.Time.now().to_sec() - stamp
        if age > .18 or age < -.05:
            drop('stale_or_future_cloud_stamp')
            return
        names = [field.name for field in message.fields]
        columns = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
        points = np.asarray(list(point_cloud2.read_points(message, field_names=columns, skip_nans=True)), dtype=float)
        if points.size == 0:
            drop('empty_cloud')
            return
        converted = world_to_base(points[:, :3], truth['position'], truth['quaternion'])
        intensity = points[:, 3] if points.shape[1] == 4 else np.zeros(len(points))
        rows = np.column_stack((converted, intensity))
        header = Header(stamp=message.header.stamp, frame_id='base_link')
        forwarded = point_cloud2.create_cloud(header, fields, rows)
        sent_at = now()
        with lock:
            shared['sent_truth'][stamp] = truth
            shared['sent'].append({'wall': sent_at, 'stamp': stamp, 'source_frame': message.header.frame_id,
                'target_frame': 'base_link', 'point_count': len(rows), 'truth': truth,
                'source_data_sha256': hashlib.sha256(bytes(message.data)).hexdigest(),
                'forwarded_data_sha256': hashlib.sha256(bytes(forwarded.data)).hexdigest(),
                'source_xyz_sample': points[:8, :3].tolist(), 'forwarded_xyz_sample': converted[:8].tolist()})
        output_cloud.publish(forwarded)
        last_sent[0] = sent_at

    def wait_for(predicate, timeout, label, feed=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            tick(feed)
            time.sleep(.02)
        raise RuntimeError('HDL readiness timeout: ' + label)

    def pump(seconds, enabled=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            tick(enabled)
            time.sleep(.02)

    launch_xml = make_launch(args.map_pcd, prefix, (args.init_x, args.init_y, args.init_z, args.init_yaw),
                             args.ndt_resolution)
    children = []
    with tempfile.TemporaryDirectory(prefix='hdl_probe_') as directory:
        launch_path = Path(directory) / 'candidate.launch'
        launch_path.write_text(launch_xml)
        with open(Path(directory) / 'launch.log', 'w+b') as launch_log, \
                open(Path(directory) / 'guard.log', 'w+b') as guard_log:
            failure, stop_window, recovery_window = None, None, None
            try:
                launch = subprocess.Popen(['roslaunch', str(launch_path)], stdout=launch_log, stderr=subprocess.STDOUT)
                children.append(launch)
                if args.with_lease_gate:
                    command = [sys.executable, '-B', str(ROOT / 'adapters/localization_state.py'),
                        '--namespace', prefix + '/guard', '--odom-topic', prefix + '/odom',
                        '--health-topic', prefix + '/healthy', '--timeout-s', '1.0',
                        '--keep-alive-s', str(args.measure_seconds + 80)]
                    children.append(subprocess.Popen(command, stdout=guard_log, stderr=subprocess.STDOUT))
                wait_for(lambda: bool(shared['truth']) and shared['cloud'] is not None,
                         25, 'timestamped GT odometry and lidar')
                wait_for(lambda: output_cloud.get_num_connections() > 0
                         and leg_twist.get_num_connections() > 0 and shared['map_ready'],
                         25, 'nodelet inputs, seeded twist, and matching map')
                # Keep scans closed until the undeclared twist pointer has received a value.
                pump(.5, enabled=False)
                wait_for(lambda: len(shared['pairs']) >= 3, 30, 'three real HDL odometry outputs', feed=True)
                if args.with_lease_gate:
                    wait_for(lambda: shared['guard_states'] and shared['guard_states'][-1]['state'] == 'HEALTHY',
                             12, 'HDL-derived lease health', feed=True)
                    leases.publish(UInt64(data=1))
                pump(args.measure_seconds)
                stop_window = [now()]
                pump(args.pause_seconds, enabled=False)
                stop_window.append(now())
                recovery_window = [now()]
                previous_pairs = len(shared['pairs'])
                wait_for(lambda: len(shared['pairs']) >= previous_pairs + 3, 20, 'HDL scan recovery', feed=True)
                recovery_window.append(now())
                if args.with_lease_gate:
                    # New real odometry alone may not revive a revoked trajectory.
                    pump(.3)
                    leases.publish(UInt64(data=2))
                    pump(.4)
            except Exception as error:
                failure = str(error)
            finally:
                # Stop exactly our manager/loaders, then let roslaunch reap its children.
                try:
                    import rosnode
                    rosnode.kill_nodes([prefix + '/localizer', prefix + '/map_server', prefix + '/manager'])
                except Exception:
                    pass
                for process in reversed(children):
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=3)
                for subscription in subscriptions:
                    subscription.unregister()
            launch_log.seek(0)
            guard_log.seek(0)
            logs = {'launch_tail': launch_log.read()[-12000:].decode(errors='replace'),
                    'guard_tail': guard_log.read()[-4000:].decode(errors='replace')}
    paired = [row for row in shared['pairs'] if 'xy_error_m' in row]
    values = np.array([row['xy_error_m'] for row in paired])
    errors = {'samples': len(values), 'max_xy_m': float(np.max(values)) if len(values) else None,
              'p95_xy_m': float(np.percentile(values, 95)) if len(values) else None,
              'max_abs_yaw_rad': max((abs(row['yaw_error_rad']) for row in paired), default=None),
              'z_forced_zero_by_upstream': True}
    from_calibration = ROOT / 'calibration/calibration.py'
    sys.path.insert(0, str(from_calibration.parent))
    import calibration
    comparator = calibration.FROZEN['tolerances']['max_localization_error_m']['value']
    result = {'measurement_complete': failure is None and len(paired) >= 3,
        'failure': failure, 'role': 'isolated monitor candidate', 'backend_accepted': False,
        'input_semantics': 'world-valued lidar numerically transformed by timestamp-matched GT base pose',
        'ground_truth_assisted_fixture': True, 'points_topic': args.points_topic,
        'truth_topic': args.truth_topic, 'map_pcd': str(args.map_pcd),
        'map_sha256': hashlib.sha256(Path(args.map_pcd).read_bytes()).hexdigest(),
        'launch_xml': launch_xml, 'errors': errors,
        'comparison_only': {'lio_scoped_limit_m': comparator,
            'within_lio_scoped_limit': bool(len(values) and np.max(values) <= comparator),
            'same_acceptance_scope': False}, 'stop_window_wall': stop_window,
        'recovery_window_wall': recovery_window, 'raw': {key: list(value) if isinstance(value, deque) else value
            for key, value in shared.items() if key not in ('cloud', 'sent_truth')}, 'logs': logs}
    return result


def self_test():
    import unittest
    class WiringTests(unittest.TestCase):
        def test_world_to_base_is_numeric_inverse_transform(self):
            q = [0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)]
            np.testing.assert_allclose(world_to_base([[1., 3., .11]], [1., 2., .11], q),
                                       [[1., 0., 0.]], atol=1e-8)

        def test_pose_time_join_interpolates_short_arc_and_refuses_old_truth(self):
            def sample(stamp, yaw):
                return {'stamp': stamp, 'position': [stamp, 0., .11],
                        'quaternion': [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]}
            values = [sample(1., math.radians(179)), sample(1.04, math.radians(-179))]
            result = interpolate_truth(values, 1.02)
            self.assertAlmostEqual(result['position'][0], 1.02)
            np.testing.assert_allclose(rotation_matrix(result['quaternion']) @ [1, 0, 0], [-1, 0, 0], atol=1e-8)
            self.assertIsNone(interpolate_truth(values, 1.20))

        def test_launch_isolates_all_absolute_topics_and_disables_global_initialization(self):
            tree = ET.fromstring(make_launch('/tmp/matching_map.pcd'))
            localizer = next(node for node in tree.findall('node') if node.get('name') == 'localizer')
            remap = {item.get('from'): item.get('to') for item in localizer.findall('remap')}
            self.assertEqual(remap['/odom'], '/hdlcandidate/odom')
            self.assertEqual(remap['/leg_odom2'], '/hdlcandidate/leg_odom2')
            self.assertEqual(remap['/tf'], '/hdlcandidate/tf')
            params = {item.get('name'): item.get('value') for item in localizer.findall('param')}
            self.assertEqual(params['use_global_localization'], 'false')
            self.assertEqual(params['use_imu'], 'false')
            self.assertEqual(params['init_pos_z'], '0.11')

        def test_invalid_transform_and_nan_scores_are_not_green(self):
            with self.assertRaises(ValueError):
                world_to_base([[0., 0., 0.]], [0., 0., 0.], [0., 0., 0., 0.])
            self.assertIsNone(finite_or_none(float('nan')))
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(WiringTests))
    return 0 if result.wasSuccessful() else 6


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--namespace', default='/hdlcandidate')
    parser.add_argument('--points-topic', default='/rslidar_points')
    parser.add_argument('--truth-topic', default='/sim/odom_gt')
    parser.add_argument('--map-pcd', default='/repo/data/maps/m1_world_synth_v1.pcd')
    parser.add_argument('--init-x', type=float, default=0.)
    parser.add_argument('--init-y', type=float, default=0.)
    parser.add_argument('--init-z', type=float, default=.11)
    parser.add_argument('--init-yaw', type=float, default=0.)
    parser.add_argument('--ndt-resolution', type=float, default=1.)
    parser.add_argument('--cloud-period', type=float, default=.20)
    parser.add_argument('--measure-seconds', type=float, default=12.)
    parser.add_argument('--pause-seconds', type=float, default=2.)
    parser.add_argument('--with-lease-gate', action='store_true')
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    result = measure(args)
    if args.out:
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key not in ('raw', 'logs', 'launch_xml')},
                     ensure_ascii=False, allow_nan=False))
    return 0 if result['measurement_complete'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
