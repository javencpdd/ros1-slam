#!/usr/bin/env python3
"""Live native hdl /relocalize smoke on the m1 Gazebo world.

The m1 lidar publishes sensor-local XYZ under frame_id=rslidar; the URDF mount
has zero x/y offset (pitch only), and BBS uses x/y only, so the harness relabels
the scan base_link and feeds it unchanged.  Gazebo truth is used only to score
the result, never as localization input.  The test proves the native hdl chain
(globalmap -> BBS query -> pose reset -> /hdl_odom) and the effect of the native
/relocalize service.
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
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
LAUNCH = Path('/opt/ros1-slam/ws/src/20_localization/lite3_localization/launch/hdl_m1_global_smoke.launch')
PORT, GAZEBO_PORT = 11650, 11651
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/hdl_native_ros',
                  ROS_LOG_DIR='/tmp/hdl_native_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/hdl_native_ros').mkdir(exist_ok=True)
Path('/tmp/hdl_native_logs').mkdir(exist_ok=True)


def rotation_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_error(estimate, truth):
    def yaw(quaternion):
        matrix = rotation_matrix(quaternion)
        return math.atan2(matrix[1, 0], matrix[0, 0])
    delta = yaw(estimate) - yaw(truth)
    return math.atan2(math.sin(delta), math.cos(delta))


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def topic_graph():
    try:
        import rosgraph
        state = rosgraph.Master('/hdl_native_diag').getSystemState()
        wanted = ('/velodyne_points', '/leg_odom2', '/hdl_odom', '/rslidar_points',
                  '/globalmap', '/relocalize')
        return {topic: {'publishers': [name for name, names in state[0] if topic in names],
                        'subscribers': [name for name, names in state[1] if topic in names]}
                for topic in wanted}
    except Exception as error:
        return {'error': repr(error)}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--poses', default='1.5,0;0,1.0')
    parser.add_argument('--prior', default='0,0')
    parser.add_argument('--mode', choices=('lost', 'tracked'), default='lost')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--gate-m', type=float, default=1.0)
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    import rospy
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from hdl_global_localization.srv import QueryGlobalLocalization
    from hdl_localization.msg import ScanMatchingStatus
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from std_srvs.srv import Empty
    master = gazebo = hdl = None
    stop_feed = threading.Event()
    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/hdl_native_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], check=True,
                       env=dict(os.environ))
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/hdl_native_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('hdl_native_relocalize_smoke', anonymous=True, disable_signals=True)
        rospy.wait_for_message('/clock', Clock, timeout=90)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=90)
        rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        shared = {'cloud': None, 'truth': None, 'odom': None, 'sent': 0,
                  'local': None, 'status': None, 'paused': False}
        lock = threading.Lock()

        def on_cloud(message):
            with lock:
                shared['cloud'] = message

        def on_truth(message):
            names = list(message.name)
            if 'lite3_planar' not in names:
                return
            pose = message.pose[names.index('lite3_planar')]
            p, q = pose.position, pose.orientation
            with lock:
                shared['truth'] = {'position': np.array([p.x, p.y, p.z]),
                                   'quaternion': [q.x, q.y, q.z, q.w]}

        def on_odom(message):
            with lock:
                shared['odom'] = message

        def on_status(message):
            shared['status'] = {'has_converged': bool(message.has_converged),
                                'inlier_fraction': float(message.inlier_fraction),
                                'matching_error': float(message.matching_error)}

        rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=2)
        rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2)
        rospy.Subscriber('/hdl_odom', Odometry, on_odom, queue_size=50)
        rospy.Subscriber('/status', ScanMatchingStatus, on_status, queue_size=50)
        cloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=2)
        leg_pub = rospy.Publisher('/leg_odom2', Odometry, queue_size=2, latch=True)
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
                    cloud, paused = shared['cloud'], shared['paused']
                if not paused and cloud is not None and cloud_pub.get_num_connections():
                    points = np.asarray(list(pc2.read_points(
                        cloud, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
                    if len(points):
                        # Sensor-local x/y equal base_link x/y; the 15 deg mount pitch is z-only.
                        header = rospy.Header(stamp=cloud.header.stamp, frame_id='base_link')
                        message = pc2.create_cloud(header, fields, points)
                        cloud_pub.publish(message)
                        with lock:
                            shared['sent'] += 1
                            shared['local'] = message
                time.sleep(.2)

        threading.Thread(target=feed_loop, daemon=True).start()
        hdl = subprocess.Popen(['roslaunch', str(LAUNCH)], start_new_session=True,
                               stdout=open('/tmp/hdl_native_launch.log', 'ab'),
                               stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.wait_for_service('/relocalize', timeout=120)
        deadline = time.monotonic() + 90
        diag_at = time.monotonic() + 30
        diagnosed = None
        while time.monotonic() < deadline and shared['odom'] is None:
            if diagnosed is None and time.monotonic() > diag_at:
                diagnosed = topic_graph()
            time.sleep(.1)
        if shared['odom'] is None:
            print(json.dumps({'ok': False, 'error': 'hdl_odom did not start',
                              'topics': diagnosed, 'clouds_sent': shared['sent']},
                             ensure_ascii=False, indent=1), flush=True)
            return 6
        relocalize = rospy.ServiceProxy('/relocalize', Empty)
        query = rospy.ServiceProxy('/hdl_global_localization/query', QueryGlobalLocalization)
        initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                          queue_size=2, latch=True)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        prior_x, prior_y = (float(v) for v in args.prior.split(','))
        rows = []
        for _ in range(args.repeat):
            for pair in args.poses.split(';'):
                x, y = (float(v) for v in pair.split(','))
                state = ModelState(model_name='lite3_planar', reference_frame='world')
                state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, .112
                state.pose.orientation.w = 1.
                set_state(state)
                time.sleep(2.0)
                with lock:
                    truth = None if shared['truth'] is None else dict(
                        position=shared['truth']['position'].copy(),
                        quaternion=list(shared['truth']['quaternion']))
                    last_odom = shared['odom']
                # lost: stop scans, reset to a known wrong prior, then native /relocalize.
                # tracked: call /relocalize while the filter is already tracking.
                if args.mode == 'lost':
                    with lock:
                        shared['paused'] = True
                    time.sleep(.4)
                    prior = PoseWithCovarianceStamped()
                    prior.header.stamp, prior.header.frame_id = rospy.Time.now(), 'map'
                    prior.pose.pose.position.x, prior.pose.pose.position.y = prior_x, prior_y
                    prior.pose.pose.orientation.w = 1.
                    for _ in range(5):
                        initialpose_pub.publish(prior)
                        time.sleep(.05)
                    time.sleep(.3)
                service_ok = True
                try:
                    relocalize()
                except Exception as error:
                    service_ok = False
                    service_error = repr(error)
                if args.mode == 'lost':
                    with lock:
                        shared['paused'] = False
                    time.sleep(2.5)
                else:
                    time.sleep(3.0)
                with lock:
                    after = shared['odom']
                    local = shared['local']
                    status = shared['status']
                truth_xy = [float(truth['position'][0]), float(truth['position'][1])]
                last_xy = [last_odom.pose.pose.position.x, last_odom.pose.pose.position.y]
                after_xy = [after.pose.pose.position.x, after.pose.pose.position.y]
                after_q = after.pose.pose.orientation
                last_error = math.hypot(last_xy[0] - truth_xy[0], last_xy[1] - truth_xy[1])
                after_error = math.hypot(after_xy[0] - truth_xy[0], after_xy[1] - truth_xy[1])
                if args.mode == 'lost':
                    baseline_xy = [prior_x, prior_y]
                    baseline_error = math.hypot(prior_x - truth_xy[0], prior_y - truth_xy[1])
                else:
                    baseline_xy, baseline_error = last_xy, last_error
                bbs_poses, bbs_errors, bbs_error = [], [], None
                if local is not None:
                    try:
                        response = query(8, local)
                        bbs_poses = [[p.position.x, p.position.y, p.position.z]
                                     for p in response.poses]
                        bbs_errors = [math.hypot(p[0] - truth_xy[0], p[1] - truth_xy[1])
                                      for p in bbs_poses]
                    except Exception as error:
                        bbs_error = repr(error)
                rows.append({'target_xy': [x, y], 'truth_xy': truth_xy,
                             'baseline_xy': baseline_xy, 'baseline_error_m': baseline_error,
                             'odom_last_xy': last_xy, 'error_last_odom_m': last_error,
                             'odom_after_xy': after_xy, 'error_after_m': after_error,
                             'worse_than_last_odom': after_error > last_error + .25,
                             'yaw_error_after_rad': yaw_error(
                                 [after_q.x, after_q.y, after_q.z, after_q.w],
                                 truth['quaternion']),
                             'service_returned': service_ok,
                             'scan_matching': status,
                             'bbs_candidates_xy': [pose[:2] for pose in bbs_poses],
                             'bbs_xy_errors_m': bbs_errors,
                             'bbs_best_error_m': min(bbs_errors) if bbs_errors else None,
                             'bbs_candidates_available': bool(bbs_poses),
                             **({} if bbs_error is None else {'bbs_error': bbs_error}),
                             **({} if service_ok else {'service_error': service_error})})
        result = {'ok': all(row['service_returned'] and row['error_after_m'] <= args.gate_m
                            for row in rows),
                  'gate_m': args.gate_m, 'mode': args.mode, 'poses': rows,
                  'input_semantics': 'sensor-local m1 lidar XYZ relabelled base_link; mount x/y '
                                     'offset is zero (pitch is z-only); Gazebo truth is scoring '
                                     'only',
                  **({'lost_state': 'scans paused, estimator reset to prior %s, native '
                                    '/relocalize re-seeds from the last real scan' % args.prior}
                     if args.mode == 'lost' else
                     {'tracked_state': 'filter already tracking; native /relocalize called '
                                       'without a loss trigger'}),
                  'native_service': '/relocalize advertised by hdl_localization_nodelet',
                  'clouds_sent': shared['sent']}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        stop_feed.set()
        for process in (hdl, gazebo, master):
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
