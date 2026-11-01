#!/usr/bin/env python3
"""Live A6 recovery: Gazebo scan -> three BBS consensus -> /relocalize -> A6 node."""
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
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'regression'))
from candidate_consensus import CandidateConsensus  # noqa: E402
from hdl_global_consensus_smoke import read_pcd, RESOLUTIONS  # noqa: E402

SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
PORT, GAZEBO_PORT = 11640, 11641
PCD = '/opt/ros1-slam/data/maps/m1_world_synth_v1.pcd'
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/a6_live_ros',
                  ROS_LOG_DIR='/tmp/a6_live_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/a6_live_ros').mkdir(exist_ok=True)
Path('/tmp/a6_live_logs').mkdir(exist_ok=True)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    import rospy
    import yaml
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String, UInt64
    from std_srvs.srv import Empty, EmptyResponse
    from hdl_global_localization.srv import SetGlobalMap, QueryGlobalLocalization

    master = gazebo = None
    bbs_nodes = []
    a6 = None
    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/a6_live_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_live_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('a6_recovery_live_smoke', anonymous=True, disable_signals=True)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        truth_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)
        names = list(truth_msg.name)
        truth = truth_msg.pose[names.index('lite3_planar')].position
        points = read_pcd(PCD)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        header = rospy.Header()
        header.frame_id, header.stamp = 'map', rospy.Time.now()
        global_cloud = pc2.create_cloud_xyz32(header, xyz[::5])
        base = HERE.parents[2] / '20_localization' / 'hdl_global_localization' / 'config'
        for resolution, name in RESOLUTIONS:
            for config_name in ('general_config.yaml', 'bbs_config.yaml'):
                values = yaml.safe_load((base / config_name).read_text())
                for key, value in values.items():
                    rospy.set_param('/%s/%s' % (name, key), value)
            rospy.set_param('/%s/bbs/map_min_z' % name, .05)
            rospy.set_param('/%s/bbs/map_max_z' % name, 1.0)
            rospy.set_param('/%s/bbs/scan_min_z' % name, -.3)
            rospy.set_param('/%s/bbs/scan_max_z' % name, .3)
            rospy.set_param('/%s/bbs/map_resolution' % name, resolution)
            node = subprocess.Popen(['rosrun', 'hdl_global_localization',
                                     'hdl_global_localization_node', '__name:=' + name],
                                    start_new_session=True,
                                    stdout=open('/tmp/a6_live_%s.log' % name, 'ab'),
                                    stderr=subprocess.STDOUT, env=dict(os.environ))
            bbs_nodes.append(node)
            for service in ('/%s/set_global_map' % name, '/%s/query' % name):
                rospy.wait_for_service(service, timeout=30)
            rospy.ServiceProxy('/%s/set_global_map' % name, SetGlobalMap)(global_cloud)
        candidate_pub = rospy.Publisher('/a6/recovery/candidate', String, queue_size=5)
        state = {'cloud': cloud, 'healthy': True, 'x': truth.x, 'y': truth.y,
                 'relocalize_calls': 0, 'candidate': None, 'truth': [truth.x, truth.y]}
        lock = threading.Lock()

        def query_consensus(_request):
            with lock:
                state['relocalize_calls'] += 1
                cloud_now = state['cloud']
            scan = list(pc2.read_points(cloud_now, field_names=('x', 'y', 'z'), skip_nans=True))
            query_cloud = pc2.create_cloud_xyz32(cloud_now.header, scan)
            samples = []
            for resolution, name in RESOLUTIONS:
                response = rospy.ServiceProxy('/%s/query' % name, QueryGlobalLocalization)(
                    5, query_cloud)
                if response.poses:
                    p = response.poses[0].position
                    samples.append({'source': resolution, 'pose': (p.x, p.y, p.z)})
            accepted = CandidateConsensus(radius=.6, min_support=2).select(samples)
            with lock:
                state['candidate'] = {'samples': samples, 'accepted': accepted,
                                      'calls': state['relocalize_calls']}
            if accepted:
                candidate_pub.publish(String(data=json.dumps({
                    'pose': [accepted['pose'][0], accepted['pose'][1], 0.0],
                    'score': 1.0})))
            return EmptyResponse()

        rospy.Service('/relocalize', Empty, query_consensus)
        a6 = subprocess.Popen([sys.executable, '-B', str(HERE / 'a6_recovery_node.py'),
                               '--odom-topic', '/Odometry',
                               '--candidate-topic', '/a6/recovery/candidate',
                               '--lease-in-topic', '/a6/recovery/lease_in',
                               '--relocalize-service', '/relocalize',
                               '--deadline-s', '20'],
                              start_new_session=True,
                              stdout=open('/tmp/a6_live_node.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
        status = {'latest': None, 'allowed': False, 'lease': None, 'backend': None}
        rospy.Subscriber('/a6/recovery/status', String,
                         lambda m: status.update(latest=json.loads(m.data)), queue_size=20)
        rospy.Subscriber('/a6/recovery/command_allowed', Bool,
                         lambda m: status.update(allowed=m.data), queue_size=20)
        rospy.Subscriber('/a6/recovery/lease', UInt64,
                         lambda m: status.update(lease=m.data), queue_size=5)
        rospy.Subscriber('/a6/recovery/backend', String,
                         lambda m: status.update(backend=json.loads(m.data)), queue_size=5)
        odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=20)
        lease_pub = rospy.Publisher('/a6/recovery/lease_in', UInt64, queue_size=5, latch=True)
        stop_thread = threading.Event()

        def odom_loop():
            stamp = 100.0
            while not stop_thread.is_set() and not rospy.is_shutdown():
                with lock:
                    healthy = state['healthy']
                    x, y = state['x'], state['y']
                if healthy:
                    message = Odometry()
                    message.header.frame_id, message.header.stamp = 'map', rospy.Time.from_sec(stamp)
                    message.pose.pose.position.x, message.pose.pose.position.y = x, y
                    message.pose.pose.orientation.w = 1.
                    message.pose.covariance[0] = message.pose.covariance[7] = .01
                    odom_pub.publish(message)
                    stamp += .05
                time.sleep(.05)

        threading.Thread(target=odom_loop, daemon=True).start()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and status['latest'] is None:
            time.sleep(.05)
        lease_pub.publish(UInt64(data=1))
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and not (status['allowed'] and status['lease'] == 1):
            lease_pub.publish(UInt64(data=1))
            time.sleep(.1)
        result = {'initial_tracking': bool(status['allowed']),
                  'initial_lease': status['lease'],
                  'initial_phase': (status['latest'] or {}).get('phase')}
        with lock:
            initial_calls = (state['candidate'] or {}).get('calls') or 0
            state['healthy'] = False
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and (status['latest'] or {}).get('phase') != 'RELOCALIZING':
            time.sleep(.05)
        result['loss_phase'] = (status['latest'] or {}).get('phase')
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            with lock:
                candidate_now = state['candidate']
            if (candidate_now and candidate_now.get('accepted')
                    and candidate_now.get('calls', 0) > initial_calls
                    and (status['backend'] or {}).get('lease') == 2):
                break
            time.sleep(.05)
        result['backend_after_relocalize'] = status['backend']
        result['lease_before_resume'] = status['lease']
        with lock:
            candidate_now = state['candidate']
            accepted = (candidate_now or {}).get('accepted')
            state['x'], state['y'] = ((accepted['pose'][0], accepted['pose'][1])
                                      if accepted else (truth.x, truth.y))
            state['healthy'] = True
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (status['allowed'] and status['lease'] == 2):
            time.sleep(.05)
        with lock:
            candidate = state['candidate']
            truth_now = list(state['truth'])
        result.update({'recovered': bool(status['allowed']),
                       'final_lease': status['lease'],
                       'final_phase': (status['latest'] or {}).get('phase'),
                       'candidate': candidate,
                       'truth_xy': truth_now,
                       'candidate_error_m': (math.hypot(candidate['accepted']['pose'][0] - truth_now[0],
                                                        candidate['accepted']['pose'][1] - truth_now[1])
                                             if candidate and candidate.get('accepted') else None),
                       'relocalize_calls': (candidate or {}).get('calls')})
        result['ok'] = bool(result['initial_tracking'] and result['loss_phase'] == 'RELOCALIZING'
                            and result['backend_after_relocalize'] and result['recovered']
                            and result['final_lease'] == 2 and result['candidate_error_m'] is not None
                            and result['candidate_error_m'] <= .5)
        print(json.dumps(result, ensure_ascii=False, indent=1))
        return 0 if result['ok'] else 6
    finally:
        try:
            stop_thread.set()
        except Exception:
            pass
        for process in bbs_nodes + [a6, gazebo, master]:
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
