#!/usr/bin/env python3
"""Live Gazebo scan -> hdl_global_localization query -> candidate vs Gazebo truth."""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
PORT, GAZEBO_PORT = 11620, 11621
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/hdl_global_live_ros',
                  ROS_LOG_DIR='/tmp/hdl_global_live_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/hdl_global_live_ros').mkdir(exist_ok=True)
Path('/tmp/hdl_global_live_logs').mkdir(exist_ok=True)


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
        n = int([line.split()[1] for line in header if line.startswith(b'POINTS')][0])
        data = stream.read()
    dtype = np.dtype([(fields[i].decode(), 'f%d' % sizes[i]) for i in range(len(fields))])
    return np.frombuffer(data, dtype=dtype, count=n)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--out')
    parser.add_argument('--map-min-z', type=float, default=0.05)
    parser.add_argument('--map-max-z', type=float, default=1.0)
    parser.add_argument('--scan-min-z', type=float, default=-0.3)
    parser.add_argument('--scan-max-z', type=float, default=0.3)
    parser.add_argument('--map-resolution', type=float, default=0.2)
    args = parser.parse_args()
    import rospy
    import yaml
    from gazebo_msgs.msg import ModelStates
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from hdl_global_localization.srv import SetGlobalMap, QueryGlobalLocalization
    master = gazebo = node = None
    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/hdl_global_live_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/hdl_global_live_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('hdl_global_live_smoke', anonymous=True, disable_signals=True)
        base = HERE.parents[2] / '20_localization' / 'hdl_global_localization' / 'config'
        for name in ('general_config.yaml', 'bbs_config.yaml'):
            values = yaml.safe_load((base / name).read_text())
            for key, value in values.items():
                rospy.set_param('/hdl_global_localization/' + key, value)
        rospy.set_param('/hdl_global_localization/bbs/map_min_z', args.map_min_z)
        rospy.set_param('/hdl_global_localization/bbs/map_max_z', args.map_max_z)
        rospy.set_param('/hdl_global_localization/bbs/scan_min_z', args.scan_min_z)
        rospy.set_param('/hdl_global_localization/bbs/scan_max_z', args.scan_max_z)
        rospy.set_param('/hdl_global_localization/bbs/map_resolution', args.map_resolution)
        node = subprocess.Popen(['rosrun', 'hdl_global_localization', 'hdl_global_localization_node',
                                 '__name:=hdl_global_localization'], start_new_session=True,
                                stdout=open('/tmp/hdl_global_live_node.log', 'ab'),
                                stderr=subprocess.STDOUT, env=dict(os.environ))
        for service in ('/hdl_global_localization/set_global_map', '/hdl_global_localization/query'):
            rospy.wait_for_service(service, timeout=30)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        truth_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)
        names = list(truth_msg.name)
        truth = truth_msg.pose[names.index('lite3_planar')].position
        points = read_pcd(args.pcd)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        header = rospy.Header()
        header.frame_id, header.stamp = 'map', rospy.Time.now()
        set_map = rospy.ServiceProxy('/hdl_global_localization/set_global_map', SetGlobalMap)
        set_map(pc2.create_cloud_xyz32(header, xyz[::5]))
        query_cloud = pc2.create_cloud_xyz32(cloud.header,
                                             list(pc2.read_points(cloud, field_names=('x', 'y', 'z'),
                                                                  skip_nans=True)))
        query = rospy.ServiceProxy('/hdl_global_localization/query', QueryGlobalLocalization)
        response = query(5, query_cloud)
        poses = [[p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                 for p in response.poses]
        errors = [math.hypot(p[0] - truth.x, p[1] - truth.y) for p in poses]
        best = min(range(len(errors)), key=lambda i: errors[i]) if errors else None
        result = {'ok': bool(poses), 'candidates': len(poses), 'poses': poses,
                  'posthoc_truth_xy': [truth.x, truth.y],
                  'posthoc_xy_errors_m': errors,
                  'best_candidate': best,
                  'best_xy_error_m': errors[best] if best is not None else None,
                  'service_errors': list(response.errors),
                  'inlier_fractions': list(response.inlier_fractions),
                  'map_points_in': int(len(xyz)), 'scan_points': int(len(list(
                      pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)))),
                  'bbs_slice': {'map_min_z': args.map_min_z, 'map_max_z': args.map_max_z,
                                'scan_min_z': args.scan_min_z, 'scan_max_z': args.scan_max_z,
                                'map_resolution': args.map_resolution}}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        for process in (node, gazebo, master):
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
