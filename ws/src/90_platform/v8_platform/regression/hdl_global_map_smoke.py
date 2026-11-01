#!/usr/bin/env python3
"""Smoke: hdl_global_localization with the m1 synthetic PCD and an m1-height BBS slice."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PORT = 11610
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_IP='127.0.0.1',
                  ROS_HOME='/tmp/hdl_global_ros', ROS_LOG_DIR='/tmp/hdl_global_logs',
                  PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/hdl_global_ros').mkdir(exist_ok=True)
Path('/tmp/hdl_global_logs').mkdir(exist_ok=True)


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--out')
    parser.add_argument('--map-min-z', type=float, default=0.05)
    parser.add_argument('--map-max-z', type=float, default=1.0)
    parser.add_argument('--scan-min-z', type=float, default=-0.3)
    parser.add_argument('--scan-max-z', type=float, default=0.3)
    args = parser.parse_args()
    import rospy
    import yaml
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from hdl_global_localization.srv import SetGlobalMap, QueryGlobalLocalization
    master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                              stdout=open('/tmp/hdl_global_master.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
    node = None
    try:
        time.sleep(3)
        rospy.init_node('hdl_global_map_smoke', anonymous=True, disable_signals=True)
        base = HERE.parents[2] / '20_localization' / 'hdl_global_localization' / 'config'
        for name in ('general_config.yaml', 'bbs_config.yaml'):
            values = yaml.safe_load((base / name).read_text())
            for key, value in values.items():
                rospy.set_param('/hdl_global_localization/' + key, value)
        rospy.set_param('/hdl_global_localization/bbs/map_min_z', args.map_min_z)
        rospy.set_param('/hdl_global_localization/bbs/map_max_z', args.map_max_z)
        rospy.set_param('/hdl_global_localization/bbs/scan_min_z', args.scan_min_z)
        rospy.set_param('/hdl_global_localization/bbs/scan_max_z', args.scan_max_z)
        node = subprocess.Popen(['rosrun', 'hdl_global_localization', 'hdl_global_localization_node',
                                 '__name:=hdl_global_localization'], start_new_session=True,
                                stdout=open('/tmp/hdl_global_node.log', 'ab'),
                                stderr=subprocess.STDOUT, env=dict(os.environ))
        for service in ('/hdl_global_localization/set_global_map', '/hdl_global_localization/query'):
            rospy.wait_for_service(service, timeout=30)
        points = read_pcd(args.pcd)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        header = rospy.Header()
        header.frame_id, header.stamp = 'map', rospy.Time.now()
        global_cloud = pc2.create_cloud_xyz32(header, xyz[::5])
        set_map = rospy.ServiceProxy('/hdl_global_localization/set_global_map', SetGlobalMap)
        set_map(global_cloud)
        in_band = xyz[(xyz[:, 2] >= 0.25) & (xyz[:, 2] <= 0.65)]
        scan = in_band[np.linalg.norm(in_band[:, :2], axis=1) < 8.0][::3].copy()
        scan[:, 2] -= 0.45
        query_cloud = pc2.create_cloud_xyz32(header, scan)
        query = rospy.ServiceProxy('/hdl_global_localization/query', QueryGlobalLocalization)
        response = query(5, query_cloud)
        result = {'ok': bool(len(response.poses) > 0),
                  'candidates': len(response.poses),
                  'poses': [[p.position.x, p.position.y, p.position.z,
                             p.orientation.x, p.orientation.y, p.orientation.z,
                             p.orientation.w] for p in response.poses],
                  'errors': list(response.errors),
                  'inlier_fractions': list(response.inlier_fractions),
                  'map_points': int(len(xyz)), 'scan_points': int(len(scan)),
                  'bbs_slice': {'map_min_z': args.map_min_z, 'map_max_z': args.map_max_z,
                                'scan_min_z': args.scan_min_z, 'scan_max_z': args.scan_max_z}}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        for proc in (node, master):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
