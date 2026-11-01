#!/usr/bin/env python3
"""Live Gazebo scan -> three BBS resolutions -> candidate consensus -> Gazebo truth."""
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
sys.path.insert(0, str(HERE.parent / 'adapters'))
from candidate_consensus import CandidateConsensus  # noqa: E402

SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
PORT, GAZEBO_PORT = 11630, 11631
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/hdl_consensus_ros',
                  ROS_LOG_DIR='/tmp/hdl_consensus_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/hdl_consensus_ros').mkdir(exist_ok=True)
Path('/tmp/hdl_consensus_logs').mkdir(exist_ok=True)
RESOLUTIONS = ((0.5, 'res05'), (0.2, 'res02'), (0.1, 'res01'))


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
    parser.add_argument('--radius', type=float, default=0.6)
    args = parser.parse_args()
    import rospy
    import yaml
    from gazebo_msgs.msg import ModelStates
    from sensor_msgs import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from hdl_global_localization.srv import SetGlobalMap, QueryGlobalLocalization
    master = gazebo = None
    nodes = []
    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/hdl_consensus_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/hdl_consensus_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('hdl_global_consensus_smoke', anonymous=True, disable_signals=True)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        truth_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)
        names = list(truth_msg.name)
        truth = truth_msg.pose[names.index('lite3_planar')].position
        points = read_pcd(args.pcd)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        header = rospy.Header()
        header.frame_id, header.stamp = 'map', rospy.Time.now()
        global_cloud = pc2.create_cloud_xyz32(header, xyz[::5])
        scan = list(pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True))
        query_cloud = pc2.create_cloud_xyz32(cloud.header, scan)
        base = HERE.parents[2] / '20_localization' / 'hdl_global_localization' / 'config'
        samples = []
        per_node = []
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
                                    stdout=open('/tmp/hdl_consensus_%s.log' % name, 'ab'),
                                    stderr=subprocess.STDOUT, env=dict(os.environ))
            nodes.append(node)
            for service in ('/%s/set_global_map' % name, '/%s/query' % name):
                rospy.wait_for_service(service, timeout=30)
            rospy.ServiceProxy('/%s/set_global_map' % name, SetGlobalMap)(global_cloud)
            response = rospy.ServiceProxy('/%s/query' % name, QueryGlobalLocalization)(
                5, query_cloud)
            poses = [[p.position.x, p.position.y, p.position.z,
                      p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w] for p in response.poses]
            per_node.append({'resolution': resolution, 'node': name, 'poses': poses,
                             'service_errors': list(response.errors)})
            if poses:
                samples.append({'source': resolution, 'pose': poses[0][:3]})
        gate = CandidateConsensus(radius=args.radius, min_support=2)
        accepted = gate.select(samples)
        result = {'truth_xy': [truth.x, truth.y], 'nodes': per_node,
                  'consensus': accepted,
                  'accepted_xy_error_m': (math.hypot(accepted['pose'][0] - truth.x,
                                                    accepted['pose'][1] - truth.y)
                                          if accepted else None),
                  'ok': bool(accepted),
                  'radius': args.radius,
                  'scan_points': len(scan),
                  'bbs_slice': {'map_min_z': .05, 'map_max_z': 1.0,
                                'scan_min_z': -.3, 'scan_max_z': .3}}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        for process in nodes + [gazebo, master]:
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
