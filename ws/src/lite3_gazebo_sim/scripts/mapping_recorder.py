#!/usr/bin/env python3
"""Accumulate a 3D point-cloud map from the Gazebo LiDAR and ground-truth pose.

Simulation mapping baseline: /rslidar_points is transformed with /sim/odom_gt
and the URDF base->rslidar mount, then voxel-accumulated and written to PCD on
shutdown. This is not LIO/SLAM; it is the exact-pose mapping reference used by
the 3D support-map pipeline. Keyboard teleop drives the robot in another shell.
"""
import argparse
import hashlib
import json
import math
from collections import OrderedDict
from pathlib import Path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    s = 2.0 / (x * x + y * y + z * z + w * w)
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def mount_rotation(pitch_deg):
    angle = math.radians(pitch_deg)
    return np.array([[math.cos(angle), 0.0, math.sin(angle)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(angle), 0.0, math.cos(angle)]])


def write_pcd(path, points):
    points = np.asarray(points, dtype=np.float32)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
        'COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS %d\nDATA binary\n' % (len(points), len(points))
    )
    data = np.zeros(len(points), dtype=[('x', '<f4'), ('y', '<f4'),
                                        ('z', '<f4'), ('intensity', '<f4')])
    data['x'], data['y'], data['z'] = points[:, 0], points[:, 1], points[:, 2]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(header.encode('ascii') + data.tobytes())


class Recorder:
    def __init__(self, args):
        self.args = args
        self.voxels = OrderedDict()
        self.scans = 0
        self.odom = None
        self.rotation = mount_rotation(args.pitch_deg)
        self.z_offset = np.array([0.0, 0.0, args.lidar_z])
        self.publisher = rospy.Publisher(args.cloud_topic, PointCloud2, queue_size=1)
        self.status_pub = rospy.Publisher(args.status_topic, String, queue_size=1)
        rospy.Subscriber(args.scan_topic, PointCloud2, self.on_scan, queue_size=2)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_odom, queue_size=20)
        rospy.Timer(rospy.Duration(1.0), self.publish)
        rospy.on_shutdown(self.finish)

    def on_odom(self, message):
        self.odom = message

    def on_scan(self, message):
        if self.odom is None:
            return
        local = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
        if not len(local):
            return
        ranges = np.linalg.norm(local, axis=1)
        local = local[(ranges >= self.args.min_range) & (ranges <= self.args.max_range)]
        if not len(local):
            return
        base = local @ self.rotation.T + self.z_offset
        inside = ((np.abs(base[:, 0]) <= 0.30) & (np.abs(base[:, 1]) <= 0.25)
                  & (base[:, 2] >= -0.18) & (base[:, 2] <= 0.18))
        base = base[~inside]
        pose = self.odom.pose.pose
        world = base @ quat_matrix((pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w)).T
        world += np.array([pose.position.x, pose.position.y, pose.position.z])
        keys = np.floor(world / self.args.voxel).astype(np.int64)
        for key, point in zip(map(tuple, keys), world):
            if key not in self.voxels:
                self.voxels[key] = point
        self.scans += 1

    def cloud(self):
        if not self.voxels:
            return None
        points = np.asarray(list(self.voxels.values()), dtype=np.float32)
        message = PointCloud2()
        message.header.frame_id = self.args.frame
        message.header.stamp = rospy.Time.now()
        message.height, message.width = 1, len(points)
        message.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                          PointField('y', 4, PointField.FLOAT32, 1),
                          PointField('z', 8, PointField.FLOAT32, 1)]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.is_dense = True
        message.data = np.ascontiguousarray(points).tobytes()
        return message

    def publish(self, _event):
        cloud = self.cloud()
        if cloud is not None:
            self.publisher.publish(cloud)
        self.status_pub.publish(String(json.dumps({
            'points': len(self.voxels), 'scans': self.scans,
            'frame': self.args.frame, 'voxel_m': self.args.voxel,
        })))

    def finish(self):
        if getattr(self, '_finished', False):
            return
        self._finished = True
        if not self.voxels:
            rospy.logwarn('[mapping_recorder] no points; nothing written')
            return
        points = np.asarray(list(self.voxels.values()), dtype=np.float32)
        write_pcd(self.args.out, points)
        summary = {
            'out': str(Path(self.args.out).resolve()), 'points': len(points),
            'scans': self.scans, 'frame': self.args.frame, 'voxel_m': self.args.voxel,
            'src_topic': self.args.scan_topic, 'pose_topic': '/sim/odom_gt',
            'lidar_z_m': self.args.lidar_z, 'pitch_deg': self.args.pitch_deg,
            'sha256': hashlib.sha256(Path(self.args.out).read_bytes()).hexdigest(),
        }
        Path(self.args.out).with_suffix('.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        rospy.loginfo('[mapping_recorder] wrote %s (%d points)', self.args.out, len(points))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/opt/ros1-slam/data/maps/mapped_m1.pcd')
    parser.add_argument('--scan-topic', default='/rslidar_points')
    parser.add_argument('--cloud-topic', default='/mapping/cloud')
    parser.add_argument('--status-topic', default='/mapping/status')
    parser.add_argument('--frame', default='odom')
    parser.add_argument('--voxel', type=float, default=0.03)
    parser.add_argument('--min-range', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=12.0)
    parser.add_argument('--lidar-z', type=float, default=0.34)
    parser.add_argument('--pitch-deg', type=float, default=15.0)
    args = parser.parse_args()
    rospy.init_node('mapping_recorder')
    Recorder(args)
    rospy.loginfo('[mapping_recorder] recording %s -> %s', args.scan_topic, args.out)
    rospy.spin()


if __name__ == '__main__':
    main()
