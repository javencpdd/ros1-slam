#!/usr/bin/env python3
"""Dynamic obstacles from the registered cloud -> /v8/local/obstacles.

It always publishes at ~rate_hz (an empty list when there is no cloud or pose),
so it doubles as the local planner's obstacle heartbeat. It never publishes a
velocity or a gait command.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import json
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

import dynamic_obstacles as dyn
from object_tracker import MultiObjectTracker


def cloud_xyz(msg, max_points=80000):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return None
    if any(fields[name].datatype != PointField.FLOAT32 for name in ('x', 'y', 'z')):
        return None
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    count = msg.width * msg.height
    if count == 0:
        return np.empty((0, 3))
    points = np.frombuffer(msg.data, dtype=dtype, count=count)
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) > max_points:
        xyz = xyz[::int(np.ceil(len(xyz) / float(max_points)))]
    return xyz


def transform_points(points, transform):
    q = transform.transform.rotation
    t = transform.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return points.dot(matrix.T) + np.array([t.x, t.y, t.z])


class DynamicObstaclesNode(object):
    def __init__(self):
        self.enable = bool(rospy.get_param('~enable', True))
        self.cloud_topic = rospy.get_param('~cloud_topic', '/cloud_registered')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.rate_hz = float(rospy.get_param('~rate_hz', 10.0))
        self.forward_m = float(rospy.get_param('~roi_forward_m', 6.0))
        self.lateral_m = float(rospy.get_param('~roi_lateral_m', 3.0))
        self.min_height_m = float(rospy.get_param('~min_height_m', 0.15))
        self.max_height_m = float(rospy.get_param('~max_height_m', 1.6))
        self.voxel_m = float(rospy.get_param('~voxel_m', 0.10))
        self.min_points = int(rospy.get_param('~min_points', 8))
        self.max_objects = int(rospy.get_param('~max_objects', 32))
        self.self_radius_m = float(rospy.get_param('~self_radius_m', 0.45))
        self.max_speed_mps = float(rospy.get_param('~max_speed_mps', 1.5))
        self.gate_m = float(rospy.get_param('~gate_m', 0.8))
        self.ttl_s = float(rospy.get_param('~ttl_s', 1.0))
        self.obstacle_topic = rospy.get_param('~obstacle_topic', '/v8/local/obstacles')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/perception/status')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.blind_timeout_s = float(rospy.get_param('~blind_timeout_s', 0.0))
        self.last_cloud_wall = None
        self.use_tracker = bool(rospy.get_param('~use_tracker', True))
        self.tracker = (MultiObjectTracker(
            gate_m=float(rospy.get_param('~track_gate_m', 0.8)),
            max_miss=int(rospy.get_param('~max_miss', 3)),
            sigma_a=float(rospy.get_param('~sigma_a', 0.8)),
            sigma_pos=float(rospy.get_param('~sigma_pos', 0.08)),
            max_speed_mps=self.max_speed_mps,
            max_tracks=self.max_objects) if self.use_tracker else None)

        self.pose = None
        self.cloud = None
        self.cloud_stamp = None
        self.previous_boxes = None
        self.previous_time = None
        self.last_payload = []
        self.last_status = {'state': 'init'}

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher(self.obstacle_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=1)

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_stamp = msg.header.stamp.to_sec() or rospy.get_time()
        self.last_cloud_wall = rospy.get_time()

    def on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def process(self):
        now = rospy.get_time()
        if not self.enable:
            return [], {'state': 'disabled'}
        if self.pose is None:
            return [], {'state': 'no_pose'}
        if self.cloud is None:
            return [], {'state': 'no_cloud'}
        points = cloud_xyz(self.cloud)
        if points is None:
            return [], {'state': 'unsupported_cloud'}
        frame = self.cloud.header.frame_id
        if frame and frame != self.map_frame:
            try:
                transform = self.buffer.lookup_transform(
                    self.map_frame, frame, self.cloud.header.stamp, rospy.Duration(0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                return [], {'state': 'tf_missing', 'detail': str(error)[:80]}
            points = transform_points(points, transform)
        floor = self.pose[2]
        kept = dyn.roi_filter(points, (self.pose[0], self.pose[1]), floor,
                              self.forward_m, self.lateral_m,
                              self.min_height_m, self.max_height_m)
        kept = dyn.self_filter(kept, (self.pose[0], self.pose[1]), self.self_radius_m)
        centers, keys = dyn.voxel_downsample(kept, self.voxel_m)
        boxes = dyn.cluster_boxes(centers, keys, self.min_points, self.max_objects)
        dt = (now - self.previous_time) if self.previous_time is not None else 0.0
        if self.tracker is not None and dt > 0:
            tracks = self.tracker.update(boxes, dt)
            payload = []
            for track in tracks:
                payload.append({
                    'min': [round(v, 3) for v in track.box()['min']],
                    'max': [round(v, 3) for v in track.box()['max']],
                    'velocity': [round(v, 3) for v in track.velocity()],
                    'track_id': track.id, 'age': track.age, 'missed': track.missed,
                    'stamp': round(now, 3),
                    'active_from': round(now - self.ttl_s, 3),
                    'active_until': round(now + self.ttl_s, 3)})
            state = {'state': 'ok', 'points': int(len(points)),
                     'kept': int(len(kept)), 'voxels': int(len(keys)),
                     'objects': len(payload), 'tracks': len(tracks),
                     'detections': len(boxes)}
        else:
            velocities = dyn.associate(self.previous_boxes, boxes, dt,
                                       gate_m=self.gate_m, max_speed_mps=self.max_speed_mps)
            payload = dyn.to_payload(boxes, velocities, now, self.ttl_s)
            state = {'state': 'ok', 'points': int(len(points)),
                     'kept': int(len(kept)), 'voxels': int(len(keys)),
                     'objects': len(boxes), 'tracker': 'off'}
        self.previous_boxes = boxes
        self.previous_time = now
        return payload, state

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            started = time.monotonic()
            try:
                payload, status = self.process()
            except Exception as error:  # keep the heartbeat fail-closed
                payload, status = [], {'state': 'error', 'detail': str(error)[:120]}
            blind = (self.blind_timeout_s > 0
                     and (self.last_cloud_wall is None
                          or rospy.get_time() - self.last_cloud_wall > self.blind_timeout_s))
            if blind:
                # fail-closed: stop publishing so the local planner's heartbeat
                # times out and the robot stops instead of assuming a clear path.
                status = {'state': 'blind',
                          'silent_s': round(rospy.get_time() - (self.last_cloud_wall or rospy.get_time()), 2)}
                self.status_pub.publish(String(data=json.dumps(status)))
                rate.sleep()
                continue
            self.pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
            status['latency_ms'] = round((time.monotonic() - started) * 1000.0, 1)
            status['objects'] = status.get('objects', len(payload))
            self.status_pub.publish(String(data=json.dumps(status)))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('v8_dynamic_obstacles')
    DynamicObstaclesNode().spin()
