#!/usr/bin/env python3
"""Dynamic obstacles from the registered cloud -> /v8/local/obstacles.

It publishes the obstacle heartbeat **only when a real, fresh cloud was processed
successfully and no obstacle was detected** (an empty list is then legitimate).
Any other state — no_cloud / no_pose / unsupported_cloud / tf_missing / error /
blind / stale_frame — publishes **nothing** on the obstacle topic, so the local
planner's input goes stale and it fails closed. The six states are reported on a
**separate** status topic that never participates in the local planner's gating.
It never publishes a velocity or a gait command.
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


MIN_BOX_EXTENT_M = 0.02


def box_minmax(lo, hi, min_extent=MIN_BOX_EXTENT_M):
    """把跟踪框规范成**严格正体积**的三维盒（保留 3 位小数）。

    2026-11-24：薄墙/平面在聚类后某一轴厚度为 0（四舍五入到 3 位后更常见），
    而 local 的 `_obstacles()` 要求三轴都 `lo < hi`，否则整帧按 `E_LOCAL_INPUT` 丢弃，
    表现就是 `E_LOCAL_OBSTACLES_STALE` + 永久零速。这里只把退化轴撑到最小区分厚度，
    不搬动盒子中心、不改变其它轴尺寸。
    """
    out_lo, out_hi = [], []
    for a, b in zip(lo, hi):
        a, b = round(float(a), 3), round(float(b), 3)
        if b - a < min_extent:
            center = (a + b) / 2.0
            a, b = round(center - min_extent / 2.0, 3), round(center + min_extent / 2.0, 3)
            if b - a < min_extent:      # 四舍五入再次压平
                b = round(a + min_extent, 3)
        out_lo.append(a)
        out_hi.append(b)
    return out_lo, out_hi


def should_publish(status):
    """只有 process() 明确回 ok 才允许在障碍话题上发（此时空表＝确认无障碍）。

    单列成模块级纯函数，是为了让"六态一律不发"这条不变量能被无 ROS 的单测钉住——
    此前它正是测试盲区（`grep blind framework/tests` 零命中），
    结果实现与 docstring 打架、local 永久零速却没人发现。
    """
    return isinstance(status, dict) and status.get('state') == 'ok'


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
        # AABB 细化：默认 1.0 = 保持「一个连通分量一个框」的历史行为。
        # 传更小的值才启用递归二分（实测同一帧面积 6.62 -> 1.46 m2、空洞率 78% -> 0.1%，
        # 且未覆盖点数不增加）。膨胀距离不变，所以这是收紧包围盒，不是放宽安全。
        self.min_fill = float(rospy.get_param('~min_fill', 1.0))
        self.min_extent_m = float(rospy.get_param('~min_extent_m', 0.45))
        self.max_depth = int(rospy.get_param('~max_depth', 4))
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
        self.processed_stamp = None   # 已处理的点云帧戳：同一帧重复到达不得二次刷新心跳
        self.last_status = {'state': 'init'}

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher(self.obstacle_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=1)

    def _gate(self, now):
        """纯判据：这一次循环到底该不该发布有效障碍表。

        返回 (publish, state)。state 只用于诊断话题，**不参与 local 的判定**。
        规则（2026-11-25 真人裁定）：
          - 从未收到点云 / 超时未收 ⇒ blind，不发；
          - 点云帧没变（同一 stamp 重复到达）⇒ stale_frame，不发；
          - 其余交给 process()，只有 process 明确回 'ok' 才发。
        任何"未知/异常"一律不发 —— 空表等于给 local 发"这条路是通的"，
        在没看过点云的时候发空表，就是把"感知没起来"伪装成"一路畅通"。
        """
        if self.blind_timeout_s > 0:
            if self.last_cloud_wall is None:
                return False, {'state': 'blind', 'silent_s': 0.0}
            silent = now - self.last_cloud_wall
            if silent > self.blind_timeout_s:
                return False, {'state': 'blind', 'silent_s': round(silent, 2)}
        if self.cloud_stamp is not None and self.cloud_stamp == self.processed_stamp:
            # ★这一拍没有新帧（轮询 10Hz 与点云 10Hz 常撞帧），**不是**"数据陈旧"★：
            # 只跳过本次发布（避免用旧帧重复刷新心跳＝假健康），但状态要如实叫 duplicate_frame，
            # 且不得把 heartbeat_valid 打成 false（此前误标 stale_frame，被 local 当异常读了）。
            return False, {'state': 'duplicate_frame', 'stamp': round(self.cloud_stamp, 3),
                           'note': 'no new cloud frame this tick'}
        return True, None

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
        boxes = dyn.cluster_boxes(centers, keys, self.min_points, self.max_objects,
                                  min_fill=self.min_fill, min_extent_m=self.min_extent_m,
                                  max_depth=self.max_depth, voxel_m=self.voxel_m)
        dt = (now - self.previous_time) if self.previous_time is not None else 0.0
        if self.tracker is not None and dt > 0:
            tracks = self.tracker.update(boxes, dt)
            payload = []
            for track in tracks:
                payload.append({
                    'min': box_minmax(track.box()['min'], track.box()['max'])[0],
                    'max': box_minmax(track.box()['min'], track.box()['max'])[1],
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
            now = rospy.get_time()
            publish, early = self._gate(now)
            if early is not None:
                status = dict(early)
                status['heartbeat_valid'] = False
            else:
                try:
                    payload, status = self.process()
                except Exception as error:      # 异常也不得刷新心跳
                    payload, status = None, {'state': 'error', 'detail': str(error)[:120]}
                self.processed_stamp = self.cloud_stamp
                ok = should_publish(status)
                status['heartbeat_valid'] = ok
                if ok:
                    # 只有"真实点云 + 成功处理"才允许发；此时空表才等价于"确认无障碍"
                    self.pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
                else:
                    status['published'] = False
            status['latency_ms'] = round((time.monotonic() - started) * 1000.0, 1)
            # 诊断话题：六态都发，且**不被 local 订阅**，只用于观察
            self.status_pub.publish(String(data=json.dumps(status)))
            rate.sleep()


