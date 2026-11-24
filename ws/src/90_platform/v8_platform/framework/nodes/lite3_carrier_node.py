#!/usr/bin/env python3
"""Validated map/base pose and localization health; never a command publisher."""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from common.localization_health import HealthConfig, evaluate as health_evaluate
from common.pose_safety import PoseError, PoseFreshness, finite_vector, normalized_quaternion


def _validate_pose(pose):
    p, q = pose.position, pose.orientation
    finite_vector((p.x, p.y, p.z), 3)
    q.x, q.y, q.z, q.w = normalized_quaternion((q.x, q.y, q.z, q.w))


def _pose_from_odom(message, frame, base='base_link'):
    if message.header.frame_id != frame or message.child_frame_id != base:
        raise PoseError('E_POSE_FRAME')
    pose = PoseStamped()
    pose.header = copy.deepcopy(message.header)
    pose.pose = copy.deepcopy(message.pose.pose)
    _validate_pose(pose.pose)
    return pose


def _pose_from_transform(transform, frame, base='base_link'):
    if transform.header.frame_id != frame or transform.child_frame_id != base:
        raise PoseError('E_POSE_FRAME')
    pose = PoseStamped()
    pose.header = copy.deepcopy(transform.header)
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
    _validate_pose(pose.pose)
    return pose


class Lite3Carrier:
    def __init__(self):
        self.pose_source = rospy.get_param('~pose_source', 'tf')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.localization_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/carrier/status')
        self.input_pose_topic = rospy.get_param('~input_pose_topic', '/odom')
        self.rate_hz = float(rospy.get_param('~rate_hz', 20.0))
        self.stale_s = float(rospy.get_param('~stale_s', 0.5))
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.:
            raise ValueError('rate_hz must be positive and finite')
        if self.pose_source not in ('tf', 'topic'):
            raise ValueError('unknown ~pose_source: %s' % self.pose_source)
        if not self.map_frame or not self.base_frame or self.map_frame == self.base_frame:
            raise ValueError('distinct map_frame/base_frame required')
        self.guard = PoseFreshness(self.stale_s)
        # ★定位健康的三条附加判据（2026-11-24）★：原来只有"来源新鲜"，而 HDL 停止收敛
        # 或跳到别处时 TF 照样在更新 —— 只看新鲜度会一直报健康。收敛与质量没有话题就是
        # "未知"，按 fail-closed 判不健康；唯一降级口 allow_unverified 需显式打开且留痕。
        self.health_cfg = HealthConfig(
            max_plausible_mps=float(rospy.get_param('~max_plausible_mps', 1.5)),
            jump_max_m=float(rospy.get_param('~jump_max_m', 0.50)),
            jump_max_rad=float(rospy.get_param('~jump_max_rad', 0.60)),
            quality_min=float(rospy.get_param('~quality_min', 0.0)),
            quality_stale_s=float(rospy.get_param('~quality_stale_s', 1.0)),
            converged_stale_s=float(rospy.get_param('~converged_stale_s', 1.0)),
            require_quality=bool(rospy.get_param('~require_quality', True)),
            require_converged=bool(rospy.get_param('~require_converged', True)),
            allow_unverified=bool(rospy.get_param('~allow_unverified', False)))
        self.quality_topic = rospy.get_param('~quality_topic', '')
        self.converged_topic = rospy.get_param('~converged_topic', '')
        self.quality = None
        self.quality_at = None
        self.converged = None
        self.converged_at = None
        self.latest_at = None
        self.prev = None          # (x, y, yaw, monotonic)
        self.lock = threading.Lock()
        self.latest = None
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=20)
        self.loc_pub = rospy.Publisher(self.localization_topic, Bool, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5)
        if self.pose_source == 'tf':
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer)
        else:
            rospy.Subscriber(self.input_pose_topic, Odometry, self.on_odom, queue_size=20)
        if self.quality_topic:
            rospy.Subscriber(self.quality_topic, Float64, self.on_quality, queue_size=10)
        if self.converged_topic:
            rospy.Subscriber(self.converged_topic, Bool, self.on_converged, queue_size=10)

    def _invalidate(self, reason):
        self.guard.invalidate(reason)
        self.latest = None

    def _accept(self, message, converter):
        source_stamp = message.header.stamp.to_sec()
        try:
            pose = converter(message, self.map_frame, self.base_frame)
        except (PoseError, AttributeError, TypeError, ValueError) as error:
            # Rejected geometry still records the observed stamp, but cannot
            # renew source progress or release the previous-epoch quarantine.
            self.guard.observe(source_stamp, rospy.Time.now().to_sec(), time.monotonic(),
                               rejection=getattr(error, 'code', 'E_POSE_INPUT'))
            self.latest = None
            return
        status, advanced = self.guard.observe(
            source_stamp, rospy.Time.now().to_sec(), time.monotonic())
        if not status['ok']:
            self.latest = None
            return
        if advanced:
            self.latest = pose
            self.latest_at = time.monotonic()

    def on_quality(self, message):
        self.quality = float(message.data)
        self.quality_at = time.monotonic()

    def on_converged(self, message):
        self.converged = bool(message.data)
        self.converged_at = time.monotonic()

    def on_odom(self, message):
        with self.lock:
            try:
                self._accept(message, _pose_from_odom)
            except (PoseError, AttributeError, TypeError, ValueError) as error:
                self._invalidate(getattr(error, 'code', 'E_POSE_INPUT'))

    def _health(self, pose_fresh, pose):
        """收敛／质量／跳变三条附加判据；无数据即"未知"，由 HealthConfig 决定是否放行。"""
        dt_s = delta_m = delta_rad = None
        if pose is not None and self.prev is not None and self.latest_at is not None:
            dt_s = max(0.0, self.latest_at - self.prev[3])
            delta_m = math.hypot(pose.pose.position.x - self.prev[0],
                                 pose.pose.position.y - self.prev[1])
            q = pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            diff = abs(yaw - self.prev[2])
            delta_rad = min(diff, 2.0 * math.pi - diff)
        now = time.monotonic()
        health = health_evaluate(
            pose_fresh=pose_fresh, dt_s=dt_s, delta_m=delta_m, delta_rad=delta_rad,
            quality=self.quality,
            quality_age_s=None if self.quality_at is None else now - self.quality_at,
            converged=self.converged,
            converged_age_s=None if self.converged_at is None else now - self.converged_at,
            config=self.health_cfg)
        if pose is not None and self.latest_at is not None:
            q = pose.pose.orientation
            self.prev = (pose.pose.position.x, pose.pose.position.y,
                         math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                         self.latest_at)
        return health

    def step(self):
        if self.pose_source == 'tf':
            try:
                transform = self.buffer.lookup_transform(
                    self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.))
                with self.lock:
                    try:
                        self._accept(transform, _pose_from_transform)
                    except (PoseError, AttributeError, TypeError, ValueError) as error:
                        self._invalidate(getattr(error, 'code', 'E_POSE_INPUT'))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                with self.lock:
                    self._invalidate('E_POSE_TF')
        with self.lock:
            status = self.guard.check(rospy.Time.now().to_sec(), time.monotonic())
            if not status['ok']:
                self.latest = None
            pose = copy.deepcopy(self.latest)
            health = self._health(status['ok'], pose)
            healthy = status['ok'] and pose is not None and health['ok']
            # The final check and queued publishes share the callback lock:
            # an invalidation cannot complete before an old healthy publish.
            if healthy:
                self.pose_pub.publish(pose)
            self.loc_pub.publish(Bool(data=healthy))
            self.status_pub.publish(String(data=json.dumps(dict(
                status, source=self.pose_source, ok=healthy, health=health,
                frame=self.map_frame, base_frame=self.base_frame))))
            return healthy

    def spin(self):
        period = 1. / self.rate_hz
        try:
            while not rospy.is_shutdown():
                started = time.monotonic()
                self.step()
                time.sleep(max(0., period - (time.monotonic() - started)))
        finally:
            self.loc_pub.publish(Bool(data=False))


if __name__ == '__main__':
    rospy.init_node('v8_lite3_carrier')
    Lite3Carrier().spin()
