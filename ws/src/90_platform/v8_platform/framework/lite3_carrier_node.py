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
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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

    def on_odom(self, message):
        with self.lock:
            try:
                self._accept(message, _pose_from_odom)
            except (PoseError, AttributeError, TypeError, ValueError) as error:
                self._invalidate(getattr(error, 'code', 'E_POSE_INPUT'))

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
            healthy = status['ok'] and pose is not None
            # The final check and queued publishes share the callback lock:
            # an invalidation cannot complete before an old healthy publish.
            if healthy:
                self.pose_pub.publish(pose)
            self.loc_pub.publish(Bool(data=healthy))
            self.status_pub.publish(String(data=json.dumps(dict(
                status, source=self.pose_source, ok=healthy,
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
