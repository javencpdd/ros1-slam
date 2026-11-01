#!/usr/bin/env python3
"""Localization source adapter and GT/LIO error monitor.

Sources:
  sim_gt    - /sim/odom_gt (Gazebo WorldPose)
  faster_lio- /Odometry (faster-lio)
  ndt       - /ndt_pose (monitor-only by default; not a navigation source)
The node publishes the selected pose on the local planner contract and a
rolling GT-vs-selected error for later comparison. NDT is deliberately marked
not navigation-ready until a rejection/lease gate is wired.
"""
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_from_odom(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def error_between(gt, estimate):
    dx, dy, dz = (estimate[i] - gt[i] for i in range(3))
    dyaw = math.atan2(math.sin(yaw_of(estimate[3:]) - yaw_of(gt[3:])),
                      math.cos(yaw_of(estimate[3:]) - yaw_of(gt[3:])))
    return {'xy_m': math.hypot(dx, dy), 'z_m': abs(dz),
            'yaw_rad': abs(dyaw), 'distance_m': math.sqrt(dx * dx + dy * dy + dz * dz)}


def main():
    rospy.init_node('v8_localization_source')
    source = rospy.get_param('~source', 'sim_gt')
    if source not in ('sim_gt', 'faster_lio', 'ndt'):
        raise SystemExit('unsupported localization source: %s' % source)
    frame = rospy.get_param('~frame', 'map')
    max_age = float(rospy.get_param('~max_age_s', 1.0))
    navigation_allowed = bool(rospy.get_param('~navigation_allowed', source != 'ndt'))
    gt_topic = rospy.get_param('~gt_topic', '/sim/odom_gt')
    lio_topic = rospy.get_param('~lio_topic', '/Odometry')
    ndt_topic = rospy.get_param('~ndt_topic', '/ndt_pose')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    ok_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
    status_topic = rospy.get_param('~status_topic', '/v8/local/localization_status')
    error_topic = rospy.get_param('~error_topic', '/v8/local/localization_error')
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=20)
    ok_pub = rospy.Publisher(ok_topic, Bool, queue_size=1)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10)
    error_pub = rospy.Publisher(error_topic, String, queue_size=20)
    state = {'gt': None, 'gt_at': -1e100, 'selected': None, 'selected_at': -1e100,
             'errors': [], 'count': 0, 'last_error': None}

    def publish_ok(ok):
        ok_pub.publish(Bool(data=bool(ok)))

    def select(pose, now, label):
        state['selected'], state['selected_at'] = pose, now
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = frame, rospy.Time.now()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose[:3]
        message.pose.orientation.x, message.pose.orientation.y = pose[3], pose[4]
        message.pose.orientation.z, message.pose.orientation.w = pose[5], pose[6]
        pose_pub.publish(message)
        if state['gt'] is not None:
            error = error_between(state['gt'], pose)
            state['last_error'] = error
            state['errors'].append(error)
            state['count'] += 1
            error_pub.publish(String(data=json.dumps(dict(error, source=label),
                                                     ensure_ascii=False)))
        fresh = now - state['selected_at'] <= max_age
        publish_ok(navigation_allowed and fresh)
        status_pub.publish(String(data=json.dumps({
            'ok': navigation_allowed and fresh, 'source': label,
            'navigation_allowed': navigation_allowed,
            'age_s': max(0., now - state['selected_at']),
            'samples': state['count'], 'last_error': state['last_error']},
            ensure_ascii=False)))

    def on_gt(message):
        clock = message.header.stamp.to_sec()
        state['gt'], state['gt_at'] = pose_from_odom(message), clock
        if source == 'sim_gt':
            select(state['gt'], clock, 'sim_gt')

    def on_lio(message):
        if source == 'faster_lio':
            select(pose_from_odom(message), message.header.stamp.to_sec(), 'faster_lio')

    def on_ndt(message):
        if source != 'ndt':
            return
        p, q = message.pose.position, message.pose.orientation
        select((p.x, p.y, p.z, q.x, q.y, q.z, q.w),
               message.header.stamp.to_sec() if message.header.stamp.to_sec() > 0 else time.monotonic(),
               'ndt')

    rospy.Subscriber(gt_topic, Odometry, on_gt, queue_size=20)
    if source == 'faster_lio':
        rospy.Subscriber(lio_topic, Odometry, on_lio, queue_size=20)
    if source == 'ndt':
        rospy.Subscriber(ndt_topic, PoseStamped, on_ndt, queue_size=10)
    rospy.loginfo('localization source=%s navigation_allowed=%s', source, navigation_allowed)
    print('READY', flush=True)
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        fresh = state['selected'] is not None and now - state['selected_at'] <= max_age
        publish_ok(navigation_allowed and fresh)
        status_pub.publish(String(data=json.dumps({
            'ok': navigation_allowed and fresh, 'source': source,
            'navigation_allowed': navigation_allowed,
            'fresh': fresh, 'samples': state['count'],
            'last_error': state['last_error']}, ensure_ascii=False)))
        rate.sleep()


if __name__ == '__main__':
    main()
