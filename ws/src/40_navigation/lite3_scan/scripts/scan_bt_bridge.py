#!/usr/bin/env python3
"""scan_bt_bridge.py — SCAN ↔ v8/BT contract adapter (translate only, never plan).

SCAN owns planning/tracking; the v8 behavior tree owns goal sequencing.  The
BT's planner contract is `/global_path` + `/global_path/status` and its arrival
contract can be an external Bool.  This node maps SCAN onto those:

  /planning/bspline (scan_planner/Bspline) → /global_path (nav_msgs/Path)
                                           → /global_path/status (JSON ok/points)
  /scan_planner/goal (PoseStamped)         → status E_PLAN_RUNNING + arrival False
  /lite3/scan/arrived (Bool)               → /v8/bt/arrived (Bool)
  /LIO/odom_vehicle (Odometry)             → /v8/local/pose (PoseStamped, optional)

The B-spline is sampled with the same de Boor rule as
`verify/p2_goal_plan.py` (degree inferred from the knot vector, never trusted
from `order`).  Arrival is SCAN's own monitor signal (`scan_arrival_monitor`),
not the B-spline end point: the planner adjusts its end point away from the
requested goal, so the trajectory end cannot prove arrival.
"""
import json

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

from scan_planner.msg import Bspline


def de_boor(t, ctrl, knots):
    """De Boor evaluation of a 2D B-spline; degree inferred from the knots."""
    n = len(ctrl)
    if n == 0 or len(knots) < 3:
        return None
    p = len(knots) - n - 1
    if p < 1 or n < p + 1 or len(knots) < n + p + 1:
        return None
    lo, hi = knots[p], knots[n]
    if hi <= lo or not (lo - 1e-9 <= t <= hi + 1e-9):
        return None
    if t >= hi - 1e-12:
        i = n - 1
    else:
        i = p
        while i < n - 1 and not (knots[i] <= t < knots[i + 1]):
            i += 1
    d = [[float(ctrl[i - p + j][0]), float(ctrl[i - p + j][1])] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            left, right = knots[i - p + j], knots[i + 1 + j - r]
            den = right - left
            a = 0.0 if abs(den) < 1e-12 else (t - left) / den
            a = min(max(a, 0.0), 1.0)
            d[j] = [(1 - a) * d[j - 1][0] + a * d[j][0],
                    (1 - a) * d[j - 1][1] + a * d[j][1]]
    return d[p]


def sample_bspline(bs, samples=120):
    """Sample a scan_planner/Bspline into [(x, y), ...]; [] when malformed."""
    ctrl = [(q.x, q.y) for q in bs.pos_pts]
    knots = list(bs.knots)
    n = len(ctrl)
    if n == 0 or len(knots) < 3:
        return []
    p = len(knots) - n - 1
    if p < 1 or n < p + 1 or len(knots) < n + p + 1:
        return []
    lo, hi = knots[p], knots[n]
    if hi <= lo:
        return []
    count = max(2, int(samples))
    step = (hi - lo) / (count - 1)
    points = []
    for k in range(count):
        q = de_boor(lo + step * k, ctrl, knots)
        if q is not None:
            points.append(q)
    return points


def path_message(points, frame, stamp):
    message = Path()
    message.header.frame_id = frame
    message.header.stamp = stamp
    for x, y in points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def status_message(ok, points=0):
    if ok:
        return String(data=json.dumps({'ok': True, 'points': int(points)}))
    return String(data=json.dumps({'ok': False, 'reason': 'E_PLAN_RUNNING'}))


def main():
    rospy.init_node('scan_bt_bridge')
    bspline_topic = rospy.get_param('~bspline_topic', '/planning/bspline')
    goal_topic = rospy.get_param('~goal_topic', '/scan_planner/goal')
    arrived_topic = rospy.get_param('~arrived_topic', '/lite3/scan/arrived')
    bt_arrived_topic = rospy.get_param('~bt_arrived_topic', '/v8/bt/arrived')
    path_topic = rospy.get_param('~path_topic', '/global_path')
    status_topic = rospy.get_param('~path_status_topic', '/global_path/status')
    pose_in_topic = rospy.get_param('~pose_in_topic', '/LIO/odom_vehicle')
    pose_out_topic = rospy.get_param('~pose_out_topic', '/v8/local/pose')
    frame = rospy.get_param('~frame', 'camera_init')
    samples = int(rospy.get_param('~samples', 120))

    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=1, latch=True)
    arrived_pub = rospy.Publisher(bt_arrived_topic, Bool, queue_size=1, latch=True)

    def on_goal(_message):
        # A new request resets plan readiness and the previous arrival latch.
        status_pub.publish(status_message(False))
        arrived_pub.publish(Bool(data=False))
        rospy.loginfo('SCAN goal observed: path status reset to E_PLAN_RUNNING')

    def on_bspline(message):
        points = sample_bspline(message, samples)
        if len(points) < 2:
            rospy.logwarn_throttle(5.0, 'bspline with %d sampled points ignored', len(points))
            return
        path_pub.publish(path_message(points, frame, rospy.Time.now()))
        status_pub.publish(status_message(True, len(points)))
        rospy.loginfo_throttle(5.0, 'bspline -> %s: %d points', path_topic, len(points))

    def on_arrived(message):
        arrived_pub.publish(Bool(data=message.data))
        rospy.loginfo('SCAN arrival=%s', message.data)

    def on_pose(message):
        if not pose_out_topic:
            return
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        pose_pub.publish(pose)

    pose_pub = rospy.Publisher(pose_out_topic, PoseStamped, queue_size=1) if pose_out_topic else None
    rospy.Subscriber(bspline_topic, Bspline, on_bspline, queue_size=5)
    rospy.Subscriber(goal_topic, PoseStamped, on_goal, queue_size=5)
    rospy.Subscriber(arrived_topic, Bool, on_arrived, queue_size=5)
    if pose_pub is not None:
        rospy.Subscriber(pose_in_topic, Odometry, on_pose, queue_size=5)

    status_pub.publish(status_message(False))
    rospy.loginfo('scan_bt_bridge up: bspline=%s path=%s status=%s arrived=%s->%s pose=%s->%s',
                  bspline_topic, path_topic, status_topic, arrived_topic,
                  bt_arrived_topic, pose_in_topic, pose_out_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
