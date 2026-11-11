#!/usr/bin/env python3
"""bt_scan_probe.py — drive the BT→SCAN chain and judge the arrival path.

Prerequisites (started by v_scan_bt.sh): m2_gazebo, m2_mapping, 60_safety,
70_scan, scan_bt_nav.  This probe only publishes the goal and observes; it
never writes SCAN topics other than /v8/bt/goal (the BT's input).

Verdict gates (A7 P3 口径, zero self-set values):
  - /global_path/status ready with points>0 within READY_S
  - BT outcome=reached within REACH_S
  - distance from /LIO/odom_vehicle to the requested goal at reached <= 0.35 m
  - SCAN's /lite3/scan/arrived seen True
Exit: 0 PASS, 1 FAIL, 9 INVALID (a prerequisite/topic never appeared).
"""
import json
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

GOAL = (-0.474, 3.598)
READY_S = 30.0
REACH_S = 240.0
END_TOL = 0.35


def main():
    rospy.init_node('bt_scan_probe', anonymous=True)
    state = {'status': None, 'status_at': None, 'ready': None, 'ready_at': None,
             'odom': None, 'arrived': None, 'events': []}

    def on_status(message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        state['status'] = value
        state['status_at'] = time.monotonic()
        key = (value.get('event'), value.get('outcome'), value.get('path_points'))
        if not state['events'] or state['events'][-1] != key:
            state['events'].append(key)

    def on_path_status(message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if value.get('ok') and value.get('points', 0) > 0 and state['ready'] is None:
            state['ready'] = value
            state['ready_at'] = time.monotonic()

    def on_odom(message):
        p = message.pose.pose.position
        state['odom'] = (p.x, p.y)

    def on_arrived(message):
        state['arrived'] = bool(message.data)

    rospy.Subscriber('/v8/bt/status', String, on_status, queue_size=5)
    rospy.Subscriber('/global_path/status', String, on_path_status, queue_size=5)
    rospy.Subscriber('/LIO/odom_vehicle', Odometry, on_odom, queue_size=5)
    rospy.Subscriber('/lite3/scan/arrived', Bool, on_arrived, queue_size=5)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)

    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if state['status'] is not None and state['odom'] is not None:
            break
        time.sleep(0.2)
    if state['status'] is None or state['odom'] is None:
        print(json.dumps({'verdict': 'INVALID', 'reason': 'BT status or odom never seen'}))
        return 9

    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = GOAL[0], GOAL[1], 0.0
    goal.pose.orientation.w = 1.0
    t0 = time.monotonic()
    goal_pub.publish(goal)

    verdict = 'FAIL'
    reason = 'timeout without outcome=reached'
    while time.monotonic() - t0 < REACH_S and not rospy.is_shutdown():
        if state['ready'] is None and time.monotonic() - t0 > READY_S:
            verdict, reason = 'INVALID', 'no ready global path within %.0f s' % READY_S
            break
        value = state['status'] or {}
        if value.get('outcome'):
            if value.get('outcome') == 'reached':
                verdict, reason = 'PASS', 'outcome=reached'
            else:
                verdict, reason = 'FAIL', 'outcome=%s' % value.get('outcome')
            break
        time.sleep(0.25)

    odom = state['odom'] or (float('nan'), float('nan'))
    dist = ((odom[0] - GOAL[0]) ** 2 + (odom[1] - GOAL[1]) ** 2) ** 0.5
    arrived = state['arrived']
    if verdict == 'PASS' and dist > END_TOL:
        verdict, reason = 'FAIL', 'endpoint %.3f m > %.2f m' % (dist, END_TOL)
    if verdict == 'PASS' and arrived is not True:
        verdict, reason = 'FAIL', 'SCAN arrived signal never true'

    print(json.dumps({
        'verdict': verdict, 'reason': reason, 'goal': list(GOAL),
        'path_points': (state['ready'] or {}).get('points'),
        'ready_s': None if state['ready_at'] is None else round(state['ready_at'] - t0, 2),
        'reached_s': None if not (state['status'] or {}).get('outcome')
        else round(state['status_at'] - t0, 2),
        'dist_end_m': round(dist, 3), 'arrived': arrived,
        'outcome': (state['status'] or {}).get('outcome'),
        'outcome_seq': (state['status'] or {}).get('outcome_seq'),
        'planner_mode': (state['status'] or {}).get('planner_mode'),
        'events': state['events'][-8:],
    }, ensure_ascii=False))
    return 0 if verdict == 'PASS' else (9 if verdict == 'INVALID' else 1)


if __name__ == '__main__':
    sys.exit(main())
