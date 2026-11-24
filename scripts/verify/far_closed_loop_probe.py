#!/usr/bin/env python3
"""FAR 导航闭环四项验收采集器：只订阅+发目标/取消，不伪造任何读数。"""
import json, math, sys, time
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String

ev = {'phases': {}}
state = {'pose': None, 'speed': [], 'arrived': None, 'cancel_ack': 0, 'status': []}

def on_pose(m):
    state['pose'] = (m.pose.position.x, m.pose.position.y, time.time())

def on_cmd(m):
    state['speed'].append((math.hypot(m.linear.x, m.linear.y), time.time()))

def on_arr(m):
    state['arrived'] = bool(m.data)

def on_ack(m):
    state['cancel_ack'] += 1

def on_st(m):
    state['status'].append(m.data[:120])

def phase(name, secs, goal=None, cancel_after=None, loc_ok=None):
    state['speed'] = []
    p0 = state['pose'][:2] if state['pose'] else None
    t0 = time.time()
    if loc_ok is not None:
        loc_pub.publish(Bool(data=loc_ok))
    if goal is not None:
        g = PoseStamped(); g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y = goal
        g.pose.orientation.w = 1.0
        goal_pub.publish(g)
    sent_cancel = False
    while time.time() - t0 < secs and not rospy.is_shutdown():
        time.sleep(0.2)
        if cancel_after is not None and not sent_cancel:
            sp = [s for s, _ in state['speed'][-10:]]
            if sp and max(sp) > 0.05 and time.time() - t0 > cancel_after:
                cancel_pub.publish(String(data='1:mission:accept_test'))
                sent_cancel = True
    p1 = state['pose'][:2] if state['pose'] else None
    sp = [s for s, _ in state['speed']]
    ev['phases'][name] = {
        'secs': round(time.time() - t0, 1),
        'samples': len(sp),
        'max_speed': round(max(sp), 3) if sp else 0.0,
        'nonzero_ratio': round(sum(1 for s in sp if s > 0.01) / len(sp), 3) if sp else 0.0,
        'displacement_m': round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 3) if (p0 and p1) else None,
        'arrived_flag': state['arrived'],
        'cancel_ack': state['cancel_ack'],
        'cancel_sent': sent_cancel,
    }

rospy.init_node('far_accept')
goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=2)
cancel_pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=2)
loc_pub = rospy.Publisher('/v8/local/localization_ok', Bool, queue_size=2)
rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=5)
rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)
rospy.Subscriber('/v8/bt/arrived', Bool, on_arr, queue_size=5)
rospy.Subscriber('/v8/local/cancel_ack', String, on_ack, queue_size=5)
rospy.Subscriber('/v8/bt/status', String, on_st, queue_size=50)
time.sleep(3.0)
phase('A_normal', 30.0, goal=(1.0, 0.0))
phase('B_loc_lost', 6.0, loc_ok=False)
phase('C_cancel', 25.0, goal=(1.0, 0.0), cancel_after=4.0)
with open(sys.argv[1], 'w') as fh:
    json.dump(ev, fh, ensure_ascii=False, indent=1)
print(json.dumps(ev, ensure_ascii=False))
