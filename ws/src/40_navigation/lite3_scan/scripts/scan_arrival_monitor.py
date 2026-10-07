#!/usr/bin/env python3
"""scan_arrival_monitor.py —— SCAN 到达信号观测件（O-1 口径，壳包自有件，只读上游）。

盘上 SCAN 无对外「到达」信号：FSM 判段完成只看轨迹时间耗尽（scan_replan_fsm.cpp:670→:691 静默
回 WAIT_TARGET），controller 的 finish_dist 只把 cmd 置零（:221）。A7 P3 的动态闭环判据因此没有
可挂的实体。本件补该缺口，参考点钉在**我方发出的请求 goal**——不是 bspline 末点、也不是被
adjustGlobalTargetIfOccupied 改写的 end_pt_，因为跟踪层收敛目标是当前轨迹末点（controller
:187-188），拿它当到达必假绿。判据不引用任何上游阈。失败侧不造信号（FSM 无失败事件），超时语义
归调用方 harness。新 goal 即 reset，防上一段 arrived 短路下一段（P4/#7 前置）。
"""
import time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

rospy.init_node('scan_arrival_monitor')

TOL = float(rospy.get_param('~tol_arrive', 0.20))
SETTLE = float(rospy.get_param('~settle_s', 1.0))
POSE_TOPIC = rospy.get_param('~pose_topic', '/LIO/odom_vehicle')
GOAL_TOPIC = rospy.get_param('~goal_topic', '/scan_planner/goal')

state = {'goal': None, 'in_window': False, 'win_t0': None, 'arrived': False}


def publish_arrived(val):
    state['arrived'] = val
    arrived_pub.publish(Bool(data=val))


def goal_cb(msg):
    # 新 goal 即 reset（P4/#7 的多段语义前置：上一段的 arrived 不得短路下一段）
    state['goal'] = (msg.pose.position.x, msg.pose.position.y)
    state['in_window'] = False
    state['win_t0'] = None
    publish_arrived(False)
    rospy.loginfo('goal received at (%.3f, %.3f) -> arrived reset, tol=%.2fm settle=%.1fs',
                  state['goal'][0], state['goal'][1], TOL, SETTLE)


def pose_cb(msg):
    if state['goal'] is None:
        return
    p = msg.pose.pose.position   # nav_msgs/Odometry 的位姿在 pose.pose（写成 pose.position 会每拍抛异常）
    d = ((p.x - state['goal'][0]) ** 2 + (p.y - state['goal'][1]) ** 2) ** 0.5
    dist_pub.publish(Float32(data=d))
    now = time.monotonic()
    if d > TOL:
        state['in_window'] = False
        state['win_t0'] = None
        return
    if not state['in_window']:
        state['in_window'] = True
        state['win_t0'] = now
        return
    if state['arrived']:
        return
    if now - state['win_t0'] >= SETTLE:
        publish_arrived(True)
        rospy.loginfo('arrived: dist=%.3fm held <= %.2fm for %.2fs', d, TOL, now - state['win_t0'])


arrived_pub = rospy.Publisher('/lite3/scan/arrived', Bool, queue_size=1, latch=True)
dist_pub = rospy.Publisher('/lite3/scan/arrival_dist', Float32, queue_size=10)
rospy.Subscriber(GOAL_TOPIC, PoseStamped, goal_cb, queue_size=5)
rospy.Subscriber(POSE_TOPIC, Odometry, pose_cb, queue_size=5)

publish_arrived(False)
rospy.loginfo('SCAN arrival monitor up: pose=%s goal=%s tol=%.2f settle=%.1f',
              POSE_TOPIC, GOAL_TOPIC, TOL, SETTLE)
rospy.spin()
