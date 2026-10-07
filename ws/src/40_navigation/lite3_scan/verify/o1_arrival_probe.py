#!/usr/bin/env python3
"""o1_arrival_probe.py —— scan_arrival_monitor 的注入式功能用例（v_scan_o1.sh 第三节调用）。

无车可跑：只喂假 goal 与假里程计，验观测件自身的语义（阈内持续窗、跳变、新 goal reset、窗被打断）。
不验「车真到没到」——那是 A-4/A-6，归 P3（需 J3 速度指令接 M6 与 /Odometry 双写者先闭合）。
每个用例自采 /lite3/scan arrival_dist 作为「件确实收到注入」的旁证，避免把「没收到」当成「没到达」。
"""
import time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

TOL = 0.20
SETTLE = 1.0

seen = {'arr': None, 'dist': None}
rospy.init_node('o1_arrival_probe')
rospy.Subscriber('/lite3/scan/arrived', Bool, lambda m: seen.__setitem__('arr', bool(m.data)), queue_size=20)
rospy.Subscriber('/lite3/scan/arrival_dist', Float32, lambda m: seen.__setitem__('dist', round(float(m.data), 3)), queue_size=20)
gp = rospy.Publisher('/scan_planner/goal', PoseStamped, queue_size=5)
op = rospy.Publisher('/LIO/odom_vehicle', Odometry, queue_size=20)
time.sleep(1.5)


def goal(x, y):
    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = 'world'
    g.pose.position.x, g.pose.position.y, g.pose.orientation.w = x, y, 1.0
    for _ in range(3):
        gp.publish(g)
        time.sleep(0.15)
    time.sleep(0.3)


def hold(x, y, secs):
    """以 50Hz 持续喂位姿 secs 秒（墙钟），并返回其间自采到的最后距离。"""
    t0 = time.time()
    while time.time() - t0 < secs:
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        op.publish(o)
        time.sleep(0.02)
    return seen['dist']


def say(name, ok, note):
    print('O1 %s=%s %s' % (name, 'PASS' if ok else 'FAIL', note))


r = 0
# 1) 阈外：距离 1.0m，持续 2s 也不得报到达
goal(1.0, 0.0)
hold(2.0, 0.0, 2.0)
ok = (seen['arr'] is False) and (seen['dist'] is not None)
say('case1_far', ok, 'dist=%s arrived=%s（旁证 dist 非空=件确实收到注入）' % (seen['dist'], seen['arr']))
r |= 0 if ok else 1

# 2) 阈内持续 > SETTLE：必须由假跳真
goal(1.0, 0.0)
hold(1.05, 0.0, SETTLE + 1.0)
ok = seen['arr'] is True and (seen['dist'] is not None and seen['dist'] <= TOL)
say('case2_near', ok, 'dist=%s arrived=%s（期望 True，阈 %.2f 窗 %.1fs）' % (seen['dist'], seen['arr'], TOL, SETTLE))
r |= 0 if ok else 1

# 3) 新 goal 即 reset：上一段的 True 不得短路下一段
goal(3.0, 0.0)
time.sleep(0.5)
ok = seen['arr'] is False
say('case3_reset', ok, 'arrived=%s（发新 goal 后必须回假）' % (seen['arr'],))
r |= 0 if ok else 1

# 4) 窗被打断：阈内只停 0.4s（< SETTLE）后撤回阈外，补足 SETTLE 后仍不得报到达
goal(3.0, 0.0)
hold(2.90, 0.0, 0.4)
hold(3.60, 0.0, SETTLE + 0.8)
ok = seen['arr'] is False
say('case4_break', ok, 'arrived=%s（持续窗未满足，不得跳真）' % (seen['arr'],))
r |= 0 if ok else 1

# 5) 打断后重新收敛：再持续 SETTLE 以上必须跳真（证第 4 用例是窗断而非件已死）
hold(2.95, 0.0, SETTLE + 1.0)
ok = seen['arr'] is True
say('case4_reconverge', ok, 'dist=%s arrived=%s' % (seen['dist'], seen['arr']))
r |= 0 if ok else 1

raise SystemExit(r)
