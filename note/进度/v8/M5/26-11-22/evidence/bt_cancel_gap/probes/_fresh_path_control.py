#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_fresh_path_control.py —— 「不取消、直接换目标」的对照曲线（T-20261122-04 补）。

## 为什么要有这条对照

恢复判据改成「新目标必须出非空路径并交给局部规划器」之后，实测收敛要 10—19 秒：上游 FAR
的 `/way_point` 不是目标本身，而是它 V-图上的顶点，换目标后要若干规划周期才落到目标上。
这里必须回答一个问题：

  这 10 秒是**故障停的副作用**（恢复不干净），还是**上游 FAR 换目标的固有行为**（与故障无关）？

判法只能是实测对照：同一套栈、同两条目标、同样的间隔，**不发取消**，量同一条收敛曲线。
若对照也在同一量级，则恢复后的收敛慢属于上游固有行为，不能记成恢复缺陷；若对照是秒内、
故障后是 10 秒以上，那才是恢复没做干净。

## 与主探针的关系

不是主探针的替代，只作旁证。它不判 PASS/FAIL（没有安全语义），只打出 `CT_CONVERGE_S`
与末点时间线，供与 `SW_FRESH_CONVERGE_S` 对表。

## 用法（容器内、栈已就绪）

  python3 _fresh_path_control.py --case nocancel --tag nocancel
退出码恒 0；异常走 `CT_ABORT=`。`--case` 是给运行器统一接口用的，本件不用。
"""
import argparse
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_GOAL = (2.0, 1.0)
WARM_PATH_WINDOW_S = 20.0
SETTLE_S = 1.5
SECOND_PATH_WINDOW_S = 25.0

PATH_FRAMES = []
WAYPOINTS = []
GOAL_POINTS = []
POSE = {'xy': None}


def near(xy, target, tol=0.5):
    return abs(xy[0] - target[0]) <= tol and abs(xy[1] - target[1]) <= tol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', default='nocancel')
    parser.add_argument('--tag', default='control')
    parser.add_argument('--second-window', type=float, default=SECOND_PATH_WINDOW_S)
    parser.add_argument('--expect', default='')
    args, _unknown = parser.parse_known_args()

    rospy.init_node('gap_fresh_path_control', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)

    def on_path(msg):
        if not msg.poses:
            return
        last = msg.poses[-1].pose.position
        PATH_FRAMES.append((len(msg.poses), (round(last.x, 2), round(last.y, 2)),
                            time.monotonic()))

    def on_waypoint(msg):
        p = msg.point
        WAYPOINTS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_point_goal(msg):
        p = msg.point
        GOAL_POINTS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_pose(msg):
        p = msg.pose.position
        POSE['xy'] = (round(p.x, 2), round(p.y, 2))

    rospy.Subscriber('/global_path', Path, on_path, queue_size=20)
    rospy.Subscriber('/way_point', PointStamped, on_waypoint, queue_size=20)
    rospy.Subscriber('/goal_point', PointStamped, on_point_goal, queue_size=20)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=5)

    def send_goal(tag, xy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xy[0], xy[1], GOAL_Z
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        print('CT_GOAL_SENT tag=%s xy=(%.2f, %.2f)' % (tag, xy[0], xy[1]), flush=True)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            time.sleep(0.05)

    # ---- 暖机腿：等 robot 站住、上游图初始化完，并确认「首次出路径」是几秒 ----
    # ★先等位姿、再等 3 秒★：首版没有这一步，栈刚起来就发目标，结果位姿一直是 (0,0)、
    # 机器人一步没动、上游 FAR 退回静态环境的「前方 5 米虚拟目标」(5.0, 0.11)，两条目标
    # 都当没收到——那次对照读数作废（`probe_ctl1`）。主探针有 `GOAL_DELAY_S=3.0`，这里必须对齐。
    pose_deadline = time.monotonic() + 20.0
    while time.monotonic() < pose_deadline and not rospy.is_shutdown():
        if POSE['xy'] is not None:
            break
        time.sleep(0.1)
    print('CT_POSE_READY=%s' % (POSE['xy'],), flush=True)
    wait(3.0)
    print('CT_POSE_BEFORE_WARM=%s' % (POSE['xy'],), flush=True)
    warm_before = len(PATH_FRAMES)
    send_goal('warm', FIRST_GOAL)
    warm_sent_at = time.monotonic()
    warm_hit = None
    deadline = warm_sent_at + WARM_PATH_WINDOW_S
    while time.monotonic() < deadline and not rospy.is_shutdown():
        warm_hit = next((f for f in PATH_FRAMES[warm_before:]
                         if f[0] >= 2 and near(f[1], FIRST_GOAL)), None)
        if warm_hit is not None:
            break
        time.sleep(0.2)
    print('CT_WARM_PATH_S=%s（首条贴住暖机目标的路径用了多少秒；None＝没出）'
          % (round(warm_hit[2] - warm_sent_at, 1) if warm_hit else None), flush=True)
    wait(SETTLE_S)

    # ---- 对照组：不取消，直接换目标 ----
    second_before = len(PATH_FRAMES)
    wp_before = len(WAYPOINTS)
    gp_before = len(GOAL_POINTS)
    pose_at_send = POSE['xy']
    send_goal('second', SECOND_GOAL)
    second_sent_at = time.monotonic()

    hit = None
    tails = []
    timeline = []
    deadline = second_sent_at + args.second_window
    while time.monotonic() < deadline and not rospy.is_shutdown():
        frames = [f for f in PATH_FRAMES[second_before:] if f[0] >= 2 and f[2] >= second_sent_at]
        hit = next((f for f in frames if near(f[1], SECOND_GOAL)), None)
        for frame in frames:
            if frame[1] not in tails:
                tails.append(frame[1])
                timeline.append((round(frame[2] - second_sent_at, 1), frame[1]))
        if hit is not None:
            break
        time.sleep(0.2)

    way_pts = [(w[0], w[1]) for w in WAYPOINTS[wp_before:] if w[2] >= second_sent_at]
    goal_pts = [(g[0], g[1]) for g in GOAL_POINTS[gp_before:] if g[2] >= second_sent_at]
    print('CT_POSE_AT_SEND=%s CT_POSE_AT_END=%s' % (pose_at_send, POSE['xy']), flush=True)
    print('CT_GOAL_POINT n=%d HIT=%s' % (len(goal_pts),
                                         any(near(g, SECOND_GOAL, 0.05) for g in goal_pts)),
          flush=True)
    print('CT_WAYPOINT n=%d HIT=%s FIRST=%s'
          % (len(way_pts), any(near(w, SECOND_GOAL) for w in way_pts),
             way_pts[0] if way_pts else None), flush=True)
    print('CT_CONVERGE_S=%s（不取消、直接换目标，首帧末点贴住新目标用了多少秒）'
          % (round(hit[2] - second_sent_at, 1) if hit else None), flush=True)
    print('CT_TAIL_TIMELINE=%s' % (timeline[:8],), flush=True)
    print('CT_TAILS=%s' % (tails[:8],), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
