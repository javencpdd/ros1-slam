#!/usr/bin/env python3
"""发目标直到车真的动了就停手 —— 给「取消安全」测试造一个干净的前提。

为什么需要它：取消测试要求「取消之后不再有新任务进来」，否则系统重新起步是
**正确行为**，会被误读成"取消没生效"。但若目标只发固定 3 s，BT 往往还没收到
就开始规划，车压根不动（实测 cmd_frames=2212 但零非零帧）。

本节点的规则：
  · 车未动 ⇒ 每 rate_s 秒重发一次 goal（topic 无 latch，BT 漏帧也能补上）；
  · 车一旦动了（|v| ≥ on_speed）⇒ 立刻停发，并再等 grace_s 秒退出。

这样目标的存在期恰好止于运动开始，取消注入在其后，判据不会被新任务污染。

用法：
  goal_until_motion.py --x 5.2 --y 1.8 --timeout-s 45 --out goal.json
"""
import argparse
import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, default=0.0)
    ap.add_argument('--frame', default='map')
    ap.add_argument('--topic', default='/v8/bt/goal')
    ap.add_argument('--start-topic', default='/v8/3d/global/start')
    ap.add_argument('--on-speed', type=float, default=0.02)
    ap.add_argument('--rate-s', type=float, default=0.4)
    ap.add_argument('--start-rate-s', type=float, default=1.0)
    ap.add_argument('--timeout-s', type=float, default=45.0)
    ap.add_argument('--grace-s', type=float, default=0.3)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('goal_until_motion', anonymous=True)
    goal_pub = rospy.Publisher(args.topic, PoseStamped, queue_size=5)
    start_pub = rospy.Publisher(args.start_topic, PoseStamped, queue_size=5)

    state = {'moved_at': None, 'max_v': 0.0}

    def on_cmd(msg):
        v = math.hypot(msg.linear.x, msg.linear.y)
        if v > state['max_v']:
            state['max_v'] = v
        if state['moved_at'] is None and v >= args.on_speed:
            state['moved_at'] = time.monotonic()

    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    # 等 publisher 与 BT 的订阅握手（rospy 首帧容易丢）
    t_conn = time.monotonic()
    while goal_pub.get_num_connections() == 0 and time.monotonic() - t_conn < 20.0:
        time.sleep(0.1)

    def make():
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = args.frame
        m.pose.position.x = args.x
        m.pose.position.y = args.y
        m.pose.position.z = args.z
        m.pose.orientation.w = 1.0
        return m

    t0 = time.monotonic()
    n_start = n_goal = 0
    next_goal = 0.0
    next_start = 0.0
    while state['moved_at'] is None and time.monotonic() - t0 < args.timeout_s:
        now = time.monotonic() - t0
        if now >= next_start:
            start_pub.publish(make())
            n_start += 1
            next_start = now + args.start_rate_s
        if now >= next_goal:
            goal_pub.publish(make())
            n_goal += 1
            next_goal = now + args.rate_s
        time.sleep(0.02)

    moved = state['moved_at'] is not None
    if moved:
        time.sleep(args.grace_s)  # 让最后一帧 goal 落地，然后彻底不再发
    result = {'moved': moved, 'max_v': round(state['max_v'], 3),
              'publish_s': round(time.monotonic() - t0, 2),
              'n_goal': n_goal, 'n_start': n_start,
              'conns': goal_pub.get_num_connections()}
    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if moved else 2


if __name__ == '__main__':
    sys.exit(main())
