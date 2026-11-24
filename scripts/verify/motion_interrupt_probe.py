#!/usr/bin/env python3
"""运动中断探针：一个进程同时管住「录制 → 闭环注入 → 收尾 → 离线分析」。

为什么要单进程：此前五种方案全部败在**进程管理**上，与被测系统无关——
  · 固定 sleep 注入：1.2 s 太早（车未起步）、45 s 太晚（车已停），而运动窗口只有 ~2.4 s；
  · pkill 收尾：模式匹配到执行它的 shell 自身，把脚本杀了，bag 未关闭；
  · `setsid rosbag & BAGPID=$!`：`$!` 是 setsid 的 pid，SIGINT 打给了包装进程，
    rosbag 收不到、不 flush ⇒ 读出来 0 条消息（实测 318 KB 但 cmd=0）。

本脚本用 subprocess.Popen 直接持有 rosbag 句柄，send_signal(SIGINT) + wait() 保证落盘；
注入由 /cmd_vel 首帧非零闭环触发，不依赖任何 sleep 值。

用法：
  motion_interrupt_probe.py --mode stop|cancel --hold-s 10 --out result.json
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

STOP_SPEED = 0.02


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('stop', 'cancel'), required=True)
    ap.add_argument('--bag', default='')
    ap.add_argument('--on-speed', type=float, default=STOP_SPEED)
    ap.add_argument('--hold-s', type=float, default=10.0)
    ap.add_argument('--wait-motion-s', type=float, default=60.0)
    ap.add_argument('--action', default='rosnode kill /hdl_gate')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    bag_path = args.bag or os.path.abspath('motion.bag')

    # 1) 起 rosbag：直接持有句柄，收尾时对它单独发 SIGINT
    recorder = subprocess.Popen(
        ['rosbag', 'record', '/cmd_vel', '/hdl_odom', '/sim/odom_gt',
         '/v8/local/cancel_ack', '/v8/local/localization_ok',
         '-O', bag_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    state = {'frames': 0, 'first': None}

    def on_cmd(msg):
        state['frames'] += 1
        if state['first'] is None:
            v = math.hypot(msg.linear.x, msg.linear.y)
            if v >= args.on_speed:
                state['first'] = (time.monotonic(), v)

    rospy.init_node('motion_interrupt_probe', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    # 2) 等首帧非零（车真的动了）
    t0 = time.monotonic()
    while not rospy.is_shutdown() and state['first'] is None:
        if time.monotonic() - t0 > args.wait_motion_s:
            break
        time.sleep(0.01)

    fired = state['first'] is not None
    if fired:
        fire_at, fire_v = state['first']
        if args.mode == 'stop':
            subprocess.call(args.action.split())
            act = args.action
        else:
            pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=5)
            rospy.sleep(0.3)
            text = '1:all:operator'
            pub.publish(String(data=text))
            rospy.sleep(0.2)
            pub.publish(String(data=text))
            act = 'cancel ' + text
        print('INJECT at v=%.3f action=%s' % (fire_v, act))
        # 3) 注入后继续观察 hold_s
        t_fire = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - t_fire < args.hold_s:
            time.sleep(0.05)
    else:
        print('NO_MOTION: %.0f s 内未见非零 cmd（frames=%d）' % (args.wait_motion_s, state['frames']))

    # 4) 收尾：对 rosbag 自身发 SIGINT 并等它退出（保证 flush）
    recorder.send_signal(2)
    try:
        recorder.wait(timeout=15)
    except subprocess.TimeoutExpired:
        recorder.kill()

    # 5) 建索引
    subprocess.call(['rosbag', 'reindex', bag_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    result = {'mode': args.mode, 'motion_seen': fired,
              'cmd_frames': state['frames'], 'bag': bag_path,
              'hold_s': args.hold_s}
    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False))
    return 0 if fired else 2


if __name__ == '__main__':
    sys.exit(main())
