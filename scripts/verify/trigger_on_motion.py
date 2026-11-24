#!/usr/bin/env python3
"""在 /cmd_vel 首个非零帧出现的瞬间执行注入动作，不依赖任何 sleep 值。

为什么要这个：车在 tick 0.6 / step 0.3 下走 1.2 m 只要约 2.4 s，运动窗口极短。
用固定 sleep 对准这个窗口反复失败（1.2 s 太早车没起步、45 s 太晚车已停），
而"太早注入"会让定位在起步前就断掉 —— 车永远起不来，测出来全是零。

闭环触发把"什么时候注入"交给 cmd_vel 自己：第一帧非零 = 车真的动了 = 此刻注入。
"""
import argparse
import math
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist

STOP_SPEED = 0.02


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--on-speed', type=float, default=STOP_SPEED)
    ap.add_argument('--action', default='rosnode kill /hdl_gate',
                    help='注入命令；默认杀掉定位闸门 relay，只断数据')
    ap.add_argument('--hold-s', type=float, default=12.0, help='注入后继续观察的秒数')
    ap.add_argument('--timeout-s', type=float, default=40.0, help='等首帧非零的上限')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    state = {'fired': False, 'at': None, 'frames': 0, 'first': None}

    def on_cmd(msg):
        state['frames'] += 1
        v = math.hypot(msg.linear.x, msg.linear.y)
        if state['first'] is None and v >= args.on_speed:
            state['first'] = (time.monotonic(), v)

    rospy.init_node('trigger_on_motion', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    t0 = time.monotonic()
    # 等首帧非零
    while not rospy.is_shutdown() and state['first'] is None:
        if time.monotonic() - t0 > args.timeout_s:
            print('TIMEOUT: %.1f s 内未见非零 cmd（frames=%d）'
                  % (args.timeout_s, state['frames']))
            return 2
        time.sleep(0.01)

    fire_at, fire_v = state['first']
    rc = subprocess.call(args.action.split())
    print('INJECT at %.3f (v=%.3f, %s) rc=%d' % (fire_at, fire_v, args.action, rc))

    # 注入后继续记录，直到 hold_s
    t_fire = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - t_fire < args.hold_s:
        time.sleep(0.05)

    if args.out:
        with open(args.out, 'w') as stream:
            stream.write('fire_at=%.3f v=%.3f frames=%d rc=%d\n'
                         % (fire_at, fire_v, state['frames'], rc))
    return 0


if __name__ == '__main__':
    sys.exit(main())
