#!/usr/bin/env python3
"""A2 停止链运行面 cell：mux hold→零速、释放恢复、源超时→零速。

前置：safe_cmd_vel_mux 已在跑（start_mux.sh），/cmd_vel 唯一写者应为 /mux。
本件只发 /follow/cmd_vel 与 hold 输入、观测 /cmd_vel，不改任何配置。
用法：python3 -B a2_stop_chain.py [--out /work/a2.json]
"""
import argparse
import json
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    rospy.init_node('a2_stop_chain', anonymous=True)

    cmd = {'t': None, 'v': None}
    rospy.Subscriber('/cmd_vel', Twist,
                     lambda m: cmd.update(t=time.time(), v=(m.linear.x, m.linear.y, m.angular.z)))
    pub_src = rospy.Publisher('/follow/cmd_vel', Twist, queue_size=5)
    pub_hold = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1)

    def wait_for(pred, timeout, label):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if cmd['v'] is not None and pred(cmd['v']):
                return True
            time.sleep(0.005)
        raise RuntimeError('timeout waiting for ' + label)

    nonzero = lambda v: abs(v[0]) + abs(v[1]) + abs(v[2]) > 0.01
    zero = lambda v: abs(v[0]) + abs(v[1]) + abs(v[2]) <= 0.01

    rate = rospy.Rate(20)
    result = {'mux_absent': False}

    # 1) 建立非零流
    t0 = time.time()
    while time.time() - t0 < 3.0:
        pub_src.publish(Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        rate.sleep()
    wait_for(nonzero, 5.0, 'nonzero_cmd')
    result['nonzero_before'] = list(cmd['v'])

    # 2) hold → 零速（发一次 hold，继续发源以证明 hold 压过源）
    hold_sent = time.time()
    pub_hold.publish(Bool(data=True))
    t_hold = time.time()
    while time.time() - t_hold < 5.0:
        pub_src.publish(Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        if zero(cmd['v']) and cmd['t'] is not None and cmd['t'] >= hold_sent:
            break
        rate.sleep()
    result['hold_to_zero_wall_s'] = round(cmd['t'] - hold_sent, 4)
    result['hold_zero'] = zero(cmd['v'])

    # 3) 释放 hold → 恢复
    pub_hold.publish(Bool(data=False))
    t_rel = time.time()
    resumed = False
    while time.time() - t_rel < 5.0:
        pub_src.publish(Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        if nonzero(cmd['v']):
            resumed = True
            break
        rate.sleep()
    result['resume_after_s'] = round(time.time() - t_rel, 4) if resumed else None
    result['resumed'] = resumed

    # 4) 停发源 → 0.5 s 超时后零速
    stop_sent = time.time()
    wait_for(zero, 5.0, 'zero_after_source_stop')
    result['source_timeout_to_zero_wall_s'] = round(cmd['t'] - stop_sent, 4)
    result['zero_after_stop'] = zero(cmd['v'])

    ok = (result['hold_zero'] and result['resumed'] and result['zero_after_stop']
          and result['hold_to_zero_wall_s'] is not None)
    result['verdict'] = 'PASS' if ok else 'FAIL'
    text = json.dumps(result, ensure_ascii=False)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
    return 0 if ok else 6


if __name__ == '__main__':
    import sys
    sys.exit(main())
