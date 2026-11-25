#!/usr/bin/env python3
"""定位恢复探针：把定位数据闸门恢复后，断言车**不自行复动**。

为什么必须有它：`motion_interrupt_probe --mode stop` 只验"断流后会停"，不验"恢复后不会自己走"。
后者是真实风险——定位恢复瞬间位姿可能跳变，局部规划器以为"离目标还远"就会重新起步，
这在验收里属于"定位失效未安全收敛"。

判据：
  - `/hdl_odom` 恢复后有消息（定位确实回来了）；
  - 观察窗内 `/sim/odom_gt`（真值）位移 ≤ --tol-m（默认 0.05 m）。

用法：
  python3 scripts/verify/loc_recovery_probe.py --seconds 6 --out <ev>/loc_recovery.json
"""
import argparse
import json
import math
import time

import rospy
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--tol-m', type=float, default=0.05)
    ap.add_argument('--pose-topic', default='/hdl_odom')
    ap.add_argument('--gt-topic', default='/sim/odom_gt')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('loc_recovery_probe', anonymous=True)
    state = {'est_n': 0, 'gt_first': None, 'gt_last': None}

    def on_est(_m):
        state['est_n'] += 1

    def on_gt(m):
        p = m.pose.pose.position
        if state['gt_first'] is None:
            state['gt_first'] = (p.x, p.y)
        state['gt_last'] = (p.x, p.y)

    rospy.Subscriber(args.pose_topic, Odometry, on_est, queue_size=20)
    rospy.Subscriber(args.gt_topic, Odometry, on_gt, queue_size=20)

    t0 = time.monotonic()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() - t0 < args.seconds:
        rate.sleep()

    moved = None
    if state['gt_first'] and state['gt_last']:
        moved = math.hypot(state['gt_last'][0] - state['gt_first'][0],
                           state['gt_last'][1] - state['gt_first'][1])
    checks = {
        'pose_resumed': state['est_n'] > 0,
        'no_remotion': (moved is not None and moved <= args.tol_m),
    }
    res = {'seconds': args.seconds, 'pose_msgs': state['est_n'],
           'moved_m': round(moved, 4) if moved is not None else None,
           'tol_m': args.tol_m, 'checks': checks,
           'pass': all(checks.values())}
    text = json.dumps(res, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')
    return 0 if res['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
