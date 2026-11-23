#!/usr/bin/env python3
"""量测 HDL 绝对定位精度：采样 /hdl_odom 与 /sim/odom_gt，看误差是恒定偏置还是发散。

为什么要区分这两者：
  恒定偏置 ⇒ 坐标系/地图对齐问题，可以修（比如 map→odom 的初值或 z_align）；
  随时间发散 ⇒ 配准在该场景退化（点云特征少），只能换位置或改参数，不能靠"等它收敛"。
真值只用于**核对**，不参与任何控制或判定输入。

输出：mean/max XY 误差、yaw 误差、首尾误差漂移（判断是否发散）。
"""
import argparse
import json
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Acc(object):
    def __init__(self):
        self.est = None
        self.gt = None
        self.samples = []
        rospy.Subscriber('/hdl_odom', Odometry, self.on_est, queue_size=20)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_gt, queue_size=20)

    def on_est(self, m):
        p = m.pose.pose
        # wall clock：仿真时间在暂停时不动，用它计时会让采样循环死等
        self.est = (p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('loc_accuracy', anonymous=True)
    acc = Acc()
    t0 = time.monotonic()
    rows = []
    try:
        while not rospy.is_shutdown() and time.monotonic() - t0 < args.duration:
            if acc.est is not None and acc.gt is not None:
                ex, ey = acc.est[0] - acc.gt[0], acc.est[1] - acc.gt[1]
                rows.append({
                    't': round(acc.est[3] - t0, 2),
                    'dx': round(ex, 3), 'dy': round(ey, 3),
                    'xy': round(math.hypot(ex, ey), 3),
                    'dyaw': round(wrap(acc.est[2] - acc.gt[2]), 3),
                })
            time.sleep(0.1)
    except rospy.ROSInterruptException:
        pass

    out = {'duration_s': args.duration, 'samples': len(rows)}
    if rows:
        xy = [r['xy'] for r in rows]
        out['xy_mean_m'] = round(sum(xy) / len(xy), 4)
        out['xy_max_m'] = round(max(xy), 4)
        out['dx_mean'] = round(sum(r['dx'] for r in rows) / len(rows), 3)
        out['dy_mean'] = round(sum(r['dy'] for r in rows) / len(rows), 3)
        out['dyaw_mean_rad'] = round(sum(r['dyaw'] for r in rows) / len(rows), 4)
        out['dyaw_max_rad'] = round(max(abs(r['dyaw']) for r in rows), 4)
        # 漂移：后 1/3 与 前 1/3 的误差均值差；> 0.05 m 视为发散
        n = max(1, len(rows) // 3)
        head = sum(r['xy'] for r in rows[:n]) / n
        tail = sum(r['xy'] for r in rows[-n:]) / n
        out['drift_head_m'] = round(head, 4)
        out['drift_tail_m'] = round(tail, 4)
        out['diverging'] = abs(tail - head) > 0.05
        out['offset_constant'] = (not out['diverging']) and out['xy_mean_m'] > 0.15
        out['verdict'] = ('PASS' if out['xy_max_m'] <= 0.15 else
                          ('OFFSET' if out['offset_constant'] else 'DIVERGING'))
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if args.out:
        open(args.out, 'w').write(json.dumps({'summary': out, 'rows': rows},
                                             ensure_ascii=False) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
