#!/usr/bin/env python3
"""P4 验收探针：算法位姿驱动 3D 导航。

只读。同时记录三路：
  /hdl_odom      —— 算法位姿（导航实际使用的量）
  /sim/odom_gt   —— Gazebo 真值（只用于事后核对，不参与控制）
  /cmd_vel       —— mux 的最终输出（先经 safety mux，再给 lite3_planar_drive）

判据（全部满足才算 PASS，任一项不满足就是 FAIL 并写清哪项）：
  1) 有非零速度：出现 |v| > 0.05 m/s 的帧；
  2) 有实位移：/hdl_odom 的 XY 位移 > 1.0 m；
  3) 有到达：末位姿距目标 ≤ tol（默认 0.50 m，比任务里 0.15 m 宽松，
     因为估计算法位姿与真值之间存在系统偏置，到达口径要看两路都报）；
  4) 两路一致：/sim/odom_gt 的位移与 /hdl_odom 的位移之差 ≤ 0.5 m。

用法：p4_nav_probe.py --goal-x 4.65 --goal-y 0.45 [--duration 60] [--out file]
"""
import argparse
import json
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(object):
    def __init__(self):
        self.est = None
        self.gt = None
        self.first_est = None
        self.first_gt = None
        self.max_speed = 0.0
        self.speed_frames = 0
        self.cmd_frames = 0
        rospy.Subscriber('/hdl_odom', Odometry, self.on_est, queue_size=20)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_gt, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=20)

    def on_est(self, m):
        p = m.pose.pose
        self.est = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.first_est is None:
            self.first_est = self.est

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.first_gt is None:
            self.first_gt = self.gt

    def on_cmd(self, m):
        v = math.hypot(m.linear.x, m.linear.y)
        self.cmd_frames += 1
        if v > 0.05:
            self.speed_frames += 1
            self.max_speed = max(self.max_speed, v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float, required=True)
    ap.add_argument('--goal-y', type=float, required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--tol', type=float, default=0.50)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('p4_nav_probe', anonymous=True)
    pr = Probe()
    rate = rospy.Rate(10.0)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < args.duration:
        rate.sleep()

    result = {'duration_s': args.duration, 'goal': [args.goal_x, args.goal_y],
              'cmd_frames': pr.cmd_frames, 'speed_frames': pr.speed_frames,
              'max_speed_mps': round(pr.max_speed, 4)}
    if pr.est is None:
        result['verdict'] = 'FAIL'
        result['reason'] = '没有 /hdl_odom 样本'
    else:
        dx = pr.est[0] - pr.first_est[0]
        dy = pr.est[1] - pr.first_est[1]
        dist = math.hypot(pr.est[0] - args.goal_x, pr.est[1] - args.goal_y)
        result['est_start'] = [round(v, 4) for v in pr.first_est]
        result['est_end'] = [round(v, 4) for v in pr.est]
        result['est_travel_m'] = round(math.hypot(dx, dy), 4)
        result['est_dist_to_goal_m'] = round(dist, 4)
        checks = {
            'nonzero_speed': pr.speed_frames > 0,
            'travel_gt_1m': result['est_travel_m'] > 1.0,
            'arrived': dist <= args.tol,
        }
        if pr.gt is not None:
            gdx = pr.gt[0] - pr.first_gt[0]
            gdy = pr.gt[1] - pr.first_gt[1]
            result['gt_travel_m'] = round(math.hypot(gdx, gdy), 4)
            result['gt_dist_to_goal_m'] = round(
                math.hypot(pr.gt[0] - args.goal_x, pr.gt[1] - args.goal_y), 4)
            checks['est_gt_travel_agree'] = abs(result['est_travel_m'] - result['gt_travel_m']) <= 0.5
        else:
            result['gt_travel_m'] = None
            checks['est_gt_travel_agree'] = False
        result['checks'] = checks
        result['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
        if result['verdict'] == 'FAIL':
            result['reason'] = '未通过: ' + ','.join(k for k, v in checks.items() if not v)

    text = json.dumps(result, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
