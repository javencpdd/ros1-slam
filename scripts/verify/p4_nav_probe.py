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
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(object):
    def __init__(self, pose_topic='/hdl_odom'):
        self.pose_topic = pose_topic
        self.est = None
        self.gt = None
        self.first_est = None
        self.first_gt = None
        self.max_speed = 0.0
        self.speed_frames = 0
        self.cmd_frames = 0
        self.cmd_times = []          # 采样覆盖判据用：cmd 到达的墙钟时刻
        self.bt_arrived = False      # ★BT 自己的到达事件★：只看真值距离会漏判"规划器没报到"
        # ★按类型只订一次★（同一 topic 建两种类型订阅会让其中一条永远空转，也掩盖类型配错）：
        # M1 的 /hdl_odom 是 Odometry，M2 的 /v8/local/pose 是 PoseStamped。
        resolved = rospy.resolve_name(pose_topic)
        published = dict(rospy.get_published_topics())
        mtype = published.get(resolved) or published.get(pose_topic) or ''
        if 'PoseStamped' in mtype:
            rospy.Subscriber(pose_topic, PoseStamped, self.on_est_pose, queue_size=20)
        elif 'Odometry' in mtype:
            rospy.Subscriber(pose_topic, Odometry, self.on_est_odom, queue_size=20)
        else:
            # 类型未知：等一小会儿再判一次；仍未知则明确报错（不猜类型）
            for _ in range(20):
                rospy.sleep(0.25)
                published = dict(rospy.get_published_topics())
                mtype = published.get(resolved) or ''
                if mtype:
                    break
            if 'PoseStamped' in mtype:
                rospy.Subscriber(pose_topic, PoseStamped, self.on_est_pose, queue_size=20)
            elif 'Odometry' in mtype:
                rospy.Subscriber(pose_topic, Odometry, self.on_est_odom, queue_size=20)
            else:
                rospy.logerr('位姿话题 %s 类型未知（未发布？），本探针不会订阅任何类型', pose_topic)
        # ★BT 的到达事件在 `/v8/bt/status`（std_msgs/String + JSON）里★，字段 `last_event`/`goal_outcome`
        # 取 "reached"。`/v8/bt/arrived` 是 far_stack 里 FAR 适配器的专用话题，support 组合没有它，
        # 用那条会永远判 False（会误伤已通过的基线）。
        rospy.Subscriber('/v8/bt/status', String, self.on_arrived, queue_size=10)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_gt, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=20)

    def on_est_odom(self, m):
        p = m.pose.pose
        self._take_est(p)

    def on_est_pose(self, m):
        self._take_est(m.pose)

    def _take_est(self, p):
        self.est = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.first_est is None:
            self.first_est = self.est

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.first_gt is None:
            self.first_gt = self.gt

    def on_arrived(self, m):
        text = m.data or ''
        if '"reached"' in text or "'reached'" in text:
            self.bt_arrived = True

    def on_cmd(self, m):
        v = math.hypot(m.linear.x, m.linear.y)
        self.cmd_frames += 1
        # ★墙钟，不用 rospy.get_time()★：本探针的窗口是"探针自己的采样时长"，
        # 用仿真钟会被仿真速率带偏（实测 45 仿秒＝18 实秒：目标还没发，窗口已经结束）。
        self.cmd_times.append(time.monotonic())
        if v > 0.05:
            self.speed_frames += 1
            self.max_speed = max(self.max_speed, v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float, required=True)
    ap.add_argument('--goal-y', type=float, required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--pose-topic', default='/hdl_odom',
                    help='算法位姿话题（M2+LIO 组合传 /Odometry）')
    ap.add_argument('--tol', type=float, default=0.50)
    ap.add_argument('--arrive-tol', type=float, default=0.15,
                    help='真实到达容差（用 /sim/odom_gt 判，不是估计位姿）')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('p4_nav_probe', anonymous=True)
    pr = Probe(pose_topic=args.pose_topic)
    rate = rospy.Rate(10.0)
    # ★窗口用墙钟★：/clock 是仿真时间，实测 45 仿秒只对应约 18 实测秒 ——
    # 用 rospy.Time 计时会让"探针窗口"在目标下发之前就结束（读数变成"车没动"）。
    t0 = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - t0 < args.duration:
        rate.sleep()

    # 采样覆盖：cmd 名义 50~80 Hz，出现 >0.5 s 断档说明采集中断过，结论不可信
    max_gap = None
    if len(pr.cmd_times) > 1:
        max_gap = max(b - a for a, b in zip(pr.cmd_times, pr.cmd_times[1:]))
    result = {'duration_s': args.duration, 'goal': [args.goal_x, args.goal_y],
              'pose_topic': args.pose_topic,
              'cmd_frames': pr.cmd_frames, 'speed_frames': pr.speed_frames,
              'max_speed_mps': round(pr.max_speed, 4),
              'bt_arrived': pr.bt_arrived,
              'max_cmd_gap_s': round(max_gap, 3) if max_gap is not None else None}
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
        initial = math.hypot(pr.first_est[0] - args.goal_x, pr.first_est[1] - args.goal_y)
        result['initial_dist_to_goal_m'] = round(initial, 3)
        checks = {
            'nonzero_speed': pr.speed_frames > 0,
            # 原判据写死 1 m，短程目标（<2 m）下永远不成立；改成"至少走完初始距离的一半"，
            # 语义等价于"确实在朝目标推进"，且不涉及安全参数。
            'travel_half_of_initial': result['est_travel_m'] >= 0.5 * max(initial, 1e-6),
            'arrived': dist <= args.tol,
            # ★BT 到达事件★：规划/执行链自己报了 reached 才算闭环走完；
            # 只有真值近、BT 没报，通常是"卡在目标附近但树没收敛"，不能算过。
            'bt_reached': pr.bt_arrived,
            # ★采样连续★：cmd 断档 >0.5 s 说明探针自身漏采，读数不可信
            'sampling_continuous': (result['max_cmd_gap_s'] is not None
                                    and result['max_cmd_gap_s'] <= 0.5),
        }
        if pr.gt is not None:
            gdx = pr.gt[0] - pr.first_gt[0]
            gdy = pr.gt[1] - pr.first_gt[1]
            result['gt_travel_m'] = round(math.hypot(gdx, gdy), 4)
            result['gt_dist_to_goal_m'] = round(
                math.hypot(pr.gt[0] - args.goal_x, pr.gt[1] - args.goal_y), 4)
            checks['est_gt_travel_agree'] = abs(result['est_travel_m'] - result['gt_travel_m']) <= 0.5
            # ★真实到达用真值判★：估计位姿说"到了"不算，必须 gt 到 goal 也在容差内。
            # BT 的 reached 用的是估计位姿，两者可能不一致（定位误差大时尤其）。
            checks['gt_arrived'] = result['gt_dist_to_goal_m'] <= args.arrive_tol
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
