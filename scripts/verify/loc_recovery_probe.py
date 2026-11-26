#!/usr/bin/env python3
"""定位恢复探针：把定位数据闸门恢复后，断言车**不自行复动**。

为什么必须有它：`motion_interrupt_probe --mode stop` 只验"断流后会停"，不验"恢复后不会自己走"。
后者是真实风险——定位恢复瞬间位姿可能跳变，局部规划器以为"离目标还远"就会重新起步，
这在验收里属于"定位失效未安全收敛"。

判据：
  - `/hdl_odom` 恢复后有消息（定位确实回来了）；
  - 观察窗内 `/sim/odom_gt`（真值）**连续轨迹**的最大偏离 ≤ --tol-m（默认 0.05 m）；
  - 采样必须连续（相邻样本间隔 ≤0.5 s，且速度样本数 ≥10），**字段缺失不得按 PASS**。

★采样必须在"恢复闸门"之前开始★：`case_loc` 先起本探针、再重启 relay，这样恢复瞬间的
速度与轨迹都被覆盖到；如果先恢复再采样，"恢复后立刻窜出去"的那一段会被漏掉。

用法：
  python3 scripts/verify/loc_recovery_probe.py --seconds 6 --out <ev>/loc_recovery.json
"""
import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
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
    state = {'est_n': 0, 'gt_first': None, 'gt_last': None,
             'gt_times': [], 'gt_pts': [], 'cmd_times': [], 'cmd_v': []}

    def on_est(_m):
        state['est_n'] += 1

    def on_cmd(m):
        state['cmd_times'].append(time.monotonic())
        state['cmd_v'].append(math.hypot(m.linear.x, m.linear.y))

    def on_gt(m):
        p = m.pose.pose.position
        if state['gt_first'] is None:
            state['gt_first'] = (p.x, p.y)
        state['gt_last'] = (p.x, p.y)
        state['gt_times'].append(time.monotonic())
        state['gt_pts'].append((p.x, p.y))

    # ★按类型只订一次★：同一 topic 建两种类型订阅，另一条永远空转且掩盖配错
    resolved = rospy.resolve_name(args.pose_topic)
    published = dict(rospy.get_published_topics())
    mtype = published.get(resolved) or ''
    if 'PoseStamped' in mtype:
        rospy.Subscriber(args.pose_topic, PoseStamped, lambda _m: on_est(None), queue_size=20)
    elif 'Odometry' in mtype:
        rospy.Subscriber(args.pose_topic, Odometry, on_est, queue_size=20)
    else:
        for _ in range(20):
            rospy.sleep(0.25)
            mtype = (dict(rospy.get_published_topics()).get(resolved) or '')
            if mtype:
                break
        if 'PoseStamped' in mtype:
            rospy.Subscriber(args.pose_topic, PoseStamped, lambda _m: on_est(None), queue_size=20)
        elif 'Odometry' in mtype:
            rospy.Subscriber(args.pose_topic, Odometry, on_est, queue_size=20)
        else:
            rospy.logerr('位姿话题 %s 类型未知，本探针不订阅', args.pose_topic)
    rospy.Subscriber(args.gt_topic, Odometry, on_gt, queue_size=20)
    from geometry_msgs.msg import Twist
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    t0 = time.monotonic()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() - t0 < args.seconds:
        rate.sleep()

    # 连续轨迹：相对**首样本**的最大偏离（不是只比首尾，避免中途窜出去又回来看不到）
    moved = None
    if state['gt_pts']:
        x0, y0 = state['gt_pts'][0]
        moved = max(math.hypot(x - x0, y - y0) for x, y in state['gt_pts'])
    gt_gap = None
    if len(state['gt_times']) > 1:
        gt_gap = max(b - a for a, b in zip(state['gt_times'], state['gt_times'][1:]))
    cmd_gap = None
    if len(state['cmd_times']) > 1:
        cmd_gap = max(b - a for a, b in zip(state['cmd_times'], state['cmd_times'][1:]))
    max_v = max(state['cmd_v']) if state['cmd_v'] else None
    checks = {
        'pose_resumed': state['est_n'] > 0,
        'no_remotion': (moved is not None and moved <= args.tol_m),
        # 覆盖度：真值与速度都要有连续样本；缺失即视为未验，不得按 PASS
        'sampling_coverage': (gt_gap is not None and gt_gap <= 0.5
                              and cmd_gap is not None and cmd_gap <= 0.5
                              and len(state['cmd_v']) >= 10),
    }
    res = {'seconds': args.seconds, 'pose_msgs': state['est_n'],
           'gt_samples': len(state['gt_pts']), 'cmd_samples': len(state['cmd_v']),
           'moved_m': round(moved, 4) if moved is not None else None,
           'max_speed_mps': round(max_v, 4) if max_v is not None else None,
           'max_gt_gap_s': round(gt_gap, 3) if gt_gap is not None else None,
           'max_cmd_gap_s': round(cmd_gap, 3) if cmd_gap is not None else None,
           'tol_m': args.tol_m, 'checks': checks,
           'pass': all(checks.values())}
    text = json.dumps(res, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')
    return 0 if res['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
