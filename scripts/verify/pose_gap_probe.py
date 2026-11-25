#!/usr/bin/env python3
"""位姿断档探针：记录算法位姿（默认 /v8/local/pose）与上游 LIO（/hdl_odom）的到达间隔，
用来回答"局部规划器为什么在某一刻报 E_POSE_MISSING"。

为什么需要它：`/v8/local/status` 里的 `E_LOCAL_NOT_READY` 是 `E_POSE_MISSING` 的映射
（`local_planner/ros_node_signals.py:109`），而同一份 status 里 `pose_clock.ok` 又可能是 true ⇒
只有**到达间隔**能区分"持续断"与"瞬时缺帧"。`rostopic hz` 只给均值，看不到单次断档。

用法：
  python3 scripts/verify/pose_gap_probe.py --duration 30 --out /tmp/gaps.json
输出：{"pose_topic": ..., "n": ..., "max_gap_s": ..., "gaps": [{"at_wall": .., "gap_s": ..}, ...]}
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose-topic', default='/v8/local/pose')
    ap.add_argument('--lio-topic', default='/hdl_odom')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--gap-threshold', type=float, default=0.35,
                    help='只记录超过该阈值的间隔（秒）')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('pose_gap_probe', anonymous=True)
    state = {'pose': [], 'lio': []}

    def on_pose(msg):
        state['pose'].append((time.monotonic(), msg.header.stamp.to_sec()))

    def on_lio(msg):
        state['lio'].append((time.monotonic(), msg.header.stamp.to_sec()))

    rospy.Subscriber(args.pose_topic, PoseStamped, on_pose, queue_size=50)
    rospy.Subscriber(args.lio_topic, Odometry, on_lio, queue_size=50)

    t0 = time.monotonic()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() - t0 < args.duration:
        rate.sleep()

    def summarize(series, label):
        if len(series) < 2:
            return {'topic': label, 'n': len(series), 'max_gap_s': None, 'gaps': []}
        gaps = []
        for (w0, _), (w1, _) in zip(series, series[1:]):
            g = w1 - w0
            if g > args.gap_threshold:
                gaps.append({'at_wall': round(w1 - t0, 3), 'gap_s': round(g, 3)})
        span = series[-1][0] - series[0][0]
        return {'topic': label, 'n': len(series),
                'rate_hz': round((len(series) - 1) / span, 2) if span > 0 else None,
                'max_gap_s': round(max(w1 - w0 for (w0, _), (w1, _) in zip(series, series[1:])), 3),
                'gaps': gaps[:40]}

    res = {'pose': summarize(state['pose'], args.pose_topic),
           'lio': summarize(state['lio'], args.lio_topic)}
    text = json.dumps(res, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')


if __name__ == '__main__':
    main()
