#!/usr/bin/env python3
"""量测话题的消息间隔分布，用于给"新鲜度超时"定依据。

判据不能拍脑袋：要先知道传感器实际多久来一帧、最长空档多少，
才能判断某个超时值是否会误停（假阳性）或太宽（失效后仍放行）。

用法：
  topic_interval.py --topic /hdl_odom --duration 20 --timeout-ms 300
输出：样本数、均值、p50/p95/p99、最大间隔，以及"超过给定超时的空档次数"。
"""
import argparse
import json
import sys
import time

import rospy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/hdl_odom')
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--timeout-ms', type=float, default=300.0,
                    help='候选超时；统计有多少空档会超过它（即会误停的次数）')
    ap.add_argument('--out', default='')
    ap.add_argument('--type', default='nav_msgs/Odometry',
                    help='消息类型；AnyMsg 对具体类型不匹配（实测收 0 条）')
    args = ap.parse_args()

    stamps = []

    def on_msg(_msg):
        # 用墙钟记录到达时刻：我们要的是"发布节奏"，与仿真时间无关
        stamps.append(time.monotonic())

    rospy.init_node('topic_interval', anonymous=True)
    parts = args.type.split('/')
    module = __import__('%s.msg' % parts[0], fromlist=[parts[1]])
    msg_type = getattr(module, parts[1])
    rospy.Subscriber(args.topic, msg_type, on_msg, queue_size=200)
    t0 = time.monotonic()
    try:
        while not rospy.is_shutdown() and time.monotonic() - t0 < args.duration:
            time.sleep(0.05)
    except rospy.ROSInterruptException:
        pass

    gaps = [round((stamps[i + 1] - stamps[i]) * 1000.0, 1) for i in range(len(stamps) - 1)]
    result = {'topic': args.topic, 'duration_s': args.duration, 'samples': len(stamps)}
    if gaps:
        ordered = sorted(gaps)
        def pct(p):
            idx = min(len(ordered) - 1, int(round(p / 100.0 * (len(ordered) - 1))))
            return ordered[idx]
        result.update({
            'gap_mean_ms': round(sum(gaps) / len(gaps), 1),
            'gap_p50_ms': pct(50), 'gap_p95_ms': pct(95), 'gap_p99_ms': pct(99),
            'gap_max_ms': max(gaps),
            'implicit_rate_hz': round(1000.0 / (sum(gaps) / len(gaps)), 2) if sum(gaps) else None,
            'timeout_candidate_ms': args.timeout_ms,
            'gaps_over_timeout': sum(1 for g in gaps if g > args.timeout_ms),
            'timeout_safe': all(g <= args.timeout_ms for g in gaps),
        })
    else:
        result['error'] = 'no_gaps'
    print(json.dumps(result, ensure_ascii=False, indent=1))
    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
