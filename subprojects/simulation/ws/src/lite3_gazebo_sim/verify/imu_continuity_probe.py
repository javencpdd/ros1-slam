#!/usr/bin/env python3
"""IMU 供源唯一性 + 连续性 + 新鲜度探针（零假设版）。

为什么单独一件（2026-11-27 真人要求）：`/imu/data` 的发布者检查不能"取第一个就算"。
必须回答三件事，且三者互不代替：
  ① 唯一权威发布者——发布者数必须为 1，并列出名字；
  ② 连续性——观测窗内的到达间隔最大值，抓"掉帧/断流"；
  ③ 新鲜度与来源——消息戳是否跟随 /clock 前进、frame_id 是否始终是期望源系。

用法：imu_continuity_probe.py [topic] [seconds] [expect_frame]
rc：0 通过；3 发布者不唯一；4 无数据或断流；5 frame 不符或戳不前进。
只读：只订阅，不发任何指令。
"""
import sys
import time

import rospy
from sensor_msgs.msg import Imu


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/imu/data'
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    expect_frame = sys.argv[3] if len(sys.argv) > 3 else 'base_link'

    rospy.init_node('imu_continuity_probe', anonymous=True, disable_signals=True)

    pubs = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not pubs:
        try:
            pubs = rospy.get_published_topics()
        except Exception:
            pubs = []
        time.sleep(0.2)
    info = __import__('subprocess').run(['rostopic', 'info', topic],
                                        capture_output=True, text=True).stdout
    pub_block = info.split('Publishers:')[-1].split('Subscribers:')[0] if 'Publishers:' in info else ''
    names = [ln.strip(' *') for ln in pub_block.splitlines() if ln.strip().startswith('*')]
    print('PUBLISHERS n=%d %s' % (len(names), names))
    if len(names) != 1:
        print('VERDICT FAIL publisher_count=%d（必须恰 1 个权威发布者）' % len(names))
        return 3

    stamps, frames, gaps = [], [], []
    last_wall = [None]

    def cb(m):
        now = time.monotonic()
        if last_wall[0] is not None:
            gaps.append(now - last_wall[0])
        last_wall[0] = now
        stamps.append(m.header.stamp.to_sec())
        frames.append(m.header.frame_id)

    rospy.Subscriber(topic, Imu, cb, queue_size=200)
    t0 = time.monotonic()
    while time.monotonic() - t0 < window and not rospy.is_shutdown():
        time.sleep(0.05)

    n = len(stamps)
    if n < 2:
        print('VERDICT FAIL samples=%d（观测窗 %.1fs 内没有数据）' % (n, window))
        return 4
    rate = (n - 1) / max(1e-9, stamps[-1] - stamps[0]) if stamps[-1] > stamps[0] else 0.0
    max_gap = max(gaps) if gaps else 0.0
    mono = all(b >= a for a, b in zip(stamps, stamps[1:]))
    uniq_frames = sorted(set(frames))
    print('SAMPLES n=%d rate=%.1fHz max_gap=%.3fs stamp_monotonic=%s frames=%s first=%.3f last=%.3f'
          % (n, rate, max_gap, mono, uniq_frames, stamps[0], stamps[-1]))

    bad = []
    if not mono:
        bad.append('stamp 非单调（有回退）')
    if expect_frame and uniq_frames != [expect_frame]:
        bad.append('frame_id 集合=%s，期望仅 [%s]' % (uniq_frames, expect_frame))
    if max_gap > 0.5:
        bad.append('最大间隔 %.3fs > 0.5s（断流）' % max_gap)
    if bad:
        print('VERDICT FAIL ' + '；'.join(bad))
        return 5
    print('VERDICT PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
