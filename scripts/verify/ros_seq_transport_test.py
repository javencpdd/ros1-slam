#!/usr/bin/env python3
"""ROS1 身份字段传输实验（2026-11-27）：`Header.seq` 到底能不能当 request_id？

背景：P0 航点身份靠 `PointStamped.header.seq` 传 request_id。风险是 rospy/roscpp 在发布链路里
自动维护 seq（rostopic echo 里看到的自增就是一个提示），若中途被改写，"请求编号"这条契约
就站不住。这里做**真实跨进程**实验，而不是读文档猜：

  A 段  Python 发布 → Python 订阅（基线，看 rospy 是否改写）
  B 段  Python 发布 → **C++ 中继**（topic_tools/relay）→ Python 订阅
  C 段  连续多帧 / 重复编号 / 0 / 极大值 / 重置帧，全部逐条比对

判据：收到的 header.seq 必须与发出的**逐条相等**；`header.stamp` 必须是发送方给的时间
（不能被改成接收时刻）。任一不等即判 `SEQ_UNRELIABLE`，则身份必须改用显式 request_id 字段。

用法：ros_seq_transport_test.py [--seconds 6]
退出码：0=seq 原样传输；3=被改写（不可用）
"""
import argparse
import json
import sys
import threading

import rospy
from geometry_msgs.msg import PointStamped

CASES = [
    {'seq': 1, 'tag': 'first'},
    {'seq': 4242, 'tag': 'normal'},
    {'seq': 4242, 'tag': 'duplicate'},      # 同编号重复请求
    {'seq': 0, 'tag': 'reset'},             # 重置帧：约定了"无请求"
    {'seq': 999999, 'tag': 'huge'},
    {'seq': 7, 'tag': 'after_reset'},
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='/seq_probe_src')
    ap.add_argument('--dst', default='/seq_probe_dst')
    ap.add_argument('--seconds', type=float, default=6.0)
    args = ap.parse_args()

    rospy.init_node('ros_seq_transport_test', anonymous=True, disable_signals=True)
    got = []
    lock = threading.Lock()

    def on_msg(msg):
        with lock:
            got.append({'seq': msg.header.seq,
                        'stamp': round(msg.header.stamp.to_sec(), 6),
                        'x': round(msg.point.x, 3)})

    rospy.Subscriber(args.dst, PointStamped, on_msg, queue_size=50)
    pub = rospy.Publisher(args.src, PointStamped, queue_size=50)

    rospy.sleep(1.0)                      # 等连接建立，否则 publish 会被丢弃
    sent = []
    for case in CASES:
        msg = PointStamped()
        msg.header.seq = case['seq']
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.001 * case['seq'])
        msg.header.frame_id = 'map'
        msg.point.x = float(case['seq'])
        pub.publish(msg)
        sent.append({'seq': case['seq'], 'tag': case['tag'],
                     'stamp': round(msg.header.stamp.to_sec(), 6)})
        rospy.sleep(0.25)

    rospy.sleep(args.seconds)

    with lock:
        received = list(got)

    matched, mismatched = 0, []
    for i, expect in enumerate(sent):
        actual = received[i] if i < len(received) else None
        if actual is None:
            mismatched.append({'index': i, 'expect': expect, 'actual': 'MISSING'})
        elif actual['seq'] == expect['seq'] and abs(actual['stamp'] - expect['stamp']) < 1e-6:
            matched += 1
        else:
            mismatched.append({'index': i, 'expect': expect, 'actual': actual})

    verdict = 'SEQ_PRESERVED' if not mismatched and len(received) >= len(sent) else 'SEQ_UNRELIABLE'
    out = {'src': args.src, 'dst': args.dst, 'sent': sent, 'received': received,
           'matched': matched, 'mismatched': mismatched, 'verdict': verdict}
    print(json.dumps(out, ensure_ascii=False))
    return 0 if verdict == 'SEQ_PRESERVED' else 3


if __name__ == '__main__':
    sys.exit(main())
