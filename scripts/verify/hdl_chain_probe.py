#!/usr/bin/env python3
"""HDL 断流逐级定位（2026-11-26 裁决①）：按链路顺序找**第一个没有数据的环节**。

链路：`/rslidar_points` → `/velodyne_points` → `/hdl_odom_raw` → `/hdl_odom`
      → `/v8/local/pose` → `/odom_world`

每一级只回答三个问题（不做任何"重启"动作）：
  · 有没有 publisher / subscriber（`rosnode info` 那层的连接面）
  · 能否在超时内收到**一条**消息（latched 话题也能收到，不靠 hz）
  · 若是流式话题，附带实测频率

判定：按顺序第一个"无消息"的环节即断点，输出 `FIRST_EMPTY=<topic>` 并给出该环节的
publisher/subscriber 清单（这决定了是"上游没发"还是"下游没接"）。

用法：
  hdl_chain_probe.py [--timeout 4] [--json /tmp/chain.json]
"""
import argparse
import json
import sys

import rospy

CHAIN = ['/rslidar_points', '/velodyne_points', '/hdl_odom_raw', '/hdl_odom',
         '/v8/local/pose', '/odom_world']


def topic_type(topic):
    try:
        types = dict(rospy.get_published_topics())
        return types.get(topic)
    except Exception:
        return None


def probe(topic, timeout):
    """返回 dict：registered / got_message / rate_hz / publish_master / notes。"""
    out = {'topic': topic, 'registered': False, 'got_message': False, 'rate_hz': None}
    try:
        pubs, subs = rospy.get_published_topics(), None
        out['registered'] = topic in dict(pubs)
    except Exception as error:
        out['notes'] = str(error)[:80]
    t = topic_type(topic)
    out['type'] = t
    if not t:
        out['notes'] = '无发布者（话题未注册）'
        return out
    cls = rospy.get_message_class(t) if hasattr(rospy, 'get_message_class') else None
    if cls is None:
        import importlib
        pkg, name = t.split('/')
        cls = getattr(importlib.import_module(pkg + '.msg'), name)
    try:
        rospy.wait_for_message(topic, cls, timeout=timeout)
        out['got_message'] = True
    except Exception as error:
        out['notes'] = ('等待超时' if 'timeout' in str(error).lower() else str(error)[:80])
        return out
    # 频率只对"会重复发"的话题有意义；latched 话题收到一条就够，不参与判定
    try:
        import subprocess
        res = subprocess.run(['rostopic', 'hz', topic, '-w', '20'],
                             capture_output=True, text=True, timeout=timeout + 2)
        for line in (res.stdout or '').splitlines():
            if 'average rate' in line:
                out['rate_hz'] = float(line.split(':')[-1].strip().split()[0])
    except Exception:
        pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=4.0)
    ap.add_argument('--json', default='')
    args = ap.parse_args()
    rospy.init_node('hdl_chain_probe', anonymous=True, disable_signals=True)

    rows, first_empty = [], None
    for topic in CHAIN:
        row = probe(topic, args.timeout)
        rows.append(row)
        mark = 'OK ' if row['got_message'] else '—— '
        print('%s%-22s type=%-34s rate=%s %s'
              % (mark, topic, row.get('type'), row.get('rate_hz'), row.get('notes', '')))
        if not row['got_message'] and first_empty is None:
            first_empty = topic

    if first_empty:
        print('FIRST_EMPTY=%s' % first_empty)
    else:
        print('FIRST_EMPTY=-')
    if args.json:
        with open(args.json, 'w') as fh:
            fh.write(json.dumps({'rows': rows, 'first_empty': first_empty},
                                ensure_ascii=False) + '\n')
    return 1 if first_empty else 0


if __name__ == '__main__':
    sys.exit(main())
