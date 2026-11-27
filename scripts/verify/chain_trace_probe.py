#!/usr/bin/env python3
"""M2 目标链逐段取证：**先订阅、后发目标**，按同一 mission 看每一段有没有数据。

为什么需要它（2026-11-27）：`bt_goal_once` 的 accepted 只证明 BT 收了任务，
不证明它过了 CheckLocalization / RequestPath；而 RequestPath 的 goal 只发一次、
不 latch，事后 `rostopic echo` 可能整段漏采，于是"看起来没发"，其实是没接住。

链路（按实际 Graph 核过的话题名，不按默认猜）：
  /v8/bt/goal_task → /far/goal_task → /far/waypoint_tagged
  → /v8/3d/global/path_task → /v8/local/goal_task → /cmd_vel

用法：chain_trace_probe.py --x 6.5 --y 2.6 [--window 25] [--out out.json]
只读订阅 + 发一条目标；不发 cmd_vel、不改任何节点参数。
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from std_msgs.msg import String

CHAIN = [
    ('bt_goal_task', '/v8/bt/goal_task', String),
    ('far_goal_task', '/far/goal_task', String),
    ('waypoint_tagged', '/far/waypoint_tagged', String),
    ('global_path_task', '/v8/3d/global/path_task', String),
    ('local_goal_task', '/v8/local/goal_task', String),
    ('bt_status', '/v8/bt/status', String),
    ('global_status', '/v8/3d/global/status', String),
    ('terrain_cloud', '/terrain_cloud', None),
    ('cmd_vel', '/cmd_vel', Twist),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--window', type=float, default=25.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    rospy.init_node('chain_trace_probe', anonymous=True, disable_signals=True)
    rec = {name: {'n': 0, 'first': None, 'last': None, 'sample': None} for name, _, _ in CHAIN}

    def mk(name):
        def cb(msg):
            now = time.monotonic()
            r = rec[name]
            r['n'] += 1
            if r['first'] is None:
                r['first'] = now
                if hasattr(msg, 'data'):
                    r['sample'] = str(msg.data)[:200]
            r['last'] = now
        return cb

    for name, topic, typ in CHAIN:
        if typ is not None:
            rospy.Subscriber(topic, typ, mk(name), queue_size=20)

    # 先等订阅生效，再发目标——顺序反了就等于用"事后 echo"取证
    time.sleep(2.0)
    t0 = time.monotonic()
    pub = rospy.Publisher('/v8/bt/goal_task', String, queue_size=5)
    task = {'request_id': 'chain-%d' % int(t0), 'gen': 1,
            'x': a.x, 'y': a.y, 'z': 0.0, 'frame': 'map'}
    for _ in range(3):
        pub.publish(String(data=json.dumps(task)))
        time.sleep(0.2)
    sent_at = time.monotonic()

    while time.monotonic() - t0 < a.window and not rospy.is_shutdown():
        time.sleep(0.2)

    rows = []
    for name, topic, _ in CHAIN:
        r = rec[name]
        rows.append({'segment': name, 'topic': topic, 'msgs': r['n'],
                     'first_after_goal_s': (None if r['first'] is None
                                            else round(r['first'] - sent_at, 3)),
                     'sample': r['sample']})
    first_dead = next((x['segment'] for x in rows if x['msgs'] == 0), None)
    result = {'goal': [a.x, a.y], 'window_s': a.window, 'rows': rows,
              'first_silent_segment': first_dead,
              'verdict': 'CHAIN_OK' if first_dead is None else 'BREAK_AT_%s' % first_dead}
    print(''.join('%-18s %-28s msgs=%-4d first=%s\n'
                  % (x['segment'], x['topic'], x['msgs'], x['first_after_goal_s']) for x in rows))
    print('VERDICT ' + result['verdict'])
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(json.dumps(result, ensure_ascii=False, indent=1))
    return 0 if first_dead is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
