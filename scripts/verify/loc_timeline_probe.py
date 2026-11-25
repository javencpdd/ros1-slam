#!/usr/bin/env python3
"""故障后"重新武装"失败时的完整时间线记录器（真人裁决 2026-11-26 第 7 条）。

为什么需要：只判"cmd_vel 非零"会漏掉真正的断点。本探针按 10 Hz 同时记录
  local 状态（stop/reason/pose_clock/generation 相关字段）、BT 状态、path_goal 状态、
  goal 与 goal_meta 的到达计数、mux 状态、cmd_vel 速度，
落成 TSV，一眼能看出"谁在拒、拒在哪一环"。

用法：
  python3 scripts/verify/loc_timeline_probe.py --seconds 40 --out /tmp/timeline.tsv
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=40.0)
    ap.add_argument('--out', default='/tmp/loc_timeline.tsv')
    args = ap.parse_args()

    rospy.init_node('loc_timeline_probe', anonymous=True)
    s = {'local': '', 'bt': '', 'pg': '', 'mux': '', 'v': 0.0,
         'goal_n': 0, 'meta_n': 0, 'meta_last': ''}

    def sub(topic, key, msg_type=String):
        def cb(m):
            if msg_type is String:
                s[key] = (m.data or '')[:400]
            else:
                s['v'] = (m.linear.x ** 2 + m.linear.y ** 2) ** 0.5
        rospy.Subscriber(topic, msg_type, cb, queue_size=20)

    sub('/v8/local/status', 'local')
    sub('/v8/bt/status', 'bt')
    sub('/v8/3d/path_goal/status', 'pg')
    sub('/v8/3d/mux/status', 'mux')
    sub('/cmd_vel', 'cmd', Twist)

    def on_goal(_m):
        s['goal_n'] += 1

    def on_meta(m):
        s['meta_n'] += 1
        s['meta_last'] = (m.data or '')[:160]

    rospy.Subscriber('/v8/local/goal', __import__('geometry_msgs.msg', fromlist=['PoseStamped']).PoseStamped,
                     on_goal, queue_size=20)
    rospy.Subscriber('/v8/local/goal_meta', String, on_meta, queue_size=20)

    def pick(text, *keys):
        try:
            value = json.loads(text)
        except Exception:
            return ''
        out = []
        for k in keys:
            if k in value:
                out.append('%s=%s' % (k, value[k]))
        return ' '.join(out)

    t0 = time.monotonic()
    with open(args.out, 'w') as fh:
        fh.write('t\tv\tgoals\tmetas\tlocal\ttask\tpg\tmux\tmeta_last\n')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() - t0 < args.seconds:
            fh.write('%.2f\t%.3f\t%d\t%d\t%s\t%s\t%s\t%s\t%s\n' % (
                time.monotonic() - t0, s['v'], s['goal_n'], s['meta_n'],
                pick(s['local'], 'stop', 'reason', 'pose_error', 'last_error'),
                pick(s['bt'], 'event', 'queue_depth', 'goal_seq', 'current_seq', 'outcome', 'root'),
                pick(s['pg'], 'ok', 'reason', 'index', 'poses', 'generation'),
                pick(s['mux'], 'reason', 'velocity'),
                s['meta_last']))
            fh.flush()
            rate.sleep()
    print('timeline=%s' % args.out)


if __name__ == '__main__':
    main()
