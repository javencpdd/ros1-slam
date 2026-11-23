#!/usr/bin/env python3
"""读取一次 /v8/bt/status 并结构化打印，用于定位行为树卡在哪个叶节点。

为什么单独写：`/v8/bt/status` 是 std_msgs/String，内容是转义过的 JSON，
用 rostopic echo 加 cut 会被截断、用 sed 反转义又会踩编码。直接 rospy 取原始字段最稳。

判据用法：不能只看 root=RUNNING，要看 nodes[] 里 wait_path / follow_path 各自的状态。
"""
import json
import sys

import rospy
from std_msgs.msg import String

KEYS = ('root', 'event', 'outcome', 'outcome_seq', 'path_state', 'path_points',
        'planner_mode', 'goal_seq', 'current_seq', 'goals_total', 'goal_index',
        'queue_depth', 'leg_active', 'arrival_signal', 'localization_ok',
        'localization_check', 'cancel_requested', 'attempts')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else ''
    rospy.init_node('bt_status_probe', anonymous=True)
    try:
        msg = rospy.wait_for_message('/v8/bt/status', String, timeout=10.0)
    except rospy.ROSException:
        print('NO_STATUS  # 10 s 内没收到 /v8/bt/status')
        return 2

    d = json.loads(msg.data)
    for k in KEYS:
        if k in d:
            print('%-18s %s' % (k, d[k]))
    print('--- nodes ---')
    for n in d.get('nodes', []):
        if isinstance(n, dict):
            print('  %-34s %s' % (n.get('name', '?'), n.get('status', '?')))
        else:
            print('  %s' % (n,))
    if out:
        open(out, 'w').write(msg.data + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
