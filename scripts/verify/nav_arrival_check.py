#!/usr/bin/env python3
"""导航到达核对：按 POSE_TOPIC 的**实际类型**取算法位姿，与真值一起判到达。

为什么要独立成文件：原来这段是 `p4_regression.sh` 里的内联 heredoc，既硬编码 `Odometry`
（M2 的 `/v8/local/pose` 是 PoseStamped，必然超时），又读不到未导出的环境变量。
"""
import json
import math
import os
import sys

import rospy
from nav_msgs.msg import Odometry


def main():
    gx, gy, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    tol = float(os.environ.get('NAV_GT_TOL_M', '0.15'))
    rospy.init_node('nav_arrival_check', anonymous=True)
    topic = os.environ.get('POSE_TOPIC', '/hdl_odom')
    mtype = dict(rospy.get_published_topics()).get(rospy.resolve_name(topic), '')
    if 'PoseStamped' in mtype:
        from geometry_msgs.msg import PoseStamped
        e = rospy.wait_for_message(topic, PoseStamped, timeout=10).pose.position
    elif 'Odometry' in mtype:
        e = rospy.wait_for_message(topic, Odometry, timeout=10).pose.pose.position
    else:
        raise SystemExit('位姿话题 %s 类型未知（未发布？）' % topic)
    g = rospy.wait_for_message('/sim/odom_gt', Odometry, timeout=10).pose.pose.position
    res = {'pose_topic': topic, 'pose_type': mtype,
           'est_to_goal_m': round(math.hypot(e.x - gx, e.y - gy), 3),
           'gt_to_goal_m': round(math.hypot(g.x - gx, g.y - gy), 3),
           'est_gt_gap_m': round(math.hypot(e.x - g.x, e.y - g.y), 3),
           'verdict': 'PASS' if math.hypot(g.x - gx, g.y - gy) <= tol else 'FAIL'}
    text = json.dumps(res, ensure_ascii=False)
    print(text)
    open(out, 'w').write(text + '\n')
    return 0 if res['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
