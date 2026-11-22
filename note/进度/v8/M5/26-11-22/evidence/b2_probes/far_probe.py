#!/usr/bin/env python3
"""B2 far 分支探针：起 goal、收 /global_path 与 /v8/local/goal，把读数打成机器可读行。

放在 .git 下（不入仓），容器内路径 /opt/ros1-slam/.git/far_probe.py。
"""
import sys
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

GOT = {'paths': [], 'status': [], 'local_goal': None, 'waypoints': [], 'arrived': 0}


def main():
    rospy.init_node('far_probe', anonymous=True)
    rospy.Subscriber('/global_path', Path, lambda m: GOT['paths'].append(len(m.poses)))
    rospy.Subscriber('/global_path/status', String, lambda m: GOT['status'].append(m.data))
    rospy.Subscriber('/v8/local/goal', PoseStamped, lambda m: GOT.__setitem__(
        'local_goal', (m.pose.position.x, m.pose.position.y, m.pose.position.z)))
    rospy.Subscriber('/way_point', PointStamped, lambda m: GOT['waypoints'].append(
        (m.point.x, m.point.y, m.point.z)))

    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    time.sleep(2.0)                                  # 等连接建立
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y = 2.0, 0.0
    goal.pose.position.z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.11
    goal.pose.orientation.w = 1.0
    for _ in range(3):
        goal_pub.publish(goal)
        time.sleep(0.5)

    deadline = time.time() + float(sys.argv[2]) if len(sys.argv) > 2 else time.time() + 60.0
    while time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.5)

    print('PROBE_PATHS_TOTAL=%d' % len(GOT['paths']))
    print('PROBE_POSES_UNIQ=%s' % sorted(set(GOT['paths'])))
    print('PROBE_WAYPOINTS=%d' % len(GOT['waypoints']))
    print('PROBE_LAST_WAYPOINT=%s' % (GOT['waypoints'][-1] if GOT['waypoints'] else None))
    print('PROBE_LOCAL_GOAL=%s' % (GOT['local_goal'],))
    print('PROBE_STATUS_TAIL=%s' % (GOT['status'][-4:],))
    print('PROBE_ERR_STATUS=%s' % sorted({s for s in GOT['status'] if '"ok": false' in s})[:6])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
