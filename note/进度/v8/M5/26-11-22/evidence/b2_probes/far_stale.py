#!/usr/bin/env python3
"""B2 航点超时门：干净的 far 任务跑到有路径后杀掉 far_planner，
期望适配器报 E_FAR_WAYPOINT_STALE 并自报一次取消（seq ≥ 900000000）。

与取消门分开跑：取消会把 state.halted 置真，stale() 在 halted 时按规定不报，
两件事混在一次运行里会互相遮住。
"""
import subprocess
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

GOAL_Z = 0.11
STAMPS = set()
STATUS = []
CANCEL_MSGS = []


def main():
    rospy.init_node('far_stale', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/global_path', Path, lambda m: STAMPS.add(m.header.stamp.to_nsec()),
                     queue_size=10)
    rospy.Subscriber('/global_path/status', String, lambda m: STATUS.append(m.data), queue_size=100)
    rospy.Subscriber('/v8/local/cancel', String, lambda m: CANCEL_MSGS.append(m.data), queue_size=100)

    def send_goal(tag):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y, g.pose.position.z = 2.0, 0.0, GOAL_Z
        g.pose.orientation.w = 1.0
        goal_pub.publish(g)
        print('STALE_GOAL_SENT tag=%s' % tag)

    time.sleep(3.0)
    attempts = 0
    while attempts < 8 and not STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts)
        end = time.time() + 12.0
        while time.time() < end and not STAMPS:
            time.sleep(0.3)
    print('STALE_WARM_ATTEMPTS=%d' % attempts)
    print('STALE_PATHS=%d' % len(STAMPS))
    if not STAMPS:
        print('STALE_ABORT=no_path')
        return 0

    STATUS.clear()
    CANCEL_MSGS.clear()
    rc = subprocess.call(['rosnode', 'kill', '/far_planner'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print('STALE_KILL_RC=%d' % rc)
    # 超时按 ROS 时间计，而仿真里 /clock 与墙钟不同步，所以这里按墙钟轮询到 90 s，
    # 同时把 ROS 时间推进量打出来，用来判断到底是被时钟拖住还是被 halted 抑制。
    t_ros0, t_wall0 = rospy.get_time(), time.time()
    end = time.time() + 90.0
    while time.time() < end and not any('E_FAR_WAYPOINT_STALE' in s for s in STATUS):
        time.sleep(1.0)
    t_ros1, t_wall1 = rospy.get_time(), time.time()
    print('STALE_ROS_ADVANCE=%.3f STALE_WALL_ADVANCE=%.3f' % (t_ros1 - t_ros0, t_wall1 - t_wall0))
    print('STALE_USE_SIM_TIME=%s' % rospy.get_param('/use_sim_time', None))
    print('STALE_STATUS_MSGS=%d' % len(STATUS))
    print('STALE_REPORTED=%s' % any('E_FAR_WAYPOINT_STALE' in s for s in STATUS))
    print('STALE_SELF_CANCEL=%s' % any(m.split(':')[0].isdigit() and int(m.split(':')[0]) >= 900000000
                                       for m in CANCEL_MSGS))
    print('STALE_STATUS_TAIL=%s' % STATUS[-3:])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
