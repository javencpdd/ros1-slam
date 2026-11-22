#!/usr/bin/env python3
"""B2 far 分支验收：四项判据 ＋ 取消三项 ＋ 取消后重发。

关键口径
  1) FAR 冷启动约 30 s 才建好可见图，此前 BT 会连续收到 E_FAR_WARMUP。首目标必然落空，
     所以这里按 BT 的重试语义重发 goal，并把尝试次数记进 ACCEPT_WARM_ATTEMPTS（这是真实行为，不是缺陷掩盖）。
  2) /global_path 是 latched 的，订阅端会收到一次历史重放，因此「产出恢复」用 **唯一时间戳数** 判定，
     不用消息条数。
"""
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

GOAL_Z = 0.11
CANCEL_SEQ = 4343
STAMPS = set()
PATH_SIZES = []
STATUS = []
CANCEL_MSGS = []
ARRIVED = []
LAST = {'z': None}


def main():
    rospy.init_node('far_accept', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    cancel_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)

    def on_path(m):
        STAMPS.add(m.header.stamp.to_nsec())
        PATH_SIZES.append(len(m.poses))
        if m.poses:
            LAST['z'] = m.poses[-1].pose.position.z

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/global_path/status', String, lambda m: STATUS.append(m.data), queue_size=100)
    rospy.Subscriber('/v8/local/cancel', String, lambda m: CANCEL_MSGS.append(m.data), queue_size=100)
    rospy.Subscriber('/v8/bt/arrived', Bool, lambda m: ARRIVED.append(bool(m.data)), queue_size=10)

    def send_goal(tag):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y, g.pose.position.z = 2.0, 0.0, GOAL_Z
        g.pose.orientation.w = 1.0
        goal_pub.publish(g)
        print('ACCEPT_GOAL_SENT tag=%s z=%.3f' % (tag, GOAL_Z))

    time.sleep(3.0)
    # ---- 门 0：预热重试直到出现第一条路径
    attempts = 0
    while attempts < 8 and not STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts)
        end = time.time() + 12.0
        while time.time() < end and not STAMPS:
            time.sleep(0.3)
    print('ACCEPT_WARM_ATTEMPTS=%d' % attempts)
    print('ACCEPT_WARM_PATHS=%d' % len(PATH_SIZES))
    print('ACCEPT_POSES_UNIQ=%s' % sorted(set(PATH_SIZES)))
    print('ACCEPT_LAST_POSE_Z=%s' % LAST['z'])
    print('ACCEPT_ARRIVED_MSGS=%d' % len(ARRIVED))
    print('ACCEPT_WARMUP_REFUSED=%d' % sum(1 for s in STATUS if 'E_FAR_WARMUP' in s))

    # ---- 门 1：取消三项
    stamps_before = len(STAMPS)
    CANCEL_MSGS.clear()
    cancel_pub.publish(String(data='%d:all:accept_test' % CANCEL_SEQ))
    time.sleep(1.0)
    print('ACCEPT_CANCEL_SEEN=%s' % any('E_FAR_CANCEL_RECEIVED' in s and str(CANCEL_SEQ) in s
                                        for s in STATUS))
    time.sleep(6.0)
    print('ACCEPT_CANCEL_STOPPED=%s' % (len(STAMPS) - stamps_before <= 1))
    print('ACCEPT_CANCEL_NO_ECHO=%s' % (sum(1 for m in CANCEL_MSGS
                                            if m.split(':')[0] == str(CANCEL_SEQ)) == 1))

    # ---- 门 2：取消后重发 goal（用唯一时间戳数判恢复）
    stamps_before = len(STAMPS)
    attempts = 0
    while attempts < 4 and len(STAMPS) <= stamps_before:
        attempts += 1
        send_goal('regoal%d' % attempts)
        end = time.time() + 12.0
        while time.time() < end and len(STAMPS) <= stamps_before:
            time.sleep(0.3)
    print('ACCEPT_REGOAL_ATTEMPTS=%d' % attempts)
    print('ACCEPT_REGOAL_NEW_STAMPS=%d' % (len(STAMPS) - stamps_before))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
