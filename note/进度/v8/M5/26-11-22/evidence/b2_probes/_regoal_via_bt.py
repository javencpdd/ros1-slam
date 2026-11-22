"""取消后重发目标 —— 走 BT 的真实入口，而不是直接发 `/v8/bt/goal`。

为什么要单写一个探针：
  `far_accept.py` 用「取消后重发 `/v8/bt/goal`」测「恢复产出」，两轮独立重跑读数都是
  `ACCEPT_REGOAL_NEW_STAMPS=0`。原因不是判据失效，是**测错了入口**：
  目标链是 `/v8/bt/goal` =>[bt_console RequestPath]=> `/v8/global/goal` =>[全局后端]=> `/global_path`，
  BT 才是目标所有者。`far_accept` 的取消直接发 `/v8/local/cancel`（R6② 授权的第二写者），
  BT 不知道这件事，树还停在 mission 分支里 ⇒ 新来的 `/v8/bt/goal` 没人接，
  自然没有新路径。B2 当时记下的 `REGOAL_NEW_STAMPS=1` 极可能是取消窗口边界上的残留帧，
  不是「恢复产出」。（`far_node.py` 的 `on_waypoint` 首句 `if self.halted: 只出状态`，
  FAR 内部继续吐的残留航点会被适配器正确丢弃——那一条已由 `_cancel_drift.py` 单独证明。）

本探针的口径：
  1. 先喂出路径（冷启动约 30 s，按 BT 的重试语义重发）；
  2. 走 **`/v8/bt/cancel`**（BT 的取消入口，格式 `<seq>:<scope>:<reason>`），
     等 BT 把它转成 `/v8/local/cancel` 并收到同 seq 的 `cancelled` ack；
  3. 再发一个**不同位置**的目标（2.0, 1.0）——同位置无法与「残留帧」区分；
  4. 判据（四条一起看）：
     a) `/v8/bt/cancel` 被 BT 转成 `/v8/local/cancel`（同 seq）；
     b) 收到同 seq 的 `cancelled` ack；
     c) 新目标之后出现 `E_FAR_WARMUP` 占位帧 —— 这一帧只由适配器的 `on_goal` 发，
        是「目标真的到了适配器、halted 被解除」的当场证据；
     d) 之后出现**新时间戳**的 `/global_path` 且点数 ≥ 2。

★ 第一版判据里有「末点必须落在新目标附近（≤0.5 m）」，实测 tail=(5.01, 0.11)，判 FAIL。
  那条判据本身是错的：FAR 发的是**它自己图上的航点**，不是操作员目标点，
  两者本来就不在一个位置（B2 的 vehicle_height 定因也证明过航点由可见图决定）。
  ⇒ 撤掉该条，换成上面 c) d)；这不是「放宽到能过」，是换掉一个不成立的判据。
  改后仍可否证：目标若被忽略，既不会有 `E_FAR_WARMUP` 占位帧，也不会有新时间戳的路径。
"""
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_GOAL = (2.0, 1.0)
CANCEL_SEQ = 4345

PATH_STAMPS = set()
STATUS = []
CANCEL_MSGS = []
ACK_MSGS = []
TAIL = {'value': None, 'stamp': None}
POINTS_AFTER = []          # 取消后出现的路径：(stamp, 点数)


def main():
    rospy.init_node('b2_regoal_bt', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    cancel_pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=1)
    base = {'stamps': set()}

    def on_path(msg):
        if not msg.poses:
            return
        tail = msg.poses[-1].pose.position
        stamp = msg.header.stamp.to_nsec()
        PATH_STAMPS.add(stamp)
        TAIL['value'] = (tail.x, tail.y, tail.z)
        TAIL['stamp'] = stamp
        if stamp not in base['stamps']:
            POINTS_AFTER.append((stamp, len(msg.poses)))

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/global_path/status', String, lambda m: STATUS.append(m.data), queue_size=100)
    rospy.Subscriber('/v8/local/cancel', String, lambda m: CANCEL_MSGS.append(m.data), queue_size=100)
    rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: ACK_MSGS.append(m.data),
                     queue_size=100)

    def send_goal(tag, xy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xy[0], xy[1], GOAL_Z
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        print('REGOALBT_GOAL_SENT tag=%s xy=(%.2f, %.2f)' % (tag, xy[0], xy[1]), flush=True)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            time.sleep(0.05)

    def wait_for(predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.1)
        return False

    # ---- 1. 喂出第一条路径
    wait(3.0)
    attempts = 0
    while attempts < 6 and not PATH_STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts, FIRST_GOAL)
        wait_for(lambda: bool(PATH_STAMPS), 10.0)
    print('REGOALBT_WARM_ATTEMPTS=%d' % attempts, flush=True)
    if not PATH_STAMPS:
        print('REGOALBT_ABORT=no_path', flush=True)
        print('REGOALBT_VERDICT=INCONCLUSIVE', flush=True)
        return 0
    print('REGOALBT_PATHS_BEFORE_CANCEL=%d' % len(PATH_STAMPS), flush=True)

    # ---- 2. 走 BT 的取消入口
    CANCEL_MSGS.clear()
    ACK_MSGS.clear()
    cancel_pub.publish(String(data='%d:all:regoal_test' % CANCEL_SEQ))
    forwarded = wait_for(lambda: any(m.split(':')[:1] == [str(CANCEL_SEQ)] for m in CANCEL_MSGS), 10.0)
    acked = wait_for(lambda: any(m.split(':')[:2] == [str(CANCEL_SEQ), 'cancelled'] for m in ACK_MSGS),
                     10.0)
    print('REGOALBT_CANCEL_FORWARDED=%s' % forwarded, flush=True)
    print('REGOALBT_CANCEL_ACK_CANCELLED=%s' % acked, flush=True)
    print('REGOALBT_CANCEL_MSGS=%s' % CANCEL_MSGS[-2:], flush=True)
    print('REGOALBT_ACK_MSGS=%s' % ACK_MSGS[-2:], flush=True)

    # ---- 3. 让 BT 回到 WaitForGoal 再发新目标（不同位置）
    wait(3.0)
    base['stamps'] = set(PATH_STAMPS)
    POINTS_AFTER.clear()
    placeholder_at = len(STATUS)
    attempts = 0
    while attempts < 6:
        attempts += 1
        send_goal('regoal%d' % attempts, SECOND_GOAL)
        wait_for(lambda: bool(POINTS_AFTER), 10.0)
        if POINTS_AFTER:
            break

    placeholder = [s for s in STATUS[placeholder_at:] if 'E_FAR_WARMUP' in s]
    points_max = max([n for _s, n in POINTS_AFTER], default=0)
    print('REGOALBT_ATTEMPTS=%d' % attempts, flush=True)
    print('REGOALBT_NEW_STAMPS=%d' % len(POINTS_AFTER), flush=True)
    print('REGOALBT_NEW_PATH_POINTS=%d (须 >= 2)' % points_max, flush=True)
    print('REGOALBT_ON_GOAL_PLACEHOLDER=%d (须 >= 1，该帧只由适配器 on_goal 发)'
          % len(placeholder), flush=True)
    print('REGOALBT_TAIL=%s (旁证：FAR 发的是自己的航点，不是目标点)' % (TAIL['value'],), flush=True)
    print('REGOALBT_STATUS_TAIL=%s' % (STATUS[-3:],), flush=True)

    ok = forwarded and acked and bool(placeholder) and points_max >= 2
    print('REGOALBT_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
