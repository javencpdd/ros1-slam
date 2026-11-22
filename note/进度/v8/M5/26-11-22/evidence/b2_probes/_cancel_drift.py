"""B2 取消后「不得重新起步」的正控：喂饱路径 → 取消 → 量车还动不动。

为什么单独一个探针：
  卡面第 8 条要求「adapter 残留航点不得重新起步，`/v8/local/goal` 不再被接纳，
  且位置漂移 <= 0.05 m」。前两句能看话题条数，最后一句必须量位姿，而当时那批
  探针只 print 不落盘，这一项从来没有被任何探针量过。

口径（三处容易量错的地方，都在这里钉住）：
  1. **判据从「刹停之后」起算**，不是从「取消那一刻」起算。取消瞬间车还在跑，
     `P1 - P0` 里混着刹车距离，那是正常行为，不是「重新起步」。所以取
     `P_brake`（取消后 4 s）→ `P1`（再 8 s）的位移作判据，`P0 → P_brake`
     只作旁证打印，叫 `DRIFT_BRAKE_M`。
  2. **位姿通道要活着**。车不动时如果位姿也断了，`P1` 会等于旧值，漂移恒 0，
     这是个假绿。所以数后窗内的位姿帧数，少于 5 帧直接判 FAIL。
  3. **等待一律墙钟**（`time.monotonic`）：仿真 `/use_sim_time` 为真，
     墙钟只推进约 0.44 倍。

输出形如 `DRIFT_*=`，供 `b2_probes.sh` 原样落盘。
"""
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

GOAL_Z = 0.11
CANCEL_SEQ = 4344
RESTART_TOL_M = 0.05
BRAKE_SETTLE_S = 4.0
RESTART_WINDOW_S = 8.0
POSE_MIN_FRAMES = 5

PATH_STAMPS = set()
STATUS = []
CANCEL_MSGS = []
GOAL_STAMPS = set()
POSE = {'value': None, 'frames': 0}


def main():
    rospy.init_node('b2_cancel_drift', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    cancel_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)

    def on_path(msg):
        PATH_STAMPS.add(msg.header.stamp.to_nsec())

    def on_status(msg):
        STATUS.append(msg.data)

    def on_cancel(msg):
        CANCEL_MSGS.append(msg.data)

    def on_local_goal(msg):
        GOAL_STAMPS.add(msg.header.stamp.to_nsec())

    def on_pose(msg):
        position = msg.pose.position
        POSE['value'] = (position.x, position.y, position.z)
        POSE['frames'] += 1

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/global_path/status', String, on_status, queue_size=100)
    rospy.Subscriber('/v8/local/cancel', String, on_cancel, queue_size=100)
    rospy.Subscriber('/v8/local/goal', PoseStamped, on_local_goal, queue_size=10)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=10)

    def send_goal(tag):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 2.0, 0.0, GOAL_Z
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        print('DRIFT_GOAL_SENT tag=%s z=%.3f' % (tag, GOAL_Z), flush=True)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            time.sleep(0.05)

    def delta(a, b):
        return max(abs(a[i] - b[i]) for i in range(3)) if a and b else None

    # ---- 前置：把路径喂出来（FAR 冷启动约 30 s，重发是 BT 的真实重试语义）
    wait(3.0)
    attempts = 0
    while attempts < 6 and not PATH_STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts)
        end = time.monotonic() + 10.0
        while time.monotonic() < end and not PATH_STAMPS:
            time.sleep(0.2)
    print('DRIFT_WARM_ATTEMPTS=%d' % attempts, flush=True)
    if not PATH_STAMPS:
        print('DRIFT_ABORT=no_path', flush=True)
        print('DRIFT_VERDICT=INCONCLUSIVE', flush=True)
        return 0

    # 等到局部目标真的下发过（证明 path_goal → local 这条边通）
    end = time.monotonic() + 10.0
    while time.monotonic() < end and not GOAL_STAMPS:
        time.sleep(0.2)
    wait(2.0)                                     # 让它真的开始走

    pose0 = POSE['value']
    local_goal_before = len(GOAL_STAMPS)
    path_before = len(PATH_STAMPS)

    # ---- 取消
    cancel_pub.publish(String(data='%d:all:drift_test' % CANCEL_SEQ))
    seen = False
    end = time.monotonic() + 5.0
    while time.monotonic() < end and not seen:
        seen = any('E_FAR_CANCEL_RECEIVED' in s and str(CANCEL_SEQ) in s for s in STATUS)
        time.sleep(0.05)

    wait(BRAKE_SETTLE_S)                          # 刹车距离落在这段里，不进判据
    pose_brake = POSE['value']
    frames0 = POSE['frames']
    path_after_cancel = set(PATH_STAMPS)
    goal_after_cancel = set(GOAL_STAMPS)

    wait(RESTART_WINDOW_S)                        # 判据窗口：这一段里不许再动
    pose1 = POSE['value']

    print('DRIFT_CANCEL_SEEN=%s' % seen, flush=True)
    print('DRIFT_CANCEL_ECHO=%d (可接受 1)' % sum(1 for m in CANCEL_MSGS
                                                 if m.split(':')[0] == str(CANCEL_SEQ)), flush=True)
    print('DRIFT_LOCAL_GOAL_BEFORE=%d' % local_goal_before, flush=True)
    print('DRIFT_PATH_STAMPS_BEFORE=%d' % path_before, flush=True)
    print('DRIFT_P0=(%.3f, %.3f, %.3f)' % pose0 if pose0 else 'DRIFT_P0=None', flush=True)
    print('DRIFT_P_BRAKE=(%.3f, %.3f, %.3f)' % pose_brake if pose_brake else 'DRIFT_P_BRAKE=None',
          flush=True)
    print('DRIFT_P1=(%.3f, %.3f, %.3f)' % pose1 if pose1 else 'DRIFT_P1=None', flush=True)
    brake = delta(pose0, pose_brake)
    restart = delta(pose_brake, pose1)
    print('DRIFT_BRAKE_M=%s (旁证，不入判据)' % ('%.3f' % brake if brake is not None else 'None'),
          flush=True)
    print('DRIFT_RESTART_M=%s (判据 <= %.2f)'
          % ('%.3f' % restart if restart is not None else 'None', RESTART_TOL_M), flush=True)
    post_path = len(PATH_STAMPS - path_after_cancel)
    post_goal = len(GOAL_STAMPS - goal_after_cancel)
    post_pose = POSE['frames'] - frames0
    print('DRIFT_POST_CANCEL_PATH_STAMPS=%d' % post_path, flush=True)
    print('DRIFT_POST_CANCEL_LOCAL_GOAL=%d' % post_goal, flush=True)
    print('DRIFT_POSE_FRAMES_AFTER=%d (须 >= %d，否则位姿断＝假绿)'
          % (post_pose, POSE_MIN_FRAMES), flush=True)

    ok = (seen and restart is not None and restart <= RESTART_TOL_M
          and post_path == 0 and post_goal == 0 and post_pose >= POSE_MIN_FRAMES)
    print('DRIFT_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
