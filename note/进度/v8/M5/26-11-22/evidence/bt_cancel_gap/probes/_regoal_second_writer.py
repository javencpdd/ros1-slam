"""8 秒窗探针：第二写者取消后，BT 到底多久才肯接下的目标。

要回答的问题与 `_regoal_via_bt.py` 不同：
  · `_regoal_via_bt.py` 问「走 BT 入口取消，之后重发目标能不能被接」——答案是能（run4 PASS）。
  · 本探针问「**不走 BT 入口**（执行层自诊断取消，或操作员违规直发 `/v8/local/cancel`）时，
    重发的目标要多久才被接」。这正是 `T-20261122-03` 第十一节登记的缺口。

为什么必须换一个判据（这是本探针存在的全部理由）：
  旧判据是「最终是否出现新时间戳路径」。只要等得够久（>到达预算 60 s 地板），
  等 mission 自己超时进 Recovery，队列里的目标**一定会**被执行 ⇒ 旧判据必然变 True，
  **分不出「接得快」与「等到超时才接」**。所以本探针把观察窗钉死在 8 秒，
  而预算地板是 60 s（`arrival_budget_s = max(60, L/0.02)`），窗口小一个量级才有分辨力。

三个判据（取自 `note/架构/双后端可替换_验证记录_2026-11-22.md` 第 11.5 节）：
  J1 `/v8/global/goal` 上出现目标 2 的坐标 —— 说明 `RequestPath` 真跑了（BT 取队了）。
  J2 `/global_path` 出现 t0 之后的**新时间戳**帧且点数 ≥ 2 —— 说明全局后端真出了新路径。
  J3 `/v8/bt/status` 的 `goal_seq` 比 t0 时大 —— 说明 `WaitForGoal::takeGoal` 真的 pop 了。
三条一起看：J1 最灵（取队后毫秒级发），J2 依赖后端，J3 是 BT 自己的计数。

时间轴（全部墙钟 `time.monotonic`：仿真 `/use_sim_time` 为真，墙钟只推进约 0.44 倍）：
  t0        发取消（`--mode direct` 发 `/v8/local/cancel`；`--mode bt` 发 `/v8/bt/cancel` 作反控）
  t0+3.0    发目标 2（与 `_regoal_via_bt.py` 的等待同量级；这样 8 秒窗里还有 5 秒给 BT 反应）
  t0+8.0    收窗、判读

两个模式的意义：
  `--mode direct` 期望 FAIL（修复前的现状，就是本卡要留的失败证据）；
  `--mode bt`     必须 PASS（反控：证明探针对「本该接上的情形」不失灵，不是恒红空控）。

用法（在 `ros1-slam-sim` 容器内、栈已就绪）：
  python3 _regoal_second_writer.py --mode direct --tag runA
退出码：0 恒（判读看 stdout 的 `SW_VERDICT=`），异常路径给 `SW_ABORT=`。
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_GOAL = (2.0, 1.0)
SECOND_WRITER_SEQ = 900000001     # 与适配器自诊断取消同号段的下一号，模拟「第二写者」
BT_ENTRY_SEQ = 1                  # BT 取 max(自身计数+1, 帧里的 seq)，写 1 安全

GOAL_DELAY_S = 3.0
WINDOW_S = 8.0

PATH_STAMPS = set()
GLOBAL_GOALS = []        # 取消后 /v8/global/goal 上的点
BT_GOAL_SEQ = {'value': 0}
CANCEL_OUT = []          # /v8/local/cancel 上看到的帧（BT 转发的）
ACK_IN = []              # /v8/local/cancel_ack
PATH_STATUS = {'ok': False, 'points': 0, 'reason': ''}   # /global_path/status 最新值

# 取消落在哪个节点，是这个缺口最要命的自变量（run1／run2 各抓到一例）：
#   · 落在 WaitForPath（路径还没 ready）⇒ FAR 的 E_FAR_HALTED 不在 BT 的瞬态白名单里
#     ⇒ WaitForPath 当场 FAILURE ⇒ mission 立刻收尾 ⇒ 下一次 tick 就把队列里的目标取走，
#     8 秒窗里表现为 PASS（**且旧目标被自动执行**）；
#   · 落在 WaitArrival／FollowPath（路径已 ready）⇒ 位姿不再前进、无判据可依
#     ⇒ 只能等满预算，8 秒窗里表现为 FAIL。
# 所以探针必须把落点记下来，否则同一模式两次读数会一个 FAIL 一个 PASS，看着像探针不稳。
SETTLE_AFTER_PATH_S = 1.5   # 首帧路径之后再等一会儿，让 BT 走出 WaitForPath，落点才稳定在 ARRIVAL


def near(xy, target, tol=0.05):
    return abs(xy[0] - target[0]) <= tol and abs(xy[1] - target[1]) <= tol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('direct', 'bt'), required=True)
    parser.add_argument('--tag', default='run')
    args = parser.parse_args()

    rospy.init_node('gap_regoal_second_writer', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    second_writer_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)
    bt_entry_pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=1)

    def on_path(msg):
        if not msg.poses:
            return
        PATH_STAMPS.add(msg.header.stamp.to_nsec())

    def on_global_goal(msg):
        GLOBAL_GOALS.append((msg.pose.position.x, msg.pose.position.y))

    def on_status(msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict) and 'goal_seq' in payload:
            BT_GOAL_SEQ['value'] = int(payload['goal_seq'])

    def on_path_status(msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            PATH_STATUS['ok'] = bool(payload.get('ok'))
            PATH_STATUS['points'] = int(payload.get('points') or 0)
            PATH_STATUS['reason'] = str(payload.get('reason') or '')

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/global_path/status', String, on_path_status, queue_size=50)
    rospy.Subscriber('/v8/global/goal', PoseStamped, on_global_goal, queue_size=10)
    rospy.Subscriber('/v8/bt/status', String, on_status, queue_size=50)
    rospy.Subscriber('/v8/local/cancel', String, lambda m: CANCEL_OUT.append(m.data),
                     queue_size=100)
    rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: ACK_IN.append(m.data),
                     queue_size=100)

    def send_goal(tag, xy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xy[0], xy[1], GOAL_Z
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        print('SW_GOAL_SENT tag=%s xy=(%.2f, %.2f)' % (tag, xy[0], xy[1]), flush=True)

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

    print('SW_MODE=%s TAG=%s' % (args.mode, args.tag), flush=True)

    # ---- 1. 喂出第一条路径（冷启动约 30 s）
    wait(3.0)
    attempts = 0
    while attempts < 6 and not PATH_STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts, FIRST_GOAL)
        wait_for(lambda: bool(PATH_STAMPS), 10.0)
    print('SW_WARM_ATTEMPTS=%d' % attempts, flush=True)
    if not PATH_STAMPS:
        print('SW_ABORT=no_path（上游没起步，读数不可用）', flush=True)
        print('SW_VERDICT=INCONCLUSIVE', flush=True)
        return 0
    print('SW_PATHS_BEFORE_CANCEL=%d' % len(PATH_STAMPS), flush=True)

    # 让 BT 走出 WaitForPath 再取消：首帧路径到 FollowPath 有一小段时间，太早取消会落在
    # WaitForPath，那一支的结局完全不同（见文件头 SETTLE_AFTER_PATH_S 的说明）。
    wait(SETTLE_AFTER_PATH_S)
    path_ready = bool(PATH_STATUS['ok']) and PATH_STATUS['points'] > 0
    phase = 'ARRIVAL' if path_ready else 'PATH_PENDING'
    print('SW_CANCEL_PHASE=%s（ARRIVAL＝路径已 ready，取消落在 WaitArrival／FollowPath；'
          'PATH_PENDING＝还在等路径）' % phase, flush=True)
    print('SW_PATH_STATUS_AT_T0=%s' % (dict(PATH_STATUS),), flush=True)

    # ---- 2. t0：发取消（两种模式只在这里分岔）
    CANCEL_OUT.clear()
    ACK_IN.clear()
    GLOBAL_GOALS.clear()
    base_stamps = set(PATH_STAMPS)
    seq = SECOND_WRITER_SEQ if args.mode == 'direct' else BT_ENTRY_SEQ
    seq_at_t0 = BT_GOAL_SEQ['value']
    t0 = time.monotonic()
    if args.mode == 'direct':
        second_writer_pub.publish(String(data='%d:all:second_writer' % seq))
        print('SW_CANCEL_SENT topic=/v8/local/cancel seq=%d（第二写者）' % seq, flush=True)
    else:
        bt_entry_pub.publish(String(data='%d:all:control' % seq))
        print('SW_CANCEL_SENT topic=/v8/bt/cancel seq=%d（反控：走 BT 入口）' % seq, flush=True)

    forwarded = wait_for(lambda: any(m.split(':')[:1] == [str(seq)] for m in CANCEL_OUT), 10.0)
    acked = wait_for(
        lambda: any(m.split(':')[:2] == [str(seq), 'cancelled'] for m in ACK_IN), 10.0)
    # direct 模式下这条帧来自适配器自己（或操作员直发），bt 模式下才是 BT 转发的
    print('SW_CANCEL_FORWARDED=%s（%s）'
          % (forwarded, 'BT 转发' if args.mode == 'bt' else '第二写者自己发的帧，BT 不知情'),
          flush=True)
    print('SW_CANCEL_ACK_CANCELLED=%s' % acked, flush=True)

    # ---- 3. t0+3.0：发目标 2
    wait(GOAL_DELAY_S)
    send_goal('second', SECOND_GOAL)

    # ---- 4. 收到 t0+8.0 为止
    remaining = WINDOW_S - (time.monotonic() - t0)
    wait(max(0.0, remaining))
    elapsed = time.monotonic() - t0

    # ---- 5. 判读（三条各自报，不看总体印象）
    new_stamps = sorted(PATH_STAMPS - base_stamps)
    j1 = any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
    j2 = len(new_stamps) >= 1
    j3 = BT_GOAL_SEQ['value'] > seq_at_t0

    print('SW_WINDOW_S=%.1f ELAPSED_S=%.1f' % (WINDOW_S, elapsed), flush=True)
    print('SW_J1_GLOBAL_GOAL_SEEN=%s（/v8/global/goal 上出现目标 2）' % j1, flush=True)
    print('SW_J2_NEW_PATH_STAMPS=%d（t0 后新时间戳路径帧，须 >= 1）' % len(new_stamps), flush=True)
    print('SW_J3_BT_GOAL_SEQ=%d->%d（须变大）' % (seq_at_t0, BT_GOAL_SEQ['value']), flush=True)
    print('SW_GLOBAL_GOALS_SEEN=%s' % (GLOBAL_GOALS[-4:],), flush=True)
    print('SW_CANCEL_OUT_TAIL=%s' % (CANCEL_OUT[-2:],), flush=True)
    print('SW_ACK_IN_TAIL=%s' % (ACK_IN[-2:],), flush=True)

    ok = j1 and j2 and j3
    # 期望值由「取消落点」决定，不由模式决定：只有落在 ARRIVAL 的第二写者取消才该 FAIL。
    # direct + PATH_PENDING 是「E_FAR_HALTED 被 WaitForPath 当场拒掉」那条快路，PASS 属预期。
    expected = not (args.mode == 'direct' and phase == 'ARRIVAL')
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    print('SW_EXPECT=%s（模式=%s 落点=%s）'
          % ('PASS' if expected else 'FAIL', args.mode, phase), flush=True)
    print('SW_DISCRIMINATING=%s（不一致＝本卡的因果假设被否，不许当读数错误糊过去）'
          % (ok == expected), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
