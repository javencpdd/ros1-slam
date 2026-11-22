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

★取消落点必须读 BT 自己的节点状态，不许靠猜★
缺口只在取消落进 `WaitArrival`／`FollowPath` 时才表现为「卡死」；落进 `WaitForPath` 时是
另一幅面孔（`E_FAR_HALTED` 不在瞬态白名单 ⇒ mission 当场收尾 ⇒ 队列里的旧目标被自动取走）。
run1／run2 各抓到一例，同一模式一轮 FAIL 一轮 PASS 看着像探针不稳。现在改成读
`/v8/bt/status` 的 `nodes` 数组，取 `wait_path`／`wait_arrival`／`follow_path` 谁在 RUNNING
（判读顺序即树的执行顺序），落点由 BT 自己报，不由探针推测。

★`--queue-old N`（T-20261122-04 施工阶段新增）★
上面那套判据测的是「外部取消之后**新下发**的目标能不能被接」。故障停还有第二件事要证：
**取消之前就已经排在队列里的旧目标，不得在故障停之后自己跑起来**。旧目标与「新目标」的区别
在队列里，不在话题上，所以必须先把旧目标塞进队列再取消：

  t0-1.5    塞 N 条旧目标（坐标 OLD_GOAL，与新目标不同坐标，便于从 `/v8/global/goal` 认出来）
  t0        发取消
  t0+8.0    判「旧目标没被派发、`goal_seq` 没涨、队列被清空、`pending_dropped` ≥ N」
  t0+8.0    再发一条**新**目标（模拟操作员重新明确下发）
  t0+16.0   判「新目标在窗口内被接」——证明故障停没把 BT 弄成砖，只是不再自动接着跑旧目标

两段必须分开判：只判「旧目标没跑」的话，一个永久卡死的 BT 也能拿满分（修复前它在 ARRIVAL
相位正是卡死状态）；所以第二段「新目标必须跑得起来」是把「安全停止」与「死机」区分开的反控。

`--phase pending` 是给 PATH_PENDING 相位用的：先暖机拿到首帧路径（让适配器收敛、航点流跑起来），
把旧目标排进队列，然后发一帧外部到达信号 `/v8/bt/arrived=true` 把当前腿收掉——BT 会立刻取走
队首那条旧目标并进入 `WaitForPath`，这时再发取消，落点才在 PATH_PENDING。直接等「自然取消在
WaitForPath」是撞运气，run2 那次就是这么撞上的。

用法（在 `ros1-slam-sim` 容器内、栈已就绪）：
  python3 _regoal_second_writer.py --mode direct --tag runA
  python3 _regoal_second_writer.py --mode direct --queue-old 3 --phase arrival --tag runB
  python3 _regoal_second_writer.py --mode direct --queue-old 3 --phase pending --tag runC
退出码：0 恒（判读看 stdout 的 `SW_VERDICT=`），异常路径给 `SW_ABORT=`。
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_GOAL = (2.0, 1.0)
OLD_GOAL = (2.0, -1.0)           # 旧目标专用坐标：只出现在「取消前就排队」的目标上
SECOND_WRITER_SEQ = 900000001     # 与适配器自诊断取消同号段的下一号，模拟「第二写者」
BT_ENTRY_SEQ = 1                  # BT 取 max(自身计数+1, 帧里的 seq)，写 1 安全

GOAL_DELAY_S = 3.0
WINDOW_S = 8.0
OLD_QUEUE_SETTLE_S = 1.5         # 旧目标入队后等一会儿，确保它们在取消之前真的排在队列里
OLD_GOAL_SPACING_S = 0.3         # 两条旧目标之间的间隔：/v8/bt/goal 订阅队列只有 1
FRESH_WINDOW_S = 8.0             # 故障停之后新目标的观察窗
DEQUEUE_WAIT_S = 20.0            # pending 相位：等「取队」出现的最长等待

PATH_STAMPS = set()
GLOBAL_GOALS = []        # 取消后 /v8/global/goal 上的点
BT_STATUS = {'goal_seq': 0, 'queue_depth': 0, 'pending_dropped': 0,
             'cancel_origin': '', 'event': '', 'nodes': {}, 'raw': ''}
CANCEL_OUT = []          # /v8/local/cancel 上看到的帧（BT 转发的）
ACK_IN = []              # /v8/local/cancel_ack
PATH_STATUS = {'ok': False, 'points': 0, 'reason': ''}   # /global_path/status 最新值

# 取消落在哪个节点是这个缺口最要命的自变量，所以这两个常量只服务于「暖机后落点稳定在 ARRIVAL」。
SETTLE_AFTER_PATH_S = 1.5


def near(xy, target, tol=0.05):
    return abs(xy[0] - target[0]) <= tol and abs(xy[1] - target[1]) <= tol


def landing_node(nodes):
    """按树的执行顺序取第一个 RUNNING 的 mission 节点——就是取消落点。"""
    for name in ('wait_path', 'wait_arrival', 'follow_path', 'dwell_at_goal',
                 'request_path', 'wait_goal', 'cancel_current'):
        if nodes.get(name) == 'RUNNING':
            return name
    return '?'


def phase_of(node):
    if node in ('wait_arrival', 'follow_path'):
        return 'ARRIVAL'
    if node == 'wait_path':
        return 'PATH_PENDING'
    return 'OTHER(%s)' % node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('direct', 'bt'), required=True)
    parser.add_argument('--queue-old', type=int, default=0,
                        help='取消前先塞 N 条旧目标，验证故障停后它们不会自己跑起来')
    parser.add_argument('--phase', choices=('arrival', 'pending'), default='arrival',
                        help='取消要落进哪个相位；pending 需要 queue-old >= 1')
    parser.add_argument('--fresh-window', type=float, default=FRESH_WINDOW_S,
                        help='故障停后新目标的观察窗（冷启动场景要放大）')
    parser.add_argument('--expect', choices=('auto', 'pass', 'fail'), default='auto',
                        help='auto＝按修复前的因果假设推期望；修复后用 pass 钉住新期望')
    parser.add_argument('--tag', default='run')
    args = parser.parse_args()

    rospy.init_node('gap_regoal_second_writer', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    arrival_pub = rospy.Publisher('/v8/bt/arrived', Bool, queue_size=1)
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
        if not isinstance(payload, dict):
            return
        if 'goal_seq' in payload:
            BT_STATUS['goal_seq'] = int(payload['goal_seq'])
        for key in ('queue_depth', 'pending_dropped'):
            if key in payload:
                BT_STATUS[key] = int(payload[key] or 0)
        for key in ('cancel_origin', 'event'):
            if key in payload:
                BT_STATUS[key] = str(payload[key] or '')
        nodes = payload.get('nodes')
        if isinstance(nodes, list):
            BT_STATUS['nodes'] = {str(n.get('name')): str(n.get('status'))
                                  for n in nodes if isinstance(n, dict)}
        BT_STATUS['raw'] = msg.data

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

    def do_cancel(mode):
        """发取消并返回 (t0, 落点节点名, 相位, seq, 发帧前的 path 状态快照)。

        落点取发帧前一瞬的节点状态：取消一旦生效，BT 会转进 `cancel_current`，
        之后再读就分不出原来停在哪个节点了。
        """
        node = landing_node(BT_STATUS['nodes'])
        snapshot = dict(PATH_STATUS)
        seq = SECOND_WRITER_SEQ if mode == 'direct' else BT_ENTRY_SEQ
        t0 = time.monotonic()
        if mode == 'direct':
            second_writer_pub.publish(String(data='%d:all:second_writer' % seq))
            print('SW_CANCEL_SENT topic=/v8/local/cancel seq=%d（第二写者）' % seq, flush=True)
        else:
            bt_entry_pub.publish(String(data='%d:all:control' % seq))
            print('SW_CANCEL_SENT topic=/v8/bt/cancel seq=%d（反控：走 BT 入口）' % seq, flush=True)
        return t0, node, phase_of(node), seq, snapshot

    print('SW_MODE=%s TAG=%s QUEUE_OLD=%d PHASE_ARG=%s'
          % (args.mode, args.tag, args.queue_old, args.phase), flush=True)
    if args.phase == 'pending' and args.queue_old < 1:
        print('SW_ABORT=pending 相位必须有旧目标可被取走（--queue-old >= 1）', flush=True)
        print('SW_VERDICT=INCONCLUSIVE', flush=True)
        return 0

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
    print('SW_PATH_STATUS_AFTER_WARM=%s' % (dict(PATH_STATUS),), flush=True)

    # ---- 2.（仅 --queue-old）把旧目标塞进队列，等它们真的排上
    if args.queue_old > 0:
        for index in range(args.queue_old):
            send_goal('old%d' % (index + 1), OLD_GOAL)
            # `/v8/bt/goal` 的订阅队列是 1（bt_console.cpp:1266-1267），三条背靠背发下去
            # 只会留下一条（实测 depth 停在 1）。必须留出回调时间，否则「队列里有 3 条旧目标」
            # 这个前提根本没成立，后面的判据全是空跑。
            wait(OLD_GOAL_SPACING_S)
        wait(OLD_QUEUE_SETTLE_S)
        print('SW_QUEUE_DEPTH_BEFORE_CANCEL=%d（须 >= %d，否则旧目标没排上，本题无意义）'
              % (BT_STATUS['queue_depth'], args.queue_old), flush=True)

    # ---- 3.（仅 --phase pending）用外部到达信号收掉当前腿，让 BT 立刻取队首旧目标
    frames_at_dequeue = None
    if args.phase == 'pending':
        seq_before = BT_STATUS['goal_seq']
        arrival_pub.publish(Bool(data=True))
        print('SW_ARRIVAL_PUSHED seq_before=%d（/v8/bt/arrived=true，把当前腿判到达）'
              % seq_before, flush=True)
        dequeued = wait_for(lambda: BT_STATUS['goal_seq'] > seq_before, DEQUEUE_WAIT_S)
        frames_at_dequeue = len(PATH_STAMPS)
        print('SW_PENDING_DEQUEUED=%s（goal_seq %d->%d，此刻路径帧数=%d）'
              % (dequeued, seq_before, BT_STATUS['goal_seq'], frames_at_dequeue), flush=True)

    # ---- 4. t0：发取消（两种模式只在这里分岔）
    CANCEL_OUT.clear()
    ACK_IN.clear()
    GLOBAL_GOALS.clear()
    base_stamps = set(PATH_STAMPS)
    seq_at_t0 = BT_STATUS['goal_seq']
    depth_at_t0 = BT_STATUS['queue_depth']
    # `/v8/bt/status` 只有 2 Hz，节点快照最多滞后 0.5 s（pending 相位下这条腿到出路径只隔
    # 0.15 s，快照很可能还停在 wait_path）⇒ 落点另用「取队之后有没有新的路径帧」来钉，
    # 帧计数是事件不是采样，不受刷新率影响。
    frames_since_dequeue = (None if frames_at_dequeue is None
                            else len(PATH_STAMPS) - frames_at_dequeue)
    t0, node, phase, seq, snapshot = do_cancel(args.mode)
    print('SW_BT_LANDING_NODE=%s PHASE=%s（发帧前一瞬 BT 的 RUNNING 节点，最多滞后 0.5 s）'
          % (node, phase), flush=True)
    print('SW_FRAMES_SINCE_DEQUEUE=%s（0＝这条腿还没出过路径 ⇒ 落在 WaitForPath）'
          % frames_since_dequeue, flush=True)
    print('SW_PATH_STATUS_AT_T0=%s' % (snapshot,), flush=True)

    forwarded = wait_for(lambda: any(m.split(':')[:1] == [str(seq)] for m in CANCEL_OUT), 10.0)
    acked = wait_for(
        lambda: any(m.split(':')[:2] == [str(seq), 'cancelled'] for m in ACK_IN), 10.0)
    # direct 模式下这条帧来自适配器自己（或操作员直发），bt 模式下才是 BT 转发的
    print('SW_CANCEL_FORWARDED=%s（%s）'
          % (forwarded, 'BT 转发' if args.mode == 'bt' else '第二写者自己发的帧，BT 不知情'),
          flush=True)
    print('SW_CANCEL_ACK_CANCELLED=%s' % acked, flush=True)

    if args.queue_old > 0:
        return judge_queue_old(args, t0, base_stamps, seq_at_t0, depth_at_t0,
                               send_goal, wait, wait_for)

    # ---- 5. t0+3.0：发目标 2
    wait(GOAL_DELAY_S)
    send_goal('second', SECOND_GOAL)

    # ---- 6. 收到 t0+8.0 为止
    remaining = WINDOW_S - (time.monotonic() - t0)
    wait(max(0.0, remaining))
    elapsed = time.monotonic() - t0

    # ---- 7. 判读（三条各自报，不看总体印象）
    new_stamps = sorted(PATH_STAMPS - base_stamps)
    j1 = any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
    j2 = len(new_stamps) >= 1
    j3 = BT_STATUS['goal_seq'] > seq_at_t0

    print('SW_WINDOW_S=%.1f ELAPSED_S=%.1f' % (WINDOW_S, elapsed), flush=True)
    print('SW_J1_GLOBAL_GOAL_SEEN=%s（/v8/global/goal 上出现目标 2）' % j1, flush=True)
    print('SW_J2_NEW_PATH_STAMPS=%d（t0 后新时间戳路径帧，须 >= 1）' % len(new_stamps), flush=True)
    print('SW_J3_BT_GOAL_SEQ=%d->%d（须变大）' % (seq_at_t0, BT_STATUS['goal_seq']), flush=True)
    print('SW_GLOBAL_GOALS_SEEN=%s' % (GLOBAL_GOALS[-4:],), flush=True)
    print('SW_CANCEL_OUT_TAIL=%s' % (CANCEL_OUT[-2:],), flush=True)
    print('SW_ACK_IN_TAIL=%s' % (ACK_IN[-2:],), flush=True)
    print('SW_CANCEL_ORIGIN=%s PENDING_DROPPED=%d QUEUE_DEPTH=%d EVENT=%s'
          % (BT_STATUS['cancel_origin'], BT_STATUS['pending_dropped'],
             BT_STATUS['queue_depth'], BT_STATUS['event']), flush=True)

    ok = j1 and j2 and j3
    # 期望值来源：
    #   auto ＝修复前的因果假设（只有落在 ARRIVAL 的第二写者取消才该 FAIL；direct + PATH_PENDING
    #          是「E_FAR_HALTED 被 WaitForPath 当场拒掉」那条快路，PASS 属预期）；
    #   pass ＝修复后的新期望（两条相位都该在 8 秒窗里接上新目标）。
    if args.expect == 'auto':
        expected = not (args.mode == 'direct' and phase == 'ARRIVAL')
        source = 'auto'
    else:
        expected = args.expect == 'pass'
        source = args.expect
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    print('SW_EXPECT=%s（模式=%s 落点=%s 来源=%s）'
          % ('PASS' if expected else 'FAIL', args.mode, phase, source), flush=True)
    print('SW_DISCRIMINATING=%s（不一致＝本卡的因果假设被否，不许当读数错误糊过去）'
          % (ok == expected), flush=True)
    return 0


def judge_queue_old(args, t0, base_stamps, seq_at_t0, depth_at_t0, send_goal, wait, wait_for):
    """故障停之后：旧目标不得自动执行，但操作员新下发的目标必须跑得起来。

    两段分开判的理由见文件头：只判「旧目标没跑」，一个永久卡死的 BT 也能满分。
    """
    wait(max(0.0, WINDOW_S - (time.monotonic() - t0)))
    old_dispatched = any(near(xy, OLD_GOAL) for xy in GLOBAL_GOALS)
    new_stamps = sorted(PATH_STAMPS - base_stamps)
    seq_after = BT_STATUS['goal_seq']
    depth_end = BT_STATUS['queue_depth']
    dropped = BT_STATUS['pending_dropped']
    origin = BT_STATUS['cancel_origin']
    event = BT_STATUS['event']

    # 队列必须在故障停之后是空的，且真的丢过东西。depth_at_t0 取的是 2 Hz 状态快照，
    # pending 相位下可能比真实值大（上一拍还没刷新），所以判据只要求「丢过 >= 1」。
    cleared = depth_end == 0 and (dropped >= 1 if depth_at_t0 > 0 else True)
    queue_safe = ((not old_dispatched) and seq_after == seq_at_t0 and cleared)
    print('SW_OLD_QUEUE_WINDOW_S=%.1f' % WINDOW_S, flush=True)
    print('SW_QUEUE_DEPTH_AT_T0=%d（取消前队列深度快照，仅作上下文）' % depth_at_t0, flush=True)
    print('SW_OLD_DISPATCHED=%s（/v8/global/goal 上出现旧目标坐标，须 False）'
          % old_dispatched, flush=True)
    print('SW_OLD_GOAL_SEQ=%d->%d（须不变）' % (seq_at_t0, seq_after), flush=True)
    print('SW_OLD_NEW_PATH_STAMPS=%d（旧目标若跑起来这里会 >0）' % len(new_stamps), flush=True)
    print('SW_QUEUE_DEPTH_END=%d PENDING_DROPPED=%d（须 0 / >= 1）'
          % (depth_end, dropped), flush=True)
    print('SW_CANCEL_ORIGIN=%s EVENT=%s' % (origin, event), flush=True)
    print('SW_QUEUE_SAFE=%s' % queue_safe, flush=True)

    # ---- 第二段：操作员重新下发一条新目标，窗口内必须被接（证明不是卡死）
    #
    # ★判据必须「等齐三项再采」，不能「一看到派发就采」★（probe_fix2b 第一版踩过）：
    # `/v8/bt/status` 只有 2 Hz、路径帧要等后端出第一帧，派发却只要 50 ms，一看到派发就采样
    # 会把「真接上了」读成 False，于是探针 0.1 秒后就急着重发第二条——那是探针自己造出来的
    # 假失败（同一次 launch.log 里 `follow path -> dispatch goal 2` 明明已经出现）。
    #
    # 三项里 `派发` 与 `goal_seq` 都是逐腿事件，能证明「这一条腿被接」；路径帧只证明
    # 「后端重新出帧了」，单独用会把下一条腿的帧算到这条头上，所以它只作旁证、并单独报。
    attempts = 0
    fresh_ok = False
    fresh_history = []
    while attempts < 2 and not fresh_ok:
        attempts += 1
        base2 = set(PATH_STAMPS)
        seq_before_fresh = BT_STATUS['goal_seq']
        GLOBAL_GOALS.clear()
        send_goal('fresh%d' % attempts, SECOND_GOAL)
        fresh_sent_at = time.monotonic()
        deadline = fresh_sent_at + args.fresh_window
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
                    and BT_STATUS['goal_seq'] > seq_before_fresh):
                break
            time.sleep(0.1)
        waited = time.monotonic() - fresh_sent_at
        seen = any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
        seq_after_fresh = BT_STATUS['goal_seq']
        fresh_paths = sorted(PATH_STAMPS - base2)
        fresh_ok = seen and seq_after_fresh > seq_before_fresh
        fresh_history.append((attempts, seen, seq_before_fresh, seq_after_fresh,
                              len(fresh_paths), round(waited, 1)))
        print('SW_FRESH_WINDOW_S=%.1f WAITED_S=%.1f ATTEMPT=%d'
              % (args.fresh_window, waited, attempts), flush=True)
        print('SW_FRESH_STAMP_SEEN=%s（/v8/global/goal 上出现新目标坐标＝RequestPath 真跑了）'
              % seen, flush=True)
        print('SW_FRESH_GOAL_SEQ=%d->%d（须变大）'
              % (seq_before_fresh, seq_after_fresh), flush=True)
        print('SW_FRESH_PATHS=%d（旁证：这条腿之后后端出的路径帧数）'
              % len(fresh_paths), flush=True)
        print('SW_FRESH_OK=%s（派发 + 取队两项为判据）' % fresh_ok, flush=True)
    print('SW_FRESH_ATTEMPTS=%d（1＝一次就接；2＝第一条被拒后重发才接，属既有行为）'
          % attempts, flush=True)
    print('SW_FRESH_HISTORY=%s（次数, 派发, seq前, seq后, 路径帧, 等待秒）'
          % (fresh_history,), flush=True)
    print('SW_GLOBAL_GOALS_SEEN=%s PATHS_TOTAL=%d' % (GLOBAL_GOALS[-4:], len(PATH_STAMPS)),
          flush=True)
    print('SW_BT_STATUS_RAW_TAIL=%s' % (BT_STATUS['raw'][-160:],), flush=True)

    ok = queue_safe and fresh_ok
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    print('SW_EXPECT=PASS（queue-old：旧目标停住 + 新目标可下发，两段都要）', flush=True)
    print('SW_DISCRIMINATING=%s' % (ok,), flush=True)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
