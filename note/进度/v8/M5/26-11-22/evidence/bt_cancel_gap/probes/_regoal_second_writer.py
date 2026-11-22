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
  t0+33.0   判「新目标被接**且后端重新出非空路径**」——证明故障停没把 BT 弄成砖，也没把规划链路弄死

两段必须分开判：只判「旧目标没跑」的话，一个永久卡死的 BT 也能拿满分（修复前它在 ARRIVAL
相位正是卡死状态）；所以第二段「新目标必须跑得起来」是把「安全停止」与「死机」区分开的反控。

★第二段判据 2026-11-23 加严★：原来只判「派发 + `goal_seq` 增长」，两件都是 50 ms 级事件，
探针在 0.1—0.6 秒就判 PASS，此时后端还没出路径（`SW_FRESH_PATHS` 四轮全 0）。那只能证明 BT
恢复了接单，不能证明 FAR 恢复了导航。现在要求三段全过：
  ① 派发（`/v8/global/goal` 出现新目标）＋ 取队（`goal_seq` 增长）
  ② `/global_path` 出一条**新时间戳、点数 ≥ 2** 的路径，且末点贴着新目标（0.5 米内）
  ③ 这条路径经 `/v8/local/goal` 交到局部规划器（局部目标同样贴着新目标）
只有 ① 过 ⇒ 判 FAIL 并显式打 `SW_FRESH_PATHS=0`，不许再算恢复导航。

★判据第二轮修正（2026-11-23）★：②的采样点从「窗内第一帧」改成「窗内任一帧」，并加 `SW_FRESH_CHAIN`
把链上每一跳打出来（`/v8/global/goal` → `/goal_point` → `/way_point` → `/global_path` →
`/v8/local/goal`）。实测第一帧的末点常常是**旧目标**那个航点（新目标写进后端入口后，上游回调还在
排队，上一拍的航点先到）——按第一帧判会把「后端还没轮到」误判成「后端没重新规划」，而两者的处置
完全不同：前者改采样，后者要查后端。

★判据第三轮修正（2026-11-23）★：②不再以「末点贴住操作员目标」为准，改判**重规划里程碑**，
收敛时长降为读数（`SW_FRESH_CONVERGE_S`）。依据两条实测：
  · 不取消、直接换目标的对照组 `_fresh_path_control.py` 同样要 12.9／13.4／15.0／18.0 秒
    （n=4）才让末点落到目标上，且换相序列同样先在图顶点间游走 ⇒ 这段延迟与故障停无关；
  · 上游 `ProjectNavWaypoint`（`far_planner.cpp:339-378`）在航点距机器小于 `local_planner_range`
    （5.0 m）时沿航向把航点**外推 5 米**，而 `/way_point` 等于目标本身只在 `PlanGlobalPath`
    判出收敛那一拍（`graph_planner.cpp:163-178`）⇒「末点＝目标」是到达事件，不是规划事件。
现判据六项：派发、取队、适配器转发新目标、非空路径 ≥ 2 点、末点与发帧前那条差 > 0.3 m、局部目标
等于某条新路径的末点。前四项证明链路通，第五项证明是**换目标重规划**而非旧路径重放，第六项证明
目标真的交到了局部规划器。

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
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_GOAL = (2.0, 1.0)
OLD_GOAL = (2.0, -1.0)           # 旧目标专用坐标：只出现在「取消前就排队」的目标上
SECOND_WRITER_SEQ = 900000001
MARKER_SEQ = 900000001           # marker 模式冒充「后端已经收到这个号的取消」     # 与适配器自诊断取消同号段的下一号，模拟「第二写者」
BT_ENTRY_SEQ = 1                  # BT 取 max(自身计数+1, 帧里的 seq)，写 1 安全

GOAL_DELAY_S = 3.0
WINDOW_S = 8.0
OLD_QUEUE_SETTLE_S = 1.5         # 旧目标入队后等一会儿，确保它们在取消之前真的排在队列里
OLD_GOAL_SPACING_S = 0.3         # 两条旧目标之间的间隔：/v8/bt/goal 订阅队列只有 1
FRESH_WINDOW_S = 8.0             # 故障停后「新目标被接（派发 + 取队）」的观察窗
FRESH_PATH_WINDOW_S = 25.0       # 故障停后「后端真出一条非空路径」的观察窗（规划要几秒，8 秒不够）
DEQUEUE_WAIT_S = 20.0            # pending 相位：等「取队」出现的最长等待

PATH_STAMPS = set()
PATH_FRAMES = []         # /global_path 的每一帧：(stamp_ns, 点数, 末点xy, 收到时刻)
LOCAL_GOALS = []         # /v8/local/goal 的每一次：(x, y, 收到时刻)
PATH_GOAL_STATUS = {}    # /v8/3d/path_goal/status 最新值（带 poses 与 goal 坐标）
GLOBAL_GOALS = []        # 取消后 /v8/global/goal 上的点
BT_STATUS = {'goal_seq': 0, 'queue_depth': 0, 'pending_dropped': 0,
             'cancel_origin': '', 'event': '', 'nodes': {}, 'raw': ''}
CANCEL_OUT = []          # /v8/local/cancel 上看到的帧（BT 转发的）
ACK_IN = []              # /v8/local/cancel_ack
PATH_STATUS = {'ok': False, 'points': 0, 'reason': ''}   # /global_path/status 最新值
# ---- 链路逐点读数（第二施工段补）：为什么新目标没出路径，必须看链上每一跳 ----
#   /v8/global/goal  BT 写后端           ← 已有 GLOBAL_GOALS
#   /goal_point      适配器转发给上游 FAR
#   /way_point       上游 FAR 回的航点
#   /global_path     适配器出的两点路径
#   /v8/local/goal   路径转出来的局部目标
# 只判头尾会把「上游没接新目标」误读成「探针采样太早」——这两件事的处置完全不同。
GOAL_POINTS = []         # /goal_point 的每一次：(x, y, 收到时刻)
WAYPOINTS = []           # /way_point 的每一次：(x, y, 收到时刻)
POSE = {'xy': None}      # /v8/local/pose 最新位姿

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
    parser.add_argument('--mode', choices=('direct', 'bt', 'marker'), required=True,
                        help='marker＝只发后端回显、不发取消帧也不等回执（无 ACK 场景）')
    parser.add_argument('--queue-old', type=int, default=0,
                        help='取消前先塞 N 条旧目标，验证故障停后它们不会自己跑起来')
    parser.add_argument('--phase', choices=('arrival', 'pending'), default='arrival',
                        help='取消要落进哪个相位；pending 需要 queue-old >= 1')
    parser.add_argument('--fresh-window', type=float, default=FRESH_WINDOW_S,
                        help='故障停后新目标的观察窗（冷启动场景要放大）')
    parser.add_argument('--fresh-path-window', type=float, default=FRESH_PATH_WINDOW_S,
                        help='故障停后等新路径的观察窗；规划链路本身要几秒，不能和派发窗同量级')
    parser.add_argument('--expect', choices=('auto', 'pass', 'fail'), default='auto',
                        help='auto＝按修复前的因果假设推期望；修复后用 pass 钉住新期望')
    parser.add_argument('--tag', default='run')
    args = parser.parse_args()

    rospy.init_node('gap_regoal_second_writer', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    arrival_pub = rospy.Publisher('/v8/bt/arrived', Bool, queue_size=1)
    second_writer_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)
    path_status_pub = rospy.Publisher('/global_path/status', String, queue_size=1)
    bt_entry_pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=1)

    def on_path(msg):
        if not msg.poses:
            return
        stamp = msg.header.stamp.to_nsec()
        PATH_STAMPS.add(stamp)
        last = msg.poses[-1].pose.position
        PATH_FRAMES.append((stamp, len(msg.poses), (round(last.x, 2), round(last.y, 2)),
                            time.monotonic()))

    def on_local_goal(msg):
        p = msg.pose.position
        LOCAL_GOALS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_path_goal_status(msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            PATH_GOAL_STATUS.clear()
            PATH_GOAL_STATUS.update(payload)

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

    def on_point_goal(msg):
        p = msg.point
        GOAL_POINTS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_waypoint(msg):
        p = msg.point
        WAYPOINTS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_pose(msg):
        p = msg.pose.position
        POSE['xy'] = (round(p.x, 2), round(p.y, 2))

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/global_path/status', String, on_path_status, queue_size=50)
    rospy.Subscriber('/v8/local/goal', PoseStamped, on_local_goal, queue_size=20)
    rospy.Subscriber('/v8/3d/path_goal/status', String, on_path_goal_status, queue_size=20)
    rospy.Subscriber('/v8/global/goal', PoseStamped, on_global_goal, queue_size=10)
    # 链上三跳（后端入口之后）：适配器转给上游的目标、上游回的航点、当前位姿。
    # 这三条不参与判据，只用于把「探针采样太早」与「上游没接新目标」分开。
    rospy.Subscriber('/goal_point', PointStamped, on_point_goal, queue_size=20)
    rospy.Subscriber('/way_point', PointStamped, on_waypoint, queue_size=20)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=5)
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
        if mode == 'direct':
            seq = SECOND_WRITER_SEQ
        elif mode == 'marker':
            seq = MARKER_SEQ
        else:
            seq = BT_ENTRY_SEQ
        t0 = time.monotonic()
        if mode == 'direct':
            second_writer_pub.publish(String(data='%d:all:second_writer' % seq))
            print('SW_CANCEL_SENT topic=/v8/local/cancel seq=%d（第二写者）' % seq, flush=True)
        elif mode == 'marker':
            # ★无 ACK 场景★：只发后端自己的回显帧（E_FAR_CANCEL_RECEIVED 带 seq），
            # 既不发取消帧、也不会有任何回执。BT 若只认回执，这一轮必红。
            frame = json.dumps({'ok': False, 'points': 0,
                                'reason': 'E_FAR_CANCEL_RECEIVED', 'seq': seq})
            path_status_pub.publish(String(data=frame))
            print('SW_CANCEL_SENT topic=/global_path/status seq=%d（后端回显，无 ACK）'
                  % seq, flush=True)
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
    if args.mode == 'bt':
        forwarded_note = 'BT 转发'
    elif args.mode == 'marker':
        forwarded_note = '本轮不发取消帧，只看后端回显够不够'
    else:
        forwarded_note = '第二写者自己发的帧，BT 现在能从线上看到它'
    print('SW_CANCEL_FORWARDED=%s（%s）' % (forwarded, forwarded_note), flush=True)
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

    # ---- 第二段：操作员重新下发一条新目标，必须「接到并真的规划出来」
    #
    # ★判据必须「等齐再采」，但「等齐」的内容本批改了★：
    # 旧版等的是「派发 + goal_seq 增长」，两件都是 50 ms 级的逐腿事件，于是探针在 0.1—0.6 秒
    # 就断言成功——那时候后端还没出路径，`SW_FRESH_PATHS` 四轮全 0 却仍判 PASS。这只证明
    # BT 肯接单，不证明 FAR 恢复了导航。
    # 现版分三段等：
    #   ① 派发 + 取队（逐腿事件，`--fresh-window`，缺省 8 秒）
    #   ② 后端真出一条非空路径（`--fresh-path-window`，缺省 25 秒；规划链路是秒级）
    #   ③ 这条路径经 `/v8/local/goal` 交到局部规划器（局部目标须贴着新目标）
    # 三段全过才算恢复导航。①单独过只说明队列没卡死，②③才说明规划链路回来了。
    attempts = 0
    fresh_ok = False
    fresh_history = []
    while attempts < 2 and not fresh_ok:
        attempts += 1
        seq_before_fresh = BT_STATUS['goal_seq']
        GLOBAL_GOALS.clear()
        frames_before = len(PATH_FRAMES)
        local_before = len(LOCAL_GOALS)
        # 链上两条旁证的下标必须在 send_goal **之前**抓：实测 `/goal_point` 在派发那一拍
        # 就发出去了（适配器 on_goal 是秒内响应），等到「派发窗」结束时再抓下标，正好把
        # 那一条漏掉——首轮就是这么读到 `GOAL_POINTS=[]`、错判成「适配器没转发新目标」。
        goal_pt_before = len(GOAL_POINTS)
        wp_before = len(WAYPOINTS)
        pose_at_fresh = POSE['xy']
        # 发帧前最后一条非空路径的末点：判「后端是否真的重规划」要拿它当基准。
        # 只判「窗内有新帧」不够——后端把旧路径按新时间戳重发一遍也能满足，那不是恢复。
        pre_frames = [f for f in PATH_FRAMES[:frames_before] if f[1] >= 2]
        pre_tail = pre_frames[-1][2] if pre_frames else None
        # `/goal_point` 是 far 适配器专有的中间话题（`far_node.py` 的 `far_goal_topic`），
        # support 后端整条链没有它。判据③只对 far 链有意义，所以先记「链上有没有这两条话题」：
        # 暖机腿已经跑过，far 链一定有 `/way_point`；support 链两条都没有。
        # ★不能把「话题不存在」判成「没转发」★——那是把判据套到了不适用的后端上（实测
        # `probe_support4/round_direct2` 五项全绿却被第③项拖成 FAIL）。
        chain_present = (len(WAYPOINTS) > 0) or (len(GOAL_POINTS) > 0)
        send_goal('fresh%d' % attempts, SECOND_GOAL)
        fresh_sent_at = time.monotonic()

        # 第一段：等「派发 + 取队」。这两项是逐腿事件，50 ms 量级就该出现。
        deadline = fresh_sent_at + args.fresh_window
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
                    and BT_STATUS['goal_seq'] > seq_before_fresh):
                break
            time.sleep(0.1)
        waited_dispatch = time.monotonic() - fresh_sent_at
        seen = any(near(xy, SECOND_GOAL) for xy in GLOBAL_GOALS)
        seq_after_fresh = BT_STATUS['goal_seq']

        # 第二段：等「后端真出一条指向新目标的非空路径」。这一段是本批补上的必过项——
        # 派发只证明 BT 接单，路径才证明 FAR 真的重新规划了。规划链路是秒级，
        # 所以必须单独给窗口，不能沿用派发那一档（旧版就是在这里读 0 还判 OK）。
        #
        # ★不能取「窗内第一帧」★（首轮实测踩到）：新目标写进 `/v8/global/goal` 之后，
        # 上游 FAR 的回调还在排队，它上一拍吐的旧目标航点仍会先到一帧，于是窗内第一帧的
        # 末点是**旧目标**（实测 (2.0, 0.0)，新目标是 (2.0, 1.0)）。那一帧既不能当成功，
        # 也不该当成「没出路径」——所以要一路收到窗口尽头。
        #
        # ★★第三轮修正（2026-11-23）：判据从「末点贴住操作员目标」改到「重规划里程碑」★★
        # 依据是两条实测事实，不是为了让读数变绿：
        #   ① 不取消、直接换目标的对照组（`_fresh_path_control.py`，n=4）同样要 12.9—18.0 s
        #      才把末点落到目标上，且末点序列同样先在图顶点间游走 ⇒ 这段延迟与故障停无关；
        #   ② 上游 `ProjectNavWaypoint`（`far_planner.cpp:339-378`）在航点距机器 <
        #      `local_planner_range`(5.0 m) 时会**沿航向把航点外推 5 米**；而 `/way_point`
        #      变成目标本身，只在 `PlanGlobalPath` 判出收敛/到达那一拍（`graph_planner.cpp:163-178`
        #      `GoalReset` 后 `is_goal_init_=false`）。
        # 也就是说「末点＝目标」是**到达事件**（受机器人速度与上游到达判据支配），不是**规划事件**。
        # 拿它当「恢复导航」的判据，等于把判据落在会变的代理量上（与坑 88/89 同族）。
        # 现判据改看三件确定性事件，收敛时长另立读数如实报：
        #   ②a 新帧存在且末点**不同于**发帧前的末点（> 0.3 m）＝后端真的重规划了，不是旧路径重放；
        #   ②b 至少 2 点；
        #   ③ `/v8/local/goal` 收到的点**等于某条新路径的末点**＝对应目标交到了局部规划器。
        path_deadline = time.monotonic() + args.fresh_path_window
        new_frames = []
        matched = None
        while time.monotonic() < path_deadline and not rospy.is_shutdown():
            new_frames = [f for f in PATH_FRAMES[frames_before:]
                          if f[1] >= 2 and f[3] >= fresh_sent_at]
            matched = next((f for f in new_frames if near(f[2], SECOND_GOAL, 0.5)), None)
            if matched is not None:
                break
            time.sleep(0.2)
        waited_path = time.monotonic() - fresh_sent_at
        path_ok = bool(new_frames)
        last_frame = matched or (new_frames[-1] if new_frames else (0, 0, None))
        path_points = last_frame[1]
        path_tail = last_frame[2]
        tails = []
        timeline = []
        for frame in new_frames:
            if frame[2] not in tails:
                tails.append(frame[2])
                timeline.append((round(frame[3] - fresh_sent_at, 1), frame[2]))
        # 重规划判据：新帧里至少一条末端与发帧前那条差得开（> 0.3 m）。
        # 阈值取 0.3：上游在两次规划之间会把航点挪几十分之一米（图顶点重合、位姿微动），
        # 那是「同一条路径重发」而不是换了目标；实测换目标后的第一次换相都在米级。
        replanned = [t for t in tails
                     if pre_tail is None
                     or abs(t[0] - pre_tail[0]) > 0.3 or abs(t[1] - pre_tail[1]) > 0.3]
        path_replanned = bool(replanned)
        # 收敛读数（只报不判）：第一帧末点贴住操作员目标的时刻。
        converge_s = (round(matched[3] - fresh_sent_at, 1) if matched else None)

        # 第三段：局部规划器的输入口（/global_path 经 v8_path_goal 转出来的
        # /v8/local/goal）必须真的收到**这条路径的目标**，光有全局路径不算交给局部。
        pg_ok = bool(PATH_GOAL_STATUS.get('ok')) and int(PATH_GOAL_STATUS.get('poses') or 0) >= 2
        local_new = [g for g in LOCAL_GOALS[local_before:] if g[2] >= fresh_sent_at]
        local_matched = None
        for goal_xy in local_new:
            hit_tail = next((t for t in tails if near((goal_xy[0], goal_xy[1]), t, 0.3)), None)
            if hit_tail is not None:
                local_matched = ((goal_xy[0], goal_xy[1]), hit_tail)
                break
        local_hit = next((g for g in local_new if near((g[0], g[1]), SECOND_GOAL, 0.5)), None)
        local_xy = (local_new[-1][0], local_new[-1][1]) if local_new else None
        local_ok = local_matched is not None
        goal_pts = [(g[0], g[1]) for g in GOAL_POINTS[goal_pt_before:] if g[2] >= fresh_sent_at]
        # 第③项按后端适用：far 链必须看到适配器把新目标转给上游；support 链没有这条话题，跳过。
        goal_pt_hit = any(near(g, SECOND_GOAL, 0.1) for g in goal_pts)
        goal_pt_ok = goal_pt_hit if chain_present else True
        way_pts = [(w[0], w[1]) for w in WAYPOINTS[wp_before:] if w[2] >= fresh_sent_at]

        fresh_ok = bool(seen and seq_after_fresh > seq_before_fresh and goal_pt_ok
                        and path_ok and path_replanned and path_points >= 2 and local_ok)
        fresh_history.append((attempts, seen, seq_after_fresh - seq_before_fresh,
                              path_points, round(waited_path, 1), local_ok))
        print('SW_FRESH_WINDOW_S=%.1f WAITED_DISPATCH_S=%.1f WAITED_PATH_S=%.1f ATTEMPT=%d'
              % (args.fresh_window, waited_dispatch, waited_path, attempts), flush=True)
        print('SW_FRESH_STAMP_SEEN=%s（/v8/global/goal 上出现新目标坐标＝RequestPath 真跑了）'
              % seen, flush=True)
        print('SW_FRESH_GOAL_SEQ=%d->%d（须变大）'
              % (seq_before_fresh, seq_after_fresh), flush=True)
        print('SW_FRESH_PATHS=%d（这条腿之后后端出的非空路径帧数，须 >= 1）'
              % len(new_frames), flush=True)
        print('SW_FRESH_PATH_POINTS=%d PATH_TAIL=%s（须 >= 2 点；末点是后端此刻给的目标）'
              % (path_points, path_tail), flush=True)
        print('SW_FRESH_PRE_TAIL=%s（发帧前最后一条路径的末点，作重规划基准）'
              % (pre_tail,), flush=True)
        print('SW_FRESH_PATH_TAILS=%s（窗内出现过所有末点的先后序列，截前 8）'
              % (tails[:8],), flush=True)
        print('SW_FRESH_CONVERGE_S=%s（读数不判：首帧末点贴住操作员目标用了多少秒；'
              'None＝窗内没贴上，属上游到达判据与车速支配）' % converge_s, flush=True)
        print('SW_FRESH_TAIL_TIMELINE=%s（末点换相的相对时刻，看的是上游收敛过程）'
              % (timeline[:8],), flush=True)
        print('SW_FRESH_PATH_REPLANNED=%s（须 True：窗内末点与发帧前那条差 > 0.3 m'
              '＝后端真的换目标重规划，不是旧路径重放）' % path_replanned, flush=True)
        print('SW_FRESH_LOCAL_GOAL=%s LOCAL_MATCH=%s LOCAL_LAST=%s'
              '（须 True：局部规划器输入口收到的点等于某条新路径的末点）'
              % (local_ok, local_matched, local_xy), flush=True)
        print('SW_FRESH_LOCAL_HIT_GOAL=%s（读数不判：局部目标是否已贴住操作员目标）'
              % (local_hit is not None), flush=True)
        print('SW_FRESH_PATH_GOAL_STATUS=%s' % (PATH_GOAL_STATUS,), flush=True)
        wp_hit = any(near(w, SECOND_GOAL, 0.5) for w in way_pts)
        print('SW_FRESH_CHAIN POSE_AT_SEND=%s POSE_AT_END=%s（位姿：派发那一瞬 / 收工那一瞬）'
              % (pose_at_fresh, POSE['xy']), flush=True)
        print('SW_FRESH_CHAIN_GOAL_POINT n=%d APPLICABLE=%s HIT=%s LAST=%s'
              '（适配器转给上游 FAR 的 /goal_point；support 后端无此话题，该行 APPLICABLE=False）'
              % (len(goal_pts), chain_present, goal_pt_hit,
                 goal_pts[-1] if goal_pts else None), flush=True)
        print('SW_FRESH_CHAIN_WAYPOINT n=%d HIT=%s FIRST=%s'
              '（上游回的 /way_point，HIT＝窗内出现过贴新目标的航点）'
              % (len(way_pts), wp_hit, way_pts[0] if way_pts else None), flush=True)
        print('SW_FRESH_OK=%s（派发 + 取队 + 适配器转发新目标 + 非空重规划路径'
              ' + 局部目标等于该路径末点 六項全过）'
              % fresh_ok, flush=True)
        # ★只在「没派发」时重发★：派发成功但路径没出来，重发的那条只会在队列里排队
        # （实测第二轮 `goal queued depth=1` 之后再无 `dequeued`），既修不了路径，还往
        # 下一轮验收里塞一条没人执行的目标。旧版无条件重发，把这种沉积当成了「接单失败」。
        if fresh_ok or seen:
            break
    print('SW_FRESH_ATTEMPTS=%d（1＝一次就接；>1＝首条没派发、重发才接）'
          % attempts, flush=True)
    print('SW_FRESH_HISTORY=%s（次数, 派发, seq增, 路径点数, 等路径秒, 局部目标）'
          % (fresh_history,), flush=True)
    print('SW_GLOBAL_GOALS_SEEN=%s PATHS_TOTAL=%d' % (GLOBAL_GOALS[-4:], len(PATH_STAMPS)),
          flush=True)
    print('SW_BT_STATUS_RAW_TAIL=%s' % (BT_STATUS['raw'][-160:],), flush=True)

    ok = queue_safe and fresh_ok
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    print('SW_EXPECT=PASS（旧目标停住 + 新目标重新出非空路径并交给局部规划器，两段都要）',
          flush=True)
    print('SW_DISCRIMINATING=%s' % (ok,), flush=True)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
