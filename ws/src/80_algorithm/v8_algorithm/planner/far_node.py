#!/usr/bin/env python3
"""FAR 全局后端适配器：语义地形云、两点/带前视的步态路径、取消写者。

分层理由：本模块**顶层只 import json 与 math**，全部 ROS 依赖留在 `main()` 里。
这样七条判据（路径帧数、z 不被压平、h 归类、缺输入只出状态、取消后仍 halted、
前视补点、远航点不补点）能在宿主上离线跑——不需要 roscore，也不需要 overlay 已经编译过。

接口契约（现读上游 `far_planner.cpp:15-30` 与 `config/default.yaml`）：

| 方向 | 话题 | 类型 | 本模块的角色 |
|---|---|---|---|
| 订 | `/scan_cloud` | `PointCloud2` | 直通转发（碰撞检查用原始扫描） |
| 订 | `/goal_point` | `PointStamped` | 由 `/v8/global/goal` 降维产生 |
| 订 | `/way_point` | `PointStamped` | FAR 的 3D 航点，本模块把它变成 Path |
| 订 | `/far_reach_goal_status` | `Bool` | FAR 自报到达（只作旁证读数，不转给 BT） |
| 发 | `/terrain_cloud` | `PointCloud2` | `intensity` ＝ 离地高 h |
| 发 | `/global_path` | `Path` | **两点** `[当前位姿, 航点]`，给 `path_goal` 与 BT |
| 发 | `/v8/gait_guard/path` | `Path` | 两点或**三点**（近距补前视点），只给 `gait_guard` |
| 发 | `/v8/bt/arrived` | `Bool` | 到达判据（`latched`）：`\|pose−goal\|_3d ≤ hypot(0.25, 0.30)` |

★ 两条路径话题必须分开 ★ `path_goal_node.py` 取 `poses[-1]` 当局部目标。
步态用的延长点若混进 `/global_path`，局部目标会被推到航点之外——车会越过 FAR 给的
安全落点继续走。延长点只属于步态规划器，不属于目标语义。

★ 不要复用上游 `far_planner.launch`：它连带起 `rviz` 与 `graph_decoder`，两者都不在最小包集里。
"""
import json
import math

FREE_Z_M = 0.2          # 与上游 `Util/terrain_free_Z: 0.2` 同值；h < 此值算地面
GROUND_CELL_M = 0.2     # 地面栅格边长（米）
LOOKAHEAD_M = 1.5       # 步态路径最小前视距离（米）
LOOKAHEAD_MARGIN_M = 0.3   # 延长段在 lookahead 之外多留的余量（米）：总长 = lookahead + 此值
CANCEL_SEQ_BASE = 900000000   # 取消序号起点：避开 BT 的 cancel_seq（从 1 起递增）
# ★航点"同一个"的量化步长★：FAR 每次事件都会吐一条新 `/way_point`，点位在厘米级抖动。
# 小于这个步长的位移不算"有效航点变化"，不得推进 path_revision（否则等于每帧新任务）。
WAYPOINT_QUANTUM_M = 0.05
# ★mission 切换后的保护窗★：窗口内到达的航点视为"上一目标在途残留"，不打上新 mission 标号。
STALE_WAYPOINT_GUARD_S = 0.30
ARRIVAL_R_XY_M = 0.25      # 到达判据的水平半径（米）：与 BT 树 WaitArrival 的 distance 同值
ARRIVAL_Z_TOL_M = 0.30     # 到达判据的竖向容差（米）：与 support_map 的 z_tol 同量级

ACK_OUTCOMES = ('cancelled', 'ack_fail', 'unacked')


def _xyz(value):
    """从 PoseStamped 样对象或 `(x, y, z)` 取三元组。

    鸭子类型：只读 `.pose.position` 的 x/y/z，不 import 任何 ROS 消息包，
    所以离线测试可以喂字段形状相同的替身，生产侧喂真消息也照样成立。
    """
    pos = getattr(getattr(value, 'pose', value), 'position', value)
    if hasattr(pos, 'x'):
        return float(pos.x), float(pos.y), float(pos.z)
    seq = tuple(float(v) for v in pos)
    if len(seq) != 3:
        raise ValueError('point must have 3 components')
    return seq


def apply_transform(points, translation, rotation):
    """把点从源 frame 转到目标 frame：先按四元数旋转，再平移。

    `translation` 是 `(tx, ty, tz)`，`rotation` 是 `(qx, qy, qz, qw)`（未归一化也接受）。

    为什么自己算而不用 `tf2_sensor_msgs.do_transform_cloud`：
    那个包（`ros-noetic-tf2-sensor-msgs`）在本镜像里没装，而它做的也只是这件事。
    自己算顺带把「转帧」变成可离线测的纯函数——纯平移近似在 1 m 外就会带来 >0.2 m 的
    h 误差，这条判据值得有测试盯着。
    """
    qx, qy, qz, qw = (float(v) for v in rotation)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        qx = qy = qz = 0.0
        qw = 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx, ty, tz = (float(v) for v in translation)
    # 常用项展开，避免每个点算一次矩阵
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    out = []
    for x, y, z in points:
        out.append((
            (1. - 2. * (yy + zz)) * x + 2. * (xy - wz) * y + 2. * (xz + wy) * z + tx,
            2. * (xy + wz) * x + (1. - 2. * (xx + zz)) * y + 2. * (yz - wx) * z + ty,
            2. * (xz - wy) * x + 2. * (yz + wx) * y + (1. - 2. * (xx + yy)) * z + tz,
        ))
    return out


def ground_z_by_cell(points, cell_m=GROUND_CELL_M):
    """每 `cell_m` 栅格取最小 z 当地面，返回 `{(ix, iy): z}`。

    取最小值的理由：同一格里最高的点是障碍物，最低的才是地面。
    """
    cells = {}
    for point in points:
        x, y, z = _xyz(point)
        key = (int(math.floor(x / cell_m)), int(math.floor(y / cell_m)))
        if key not in cells or z < cells[key]:
            cells[key] = z
    return cells


def terrain_cloud(points, h=None, cell_m=GROUND_CELL_M, flatten=True):
    """把 `map` 系点做成 FAR 要的语义地形，每点返回 `(x, y, z_out, height)`。

    `h=None`：按每格最小 z 估地面（真实路径——地面有起伏，逐格抬升）。
    `h` 给定：地面高度已知（单平面输入，如仿真平地或已平整好的一层），直接减。

    `flatten=True`：`z_out` 取该格地面高度，点被压到地面上（CMU terrain_map 的形态）。
    `flatten=False`：`z_out` 保留点的真实高度，障碍物保留自身高程。

    两种形态的 `height` 完全相同，差别只在 FAR 的地图栅格把点记在哪个高度层。
    """
    items = [_xyz(p) for p in points]
    if h is None:
        cells = ground_z_by_cell(items, cell_m)
        out = []
        for x, y, z in items:
            key = (int(math.floor(x / cell_m)), int(math.floor(y / cell_m)))
            ground = cells[key]
            out.append((x, y, ground if flatten else z, z - ground))
        return out
    ground = float(h)
    return [(x, y, ground if flatten else z, z - ground) for x, y, z in items]


def classify(height):
    """离地高归 `free` / `obs`：判据是 `height < FREE_Z_M`（严格小于）。

    与上游对 `terrain_free_Z` 的用法一致——正好 0.2 m 的台阶算障碍，不算地面。
    """
    return 'free' if float(height) < FREE_Z_M else 'obs'


def two_point_path(pose, waypoint):
    """两点 Path：首点取当前位姿、末点取 FAR 航点，**z 原样带过，绝不压平**。

    压平 z 的后果不是「不好看」：局部规划器会拿到一个和真实目标差几米的落点，
    多层场景里直接变成撞楼板或撞层间平台。
    """
    return [_xyz(pose), _xyz(waypoint)]


def gait_path(pose, waypoint, lookahead_m=LOOKAHEAD_M, margin_m=LOOKAHEAD_MARGIN_M):
    """两点，必要时补第三点，让步态前瞻扫得到 `lookahead_m` 以外的一小段。

    `_xyz(waypoint) - _xyz(pose)` 的水平距 `d >= lookahead_m + margin_m`：两点就够
    （扫描本来就到不了航点，补了也是浪费）。
    `d < lookahead_m + margin_m`：沿同方向补一点，使折线总长恰为 `lookahead_m + margin_m`。
    只补一次、不外推递归；补点只在原方向上延长——不改变航向，不引入横向偏移。

    ★ 为什么总长是固定值而不是 `d + lookahead_m` ★ `stair_ahead` 按
    `travelled > lookahead_m` 截断扫描，缺的从来不是「航点那 0.3 m」，而是
    「航点之外还要再走一段才能凑够 lookahead」。总长固定成 `lookahead_m + margin_m`，
    扫描距离就与航点远近无关；写成 `d + lookahead_m` 会让近航点白补一大截、
    远航点又白补一截，两头都不省。

    ★ 这个点只许发给步态规划器 ★ 送进 `/global_path` 时 `pos_goal` 取 `poses[-1]`，
    局部目标会被推到航点之外。
    """
    start, goal = _xyz(pose), _xyz(waypoint)
    dx, dy, dz = goal[0] - start[0], goal[1] - start[1], goal[2] - start[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    reach = float(lookahead_m) + float(margin_m)
    if dist <= 1e-9 or dist >= reach:
        return [start, goal]
    scale = reach / dist
    return [start, goal,
            (start[0] + dx * scale, start[1] + dy * scale, start[2] + dz * scale)]


def arrival_radius(r_xy=ARRIVAL_R_XY_M, z_tol=ARRIVAL_Z_TOL_M):
    """到达判据的半径：`hypot(r_xy, z_tol)`（水平 0.25、竖向 0.30 时约 0.3905 m）。

    ★ 计划 §4.5 把判据写成两段「`|pose-goal|_3d ≤ r3` 或（水平 ≤ r_xy 且 |Δz| ≤ z_tol）」，
    这两段**判得完全一样**：水平 ≤ r_xy 且 |Δz| ≤ z_tol ⇒ 三点范数 ≤ hypot(r_xy, z_tol) = r3。
    第二段永远不接纳第一段拒绝的点，写两个门限只会多出一处「两套门限互相矛盾」的机会。
    所以实现只留**一个半径**；两个参数仍分开配——r_xy 要对齐 BT 树里的 0.25，
    z_tol 要能单独调（它才是跟楼层语义挂钩的那个）。
    """
    return math.hypot(float(r_xy), float(z_tol))


def is_arrived(pose, goal, r_xy=ARRIVAL_R_XY_M, z_tol=ARRIVAL_Z_TOL_M):
    """3D 到达判据：`|pose - goal|_3d ≤ hypot(r_xy, z_tol)`。

    ★ 这是**近似式**，不等于「到对了楼层」★ 两楼层间距若小于 `2 × z_tol` 它分不开。
    真正稳的判据是「pose 所在支撑面 id ＝ goal 指定面 id」，属押后项 D1；
    D1 完成前任何交付说明不得写「支持跨楼层到达」。
    """
    if pose is None or goal is None:
        raise ValueError('pose and goal are both required')
    a, b = _xyz(pose), _xyz(goal)
    dist = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
    return dist <= arrival_radius(r_xy, z_tol)


def status_json(ok, points, reason, **extra):
    """状态帧。字段名与 support 版 `topic_node` 的 status 同名（`ok`/`points`/`reason`），
    这样 BT 侧与日志侧可以用同一套 grep，不用按后端分支。
    """
    value = {'ok': bool(ok), 'points': int(points), 'reason': reason}
    value.update(extra)
    return json.dumps(value, ensure_ascii=False)


# ★ 暖机占位帧的 reason 必须落在 BT 的白名单里 ★
# `bt_console.cpp:161-165` 的 `transient_plan_reason` 是一张**硬编码白名单**：
# 空串 / `E_PLAN_RUNNING` / `E_PLAN_NOT_READY` / `E_PLAN_CANCELLED_REQUESTED`。
# 不在这四个里面的 reason，`WaitForPath` 会直接 `return FAILURE`
# （`bt_console.cpp:324-332`，日志 `[bt] global path refused: <reason>`），任务当场结束、
# 不会等下一帧。B2 的计划曾假定 `E_FAR_WARMUP` 属于「瞬态那一家」，实测证明不在——
# 开了 BT 之后每条目标都在暖机帧上被拒（见 `note/进度/v8/M5/26-11-22/evidence/b5_arrival2/budget/launch.log`
# 的两条 `global path refused: E_FAR_WARMUP`）。所以占位帧对外报 BT 认识的名字，
# 适配器自己的诊断名放 `detail` 字段带走。
BT_TRANSIENT_REASONS = ('E_PLAN_RUNNING', 'E_PLAN_NOT_READY', 'E_PLAN_CANCELLED_REQUESTED')
WARMUP_PLAN_REASON = 'E_PLAN_RUNNING'


def cancel_text(seq, scope='current', reason='far'):
    """组取消帧 `<seq>:<scope>:<reason>`。

    与 `local_planner/cancel_policy.parse_cancel` 同一契约（scope ∈ current/all，
    reason 不得含 `;` 与换行）；不直接 import 它，是为了让本模块顶层保持零 ROS/零跨包依赖。
    契约一致性由 `test_far_node.py` 的互检用例守住。
    """
    if scope not in ('current', 'all'):
        raise ValueError('scope must be current or all')
    text = '%d:%s:%s' % (int(seq), scope, reason)
    if ';' in text or '\n' in text:
        raise ValueError('reason must not contain ; or newline')
    return text


def parse_ack(text):
    """解 `<seq>:<outcome>`，返回 `(seq, outcome)` 或 `None`。

    outcome 只认终态三种，其余（含超时前的空帧）一律当「还没 ack」。
    """
    if not isinstance(text, str):
        return None
    parts = text.strip().split(':', 1)
    if len(parts) != 2 or not parts[0].isdigit():
        return None
    if parts[1] not in ACK_OUTCOMES:
        return None
    return int(parts[0]), parts[1]


class AdapterState:
    """适配器的可测内核：谁在写航点、被谁停、停到什么时候解除。

    `halted` **只被新 goal 解除**。取消之后 FAR 内部可能还在吐残留航点（它的图还在收敛），
    那些航点一律不接纳——否则取消等于没取消，机器人会立刻沿旧航点继续走。
    """

    def __init__(self, lookahead_m=LOOKAHEAD_M, seq_base=CANCEL_SEQ_BASE,
                 r_xy=ARRIVAL_R_XY_M, z_tol=ARRIVAL_Z_TOL_M):
        self.halted = False
        self.lookahead_m = float(lookahead_m)
        self.seq_base = int(seq_base)
        self.cancel_seq = None
        self.last_waypoint = None
        self.last_waypoint_at = None
        self.waypoint_count = 0
        self.pending = None          # 自己发起、还没等到 ack 的那一次取消
        # 到达判据自己算，不转发上游 `/far_reach_goal_status`：上游只比 `converge_dist`
        # （3D 范数，仓外 config/default.yaml 为 0.5），既没有 `z_tol` 也没有可调的 r_xy，
        # 验收 3 的「偏置标定写回 z_tol」在那边无处落账。
        self.r_xy = float(r_xy)
        self.z_tol = float(z_tol)
        self.target = None           # 操作员目标（3D 元组），新 goal 才换
        self.arrived = False         # 上一次的判定结果，用于「只在变化时发」
        # 上游自报到达的旁证读数。**只作附加字段带走，不参与 ok/reason**：
        # `/global_path/status` 是 BT `WaitForPath` 的判据（`bt_console.cpp:315-332`），
        # 拿「还没到」去写 `ok=false`，等于拿到达语义改写路径语义。
        self.far_reach = None
        # ★原子任务身份（2026-11-26 真人裁决③）★
        # 本适配器**不造 mission**：身份只来自 BT 的 `goal_task`（原子消息）。
        # 它只负责把 FAR 的航点"挂到当前 mission 上"，并决定要不要推进 path_revision。
        self.mission = None              # {mission_id, path_revision, waypoint_index, execution_generation}
        self.mission_switched_at = None  # 墙钟：用于"晚到旧航点"保护窗
        self.waypoint_key = None         # 量化后的有效航点（日志/读数用）
        self.waypoint_point = None       # 上一次"有效"航点的原始坐标（判距离用）
        self.path_revision = 0
        self.waypoint_index = 0
        # 读数：任务帧发布数 / 有效航点变化次数 / 丢弃的晚到航点数 / 失效广播次数
        self.task_publishes = 0
        self.effective_changes = 0
        self.stale_waypoints = 0
        self.invalidations = 0
        self.identity_source = None      # 'goal_task'（权威）| 'goal_meta'（兜底）
        self.legacy_identity_ignored = 0

    # ---- 生命周期 ----
    def next_cancel_seq(self):
        """单调递增，起点 `seq_base`。"""
        self.cancel_seq = self.seq_base if self.cancel_seq is None else self.cancel_seq + 1
        return self.cancel_seq

    def on_goal(self, goal=None):
        """新 goal：解除 halted 并清掉残留航点记录。返回 True 表示这次是从 halted 恢复。

        同时清掉待确认的取消记录：新 goal 意味着操作员重新下达任务、人已接管，
        此时再报「上一轮取消没被 ack」只会在正常行驶里插一条误导性故障。

        `goal` 给定时记为到达判据的目标并**把 arrived 归零**——上一任务留下的 true
        若不清，BT 的 `arrival_signal` 会在新任务一开始就成立（`bt_console.cpp:1065-1070`
        的 `arrivalCallback` 是直接赋值，不是置位），`WaitArrival` 立刻判到达。
        """
        was_halted = self.halted
        self.halted = False
        self.last_waypoint = None
        self.last_waypoint_at = None
        self.pending = None
        self.target = None if goal is None else _xyz(goal)
        self.arrived = False
        return was_halted

    def poll_arrival(self, pose):
        """按当前位姿判到达，返回 `(frame, changed)`。

        `frame` 为 `True`/`False`，`None` 表示「还没目标或没有位姿，这一轮不该发信号」——
        发 `False` 会把 BT 可能已经收到的 `true` 清掉，没有目标时不该有这种副作用。
        `changed` 为真才发布：5 Hz 刷同一个值会把 BT 的事件日志淹掉。
        """
        if self.target is None or pose is None or self.halted:
            return None, False
        arrived = is_arrived(pose, self.target, self.r_xy, self.z_tol)
        changed = arrived != self.arrived
        self.arrived = arrived
        return arrived, changed

    def on_cancel(self, seq=None):
        """登记取消：置 halted、记住 seq。不传 seq 就自己取下一个。"""
        seq = self.next_cancel_seq() if seq is None else int(seq)
        self.cancel_seq = seq
        self.halted = True
        self.last_waypoint = None
        self.last_waypoint_at = None
        self.arrived = False
        return seq

    def on_ack(self, text):
        """读 cancel_ack。只认与本次取消同 seq 的终态，其余（含别的 seq）返回 None。"""
        parsed = parse_ack(text)
        if parsed is None or self.cancel_seq is None:
            return None
        seq, outcome = parsed
        if seq != self.cancel_seq:
            return None
        return seq, outcome

    def request_cancel(self, now=None, reason='far_fault'):
        """适配器**自己**发起取消（R6② 的故障停止手段）。返回 `(seq, 帧文本)`。

        ★ 只在自身判出故障时用（航点超时），不回声外部取消 ★
        `/v8/local/cancel` 已经有 BT 这个写者；回声会造成同一 seq 两次发布，
        而且会把「谁停的」这件事搅浑，事后查日志分不出是 BT 停的还是适配器停的。
        """
        seq = self.on_cancel()
        self.pending = {'seq': seq, 'at': now, 'acked': False, 'reported': False}
        return seq, cancel_text(seq, 'all', reason)

    def note_ack(self, text):
        """记 ack：先按本实例的待确认 seq 记账，再走 on_ack 的语义。"""
        parsed = parse_ack(text)
        if parsed is not None and self.pending is not None:
            if parsed[0] == self.pending['seq']:
                self.pending['acked'] = True
        return self.on_ack(text)

    def ack_timeout(self, now, timeout_s):
        """自己发的取消迟迟没有 ack ⇒ 报 `E_FAR_CANCEL_UNACKED`，只报一次。

        ★ 不写任何速度话题 ★ 车能不能停取决于 cancel 有没有被局部规划器接受；
        适配器若自己往 /cmd_vel_raw 或 /cmd_vel 补零，就越过了安全 mux 的单写者约束。
        所以 ack 不来时只能高声上报，不能自造运动指令。
        """
        pending = self.pending
        if pending is None or pending['acked'] or pending['reported']:
            return None
        if now is None or now - pending['at'] < float(timeout_s):
            return None
        pending['reported'] = True
        return status_json(False, 0, 'E_FAR_CANCEL_UNACKED', seq=pending['seq'])

    # ---- 任务身份（原子契约） ----
    @staticmethod
    def _key(waypoint, quantum=WAYPOINT_QUANTUM_M):
        """航点的"有效身份"：按 `quantum` 量化。厘米级抖动不该被当成新航点。"""
        q = max(float(quantum), 1e-6)
        x, y, z = _xyz(waypoint)
        return (round(x / q), round(y / q), round(z / q))

    def on_task_identity(self, payload, now=None):
        """接 BT 的原子任务身份，返回 `'new' | 'same' | 'stale' | 'ignored'`。

        * `new`  —— 换 mission：清掉上一目标的航点缓存与版本号，开保护窗。
        * `same` —— 同一 mission：只更新执行代次（≥ 才收），**不重建任务**。
        * `stale`—— 旧 mission 晚到（编号更小）：直接忽略，绝不把旧航点标成新任务。
        """
        if not isinstance(payload, dict) or 'mission_id' not in payload:
            return 'ignored'
        try:
            mission = int(payload['mission_id'])
            generation = int(payload.get('execution_generation', 0))
        except (TypeError, ValueError):
            return 'ignored'
        now = 0.0 if now is None else float(now)
        current = self.mission
        if current is not None:
            if mission < int(current['mission_id']):
                return 'stale'
            if mission == int(current['mission_id']):
                if generation > int(current.get('execution_generation', 0)):
                    current['execution_generation'] = generation
                return 'same'
        self.mission = {'mission_id': mission,
                        'path_revision': 0,
                        'waypoint_index': 0,
                        'execution_generation': generation}
        self.path_revision = 0
        self.waypoint_index = 0
        self.waypoint_key = None
        self.waypoint_point = None
        self.mission_switched_at = now
        return 'new'

    def task_message(self, pose, waypoint, now=None):
        """把一次航点事件变成**原子任务帧**（dict）；不该发时返回 `None`。

        语义（真人裁决③）：位姿变化**不算**新任务；只有"有效航点"变化才推进 `path_revision`。
        同一 revision 的重复发布是**几何刷新**，下游据此更新首点而**不得重置**执行状态。

        返回 `None` 的三种情况都会计数，便于事后区分"没发"和"发了被丢"：
          · `E_FAR_HALTED`（取消后残留航点）——不计；
          · 没有身份（BT 还没下发 mission）——不计；
          · mission 切换保护窗内的航点（上一目标在途残留）——计入 `stale_waypoints`。
        """
        if pose is None or waypoint is None or self.mission is None or self.halted:
            return None
        now = 0.0 if now is None else float(now)
        if (self.mission_switched_at is not None
                and now - self.mission_switched_at < STALE_WAYPOINT_GUARD_S):
            self.stale_waypoints += 1
            return None
        # ★按**距离**判"是不是同一个航点"，不要用量化分桶★：
        # 分桶在桶边界上会把 ±1 cm 的抖动判成"换了航点"（4.03 与 4.00 分属两桶），
        # 于是位姿稍动就推进一个版本，等于把抖动变成新任务。
        point = _xyz(waypoint)
        key = self._key(point)
        fresh = (self.waypoint_point is None
                 or math.dist(self.waypoint_point, point) > WAYPOINT_QUANTUM_M)
        if fresh:
            self.waypoint_point = point
            self.waypoint_key = key
            self.path_revision += 1
            self.waypoint_index += 1
            self.effective_changes += 1
        payload = {'mission_id': int(self.mission['mission_id']),
                   'path_revision': int(self.path_revision),
                   'waypoint_index': int(self.waypoint_index),
                   'execution_generation': int(self.mission['execution_generation']),
                   'poses': [list(_xyz(pose)), list(_xyz(waypoint))],
                   'source': 'far'}
        self.task_publishes += 1
        return payload

    def invalidate(self, reason, now=None):
        """★失效广播★：路径不可用时发一条**必定不被去重**的任务帧。

        为什么要单列：`path_revision` 只在航点变化时动，若失效也走同一个门，
        下游会把"路径失效"当成重复帧丢掉——安全信号绝不能靠"内容有无变化"来决定是否传达。
        所以失效帧推进 revision 并带 `invalidated` 字段，还带 `poses: []`（清空活动路径）。
        """
        if self.mission is None:
            return None
        self.path_revision += 1
        self.invalidations += 1
        return {'mission_id': int(self.mission['mission_id']),
                'path_revision': int(self.path_revision),
                'waypoint_index': int(self.waypoint_index),
                'execution_generation': int(self.mission['execution_generation']),
                'poses': [],
                'invalidated': str(reason)[:80],
                'source': 'far'}

    # ---- 航点 ----
    def on_waypoint(self, pose, waypoint, now=None):
        """返回 `(path_points, gait_points, status_text)`。

        两条路径**在意料分歧时给出不同点数**，这是本方法返回两个列表的原因：

        * `path_points` —— 恒为 `[位姿, 航点]` 两点，发 `/global_path`。目标语义只认那两个。
        * `gait_points` —— 航点近于 `lookahead_m` 时多补一个延长点，发 `/v8/gait_guard/path`。

        缺输入或处于 halted 时**只出状态、两条都不出**。
        """
        if pose is None:
            return [], [], status_json(False, 0, 'E_FAR_NO_POSE')
        if waypoint is None:
            return [], [], status_json(False, 0, 'E_FAR_NO_WAYPOINT')
        if self.halted:
            return [], [], status_json(False, 0, 'E_FAR_HALTED', seq=self.cancel_seq)
        path_points = two_point_path(pose, waypoint)
        gait_points = gait_path(pose, waypoint, self.lookahead_m)
        self.last_waypoint = _xyz(waypoint)
        self.last_waypoint_at = now
        self.waypoint_count += 1
        # status 的 `points` 报主路径（`/global_path`）帧数，步态侧单独记一个字段：
        # 两个数混用一个字段，事后查日志就分不清「谁少了一点」。
        return path_points, gait_points, status_json(True, len(path_points), 'ok',
                                                     gait_points=len(gait_points),
                                                     far_reach=self.far_reach)

    def stale(self, now, timeout_s):
        """航点超时。正常返回 None，超时返回 `E_FAR_WAYPOINT_STALE` 状态帧。

        FAR 是事件驱动的：图上没有新事件就不发 `/way_point`，所以这个超时门不能设太紧，
        阈值取「实测最长间隔的 2 倍」（B2 会记读数）。
        """
        if self.last_waypoint_at is None or self.halted:
            return None
        if (now - self.last_waypoint_at) < float(timeout_s):
            return None
        return status_json(False, 0, 'E_FAR_WAYPOINT_STALE',
                           since_s=round(float(now - self.last_waypoint_at), 3))


def main():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PointStamped, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool, String
    import sensor_msgs.point_cloud2 as pc2

    rospy.init_node('v8_global_far')
    frame = rospy.get_param('~world_frame', 'map')
    cloud_topic = rospy.get_param('~cloud_topic', '/rslidar_points')
    scan_in = rospy.get_param('~scan_in_topic', '/rslidar_points')
    scan_out = rospy.get_param('~scan_topic', '/scan_cloud')
    pose_topic = rospy.get_param('~start_topic', '/v8/local/pose')
    goal_topic = rospy.get_param('~goal_topic', '/v8/global/goal')
    terrain_topic = rospy.get_param('~terrain_topic', '/terrain_cloud')
    odom_topic = rospy.get_param('~odom_topic', '/odom_world')
    far_goal_topic = rospy.get_param('~far_goal_topic', '/goal_point')
    waypoint_topic = rospy.get_param('~waypoint_topic', '/way_point')
    reach_topic = rospy.get_param('~reach_topic', '/far_reach_goal_status')
    # ★原子任务通道（裁决③/⑥）★：身份只从 BT 的 goal_task 来；路径以原子帧下发。
    task_topic = rospy.get_param('~task_topic', '/v8/bt/goal_task')
    legacy_task_topic = rospy.get_param('~legacy_task_topic', '/v8/bt/goal_meta')
    path_task_topic = rospy.get_param('~path_task_topic', '/v8/3d/global/path_task')
    refresh_hz = float(rospy.get_param('~path_task_refresh_hz', 2.5))
    path_topic = rospy.get_param('~path_topic', '/global_path')
    gait_path_topic = rospy.get_param('~gait_path_topic', '/v8/gait_guard/path')
    status_topic = rospy.get_param('~status_topic', '/global_path/status')
    cancel_topic = rospy.get_param('~cancel_topic', '/v8/local/cancel')
    ack_topic = rospy.get_param('~cancel_ack_topic', '/v8/local/cancel_ack')
    timeout_s = float(rospy.get_param('~waypoint_timeout_s', 10.0))
    cancel_ack_timeout_s = float(rospy.get_param('~cancel_ack_timeout', 2.0))
    lookahead_m = float(rospy.get_param('~lookahead_m', LOOKAHEAD_M))
    terrain_range_m = float(rospy.get_param('~terrain_range_m', 15.0))
    terrain_rate_hz = float(rospy.get_param('~terrain_rate_hz', 2.5))
    terrain_flatten = bool(rospy.get_param('~terrain_flatten', True))
    # ★已知地面高度（可选）★：默认 None = 沿用"逐格最小 z 估地面"。
    # 为什么需要它：逐格最低点会把**只有墙面、没有地面回波**的格子判成"地面=墙面"，
    # 于是该格所有点高度 h≈0 ⇒ 被上游判成 free（`utility.cpp` 用 intensity<terrain_free_Z 判可通行）。
    # 平地世界（地面高度已知=0）应直接给 h，而不是让每格自己猜。实测 m1 平地：
    # 不给该参数时 /terrain_cloud 的 z 中位数 1.55 m（墙面低点），free 占 91%，FAR 建不出可见性图。
    _ground_z = rospy.get_param('~terrain_ground_z', None)
    terrain_ground_z = None if _ground_z is None else float(_ground_z)
    tf_timeout_s = float(rospy.get_param('~tf_timeout_s', 1.0))
    # 到达信号：给 BT 的 arrival_topic 用（std_msgs/Bool）。空则不发，BT 保持自己的 xy 判据。
    arrival_topic = rospy.get_param('~arrival_topic', '')
    arrival_r_xy = float(rospy.get_param('~arrival_r_xy', ARRIVAL_R_XY_M))
    arrival_z_tol = float(rospy.get_param('~arrival_z_tol', ARRIVAL_Z_TOL_M))

    state = AdapterState(lookahead_m=lookahead_m, r_xy=arrival_r_xy, z_tol=arrival_z_tol)
    pose = {'value': None}
    gate = {'next_terrain': 0.0}

    terrain_pub = rospy.Publisher(terrain_topic, PointCloud2, queue_size=1)
    scan_pub = rospy.Publisher(scan_out, PointCloud2, queue_size=1)
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=2)
    far_goal_pub = rospy.Publisher(far_goal_topic, PointStamped, queue_size=1)
    # 两条路径话题都**不 latch**：路径是「这一刻的位姿 + 这一刻的航点」，
    # 一旦 latched，后加入的订阅者会先吃一条过期路径，把车往已经走过的地方带。
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=False)
    gait_path_pub = rospy.Publisher(gait_path_topic, Path, queue_size=1, latch=False)
    # 原子路径帧也**不 latch**：它是"这一刻的位姿 + 这一刻的航点"，latch 会把过期首点喂给后来者。
    path_task_pub = rospy.Publisher(path_task_topic, String, queue_size=5, latch=False)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)
    cancel_pub = rospy.Publisher(cancel_topic, String, queue_size=1)
    # 到达信号**要 latch**：BT 可能比本节点晚启动（`bt_console` 在树加载后才订阅），
    # 一次性发布不 latch 就等于没发。与 `scan_bt_bridge.py` 的 `/lite3/scan/arrived` 同做法。
    arrival_pub = (rospy.Publisher(arrival_topic, Bool, queue_size=1, latch=True)
                   if arrival_topic else None)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

    def task_readings():
        """原子任务侧读数（裁决③要求的六项里属于本层的那几项）。"""
        mission = state.mission or {}
        return {'mission_id': mission.get('mission_id'),
                'path_revision': state.path_revision,
                'waypoint_index': state.waypoint_index,
                'execution_generation': mission.get('execution_generation'),
                'task_publishes': state.task_publishes,
                'effective_changes': state.effective_changes,
                'stale_waypoints': state.stale_waypoints,
                'invalidations': state.invalidations,
                'identity_source': state.identity_source,
                'legacy_identity_ignored': state.legacy_identity_ignored}

    def publish_status(text):
        status_pub.publish(String(data=text))

    def to_path(points, stamp):
        """把三元组列表封成 `Path`。两条话题共用这个封装，保证同帧、同戳、首点逐字相同。

        首点必须与当前位姿完全一致：`gait_guard` 与 `path_goal` 都拿 `poses[0]` 当起点，
        起点和位姿差一点，就等于凭空给车加了一个跳变。
        """
        path = Path()
        path.header.frame_id, path.header.stamp = frame, stamp
        for x, y, z in points:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose.position.x, stamped.pose.position.y = x, y
            stamped.pose.position.z = z
            stamped.pose.orientation.w = 1.
            path.poses.append(stamped)
        return path

    def to_map(points, source_frame, stamp):
        """把点转到 `frame`。只做纯平移近似会在 1 m 外带进 >0.2 m 的 h 误差，必须走真 tf。

        先按点云戳查，查不到就退到「最新可用变换」：仿真里点云戳常比 tf 缓存起点早几十毫秒
        （Lookup would require extrapolation into the past），而 20 Hz 的 tf 与 0.34 m 安装偏移
        在车体接近静止时用最新变换带来的误差远小于 0.2 m 的 h 判据。
        """
        if source_frame == frame:
            return points
        try:
            transform = tf_buffer.lookup_transform(frame, source_frame, stamp,
                                                   rospy.Duration(tf_timeout_s))
        except Exception:
            transform = tf_buffer.lookup_transform(frame, source_frame, rospy.Time(0),
                                                   rospy.Duration(tf_timeout_s))
        t, q = transform.transform.translation, transform.transform.rotation
        return apply_transform(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def on_cloud(message):
        # 节流：FAR 主循环 2.5 Hz，点云 10 Hz。不节流的话 Python 侧转换会把回调堵死，
        # 表现是 /terrain_cloud 频率掉到 1 Hz 以下、FAR 的图迟迟不收敛。
        now = rospy.get_time()
        if now < gate['next_terrain']:
            return
        gate['next_terrain'] = now + (1.0 / terrain_rate_hz if terrain_rate_hz > 0 else 0.0)
        try:
            points = [(p[0], p[1], p[2]) for p in pc2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)]
            points = to_map(points, message.header.frame_id, message.header.stamp)  # ★ 先转 map 系再算 h ★
            # 裁剪以车为心，不以地图原点为心：上游 terrain_range 是「看多远」，
            # 拿地图原点当心会在远离原点时把近处全裁掉，FAR 收到一张空地形。
            origin = pose['value']
            if origin is not None and terrain_range_m > 0:
                ox, oy = origin[0], origin[1]
                points = [p for p in points
                          if math.hypot(p[0] - ox, p[1] - oy) <= terrain_range_m]
            levelled = terrain_cloud(points, h=terrain_ground_z, flatten=terrain_flatten)
        except Exception as error:                     # tf 未就绪、点云字段缺失都走这里
            publish_status(status_json(False, 0, 'E_FAR_TERRAIN', detail=str(error)[:120]))
            return
        header = message.header
        header.frame_id = frame
        terrain_pub.publish(pc2.create_cloud(header, fields, levelled))

    def on_scan(message):
        # 直通，不转帧：上游 ScanCallBack 在 is_static_env=true 时第一句就 return
        # （far_planner.cpp:605），这条链路在当前配置下根本不消费 /scan_cloud。
        scan_pub.publish(message)

    def on_pose(message):
        p = message.pose.position
        pose['value'] = (p.x, p.y, p.z)
        odom = Odometry()
        odom.header = message.header
        odom.pose.pose = message.pose
        odom_pub.publish(odom)

    def on_goal(message):
        """操作员目标：降维成 PointStamped 给 FAR，解除 halted，并重置到达判据。

        z 原样带过——上游 `is_multi_layer=true` 时才不会把它压回地面高度。

        新目标当场发一帧 `arrived=false`：上一任务若已判到达，latch 上留着 true，
        BT 再来一次 `WaitArrival` 会在第一拍就成立。`bt_console.cpp:1065-1070` 的
        `arrivalCallback` 是直接赋值（`arrival_signal = msg->data`），回 false 是安全的。
        """
        p = message.pose.position
        state.on_goal((p.x, p.y, p.z))
        if arrival_pub is not None:
            arrival_pub.publish(Bool(data=False))
        point = PointStamped()
        point.header = message.header
        point.point.x, point.point.y, point.point.z = p.x, p.y, p.z
        far_goal_pub.publish(point)
        # 占位帧用 BT 认的瞬态名（见 `BT_TRANSIENT_REASONS` 上方的注释），
        # 否则 `WaitForPath` 会在暖机这一拍判 FAILURE。
        publish_status(status_json(False, 0, WARMUP_PLAN_REASON, detail='E_FAR_WARMUP'))

    def publish_task(pose_value, waypoint, now=None):
        """发一帧原子任务。返回是否真的发了（便于测试与读数）。"""
        payload = state.task_message(pose_value, waypoint, now=now)
        if payload is None:
            return False
        path_task_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        return True

    def on_task_identity(message, source='goal_task'):
        """BT 的任务身份。★单一权威输入★：`goal_task`（身份+目标同一条消息）。

        `goal_meta` 是 legacy 旁路：只在**还没见过任何 goal_task** 时用来兜底，
        一旦原子通道建立就永久让位（计数 `legacy_identity_ignored`）——
        否则"目标来自 legacy goal、身份来自 goal_meta"这种两条消息拼出来的任务，
        正是跨话题不保序时会把 A 的身份配到 B 的目标上的那类错误。
        """
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if source != 'goal_task' and state.identity_source == 'goal_task':
            state.legacy_identity_ignored += 1
            return
        result = state.on_task_identity(payload, now=rospy.get_time())
        if result in ('new', 'same'):
            state.identity_source = source
        if result == 'stale':
            state.stale_waypoints += 1

    def on_task_goal_meta(message):
        on_task_identity(message, source='goal_meta')

    def on_waypoint(message):
        point = message.point
        path_points, gait_points, text = state.on_waypoint(
            pose['value'], (point.x, point.y, point.z), now=rospy.get_time())
        publish_status(text)
        if not path_points:
            return
        stamp = rospy.Time.now()
        path_pub.publish(to_path(path_points, stamp))
        # ★必须把 now 传进去★：`task_message` 的默认 now=0.0，而保护窗判据是
        # `now - mission_switched_at < 0.3`；0 减一个正值恒为负 ⇒ 每一条航点都被当"晚到"丢掉，
        # 只剩主循环那 2.5 Hz 的几何刷新在发（实测 path_task 里 revision 一直停在 1）。
        publish_task(pose['value'], (point.x, point.y, point.z), now=rospy.get_time())
        # 步态路径另发一条：`/global_path` 的末点必须就是 FAR 航点本身，
        # 前视延长点只喂 `gait_guard`。远航点时两者点数相同，近航点时这里多一个点。
        gait_path_pub.publish(to_path(gait_points, stamp))

    def on_reach(message):
        """上游自报到达：**只记旁证，不写路径状态、不转发到达信号**。

        上游的判据是 `(odom - goal).norm() < converge_dist`（3D 范数，仓外
        `config/default.yaml` 为 0.5，`matterport.yaml` 为 0.25）。它比本适配器的
        `hypot(0.25, 0.30)=0.39` 更松，且没有可调的 `z_tol`——两套门限同时接到
        `/v8/bt/arrived` 上，谁松谁生效，等于没有 z_tol。所以到达由
        `state.poll_arrival` 一处判定，上游这条只留作旁证读数。

        ★ 也不要把它写到 `/global_path/status` ★ 那条话题是 BT `WaitForPath` 的判据：
        `ok=false` 配上不在白名单的 reason 会让它当场判失败（`bt_console.cpp:324-332`）。
        上游「还没到」是常态，与「这条路能不能用」无关。实测 BT 起来 2.56 s 就吃过一条
        `[bt] global path refused: E_FAR_NOT_REACHED`
        （`note/进度/v8/M5/26-11-22/evidence/b5_arrival2/budget3/launch.log`）。
        现在它只进 `far_reach` 字段，随下一帧路径状态带走。
        """
        state.far_reach = bool(message.data)

    def on_cancel(message):
        """外部取消（BT 是既有写者）：只让自己停，不回声。

        回声会让同一个 seq 出现两次发布，事后查日志分不出「谁停的」。
        """
        parts = message.data.split(':', 2)
        if len(parts) < 2 or not parts[0].isdigit() or parts[1] not in ('current', 'all'):
            return
        seq = state.on_cancel(int(parts[0]))
        publish_status(status_json(False, 0, 'E_FAR_CANCEL_RECEIVED', seq=seq))

    def on_ack(message):
        confirmed = state.note_ack(message.data)
        if confirmed is None:
            return
        seq, outcome = confirmed
        publish_status(status_json(outcome == 'cancelled', 0,
                                   'ok' if outcome == 'cancelled' else 'E_FAR_CANCEL_' + outcome.upper(),
                                   seq=seq))

    rospy.Subscriber(cloud_topic, PointCloud2, on_cloud, queue_size=1)
    rospy.Subscriber(scan_in, PointCloud2, on_scan, queue_size=1)
    rospy.Subscriber(pose_topic, PoseStamped, on_pose, queue_size=2)
    rospy.Subscriber(goal_topic, PoseStamped, on_goal, queue_size=1)
    rospy.Subscriber(task_topic, String, on_task_identity, queue_size=5)
    rospy.Subscriber(legacy_task_topic, String, on_task_goal_meta, queue_size=5)
    rospy.Subscriber(waypoint_topic, PointStamped, on_waypoint, queue_size=1)
    rospy.Subscriber(reach_topic, Bool, on_reach, queue_size=1)
    rospy.Subscriber(cancel_topic, String, on_cancel, queue_size=1)
    rospy.Subscriber(ack_topic, String, on_ack, queue_size=1)

    publish_status(status_json(True, 0, 'ok', waiting=True))
    rate = rospy.Rate(5.0)
    refresh = {'at': -1e9}
    while not rospy.is_shutdown():
        now = rospy.get_time()
        # ★同版本几何刷新★：两点路径的首点必须跟着位姿走（FAR 的输入是实时位姿），
        # 但**不许**推进 path_revision —— 位姿变化不是新任务。下游据此只换首点、不重置执行。
        if (refresh_hz > 0 and state.mission is not None and state.last_waypoint is not None
                and pose['value'] is not None and now - refresh['at'] >= 1.0 / refresh_hz):
            refresh['at'] = now
            publish_task(pose['value'], state.last_waypoint, now=now)
        stale_status = state.stale(now, timeout_s)
        if stale_status is not None:
            publish_status(stale_status)
            # 航点断了就自己发一次取消（故障停止手段），并记住 seq 等 ack
            _seq, frame = state.request_cancel(now, 'far_waypoint_stale')
            cancel_pub.publish(String(data=frame))
            # ★失效广播：单独一帧、revision 前进、poses 清空★（安全信号不参与去重/限频）
            invalidation = state.invalidate('far_waypoint_stale')
            if invalidation is not None:
                path_task_pub.publish(String(data=json.dumps(invalidation, ensure_ascii=False)))
            state.last_waypoint_at = now            # 同一轮故障只触发一次
        unacked = state.ack_timeout(now, cancel_ack_timeout_s)
        if unacked is not None:
            publish_status(unacked)
        # 到达判定放在本节点：5 Hz 比上游 2.5 Hz 更密，且只在结果翻转时发一帧
        arrived, changed = state.poll_arrival(pose['value'])
        if changed and arrived is not None and arrival_pub is not None:
            arrival_pub.publish(Bool(data=arrived))
            # ★ `ok` 不能因为「还没到达」变假 ★ `/global_path/status` 是 BT `WaitForPath`
            #   的判据（`bt_console.cpp:315-332`），status=false 配一个不在白名单的 reason
            #   会被读成路径失败。「未到达」在行驶中本来就是常态，与路径质量无关；
            #   到达语义只走 `/v8/bt/arrived`，状态帧里留 `arrived` 作读数。
            #   还没出过路径时给瞬态名，避免 `ok=true, points=0` 把 BT 拖到 30 s 超时。
            publish_status(status_json(True, state.waypoint_count,
                                       'ok' if state.waypoint_count > 0 else WARMUP_PLAN_REASON,
                                       arrived=arrived, task=task_readings()))
        rate.sleep()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
