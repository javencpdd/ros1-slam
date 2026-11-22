# probes · 8 秒窗探针（`T-20261122-04`）

回答一个问题：**第二写者取消（执行层自诊断，或操作员违规直发 `/v8/local/cancel`）之后，
BT 到底多久才肯接下一个目标。**

## 两件

| 文件 | 作用 |
|---|---|
| `_regoal_second_writer.py` | 探针本体。`--mode direct`（第二写者，期望 FAIL）／`--mode bt`（反控，必须 PASS） |
| `_ack_face_probe.py` | 误触发面探针。`--case stray|late|duplicate|idle|wire|noack|idle_wire|race` 八格，判据取 `/v8/bt/status` 的多枚字段 |
| `_fresh_path_control.py` | ★对照组★ 同两条目标、同样间隔，但**不发取消**，只量「换目标→路径末端落到目标」要多久。用来把「故障恢复慢」与「上游 FAR 换目标的固有慢」分开 |
| `gap_probe.sh` | 运行器：每轮各起一套独立栈（`mode_sim_bt.launch global_backend:=far arrival_topic:=/v8/bt/arrived`），跑探针，收队内统计 |

用法（在 `ros1-slam-sim` 容器内）：

```bash
cd /opt/ros1-slam/note/进度/v8/M5/26-11-22/evidence/bt_cancel_gap/probes
bash gap_probe.sh /opt/ros1-slam/note/进度/v8/M5/26-11-22/evidence/bt_cancel_gap/probe_run2
# 只跑某几轮：ROUNDS=direct,direct bash gap_probe.sh <out>
```

## 两段判据与窗口（第三版，2026-11-23）

**第一段 · 旧目标不许自己跑**（`t0` 发取消 → `t0+8.0` 收窗）：`SW_OLD_DISPATCHED=False`、
`SW_OLD_GOAL_SEQ` 不变、`SW_QUEUE_DEPTH_END=0`、`PENDING_DROPPED ≥ 1`。四项全绿才叫「停住了」。

★这一段为什么必须是 8 秒★：到达预算地板是 60 秒（`max(60, L/0.02)`）。旧判据「最终有没有新路径」
等得够久必然为真 ⇒ 同时容纳「立刻接」与「100 秒后才接」，没有分辨力。8 秒小一个量级。

**第二段 · 新目标必须真的规划出来并交给局部**（发新目标 → 25 秒路径窗，六项）：

| # | 读数 | 含义 |
|---|---|---|
| ① | `SW_FRESH_STAMP_SEEN` | BT 把新目标写到 `/v8/global/goal` |
| ② | `SW_FRESH_GOAL_SEQ` 变大 | BT 取队，队列没卡死 |
| ③ | `SW_FRESH_CHAIN_GOAL_POINT HIT` | 适配器把新目标转给上游（仅 far 链；support 链 `APPLICABLE=False` 自动跳过） |
| ④ | `SW_FRESH_PATHS ≥ 1` ＋ `PATH_POINTS ≥ 2` | 后端真出了非空路径 |
| ⑤ | `SW_FRESH_PATH_REPLANNED` | 末点与发帧前那条差 > 0.3 m ＝换目标重规划，不是旧路径重放 |
| ⑥ | `SW_FRESH_LOCAL_GOAL` | `/v8/local/goal` 收到的点等于某条新路径末点 |

`SW_FRESH_CONVERGE_S`（末点贴住操作员目标的秒数）**只报不判**：它受上游到达判据与车速支配
（上游 `ProjectNavWaypoint` 会把近目标的航点外推 5 米，`/way_point` 等于目标只在收敛那一拍）。
实测带取消 9.6—24.9 秒，不取消的对照组 12.9—18.0 秒，同分布。详见设计件 §8.12。

## 两个坑（都实测踩过，留档防复发）

1. **队内统计必须在 `cleanup` 之后采**。`roslaunch output="screen"` 走管道，栈活着时 `bt_console`
   的 stdout 还在缓冲里（已知坑 9.9，实测上百秒延迟）。第一版把 `goal queued` / `dequeued` 的
   `grep` 放在 `cleanup` 之前，三轮全读到 **0** —— 那是「没落盘」不是「没发生」。改到 `cleanup`
   之后并 `sleep 1`，同一份日志立刻读到 `goal_queued=2 dequeued=1`。
   ⇒ 凡是「探针之外再 grep 日志」的统计，都要问一句「这会儿它落盘了吗」。
2. **每轮必须各起一套独立栈**。上一轮的取消会把适配器 `halted` 置真、把 BT 的 `cancel_requested`
   与队列都带进状态。串栈的读数没有意义（`b2_probes/run1` 就是这样作废的）。

## 与 `b2_probes/` 的分工

- `b2_probes/_regoal_via_bt.py`：问「走 BT 入口取消后重发能不能被接」（答案能，run4 PASS）。
- 本目录：问「不走 BT 入口时多久才被接」（答案：8 秒窗内接不上，这就是 `T-20261122-03` 登记的缺口）。
- 两个目录都不改 `bt_console.cpp`；本目录的读数全部来自 ROS 话题，不依赖日志。

## 运行器开关（施工段新增）

| 开关 | 作用 |
|---|---|
| `ROUNDS=模式[:旧目标数[:相位]]` | 例 `direct:3:arrival`／`direct:3:pending`。相位要 `pending` 必须旧目标数 ≥ 1 |
| `GLOBAL_BACKEND=support` | 起 support 后端做零回归（默认 `far`） |
| `EXPECT=pass\|fail\|auto` | 探针期望值。修复后用 `pass` 钉住新期望，`auto` 只用于回看修复前假设 |
| `SKIP_READY_GATE=1` | 关掉就绪门。support 链不起 `terrain_frames`／`obs`，门恒判 `NOT_READY`，门读数仍落盘当旁证 |
| `PROBE_SCRIPT=_ack_face_probe.py` ＋ `PROBE_ARGS=...` | 换用误触发面探针，轮次名第一段直接当 `--case` |

## 误触发面八格（`_ack_face_probe.py`，第二施工段重写）

| `--case` | 喂进去的东西 | 期望 |
|---|---|---|
| `stray` | 有活腿时注入一条谁都没发过的 confirmed 回执 | **不停腿**，只记账（`stray_ack_seen` 加一） |
| `late` | 真外部停已升格后，同一条回执再送一次 | 不再升格、不再清队 |
| `duplicate` | 同一条回执背靠背送 5 次 | 只准升格一次（`external_cancel` 须 =1） |
| `idle` | mission 已收尾（`leg_active` 假）后注入回执 | 只记账不升格 |
| `wire` | 有活腿，直发一条外来取消帧 | **停腿**，来源 `external_wire` |
| `noack` | ★无 ACK★ 只发后端回显（`/global_path/status` 带 `seq`） | **停腿**，来源 `external_backend` |
| `idle_wire` | 空闲期发外来取消帧 | 不停腿，只记账 |
| `race` | 队列压 2 条旧目标、到达信号与外来帧只差一拍 | 旧目标不得被派发（两分支都合法，见 `--race-delay`） |

判据取 `/v8/bt/status` 的 `external_cancel_seq`／`cancel_origin`／`leg_active`／
`wire_foreign_seen`／`backend_cancel_seen`／`stray_ack_seen`／`stale_queue_dropped`。
`cancel this node did not issue` 的条数由运行器事后统计，作独立旁证（不依赖探针自报）。

`--race-delay` 只对 `race` 有效：到达信号之后还有 `Dwell` 1.0 s（`navigate_with_planner.xml:22`），
所以 0／0.06／0.15 s 都落在「腿还活着 ⇒ 升格」那一支，约 1.06 s 才落进「腿已收、队未空 ⇒ 只清队」
那一支。**两档结论必须一致**（旧目标不派发、队列清到 0），否则说明某条分支漏了清队。

## 三个作废批次（留档，别当读数用）

| 批次 | 作废原因 |
|---|---|
| `probe_fix2` | 旧目标没排上队：`/v8/bt/goal` 订阅队列只有 1，三条背靠背发下去只剩一条（实测 `depth=1`）⇒ 前提不成立 |
| `probe_fix2b` | 判定采样过早：`/v8/bt/status` 只有 2 Hz，派发只要 50 ms ⇒ 把「已接上」读成 False，随后急着重发 |
| `probe_support` | 就绪门对 support 链恒 `NOT_READY`（`terrain_frames=0 obs=0`），门没过就没跑探针 |
| `probe_ctl1` | 对照组首版没等位姿就发目标：位姿恒 `(0,0)`、机器人一步没动、上游退回静态环境的前方 5 米虚拟点，两条目标都当没收到 |

## 「新目标必须出路径」这条判据的三次修正（都实测踩过）

1. **只看「派发 + `goal_seq` 增长」**（`probe_v3`）：两件都是 50 ms 级事件，探针在 0.1—0.6 秒就判
   PASS，那时后端还没出路径（`SW_FRESH_PATHS` 四轮全 0）。只证明 BT 肯接单。
2. **取「窗内第一帧」判末点**（`probe_path1`）：第一帧末点常是**旧目标**那个航点（新目标写进
   `/v8/global/goal` 后，上游回调还在排队，上一拍的航点先到）⇒ 把「后端还没轮到」误判成
   「后端没重新规划」。改一路收到窗末（`probe_path2`／`probe_path3`），并加 `SW_FRESH_CHAIN`
   逐跳读数（`/goal_point`／`/way_point`／位姿），才看清链上每一跳都通。
3. **以「末点贴住操作员目标」为必过项**（`probe_v4`／`probe_v5`）：这条判据本身放错了位置——
   上游 `ProjectNavWaypoint`（`far_planner.cpp:339-378`）在航点距机器 < `local_planner_range`
   时沿航向把航点**外推 5 米**，`/way_point` 等于目标本身只在 `PlanGlobalPath` 判出收敛那一拍
   （`graph_planner.cpp:163-178`）。所以「末点＝目标」是**到达事件**（受车速与上游到达判据支配），
   不是**规划事件**；对照组不取消也要 12.9—18.0 秒才落到目标上。现判据改看重规划里程碑
   （末点与发帧前那条差 > 0.3 m ＋ 局部目标等于该末点），收敛时长降为读数 `SW_FRESH_CONVERGE_S`。

## 探针自己的四处缺陷（都实测踩过，写下来防下一棒当产品缺陷）

1. **队内统计必须在 `cleanup` 之后采**（见上文「两个坑」）。
2. **判定采样过早**：2 Hz 状态帧追不上 50 ms 的派发。
3. **`noack` 曾把「本轮收件箱为空」当判据**：BT 升格后会用自己的号重发取消，执行层对它回执是正常
   链路 ⇒ 探针把自己要验证的恢复动作判成了缺陷。现判「注入的那个号没有回执」。
4. **`race` 曾用 2 Hz 采样深度当「队列里有 ≥2 条」的前提**：`t1.06` 读到深度 1、实际清了 2 条，
   于是把 FAIL 判在前提上。现判「清掉了 ≥2 条」（由清队动作本身产生，不靠采样）。
5. **链上旁证的下标抓晚了**：`/goal_point` 在派发那一拍就发出，等到「派发窗」结束（0.5—0.6 s）再抓
   下标正好漏掉它，读到 `GOAL_POINTS=[]`、错判成「适配器没转发新目标」。现把下标抓在 `send_goal` 之前。
6. **重发逻辑把「派发成功但路径慢」当成「接单失败」**：旧版无条件重发，实测第二轮那条只在队列里
   排队（`goal queued depth=1` 之后再无 `dequeued`），既修不了路径，还往下一轮塞一条没人执行的目标。
   现只在「没派发」时重发。
7. **对照组没等位姿**（`probe_ctl1`）：位姿恒 `(0,0)`、机器人不动，上游退回静态环境虚拟点，读数作废。
   现先等 `/v8/local/pose` 有值再等 3 秒（与主探针 `GOAL_DELAY_S=3.0` 对齐）。
