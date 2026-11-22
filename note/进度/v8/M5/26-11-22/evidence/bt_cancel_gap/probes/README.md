# probes · 8 秒窗探针（`T-20261122-04`）

回答一个问题：**第二写者取消（执行层自诊断，或操作员违规直发 `/v8/local/cancel`）之后，
BT 到底多久才肯接下一个目标。**

## 两件

| 文件 | 作用 |
|---|---|
| `_regoal_second_writer.py` | 探针本体。`--mode direct`（第二写者，期望 FAIL）／`--mode bt`（反控，必须 PASS） |
| `_ack_face_probe.py` | 误触发面探针（T-20261122-04 施工段补）。`--case stray|late|duplicate|idle` 四格，判据取 `/v8/bt/status` 的 `external_cancel_seq` |
| `gap_probe.sh` | 运行器：每轮各起一套独立栈（`mode_sim_bt.launch global_backend:=far arrival_topic:=/v8/bt/arrived`），跑探针，收队内统计 |

用法（在 `ros1-slam-sim` 容器内）：

```bash
cd /opt/ros1-slam/note/进度/v8/M5/26-11-22/evidence/bt_cancel_gap/probes
bash gap_probe.sh /opt/ros1-slam/note/进度/v8/M5/26-11-22/evidence/bt_cancel_gap/probe_run2
# 只跑某几轮：ROUNDS=direct,direct bash gap_probe.sh <out>
```

## 三个判据与窗口

`t0` 发取消、`t0+3.0` 发目标 2、`t0+8.0` 收窗。
J1 `/v8/global/goal` 出现目标 2；J2 `/global_path` 有新时间戳帧；J3 `/v8/bt/status` 的 `goal_seq` 变大。
三条全绿才算「接上了」。

★窗口为什么必须是 8 秒★：到达预算地板是 60 秒（`max(60, L/0.02)`）。旧判据「最终有没有新路径」
等得够久必然为真 ⇒ 同时容纳「立刻接」与「100 秒后才接」，没有分辨力。8 秒小一个量级。

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

## 误触发面四格（`_ack_face_probe.py`）

| `--case` | 喂进去的东西 | 期望 |
|---|---|---|
| `stray` | 有活腿时注入一条谁都没发过的 confirmed 回执 | **会停腿**（刻意 fail-safe；代价是已知过触发） |
| `late` | 真外部停已升格后，同一条回执再送一次 | 不再升格（已记账） |
| `duplicate` | 同一条回执背靠背送 5 次 | 只准升格一次（`external_cancel` 须 =1） |
| `idle` | mission 已收尾、手里没有 RUNNING 的 mission 节点时注入回执 | 只记账不升格 |

判据只用 `external_cancel_seq` 一个字段：升格时写成那条外部号，未升格恒 0。
`external_cancel` 的日志条数由运行器事后统计，作为独立旁证（不依赖探针自报）。

## 三个作废批次（留档，别当读数用）

| 批次 | 作废原因 |
|---|---|
| `probe_fix2` | 旧目标没排上队：`/v8/bt/goal` 订阅队列只有 1，三条背靠背发下去只剩一条（实测 `depth=1`）⇒ 前提不成立 |
| `probe_fix2b` | 判定采样过早：`/v8/bt/status` 只有 2 Hz，派发只要 50 ms ⇒ 把「已接上」读成 False，随后急着重发 |
| `probe_support` | 就绪门对 support 链恒 `NOT_READY`（`terrain_frames=0 obs=0`），门没过就没跑探针 |
