# BT 编排回执驱动多目标 mission：浏览器活体实测（2026-11-10 23:28—23:50）

## 目的与边界

验证「浏览器发两个目标 → rosbridge → Nav2 式编排 BT → `/v8/bt/status` 回执 → 控制台 mission 推进到第二个目标」。全程 host-only demo 面：未接真机、未起 formal 链、未动公网发布（真人已明确不做）。作用域：`web`／`scripts/console`／`ws/src/90_platform/lite3_bt`／`note/进度`。

## 环境

| 项 | 读数 |
|---|---|
| 容器 | `ros1-slam-sim-viz`、`ros1-slam-roscore`、`ros1-slam-rosbridge`、`ros1-slam-web` 均 Up |
| rosbridge | `ws://127.0.0.1:9090`，HTTP 200 |
| 控制台 agent | `http://127.0.0.1:8780`，`/api/stacks` 报 `bt_console state=running` |
| BT 栈 | 经 agent `POST /api/stacks/bt_console/{stop,start}` 重启，加载 `NavigateWithPlanner` 新树（重启前是 `NavigateSingleGoal` 旧树，status 无 `current_seq`） |
| 浏览器 | 系统 `google-chrome --headless=new`，Node 22 自带 WebSocket 走 CDP，无需安装 agent-browser |

驱动脚本：`~/.cache/bt_mission_cdp.js`（第一次）、`bt_mission_cdp2.js`（加栈状态刷新）、`bt_mission_cdp3.js`（修复后复验）。Chrome 由脚本内部 spawn／回收，不留在后台。

## 三轮读数

| 轮次 | 结果 | 说明 |
|---|---|---|
| 1 | `任务目标 1/2 失败：发送失败`，两个目标都失败 | `goalOwner()` 判定 3D 写入权不在：页面刚加载时 `stackSnapshot` 尚未就绪。加显式 `#agent-refresh` + 等 `#agent-state` 变「agent 在线」后解决 |
| 2 | 目标 1 成功发出；注入位姿后**不推进**；注入路径就绪后**仍不推进** | 前者符合新判据（3D 不用位姿判到点），后者暴露真缺陷 |
| 3 | 阶段 A 不推进；阶段 B **`["成功","发送中"]`** | 修复后完整通过 |

## ★抓到的真缺陷（静态测不出）★

第 2 轮后读 `/v8/bt/status`（容器内 `rostopic echo`，2 Hz）：

```text
tree=NavigateWithPlanner  goal_seq=1  current_seq=1
path_state=ready  path_points=12  event="stop"   ← 注意
nodes: 全部 IDLE（Sequence 已完成、回到 wait_goal）
```

BT 确实把这个目标跑完了，但 `event` 只保留**最后一个**事件：`reached` 在一个 tick 内被 `dwell`／`stop` 覆盖。前端按 0.5 s 周期订阅，永远采不到 `reached`，`classify()` 判 `non_terminal`，mission 永久停在「发送中」。前端 classify 逻辑本身没错（离线单测全绿），缺的是 BT 侧**终态没有持久化**。

## 修复

1. `ws/src/90_platform/lite3_bt/src/bt_console.cpp`：新增 `goal_outcome`（`""`／`reached`／`failed`）与 `outcome_seq`，`recordOutcomeLocked()` 在 WaitArrival 成功／失败、WaitForPath 终态拒绝／超时、RecoveryStop 时记录，仅在 `WaitForGoal` 出队下一个目标时清空。status JSON 新增 `outcome`、`outcome_seq` 两字段，`event` 保持不变以兼容。
2. `web/assets/console/mission_events.js`：`parseStatus` 读新字段；`classify` **优先用持久 outcome**（`outcome_seq > sentSeq` 且未处理过才终态），无 `outcome` 时回退到旧的 `event` 判定，兼容旧 BT 构建。
3. `scripts/console/test_mission_events.js` 新增 5 条：`outcome=reached` 且 `event=stop` 仍推进、`failed` 判失败、重复只处理一次、旧序号 stale、以及原有 event-only 用例全部保持绿（兼容反证）。

## 第 3 轮验收读数

```text
阶段 A（只注入 /v8/local/pose 到目标 1 坐标）: ["发送中","待发"]   ← 不推进，符合预期
阶段 B（再注入 /global_path/status ok:true points:12）: ["成功","发送中"]

[23:47:07] BT 编排回执 seq=1：BT 判定到点
[23:47:07] 任务目标 1 成功：BT 判定到点 ｜ 耗时 4.7 s
[23:47:08] 任务目标 2/2 发送 (2.01, -0.47)
```

其它验收：node 单测 `ALL PASS (0 failures)`；`check_console.sh` `ALL PASS`；容器内 `catkin_make --pkg lite3_bt` 出现 `Building CXX` 与 `Built target bt_console`（真重编）；新 status 实测含 `"outcome":""`／`"outcome_seq":0`，完成后为 `reached`／1。

## 结论与残留

**浏览器发两个目标、按 BT 编排回执推进第二个目标，已活体验证通过。** 注入的 `/global_path/status` 与 `/v8/local/pose` 是测试夹具，不是真实规划器产出；真实下游规划器（`planner/topic_node.py`）与 executor ack 话题在 demo 栈中未起，故本轮证明的是**回执链与前端推进**，不是规划器算法正确性。formal／EVT／真机边界未越。

下一片候选：lease 发布者校验（11-10 第一片残留），或让 demo 栈真正起下游支持规划器以替换注入夹具。
