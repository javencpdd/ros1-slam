# SCAN 接入 v8 BT 编排（2026-11-12）

> 结论：SCAN 的规划/跟踪链已接进 v8 BT 编排树并由 BT 驱动完成一次活体到达。
> `verify/v_scan_bt.sh` 冷启动全链 PASS（120 点轨迹、reached 86.33 s、终点 0.141 m、
> SCAN arrived=True、teardown 净）。本轮不改 SCAN 上游五包，不改 v8 判据阈值。

## 1. 接入形（谁负责什么）

```
/v8/bt/goal ──[WaitForGoal/RequestPath]──> /scan_planner/goal
                                              │
                                   [scan_planner_node FSM]
                                              │ /planning/bspline
                                   [scan_bt_bridge] ──> /global_path
                                              │        └> /global_path/status
                                   [WaitForPath] ──> FollowPath（只发 goal_request，不重复规划）
/lite3/scan/arrived ──[scan_bt_bridge]──> /v8/bt/arrived ──[WaitArrival]──> Dwell → Stop
```

- BT 仍是唯一目标序列所有者：它决定何时请求路径、何时进入等待到达、dwell 与 Stop；
  SCAN 只负责规划（B 样条）与跟踪（closed_loop_controller），不改上游。
- **到达权来自 SCAN 自己的观测件** `/lite3/scan/arrived`（`scan_arrival_monitor`，
  参考点＝我方请求的 goal），不取 FSM 内部状态、不取 B 样条末点——B 样条末点会被
  `adjustGlobalTargetIfOccupied` 改写，拿它当到达会假绿。
- `scan_bt_bridge.py` 只翻译：`/planning/bspline` → `/global_path`（de Boor 采样 120 点，
  与 `verify/p2_goal_plan.py` 同一套「次数由节点向量反推」规则）＋ `{"ok":true,"points":N}`；
  `/scan_planner/goal` 出现即重置就绪态；`/LIO/odom_vehicle` 顺带映成 `/v8/local/pose`。
- BT 新增 `~arrival_topic`（`std_msgs/Bool`）信号模式：新目标入队时清 false，信号只置 true、
  不会失败目标；`~planner_result_topic`（ack）与它互斥，同时设置则节点拒启。

## 2. 改动面

| 件 | 内容 |
|---|---|
| `ws/src/90_platform/lite3_bt/src/bt_console.cpp` | `~arrival_topic` 模式；新目标清信号；状态 JSON 增 `planner_mode=signal` 与 `arrival_signal` |
| `ws/src/90_platform/lite3_bt/launch/bt_console.launch` | 新增 `arrival_topic` arg（默认空＝pose 模式） |
| `ws/src/90_platform/lite3_bt/README.md` | 三种到达模式（pose/signal/ack）与互斥说明 |
| `ws/src/40_navigation/lite3_scan/scripts/scan_bt_bridge.py` | 契约桥（新建） |
| `ws/src/40_navigation/lite3_scan/launch/scan_bt_nav.launch` | SCAN＋桥＋BT 装配（新建；不含 Gazebo/LIO/mux） |
| `ws/src/40_navigation/lite3_scan/verify/bt_scan_probe.py` | 发 `/v8/bt/goal`、观测 BT/arrived/odom、判 PASS/FAIL/INVALID（新建） |
| `ws/src/40_navigation/lite3_scan/verify/v_scan_bt.sh` | 冷启动全链 harness（新建，含锁与退净） |
| `ws/src/40_navigation/lite3_scan/verify/test_scan_bt_bridge.py` | 桥的纯函数单测 12 项（新建） |

## 3. 活体读数

### 3.1 自动化 harness（`v_scan_bt.sh`，冷启动，2026-11-12 18:54:30—18:56:55）

```json
{"verdict":"PASS","goal":[-0.474,3.598],"path_points":120,"ready_s":0.23,
 "reached_s":86.33,"dist_end_m":0.141,"arrived":true,"outcome":"reached",
 "outcome_seq":1,"planner_mode":"signal",
 "events":[["arrival_reset","",0],["arrival_start","",120],
           ["path_ready","",120],["dwell","reached",120]]}
```

链上节点 7/7（bt_console、scan_bt_bridge、goal_gate、scan_planner_node、
closed_loop_controller、scan_arrival_monitor、safe_cmd_vel_mux）；teardown 后残留进程空、
11313 监听 0。证据：`evidence/scan-bt-run/`（harness.log、probe.json、bt/scan/gazebo/
mapping/mux 日志）。

### 3.2 手工首跑（bag 交叉核对，2026-11-12 18:51—18:53）

- 13 条 B 样条 → 13 条 `/global_path`，每条 120 点；首条折线长 3.511 m。
- BT：`arrival_reset`(68.24) → `arrival_start`(69.74) → `path_ready`(74.24) →
  `dwell/reached`(107.24，signal=True) → `stop`(107.74)；之后 14/14 条状态仍 reached。
- 到达时 odom 距目标 0.144 m；终点 0.116 m；`/lite3/scan/arrived` False→True（107.05）。
- 原始 bag：仓外 `/home/jack/ros1-slam-archive/2026-11-12/scan-bt/manual-run-20261112.bag`。

### 3.3 单测

`test_scan_bt_bridge.py` 12 项 PASS（端点/中点/坏节点向量/路径消息/状态 JSON）；
`test_bt_config.py` 10 项 PASS；桥与启动装配均由 harness 活体覆盖。

## 4. 复跑

```bash
# 容器内（会自起 roscore/Gazebo/LIO/mux/SCAN/BT，需 11313/11345 空闲）
docker exec -it ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash && bash /opt/ros1-slam/ws/src/40_navigation/lite3_scan/verify/v_scan_bt.sh'
```

## 5. 边界与未纳入

- **P4 动态重规划仍是 v7 的未达成项**（上游避让非恒定、撞后 odom 千米级跳变）；
  本切片不重开该判据，也不以它作为「SCAN 接入 BT」的前置。
- 目标固定为 P3 在册的 (-0.474, 3.598)；不同目标/场景多样性未扫。
- 载体是 Gazebo 平面载体 + faster_lio 估计，不是四足接触动力学；未碰真机。
- Stop 的零速发 `/lite3/teleop/app_cmd_vel`（mux priority 3，高于 scan_planner 的 7），
  但 mux 源超时 0.5 s ⇒ 单次 Stop 只覆盖 0.5 s；本次停车最终的保持来自 SCAN 轨迹结束。
- 桥的 `frame=camera_init` 只用于路径消息头；BT 只用折线长度与点数，不消费坐标。
