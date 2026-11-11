# SCAN P4 接入 BT 编排：平地 PASS，斜坡能力缺口（2026-11-12）

> 结论先行：**平地**上 SCAN 的动态重规划已经通过 BT 编排树跑通（BT 发目标、SCAN
> 重规划、BT 判到达，PASS）。**斜坡**上 SCAN 自身规划不出轨迹（1.4°／3.5° 平滑坡
> 与 10° 台阶坡都试过）——这是 SCAN 的 2.5D 栅格/地面模型能力缺口，不是 BT 接线、
> 也不是 P4 门值问题。斜坡要通过，需要一次新决策（见 §4）。

## 1. 平地：P4 经 BT 的读数（PASS）

链：`m2_gazebo → m2_mapping → 60_safety → 70_scan → scan_bt_nav(BT+桥)`；
目标发 `/v8/bt/goal`，BT 的 RequestPath 直发 `/scan_planner/goal`，到达走
`/lite3/scan/arrived → /v8/bt/arrived`。

| 项 | 值 |
|---|---|
| 目标（LIO 系） | (-2.774, 1.550)，r=3.175 |
| 扰动 | obs_5 搬到首规 60% 处 (-1.675, 1.015)，占据图看见 72 点 |
| 重规划 | 新 B 样条身份（start_time 17.684 ≠ 16.433） |
| 避让／物理 | 轨迹离箱心 0.697 m（门 0.40）；机器人最近 0.714 m |
| A23 穿墙（3D） | 0.285 m（门 0.10；2D 原始 0.104） |
| BT 终态 | **outcome=reached, outcome_seq=1, planner_mode=signal** |
| 到达偏差／dt | 0.119 m（门 0.35）／dt_sim 41.38 s（门 120；墙钟 90.46 s） |
| 世界还原 | restored=True（delta 0.0） |
| 证据 | `evidence/scan-p4-bt/flat_bt_p4_pass.json`／`.log` |

配套脚本：`verify/p4_bt_probe.py`（BT 版 P4 探针）、`v_scan_bt.sh` 的
`SCAN_BT_PROBE=<probe>` 口、`scan_bt_nav.launch` 的 `arrival_speed_floor_mps`。

## 2. 斜坡：三次尝试与读数

| 场景 | 读数 | 结论 |
|---|---|---|
| v8 台阶坡（ramp_nav，0.06m×10） | m2 平面载体爬到第 3 级（x=1.28, z=0.27）后原地不动 | 平面载体爬不动 6cm 台阶（v8 用 kinematic 载体是有原因的） |
| 平滑 10° 坡（v1，坡顶在 x=5.5） | 先用 LIO 坐标当 world 注入，箱子落到世界外 ⇒ 该轮作废；同时暴露 LIO 原点≠world 原点的坐标错位 | 场景坐标必须按机器人出生点对齐 |
| 平滑 3.5°（v3，坡顶在原点，坡上出生，yaw=π） | SCAN 能出轨迹；但机器人 53 sim s 只走 0.14 m；Gazebo 真值卡在 x≈-0.53（平台→坡沿），cmd `/cmd_vel`=0.30 正常；LIO 与真值分叉 | 平面载体 + 坡沿/栅格；机器人过不去 |
| 平滑 3.5°（坡上出生 x=-1） | 45 s 内无首条 B 样条，`/global_path/status` 一直 `E_PLAN_RUNNING` | SCAN 在坡面上规划不出轨迹 |
| 平滑 1.4°（v4，坡上出生 x=-1） | 同上：45 s 无首条 B 样条，`E_PLAN_RUNNING` | 不是坡度阈值，是地面/占据模型本身 |

直接诊断（`evidence/scan-p4-bt/diag_*`）：`/scan_planner/cmd_vel` 与 `/cmd_vel` 都是
0.30 m/s，mux 没有限速；但 `/LIO/odom_vehicle` 与 `/sim/odom_gt` 分叉、Gazebo 真值
不动 ⇒ 先卡物理/坡沿，坡上出生时再卡 SCAN 规划。

**根因判断**：SCAN 的 `grid_map` 把坡面点按当前机体的 z 带记成占据/未知，FSM 的
A* 在坡上找不到起点到目标的自由连通；平台→坡的过渡又有平面载体的接触问题。
v8 的 ramp_nav 用支撑图 + kinematic 载体绕开了这两点，所以 v8 斜坡 PASS；
SCAN 线没有这套。

## 3. 本轮改动面

| 件 | 内容 |
|---|---|
| `verify/p4_bt_probe.py`（新） | BT 版 P4 探针：BT 发目标、动态注入、3D A23、LIO↔world 变换、世界还原断言 |
| `verify/v_scan_bt.sh` | `SCAN_BT_PROBE`／`SCAN_BT_WORLD`／`SCAN_BT_SPAWN_*`／`SCAN_BT_ARRIVAL_FLOOR` 口；仿真 speed_cap 0.5（解 mux 0.1 盲限速） |
| `launch/scan_bt_nav.launch` | 暴露 `arrival_speed_floor_mps`（斜坡低速） |
| `worlds/ramp_scan.world`（新） | SCAN 专用平滑坡（当前 1.4°；含 3.5°/10° 的调参记录在案） |
| `verify/p4_replan.py` | `P4_INJECT_Z` 环境口（坡面注入高度） |

## 4. 斜坡要过，需要一次新决策（L2）

1. **给 SCAN 补地面/坡面语义**：在 `30_3rd` 的 `grid_map`/FSM 里做地面分割或
   支撑面感知（改上游基线，或做壳侧等价件并重新对账）——工程量大、改变被测算法。
2. **混合路线**：SCAN 只在平地/2.5D 域负责规划与 P4；斜坡执行交给 v8 支撑图栈
   （已有 PASS 的 `sim_nav_slope`），P4 动态事件放在坡前平地。这样「P4+BT」在平地
   成立，「斜坡+BT」由 v8 栈成立，但**不是 SCAN 自己爬坡**。
3. **接受 SCAN 斜坡实验性缺口**：把斜坡测试登记为 INVALID/能力缺口，不进 PASS 串；
   SCAN 线只承诺平地。
4. 其它（如换一个有地面语义的局部规划器）需另立卡。

在真人选定前，本卡**不声称**「SCAN P4 平地和斜坡都通过」；平地部分已完成并留证，
斜坡部分按上面 1—3 呈报。

## 5. 追加实验：地图 z 原点下移（2026-11-12）

怀疑 A* 失败是 `grid_map/ground_height=0` 把机器人下方的坡面点压到地图 z 下界外。做法＝
壳侧复制 `advanced_param.xml` 为 `scan_advanced_param_slope.xml`（唯一差异＝
`grid_map/ground_height=-0.5`，上游件未改），`70_scan.launch` 增 `advanced_param` arg，
`v_scan_bt.sh` 增 `SCAN_ADV_PARAM` 口；1.4° 坡上出生复跑。

读数：参数确已生效（`/scan_planner_node/grid_map/ground_height: -0.5`），但 45 s 内仍无首条
B 样条，`scan.log` 里 **`a star error` 11410 条**（`evidence/scan-p4-bt/ramp_ground_height_no_path.json`）。
⇒ 排除「地图 z 下界」假设；A* 的起点/终点仍在坡面/自车的通胀格内（或未知区），需要
改 SCAN 的格语义本身，不是调一个壳侧参数能解。L2 选项不变（见 §4）。

## 6. 追加实验：壳侧地面/自车点云过滤（2026-11-12）

再做一层壳侧预处理：`scan_cloud_ground_filter.py` 把机体系 `z<-0.12` 的地面点与
`r<0.35` 的机体点滤掉，发 `/LIO/clouds_lidar_ground_filtered`；`70_scan.launch`
增 `ground_filter`/`cloud_topic` arg（默认关，斜坡 harness 开）。

读数：过滤节点确已运行（scan.log `ground filter up: /LIO/clouds_lidar -> ...`），
但 1.4° 坡上出生时仍 45 s 无首条 B 样条，`a star error` **9224 条**。
⇒ 排除「点云里地面/自车点」这一层；A* 的失败在围栏/未知区/起点格语义本身，
不是壳侧能滤掉的。斜坡必须改 SCAN 的 2.5D 格/地面语义（或走 §4 的混合路线）。

累计负结果（同一判据下）：10°/3.5°/1.4° 坡、平台/坡上出生、ground_height=-0.5、
地面/自车点过滤——六种组合均无首条 B 样条或卡在坡沿。SCAN 斜坡属能力缺口已可判定。
