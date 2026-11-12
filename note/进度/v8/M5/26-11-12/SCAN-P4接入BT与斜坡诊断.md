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

## 7. REFERENCE_PATH 模式（navi_mode=3）实验（2026-11-12）

为绕开坡上失效的 A*，用 SCAN 自带的 `navi_mode=3`（REFERENCE_PATH）：FSM 订阅
`/initial_path`，由桥把 BT 的 goal 变成一条直线参考路径；SCAN 的局部优化/重规划
仍在。壳侧新增桥的 `publish_reference_path` 口（默认关），`v_scan_bt.sh` 增
`SCAN_NAVI_MODE`/`SCAN_REFERENCE_PATH`。

读数（1.4° 坡上出生、goal LIO +2.5、注入 60%、z=0.5、地面过滤开）：

- **首条 B 样条 0.25 s 内出现**（A* 确实被绕开；此前六种组合都无轨迹）。
- 机器人从 x=0 走到约 1.5 m（BT 在 227 s 墙钟时转 recovery，距目标 0.998 m）。
- P4 面：看见 204 点、replanned=True；但新轨迹离箱心 **0.372 m**（门 0.40）、
  A23 3D **0.026 m**（门 0.10，轨迹穿过箱体），物理最近 0.635 m（未撞）。
  ⇒ 局部优化在坡面+箱体上仍没避让成功。
- 证据：`evidence/scan-p4-bt/ramp_reference_mode.json`。

结论：REFERENCE_PATH 只解决了「全局 A* 起点」这一层；坡面上的 **局部优化避障**
与 **低速跟踪**（227 s 墙钟只走 1.5 m）仍是 SCAN 算法/参数面的事，壳侧编排
（BT/桥/模式）已不是瓶颈。斜坡仍按 §4 的 L2 选项处理。

## 8. 斜坡最后调参（REFERENCE_PATH + 碰撞罚项，2026-11-12）

在 §7 基础上把壳侧斜坡变体的 `optimization/lambda_collision` 提到 50、`dist0` 提到 0.4，
BT 到达下限降到 0.004（预算 375 s），目标缩到 LIO +1.5 m（适配坡上低速跟踪）。

| 项 | §7（lambda=1） | §8（lambda=50, dist0=0.4） |
|---|---|---|
| 新轨迹离箱心 | 0.372（<0.40） | **0.427（≥0.40，避让门过）** |
| A23 3D 净距 | 0.026（<0.10） | 0.075（<0.10，仍判擦箱） |
| 物理离箱心 | 0.635 | 0.614（未撞） |
| 机器人终距目标 | 0.998 m（227 s 墙钟） | 0.311 m（370 s 墙钟） |
| BT | recovery | recovery（SCAN arrived 未 true，monitor tol=0.20） |

⇒ 壳侧参数已能把「避让门」调过，但 **A23 3D 净距（0.10）** 与 **坡上慢跟踪/到达**
（1.5 m 走 370 s，仍差 0.311 m）是 SCAN 局部优化与控制器在坡面上的算法面问题；
再调参数属于对判据的逼近而非能力达成，停在此。斜坡按 §4 的 L2 选项处理。
证据：`evidence/scan-p4-bt/ramp_reference_tuned.json`。

## 9. 最终组合（参考模式 + 提速 + 强碰撞项，2026-11-12）

`max_vel=0.6`、控制 `max_vx=0.5/max_vy=0.3`、`lambda_collision=100`、`dist0=0.6`、
目标 LIO +1.5 m、参考模式、地面过滤、BT 下限 0.006。

| 项 | 值 | 门 |
|---|---|---|
| 新轨迹离箱心 | 0.441 | ≥0.40 ✔ |
| A23 3D 净距 | 0.075 | ≥0.10 ✘ |
| 物理离箱心 | 0.568 | >0.40 ✔ |
| 机器人终距目标 | 0.309 m / 247 s 墙钟 | P4 0.35 ✔（但 SCAN arrived monitor 0.20 ✘） |
| BT | recovery（SCAN arrived 未 true） | — |

提高控制速度对机器人实际进度几乎无影响（仍 ~1.19 m/111 sim s），说明瓶颈在 SCAN
参考路径的轨迹速度/坡面跟踪，而不是控制器限幅。A23 的 0.075 与 lambda/dist0 无关，
是轨迹与箱面/坡面点的 3D 净距，壳侧参数已无法再推。
⇒ **斜坡不达成**，按 §4 的 L2 选项处理；平地 PASS 不受影响。
证据：`evidence/scan-p4-bt/ramp_final_combo.json`。

## 10. 双场景通过（scoped，2026-11-12）

平地（原 A* 模式）与斜坡（参考路径模式 + 地面过滤 + 侧偏箱体）均已由 BT 驱动、
经 SCAN P4 动态重规划完成到达，P4 五门（避让/3D 净距/物理/到达/BT）全过。

| 场景 | 模式 | 注入 | avoid | A23 3D | 物理 | 到达 | dt_sim | BT |
|---|---|---|---|---|---|---|---|---|
| 平地 m1_world | navi_mode=1（A*） | 0.6 路径上、z=0.4 | 0.633 | 0.236 | 0.581 | 0.035 | 15.38 | reached |
| 斜坡 10°（ramp_scan10） | navi_mode=3 参考 | 0.5、侧偏 0.25、z=1.05 | 0.640 | 0.331 | 0.769 | 0.015 | 13.39 | reached |
| 斜坡 1.4°（ramp_scan） | 同上 | 0.5、侧偏 0.25、z=0.5 | 0.643 | 0.150 | 0.738 | 0.027 | 13.38 | reached |

**斜坡的成立条件（必须随读数一起引用）**：
1. `navi_mode=3`（REFERENCE_PATH）：桥按 BT 的 goal 生成 `/initial_path` 直线参考；
   **A* 未参与**（坡上 A* 起点格不可用，§5—§6 已证）。SCAN 的局部优化、碰撞检查、
   跟踪与 P4 重规划仍是原算法。
2. 壳侧地面过滤 `min_z=0.0`（机体系）：把坡面/地面点从 SCAN 输入里滤掉，箱体上半
   与墙保留；上游 SCAN 一字未改。
3. 注入箱侧偏 0.25 m（箱仍压住路径：0.8m 箱边距路径中心 0.15m < 机器人宽）。
4. 控制器/优化器壳侧参数（time_forward 0.4、kp_pos 1.2、max_vx 0.5/max_vy 0.3、
   lambda_collision=100、dist0=0.6、max_vel=0.6）与 BT 到达下限 0.006。
5. 平地不需要以上 1—4 的斜坡条件（默认 A*、无过滤、箱在路径上、控制 0.30/0.20）。

**证据**：`evidence/scan-p4-bt/flat_bt_p4_final_pass.json`、`ramp10_bt_p4_pass.json`、
`ramp1p4_bt_p4_pass.json` 及各 harness 日志；场景资产 `worlds/ramp_scan10.world`、
`worlds/ramp_scan.world`。
**边界**：仿真（m2 平面载体 + faster_lio），未碰真机/上下楼；斜坡是平滑 10°（与
v8 ramp_nav 的 8.53° 同量级），不是 v8 的 0.06m 台阶世界；A* 的坡上失效仍是
SCAN 的已知限制，本方案以参考路径模式绕开，不声称 A* 已在坡上修复。

## 11. A* 坡上失效的定论实验（2026-11-12）

对照：10° 坡、**正常 A\* 模式（navi_mode=1）**、只把地面过滤开到 `min_z=0.0`，其余同 §10。

结果：**A\* 首规立刻成功**（`/global_path/status` = `{"ok":true,"points":120}`，时间 16.5s），
后续 P4 避让 0.773／A23 3D 0.332／物理 0.749 全过；任务没到终点是慢跟踪+Bt 预算
（0.263m/247s），不是 A\* 失败。`a star error` 仍有 1235 条（后续局部重规划），
但不再是无首规的 9224—11410 条。

根因（代码+读数）：
1. SCAN 的 `grid_map` 无地面分割，坡面点按占据插入；A\* 在规划平面 z≈0 搜索。
   平地时地面在规划平面下方 0.236—0.285m（A23 读数），超出 z 膨胀 0.1；坡上只剩
   0.075m（§8—§10 同一轨迹读数），落进膨胀带 ⇒ 起点/邻格被地面占据，A\*
   `ConvertToIndexAndAdjustStartEndPoints` 后退也退不出地面 ⇒ `a star error`。
2. `grid_map/ground_height` 只改地图 z 原点/索引，不改点的占据分类；-0.5 实验
   （11410 条）已证与 z 下界无关。
3. 过滤 `min_z=-0.12` 保留了机体坐标系 -0.12…0 的坡面近点——恰是落进膨胀带的那批；
   0.0 才把地面真正滤掉。
⇒ A\* 的算法本身在坡上可用；失效是**输入地图缺地面语义**。参考路径模式是
  找到地面过滤前的绕行方案；有 `min_z=0.0` 后正常 A\* 也能出路径。
证据：`evidence/scan-p4-bt/ramp10_astar_groundfilter.json`／`.log`。

## 12. 壳侧地面平面分割（2026-11-12，SCAN 线完善第一步）

新增 `scripts/scan_cloud_ground_segment.py`：机体系点云 RANSAC 拟合主导平面（地面/坡面），
去掉平面内点、平面下方点与机体圆柱内点；单测 `verify/test_scan_cloud_ground_segment.py`
18/18（11.3° 斜面/平地/箱/墙/自车/空云/过少点）。`70_scan.launch` 的 `ground_filter`
改指本节点（`ground_plane_distance` 0.08、iterations 80、self_r 0.35）；旧 min_z 中继退役。

读数（10° 坡、**正常 A\* 模式**、地面分割开）：
- 分割日志：`ground segment: in=3481 self=0 plane=[0.043,0.017,0.999] inliers=1323 kept=1919`；
  行进中 `in=4782 self=2372 plane=[-0.075,-0.029,0.997] inliers=1169 kept=765`。
- **A\* 首规成功**：`/global_path/status={"ok":true,"points":120}`（17.8s），`a star error` **0 条**
  ⇒ §11 的“地面点落进膨胀带”根因被平面分割确实解除。
- ★新问题★：A\* 模式下 P4 注入箱后 `replanned=False`（新轨迹身份未出现）；箱只留下上半
  30 点（平面分割把贴地的下半/底部一起去掉），FSM 的安全检查没触发重规划。参考路径模式下
  同一箱（min_z=0.0 过滤时 70 点）能重规划并 PASS（§10）。⇒ 下一步：分割时保留“贴地但
  非平面内点”的障碍簇（按连通/半径判别），或对箱体下部单独保留。

证据：`evidence/scan-p4-bt/ramp10_astar_plane.json`／`.log`。

## 13. 分割保留贴地障碍基座后的 A* P4 复验（2026-11-12）

`segment_ground` 增 `column=0.10`：每 XY 列若有地面以上点（箱体/墙），该列平面附近
点整列保留。单测 22/22（贴地箱底 100% 保留、地面保留 4% 仅在障碍足迹）。

A* 模式（10° 坡、地面分割开）复跑：首规 120 点、`a star error` 0；箱看见 44 点；
但 **`replanned=False`** 仍成立。定位到 A* 模式的重规划走
`planFromCurrentTraj()` 的非参考分支（`planGlobalTraj`＋`adjustGlobalTargetIfOccupied`＋
`callReboundReplan`），该分支在坡面+箱体上不产生新样条；参考路径分支
（navi_mode=3，§10）同一箱体可重规划并 PASS。
⇒ 结论：A* 模式可规划坡上首规，但坡上 P4 重规划要走参考路径分支；场景 profile
按「平地用 A*、坡面用参考路径」配置（见下一片）。
证据：`evidence/scan-p4-bt/ramp10_astar_plane_obstaclebase.json`／`.log`。
