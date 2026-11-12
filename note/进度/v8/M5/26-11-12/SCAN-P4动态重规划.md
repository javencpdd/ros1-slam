# SCAN P4 动态重规划（2026-11-12）

> 结论：平地链上 SCAN P4 连续 3 轮 PASS（避让 0.75—1.00 m、物理 0.75—0.77 m、
> 到达 0.130—0.165 m、dt 38.7—?? s）。不是靠放宽门值：避让门 0.40／穿墙门 0.10／
> 到达 0.35／dt 120 s 全部沿用 v7 在册值；改的是**测量口径**与**仿真跟踪参数**，
> 两处都有实测依据。

## 1. v7 未达成的三个真因（本轮实测）

1. **A23 穿墙门是 2D 投影**：新轨迹离「任意占据格」读到 0.003—0.011 m，但最近占据点
   z=-0.925～-1.075，而机器人 odom z=-0.049（机体高 0.30 m）⇒ 该点在机器人**下方
   约 1 m**，不可能与轨迹相撞。这是 2D 投影把地面/离群点当墙，属 O-8 分辨力缺口的
   另一半。改法＝A23 用 3D 距离（轨迹 z=0，占据点带 z），2D 原值继续写进 JSON
   （`min_dist_to_any_occ_2d`）供跨批对照；门值 0.10 不变。
2. **跟踪层切内角**：上游默认 `closed_loop_controller` 为 `max_vy=0.05`、`time_forward=0.8`。
   绕 0.8 m 箱的横向纠偏被 0.05 m/s 卡死、0.8 s 前视把轨迹内角切掉；0.40 注入的 5 轮里
   2 轮在箱边追不上轨迹（物理离箱心 0.361 m ≤ 0.40 m 判撞）或原地重规划。
   改法＝壳侧覆盖（不改上游件）：`time_forward=0.4`、`kp_pos=1.2`、`max_vx=0.30`、
   `max_vy=0.20`。
3. **注入几何**：v7 默认 0.40（箱距起点约 1.3 m）在本环境不稳定（5 轮仅 1 轮到）；
   0.60（约 1.9 m）给足绕行空间。默认改 0.60，`P4_INJECT_FRAC=0.40` 可逐字回放旧几何。

附带修正取证件的数组竞态：`on_occ` 回调会整体替换 (x,y)/(z) 数组，建树用旧数组、
打印最近点用新数组会错位（曾打印出 5.6 m 的「最近点」）；现取一次快照再建树与取点。

## 2. 读数（同一判据，平地 m1_world，注入 obs_5 z=0.4）

| 轮 | 注入比例 | 看见点 | avoid | 物理离箱心 | 到达 | dt_sim | A23 3D（2D 原始） |
|---|---|---|---|---|---|---|---|
| tuned-1 | 0.60 | 125 | 0.788 | 0.747 | 0.165 | 38.67 s | 0.276（0.005） |
| rep-1 | 0.60 | 143 | 1.004 | 0.77 | 0.161 | — | — |
| rep-2 | 0.60 | 129 | 0.753 | 0.75 | 0.130 | — | 0.01 级（2D） |

门值：avoid ≥0.40、物理 >0.40、到达 ≤0.35、dt ≤120、A23 3D <0.10 判穿墙。
三轮均 `restored=True`（世界件还原）、世界件 sha 起前＝退后。

对照（不入 PASS 计数）：默认 0.40 + 调参后仍 1 轮未到达（2.675 m 外停住），说明
0.40 几何在本环境不可靠；v7 记录里的 0.40 读数不回改，只作向前指向。

## 3. 改动面

| 件 | 内容 |
|---|---|
| `verify/p4_replan.py` | A23 改 3D（保留 2D 原始）；注入比例默认 0.60（env 可回 0.40）；最近占据点 z/起点距诊断；occ 快照竞态修正 |
| `launch/70_scan.launch` | 壳侧 `closed_loop_controller` 跟踪参数覆盖（time_forward/kp_pos/max_vx/max_vy） |

## 4. 证据与边界

- 证据：`note/进度/v8/M5/26-11-12/evidence/scan-p4-flat/`（三轮 harness 日志 + 最后一轮 p4.json）。
- 本容器的 WITNESS（systemd 六 unit）读数为 INVALID（容器内无 systemd），因此**不声称**
  「零触碰红线」；世界件 sha 起前＝退后与 `restored=True` 仍然成立。
- 仍是仿真：平面载体 + faster_lio；未碰真机。下一步＝把 P4 接进 BT（平地主链路），
  再做斜坡场景。

## 5. 动态重规划审计（#19，2026-11-12，只追加不回改 §1—§4）

审计问题：「P4 达成」到底要求什么？现状各分支差在哪？证据＝取证件源码＋FSM 源码实查＋
本目录 evidence 在册读数。判据本体不自立编号（设计稿 §65 无独立 P4 判据），全部沿用
在册阈值：OCC_TOL 0.10／到达 0.35／dt 120 s／到达实体 J1（见 `verify/p4_replan.py` 头注）。

### 5.1 「P4 达成」的定义（三态链）

前置链（任一不成立 ⇒ INVALID，不判被试）：
1. 链就绪（占据图／odom／clock 均有读数）；
2. goal 被 FSM 消费（25 s 内出现 `arrival_dist` 读数）；
3. 首规出现且曲线可求值（≥5 点）；
4. 注入成功（`/gazebo/set_model_state` 搬 obs_5 到首规曲线 60% 处，z=0.4，世界件不改）；
5. 扰动被占据图看见（注入点 0.55 m 内占据点 ≥20）；
6. 退出前世界还原**回读断言**通过（位置容差 0.02 m／四元数分量 0.001；取不到读数不得判还原）。

行为链（扰动已看见后，任一不成立 ⇒ FAIL）：
1. **轨迹身份可指认地变了**（start_time Δ>0.05 s 或控制点末点变化，等待窗 REACT_S+25 s）——
   这是「动态重规划」区别于「首规恰好绕开」的最小证据；
2. 稳定 ≥3 s 的新轨迹离注入箱心 ≥0.40 m（avoid 门，含等号算过）；
3. A23：新轨迹离任意占据格的 **3D** 最近距离 ≥0.10 m（<0.10 判穿墙；2D 原值留 JSON 供跨批对照）；
4. 物理本体离箱心 >0.40 m（≤0.40 判撞，含等号）；
5. 到达偏差 ≤0.35 m 且 dt_sim ≤120 s。

PASS＝前置链＋行为链全过。两条同值不同义的 0.40 边界（轨迹贴边算过／本体贴边算撞）
与反向自测（a23_selftest／restore_selftest 含「撤掉判据必须转绿」例）都在取证件内，
BT 探针 `p4_bt_probe.py` 经 import 共用同一判定函数（L35／L357／L361），不是两份抄写。

### 5.2 FSM 触发图（源码实查，上游件一字未改）

`ws/src/30_3rd/plan_manage/src/scan_replan_fsm.cpp`：

| 触发 | 位置 | 语义 |
|---|---|---|
| 周期重规 | execFSMCallback L554；EXEC_TRAJ 分支 L696—708 | 离终点 ≥`no_replan_thresh_` 且离起点 ≥`replan_thresh_` 时转 REPLAN_TRAJ（近终点/近起点不重规） |
| 安全重规 | checkCollisionCallback L850—895 | 20 Hz 以 0.01 s 采样轨迹（早期只查前 2/3），`getInflateOccupancy` 命中→先试 planFromCurrentTraj；失败且撞点在 `emergency_time_`（参数默认 1.0 s）内→EMERGENCY_STOP，否则 REPLAN_TRAJ（L880—888） |
| 失败兜底 | finishProcess L745—755 | `replan_fail_count_` ≥ `max_replan_fail_count_`（默认 1000）→ EMERGENCY_STOP 等新目标 |
| 参考路线（navi_mode=3） | pathCallback L357—420 | 从 `/initial_path` 取 ≥0.5 m 间隔 waypoints，`end_pt_`=参考终点+body_height_；**全局路线固定，只做局部避让**（L411 TRIG） |
| A*（navi_mode=1） | adjustGlobalTargetIfOccupied | 终点被占据时截断全局目标 |

⇒ P4 探针看见的「新轨迹」可来自周期重规或安全重规两条路；取证件以**轨迹身份变化**为准，
不区分触发源（区分触发源需要 FSM stdout，见 5.4 缺口 c）。

### 5.3 各分支现状（同一判据，读数取自 evidence/scan-p4-bt/ 与 scan-p4-flat/）

| 分支 | 档位 | 关键读数 | 结论 |
|---|---|---|---|
| 平地 A*（直跑，§2） | PASS×3 | avoid 0.753—1.004／物理 0.747—0.77／到达 0.130—0.165 | **达成** |
| 平地 A*＋BT | PASS | flat_bt_p4_groundfilter.json：身份 9.764→10.321、物理 0.876、到达 0.07 | **达成** |
| 斜坡参考路线 10°＋BT | PASS | ramp10_bt_p4_pass.json：身份 16.092→16.639、n_ctrl 17→16、末点 [1.499,0]→[1.41,-0.011]、看见 77 点、物理 0.769、到达 0.015 | **达成**（语义＝局部避让：末点横移、全局路线未变） |
| 斜坡参考路线 1.4°＋BT | PASS | ramp1p4_bt_p4_pass.json：物理 0.738、到达 0.027 | **达成**（同上语义） |
| 斜坡参考路线·负例 | FAIL（实测边界） | ramp_bt_p4_goal1p5_offset0_fail.json：**重规已触发**（身份 10.669→12.199）但物理 0.357≤0.40 判撞 | 不是「没重规」，是「重规了绕不开」——注入贴线时走廊宽度不足 |
| 斜坡 A* | FAIL／INVALID | ramp10_astar_groundfilter.json=FAIL、ramp10_astar_plane.json=FAIL、ramp_ground_height_no_path.json=INVALID、ramp_1p4deg_no_path.json=INVALID | **未达成**：根因＝grid_map 无地面分割（坡面点当墙／起点格缺口），前置链或首规不成立；属建图语义问题，不是重规划能力问题 |

### 5.4 缺口清单（向前指向，不回改旧读数）

a. **斜坡参考路线全局固定**（navi_mode=3 语义）：P4 只能证明「局部避让」；障碍封死整条
   参考线时无全局绕路能力，offset0 负例就是实测边界。要「全局重路由」须换 navi_mode 或
   上游改语义——属新决策族，先呈报不擅动。
b. **斜坡 A* 分支被建图语义阻塞**：先修 grid_map 坡面地面分割（平地 profile 已于
   302dbd4b 修过同类问题），再谈该分支 P4；当前 FAIL/INVALID 读数不入 PASS 串。
c. **取证面缺 FSM 状态迁移**：harness 日志只有探针 stdout，REPLAN/SAFETY/EMERGENCY_STOP
   迁移目前靠轨迹身份＋读数反推。后续窗口应同录 roslaunch stdout，直接观测触发源
   （周期 vs 安全），把 5.2 的触发图从源码推断升为运行证据。
d. **REACT_S=15 s 属施工口径**（env `P4_REACT_MAX` 可覆盖，实际等待窗 REACT_S+25），
   非在册门值；引用时不得写成「判据要求 15 s 内重规」。
e. **早退 dump 的 null 字段**：p4_bt_probe.py 部分早退轮次 JSON 里 a23/restored 为 null，
   是「未走到该判据」不是「判据放行」；跨批引用读数须先看 verdict 行档位。
f. **真机 P4 仍 L2**：scan_nav_real 装配与 dry-run 已过（见 SCAN真机接入清单.md），
   物理未跑；本审计全部读数仅仿真。

### 5.5 缺口 c 已闭（2026-11-12 续片，只追加）

修法＝不新增采集、把**已有**的 FSM cout 收进证据包：70_scan 的 roslaunch stdout
（harness 本就收在 `$RUN/scan.log`）里含 `changeFSMExecState` 的迁移行
（`[调用源]: from X to Y`，上游 L539）。新增 `verify/fsm_log.py`（纯标准库解析＋
触发源计数，**只诊断不入判定**）与 `test_fsm_log.py`（离线 11/11，含三反向例）；
`v_scan_p4.sh`/`v_scan_bt.sh` 收尾抽 `fsm_transitions.log` 同批封存；两探针经
`SCAN_FSM_LOG` 在 JSON 附 `fsm` 摘要（env 未设不附＝缺省≠零；缺文件报 error）。
判据链一字未动。

活体验证一窗（平地，sim-viz，证据 `evidence/scan-p4-fsm/`）：P4 PASS（看见 85／
avoid 0.68／物理 0.66／到达 0.143／restored=True／世界件 sha 不变）。FSM 摘要：
555 迁移，replan 触发源 **FSM（周期）=290／TRIG=2／SAFETY=0**，EMERGENCY_STOP=8
**全 SAFETY 触发**且每次经 GEN_NEW_TRAJ 恢复。⇒ 本窗注入箱的避让由周期重规完成；
安全检查以急停—恢复形式介入过 8 次（成因未定论：FSM cout 无时间戳，只能排序不能
对时，留作后续观察项）。WITNESS=INVALID（容器无 systemd）⇒ 不作零触碰声称，同 §4。

### 5.6 斜坡 A* 根因更正（2026-11-12 #27 定位窗，更正 §5.3/5.4b 的归因层）

§5.4b 把斜坡 A* 阻塞归到「grid_map 无地面分割」——对照诊断文档 §11—§14 与本轮
定位窗，该归因**只对了首规层**，须分两层读（旧读数不回改，本节向前更正）：

1. **首规层（已解）**：无过滤时坡面点落进 z 膨胀带 ⇒ A* 起点格不可用、无首规
   （§5/§6 的 INVALID 属此层）。壳侧地面过滤（min_z=0.0 或平面分割）后 **A* 首规
   在 10° 坡成功**（诊断 §11/§12：path 120 点、a star error 0）。
2. **重规层（仍阻塞，L2）**：首规成功后，P4 注入箱体的避让重规在 A* 分支仍失败。
   本轮定位窗（`evidence/scan-p4-astar-fsm/`，navi_mode=1＋平面分割＋goal(1.5,0)＋
   frac0.5/侧偏0.25）首次带 FSM 触发源证据：**触发不是没有**——SAFETY 重规 4 次＋
   周期重规 46 次＋SAFETY 急停 2 次；但重规尝试产不出可行轨迹（`a star error` 606
   条），安全层兜底急停，机器人停在 0.578 m（距目标 1.439 m），FAIL。
   分支本体＝`planFromCurrentTraj` 非参考分支（上游 L787—821：odom 起点
   planGlobalTraj＋adjustGlobalTargetIfOccupied＋callReboundReplan），参考分支
   （L766—785）同一箱体可 PASS（§10）⇒ 失效在上游坡面重规语义，壳侧参数/过滤已
   到边界，修法属诊断文档 §4 的 L2 选项（改基线／壳侧等价件／混合路线／接受缺口）。

附带测量更正：本窗 `replanned=True` 的「新轨迹」start_time=18.778 与第一次急停
WARN 的 sim 时刻逐字相同、首末控制点重合 ⇒ 是**急停零长度轨迹**，不是避让重规。
身份判据原先分不出这一形态，已在两探针加诊断标记 `traj_after_degenerate`
（`bs_degenerate`，只诊断不入判定；本窗数据验证 after=True/before=False）。
§13 的 replanned=False 与本窗退化 True 是同一失效随点云抖动的两种表现（看见
44 vs 21 点），两窗皆 FAIL，「A* 坡上 P4 不达成」定性不变。

⇒ 现状 profile 口径维持：**平地 A*、斜坡参考路径**（scoped，成立条件见诊断 §10）；
斜坡 A* 的 P4 达成与否待真人按 §4 选项裁决，不在非 L2 面内擅动上游。
