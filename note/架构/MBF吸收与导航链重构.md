# MBF 吸收与导航链重构设计（取消 MBF，BT 统一编排）

> 最新更新时间：2026-11-21 CST。当前执行口径见第 8 节；前七节保留为本日早先方案，不作为继续自研算法、退役 costmap 或修改真机默认开关的授权。

> 决策：真人 2026-11-21 拍板「**取消 MBF，全走 BT+自研**」；同日补充「只是尽量不自研，
> 可以学习其他成熟的方案——可以把 MBF 中对自己有用的部分学习过来加入我们的架构设计」。
> 本件是设计档，不是许可件；真机上机仍按 `note/10_真机上机Runbook.md` 与 `note/09` §9 的 L2 口径。
> 取代关系：`note/架构/编排层真机化方案.md` 的 D2（真机导航链不含 BT 树）与「BT 调 MBF action」
> 建议按本件作废；该件 P1—P3 的配置分层与镜像依赖成果仍然有效。

## 1. 现状（2026-11-21 现读）

### 1.1 已存在的 BT 原生链（MBF 无关）

| 链 | 入口 | 用途 | 证据 |
|---|---|---|---|
| 坡面 v8 链 | `70_bringup/lite3_bringup/launch/sim_nav_slope.launch`（9 节点，kinematic 载体） | 坡面/3D 支撑图导航 | note/进度/v8/M5/26-11-11、26-11-12 |
| 多层 formal 链 | `scripts/sim/two_floor_nav.py --orchestrator bt` ＋ `scenarios/two_floor_nav/graphs/navigation_bt.yaml` | 多层/单层大厅 formal 跑批 | R1121—R1125（单层大厅 0.422 m） |
| SCAN 链 | `scan_nav_sim.launch` / `scan_nav_real.launch` → `scan_bt_nav.launch` | 平地单层；真机已装配（BT+SCAN，无 MBF） | 26-11-12 真机装配记录 |

BT（`90_platform/lite3_bt`）已具备：WaitForGoal／CheckLocalization／RequestPath／WaitForPath／
FollowPath／WaitArrival／Dwell／Stop、取消分支、Recovery（`skip|retry|replan|abort`＋
`retry_max/retry_delay_s`）、goal gate/lease、profile 分层（demo/real 只含接线）。
树：`trees/navigate_with_planner.xml`。

### 1.2 MBF 仍占的三处

| 处 | 现状 |
|---|---|
| 真机 `mode_navigation.launch` | include `mbf_navigation.launch`（默认启用）；BT `enable_bt:=false` |
| 平地仿真 `mode_nav_sim.launch` / `mode_sim_teb.launch` | MBF+TEB+costmap；`m2_smoke.sh` Stage B 判的就是这条 |
| 旧件 | `lite3_task`（SMACH，真调 MBF action，零 launch 引用）；`pipeline_tracking/TaskTransfer.py`（legacy 客户端） |

退役记录：平地 v8 栈 `sim_nav_stack.launch` 已由提交 `21c0a302` 退役（K2 直写 `/cmd_vel`），
未以「单写者」形式重建——本设计 S2 补回（twist 镜像改投 `/cmd_vel_raw`，经 `m2_cmd_relay`）。

### 1.3 现状判断

- v8 链与 SCAN 链都已是 BT 原生，**取消 MBF 不缺编排器，缺的是入口迁移**（真机/平地仿真/smoke）
  与**判据面迁移**（`m2_smoke.sh` Stage B 的 MBF action 与 costmap 就绪门）。
- 高程链：`support_map` 直接服务 v8 规划器（不经 costmap）；`lite3_costmap_layers` 的
  `ElevationMergedLayer`（costmap_2d 插件）随 MBF 退出生效面，代码保留（删除另议）。

## 2. MBF 机制清单 → 吸收判定

| # | MBF 机制 | 我们的现状 | 判定 | 落点 |
|---|---|---|---|---|
| 1 | **Outcome 分层**：规划类/控制类/恢复类结果码分离（NO_PLAN_FOUND/BLOCKED_PATH vs NO_VALID_CONTROL/PAT_EXCEEDED） | `E_LOCAL_*` 25 码（控制类为主）＋ planner refusal（规划类），未分层 | **吸收**：reason 码定前缀→层（PLAN_/CTRL_/REC_），状态契约按层判 | planner/local status 契约；BT policy |
| 2 | **outcome→disposition 两级映射＋预算**（MBF recovery 配置；`lite3_task` 已有 `recovery_policy.yaml` 同构实现） | BT `recovery_mode` 全局四选一，无按码分派 | **吸收**：`bt_policy.yaml`：reason→{retry_plan,replan,wait,abort,stop}，每次尝试预算 | `lite3_bt/config`＋C++ |
| 3 | **恢复行为一等公民**（wait/rotate/back-off，各有超时/outcome） | BT Recovery 只重排队 | **部分吸收**：先加 `wait`（等输入新鲜）；`spin` 可选、经 mux、默认关；无 costmap 可清 | BT recovery 子节点 |
| 4 | **进度耐心**（PAT_EXCEEDED：patience 窗内无进展即失败） | WaitArrival 只有总超时 60 s | **吸收**：到点等待增 `progress_timeout`（剩余距离无下降即判 STALLED） | `bt_console.cpp` |
| 5 | 规划器/控制器 **pluginlib 插件化** | 固定脚本节点 | **暂不吸收**（graph/manifest 已参数化实例；插件化收益低） | — |
| 6 | **Action 接口**（goal/feedback/result/cancel） | 话题队列＋JSON 状态＋cancel/ack 已定 | **暂不吸收**（内部契约定型）；外部客户端需要时再开 façade | — |
| 7 | **Costmap 分层＋插件** | support_map＋obstacles 直供局部规划 | **不吸收**（取消 costmap） | — |
| 8 | 服务器 liveness | `liveness_m4` 心跳 | **随 MBF 退役撤 m4** | 迁移 launch |

★口径★：「吸收」指机制语义与配置面，不指代码复用；MBF 自身不留在运行时链上。

## 3. 目标架构

```text
/v8/bt/goal（队列） ── BT（唯一编排者/唯一目标所有者）
                        ├─ RequestPath  → 规划器（v8 支撑图 A* ／ SCAN）
                        ├─ WaitForPath  ← /global_path + status
                        ├─ FollowPath   → /v8/local/goal → local_native（3D）
                        ├─ WaitArrival  ← /v8/local/pose（progress patience）
                        ├─ Dwell → Stop（零速，经 mux）
                        └─ Recovery（策略表＋行为子节点）
执行面：local_native → /v8/local/cmd（3D）｜twist 镜像 → /cmd_vel_raw
        → safe_cmd_vel_mux → /cmd_vel → 底盘
安全：mux 仍是 /cmd_vel 唯一写者；BT 不算路径、不写速度（除 Stop 零速）
```

规划器路由（均 BT 编排，按运行模式选，不做「谁最后发速度」的并存）：

- **平地单层/真机**：SCAN（成熟件；`scan_nav_real.launch` 已装配）
- **坡面/多层/3D**：v8 支撑图链（自研必要项——现成方案均假设单一 2D 平面，无等价物）

## 4. 迁移图谱（逐件）

| 件 | 动作 | 片 |
|---|---|---|
| `70_bringup/.../mode_navigation.launch` | 撤 MBF include；接通 BT+规划器链（真机 topic 按 real profile）；BT 默认 true、`enable_motion` 仍默认 false | S5 |
| `lite3_navigation/launch/mbf_navigation.launch` ＋ MBF/TEB/costmap 配置 | 标记 deprecated（保留文件与 revert 路径，不再被 include） | S5 |
| `lite3_costmap_layers` | 随 MBF 退出生效面；代码保留 | S5 |
| `mode_nav_sim.launch` / `mode_sim_teb.launch` | 新增平地 BT 链 `mode_sim_bt.launch`；旧 MBF 壳标注 deprecated | S2 |
| `verify/m2_smoke.sh` Stage B | 判据不动（≤120 s、≤0.35 m、gt 参考）；执行链 MBF→v8 BT，就绪门换 v8 输入门 | S3（判据面挂账） |
| `lite3_task` / `TaskTransfer` | 保持冻结/legacy | — |
| `v8 planner enable_getpath`（MBF GetPath server） | 标注 deprecated（不再有 MBF 客户端） | S5 |

## 5. 验证与判据迁移

- Stage A 不变（直接 `/cmd_vel` 刺激，不涉编排）。
- Stage B：首次迁移跑需带负控（不起 BT/不发 goal ⇒ 应 FAIL）；阈值/预算/参考系零改动。
- 平地/坡面回归：`mode_sim_bt` 目标实跑 ＋ `sim_nav_slope` ＋ formal runner 各一格。
- 真机：real profile 逐项给值 ＋ `note/10` 流程（L2）。

## 6. 分片

- S1 设计落档（本件）
- S2 平地 BT 链 launch（`mode_sim_bt.launch`，物理平面载体、`/cmd_vel_raw` 单写者）＋实跑
- S3 `m2_smoke.sh` Stage B 迁移＋全跑（判据面挂账）
- S4 BT 吸收件（policy 表＋progress patience）＋单测
- S5 真机入口换链＋退役标注

## 7. 风险与边界

- Stage B 是唯一在册判据路径：迁移必须阈值不变＋负控，判据面按 05 A3 挂账。
- 物理平面载体＋local_native：twist 镜像与单写者已定（`/cmd_vel_raw`）；平地 vz≈0，
  垂直分量拒发不触发（坡面才有此约束）。
- 真机 topic 面（LIO/位姿来源/障碍源）需 real profile 逐项给值，属 L2。

## 8. 2026-11-21 续令：复用成熟算法，替换的是编排封装

用户要求继续完成 BT 替换 MBF 封装，并尽量复用、移植成熟开源算法。当前选择为：

```text
任务 / RViz goal → BT 队列 → NavigateWithAction
                           ├→ 显式 seq 请求 → action 适配器 → move_base
                           │                                  ├→ GlobalPlanner
                           │                                  ├→ TEB
                           │                                  └→ 原 costmap 分层
                           └← 对应 seq 的 result / 终止确认 ←───┘
候选速度 → action/lease/时效门 → cmd_vel_raw → 原安全 mux → 底盘
失败 → 先取消并确认终止 → 有限次数 retry/replan 或 abort
```

- **没有新写规划算法**：复用 Noetic `move_base 1.17.3`、`global_planner`、`TEB 0.9.1`；原高程、STVL、SOB 插件及参数保留。`move_base` 保留单目标内部规划/控制状态机，禁用内置恢复，任务排序与恢复归 BT。它不提供 MBF 的独立 GetPath/ExePath/Recovery 三套 action，旧 SMACH/MBF 客户端不被冒充为已迁移客户端。
- **不是把所有 MBF 二进制卸掉**：TEB 的发行包仍依赖 MBF 接口包；退出主链的是 `mbf_costmap_nav` 服务器。MBF 历史 launch/证据及直接调用旧 action 的验证脚本保留为历史/对照，不作为新入口。
- **上轮设计更正**：costmap 并不只能由 MBF 消费；`move_base` 可加载原 `nav_core` 规划器与 costmap 插件，因此第 2 节“取消 costmap”和第 4 节高程层退役不再执行。MBF 本身提供 map-independent/simple/mesh 分层，不能宣称成熟方案都只能单一平面；mesh_navigation 是曲面导航候选，不等价于已完成多楼层移植。
- **进度判据更正**：MBF 的 `PAT_EXCEEDED` 关注有效控制输出的等待时间；现有欧氏剩余距离改善检测并不同义，绕行可能暂时远离目标。它保持默认关闭，不能借 MBF 名义自动启用。新 action 树以执行器结果为准，不用位置距离替代成功结果。
- **执行生命周期**：应用序号在 JSON 字段中传递，不借用会被 ROS 发布器覆写的 `Header.seq`；迟到结果不推进新目标。取消未确认时持续零速并阻断队列，不能继续执行下一目标。租约失效、后端 LOST、取消超时锁定适配器，需人工处理/重启后再接续。
- **恢复范围**：复用现有重试预算/退避，按后端失败原因选 retry/replan/abort；未实现无碰撞证明的原地旋转或后退恢复，不默认对四足输出这类动作。详细 MBF 结果码不从 move_base 的粗粒度结果臆造。
- **真机边界**：只替换入口装配，`enable_motion=false`、`enable_bt=false`、缺 real profile 拒跑以及原数值保持；不在真实机器人上启动/发包/验收。平面后端拒绝非零 z 目标，不投影吞掉三维任务；原 SCAN/支撑图链继续作为独立后端，不能以平面通过签署楼梯/真机。
- **上一轮证据局限**：旧 mode_sim_bt 用无仲裁 relay，BT 零速出口不在执行链；旧 Stage B 的 lease 判断和终态断言不足，9/0 不能证明完整取消/停止安全。此次另以真实 action 和 safety mux 做限定回归，不回改旧证据、不放宽旧门值。

## 9. 开源算法候选矩阵与 BT 内替换（2026-11-21 续令）

用户要求建图/定位/导航算法优先使用或借鉴成熟开源方案，多候选在 BT 编排下可替换对跑。
当前实现只换**算法插件**，不换安全层：所有候选输出仍经 action/lease 守卫 → `/cmd_vel_raw`
→ safety mux；候选不得绕过取消确认、终止确认或真机默认开关。

| 层 | 当前主用 | 可替换候选（镜像已装） | 候选（需联网安装，未落地） | 入口 |
|---|---|---|---|---|
| 局部规划 | TEB 0.9.1 | **DWA 1.17.3**、Trajectory Rollout（base_local_planner 1.17.3）；MPC 0.0.3 已装但两套参数均致 move_base SIGSEGV（r6/r7 留证，不推荐） | eband_local_planner（无可安装索引） | `mode_sim_teb.launch local_planner:=teb|dwa|trajectory|mpc` |
| 全局规划 | GlobalPlanner 1.17.3 | **NavfnROS 1.17.3** | SBPL lattice | `global_planner:=global_planner|navfn` |
| 高程 | 本地高程生产者 + ElevationMergedLayer（已补有效性状态） | STVL（障碍层，已装，非高程） | ANYbotics elevation_mapping（**已接入并可编译/发图**：固定旧提交 + CMake3.16；r8—r10 运行时 move_base 在 0.6 m 后判无有效控制，疑未滤自车近距回波，**列可选未通过**）；elevation_mapping_cupy（ROS2，不适用） | `mode_sim_teb.launch elevation_source:=native|elevation_mapping` |
| 建图 | faster_lio（开源，本仓隔离改写存盘路径） | SCAN/lite3_scan（已装） | LIO-SAM、DLIO | 各自 launch/app |
| 定位 | hdl_localization（已装） | NDT（已装，监控级） | AMCL（apt 可得 `ros-noetic-amcl`，未装未接线） | 各自 launch/app |
| 3D/坡面 | v8 支撑图链（自研必要项） | —（现成方案均假设单一平面） | mesh_navigation（曲面候选，非多楼层等价物） | `sim_nav_slope.launch` |

- 已装候选的替换是**配置级**（plugin 类型 + 对应参数文件），不新增自研算法；
  DWA/Trajectory 参数中的机器人限速/加速度/到点容差沿用 TEB 在册值，未新增物理门值。
- 未装候选（elevation_mapping、mpc、eband、amcl）需要联网 apt/source 安装与镜像重建；
  镜像重建前只在矩阵登记，不在运行链声称可用。ANYbotics elevation_mapping 输出
  `grid_map_msgs/GridMap`（层名 `elevation`）与本仓 `ElevationMergedLayer` 消费契约同形，
  接线主要是生产者替换与时效/坐标系配置。
- BT 侧不区分算法来源：每种候选都必须通过同一 `NavigateWithAction` 成功/取消/复用/租约
  失效用例；对跑读数按窗口登记：
  `r3` TEB+GlobalPlanner 到点 0.155 m／`r4` DWA+GlobalPlanner 0.189 m／
  `r5` Trajectory+Navfn 0.190 m，三轮取消、复用、三维拒绝、租约失效与退净全过；
  失败过程 `r1`（测试断言误判 move_base 内部发布口）、`r2`（高程 current_ 缺陷）原样保留。
- 建图/定位候选不并入本次 BT action 替换：它们属于上游估计链，切换走各自 launch/app，
  但同样不得让真值冒充估计定位（工程标准 STD-03）。

### 官方来源

- [MBF 0.4.0 分层](https://github.com/naturerobots/move_base_flex/blob/0.4.0/README.md)
- [move_base 1.17.3 配置](https://github.com/ros-planning/navigation/blob/1.17.3/move_base/cfg/MoveBase.cfg)
- [move_base 单目标执行源码](https://github.com/ros-planning/navigation/blob/1.17.3/move_base/src/move_base.cpp)
- [TEB 0.9.1 nav_core 插件导出](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/teb_local_planner_plugin.xml)
- [MBF 控制输出 patience](https://github.com/naturerobots/move_base_flex/blob/0.4.0/mbf_abstract_nav/src/abstract_controller_execution.cpp)
- [mesh_navigation Noetic 候选](https://github.com/naturerobots/mesh_navigation/blob/noetic/README.md)
