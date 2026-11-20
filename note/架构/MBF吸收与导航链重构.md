# MBF 吸收与导航链重构设计（取消 MBF，BT 统一编排）

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
