# R851 独立验收 · 第三轮

- 封存时间：2026-10-28T22:24:00+08:00
- 审查者：independent_round3（全新实例，只读复核）
- 审查目标：`6a689be01ff55620d3b4ba7589173536771f948f`（R850 收尾）
- 冻结副本：`/tmp/r851`（`git archive 6a689be`）；纯 Python 与证据读取均在副本内完成
- 未改 live repo 的 tracked 文件、未改 `.ai/`、未 commit、未启停他人的容器；唯一仓内写盘为
  `note/进度/v8/M5/26-10-28/evidence/R851/independent_round3/` 四件，R846 复跑只写 `/home/jack/temp`（容器 `/work`）
- 注意：live 工作树在本轮 22:08—22:14 出现 4 个 `M`（`calibration/flat2d/driver.py`、
  `regression/runtime_batch.py`、`regression/runtime_lab.py`、`m1_gazebo.launch`）与 3 个 `??`
  （`estimated_track.py`、`noisy_localization.py`、`test_estimated_track.py`）。这些均**不在**
  `6a689be` 内，非本审查所改；本审查的代码/测试全部用 `/tmp/r851` 冻结副本。门禁 live 读数
  可能含这些未提交改动（见 §7 的 VCC 面差说明）。

**本轮判定：`budget_151_freeze = PARTIAL`，`runtime_release = PARTIAL`。**

- **#151 的数值与配置壳可以复算、可以复现**：三次原始 physics 重算得到
  0.1221/0.1153/0.1157 m 与 0.372/0.365/0.385 s（max 0.1221 m / 0.385 s），
  `PENDING_REQUIRED=[]`、两键入 `SCOPED_REQUIRED`、`DIGEST=4f570e94…`、`frozen_baseline.json`
  与 `FROZEN` 逐项一致（deep equal，digest 同值）；`052f5e9..6a689be` 只加两键，其他冻结值零变化。
- **但 #151 作为正式封版不能原样站住**：载体是速度直推（`SetLinearVel`），停车在 1 个 20 ms
  采样内完成，制动段约 4 mm；0.122 m 主要是感知/规划/mux 的时延。原 R716 / 24 §4.1 要求
  stop_distance 覆盖“感知/控制延迟**与制动**”，本件没有覆盖制动段；且 `full_chain` 不含全局规划。
  另外两键没有 `unfreeze_log`、未进 `REQUIRED` 形状表、`check_loosening` 对 budgets 只识别下调
  （0.15→0.50 返回 []，0.15→0.10 反而报 E_CAL_LOOSENED），而记录该盲区的两条测试被本批删除。
- **运行时**：A5 当前行为仍为 5/5、step 落面 z 误差 8.17e-11、独立评价零碰撞；33 套件 rc 全 0
  （308 例）。但整包 A1—A7 仍 `NOT_VALIDATED`：A1 两格 INVALID、step 根因未归因且无回归保护、
  六枚件旧侧不可恢复、无真机/四足跨层/四组合矩阵。故只放行 A5 行为面，不放行整包。

---

## 1. #151 冻结完整性

### 1.1 三次原始 physics 的独立重算

证据：`evidence/R850/noroute_run1.json`、`noroute_run2.json`、`noroute_run3.json`。
方法：按 R847 `command.txt`，取 `trigger.wall → trigger+trigger_to_physical_stop_wall_s` 窗口内
`phase=sensed_no_route` 的 physics 采样，逐段 `math.dist(xyz[:2])` 累加；物理停住＝窗口内最后一个
零线速度采样。**未使用** JSON 自报的 stop_distance 字段（该字段不存在，只有时间与净空）。

| run | 触发 sim | 物理停住 sim | 仿真停车 s | 墙钟停车 s | 报告距离 m | 重算距离 m | 停后漂移 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 1.239 | 1.611 | 0.372 | 1.1263 | 0.1221 | 0.122100 | 0.0 |
| 2 | 1.232 | 1.597 | 0.365 | 1.4979 | 0.1153 | 0.115317 | 0.0 |
| 3 | 1.482 | 1.867 | 0.385 | 0.9922 | 0.1157 | 0.115716 | 0.0 |
| **max** | — | — | **0.385** | 1.4979 | **0.1221** | **0.122100** | 0.0 |

- 三例均 `ok=true`、`reason=E_LOCAL_NO_ROUTE`、`speed_before_m_s=0.4`；
  触发事件为 `spawn_requested`，触发后 `lidar_obstacle_detected`（sensor_stamp 1.41—1.7 区间），
  再由本地规划器拒绝。墙钟 0.99—1.50 s 远大于仿真 0.365—0.385 s，报告以仿真钟为预算。
- **停后漂移为 0**：物理停住后同 phase 继续采样 4.3—5.5 s，x 保持不变
  （0.16001/0.15761/0.15961），不存在“停住后又滑一段”的窗口污染。
- 三例 `hardware=false`、`four_foot_contact_or_cross_layer=false`、`budget_151_frozen=false`
  （最后一项是 run 时状态，不是冻结后的回填）。

### 1.2 距离方法的起点截断（本轮新发现）

报告方法只累加 `wall >= trigger` 的采样，**触发点落在两个 physics 采样之间**，因此漏掉了
“触发→触发后第一个采样”这一段位移。以 x 向插值/前一采样重算：

| run | 触发前最后采样 x | 触发后第一采样 x | 触发点插值 x | 停住 x | 报告距离 | 插值起点距离 | 前采样起点距离 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.02634 | 0.03791 | 0.02892 | 0.16001 | 0.1221 | **0.1311** | **0.1337** |
| 2 | 0.03033 | 0.04230 | 0.03236 | 0.15761 | 0.1153 | **0.1253** | **0.1273** |
| 3 | 0.03591 | 0.04389 | 0.03638 | 0.15961 | 0.1157 | **0.1232** | **0.1237** |
| **max** | — | — | — | — | 0.1221 | **0.1311** | **0.1337** |

- 报告值每例少算 7.5—11.6 mm（一个 20 ms 仿真采样的位移量级）；真实 max 约 **0.1337 m**。
- 0.15 m 预算**仍覆盖**修正后的 max（余量 0.0163 m，约 10.9%—12.2%），但报告里的
  “max 0.1221×1.23≈0.15（23% 余量）”是**截断窗口上的算术**，不是真实余量；
  “距离离散 ≤6.8 mm”也是截断后的离散（真实三例范围 0.1232—0.1337，约 10.5 mm）。
- 时间窗起点是 trigger wall，因此 0.372/0.365/0.385 s 没有同类截断；但时间与距离的起点
  并不完全同源（时间含触发后第一段，距离从第一采样起算），报告没有声明这点。

### 1.3 载体：速度直推、无制动段

`evidence/R850/noroute_run1.json` 的 `raw.physics` 速度剖面：

```
sim 1.530  x=0.14804  vx=0.4
sim 1.550  x=0.15602  vx=0.4
sim 1.571  x=0.16001  vx=0.0   ← 一个 20 ms 采样内从 0.4 直接到 0
sim 1.591  x=0.16001  vx=0.0
```

- 代码路径：`ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp`
  `driveMode=dynamic` 分支每步 `parent_->SetLinearVel(v_world...)`，没有力/力矩上限、没有制动
  模型；URDF `lite3_planar.urdf.xacro` 的 `driveMode` 实际为 `dynamic`（注释里的“默认 kinematic”
  是旧句）。因此“dynamic”只保留碰撞/重力，**不含制动动力学**。
- 0.4 m/s 下 0.122 m 的组成：spawn→detect 约 0.266 s、detect→规划拒绝约 0.05—0.07 s、
  mux 归零→物理停住约 0.005—0.05 s 墙钟；真正的“撤令→停住”只有约 4 mm（0.15602→0.16001）。
- 结论：这是**平面速度载体的全链时延预算**，不是制动距离。原 `calibration.py` 的 R716 注释
  （“无制动平台测得值物理上不含制动段，不能代表该预算”）在语义上仍适用；R850 换成了 Gazebo
  dynamic 平面载体，但驱动仍是每步 `SetLinearVel`、停车仍是一步清零，没有给出制动段读数。
  FROZEN evidence 写了“不代表腿部载体制动段”，但没有写“本载体自身制动段≈4 mm、0.122 m 为
  时延主导”。`R850.md` §4.3 又承认不含全局规划段。“全链”“动力学”两词容易被读成
  “含制动、含全局规划”，措辞需收紧。

### 1.4 配置面（全部通过）

| 检查 | 读数 |
| --- | --- |
| `budgets.stop_time_s` | `value=0.50`、`scope=gazebo_planar_full_chain_sensed_no_route_0p4_simtime_n3_no_staleness`、`sample_n=3`、evidence 含 R850 三值 |
| `budgets.stop_distance_m` | `value=0.15`、同 scope、`sample_n=3`、evidence 含 R850 三值 |
| `PENDING_REQUIRED` | `[]` |
| `SCOPED_REQUIRED` | 两键均在，`min_sample_n=3`，scope 精确匹配 |
| `DIGEST` | `4f570e94f9ffad14620a23fd3f6c34f701baeb5aa2da9f3e65205714fbfeee36`；`calibration.digest()` 同值 |
| `frozen_baseline.json` | 存在；`frozen == FROZEN`（deep equal）；`digest` 同值 |
| `calibration.py freeze` | `status=pass`、`blocked=false`、`freeze_allowed=true`、`pending=[]`、rc=0 |
| `git diff 052f5e9..6a689be -- calibration.py` | 只加两键、删两条 PENDING、加两条 SCOPED、改 DIGEST；其他冻结值/证据零改 |
| 叶子级 diff（052f5e9 FROZEN → 6a689be FROZEN） | 只新增 8 个叶子（两键的 value/scope/sample_n/evidence）；无删除、无变更 |
| `check_loosening(old, new)` | `[]`（两键是新增，旧表无对应 `.value`，故不触发） |

### 1.5 治理/形状缺口（本轮新发现，均影响“封版可站性”）

1. **无 `unfreeze_log` 记录**：`FROZEN['unfreeze_log']` 共 19 条，没有任何
   `budgets.stop_time_s.value` 或 `budgets.stop_distance_m.value`。此前每个从 pending 进入
   FROZEN 的新键都有 `from='未定标（pending）' / to=value / reason` 的记录（cancel_time、
   brake peak、osmesa、p95_local、localization、success_rate 均如此）。`calibration.py` 头注
   明确“封版后变更须走显式 unfreeze（记录变更与理由），不得静默改值”。R850 属静默新增。
2. **未进 `REQUIRED` 形状表**：`REQUIRED` 中有其他所有 budgets 键的 `value/scope/sample_n/
   evidence` 形状项，但没有 `budgets.stop_time_s.*` / `budgets.stop_distance_m.*`。
   实测：保留 scope/sample_n/evidence、删掉 `value`（或置 `None`、置 `inf`）时
   `validate_frozen` 仍返回 `status=pass`、`pending=[]`；只有 `assert_digest` 会红。
   缺 evidence 会被 `_pending_report` 的 ad-hoc 循环抓到；缺 value 不会。
3. **`check_loosening` 对 budgets 的方向是反的**：
   - `0.15 → 0.50` 返回 `[]`（放宽不报）；
   - `0.15 → 0.10` 反而 `E_CAL_LOOSENED`（收紧被当成“预算/上限下调”）。
   这条盲区在 R726 的 unfreeze 记录里已自认，但本轮加入的正是两个 budget 键；若后续把
   `stop_distance_m` 从 0.15 抬到 0.50，`freeze_guard` 不会拦，只有 DIGEST/人工复核能拦。
4. **FROZEN 内仍有“仍 pending”的旧句**：`budgets.cancel_time_s.evidence` 写
   “全链停车由 budgets.stop_time_s（仍 pending）把关”；`stop_brake_peak_distance_m.evidence`
   写“全链 budgets.stop_distance_m 仍 pending”；`stop_brake_peak_time_s.evidence` 写
   “全链 budgets.stop_time_s 仍 pending”。冻结后这些句子已不成立，R850 未改。
5. **P8/05 同步**：R850 改了判据载体（calibration.py FROZEN），但 `note/05` 没有同批 F 段
   登记（R849 只加了台账断档指针）。按项目规则“动到判据载体要按 note/05 F 段回写”，这一条
   缺登记；不影响数值，但影响交接可追溯性。

### 1.6 #151 小结

- **数值可复现、配置壳正确、DIGEST/基线一致、旧值零变化**：这些是真实绿。
- **作为 #151 正式封版不成立/只能部分成立**：制动段未覆盖（速度直推一步停）、`full_chain`
  不含全局规划、距离窗口少算一个采样、两键缺 unfreeze/REQUIRED、反放宽守卫方向反了且
  守该盲区的测试被删。若把 scope 明确改写成“平面速度载体、仿真钟、感知+局部规划+mux 的
  时延预算（制动段≈4 mm，不含全局规划/真机/腿部）”，值 0.15/0.50 可保留；现有措辞不行。

---

## 2. 测试完整性

### 2.1 跑数

```
python3 -B ws/src/90_platform/v8_platform/calibration/test_calibration.py
Ran 52 tests ... OK

for f in ws/src/90_platform/v8_platform/*/test_*.py \
         ws/src/95_simulation/lite3_gazebo_sim/dynamics/test_*.py; do python3 -B "$f"; done
TOTAL=33 FAILURES=0 CASES=308
```

- 33/33 rc=0；用例 **308**，而第二轮（R844）是 **314**。差 −6 ＝ calibration 59→52（−7）
  ＋ `local_planner/test_execution.py` 12→13（+1）。
- R850.md 与 `00_总览` 只写“52 项 calibration 全过；33 套件 rc 全 0”，没有写“套件从 59 缩到 52、
  总用例从 314 缩到 308”。这不是数值错误，但属于**报告省略**。

### 2.2 新冻结测试有牙

- 在 `/tmp` 副本把 `stop_time_s.value` 改 0.60，`test_calibration.py` 出
  **4 failures + 2 errors**：`test_stop_budgets_frozen_r850`、`test_stop_full_chain_keys_frozen_r850`、
  `test_frozen_valid_and_digest_stable`、`test_cli`、`test_digest_golden_rejects_value_change`、
  `test_success_rate_metadata_complete_no_pending_r850`。
- 删掉 `stop_time_s.scope` → `validate_frozen` 抛 `E_CAL_PENDING`（scope 精确匹配）；
  `sample_n=2` → `E_CAL_PENDING`（低于 3）。
- R845 的 `test_in_flight_uncertainty_blocks_contact_certification`：在 `/tmp` 副本把
  `max(distance, observed_distance) + uncertainty_m <= 1e-6` 去掉 `+ uncertainty_m`，
  该测试 **FAIL**（`complete=True`）；当前代码 PASS。**有分辨力**，但它保护的是 uncertainty 项，
  不是 R842 的零长终段分支。

### 2.3 被删测试：7 条净减，其中 8 条是附带删除

`git diff 052f5e9..6a689be -- test_calibration.py`：**12 removed / 5 added**。
4 条属状态替换（`test_stop_full_chain_keys_stay_pending_r720`、`test_stop_fields_stay_pending_r716`、
`test_success_rate_metadata_complete_only_other_three_pending_r748`、
`test_stop_time_with_metadata_leaves_pending_r759`），其余 **8 条为附带删除**：

| 被删测试 | 对当前配置重跑的结果 |
| --- | --- |
| `test_p95_local_frozen_scoped_r722` | FAIL，但只因它断言 `status=blocked`（旧 pending 态）；值/scope/sample_n 断言本身通过 |
| `test_p95_local_scope_and_sample_guard` | PASS |
| `test_stop_chain_osmesa_proxies_scoped_r726` | FAIL，但只因断言 `pending=2`（旧 pending 态）；值/scope 断言通过 |
| `test_stop_chain_osmesa_scope_guard` | PASS |
| `test_stop_chain_osmesa_distance_guard` | PASS |
| `test_budget_raise_not_flagged_by_loosening` | **PASS** —— 它记录的正是 budgets 只识别下调的盲区；删掉后无人守 |
| `test_stop_component_raise_logged` | **PASS** —— 它检查旧预算抬高的 unfreeze 记录；删掉后新键无记录也无人守 |
| `test_resources_and_sampling_frozen_shape` | PASS —— cpu/mem 值域、seed 类型/唯一性检查被删 |

- 这 8 条里 6 条对当前配置仍绿，说明它们不是“前提被取代”，而是** collateral deletion**；
  2 条只是断言旧的 `blocked/pending=2`，本可改断言保留，而不是整条删除。
- 新增的 `test_stop_time_with_metadata_and_scope_frozen_r850` 把 `value=1.5`（3 倍预算）
  配正确 scope/evidence，断言 `validate_frozen` 通过；它实际上**演示**了形状/放宽守卫的缺口，
  不是守卫。
- `test_pending_report_not_green` 的名字仍写“not green”，现在断言 `status=pass`、`pending=[]`；
  注释区还留着“stop_time、stop_distance 保持 pending”（第 195—201 行），与新状态相反。
- 结论：**测试面被削弱**，且削弱点正好落在本轮新增的两个 budget 键的治理上。
  R850.md 只提 R716/R720 两条被取代；12 条删除中其余 10 条（含 8 条附带删除）未披露。
  这是本轮最强的“弱化测试”发现。

---

## 3. R845 处置复核

| 项 | 结果 |
| --- | --- |
| R842 hunk 是否撤销 | **是**。`6a689be:local_planner/local_native.py` 第 197—201 行恢复为 `if planned_distance <= 1e-12: if now >= target[3]: return complete`；R842 的“终段时钟接触认证”块已不在 |
| 新回归测试是否有牙 | **是**（见 §2.2）；但它只覆盖 `uncertainty_m`，不覆盖零长终段分支 |
| F5 碰撞数 16 | **是**。`a5_ros_initial_fail.json` case0 `independent_evaluation.collisions` 长度 16，`failures=['dynamic_collision']`，全在 segment 3；`trace` 9 点、无 `execution_trace`、domain 旧串 |
| TEB 两条修复轨迹 sha256 | **是**。`sc01_s0_trajectory.json` = `9d60cb5d…e313d481`，`sc09_s2_trajectory.json` = `31617058…f63ecb`，与各自 driver 的 `telemetry_sha256` 一致；两文件在 `evidence/R845/` 内 |
| `hardware` 旗标 | **部分**。R850 三件顶层 `hardware=false`；R845 `a5_ros_v4_full.json` 的 5 个 case 各有 `hardware=false`，但**顶层没有** `hardware` 键（顶层只有 `gazebo/dynamics`）。G4 只算半闭 |
| 零长终段分支的回归保护 | **无**。R845 的“不可达”证明只针对 `plan()` 结果（`valid_until ≤ target[3]`）；`track_segment` 是公开函数，外部若直接传零长 trajectory 且 `now>=target[3]`，仍会时钟认证接触。没有测试钉住这条 |

另：R845 的两份 manifest 的 `telemetry_json` 仍指向 `/tmp/r841-teb-clock-repair*` 旧容器路径；
repo 内的轨迹靠 `hashes.sha256` 与 driver 的 `telemetry_sha256` 建立映射，指针本身未改成 repo
相对路径。可复核，但交接时需知道映射。

---

## 4. R846 效率复跑

在容器 `v8cal`（`/repo` 挂 live 仓，`/work` 挂 `/home/jack/temp`，MuJoCo 2.3.7）按 R846
`command.txt` 的原命令重跑：

```
python3 -B crawl_gait.py crawl --cycle FL,HR,FR,HL,FL,HR,FR,HL --skip-final-advance \
  --kp 150 --kd 1.0 --swing-kp 100 --swing-kd 1.0 --drive-fwd 0.06 --swing-fwd 0.04 \
  --margin-min 0.05 --shift-policy incenter --normalize-step-x \
  --out /work/r851_efficiency.json
```

读数（`/home/jack/temp/r851_efficiency.json`，sha256 `94136136…b16d31`）：

| 项 | 值 | 与 R846 `sweep_norm_d0.06.json` |
| --- | --- | --- |
| `pass` | true | 同 |
| 相位 | 58 | 同（相位名逐条相同） |
| `final_x` / `final_y` / `final_z` | 0.214 / −0.06 / 0.337 | 同 |
| `completed_steps` | 8 / 8 | 同 |
| `lift_h` | 0.10 | 同 |
| `margin_min` | 0.05 | 同 |
| `slip_tol` | 0.02 | 同 |
| `y_tol` | 0.03 | 同 |
| `step_h` | 0.0 | 同 |
| `world_feet_end` | 与证据逐足相同 | 同 |

- 代码 `crawl_gait.py` blob `d84cccbd…`、sha256 `f7cf6c55…` 与 R846 `code_ref.txt` 一致。
- 负结果也在证据里：0.065 → 4/8、`E_GAIT_SUPPORT_MARGIN`；`closest` → 8/8、`pass=false`、
  final_x 0.119；0.06/0.06 摆步 → 4/8。R846 的“0.065 在第 4 步失败、closest 负结果”成立。
- 该复跑只写 `/work`，未改仓；读数与 R846 逐项一致，**效率结论可复现**。

---

## 5. R848 台阶

- 坏夹具：`force_stair_*.json` 的 `world_feet_end` 四足 x 全 <1.0（最大 0.958/0.98，
  006/080 只有 0.099），世界台阶前沿 x=1.0；`force_experiment` 在 force 件为 `[]`
  （006/force070/sX070_h04/sX070_h05/sX072_h06/sX075_h06 共 6 枚），两枚 pd 件为 `None`
  （它们不是 force 跑）。`force_stair_force070.json` 与 `force_stair_pd070.json` 的 32 个
  相位逐条相同，支持“力控桥接未激活”的撤回理由。
- 有效件：`force_world_stair06b.json` 的 `force_reports` 长度 **3**；后腿抬升（f1−f0）为
  **3.99 / 5.64 / 5.59 cm**（plain 同相位 8.11 / 8.37 / 8.96 cm），三条 force `lifted=False`
  （阈值 0.7×0.10=0.07 m），plain 三条 `lifted=True`；双方均 5/10、不摔。
- `plain_step04.json`：5/10，首失 `advancer0_HL_b` support_slip，前足 z=0.061（落在 0.04 m
  阶面），后足悬空 0.236/0.322；R848 §4.2 明确写“单级低台阶，不是字面‘两级踏面’楼梯”。
  R849 的摘要用“①两级/低台阶”略宽，但 R848 正文的边界是诚实的。
- 撤回诚实；**唯一措辞瑕疵**：R848 §1 把 8 枚坏件统称 `force_experiment=[]`，实际 6 枚 []、
  2 枚 pd 为 None。不影响结论。

---

## 6. 04/05 追加

- R849 `7ccf816` 对 `note/04` +11 行、`note/05` +4 行，**0 删除**；`git diff --unified=0` 显示
  新增块在 04 第 7090 行（原文件 7089 行）与 05 第 2580 行（原 2579 行），即**文件末尾追加**；
  旧段落、旧号、旧数字未改。
- R850 `8de88d0/6a689be` 未改 04/05（`git diff 6a689be^..6a689be -- note/04 note/05` 为空）。
- 结论：R849 的“乙”落件形式合规；R850 没有对应的 04/05 F 段同步（见 §1.5.5）。

---

## 7. 第二轮遗留项与门禁

| 项 | 状态 | 读数 |
| --- | --- | --- |
| F6 | **仍闭** | `local_planner/runtime_probe.py::start_node` 仍无 `start_new_session`（子进程继承 probe PGID）；`844c735..6a689be` 只改 `hardware` 一行；第二轮反例仍适用 |
| G6 六枚件 delta | **仍不可恢复** | `runtime_lab.py`、`evaluate.py`、`ros_node.py`、`local_native.py`、`runtime_probe.py` 均在 `d74c4ab` 首次入仓；只有 `trajectory_checker.py` 有 `5f646f9` 可比（收紧）。旧侧不可取 |
| A1 两格 INVALID | **未变** | `teb_matrix/results.json`：SC01_s0、SC09_s2 的 `sim_duration_s/rtf=null`；7 有效 + 2 INVALID，分母不变。R845 只补了轨迹，不改矩阵读数 |
| ISO | **1（未重跑）** | R843 归因：40_navigation 两件 C++ 在册例外 + 无 devel 的 4 BAD；本轮代码/脚本未动，归因不变 |
| DC | **1（重跑）** | `DC_ROOT=live bash v_deferred_clause_scan.sh`：A=1、B=16，rc=1；与 R843 同 |
| VAR | **9（INVALID，未重跑）** | 运行器入参/证据 glob 前置问题；按三态通道 INVALID，不入串；脚本未动 |
| 账构 | **5（重跑）** | `BATCHCHECK rc=5`；C4 滞后 **220 批**（R850 − R630），R843 记 212、R845 记 213。+7 来自 00_总览 新增 R844/R849/R850 条目而档头仍 R630；不是新类别，是已知断档在扩大。C1 仍“声明 R630／实动未取到” |
| VCC | **6（重跑）** | **live 工作树**：`VCC VERDICT rc=6`，RED=20（hdl `hdl_localization.launch:89` ×12 ＋ v7 `进度文档.md:972` ×8）、AMBER=46、OK=600；**git archive 面**：rc 同为 6，但 RED=14（缺 4 个 git-ignored/untracked 目标，其中 2 条变成“路径失效”）、AMBER=47、OK=606。两面 rc 一致；RED 计数依赖工作树面，不能当 commit-only 读数。R843 的 18 是当时读数；HEAD/live 20 与第二轮 G3 一致 |

- 门禁五红的**归因类别**未变；账构 C4 数字 213→220 已解释；VCC live 面 18→20 是 R843 自引导致的
  已知漂移（第二轮已记录），archive 面 14 是缺 ignored/untracked 目标的**面差**，不是“变绿”；
  两面 rc 均 6。ISO/VAR 本轮未重跑，但相关代码/脚本未变，按“未复跑”记 UNRESOLVED，不冒充绿。

---

## 8. 假绿排查

1. **step 到点 0.08 m vs 接触 1e-6**：`local_native.Config.goal_tolerance_m=0.08` 只用于水平/
   规划到点；`evaluate.py` 的接触仍 `z_tol=1e-6`，`checker.check(..., z_tol=1e-6)` 仍在。
   R845 step 末 z 误差 8.17e-11，独立评价 `ok=true`、0 dynamic/0 static collision。**不是假绿**。
   但 step 末 x=0.524 vs goal 0.6（差 0.076≤0.08），到点判据确实允许 7.6 cm 的横向差；这是
   规划器到点容差，不是接触容差，报告没有把它写成接触精度。
2. **0.15 m / 0.50 s vs 三次 raw**：见 §1.2/§1.3。触发点定义是“spawn_requested”，不是
   “传感器首次看到障碍”；窗口含 spawn 延迟（保守），但距离从第一采样起算（少算 7.5—11.6 mm）。
   物理停住后无漂移；时间以仿真钟计，墙钟 0.99—1.50 s 是仿真 0.365—0.385 s 的 2.6—4.1 倍。
   预算只在“仿真钟、平面速度载体、感知+局部规划+mux”域内成立；**不能**读成真机/腿部制动预算。
3. **0.04 m 台阶**：是单级低台阶（`Lite3_step04.xml`），不是字面两级楼梯；R848 §4.2 已自认。
   R849 的“两级/低台阶”是简称，正文边界在 R848，未见“四足全上台阶”的主张。
4. **真机/四足跨层/#151 越界主张**：R845/R850 的证据有 `hardware=false` /
   `four_foot_contact_or_cross_layer=false`，R848 自载 `hardware_validated=false`，R849 明写
   “无真机/暂缓”，R848 §4 明写“不构成四足跨层或真机证据”；未发现“真机已验证”“四足全上台阶”
   “#151 已正式验收”的文本。唯一过强的是 #151 的“全链/动力学”措辞（不含全局规划、无制动段），
   以及 `00_总览` 把它写成“冻结”。

---

## 9. 不能验证 / 未解决

1. **step 的代码级根因**仍未归因；R845 只撤回主张、补了 uncertainty 测试，没有一条测试在
   零长终段分支被删/改时变红（该分支目前无人调用，但也没有“不可达”的测试锁）。
2. **A1 两格 INVALID**（SC01_s0、SC09_s2）仍不能进分母；R845 只让补跑轨迹可复核。
3. **六枚件 delta 旧侧**不可恢复（G6），只能按现态验收；本轮的 #151/测试改动不改变该限制。
4. **整包 A1—A7** 仍 `NOT_VALIDATED`：四组合×双轨、真实双定位切换、原生三维规划进接触动力学、
   四足全上台阶、真机迁移均无本轮证据。
5. **#151 的制动段/腿部载体/真机**没有读数；全局规划段不在链内；0.15/0.50 只是平面速度载体
   的仿真时延预算。
6. **R850 的治理缺口**（unfreeze_log、REQUIRED、反放宽方向、删测试）未在本轮修复；本轮只能
   记录，不能替施工侧改。
7. **ISO/VAR 本轮未复跑**；按脚本未变与 R843 归因记“未复跑”，不冒充绿。

---

## 10. 放行结论

- **A5 当前代码的 ROS 五例行为**：可放行——5/5、独立评价 0 碰撞、step z 8.17e-11、F6 仍闭。
- **#151 两停车键**：**不按现措辞放行**。数值/配置壳可复算，但它是“平面速度载体 + 仿真钟 +
  感知/局部规划/mux 时延”的预算，不是含制动的全链停车预算；且两键缺 unfreeze、缺 REQUIRED
  形状、反放宽守卫方向反了、守盲区的测试被删。若把 scope 改写成“平面速度载体、仿真钟、
  时延预算（制动段≈4 mm、不含全局规划/真机/腿部）”，0.15/0.50 可作为该窄域的候选保留，
  但仍需补 unfreeze/REQUIRED/测试。
- **整包 runtime（A1—A7）**：**不放行**。A5 行为绿不等于整包验收；A1 两格 INVALID、step 根因
  未归因、G6 旧侧不可恢复、#151 只部分、无真机/四足跨层/四组合矩阵。

P8：未作计划书终稿、正式 A0 或许可声明；本件只出读数与判定。
本目录恰好四件（01 审查／02 复现日志／03 机读判定／04 sha256 最后生成且不自录入）。
