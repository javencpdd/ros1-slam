# gait_guard 感知接线与门值机制 · 运行说明

本件记录 2026-11-09 落到运行代码的两项改造，供运维按参数开档。**不是验收结论，不产生 PASS/FAIL，不签真机可用性。**

## 一、改了什么

| 项 | 落在哪 | 默认 |
|---|---|---|
| ②感知接线 | `framework/stair_gait_guard_node.py` 新增订阅 `/elevation_mapping/elevation_map`（`grid_map_msgs/GridMap`）→ `framework/terrain_observer.py` 产出地形特征 → 发到 `/v8/gait_guard/terrain` | `perception_mode: shadow`（只报不拦） |
| ③门值机制 | `framework/gait_gates.py` 把 5 个门值从写死常数改成「声明基线＋实测契约反算」双源解析，节点按解析结果重建 `GaitDecision` | `gate_source: declared`（在册基线逐字不变） |

**③改的是门值的产生机制，不是门值数值本身。** 在册 6 值（`lookahead 1.5 / enter 1.0 / exit 1.5 / min_step 0.05 / max_step 0.15`）默认一个没动。理由：`GAIT_PARAM_CONTRACT.md` §5 的反算式依赖 `cmd_lag_p95_s`、`switch_settle_s`、`stop_distance_m` 等真机实测字段，而空白契约模板里这些字段**全为 null**。真机回填之前，任何把 1.5 改成某个具体数字的动作都没有依据，且命中 Runbook §8 与契约 §9.3 的「旧判据门值／真机及其参数」停手面。机制落地后，回填完成即可由反算式改写生效门值，不需要再改代码。

## 二、新增 ROS 参数（全部带默认值，graph 零改动）

`graph_3d_lite3.yaml` 的 `gait_guard.params` **未改动**，参数由节点 `rospy.get_param('~x', 默认)` 读取；要开档就在 graph 的 `gait_guard.params` 加同名键，或对本机覆盖件 `setParam`。

| 参数 | 默认 | 含义 |
|---|---|---|
| `perception_mode` | `shadow` | `off` 不订阅｜`shadow` 只发诊断、决策仍走声明地图｜`assist` 感知只做否决不驱动 |
| `elevation_topic` | `/elevation_mapping/elevation_map` | 高程 GridMap 输入 |
| `terrain_topic` | `/v8/gait_guard/terrain` | 影子特征（JSON String）输出 |
| `terrain_resolution` | `0.05` | 消息 `info.resolution` 缺失时的回退格距 |
| `terrain_lookahead_m` | 同 `lookahead_m` | 感知前视距离 |
| `terrain_half_width` | `5` | 横向净宽扫描半宽（格） |
| `gate_source` | `declared` | `declared` 在册基线｜`contract` 契约反算 |
| `gait_contract` | `{}` | 契约 dict（由 param_exec 灌入，节点不直接读文件） |
| `contract_gait` | `flat_mid` | 取哪一步态的实测字段做反算 |
| `gate_margin_m` | `0.0` | 反算式的安全裕度，须现场作业单给值 |

## 三、门值反算式与拒用条件

```
lookahead_min_m = v*(cmd_lag_p95_s + switch_settle_s) + stop_distance_m + margin_m
```

`v` 取契约 `gaits.<contract_gait>.envelope.v_max_mps`；字段路径逐字取自 `gait_param_contract.yaml.template`，定义在 `gait_gates.CONTRACT_FIELDS`。以下任一情形**整体回退声明基线**并留 `refusals`，不做部分应用：

- `status: TEMPLATE_ONLY` 或 `effective: false`（真机未回填的当前状态）
- 任一必需字段为 `null`／非数值／非有限
- `declared` 缺键或非法（空集拒判，抛 `ValueError`）

两条只增不减约束：反算前视小于声明基线时保持声明值（缩短前视会降低安全裕度）；`max_step_m` 仅在契约 `step_max_m` 更小时收紧。`enter_m`／`exit_m`／`min_step_m` 没有实测依据，一律保持声明值，不反算。

## 四、为什么默认 shadow 而不是直接换输入

`stairstudy/fixtures.py:86-87` 的 `stairs_mid` 与 `ramp_envelope` 包络逐点同量（rise 0.80 / run 2.24），但阶面是竖直不连续、斜坡是连续梯度，步态需求不同。直接把高程源塞进 `stair_ahead` 会把两者判成同一件事，反而更差。因此 `terrain_observer` 必须输出 `discontinuity_type`（`vertical_step`／`continuous_slope`／`blocked`／`none`／`insufficient_data`）与 `clearance_m`，先做与声明地图的逐条对照和分歧记录，不写 `/v8/chassis/gait`。

`assist` 档语义收窄为**否决**：感知判为斜坡或堵头、而声明地图说前方有台阶时，保持平地步态，避免误切 `high_step`。感知从不主动驱动切换。开 `assist` 属 EVT 段，须先有现场验证与真人授权。

`clearance_m` 是沿路径前方窗口内横向连续有效格数乘格距的保守估计，不是真实侧向净宽；侧壁薄板、悬空、负障碍在 2.5D 高程里不可见，是已知决策盲区。

## 五、边界与状态

> 本节记录 2026-11-09 15:57 批次的边界；2026-11-09 后续批已改 `graph_3d_lite3.yaml`、
> `local_planner` 与 `stair_gait_guard_node.py`，见 §七。

- **未改**：`graph_3d_lite3.yaml`、`configs/lite3/README.md`、`gait_decision.py`、`stair_ahead.py`、契约模板三件、`boundary/protocol.py`、`message_transformer/ros2qnx.cpp`、`mode_navigation.launch`、`enable_motion`。真机链路 stays untouched。
- **未填**：任何真机实测数值。契约模板仍全为 null、仍不生效。
- **未签**：formal 验收、真机可用性、四足物理能力。仿真 PASS 不得当真机证据（Runbook §8）。
- **未碰**：`/v8/chassis/gait` 的真实下游。`shadow` 档下该话题输出与改造前逐字一致。

## 六、验收读数（本轮离线可复跑）

- 双版本对表：改造前节点备份 vs 改造后节点，4 场景 gait 序列均为 `flat_mid → high_step → flat_mid → high_step`，逐字一致。
- 模块自测：`python3 -B framework/gait_gates.py`、`python3 -B framework/terrain_observer.py`，各含主测＋注入有效性＋反控，均 `rc=0`。
- 定向单测：`test_gait_gates.py`（7）、`test_terrain_observer.py`（9）、`test_gait_guard_wiring.py`（8）；既有 `test_gait_decision.py`（7）、`test_stair_ahead.py`（4）、`test_execution.py`（34）、`test_framework.py`（31）全绿。
- 门禁：`SYN rc=0`（94 件）、`PYST rc=0`（62 件）。
- 一次性综合断言（含既有引用行号零漂移、双版本对表、影子接线活性）：`~/.cache/gait-nav-runtime-20261109/assert_all.py`，22 项全通过。

**参考**：字段字典与反算式出处 `GAIT_PARAM_CONTRACT.md`；离线决策矩阵与门控清单在 `~/.cache/gait-nav-design-20261109/`。

## 七、停车换步态（2026-11-09 后续批）

真人指令：接近台阶前停下来，然后切换步态。链路改为「先停稳、再换步态、确认后恢复」：

1. `gait_guard` 在期望步态变化时立即发布；之后在 `republish_window_s`（默认 5 s）内、且载体位姿连续 `stationary_s`（默认 0.5 s）稳定在 `stationary_eps_m`（默认 0.03 m）以内时，按 `republish_hz`（默认 2 Hz）重发同一期望步态。原因：`ros2qnx` 只在速度 ≤ `guard_gait_min_speed`（0.10 m/s）时才接受切换，且节点原先只在变化时发一次，停稳后不会补发；用位姿稳定而不是“指令已归零”来判定停稳，避免底盘还在减速时就切换。
2. 真机 graph 的 `local`（`local_native_raw`）打开 `gait_hold_enable`：收到期望步态变化后立即输出零速（reason=`gait_switch_waiting`），直到底盘 `ros2qnx` 的 `/v8/chassis/status` 报告 `gait == desired` 且持续 `gait_hold_settle_s`（默认 0.5 s）才恢复跟踪。
3. 失败面 fail-closed：状态超过 `chassis_status_stale_s`（1.5 s）不新鲜、或保持超过 `gait_hold_timeout_s`（5 s），继续零速并在 `/v8/local/status` 的 `gait_hold` 中报告 `gait_switch_status_stale`／`gait_switch_timeout`；不会自动放行。
4. 新接线：graph 增加 `gait_guard.gait → local.gait_desired` 边与 `local.chassis_status` 外部输入；`manifest_local_native_raw` 增加两个可选输入。仿真 graph 不含 `gait_guard`，`gait_hold_enable` 缺省关闭，仿真行为不变。
5. 前提：底盘桥 `ros2qnx` 必须以 `chassis_guard_enable:=true` 在同一 master 运行（`mode_navigation.launch` 的 `enable_motion:=true` 会带这个参数；独立 `message_transformer.launch` 缺省关闭，需显式设置）。桥不在或 guard 关闭时，遇到台阶会停在原地并保持，不会带着平地步态硬上。
6. 边界：停车由 `enter_m` 1.0 m 触发，实际停点还取决于检测延迟与底盘减速度；真机减速度、停止时距、切换时延均未实测，契约模板仍全 null。“停稳”判据用的是载体位姿 `/v8/local/pose`（hdl），不是底盘实测速度；定位抖动可能推迟重发，最坏情形是 local 保持到 `gait_hold_timeout_s` 后继续零速（fail-closed），不会自动放行。`/v8/local/status` 的 `gait_hold` 只是请求侧的保持状态，不证明底盘已经换步态。

离线读数（2026-11-09）：`test_gait_switch.py` 12 项、`test_pose_command_adapter.py` 52 项（含新增 hold 集成 1 项）、`test_gait_guard_wiring.py` 9 项全绿；`local_planner` 全量 158 项全绿；`cli.py check` rc=0、real app dry-run rc=0。真机未验证，G2/Q8 不变。
