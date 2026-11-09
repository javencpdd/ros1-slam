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
