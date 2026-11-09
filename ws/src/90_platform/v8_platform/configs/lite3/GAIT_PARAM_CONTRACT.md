# 「步态名 → 参数表」契约模板 · 使用与边界

> 状态：**TEMPLATE_ONLY / effective: false**。配套机器可读骨架为同目录 `gait_param_contract.yaml.template`。
> 本件是工程模板草案，不构成在册进度登记，不产生任何 PASS / FAIL 结论，也不新增真机运动许可。
> 最新更新时间：2026-11-09。

## 1. 这份模板解决什么

把真机侧实测的**各步态外部表现**数值化，回填给仿真侧的**运动学载体**作为扰动输入与能力约束，使建图、定位、导航这三条「位姿—观测」链路能在**无四足接触动力学**的条件下做有效仿真测试。

它不模拟步态本身。支撑相、接触力、落脚可行性、关节力矩、失稳恢复属于运控执行层，物理上不可由本表推出；本表只描述「这个步态在某种工况下，机体**表现出**什么速度能力、什么跟踪误差、什么时延与抖动、什么失效边界」。回填的价值在于把当前 `x + vx*dt` 的理想白盒载体升级成实测包络约束的灰盒，消除定位 / 建图仿真结果的系统性偏乐观。

**能签**：感知—规划链路在实测步态扰动下的鲁棒性与参数整定。
**不能签**：真机运控可行性、步态切换稳定性、四足物理能力。

## 2. 步态名全集与在册范围

步态名与厂商指令码均为仓内现读常量，来源 `ws/src/00_drivers/message_transformer/src/ros2qnx.cpp:40-46`、`:206-212`，**不是待填数值**。

| 步态名 | 厂商指令码 | 当前 in_scope | 说明 |
| --- | --- | --- | --- |
| `flat_low` | `0x21010300` | 否 | 平地低速档 |
| `flat_mid` | `0x21010307` | **是** | `gait_guard` 的平地默认步态 |
| `flat_high` | `0x21010303` | 否 | 平地高速档 |
| `crawl` | `0x21010406` | 否 | 爬行 |
| `grip` | `0x21010402` | 否 | 高抓地 |
| `general` | `0x21010401` | 否 | 通用 |
| `high_step` | `0x21010407` | **是** | `gait_guard` 的阶面步态 |

`in_scope` 依据 `graph_3d_lite3.yaml:95-96`：当前配置只配 `flat_gait: flat_mid`、`step_gait: high_step`，`stair_gait_guard_node.py:38-39` 也只会发出这两个名字。因此骨架**只展开这两个步态的完整字段块**；其余 5 个纳入回填时，复制 `gaits.flat_mid` 的四组字段块，并同步把 `gait_registry` 中对应的 `in_scope` 改为 `true`。不预先展开，是为了避免 5 份从未测过的空表被误读成「已覆盖」。

## 3. 字段字典

所有数值字段在模板态必须为 `null`。单位与统计口径如下；四项分组对应的仓内注入点见骨架 `injection_map`。

### 3.1 `envelope` 能力包络（硬约束）

| 字段 | 单位 | 统计口径与用途 |
| --- | --- | --- |
| `v_max_mps` | m/s | 稳态可达前进速度最大值，取多次 run 的下分位保守值；喂动态限速 |
| `v_reverse_max_mps` | m/s | 后退速度上限，同上 |
| `a_max_mps2` | m/s² | 加速段线性拟合；喂速度采样与轨迹可行性 |
| `omega_max_radps` | rad/s | 稳态原地转向角速度上限 |
| `step_max_m` | m | 可跨越最大阶高；「成功」判据须在作业单另行定义，不能由本值隐含 |
| `slope_max_deg` | ° | 可持续爬升坡度 |
| `turn_radius_min_m` | m | 稳态最小转弯半径 |
| `duty_max_s` | s | 连续运行不降额时长 |
| `width_clearance_min_m` | m | 最小通过净宽 |

注入去向：`cmd3d_mux_node.py:2-6`（唯一 `/cmd_vel` 写者，按步态名动态限速）、`graph_3d_lite3.yaml:85-100`（`gait_guard` 门限）。

### 3.2 `tracking` 跟踪误差（里程计先验）

| 字段 | 单位 | 统计口径与用途 |
| --- | --- | --- |
| `v_bias_mps` | m/s | 指令速度 − 真值速度，均值 |
| `v_std_mps` | m/s | 同上，标准差 |
| `omega_bias_radps` | rad/s | 航向角速度偏置均值 |
| `omega_std_radps` | rad/s | 航向角速度误差标准差 |
| `slip_ratio` | — | `1 − 真值位移 / 里程计位移`，打滑导致的系统性偏差 |
| `odom_scale` | — | 里程计尺度因子 |
| `lateral_drift_m_per_m` | m/m | 每米前进的横向漂移 |
| `yaw_drift_rad_per_m` | rad/m | 每米前进的航向漂移 |

注入去向：`kinematic3d_node.py:15-19`（`integrate()` 误差注入）、`gazebo_endpoint.py:24-30`（平面运动学积分）。这组决定定位先验与点云配准初值，是全套回填里对建图 / 定位结果影响最大的一组。

### 3.3 `latency_jitter` 时延与抖动（传感器与控制裕度）

| 字段 | 单位 | 统计口径与用途 |
| --- | --- | --- |
| `cmd_lag_mean_s` | s | 指令发出 → 机体起效，均值 |
| `cmd_lag_p95_s` | s | 同上 P95；喂停障余量与前视距离 |
| `switch_settle_s` | s | 步态切换请求 → 新步态进入稳态 |
| `gait_freq_hz` | Hz | 步频，即机体周期性振动基频 |
| `body_heave_amp_m` | m | 垂向周期振动幅值 |
| `body_pitch_amp_rad` | rad | 俯仰周期振动幅值 |
| `body_roll_amp_rad` | rad | 横滚周期振动幅值 |
| `imu_noise_scale` | — | 相对静置状态的 IMU 噪声倍率 |

注入去向：`kinematic3d_node.py:89-90`（stale 超时）、`gazebo_endpoint.py:108-115`（`SetModelState` 抖动注入）。机体振动三元组直接决定点云运动畸变与地图毛刺，是建图测试能否反映真机工况的关键。

### 3.4 `failure` 失效与降级

| 字段 | 单位 | 统计口径与用途 |
| --- | --- | --- |
| `switch_window_v_limit_mps` | m/s | 切换窗口允许的速度上限（零速则填 0） |
| `stop_distance_m` | m | 急停指令 → 完全静止的位移 |
| `degrade_target_gait` | 步态名 | 失效时退回的目标，须是 `gait_registry` 中已注册的名字 |
| `refusal_codes` | map | 拒绝码 → 语义映射，须与 `ros2qnx` 实际返回一致 |
| `unusable_conditions` | list | 该步态不可用的工况组合，元素为第 4 节的五元组 |

注入去向：`stair_gait_guard_node.py:38-39`（步态名与降级路径）。此组**只描述外部可观测行为**，不含支撑相完整性、落脚可行性、支撑多边形裕度等接触层量。

## 4. 工况、元信息与统计量口径

每个数值必须挂在**工况五元组**上：`surface`（地面材质）、`slope_deg`（坡度，阶面改记 `step_m`）、`speed_band`（速度档）、`payload_kg`（负载）、`surface_state`（干湿 / 磨损）。缺任一维的数值禁止入表。各步态的 `applicable_conditions` 记录它实际被实测覆盖的组合。

**不写工况的数值不得当全域常数使用**——拿平地数据判楼梯是这套回填最常见的失效方式。反向同理：`note/10_真机上机Runbook.md:231` 已明确「定位 0.22 m 是仿真带域值，不能当实机精度证书」。

`measurement_provenance` 记录测量元信息：`date`、`robot_id`、`firmware`、`sensor_suite`、`truth_source`、`truth_accuracy`、`sample_rate_hz`、`run_count`、`confidence_level`、`statistic_policy`、`operator`、`raw_data_ref`。其中 `truth_source` 与 `truth_accuracy` 必填：位姿真值来自动捕、RTK 还是全站仪，直接决定跟踪误差那组数值的可信上界。

**置信区间口径**：参数表只落点估计；每个数值的样本数 `n`、置信水平下的区间上下界、以及剔除异常 run 的规则，留在 `statistic_policy` 声明并由 `raw_data_ref` 指向的原始证据里。参数表不允许出现「只有一个 run、没有区间」的数值。

## 5. 填数规程

1. 复制 `.yaml.template` 为工作副本（见第 7 节启用流程），**不要直接改模板本体**。
2. 先填 `measurement_provenance` 与 `split`，再填参数表；没有出处的数值不落表。
3. 每个数值按第 4 节挂工况五元组，并确认该组合出现在对应步态的 `applicable_conditions` 里。
4. **禁止填示例值、占位真值或「看起来合理」的估计值**。任何非 `null` 数值都视为已完成的真机回填，须可回溯到原始证据。
5. 需要扩到其余 5 个步态时，复制 `gaits.flat_mid` 的四组字段块，保持字段名与顺序一致，再改 `in_scope`。
6. `calibration_runs` 与 `holdout_runs` 按 **run 整体**划分，交集为空；`holdout_runs` 少于 3 条不得出验收结论。
7. 每批填完跑一次第 8 节自检；模板态自检通过不等于填数后仍通过，填数后该脚本会按设计报红，这是预期行为。

## 6. 反自证三条硬规矩

1. **回填值只作扰动输入，不得同时充当验收判据。** 用实测误差模型生成位姿、再用同一组参数去「验证」定位精度，构成自证循环（空控 / 假绿）。验收必须用 `holdout_runs` 或异工况数据。
2. **禁止按帧随机切分留出集。** 同一 run 内相邻帧高度相关，按帧切会把同一段轨迹同时送进拟合与判据，等价于没有留出集。
3. **双轨分开报，不许互相顶替。** 仿真侧只报地图一致性 / 定位漂移 / 导航余量，真机侧只报步态切换稳定性 / 支撑相 / 力矩 / 能耗。在册禁止事项见 `Runbook §8`「把 V8-X1 仿真步态的 PASS 当真机可用性证据」，以及 `web/index.html` 03 页 B 区「不得由 SLAM 读数推出导航或四足物理能力」。

## 7. 启用流程（L2 关卡）

本模板**不生效**，也不自行接线。要真正投入使用，须依次：

1. 取得 L2 授权——契约 §9.3 把「真机及其参数」列为停手四类，`boundary/protocol.py:3-4` 写明真机链路 `stays untouched until an L2 authorization`。
2. 由现场作业单给出数值；`Runbook:230` 明确「现场不得自行填值，需单项作业单给值」。
3. 复制模板到生效件并改 `status` / `effective`，补进本目录 `README.md` 的文件清单；**在此之前不应被任何 launch / graph 引用**。
4. 按 `injection_map` 接线，且只做扰动注入，不改 `/cmd_vel` 唯一写者归属、不绕过 mux、不动真机链路。
5. 建 `note/10_真机上机Runbook.md:230` 所缺的「仿真→真机」对照件，把本表纳入对照；另补外参 / TF / 速度上限 / 地图版本。
6. 按第 6 节双轨验收，仿真结论只写「感知—规划链路」，不得外推运控或真机可用性。

另需注意：传感器外参目前仍是 `hdl_localization.launch:31` 的 0 占位「现场量」，`kinematic3d_node.py:22-35` 的 `snap_support` 把 z 钳到**声明支持面**而非感知结果。这两项不解决，回填数值也救不了仿真与真机的域差。MuJoCo 侧 `ONNX → 关节 PD → 力矩 → 接触` 链（`dynamics/README.md:47-49`）是另一条线，可作对照，不可替代本表用途。

## 8. 自检式（三连）

自检件已落盘为同目录 `check_gait_contract_template.py`，**零第三方依赖**（不导入 PyYAML），从仓根直接执行：

```bash
python3 -B ws/src/90_platform/v8_platform/configs/lite3/check_gait_contract_template.py
```

它按文件字面行而非 YAML 对象解析，所以在没有 PyYAML 的真机侧也能跑。判据落在「参数区有没有出现非 `null` 的值」这一层，并自带三连，防止判据自己空转：

| 环节 | 断言 |
| --- | --- |
| 主测 | 状态 `TEMPLATE_ONLY` / `effective: false`、步态名在册、72 个叶子全为 `null` |
| 注入有效性 | 把 `v_max_mps` 伪填成 1.2，判据必须报红；若印绿即为空控 |
| 反控 | 撤掉伪填值后必须恢复通过 |
| 空集拒判 | 参数叶子数不等于 60 直接报红，不允许「没扫到就算通过」 |

另附三条静态断言：模板字段与本文档第 3 节字段字典零漂移、`injection_map` 的 7 条路径全部命中真实文件、`ws/src` 下 `*.yaml` / `*.launch` 零引用且 `*.py` 除本自检件外零引用（自检件自身提及模板名属预期，显式排除）。

预期输出：

```text
主测 OK：模板空白（72 个 null 叶子）、状态 TEMPLATE_ONLY、步态名在册
注入有效性 OK：填入一个假值即被拦下
反控 OK：撤掉假值后恢复通过
字段一致性 OK：30 个字段，模板与文档零漂移
注入点存在性 OK：7 条引用全部命中真实文件
未生效性 OK：yaml/launch 零引用，py 除本自检件外零引用
```

**填数后这份脚本会按设计报红**——那是「模板已不再是空白模板」的正确信号，不是缺陷；此时应回到第 7 节走启用流程，而不是放宽断言让它变绿。

## 9. 引用源（均为仓内现读）

| 事实 | 位置 |
| --- | --- |
| 步态名与指令码全集 | `ws/src/00_drivers/message_transformer/src/ros2qnx.cpp:40-46, 206-212` |
| 当前在用的两个步态名 | `configs/lite3/graph_3d_lite3.yaml:95-96`、`framework/stair_gait_guard_node.py:38-39` |
| 运动学载体积分器 | `local_planner/kinematic3d_node.py:15-19, 22-35, 89-90` |
| Gazebo 位姿注入 | `boundary/gazebo_endpoint.py:24-30, 108-115` |
| 唯一 `/cmd_vel` 写者 | `local_planner/cmd3d_mux_node.py:2-6` |
| 位姿汇不证明接触 / 步态 | `boundary/README.md:45` |
| 参数迁移表缺口、禁止事项 | `note/10_真机上机Runbook.md:230-236` |
| MuJoCo 接触动力学链 | `ws/src/95_simulation/lite3_gazebo_sim/dynamics/README.md:47-49` |
| 真机链路 L2 边界 | `boundary/protocol.py:3-4`、`note/09_跨工具协作契约.md §9.3` |
