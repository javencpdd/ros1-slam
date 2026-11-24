# 真机迁移准备（lite3 / M20）

> 本件是**运行准备**，不是上机许可。物理真机运行属 L2：需 `note/10` 作业窗/单项作业单签署、
> 原版 preflight rc0、真人在场。lite3 读侧只读；不得向 `120:43893` 发帧；M20 属未授权扩展（V81-11）。

## 1. 已就绪（可离线验证）

- `graph_3d_lite3_bt.yaml`：lite3 真机 BT 编排图，`execution_mode=real_robot, robot_id=lite3`，
  9 实例；`path_goal` 已移除，`FollowPath` 直写 `/v8/local/goal`。
  离线校验：`cd framework && python3 cli.py check fixtures/manifests ../configs/lite3/graph_3d_lite3_bt.yaml`
  → `ok: true`（digest `c3ad3d01…`），启动序 `… global → bt → local …`。
- `v8.fw.bt_console`：`modes=[simulation, real_robot]`，仿真与真机复用同一棵 `navigate_with_planner.xml`。
- 仿真全链已跑通（`note/进度/v8/M5/26-11-12/evidence/BT-ORCH/upstairs-bt`）：
  BT 事件 `goal → request → path ready(57) → follow → dispatch`，载体距目标 `0.19 m`。
- 端口真源 `verify/port_truth.json`（活栈 33 主题）+ `verify/port_conformance.py`：声明与实测对表。

## 2. lite3 上机前仍缺（软件/证据，不是设计）

- 作业窗与单项作业单未签（`note/10`），G1—G5 门值待真人。
- `hdl_localization.launch` tf 四项仍 0.0 占位；真机 PCD 未建；定位收敛信号仓内无生产者（影子门恒 `insufficient_data`）。
- 建图与导航须分两次上机：先 `enable_motion=false` 只建图 → 定位空跑 → 最后才 `enable_motion=true`。
- **容器 preflight 路径别名漂移**：item[4] 要求 rospack 字面在 `/home/ros1-slam`，item[10]/[9-v8] 以
  `/opt/ros1-slam` 为规范别名；需把 `/home/ros1-slam/ws/src` 修为指向 `/opt/ros1-slam/ws/src` 的符号链接
  （有删实目录风险，未擅动）。否则正式窗 preflight rc=5，本平台只能出 SMOKE、不能声称 formal。

## 3. M20 迁移合同（无代码，先立接口）

M20 当前仓内无代码。要复用本平台，按角色补组件并注册 manifest：

| 角色 | 必须满足的最小接口 |
| --- | --- |
| carrier / middleware | 发 `/v8/local/pose` (PoseStamped)、`/v8/local/localization_ok` (Bool)、`/v8/3d/carrier/status` (String)；收 `/cmd_vel` (Twist) |
| global_localization | 提供定位位姿与 `localization_ok`；TF `map→base_link` 单一所有者 |
| safety_arbiter | 单写者 `/cmd_vel`；超时/夹停；hold 输入（如 `/lite3/supervisor/hold`） |
| odometry / mapping | 保留源 stamp 与真实变换；地图 `support_map/v1` 或 `occupancy_grid/v1` 与维度一致 |
| 步态（若有） | `/v8/chassis/gait` (String) 与 `/v8/chassis/status`；无步态可从图里去掉 `gait_guard` |

- 图模板：复制 `graph_3d_lite3_bt.yaml`，换 `carrier`、`robot_id`。
- **唯一明确的代码门**：`framework/execution.py` 的 `context()` 目前对 `real_robot` 只放行 `robot_id=='lite3'`
  （否则 `E_EXEC_ROBOT`）。接 M20 必须先放开该域——属**新决策族，需授权**。
- BT 树、`global3d_topic`、`local_native(_raw)`、`mux`、`perception`、`gait_guard` 与机器人无关，可直接复用。

## 4. 复现（离线）

```bash
cd ws/src/90_platform/v8_platform/framework
python3 cli.py check fixtures/manifests ../configs/lite3/graph_3d_lite3_bt.yaml
python3 -m unittest test_bt_graph_assembly test_sim_backends
cd ../verify && python3 -m unittest test_port_conformance
```

## 5. 2026-11-24 复核：§1 第一行那个「已就绪」是错的

本节更正上文的判断。**`path_goal` 不是可省的优化，缺了它真机只会走一步。** 未改图，先立据。

证据链（全部离线可复跑，不涉真机）：

1. `lite3_bt` 的 `FollowPath` 是 **`BT::SyncActionNode`**（`bt_console.cpp:472`），`tick` 里
   `goal_pub.publish(goal)` **只发一次**（`:522`），随后返回 SUCCESS，序列进入 `wait_arrival`。
2. 真机局部执行 `v8.fw.local_native_raw` 的输入口只有 `goal`，**没有 `path`**
   （`framework/fixtures/manifests/manifest_local_native_raw.json`），与仿真同源，
   是**步进式**的：一个 goal 走满 `max_step_m` 就 segment_complete。
3. 真机配置 `local_native.yaml` 的 `max_step_m: 0.15`、`tick_s: 0.5`。
4. 于是按同一组件的实测行为推断：**真机每段只走 0.15 m 就停**，BT 还要等 `advance_timeout`
   或 `retry_max` 才重试，表现为"走走停停、几乎不前进"。

仿真侧已经踩过并修掉了同一个坑：`graph_3d_m1_hdl_bt.yaml` 里加了 `v8.fw.path_goal3d`
沿全局路径逐点重发，并在该图的 `bt` 段注释里写明原因（"FollowPath 只发一次 ⇒ 车只走 0.30 m"）。
该修法**可直接搬到真机图**：`manifest_path_goal3d.json` 的 `execution.modes` 含 `real_robot`。

建议改法（**未执行，属真机件，待授权**）：

- 在 `graph_3d_lite3_bt.yaml` 增 `path_goal` 实例（`component: v8.fw.path_goal3d`，
  `path_topic: /v8/3d/global/path`、`goal_topic: /v8/local/goal`）；
- 把 `bt.out_goal_topic` 改为 `/v8/bt/out_goal`（与仿真同名，不再直写 local），
  边改 `global.path → path_goal.path`、`path_goal.goal → local.goal`；
- 改完必须重跑 `cli.py check`，并要求 digest 变化被记录。

## 6. 2026-11-24 复核：其余口径与今日实测

- `cli.py check .../graph_3d_lite3_bt.yaml` 今日仍 `ok: true`，digest **`e09e2b2e…`**。
  §1 引的 `c3ad3d01…` 是旧值，图已变更过，引用时以现跑为准。
- lite3 构建闭包（`scripts/build/target_workspace.py plan --target lite3`）今日 **27 包**：
  含 `v8_algorithm`／`v8_platform`／`v8_grid`／`lite3_bt`／`lite3_slam`／`faster_lio`／
  `hdl_localization`／`hdl_global_localization`／`lite3_navigation`／`lite3_safety`／
  `lite3_supervisor`／`lite3_scan`／`lite3_drivers`／`lite3_bringup`／`v8_ndt_odom`／
  `message_transformer`／`livox_ros_driver2`／`pipeline` 及 3rd 件。
  **不在闭包**：`v8_simulation`、`lite3_gazebo_sim`、`elevation_mapping`、`mockamap`、
  `pcd2grid`、`lite3_description`、`lite3_perception`。
  `host_only.robot_forbidden=true`（`web`、`subprojects/console`、`data/console` 已由规划器拦）。
- 建图后的 PCD → 支撑图转换件 `framework/tools/map_from_pcd.py`：依赖 `numpy` 与同目录的
  `paths`／`gazebo3d_mapping`／`support_map`，**全部在 `v8_platform` 内**（numpy 随 Noetic 自带）
  ⇒ **在闭包内、可在真机原生跑**，不需要开发机或容器。
- 仍缺（与 §2 一致，未变）：`hdl_localization.launch` 的 `tf_x/tf_y/tf_z/tf_pitch`
  四项仍是 `0.0` 占位（`:32—35`），须现场量；真机地图目录 `data/maps/real_lite3_20261103/`
  被 `.gitignore:13` 排除、且是 11-03 样张，**不等于现行真机地图**。
