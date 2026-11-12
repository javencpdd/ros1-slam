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
