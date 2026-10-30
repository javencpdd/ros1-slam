# 建图 / 定位 / 全局 / 局部 / 中间件的更换机制

> 日期：2026-10-30 · 当前态 · 事实权威仍为代码与 `note/`
> 结论先行：**换 MBF 插件最容易（改名单＋按名点单）；换建图/定位靠壳包 launch 与话题契约；中间件只能换桥、不能换总线；v8 注册表目前是设计层校验网，不是运行期插件总线。**

## 一、三层更换机制

| 层 | 怎么换 | 管到哪 | 证据 |
| --- | --- | --- | --- |
| 运行层（今天真用） | roslaunch 的 include/arg/remap ＋ MBF 的 planner/controller 名单 ＋ 配置 yaml | 换节点、换插件、换话题、换参数 | `ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch:9-15` |
| 壳包约定（代码组织） | 第三方算法原样放独立包；`lite3_*` 壳包只放 launch/config/remap；只经话题/TF 相接，禁跨包 include | 保证“换算法不改编排” | `ws/README.md:150-153` |
| v8 注册/组合层（设计层） | manifest 声明角色/io/能力 → composer 查角色、TF、命令所有权、运动学配对 → 锁文件 | **只做设计校验，未接运行装配** | `ws/src/90_platform/v8_platform/composer/composer.py:25-50`；`ws/src/90_platform/v8_platform/README.md:40-43` |

## 二、五类逐项

### 1. 建图（mapping）

| 项 | 现状 |
| --- | --- |
| 可选后端 | 只有 faster-lio 一套在线建图；`pcd2grid` 是离线点云转栅格，不接在线话题 |
| 当前装配 | `mode_mapping.launch` → `lite3_slam/launch/10_slam_mapping.launch` 加载 `faster_lio_outdoor.yaml` 并起 `run_mapping_online` |
| 接口契约 | 入 `/rslidar_points`、`/livox/imu`；出 `/Odometry`、`/cloud_registered`、TF |
| 怎么换 | 新建 `10_slam/<算法>` 包 → 在 `lite3_slam` 壳里加 launch/config → 保持上述话题与 TF → 改 `mode_mapping.launch` 的 include → 重跑门禁与 `note/11` 登记 |
| v8 设计层 | 只有 `odometry` 清单（faster_lio、lite3_slam），**没有 `mapping` 角色清单** |
| 风险 | 无第二后端可对照；换包要重标外参；建图是 `map→odom` TF 链源头，换错会污染定位与代价图 |

证据：`ws/src/10_slam/lite3_slam/launch/10_slam_mapping.launch:3,6`；`ws/src/70_bringup/lite3_bringup/launch/mode_mapping.launch:33`；`ws/README.md:115`。

### 2. 定位（localization）

| 项 | 现状 |
| --- | --- |
| 可选后端 | ① faster-lio 里程计直接当定位（nav sim 常用）；② hdl_localization 点云重定位 |
| 当前装配 | `mode_sim_hdl.launch` / `mode_localize.launch` include `lite3_localization/launch/hdl_localization.launch` |
| 接口契约 | hdl 壳内把 `/velodyne_points` remap 到 `/rslidar_points`；把第三方 `/odom` remap 成 `/hdl_odom`；`map→odom` TF |
| 怎么换 | 在 `lite3_localization` 壳里加 launch → remap 到 `/rslidar_points` → 输出固定话题（优先 `/hdl_odom` 契约）→ 用 `mode_localize`/`mode_sim_hdl` 选择 → 对齐 `map→odom` |
| v8 设计层 | 两个 `global_localization` 清单（hdl、lite3）＋ odometry 角色依赖 |
| 风险 | 没有统一 `localization` 接口；换后端要改 mode launcher、TF 与初始位姿流程；重定位质量不在 composer 校验范围 |

证据：`ws/src/20_localization/lite3_localization/launch/hdl_localization.launch:76,84`；`ws/src/70_bringup/lite3_bringup/launch/mode_sim_hdl.launch:17`；`mode_localize.launch:34`；`ws/README.md:117`。

### 3. 全局算法（global planner）

| 项 | 现状 |
| --- | --- |
| 可选后端 | MBF `planners` 名单：`GlobalPlanner`（默认）、`NavfnROS` |
| 当前装配 | `mbf_navigation.yaml` 的 `planners` 列表；`planner_teb.yaml` 与 `global_planner_params.yaml` 由 `mbf_navigation.launch:9-15` 加载 |
| 选择方式 | 默认取列表第一项；也可在 `GetPathGoal.planner` 按名点单 |
| 怎么换 | 改 `planners` 的 `{name, type}` → 补参数 yaml → 重启 MBF → 按名点单验证 |
| v8 设计层 | `v8.global_planner.apt` 清单；composer 检查地图供给角色（map_server/representation/mapping 任一）与 TF/命令双写 |
| 风险 | GlobalPlanner/Navfn 只注册 nav_core，需 MBF 的 nav_core 适配层加载；v8 不在运行链上 |

证据：`ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml:5-10`；`mbf_navigation.launch:9-15`。

### 4. 局部规划算法（local planner / controller）

| 项 | 现状 |
| --- | --- |
| 可选入口 | ① MBF `controllers`：TEB（默认）、DWA；② SCAN 规划器（独立话题）；③ A5 局部规划（实验链） |
| MBF 选择方式 | 默认列表第一项；`ExePathGoal.controller` 按名点单（DWA 复验即此路径） |
| SCAN 路径 | faster-lio → `/cloud_registered` → `plan_manage` → `/scan_planner/cmd_vel` → mux 第七源 |
| A5 路径 | 支撑图 → `local_native`+`ros_node` → 私有前缀 `/cmd`、`/trajectory`、`/status` → MuJoCo 桥或 3D 载体 |
| 怎么换（MBF） | 在 `controllers` 名单加项 → 补 `planner_*.yaml` → 用 `ExePathGoal.controller` 点单 |
| 怎么换（SCAN/A5） | 换 bringup mode 或换独立 runner；保持 mux 源话题或私有前缀契约 |
| v8 设计层 | 两个 local_planner（lite3_scan、scan_planner）＋两个 controller（teb、dwa）；composer 做四足×差速/全向配对 |
| 风险 | 三套入口不统一：MBF 插件、SCAN 话题、A5 私有前缀；A5 未进注册表 |

证据：`mbf_navigation.yaml:12-19`；`ws/src/40_navigation/lite3_navigation/verify/dwa_recheck.py:137`；`ws/README.md:118`；`ws/src/90_platform/v8_platform/regression/a5_v8x1_runtime.py:164-174`。

### 5. 中间件

| 项 | 现状 |
| --- | --- |
| ROS 侧 | ROS1 Noetic + TCPROS，一次运行一个 master；没有 ROS2/DDS 运行面 |
| 真机桥 | `message_transformer`：`qnx2ros` 监听 QNX 遥测 UDP 43897 → `/imu/data`；`ros2qnx` 订阅 `/cmd_vel` → UDP 43893（**仅真机导航 mode**） |
| 仿真侧 | `sim_adapter.py` 合成 `/rslidar_points`、`/livox/imu`；Gazebo 插件提供传感器 |
| 怎么换 | 换桥节点/包但保留 ROS 话题契约；换 launch mode 选择不同桥；真机 `ros2qnx` 的位置与禁令不动 |
| 新边界 | `boundary/` 是候选机器人边界（S1 回环完成），**未接 mux、未接虚拟/真机端点** |
| 风险 | 不能无痛换 ROS2/DDS；真机链路是厂商固定路径；换定拍 UDP 属 L2 |

证据：`ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch:9-10`；`ws/src/00_drivers/message_transformer/src/ros2qnx.cpp:44,112`；`ws/src/95_simulation/lite3_gazebo_sim/dynamics/a5_v8x1_bridge.py`（仿真另走进程内桥）；`note/00_README.md:18,23-29`。

## 三、今天的实操路径（按顺序）

1. **建图**：加算法包 → 加壳包 launch/config → 保持话题+TF → 改 `mode_mapping` include → 门禁+`note/11`。
2. **定位**：加壳包 launch → remap 到 `/rslidar_points` → 输出 `/hdl_odom` 契约 → 用 mode 选择 → 对齐 `map→odom`。
3. **全局规划**：改 MBF `planners` 名单/类型 → 补参数 → 按 `GetPathGoal.planner` 点单 → v8 侧补 manifest（设计校验）。
4. **局部规划**：改 MBF `controllers` 名单/类型 → 按 `ExePathGoal.controller` 点单；或独立规划器发布到 mux 源话题。
5. **中间件**：换桥节点不换话题契约；真机桥保持 `ros2qnx` 禁令；UDP 边界先接仿真端点（S2/S3）。

## 四、为什么还不能“一键换”

- **没有运行时装配器**：composer/orchestrator 只有 `SimulatedRunner` 验证；`v8_platform/README.md:40-43` 明确“apt 后端与 SCAN 运行面未实装运行”。
- **没有机器闸**：`ws/README.md:150-153` 承认“一算法一包、禁跨包 include/depend”目前无人守。
- **清单不全**：13 份 manifest 里没有 `mapping` 角色；A5/MuJoCo 完全绕开注册表。
- **判据未闭环**：A6“同任务接口换后端”只有设计层证据（`ws/src/90_platform/v8_platform/adapters/README.md` 的 io 匹配与 hdl⇄lite3、dwa⇄teb 接线同形），没有运行等价性。

## 五、建议的下一步

1. **S2**：把 mux→boundary→MuJoCo 虚拟端点接通，先让 `/cmd_vel` 走定拍 UDP，再做回放。
2. **补 `mapping` 角色 manifest**：至少让 faster-lio 作为 mapping 后端进注册表，composer 才能组合建图链。
3. **把 A5/SCAN 纳入 `local_planner` 角色**：统一“私有前缀 + mux 源话题”的接口描述。
4. **给壳包约定加机器闸**：检查跨包 include 与 `package.xml` 互 depend（需门禁裁定）。
