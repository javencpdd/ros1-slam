# 3D spike 设计确认 —— elevation→costmap 自研层接口设计（v1.0）

- 日期：2026-10-07｜起草：小明（施工域产出，供小张/主理人裁决）｜性质：**设计确认文档，本轮不实现任何代码**
- 依据：小美 Wave2 验收 D-W2-4 追认（noetic 无 TerrainPlanner，mesh 路线不可用）＋主理人采纳方向「自研 elevation→costmap 层（2.5D）」＋修正边界（诚实放弃项入册）；matrix_manifest v2 维度 3（T1r/T2r 判据件）；G-G8b 边界（规划/导航层仿真）。
- 参照契约：elevation_mapping_cupy（leggedrobotics，MIT；ROS2 Jazzy 主线，2026-10 官方文档定点核对，见 §2）。

## 1. 目的与范围
- 目的：给 T1r（高度图管线）与 T2r（TerrainPlanner 替代=代价引导规划）提供**最小可行的 2.5D elevation→costmap 层**接口设计，使「绕坡路径形状+路径长差」判据（C 文 X4 延伸口径）在本栈可执行。
- 范围内：节点接口（话题/帧/频率/消息/参数）、costmap 集成路线、判据对接、能力边界、预检清单。
- 范围外：实现代码、GPU/CuPy 依赖、真 3D 曲面/多层/悬挑规划（永久放弃项，见 §7）、真机越障（X4：归真机/MuJoCo）。

## 2. 参照契约（elevation_mapping_cupy，官方文档 2026-10 核对）
| 项 | 契约值 |
|---|---|
| 输入 | pointcloud 订阅（如 /camera/depth/points、/ouster_points_self_filtered，data_type=pointcloud；可多传感器）；位姿经 TF 以 update_pose_fps（10Hz）更新并移动地图中心 |
| 输出 | elevation_map（raw/filter 双发布，fps 3–10），消息=grid_map（grid_map_msgs/GridMap） |
| 图层 | 基础三层=elevation / variance / traversability（+插件后处理层：smooth/despiked/inpaint/…） |
| 参数 | resolution=0.1m、map_length=20m、base_frame=base_link、map_frame=odom（或 map）、update_variance_fps=5、map_acquire_fps=5 |
| 语义 | 单高度 2.5D 假设（每 xy 列一个高度值+方差），raycasting 清理可见性伪影；多层/悬挑不表达 |

## 3. 我方输入流（mode_nav_sim 实测锚点，G-G8b/13 CHECK 在案）
| 我方流 | 实测 | 对 elevation_mapping_cupy 契约映射 |
|---|---|---|
| /rslidar_points | 10Hz，PointCloud2 XYZI，velodyne 布局、time=0、frame=rslidar，sim_adapter 由 pcl_render_node sensor_cloud 直喂 | 对应 pointcloud 订阅；**源帧 rslidar 而非相机系**，需 TF 链完整 |
| TF map→base_link | sim_pose_tf @20Hz fresh-stamp（断流降级=沿用最后真值+5s throttle 告警）；base_link→rslidar 恒等静态 | 对应 map_frame=map、base_frame=base_link、update_pose_fps≈10–20Hz |
| /quad_0/body_pose | 50Hz 真值（on-rails） | 备选位姿源（真值直读，跳过 TF 抖动） |
| /Odometry | 真值里程计 | 备选位姿源 |
| 全局真值图 | sim_map_bridge→/map 400×400@0.1m（perlin3d 40×20 时同窗） | 静态底图参照；elevation 层与 static 层并存 |
| 平台约束 | Jetson Tegra 6 核、无独立 GPU（GPU 路线已被 G8sim R2 GLFW segfault 实证排除） | **CuPy/GPU 契约不取**，自研层走 CPU numpy（与 mockamap 116,747 点/帧量级匹配的预检见 §8） |

## 4. 最小可行层接口设计（w3_elevation_node，拟议）
### 4.1 订阅（输入）
| 话题 | 类型 | 用途 | QoS/频率 |
|---|---|---|---|
| /rslidar_points | sensor_msgs/PointCloud2 | 逐帧增量融合（机器人系→map 系经 TF） | 10Hz |
| TF（map→base_link→rslidar） | tf2 | 云变换到 map 系＋位姿外推 | 20Hz 源 |
| /quad_0/body_pose（可选 pose_source=truth） | geometry_msgs/PoseStamped | 位姿直读备选（on-rails 真值，零 TF 依赖） | 50Hz |
### 4.2 发布（输出）
| 话题 | 类型 | 内容 | 频率 |
|---|---|---|---|
| /elevation_grid | nav_msgs/OccupancyGrid | 2.5D 通行代价栅格（0.1m；0=通行/…/100=不可通行/−1=未知=未覆盖列） | 1–2Hz latched |
| /elevation_map_layers（可选增强） | grid_map_msgs/GridMap | elevation/step/roughness/NaN 四层（需 apt ros-noetic-grid-map-msgs，**预检项**） | 1–2Hz |
| /elevation_stats | diag/diagnostics 风格 std_msgs/String | NaN 率、覆盖格数、更新时延（判据采数+健康心跳） | 1Hz |
### 4.3 核心参数（yaml，壳包规范位 lite3_bringup/config/）
- resolution=0.1（与 sim_map_bridge/static 层同格）；window= Venue 包络（maze2d 12→16m、perlin3d 40×20→44×24m，对齐 W2 勘察封边模板语义）或 map_frame 定窗二选一；
- cell 统计：高度=覆盖点 z 的 P50（抗毛刺），step=max|Δh| 邻接 8 域，roughness=列内 z 一阶差分 RMS；
- 通行代价映射（三通道判据直挂 C 文 D 表 T1r 行）：slope>25° 或 step>0.15m 或 roughness>阈值→cost 100；未知列（零覆盖）→−1 未知（**不做 W2「其余=free」同款假设**——N1 教训写死：未覆盖≠自由）；
- NaN 率判据：单帧更新后 NaN 占比>30% 记 WARN（T1r 判据件）。
### 4.4 语义与红线
- 估计层排除边界（坑14）不变：本层消费 sim 真值链点云，不做任何状态估计；/cmd_vel 零接触；ros2qnx 零引用。
- 输出仅作为 costmap 输入件，规划语义改动（ inflation/代价曲线上墙）=壳包设计变更，须小张登记 04。

## 5. costmap 集成路线（两档）
| 档 | 方案 | 改动 | 适用 |
|---|---|---|---|
| A（最小可行，推荐起步） | global_costmap plugins 增第二实例 `{name: elevation_layer, type: costmap_2d::StaticLayer}` + `elevation_layer.map_topic: /elevation_grid`（noetic StaticLayer 支持 map_topic 参数；多实例同名类型不同名可挂） | mbf_navigation.yaml/global_costmap_params.yaml 1 处注册段（预登记 04 diff） | T1r/T2r 全部判据 |
| B（插件级，二期观察） | 自研 costmap_2d::Layer C++ 插件（滚动窗局部更新+滚出回收） | 新包+编译链 | 若 A 档全图重发带宽/时延不达标再启 |

## 6. 判据对接（C 文 D 表）
- T1r：elevation 更新成功/NaN 率/三通道（坡度/台阶/粗糙度）——由 /elevation_stats+/elevation_map_layers 采数；
- T2r：绕坡路径形状+路径长差——GetPath 于 /elevation_grid 注入后的 costmap 上采数（与 W2 T1 harness 同构，planner 按名点单）；
- A0 RTF 门 0.25 逐跑必探（W3 harness 口径沿用）。

## 7. 能力边界（诚实入册，小美修正边界原文）
- 2.5D 单高度假设=**永久放弃**悬挑/多层/桥洞场景规划能力；陡坎檐口高度混叠；
- 仿真仅证规划质量（路径形状），真越障归真机/MuJoCo（X4 裁决）；
- 若二期真机越障出现多层场景→mesh_navigation ROS2 重评估（F8 观察项）。

## 8. 预检清单（W4+/二期开工前置，全部 /tmp 试做）
1. grid_map_msgs 是否在装（不在→apt 申请或降级纯 OccupancyGrid 路线，A 档不依赖它）；
2. 吞吐探针：python numpy 单帧 116,747 点×10Hz 增量融合+1Hz 全图重发的 wall 耗时（Tegra 实测，超 0.5s/帧→降 res 0.15 或降 fps）；
3. costmap_2d StaticLayer 双实例同栈实测（maze2d 上零行为变化锚点=W2-0 同款 13 CHECK）；
4. 与 sim_map_bridge 的 /map 并存语义（unknown=−1 与 0 的 costmap 解释差异核验）。

## 9. 验收判据建议（G3a 式）
- 接口逐项与本 §4 表一致；§8 预检 1–3 通过；maze2d 零默认行为变化锚点全 PASS；perlin3d 上 T2r 采数一跑（绕坡/直线对照）形状可辨、A0 RTF≥0.25。
- 本文档 sha256 于落账时登记 04；实现开工前须小张接口复核+主理人批准（设计域裁决）。
