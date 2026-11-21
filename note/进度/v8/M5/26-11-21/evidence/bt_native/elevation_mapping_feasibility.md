# ANYbotics elevation_mapping 源码构建可行性（2026-11-21，隔离 scratch）

- 结论：**可构建**（容器 ros1-slam-sim-viz，cmake 3.16.3），且输出与现有消费层兼容。
  仅在 /tmp scratch 验证；未改仓、未改镜像、未接入运行链。
- 依赖二进制：apt `ros-noetic-grid-map*` 1.6.4（含 filters/cv）已装；eigen_conversions、
  pcl_ros、tf、tf_conversions、message_filters、libeigen3-dev、libpcl-dev 均已装；
  `ros-noetic-elevation-mapping` 无二进制发布。
- 源码与固定提交（shallow clone，gh-proxy）：
  - elevation_mapping `f4b082c64a3e660980da53b33c7936a8f2a2ea22`（master，v0.7.25，上游标 maintenance-discontinued）
  - kindr `32800890`（2025-02-15，pre-refactor）、kindr_ros `8d60e3f8`（2025-08-26）、
    message_logger `1ea41e48`（2026-09-02）——配 stock CMake 3.16.3，`catkin_make -j4` rc=0
  - ★注意：上述仓库 master 已于 2026-11 改为 format-3/CMake≥3.28；直接取 master 需另配 CMake 3.28
    （scratch 用 pip wheel 验证也可 rc=0，但推荐固定旧提交、不动镜像工具链）。
- 运行契约（源码 README/demo 配置）：
  - 输入：`input_sources/<name>/topic`（sensor_msgs/PointCloud2）、`sensor_processor.type: perfect|laser`；
    位姿走 `map`→`base` TF 或 `robot_pose_with_covariance_topic`。
  - 输出：`/elevation_mapping/elevation_map`（grid_map_msgs/GridMap），融合层名 `elevation`
    —— 与本仓 `ElevationMergedLayer` 的 `input_topic/layer_name` 缺省逐字一致，消费层无需改。
  - demo 地图仅 2.5×2.5 m @0.05 m，接入前需按 costmap 覆盖放大 `length_in_x/y`。
- 接入待办（下一片）：四包原样入 `30_3rd`（第三方位）+ profiles 闭包 + Dockerfile 构建 +
  `elevation_source:=native|elevation_mapping` 切换 + 同判据对跑；镜像重建与真机 TTL 另行登记。
- 备选（二进制可得但非高程 GridMap）：STVL 1.4.5、octomap_server 0.6.8、costmap_converter 0.0.13。
