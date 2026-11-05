# lite3_perception · 高程与三态栅格

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包已有自研 `scripts/w3_elevation_node.py`，不是空壳，也不是完整的多层地图或感知导航系统。

该节点订阅 `/rslidar_points`，以 `ElevGrid` 累积列统计，按 slope、step、roughness 生成
`/elevation_grid`（`nav_msgs/OccupancyGrid`）与 `/elevation_stats`（`std_msgs/String`）。
当前 `cost_map()` 实现为：未覆盖列 `-1`，有覆盖且未越阈值为 `0`，越阈值为 `100`。
这是现实现语义，不据此代签旧三态判据或 formal。

参数读取私有 `~elevation/...`，规范参数文件在
`../../70_bringup/lite3_bringup/config/elevation_costmap.yaml`。
其中 slope 25°、step 0.15 m、roughness 0.05 m 均为未标定施工值，不能作为真机通行能力上限。

`python3 scripts/w3_elevation_node.py --selftest` 是不连 ROS 的自测入口；
设计合同另见 `lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh`。

第三方 `deeprobotics_local_height_map` 仍受其独立来源/禁编规则约束。
本包二维三态栅格与 v8 多层支撑图是不同表征，不能互相替代能力结论。
