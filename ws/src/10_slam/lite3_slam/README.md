# lite3_slam · faster-lio 装配

> 最新更新时间：2026-11-06 15:17:09 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包提供 faster-lio 启动与参数装配；算法本体在第三方包 `faster_lio`。

| 入口 | 实际配置 |
| --- | --- |
| `launch/10_slam_mapping.launch` | 加载 `config/faster_lio_outdoor.yaml`，启动 `faster_lio/run_mapping_online` 与 M1 心跳 |
| `launch/10_slam_mapping_sim.launch` | 加载 `../../95_simulation/config/faster_lio_sim.yaml`，显式设置 `use_sim_time=true`，不带 M1 心跳 |

两套配置都读取 `/rslidar_points` 与 `/livox/imu`，但点格式不同：outdoor 为
`lidar_type=4`，仿真为 `lidar_type=2`、`time_scale=1e3`。不能把配置名或话题相同当作布局相同。
`lite3_gazebo_sim/launch/m1_mapping.launch`、`m2_mapping.launch` 仍加载 outdoor 配置，
仿真 LIO 应选本包的 sim 入口并核对实际点布局、时钟与 TF。

faster-lio 存盘落点为 `data/maps/lite3.pcd`，实验前后按现行证据规则备份和恢复，
不要覆盖既有样张。第三方源码只允许已登记的存盘路径隔离改写。

GT 位姿建图另走 `lite3_gazebo_sim/scripts/mapping_recorder.py` 与
`v8_platform/framework/map_from_pcd.py`；它不等于本包的 LIO/SLAM 验证。

PCD转二维栅格使用[pcd2grid.launch](../pcd2grid/launch/pcd2grid.launch)。上游README中的build_octomap.launch、map.launch与config.yaml不在当前副本中；上游原文保留，不据旧示例判断本仓入口。
