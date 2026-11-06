# lite3_drivers · 传感器与仿真数据接入

> 最新更新时间：2026-11-06 15:17:09 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包组织传感器起链和仿真消息适配，不实现 SLAM 或底盘控制算法。

| 入口 | 实际职责 |
| --- | --- |
| `launch/01_sensors.launch` | include Livox Mid-360 驱动，启动 `qnx2ros` 遥测接收和 M0 心跳 |
| `launch/95_sim.launch` | 组合 `mockamap`、`local_sensing_node` 和 `sim_adapter.py`；可选 GPU/CPU 渲染，参数为 `map_type/map_seed/map_x/map_y/map_z/map_res/map_traj` 等 |
| `scripts/sim_adapter.py` | 把仿真点云重排到 `/rslidar_points`，按模式合成 `/livox/imu` 或发布声明轨迹 |
| `config/traj_maze.yaml` | `95_sim.launch` 的默认轨迹输入 |

`95_sim.launch` 是 mockamap 渲染链，没有 `world` 参数。Gazebo SDF 世界入口在
[`lite3_gazebo_sim`](../../95_simulation/lite3_gazebo_sim/README.md)，两者不能混用。

真机驱动会接硬件与遥测 UDP，需按现场作业窗口使用。`ros2qnx` 是反向运动桥，
本包传感器入口不启动它；禁止在 stub、mapping、replay 等非真机运动模式起反向通道。
