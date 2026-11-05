# lite3_bringup · 传统 ROS 顶层编排

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包按 mode include 驱动、建图/定位、MBF、M6 和 SUP，提供传统 ROS 顶层装配。

| 入口 | 组合/职责 |
| --- | --- |
| `launch/mode_mapping.launch` | SUP + 传感器 + faster-lio |
| `launch/mode_localize.launch` | SUP + 传感器 + HDL + M6 |
| `launch/mode_nav_stub.launch` | SUP + 传感器 + M6 + MBF + stub TF；不是真实定位验收 |
| `launch/mode_nav_sim.launch` | mockamap 仿真链 + 地图/位姿桥 + M6 + MBF |
| `launch/mode_sim_hdl.launch`、`mode_sim_teb.launch` | 引入 Gazebo M2 定位/导航测试台 |
| `launch/mode_navigation.launch` | 真机 SUP/传感器/HDL/M6/MBF 与超声限速；`enable_motion` 默认 false，只有显式启用才起 `ros2qnx` |
| `launch/mapping_viz.launch` | LIO 快照和机体可视化伴侣，不是 GT 点云记录器 |

仿真地图/位姿桥在 `scripts/`，简化模型在 `urdf/Lite3_prim.urdf`；
`config/elevation_costmap.yaml` 是 M3 自研高程节点的私有 `~elevation/...` 参数规范位。
TOP 可以下行组合各模块，其他平级壳包的互 include 仍受隔离规则约束。

仿真 GT 键盘建图及 v8 app 导航入口在 `lite3_gazebo_sim` 与 `v8_platform`，不在本包。
真机导航仍须单项窗口、现场条件与 `scripts/preflight_navigation.sh` 前置；
默认关闭运动桥和静态配置存在均不能作为真机验收结论。
