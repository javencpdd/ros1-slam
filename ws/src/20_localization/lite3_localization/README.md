# lite3_localization · HDL 定位装配

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包装配第三方 HDL 全局地图与配准 nodelet，提供独立的定位输出，不包含自己的配准算法。

| 入口 | 实际职责 |
| --- | --- |
| `launch/hdl_localization.launch` | nodelet manager、`GlobalmapServerNodelet`、`HdlLocalizationNodelet`；可选全局定位与状态图 |
| `launch/hdl_m1_tracking_smoke.launch` | m1 静态地图跟踪探针，HDL TF 重映射到专用命名空间 |
| `launch/hdl_m1_global_smoke.launch` | m1 全局定位探针，同样隔离 HDL TF |
| `verify/v_fd_h_name_contract.sh` | `/odom`→`/hdl_odom` 名字与 frame 契约的静态/运行检查入口 |

主 launch 订阅的默认点云为 `/rslidar_points`、IMU 为 `/imu/data`，将 HDL 原生 `/odom`
重映射为 `/hdl_odom`。`points_topic/imu_topic/use_imu/use_global_localization` 等通过 launch
参数选择；地图读取 `/home/ros1-slam/data/maps/<map_name>.pcd`。

当前 `config/` 无参数文件；HDL 参数写在本包 launch 内，不从 `faster_lio_sim.yaml` 读取。
默认传感器外参仍需根据实际装配核对，静态参数/话题存在不代表定位精度或重定位已经验收。
仿真 M2 判据入口仍为 `lite3_gazebo_sim/verify/m2_smoke.sh`；正式/真机结论以对应证据为准。
