# lite3_scan · SCAN 装配与到达观测

> 最新更新时间：2026-11-12 10:33:00 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包提供 SCAN 上游五包的自有 launch、参数外化、消息适配与验证入口；
`ws/src/30_3rd/` 中上游源码保持基线，当前包无算法编译目标。

| 入口 | 实际职责 |
| --- | --- |
| `launch/70_scan.launch` | include 上游 `plan_manage/launch/advanced_param.xml` 并装配自有参数/到达观测 |
| `lite3_bringup: launch/scan_bt_nav.launch` | SCAN 接进 v8 BT：BT 的 RequestPath 直发 `/scan_planner/goal`，桥把 `/planning/bspline` 回填 `/global_path`，到达走 `/lite3/scan/arrived` |
| `lite3_bringup: launch/scan_nav_sim.launch` | SCAN 导航仿真单入口：`profile:=flat`（m1_world+A*）或 `profile:=ramp`（ramp_scan10+参考路径+地面分割+坡面参数），整链一次起完 |
| `lite3_bringup: launch/scan_nav_real.launch` | SCAN 导航真机装配（**真机物理运行 L2**；bringup/LIO/safety 由操作员先起；见 note/进度/v8/M5/26-11-12/SCAN真机接入清单.md） |
| `launch/lio_relay_sim.launch` | LIO 话题/TF 中继，消费模式由源话题与上层装配决定，不启动 Gazebo |
| `scripts/scan_arrival_monitor.py` | 我方请求目标的 J1 到达与距离观测，不为上游 FSM 合成失败事件 |
| `scripts/scan_bt_bridge.py` | SCAN ↔ v8/BT 契约桥（bspline→Path/status、goal 重置、arrival 转发、odom→pose；只翻译不规划） |
| `scripts/scan_cloud_ground_segment.py` | 壳侧地面平面分割（RANSAC + 自车圆柱 + 贴地障碍基座保留；A*/通用场景） |
| `scripts/scan_cloud_ground_filter.py` | 壳侧固定 `min_z` 过滤（坡面 P4 用；保留贴地障碍点） |
| `verify/v_scan_p1.sh`…`v_scan_p4.sh`、`v_scan_p3_triple.sh` | 占据/目标规划/跟踪/重规划与三次运行验证入口 |
| `verify/v_scan_bt.sh`、`bt_scan_probe.py` | BT 编排经 SCAN 规划/跟踪的冷启动全链验收（2026-11-12 PASS：120 点、reached 86.33 s、终点 0.141 m）；`SCAN_BT_PROFILE` 单入口模式 |
| `verify/p4_bt_probe.py` | BT 版 P4 动态重规划探针（注入箱、避让/3D 净距/物理/到达/BT 五门；2026-11-12 平地和 10°/1.4° 斜坡 PASS） |
| `verify/test_scan_cloud_ground_segment.py` | 地面分割纯函数单测（22 项） |
| `verify/test_scan_bt_bridge.py` | 契约桥纯函数单测（12 项） |
| `verify/v_scan_launch_expand.sh`、`v_scan_relay_equiv.sh`、`v_tf_edge_conflict.sh`、`v_scan_o1.sh` | launch、基线、TF 和到达接口合同 |

调整 SCAN 行为应使用本包参数、relay 与编排，不修改上游五包。
命令进入 M6 的 `scan_planner` 源，保持 `/cmd_vel` 唯一仲裁输出。
引用 P4 等历史结果须同时说明载体代际与实际构建件；文件数量或目录在位不是运行验收。

普通运行依赖包含 SCAN、SLAM 壳、消息、`topic_tools` 与 `tf2_ros`。
`lite3_gazebo_sim` 和 `gazebo_msgs` 仅供 `verify/` 的仿真回归使用，声明为测试依赖；机器人目标不选入这些测试依赖。
安装时保留 `launch/`、`scripts/`，`verify/` 仅在 `CATKIN_ENABLE_TESTING=ON` 时安装；目标构建入口关闭测试安装。已移除原安装清单中不存在的 `sink/` 引用。
源码内仍保留原验证脚本和历史兼容路径，真实仿真测试须在具备全部测试依赖的 x86 环境执行。构建目标范围及尚未验证的目标环境见[构建说明](../../../../tooling/build/README.md)。
