# ros1-slam 沙箱工作区

按 21 号《ros1-slam 隔离部署计划书》建立的隔离部署沙箱（目标机 lite3-f20-1-103 / 192.168.1.103）。
现行施工蓝本 **v4 + v4.1 补丁**：[计划书 v4](note/计划书v4/21_ros1-slam隔离部署计划书-v4.md) ·
[v4.1 雷达翻案补丁](note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md)（v3 原文归档于 note/计划书v3/）。
**实机传感器 = Livox Mid-360**（192.168.1.201，v4.1 实测翻案：UDP 56301/56401 → 103，C16 配置为软件遗留）。

本工作区由开发侧对照库**单向复制**建立（绝不反向写入，A6；/home/ysc 一切只读，要用先复制到本目录）；
第三方包"原样复制、一字不改"，唯一许可的第三方源码改动是 faster-lio 存盘路径的隔离改写（登记于
[note/04_配置改动diff.md](note/04_配置改动diff.md)，并受隔离自检第 [8] 项永久监控）。

## 目录结构

| 目录 | 内容 |
|---|---|
| `ws/src/{00_drivers,…,90_supervisor}/` | 第三方包副本（含 livox_ros_driver2）+ `lite3_*` 自研壳（M0–M6/TOP/SUP）；deeprobotics_local_height_map 以 CATKIN_IGNORE 标记排除（上游源码缺失，二期恢复） |
| `scripts/` | `as_ros1slam.sh`（代劳执行）、`env.sh`（环境入口，devel 守卫）、`gen_shell_pkgs.sh`、`verify/v3_isolation.sh`（隔离自检 [1]–[8]，P6 重落盘含 [9] 壳包源码检查） |
| `note/` | 部署台账：计划书归档、`进度/<阶段>/`（进度文档 + 证据）、02 问题与坑、03 来源清单、04 配置改动 diff |
| `opt/ref/` · `opt/3rdparty/` | 只读参照（导航参数底稿、ROS2 原始 yaml 存档）· Livox_SDK2 沙箱副本（/usr/local 已有系统安装，复用） |
| `data/maps/` · `data/waypoints/` | 建图产物落盘位（faster-lio 存盘路径已改写至此）· 航点目录（C6） |
| `logs/` | 运行日志（不入库） |

## 部署要点（v4.1 后）

- **M4 导航 = mbf_costmap_nav 联邦式**（GetPath/ExePath/Recovery 独立 action + 按名点单 + 错误码分级恢复；
  MBF 原生加载 nav_core 插件，global_planner/teb/clear_costmap_recovery 均直挂，无需适配器）
- **M5 编排契约先行**：task_primitives.yaml + SMACH 简版；**SUP 心跳监管**：/cmd_vel 唯一写者 = M6 safe_cmd_vel_mux
- **采图链**：livox_ros_driver2（remap → /rslidar_points，frame_id=rslidar）+ qnx2ros（/imu/data）→ faster_lio（run_mapping_online，lidar_type=4）

## 三条硬规则（全文见 [note/00_README.md](note/00_README.md)）

1. **真机安全**：`ros2qnx` 仅真机 mode；stub/采图/回放模式出现即事故；采图全程不发 `/cmd_vel`
2. **隔离**：本目录内禁出现开发侧家目录路径字符串（豁免：note/ 来源记录、opt/ref/、隔离脚本 grep 模式）
3. **对照库**：永不复制 build/devel/.git；永不删除/修改开发侧对照库；顺序不能倒（G3→G8→G9）

## 常用操作

    sudo bash scripts/as_ros1slam.sh '<命令>'      # 以 ros1-slam 身份执行
    source scripts/env.sh                          # ROS 环境（devel 已就绪）
    bash scripts/verify/v3_isolation.sh; echo $?   # 隔离自检，期望 0

当前进度与各阶段证据：[note/进度/00_总览.md](note/进度/00_总览.md)
（**P0–P5 已完成，G0–G4 全 PASS；当前 P3b 现场采图待现场执行**，作业单见
[note/进度/P3b/现场作业单.md](note/进度/P3b/现场作业单.md)；P6/P7 起节点前须 P0.5 双查 + Q8 处置确认）。

## 许可证

[Apache-2.0](LICENSE)（2026 年起，归本仓库所有者）。**仅覆盖自研内容**（`lite3_*` 壳包、`scripts/`、`note/` 文档）；
`ws/src/` 下第三方包副本各自保持其原始许可证，来源对照见 [note/03_外部文件来源清单.md](note/03_外部文件来源清单.md)。
