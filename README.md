# ros1-slam 沙箱工作区

按 21 号《ros1-slam 隔离部署计划书》建立的隔离部署沙箱（目标机 lite3-f20-1-103 / 192.168.1.103，
tegra194／aarch64——**目标机就是本机**，不在别处）。

**施工蓝本**：v4 正文 + 已落册补丁 v4.1／v4.2 + **v4.3 草案（未落册，以草案为准待裁定）**：
[v4](note/计划书v4/21_ros1-slam隔离部署计划书-v4.md) ·
[v4.1 雷达翻案](note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md) ·
[v4.2 仿真测试台](note/计划书v4/21_ros1-slam隔离部署计划书-v4.2-仿真测试台补丁.md) ·
[v4.3 楼梯通行与计划书同步纪律（草案 X1–X16）](note/计划书v4/21_ros1-slam隔离部署计划书-v4.3-楼梯通行与计划书同步纪律补丁.md)
（v3 原文归档于 `note/计划书v3/` 目录）。
**实机传感器 = Livox Mid-360**（192.168.1.201，v4.1 实测翻案：UDP 56301/56401 → 103，C16 配置为软件遗留）。
仿真侧的雷达是插件近似（竖向 50 线摊 ±90°、安装高 0.38m），其保真度缺口已挂账待裁（O-10／B6）。

本工作区由开发侧对照库**单向复制**建立（绝不反向写入，A6；/home/ysc 一切只读，要用先复制到本目录）；
第三方包"原样复制、一字不改"，唯一许可的第三方源码改动是 faster-lio 存盘路径的隔离改写（登记于
[note/04_配置改动diff.md](note/04_配置改动diff.md)，并受隔离自检第 [8] 项永久监控）。

## 我要找东西，去哪儿

| 想知道 | 去这里 | 说明 |
|---|---|---|
| **代码/配置/launch/验收件在哪** | [`ws/README.md`](ws/README.md) | 工作区定位图：11 个模块层的归属表＋"按问题查"的速查表＋红线速记 |
| **每个算法存在哪、能不能单独换** | [`ws/README.md`](ws/README.md) §六 | 存储地址总表＋独立性实测结论（跨包 `#include`＝无、`package.xml` 互 depend＝无，接缝只有话题/TF）＋"换它要动哪几处" |
| **2.5D／高程方向搁在哪、怎么捡回来** | [`ws/README.md`](ws/README.md) §6.3 | **主理人 2026-10-10 裁定：暂时搁置、留后期备用**（搁置≠删除，四处地址里的东西不许清理）；恢复入口与前置清单已写死 |
| **干到哪了、证据在哪** | [`note/进度/00_总览.md`](note/进度/00_总览.md) | 唯一进度真身（时间线口径，别信任何摘要里的旧状态） |
| **在等谁点头** | [`note/进度/00_裁定请求单_2026-10-09.md`](note/进度/00_裁定请求单_2026-10-09.md) | 按人分组（小美＝判据面／小张＝设计面／主理人＝路线与窗口） |
| **踩过什么坑** | [`note/02_问题与坑.md`](note/02_问题与坑.md) | 台账到坑 52；纪律类结论都会回写这里 |
| **哪些改动被许可** | [`note/04_配置改动diff.md`](note/04_配置改动diff.md) | 逐批改动＋`[8]` 项长期监控面 |
| **验收怎么做才算数** | [`note/05_复验作业单.md`](note/05_复验作业单.md) | A–F 段（F 段＝程序令 P8 计划书同步纪律） |
| **判据现在是哪一代** | [`note/07_判据载体代际对照表.md`](note/07_判据载体代际对照表.md) | 与 `v_harness_generation_table.sh --check` 逐行对账 |
| 三态通道（PASS/FAIL/INVALID）口径 | [`note/06_三态通道普查.md`](note/06_三态通道普查.md) | INVALID 不入串、也不得吸收并存的 FAIL |

## 目录结构

| 目录 | 内容 |
|---|---|
| `ws/src/{00_drivers,…,95_simulation}/` | 第三方包副本 + `lite3_*` 自研壳（M0–M6/TOP/SUP）＋仿真域；`deeprobotics_local_height_map` 以 `CATKIN_IGNORE` 排除（闭源、0 代码件） |
| `ws/build/`、`ws/devel/` | 编译产物，**不入库**（`.gitignore` 已排除） |
| `scripts/verify/` | 门禁四件套：`v3_isolation.sh`（[1]–[9]）、`v_sh_py_syntax.sh`、`v_harness_generation_table.sh`、`v_deferred_clause_scan.sh`；另有 `premap_check.sh`（采图前体检） |
| `scripts/` | `as_ros1slam.sh`（代劳执行）、`env.sh`（环境入口，含 foxy 清理与 devel 守卫）、`gen_shell_pkgs.sh`、`ops/`（属主/ACL 工单，需你本机 `sudo` 执行） |
| `note/` | 台账与进度真身（见上表）；计划书归档 `计划书v3/`、`计划书v4/` |
| `opt/ref/` · `opt/3rdparty/` | 只读参照（导航参数底稿、ROS2 原始 yaml 存档）· Livox_SDK2 沙箱副本 |
| `data/maps/` · `data/waypoints/` | 建图产物落盘位（**`data/maps/lite3.pcd` 是样张红线件**，各跑前后取 sha）· 航点目录 |
| `logs/` | 运行日志（`ROS_LOG_DIR` 已指此，不入库） |

## 部署要点（现行口径）

- **M4 导航 = mbf_costmap_nav 联邦式**（GetPath/ExePath/Recovery 独立 action＋按名点单＋错误码分级恢复；MBF 原生加载 nav_core 插件，无需适配器）
- **M5 编排契约先行**：`50_task/lite3_task/config/task_primitives.yaml` + SMACH 简版；**SUP 心跳监管**；`/cmd_vel` 唯一写者 = M6 `safe_cmd_vel_mux`
- **采图链**：`livox_ros_driver2`（remap → `/rslidar_points`，frame_id=rslidar）+ `qnx2ros`（`/imu/data`）→ `faster_lio`（`run_mapping_online`，lidar_type=4）
- **SCAN 集成走壳包**：上游五包在 `ws/src/30_3rd/` 按基线不改，行为改点落在 `40_navigation/lite3_scan/`
- **仿真优先于真机**：能在仿真闭环的验收不放狗（真机窗口留给 G8/G9 合测）

## 三条硬规则（全文见 [note/00_README.md](note/00_README.md)）

1. **真机安全**：`ros2qnx` 仅真机 mode；stub/采图/回放模式出现即事故；采图全程不发 `/cmd_vel`
2. **隔离**：本目录内禁出现开发侧家目录路径字符串（豁免：note/ 来源记录、opt/ref/、隔离脚本 grep 模式）
3. **对照库**：永不复制 build/devel/.git；永不删除/修改开发侧对照库；顺序不能倒（G3→G8→G9）

## 常用操作

    sudo bash scripts/as_ros1slam.sh '<命令>'      # 以 ros1-slam 身份执行
    source scripts/env.sh                          # ROS 环境（含 foxy 清理＋devel 守卫）

    # 改过代码/配置后必跑的"常见面"复验（顺序不重要，但要全绿才算收口）
    bash scripts/verify/v_sh_py_syntax.sh <被检件清单>; echo $?   # 语法面（空清单不得判绿）
    bash scripts/verify/v_harness_generation_table.sh --check     # 代际表与 git 逐行对账
    bash scripts/verify/v3_isolation.sh; echo $?                  # 隔离自检 [1]–[9]，期望 0
    bash scripts/verify/v_deferred_clause_scan.sh                 # 缓做条款到期巡检（rc=1 是提示非红）

> **跑批前请先自证调用姿势**：多数"红"其实是没 `source` 环境（ISO 项[4] 依赖 `rospack find`，
> `catkin_make` 会 rc=127）——"跑不动 ≠ 跑不过"，先复核环境再判被测件。

## 进度与状态

进度、Gate 状态、待裁项**只以 [`note/进度/00_总览.md`](note/进度/00_总览.md) 与
[`note/进度/00_裁定请求单_2026-10-09.md`](note/进度/00_裁定请求单_2026-10-09.md) 为准**，本文档不复制状态快照
（复制就会漂移，02 坑43 同族）。当前主线在仿真侧（M2/M5/SCAN 闭环与楼梯可行性预检），
真机侧 G8/G9 待窗口合测；判据载体代际现值见 `note/07_判据载体代际对照表.md`。

## 许可证

[Apache-2.0](LICENSE)（2026 年起，归本仓库所有者）。**仅覆盖自研内容**（`lite3_*` 壳包、`scripts/`、`note/` 文档）；
`ws/src/` 下第三方包副本各自保持其原始许可证，来源对照见 [note/03_外部文件来源清单.md](note/03_外部文件来源清单.md)。
