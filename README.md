# ros1-slam 沙箱工作区

按 21 号《ros1-slam 隔离部署计划书 v3》建立的隔离部署沙箱（目标机 lite3-f20-1-103 / 192.168.1.103）。
计划书全文：[note/计划书v3/21_ros1-slam隔离部署计划书-v3.md](note/计划书v3/21_ros1-slam隔离部署计划书-v3.md)。

本工作区由开发侧对照库**单向复制**建立（绝不反向写入，A6）；第三方包"原样复制、一字不改"，
唯一许可的第三方源码改动是 faster-lio 存盘路径的隔离改写（登记于
[note/04_配置改动diff.md](note/04_配置改动diff.md)，并受 v3_isolation.sh 第 [8] 项永久监控）。

## 目录结构

| 目录 | 内容 |
|---|---|
| `ws/src/{00_drivers,…,90_supervisor}/` | 第三方包副本 + `lite3_*` 自研壳（M0–M6/TOP/SUP） |
| `scripts/` | `as_ros1slam.sh`（代劳执行）、`env.sh`（环境入口）、`gen_shell_pkgs.sh`、`verify/v3_isolation.sh`（隔离自检 8 项） |
| `note/` | 部署台账：计划书归档、`进度/<阶段>/`（进度文档 + 证据）、02 问题与坑、03 来源清单、04 配置改动 diff |
| `opt/ref/` | 只读参照（导航参数底稿、ROS2 原始 yaml 存档） |
| `data/maps/` | 建图产物落盘位（faster-lio 存盘路径已改写至此） |
| `logs/` | 运行日志（不入库） |

## 三条硬规则（全文见 [note/00_README.md](note/00_README.md)）

1. **真机安全**：`ros2qnx` 仅真机 mode；stub/采图/回放模式出现即事故
2. **隔离**：本目录内禁出现开发侧家目录路径字符串（豁免：note/ 来源记录、opt/ref/、隔离脚本 grep 模式）
3. **对照库**：永不复制 build/devel/.git；永不删除/修改开发侧对照库；顺序不能倒（G3→G8→G9）

## 常用操作

    sudo bash scripts/as_ros1slam.sh '<命令>'      # 以 ros1-slam 身份执行
    source scripts/env.sh                          # ROS 环境（P5 编译后 devel 就绪）
    bash scripts/verify/v3_isolation.sh; echo $?   # 隔离自检，期望 0

当前进度与各阶段证据：[note/进度/00_总览.md](note/进度/00_总览.md)（P0–P2 已完成，G0/G1/G1.5/G2 全 PASS）。

## 许可证

[Apache-2.0](LICENSE)（2026 年起，归本仓库所有者）。**仅覆盖自研内容**（`lite3_*` 壳包、`scripts/`、`note/` 文档）；
`ws/src/` 下第三方包副本各自保持其原始许可证，来源对照见 [note/03_外部文件来源清单.md](note/03_外部文件来源清单.md)。
