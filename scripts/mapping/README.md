# 真机采图脚本

> 最新更新时间：2026-11-05 11:49:54 CST。本轮按脚本静态核对，未连接雷达、未启停任何 ROS 服务。

本目录保存 Livox + faster-lio 的真机采图工具。多数脚本仍固定 `/home/ros1-slam` 路径，并从 `m_env.sh` 加载 Noetic 和项目环境；旧现场目标为 `192.168.1.103:11312`、雷达 `192.168.1.201`；实际环境会受调用者变量影响，脚本不保证重置为该域。这些是旧现场配置，不代表当前 x86 主机可直接运行。

**当前地图准备和真机排程已经取消**，以[执行队列](../../note/计划书v8/23_v8任务队列.md)和任务卡为准。下面是取得对应现场授权后的操作说明，不能据此自动恢复采图。

## 前置与启动

确认现场路径、网络、传感器、工作区构建和独占运行窗口后，在有权限的运行账户下进入 `tmux`；会话名要与 `SESSION` 一致：

```bash
cd /home/ros1-slam/scripts/mapping
tmux new-session -A -s p3b
bash m_check.sh
export ROS_MASTER_URI=http://192.168.1.103:11312
export ROS_IP=192.168.1.103
SESSION=p3b VIZ=1 bash m_start.sh
ODOM=1 bash m_verify.sh
bash m_pose.sh
```

上述IP仅是旧现场例子，执行前替换为该现场实值。`m_env.sh` 设置的 `ROS_MASTER_URI_DEFAULT` 并不由当前 `scripts/env.sh` 消费，不能靠这个变量保证连接11312。

`m_check.sh` 包装 `premap_check.sh`，先尝试非交互 sudo，再以当前账户回退；报错须按实际输出处理。`m_start.sh` 要求当前已在 tmux 内，依次建立 `core`、`drv`、`slam` 和默认开启的 `viz` 窗。不要把驱动退出或 bind 错误一概视为无害。

| 窗口 | 内容 |
| --- | --- |
| `core` | `m_run_core.sh` 启动 `roscore -p 11312` |
| `drv` | `m_run_drv.sh` 启动 `lite3_drivers/01_sensors.launch` |
| `slam` | `m_run_slam.sh` 启动 `lite3_slam/10_slam_mapping.launch` |
| `viz` | `m_viz.sh` 启动 `mapping_viz.launch`：累积快照和 Lite3 显示模型 |

`m_verify.sh` 输出点云/IMU 频率，`ODOM=1` 时再读里程计频率。它只是采样展示，**没有实现阈值联锁或自动阻止走位**；是否有效按现行作业单和现场读数判断。

## 停机与产物

```bash
NAME=本次标签 bash m_stop.sh
```

脚本先向 faster-lio 发 SIGINT 并等待进程退出，再停驱动/master，最后重命名 `data/maps/lite3.pcd` 为 `lite3_<标签>.pcd`。缺省标签来自当次 `date`，`NAME=keep` 保留默认文件名，`KEEP_LATEST=1` 额外复制默认名。faster-lio 的退出日志和产物完整性须现场确认。

**脚本边界**：`m_stop.sh` 超时后只打印警告，仍会继续停其他进程和重命名；它不以 PCD 校验或落盘日志作强制条件。`m_clean.sh` 使用广泛的进程名匹配，`m_stop.sh` 也会匹配其他 rosmaster/roscore；两者不按 master 域精确隔离。只在确认相关进程均属本次可停止窗口时使用，不把清场脚本当普通只读检查。不要直接杀 SLAM tmux 窗来代替落盘确认。

地图各机独立，旧图、现场默认名和 registry 不能互相覆盖。本目录不会自动生成 PGM/YAML，也不会把 polished 图自动设成下游工作图。

## 查看和派生

```bash
FILE=本次图名 DECIMATE=5 bash m_view.sh
FILE=本次图名 bash m_polish.sh
FILE=本次图名 THICK=0.5 bash m_slice.sh
```

`FILE` 不含 `.pcd`；省略时各工具按 mtime 选最新文件，现场应显式指定。`m_view.sh` 发布 latched `/cloud_view`，缺少可连 master 时会尝试启动 11312 roscore，因此也不是纯离线查看。Foxglove 使用与 ROS 图连通的 **rosbridge WebSocket**，不是 `http://<主机>:11312`；协议、固定帧和网络限制见 [Foxglove 指南](../../docker/foxglove/README.md)。采图观察只订阅，不发布运动或目标。

| 脚本 | 用途与主要参数 |
| --- | --- |
| `m_env.sh` | 公共环境，由其他脚本加载 |
| `m_check.sh` | 采图前检查；第一个参数可覆盖雷达 IP |
| `m_start.sh` | tmux 窗组；`SESSION`、`VIZ` |
| `m_run_core.sh`、`m_run_drv.sh`、`m_run_slam.sh` | 各窗口运行内容 |
| `m_verify.sh`、`m_pose.sh` | 频率采样、位姿读取；`ODOM`、`N` |
| `m_viz.sh` | LIO `/map_snapshot`（`camera_init`）与模型 |
| `m_stop.sh`、`m_clean.sh` | 有停止其他进程风险的采图停机/清场 |
| `m_view.sh`、`pcd_latch_pub.py` | PCD 话题发布；`FILE`、`TOPIC`、`DECIMATE` |
| `m_polish.sh`、`pcd_polish.py` | 点云裁剪、体素和离群处理；输出由后端工具决定 |
| `m_slice.sh`、`pcd_slice.py` | 按高度切片；`THICK`、`ZMIN`、`ZMAX`、`OUTDIR` |

## 仿真建图入口

Gazebo 键盘采集不使用这套真机清场/停机脚本。它使用 `lite3_gazebo_sim/scripts/mapping_recorder.py`、`keyboard_teleop.py` 和 `docker/rviz_mapping3d.rviz`；记录器依赖 `/sim/odom_gt`，属于 GT 位姿建图。PCD 转支撑图、生成 app/task 和交互导航的完整流程见 [Docker 指南 §4.7](../../docker/README.md)。这条链不构成 LIO、物理四足或真机验收。
