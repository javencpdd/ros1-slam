# SCAN 真机接入清单（2026-11-12）

> ★真机物理运行是 L2★：本清单覆盖「软件准备 + 不动车 dry-run」；解锁、发目标、急停
> 必须真人在场并按 `note/10` 真机流程签核。103 只读；不向 120/43893 发帧。

## 0. 前提（缺一即停）

| # | 项 | 判据 |
|---|---|---|
| 0.1 | 机器人 bringup | `/LIO/odom_vehicle`、`/LIO/clouds_lidar`、`/LIO/odom_imu` 在线；`map→odom→body` TF 连续；LIO 是唯一前馈源 |
| 0.2 | 安全层 | `60_safety` 在栈（mux 唯一写者 `/cmd_vel`；`scan_planner` 源 priority 7）；机械急停/遥控可接管 |
| 0.3 | 真机 BT profile | `config/bt_console_real.yaml.template` → 复制为 `bt_console_real.yaml`，逐项确认话题；缺文件时 `profile:=real` fail-closed（`scripts/console/test_bt_config.py` 会报"no bt_console_real.yaml yet"——该件**不入仓**，由操作员在机器人上创建） |
| 0.4 | 运动使能 | 默认关（`mode_navigation` / `enable_motion`），操作员显式打开后才可能动 |
| 0.5 | 场地 | 空旷、平整、急停可达；首测 2—3 m 直线，速度 ≤0.2 m/s |

## 1. 起栈前（只读）

- `rosnode list` 对照：bringup 的 LIO/TF；`/cmd_vel` Publishers 恰为 `/safe_cmd_vel_mux`。
- `rostopic hz /LIO/odom_vehicle /LIO/clouds_lidar`；`/LIO/clouds_lidar` 帧为 `body`。
- 磁盘/日志（`/opt/ros1-slam/logs`）；CPU 负载按容量评估（见 capacity-aware 规则）。
- 机器人模式：`mode_navigation` 已建、`enable_motion=false`。

## 2. dry-run（不动车，本清单的核心验收）

```bash
# 机器人 bringup 已起；本 launch 只装 SCAN+BT+安全层
roslaunch lite3_scan scan_nav_real.launch bt_profile:=demo   # 先用 demo 确认接线
# 若已复制并确认真机 yaml：bt_profile:=real
# 只读预检（节点在位/planner_mode=signal/arrived=false//cmd_vel 全零）：
python3 ws/src/40_navigation/lite3_scan/verify/real_preflight.py --window 4
```

通过判据（全部只读）：

0. **先重建 bt_console**（真机容器自己的 devel 卷）：`catkin_make --pkg lite3_bt`；
   `strings .../bt_console | grep -E "external arrival signal|arrival_speed_floor_mps"`
   各 1 处；否则 `/v8/bt/status` 报 `planner_mode=pose`，arrival signal 永不触发
   （2026-11-12 sim-viz 实测坑，见 SCAN-P4接入BT与斜坡诊断 §17/§18）。
1. 节点在位：`scan_planner_node`、`closed_loop_controller`、`scan_arrival_monitor`、
   `scan_bt_bridge`、`bt_console`、`v8_goal_gate`、`safe_cmd_vel_mux` 均 RUNNING；
2. 话题：`/planning/bspline`（无目标时可为空）、`/global_path/status`、
   `/lite3/scan/arrived`（false）、`/v8/bt/status`（`wait_goal` RUNNING、outcome 空、
   `planner_mode=signal`）；
3. 速度：`/cmd_vel` 平均速率 20 Hz 且线速度 **0**；无 `/scan_planner/cmd_vel` 非零输出；
4. 无 `E_LOCAL_*` 锁存（`latched=false`）；BT 无 goal、无 timeout/recovery；
5. 日志无 Traceback/AttributeError；`rosnode list` 无重复/孤儿。

## 3. 首次动（L2，真人执行）

1. 打开 `enable_motion`；确认 `/cmd_vel` 仍 0；操作员持急停；
2. `/v8/bt/goal` 发 2—3 m 直线目标（平地）；观察 BT `path_ready → follow_path →
   arrival_start → reached → dwell → stop`；到达偏差 ≤0.35 m；
3. 异常即急停/`rosnode kill`；记录 `/v8/bt/status`、`/v8/local/status`?（SCAN 链无此话题）、
   `/scan_planner_node` 状态、`/cmd_vel`、`/lite3/scan/arrived`；
4. 收工：`enable_motion=false`；停栈；日志归档。

## 4. 场景参数建议

| 场景 | navi_mode | 地面过滤 | 备注 |
|---|---|---|---|
| 平坦室内 | 1（A*） | `scan_cloud_ground_segment.py`（0.08） | 默认 |
| 长坡/连续坡度 | 3（参考路径） | `scan_cloud_ground_filter.py` + `ground_min_z:=0.0`（§10 已验） | A* 坡上起点格问题见 §5—§13 |
| 动态障碍 | 同场景 | 同上 | P4 五门（避让/3D 净距/物理/到达/BT）不变 |

## 5. 已知限制（随读数引用）

- A\* 坡上失效根因＝地面点落进规划平面膨胀带（§11）；平面分割可解，box 贴地点需
  §13 的保留策略；坡上 P4 目前用参考路径 + min_z 过滤（§10）。
- 仿真验证是 m2 平面载体 + faster_lio；真机是四足接触动力学，速度/跟踪/坡上表现
  不等于仿真读数。
- v8 台阶世界（8.53° stepped）归 v8 支撑图链；SCAN 斜坡是平滑 10°/1.4° 资产。
- 未做真机物理运行；本条不声称真机通过。

## 6. 证据

- dry-run：日志、`rosnode list`、话题/状态快照、`/cmd_vel` 零速；
- 首次动（L2）：goal/arrival/stop 状态、里程计轨迹、日志、机器人模式截图；
- 归档到 `note/进度/v8/M5/<日期>/evidence/scan-real-<run>/`；代码引用记 git 提交＋blob。
