# BT-ORCH / upstairs-bt —— BT 编排框架栈 仿真冒烟

- 日期：2026-11-12
- 任务卡：`T-20261112-05`
- 性质：**SMOKE（冒烟）**，未经正式窗 preflight（原因见下），**不与 R1127 formal 窗互认**。
- 栈：v8 框架图 `graphs/navigation_bt.yaml`（`map → global → bt → local → mux3d → carrier`，无 `path_goal`）；
  BT 树 `lite3_bt/trees/navigate_with_planner.xml` 作为图中一等组件 `v8.fw.bt_console`。
- 场景：`two_floor_nav` case=`upstairs`（声明几何地图 + Gazebo 运动学载体），起点 `(5.0, 1.0, 0)`，终点 `(-5.9, 1.1, 1.6)`。

## 读数（`summary.json`）

- `ok=true`；`assembly_ready / map_configured / global_path / local_arrived / carrier_at_local_goal / markers_published` 全 `true`。
- 全局路径 57 poses；`carrier_final = (-5.71199, 1.12686, 1.6)`；
  `carrier_distance_to_local_goal_m = 0.18991883`；`carrier_z_error_m = 0.0`。
- 本地规划 `reason=arrived`，轨迹末点 `(-5.9, 1.1, 1.6)`；markers 40。

## BT 树事件（`bt_events.log`）

```
[bt] goal queued (depth=1, max=16)
[bt] goal 1 dequeued (0 waiting)
[bt] global path requested for goal 1
[bt] global path ready (57 points)
[bt] follow path -> dispatch goal -5.9, 1.1, 1.6
[bt] arrival budget 666.8 s (path 13.34 m, floor 0.020 m/s, tree 60.0 s)
```

## 为什么能证明「BT 驱动」而非 app 直发

图中**没有** `path_goal` 实例，`/v8/local/goal` 的唯一写者是 BT `FollowPath`；
`local_arrived` 与 `carrier_at_local_goal` 同时成立 ⇒ 目标由 BT 下发。
app 的 goal 走 `goal_topic: /v8/bt/goal`（BT `WaitForGoal` 输入），app 不再写 `/v8/3d/global/goal`。

## 预检漂移（环境，非代码）

`scripts/verify/v8_window_preflight.sh` 本容器 **rc=5**：item[4] 要求 rospack 解析路径字面位于 `/home/ros1-slam`，
而 item[10]/[9-v8] 以 `/opt/ros1-slam` 为规范别名（`[9-v8]` 明确要求
`readlink -f /home/ros1-slam/ws/src/40_navigation == /opt/ros1-slam/ws/src/40_navigation`）。
修复需把 `/home/ros1-slam/ws/src` 由实目录替换为指向 `/opt/ros1-slam/ws/src` 的符号链接——**有删除实目录的风险，未擅动**。
因此本跑**不声称** formal 窗，只作接线冒烟；正式窗需先修容器路径别名或换回无漂移环境。

## 不声称

四足步态 / 物理接触、真机、实时建图、完整全局走廊；
能力域仍是 **declared 几何地图 + Gazebo 运动学载体**。真机（lite3/M20）迁移只留运行准备，未上机。

## 文件

- `summary.json`：app_runner 执行结果（`ok` 与六项 checks）。
- `trace.json.gz`：观测轨迹（9951 事件，压缩）。
- `bt_events.log`：BT 树事件序列。
- `hashes.sha256`：本目录文件 sha256。
