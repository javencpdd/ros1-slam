# BT-ORCH / upstairs-formal —— BT 编排框架栈 正式窗

- 日期：2026-11-12
- 任务卡：`T-20261112-06`
- 窗口：`BT-ORCH-upstairs-r2`（回执见 `scripts/verify/_window_receipt.tsv`）
- 性质：**FORMAL**（原版 preflight `rc=0`；trace 校验 `ok`）
- 栈：`graphs/navigation_bt.yaml`（`map→global→bt→local→mux3d→carrier`，无 `path_goal`），
  BT 树 `navigate_with_planner.xml` 作为 `v8.fw.bt_console` 图中组件。
- 场景：`two_floor_nav` case=`upstairs`（声明几何地图 + Gazebo 运动学载体），
  起点 `(5.0, 1.0, 0)`，终点 `(-5.9, 1.1, 1.6)`。

## 读数（`summary.json` / `summary.app.json`）

- `ok=true`；六项 checks 全 `true`；`validation.ok=true`，`validation_error=null`。
- `carrier_final=(-5.75971, 1.12004, 1.6)`；全局路径 57 poses；本地 `reason=arrived`。
- validation：`command_integral_max_xy_error_m=0.03342`；`command_samples=2505`；
  `pose_samples=2535`；`paired=2535`；`global_paths=1`；`local_plans=26`；
  `max_command_gap_s=0.04183`；`visited_height_levels=[0..10]`。

## BT 树事件（`bt_events.log`）

```
[bt] goal queued (depth=1, max=16)
[bt] goal 1 dequeued (0 waiting)
[bt] global path requested for goal 1
[bt] global path ready (57 points)
[bt] follow path -> dispatch goal -5.9, 1.1, 1.6
[bt] arrival budget 666.8 s (path 13.34 m, floor 0.020 m/s, tree 60.0 s)
```

## 预检与环境分工

- preflight：`ISO_RC=0`、`PREFLIGHT VERDICT rc=0`（见 `summary.preflight.log`）。
- preflight 用 `/home/ros1-slam/ws/src` 满足项 [4]；**实际运行**用规范 `/opt/ros1-slam/ws/src`，
  避免 rosrun 在三份同名可执行（devel/lib + /opt/src + /home/src）间交互提示导致 carrier 不启动。
  实现见 `two_floor_nav._preflight_env`（仅 preflight 子进程覆盖别名根）。

## 失败尝试（冻存，不修饰）

`attempt1.*`：直接用别名环境跑时 `E_ORCH_READY_TIMEOUT: 未就绪: carrier`；
根因＝rosrun 三份同名可执行交互提示（`rosrun` 打印三选一后 `exec: : not found`）。
已由「preflight/运行环境分离」修复，未改任何节点/参数。

## 不声称

四足步态/物理接触、真机、实时建图、完整全局走廊；能力域仍是 **declared 几何地图 + Gazebo 运动学载体**。
`four_foot_physics=NOT_TESTED`，`global_corridor_following=NOT_ESTABLISHED: current adapter forwards final goal only`。

## 文件

- `summary.json` / `summary.app.json`：窗结果与 app 执行记录。
- `summary.preflight.log`：原版 preflight rc0。
- `summary.trace.json.gz`：观测轨迹（压缩）。
- `bt_events.log`：BT 树事件。
- `attempt1.*`：失败尝试（别名环境直跑）留底。
- `hashes.sha256`：本目录数据文件 sha256（不含 `.runtime` 日志目录）。
