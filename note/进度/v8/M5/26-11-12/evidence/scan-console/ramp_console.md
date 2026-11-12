# scan_ramp 控制台栈端到端（goal→arrival→dwell→stop）

- 时间：2026-11-12 03:04—03:09（CST）
- 栈：`scan_ramp`（master `http://127.0.0.1:11313`，world `ramp_scan10.world`，10° 坡）
- 启动：`POST /api/stacks/scan_ramp/start` → pid 696075（`roslaunch lite3_scan scan_nav_sim.launch profile:=ramp`）
- 容器：`ros1-slam-sim-viz`，19 节点在位

## 栈内参数读数

    /scan_planner_node/fsm/navi_mode=3          （REFERENCE_PATH，不走 A*）
    /scan_planner_node/manager/max_vel=0.6
    /scan_planner_node/optimization/max_vel=0.6
    /bt_console/arrival_topic=/v8/bt/arrived
    /bt_console/arrival_speed_floor_mps=0.006
    /scan_bt_bridge/publish_reference_path=true  （由 BT goal 生成 /initial_path）

## 跑：goal (1.5, 0)（frame `map`）

`rostopic pub -1 /v8/bt/goal`（z=0）后轮询（goal 后 wall 8 s 步长）：

    t=8s  event=arrival_start outcome=""       path_state=ready arrived=False x=1.225 y=-0.001
    t=16s event=stop          outcome=reached  arrived=True  x=1.512 y=-0.005
    t=24s..80s outcome=reached  arrived=True  x≈1.55 y≈-0.003

栈内 `/v8/bt/status` 快照（原始 echo，逐字）：

    "tree":"NavigateWithPlanner","root":"RUNNING","event":"stop","queue_depth":0,
    "goal_seq":1,"current_seq":1,"outcome":"reached","outcome_seq":1,
    "path_state":"ready","path_points":120,"planner_mode":"signal",
    "arrival_signal":true, nodes: wait_goal RUNNING 其余 IDLE

SCAN 侧（`/tmp/v8-console/scan_ramp.log`）：

    669:[ WARN] [1794452741.864742777, 14.381000000]: [closed_loop_controller] received bspline traj_id=1 duration=3.733
    779:[ WARN] [1794452746.711134604, 16.588000000]: [closed_loop_controller] received bspline traj_id=2 duration=3.368

判读：参考路径模式（navi_mode=3）+ BT 桥生成 `/initial_path` 生效；SCAN 两条
bspline 上坡跟踪 → `/lite3/scan/arrived=True` → BT `arrival_signal=true`、
`outcome=reached`、dwell→stop 收口。全程 1.5 m 坡道，停在目标 0.05 m 内。

## 边界

- 无 P4 动态注入（控制台跑是 demo 编排验证；P4 门禁读数见 `scan-p4-bt/`）。
- bt_console 的 stdout 在 roslaunch 下块缓冲，进程存活时 `[bt]` 行不进
  `/tmp/v8-console/scan_ramp.log`；本次判据用 `/v8/bt/status` topic 与轮询读数。
