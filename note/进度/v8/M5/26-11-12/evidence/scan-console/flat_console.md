# scan_flat 控制台栈端到端（goal→arrival→dwell→stop）

- 时间：2026-11-12 02:54—03:05（CST）
- 栈：`scan_flat`（`scripts/console/stacks.json`，master `http://127.0.0.1:11313`，world `m1_world.world`）
- 启动：`POST http://127.0.0.1:8780/api/stacks/scan_flat/start` → pid 684849
- 停止：`POST .../scan_flat/stop` → ok
- 容器：`ros1-slam-sim-viz`（独立 devel 卷 `docker_ros1_x86_sim_devel`）

## 关键前提：sim-viz 内 lite3_bt 必须按本提交重建

首次启动时 `/v8/bt/status` 报 `"planner_mode":"pose"`，`/bt_console/arrival_topic`
已为 `/v8/bt/arrived` 但节点不认（旧二进制，无 arrival signal 分支）。
sim-viz 与 ros1-slam-sim 是两套 devel 卷，源同、二进制不同：

    docker exec ros1-slam-sim-viz bash -lc \
      'source /opt/ros/noetic/setup.bash; cd /opt/ros1-slam/ws && catkin_make --pkg lite3_bt -j4'

重建后二进制：2026-11-12 02:54，395720 B；`strings` 含 `external arrival signal`、
`arrival_speed_floor_mps` 各 1 处；重启栈后 `planner_mode=signal`。

bt_console 源码：commit 6c552bee，`ws/src/90_platform/lite3_bt/src/bt_console.cpp`
blob ef017204fb7b96ffc42125dfb7dffc4fdcb4428a。

## 参数读数（栈内 rosparam）

    /bt_console/arrival_topic=/v8/bt/arrived
    /bt_console/arrival_speed_floor_mps=0.02
    /bt_console/planner_goal_topic=/scan_planner/goal
    /scan_arrival_monitor/tol_arrive=0.2  settle_s=1.0
    /scan_planner_node/fsm/navi_mode=1（平地 A*）

## 跑 1：goal (2,0) —— SCAN 截断目标，BT 诚实失败

`rostopic pub -1 /v8/bt/goal`（frame `map`，z=0）：

    [ WARN] [1794452110.634191942, 7.951000000]: [global target] Target [2.00, 0.00, -0.17] is occupied; use backward collision-free point [1.32, 0.01, -0.29].
    [ INFO] [1794452110.851975056, 8.050000000]: [bt] follow path -> dispatch goal 2, 0, 0
    [ WARN] [1794452170.916225943, 35.500000000]: [bt] arrival timeout after 60.1 s (budget 60.0 s)

轮询（wall 5 s 步长）：t=5 s x=1.165 → t=15 s 起 x≈1.29 不再前进 →
t=35 s `outcome=failed`，末位姿 x=1.2955。

判读：SCAN 内部栅格把 (2.00,0.00,-0.17) 判为占据，按自身逻辑把**目标截断**到
(1.32,0.01,-0.29) 并完成；BT 的到达权按**原目标**判（`/scan_planner/goal`），
`/v8/bt/arrived` 始终 false，arrival budget 60 s 到期 → failed。非误报到达。
`-0.17` 来自 FSM 冻结的首帧 odom z（见 §17 记录），该点高度是否落入自身膨胀带
尚待单独实验定性；当前行为是「失败得诚实」，不是「错报到达」。

## 跑 2：goal (-2.75,1.55) —— 全链 PASS

    [ INFO] [1794452466.584406524, 169.050000000]: [bt] goal 2 dequeued (0 waiting)
    [ INFO] [1794452466.808666517, 169.150000000]: [bt] global path ready (120 points)
    [ INFO] [1794452466.808966127, 169.150000000]: [bt] follow path -> dispatch goal -2.75, 1.55, 0
    [ INFO] [1794452466.809032769, 169.150000000]: [bt] arrival budget 177.2 s (path 3.54 m, floor 0.020 m/s, tree 60.0 s)
    [INFO]  [1794452510.214997, 188.610000]: SCAN arrival=True

轮询（goal 后 wall 6 s 步长）：

    t=6s  event=path_ready outcome=""       arrived=False x=-0.206 y=0.565
    t=12s event=path_ready outcome=""       arrived=False x=-1.379 y=1.342
    t=18s event=path_ready outcome=""       arrived=True  x=-2.592 y=1.530
    t=24s event=path_ready outcome=reached  arrived=True  x=-2.730 y=1.507
    t=30s..84s outcome=reached  arrived=True  x≈-2.75 y≈1.55

判读：`arrival budget 177.2 s` = max(60, 3.54/0.02)，长度缩放生效；
`SCAN arrival=True` → `outcome=reached`；机器人停在目标 0.02 m 内（x=-2.7538,
y=1.5511 处读数）。C++ 节点随 launch 存活，栈 stop 时一并清场。

## 备注

`/LIO/odom_vehicle` 与规划器 `/Odometry` 是不同话题；控制台到达权用前者
（`scan_arrival_monitor`），SCAN 跟踪用后者，二者在本次跑内一致收敛。
