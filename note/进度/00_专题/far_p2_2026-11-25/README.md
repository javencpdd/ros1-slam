# FAR 仿真 P2 验收（2026-11-25）

> 目的：把 FAR 从"包缺失 ⇒ INVALID"推进到**实测**，逐项给出 PASS/INVALID 并留证。
> 未上机；support 后端未改；安全门控未降。

## 1. 环境

```bash
source /opt/ros/noetic/setup.bash
source <ws>/devel/setup.bash
source tooling/far_overlay/devel/setup.bash
export ROS_PACKAGE_PATH=<ws>/.profiles/x86_sim/src:tooling/far_overlay/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=<ws>/devel:<ws>/.profiles/x86_sim/devel:$CMAKE_PREFIX_PATH
roscore &
roslaunch lite3_gazebo_sim m1_gazebo.launch            # 点云 10 Hz、TF map→base_link
cd <ws>/src/90_platform/v8_platform/framework
python3 cli.py plan fixtures/manifests fixtures/graphs/graph_3d_gazebo_far.yaml \
  --package-index fixtures/package_index.json --out-plan /tmp/gfar.plan.json --out-launch /tmp/gfar.launch
roslaunch /tmp/gfar.launch                            # 14 节点，含 /far_planner
```

★环境接缝★：独立 overlay 的 `setup.bash` 会覆盖 `ROS_PACKAGE_PATH` 与 `CMAKE_PREFIX_PATH`，
不按上表补齐会出现 `far_stack.launch 不是 launch 文件` 或 `Cannot locate node [bt_console]`
（两者本轮都实际踩到，见 `tooling/far_overlay/PROVENANCE.md` §5）。

## 2. 逐项判定

| 检查项 | 判定 | 证据 |
| --- | --- | --- |
| 包可用性／装配 | **PASS** | `/far_planner`＋`/v8_global_far` 起；订阅 `/odom_world`／`/goal_point`／`/scan_cloud` |
| FAR 规划在动 | **PASS** | 日志 `Adding Goal to V-Graph`、`Path Search Time: 0.187ms`、`global vertices: 35` |
| 有效路径 | **PASS** | `evidence/graph_far2/probe_graph_gazebo_far.json`：path `{"n":2,"end_x":1.0,"end_y":0.0}` |
| 取消 | **PASS** | 同上：`cancel_sent=1`、`cancel_ack=1` |
| 非零速度／BT 到达 | **INVALID** | `nonzero_cmd_samples=0`；`/v8/local/cmd_vel_3d` twist 恒 0；mux `reason=track`、`hold=false`（非 mux 拦） ⇒ 阻断点在 local |
| 运动中障碍安全 | **INVALID** | 车未动，安全行为不可观测 |
| 定位失效停止 | **INVALID** | 运行中栈退出（`/clock` 中断），未取到失效前后对照 |

## 3. 证据清单

| 文件 | 说明 |
| --- | --- |
| `evidence/graph_far/probe_graph_gazebo_far.json` | 首次运行：path=32 但 planner_goal=0（探针用了默认 `/v8/global/goal`，本图两侧同名 `/v8/3d/global/goal`）⇒ FAIL |
| `evidence/graph_far2/probe_graph_gazebo_far.json` | 修正 planner-goal 话题后：verdict=PASS，path `n=2 end=(1.0,0.0)`，cancel_ack=1 |

## 4. 与"trad_far"组合的关系

本轮只跑了 **graph Gazebo+FAR**。`trad_far`（手工链：far_stack + path_goal + local + BT + relay）
未单独跑：两者共用同一条 FAR 后端与同一份 `far_stack.launch`，而**阻断点（local 输出恒零）
在两者共有的下游**，单独再跑一次不会得到不同结论。按"不为凑数重复跑"的口径，记
**trad_far = INVALID（未测，依附同一阻断点）**，复跑命令已写在 PROVENANCE §6。

## 5. 结论

FAR 后端**可用**：包能起、接线对、可见图在增长、两点路径有效、取消闭环。
但"能导航"**未证明**：运动未发生，到达/障碍/失效停止三项无法观测。
阻断点在 `local`（`/v8/local/cmd_vel_3d` 恒零），属 v8 local_native 侧待查项，与本 overlay 无关。
