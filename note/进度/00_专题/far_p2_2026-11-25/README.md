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

## 6. 第二轮（2026-11-25 深夜）：定位到"车一步不走"的根因并修复

### 6.1 证据指向哪里

上一轮读数 `path=32 → local_goal=64`，且局部目标在两个值之间**严格交替**：

```
t=3.29 x=0.000 y=-3.060   ← 与当时机器人位姿完全相同
t=3.35 x=1.000 y=0.000    ← FAR 航点
t=4.18 x=0.000 y=-3.060
t=4.24 x=1.000 y=0.000    ...
```

`(0,-3.06)` 正是当时 `/v8/local/pose` 的位置（实测确认）。即：**每秒先把"去我站的地方"当目标
发给 local，再推进到真航点**；local 因此反复重开跟踪、输出恒零。

### 6.2 根因（`path_goal_node.py`）

`on_path` 收到任何路径都执行 `self.index = 0; self.publish_current()`：
- support 后端给的是**完整路径**，`poses[0]` 就是当前位姿，由 `tick` 靠 `arrive_m` 推进一步，
  顶层看不出来；
- FAR 给的是**两点滚动路径**（当前位姿＋航点）且每秒重发 ⇒ 每秒回到第 0 点。

这与"generation 重置"是同一件事的两面：每次 `on_path` 都 `accepted_generation += 1` 并重发第 0 点。

### 6.3 修法

| 改动 | 语义 |
| --- | --- |
| `_first_unreached()` | 新路径到达时从**第一个还没到达的点**开始；判据与 `tick` 同一把尺子（`arrive_m`），避免"跳过点与推进点打架" |
| `publish_current()` 去重 | 目标坐标与上次相同则不再发：local 每收一条新目标就重开跟踪代次，每秒重发同一个点会让它反复重启 |
| 全部点都在半径内 | 退回**最后一点**，绝不退回 0（否则等于原地不动） |

单测：`tests/test_path_goal_first_unreached.py`（5 项，含两条反控：全在半径内不得退回 0、
同一目标不得重发）；与既有 `tests/test_path_goal_cancel.py` 合计 **11 项通过**。

### 6.4 端到端：**仍未闭合**

修完重跑（FAR 图 + Gazebo，30 s 正常段）：`max_speed=0.0`、`displacement=0.0`、
`nonzero_ratio=0.0`，`/v8/bt/arrived` 无消息。即 **根因修了，车仍不走**，阻断点下移到 `local`。

四项验收本轮**全部 INVALID**：

| 项 | 判定 | 读数 |
| --- | --- | --- |
| 1 正常导航（非零运动／BT reached／真实到达） | **INVALID** | `max_speed=0.0`、`displacement=0.0`、`arrived` 无消息 |
| 2 障碍安全（运动中） | **INVALID** | 无运动，不可观测 |
| 3 定位失效（断数据后归零且存活） | **INVALID** | 断 `localization_ok` 前后均为零速，无法区分"因失效归零"与"本来就不动" |
| 4 取消安全（运动中触发） | **INVALID** | 探针要求"先有速度再发取消"，未达触发条件 ⇒ 未发取消、无 ACK |

### 6.5 环境侧硬伤（本轮 6 次重跑都栽在这里，必须记）

| 现象 | 真相 |
| --- | --- |
| 每次 exec 结束后节点全灭 | `docker exec`（不加 `-d`）会随主进程退出清掉进程组；`setsid nohup &` 也留不住。要在**同一次 exec 内**跑完全链，或用 `-d` 且确认存活 |
| 探针连的 master 与图不一样 | 镜像默认 `ROS_MASTER_URI=11313`，我另起的 `roscore` 在 11311 ⇒ **双 master**，各连各的 |
| 仓根放脚本容器里看不到 | 容器只挂载 `ws/note/scripts/tooling` 等**子目录**，仓根文件不同步 |
| `rostopic echo -n1` 把整条命令挂死 | 目标没消息时会无限等 ⇒ 一律套 `timeout` |

★结论★：FAR 侧（打包/接线/规划/两点路径/取消契约）已验；**运动闭环未验**，阻断点在
`local` 输出恒零——与 support 后端共用同一段，`p2_runtime_2026-11-23` 的"未验证清单"里
也早写着"非零速度、轨迹跟踪、到达判据"未验。故本项不是 FAR 引入造成的回归。
