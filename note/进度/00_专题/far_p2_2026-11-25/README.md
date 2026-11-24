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

## 7. 第三轮：按"对照已跑通的 M1 图"排查装配差异

### 7.1 新建对照图（只换全局后端）

`framework/fixtures/graphs/graph_3d_m1_hdl_bt_far.yaml` = `graph_3d_m1_hdl_bt.yaml` 原样保留
（M1 地图、HDL 定位、`perception`、`local`、`mux3d`、单一 carrier），**仅把 `global` 从
`v8.fw.global3d_topic` 换成 `v8.fw.global_far`**，并去掉 `map.map → global.map` 这条对 FAR 不成立的边
（FAR 不吃 2D 地图，自己从点云建地形可见图）；`global.cloud` 按 external（来自雷达驱动）声明。
`cli.py check` ok，digest `965ddcf9…`。

★装配差异清单（旧 FAR 图 vs M1 图）★：

| 项 | 旧 `graph_3d_gazebo_far.yaml` | M1 图 |
| --- | --- | --- |
| `perception`（dynamic_obstacles） | **无** | 有，且 `perception.obstacles → local.obstacles` |
| `loc`（localization_source） | **无** | 有，`loc.pose/localization` 分发给 bt/perception/local |
| `local.require_goal_seq` | **未设** | `true`（只有严格更新的代次能接管） |

### 7.2 实测拒动码（这是本轮要的"具体码"）

在对照图上发目标后读 `/v8/local/status`：

```json
{ ... "localization": 0.008, "obstacles_cb": 0, "max_age": 0.7 }
```

**`obstacles_cb: 0`** —— `local` 一条障碍消息都没收到；`local` 对障碍是 fail-closed 的，
因此不给速度。配套读数：`/v8/local/obstacles` 的**发布者存在**（`/v8/3d/perception/perception`），
但 `rostopic hz` 6 s 内 `no new messages`、`/cmd_vel` 仍 20 Hz 全零。

即：**不是 FAR 的路径、也不是 mux 的 hold**（mux 早前读数为 `reason=track`/`hold=false`），
而是**障碍心跳没来**。`dynamic_obstacles` 的实现自称"总是按 `~rate_hz` 发布（无点云时发空列表）"，
实测没发——这是下一步要修的装配/实现点，本轮**未修**（避免在没搞清前动 local 侧算法）。

### 7.3 path_goal 去重边界（第 3 条要求）

上一轮的去重键只有坐标，会吃掉两类合法下发；本轮改为 `(index, 坐标)` 并在取消时清空：

| 场景 | 行为 | 用例 |
| --- | --- | --- |
| 滚动路径每秒重发、index 与坐标都没变 | **不发**（避免 local 反复重开跟踪） | `test_control_duplicate_goal_not_republished` |
| index 前进但坐标恰好相同（连续同点） | **必须发**（同航点可连续步进） | `test_same_xyz_next_index_steps_continuously` |
| 取消后同坐标的新任务 | **必须发**（键随取消清空） | `test_after_cancel_same_xyz_publishes_again` |
| 取消后旧 stamp 的路径 | **拒绝重放**（既有 `accepts_path_stamp` 判据） | `test_path_goal_cancel.py` |

`local_planner` 全量 **181 项 OK**（含新增 7 项），框架 **345 项 OK**。

### 7.4 四项验收本轮仍全部 INVALID

| 项 | 判定 | 阻断读数 |
| --- | --- | --- |
| 真实到达 | **INVALID** | `obstacles_cb=0` ⇒ local 不给速度 ⇒ 位移 0 |
| 障碍安全（运动中） | **INVALID** | 无运动可观测；且本就不允许用假障碍数据绕过 |
| 定位失效停止 | **INVALID** | 动前即零速，无法区分"因失效归零"与"本来不动"；数据源隔离方案见 §7.5 |
| 运动中取消 | **INVALID** | 未达"先有速度"的触发条件 |

### 7.5 定位失效的隔离方案（下一轮照此做）

要求是"独立断定位数据、不得额外发布竞争性 `localization_ok`"。可行做法：
在 smokes 夹具里把 `/v8/local/pose` 的**唯一发布者**（`loc` 实例）停掉或加 `drop_pose` 开关，
观察 `/v8/local/status` 的 `localization` 年龄与 `/cmd_vel` 归零——而不是另起一个节点发
`localization_ok=false`（那会引入第二个写者，测的就不是"数据源失效"了）。
