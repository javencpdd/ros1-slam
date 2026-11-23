# P2 运行期验收 · 隔离仿真（2026-11-23）

> 基线提交：`4c6ab686`。隔离 master：`http://127.0.0.1:11313`（容器 `ros1-slam-sim-viz`）。
> 原则：**只认实际订阅到的消息**；缺一条即判 FAIL。本目录下所有 json/log 都是运行时抓的，不是静态推断。

## 1. 四组合结果矩阵

| 组合 | 结论 | 依据 |
| --- | --- | --- |
| 传统 launch + support | **PASS（链路）** | 实跑：goal→planner goal→path→local goal→cancel ack 全部观测到，见 `trad_support/probe_trad_support.json` |
| 传统 launch + FAR | **INVALID** | `rospack find far_planner` / `far_planner_msgs` 均 not found（FAR overlay 未安装），见 `cleanup_and_env.txt` |
| graph Gazebo + support | **INVALID** | 图内 `carrier` 依赖 `/gazebo/set_model_state` 且为 `required`，本环境未起 Gazebo ⇒ 栈在 carrier 处整体退出；另 app 夹具引用的 `ws/src/95_simulation/...` 已被拆分，`app_runner` 报 `E_EXEC_BACKEND` |
| graph Gazebo + FAR | **INVALID** | 同 FAR 包缺失（且叠加 Gazebo 前置） |

> 静态治理的结论不因本表改写：graph 接线仍以 `test_graph_wiring.py` 为准；运行期未取得 PASS 的两组**不作功能结论**。

## 2. 传统 + support：逐项验收（实际观测）

| 验收项 | 结果 | 证据（trad_support/probe_trad_support.json） |
| --- | --- | --- |
| BT 收到有效目标 | ✅ | `goal_sent`，`goal_subscribers=1` |
| RequestPath 发出规划请求，后端收到 | ✅ | `planner_goal` 1 次（`/v8/global/goal`，map 帧） |
| 后端产生路径与状态，BT 收到 | ✅ | `path` 1 次；`status` 43 条（含 `E_PLAN_NOT_READY` → 就绪后的规划状态） |
| FollowPath/路径触发下游目标，local 收到 | ✅ | `local_goal` 1 次（`/v8/local/goal`） |
| 局部目标无双写者 | ✅ | 该链只有 `path_goal` 在写 `/v8/local/goal`（BT 写的是 `/v8/bt/goal_request`） |
| 连续目标队列语义 | **未验证** | 本轮只发 1 个目标 |
| 取消 ACK | ✅ | `cancel_ack` 1 次；BT 日志 `cancel requested seq=1 scope=all` |
| 取消后旧目标不再执行 | **部分** | 取消后 `local_goal` 未再增长（计数停在 1） |
| 超时 / lease 丢失 / 定位异常 | **未验证** | 本轮未注入这三类故障 |
| 正常终止后无残留运动指令 | ✅ | `cleanup_and_env.txt`：`/cmd_vel`、`/cmd_vel_raw`、`/v8/local/goal` 的 pubs/subs 均为 0 |
| 速度输出 | **仅验证零速** | 全程 `cmd_vel_raw = cmd_vel = 0`；relay 透传有效，但**非零速度未验证**（探针未构造 localization_ok 等可执行条件） |

**传统链速度语义**：`local → /cmd_vel_raw → m2_cmd_relay → /cmd_vel`，实测 relay 原样透传（0→0）。该链**没有** mux/限速/hold，不得据此宣称安全仲裁。

## 3. 防假绿：负例

| 负例 | 做法 | 结果 |
| --- | --- | --- |
| 断开全局规划器 | `rosnode kill /v8_support_planner_topic` 后跑同一探针 | **FAIL:缺 path,local_goal**（`neg/probe_negative_no_planner.json`）—— planner_goal 仍是 1（BT 照常发请求），但 path 与 local goal 缺失，说明探针能区分"发了请求"与"链路通" |

另有一次**探针自身的假绿拦截**：首版取消消息用 JSON（`{"seq":1,...}`），BT 判 `malformed cancel ignored` 且无 ACK ⇒ 探针判 **FAIL:取消无 ACK**。查 `bt_policy.hpp:72` 后改为契约文本 `"<seq>:<scope>:<reason>"` 才通过。这说明判据不是"发出去就算"。

## 4. 运行期发现（非本次修复范围，已记录）

| # | 发现 | 证据 |
| --- | --- | --- |
| F1 | 取消契约是**文本** `seq:scope:reason`，不是 JSON；JSON 会被静默丢弃（仅 WARN） | `bt.log`：`malformed cancel ignored`；`bt_policy.hpp:72` |
| F2 | 渲染出的 launch 里 `tree_file` 用**仓库相对路径**（`ws/src/...`），在容器内以 devel 前缀运行时解析失败 ⇒ BT 起不来；改用绝对路径后正常 | `graph_support/launch3.log` 第一条 ERROR；本轮临时改绝对路径后 BT 起来 |
| F3 | app 夹具 `nist_sensor_3d.yaml` 仍指向已拆分的 `ws/src/95_simulation/v8_simulation/config/sim_backends.yaml` ⇒ `app_runner` 直接 `E_EXEC_BACKEND` | `app.log` |

F2/F3 属"运行前置"类问题，未在本次修改（需单独授权改 graph 默认路径或夹具），但会挡住 graph 组合的运行验收。

## 5. 复现命令

```bash
# 环境
docker exec ros1-slam-sim-viz bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313'

# 传统 + support（节点级，无 Gazebo）
rosrun v8_platform run_map_file.py _map_json:=.../map_single_floor.json _map_topic:=/v8_support_map &
rosrun v8_platform run_global_planner.py _goal_topic:=/v8/global/goal _start_topic:=/v8/local/pose &
rosrun v8_platform run_path_goal.py _path_topic:=/global_path _goal_topic:=/v8/local/goal &
rosrun v8_platform run_local_planner.py _goal_topic:=/v8/local/goal _twist_topic:=/cmd_vel_raw &
roslaunch lite3_bt bt_console.launch profile:=demo &
python3 $(rospack find lite3_gazebo_sim)/scripts/m2_cmd_relay.py /cmd_vel_raw /cmd_vel Twist &

# 探针（仓内脚本）
python3 /opt/ros1-slam/scripts/verify/p2_runtime_probe.py --combo trad_support \
  --evidence-dir /tmp/p2/trad_support --goal 1.0 0.0 --duration 15 --planner-goal-topic /v8/global/goal
```

graph 组合：`cli.py plan` 渲染到 `/tmp/g.launch` 后 `roslaunch`，需先把 `tree_file` 改绝对路径，并起 Gazebo 或去掉 `carrier`（本轮两次尝试均因 carrier/gazebo 前置失败，记 INVALID）。

## 6. 未验证清单（明确）

- 非零速度、轨迹跟踪、到达判据（`WaitArrival`）
- 连续目标队列、超时、lease 丢失、定位异常
- graph 两组合的完整链路（含 mux 超时归零/限速/hold）
- FAR 两组合（overlay 缺失）
- 真机（禁止）

## 7. 与正式验收的关系

本轮只证明「传统 + support 的**接线**在真实运行图里成立」，**不构成** formal 签署、真机验收或能力升级；VCC/formal 状态保持原样。

---

## 8. 第二轮（F2/F3 修复后重跑）

### 8.1 F2 根因与修复（tree_file）

| 环节 | 事实 |
| --- | --- |
| graph 参数 | `tree_file: ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml`（**仓库相对**） |
| app_runner 路径 | `app_runner_prepare.py:129-130` 会用 `app_config._path()` 拼成绝对路径 ⇒ 该入口本来正常 |
| 直接 roslaunch | 渲染出的 launch 原样保留相对路径；节点 cwd 是 `devel/lib/...` ⇒ `createTreeFromFile` 失败，BT 起不来 |

**修复**（`render.py` 新增 `_pkg_rel()`）：渲染时把「仓库相对、且落在某个 catkin 包内」的路径改写成 `$(find pkg)/子路径`，
由 roslaunch 解析；非路径、绝对路径、找不到包的一律原样返回（不猜）。同时 `app_config._path()` 放行 `$(find …)`，
避免 app 入口把它当相对路径再拼一次。

**验证**：四张受支持图渲染结果均为 `name="tree_file" value="$(find lite3_bt)/trees/navigate_with_planner.xml"`；
运行期 BT 能正常加载树（第二轮 graph 运行中 BT 起在 `/v8/bt/bt`）。
测试：`tests/test_asset_paths.py`（5 项，含「缺失路径原样返回」反例）。

### 8.2 F3 根因与修复（app 资产路径）

两个活跃夹具 `nist_sensor_3d.yaml` / `nist_sensor_3d_lio.yaml` 的 `simulation_declaration` 仍指向
`ws/src/95_simulation/...`（分仓后已不存在），实际资产在 `subprojects/simulation/ws/src/v8_simulation/config/sim_backends.yaml`。
已改指向新位置；测试断言「活跃夹具不再含 `ws/src/95_simulation`」「缺失资产抛 `E_EXEC_BACKEND`」。

### 8.3 Gazebo 前置检查（本轮实测通过）

| 检查 | 结果 |
| --- | --- |
| 隔离 master | `http://127.0.0.1:11313`（容器内，未连真机） |
| gzserver | Gazebo 11.15.1，可独立启动（`app_world.launch gui:=false`） |
| `/gazebo/set_model_state` | **存在**（另有 spawn_sdf/urdf 服务） |
| 插件与模型 | `libgazebo_ros_api_plugin.so` 在；`GAZEBO_MODEL_PATH` 含 `sim_ws/src` 与 worlds |

### 8.4 graph + support：仍 **INVALID**，新阻断 F4

Gazebo 起来后 `carrier` 正常启动、BT 也起来了（说明 F2 修复生效），但 **6 个节点只起了 2 个**：

```
Cannot locate node of type [framework/nodes/map_file_node.py] in package [v8_platform]
Cannot locate node of type [planner/topic_node.py] in package [v8_algorithm]
Cannot locate node of type [local_planner/ros_node.py] in package [v8_algorithm]
Cannot locate node of type [local_planner/cmd3d_mux_node.py] in package [v8_algorithm]
```

- 前三个文件在 `devel/lib/<pkg>/<子目录>/` 里**确实存在且可执行**；`cmd3d_mux_node.py` 连安装目录里都没有。
- 原因：roslaunch 解析 `type` 时只查 `devel/lib/<pkg>/` 的**顶层**可执行名，**不递归子目录** ⇒ 带 `/` 的 type 一律找不到。
- 既有 `run_*.py` shim（如 `run_global_planner.py`）就是为绕开这一点而存在，但 graph 的 manifest 仍写子目录路径。

**结论**：graph 两组合仍是 **INVALID**（阻断在 F4，与本轮 F2/F3 无关）。要跑通需三者之一：
①给四个 type 造顶层 shim 并把 manifest `type` 改为裸名（会动 contract）；
②render 把 `type` 压成 basename + 提供 shim（`ros_node.py` 在 planner 与 local_planner 下同名，需要消歧）；
③catkin 侧改为 `CATKIN_PACKAGE_BIN_DESTINATION` 顶层安装（改构建）。
三条都超出"最小修改"，需要单独授权。

### 8.5 重要更正

上一轮写「已观测到 BT 的 RequestPath 到达 `/v8/3d/global/goal`」——**只说明 BT 发出去了**。
本轮在 graph 组合里该话题没有任何订阅者（规划器节点没起来），因此**不能**据此说"目标到达后端"。
这条更正已在 §2 的说明中生效。

---

## 9. F4 修复与 graph 再跑（方案 1：顶层 shim + manifest type）

### 9.1 四个入口的核查结论

| 组件 | 实现 | 既有等价 shim | 处置 |
| --- | --- | --- | --- |
| `v8.fw.map_file`（`framework/nodes/map_file_node.py`） | 存在可执行 | **有**：`v8_platform/scripts/run_map_file.py`（同一 `sys.path` 写法，导入同一实现） | **复用**，manifest `type` → `run_map_file.py` |
| `v8.fw.global3d_topic`（`planner/topic_node.py`） | 存在可执行 | 无包内等价件（`run_global_planner.py` 在 `v8_platform`，跨包） | **新增** `v8_algorithm/scripts/run_topic_node.py` |
| `v8.fw.local_native_raw`／`v8.fw.local_native`（`local_planner/ros_node.py`） | 存在可执行 | 无包内等价件 | **新增** `v8_algorithm/scripts/run_local_ros_node.py`（两个 manifest 共用） |
| `v8.fw.cmd3d_mux`（`local_planner/cmd3d_mux_node.py`） | 存在可执行 | 无 | **新增** `v8_algorithm/scripts/run_cmd3d_mux.py` |

三个新 shim 与既有 `run_*.py` 完全同构：`rospkg` 定位包路径 → `sys.path.insert` → `import <impl>` → `<impl>.main()`，
**不复制实现、不写死绝对路径**；`CMakeLists.txt` 加了 `catkin_install_python` 声明。
`package_index.json` 补了三条**裸名**条目（`launch.check_components` 按 `pkg/type` 精确匹配，`scripts/` 前缀会不匹配）。

**未造空节点**：四个实现都真实存在且被 shim 导入；无实现的组件未做任何占位。

### 9.2 可发现性与启动验证

| 检查 | 结果 |
| --- | --- |
| `roslib.find_node` 四个 type | 全部命中（`v8_platform/scripts/run_map_file.py`、`v8_algorithm/scripts/run_*.py`） |
| `roslaunch` 的 `Cannot locate` | **0 条**（此前 4 条） |
| 实跑节点数 | **5/6**：`global`、`local`、`mux3d`、`bt` 全起（此前仅 2） |
| 单测 | **361 项 OK（skip 9）**（新增 `tests/test_node_entrypoints.py`） |

### 9.3 仍未 PASS 的两个前置（非 F4）

| # | 阻断 | 证据 |
| --- | --- | --- |
| F5 | `map_file_node` 报 `FileNotFoundError: data/maps/generated/nist_sensor_v2.json`（该文件**在仓内存在**），因节点 cwd 是 `.ros` 而路径是**仓库相对**——与 F2 同一类问题，只是发生在节点侧默认值 | `graph_support/live3_tail.log` |
| Gz | `app_world.launch` 起 gzserver 后**立即退出**（`all processes on machine have died`），导致 `carrier` 无法启动；同一 world 早前曾成功起过一次，属**不稳定** | `graph_support/gz3.log` |

⇒ **graph + support 仍判 INVALID**（前置不成立），但 F4 目标（节点能被找到并启动原有实现）**已达成**。

### 9.4 cmd3d_mux 的真实能力（只按实现验收）

`local_planner/cmd3d_mux_node.py` 的 `arbitrate()`：`hold → 0`、超时 0.4 s → `0 + E_CMD3D_STALE`、`frame != map → 0 + E_CMD3D_FRAME`、
非法速度 → `0 + E_CMD3D_INVALID`；**单一输入源**，**没有 speed_cap、没有多源优先级**。

本轮实跑观测到两条真实状态：
- `{"reason": "E_CMD3D_STALE", "velocity": [0,0,0,0]}`（无输入时归零）✅
- `{"reason": "track", "velocity": [0,0,0,0], "input_frame": "map", "age_s": 0.0029}`（有新鲜输入、但命令本身为零）✅

因此：**mux 的「单写者 / 超时归零 / frame 校验 / hold」可验；speed_cap 与优先级不得记 PASS**（实现里没有）。

### 9.5 非零运动与到达：**未取得证据**

Gazebo 未起来 + 地图未加载 ⇒ 无 carrier 位姿、无 path、`/cmd_vel` 全程为 0。**不能**据此声称作动或到达。

---

## 10. F5 修复与 Gazebo 诊断（第三轮）

### 10.1 F5 根因与修法

| 项 | 事实 |
| --- | --- |
| 资产位置 | `data/maps/generated/nist_sensor_v2.json`（仓库 `data/`，**不在任何 catkin 包内**） |
| graph 声明 | `graph_3d_gazebo_bt.yaml:14` 显式写 `map_json: data/maps/generated/nist_sensor_v2.json`（仓库相对） |
| 节点读法 | `map_file_node.py` 直接 `open(~map_json)` ⇒ 依赖 cwd（实际是 `.ros`） |
| 首版修法为何失败 | 按 `__file__` **硬数层数**求仓库根；profile 视图（`ws/.profiles/<t>/src/...`）层数不同，算到 `ws/` ⇒ 仍找不到 |

**最终修法**：`_repo_root()` 改为**向上找标记**（同时含 `data/maps` 与 `.git` 的目录），两种运行视图都能定位；
`resolve_map_path()` 对绝对路径直用、对相对路径按仓库根解析、**缺失即抛 `FileNotFoundError` 并带原值与尝试路径**（不静默兜底）。
TDD 覆盖：`test_asset_paths.py` 11 项（仓库相对可解析 / 绝对路径直用 / 缺失明确失败 / graph 声明与磁盘一致 / app 与 roslaunch 同一份）。

### 10.2 Gazebo 退出原因（先诊断后修）

| 检查 | 结果 |
| --- | --- |
| 原始 stderr / 退出码 | `app_world.launch` 只留 `all processes on machine have died`；**stderr 为空**，无 OOM（内存 31 GB、可用 17 GB） |
| 残留进程 | **发现 2 个陈旧 gzserver**（pid 50022/50060，均为 `lite3_gazebo_sim/worlds`） |
| 三次独立 headless 启动（同一 world，逐次清理本次 PID） | **3/3 存活**；不带 ROS 插件时无 `/gazebo/*` 服务（插件由 `gazebo_ros/empty_world.launch` 加载，属预期） |

⇒ 退出原因是**上一次运行留下的 gzserver 占用 gazebo master/端口**，新进程起不来，不是 world 或资源问题。
**最小修复**：启动前按「命令行包含 `lite3_gazebo_sim/worlds`」**限定范围**清理陈旧 gzserver（不 `pkill 全部`，不误杀其它 master）。
**未换 world、未用 Xvfb。**

### 10.3 graph + support 实跑结果（分级）

| 等级 | 结果 | 证据 |
| --- | --- | --- |
| **启动级** | **PASS**：7/7 进程在运行——`gazebo`、`map`、`global`、`local`、`mux3d`、`bt`、`carrier` | 节点列表；`live4_tail.log` |
| **链路级** | **FAIL**：`planner_goal` 到达（BT 发到 `/v8/3d/global/goal`），但 **global 未产出 path**；`local_goal` 无；取消 ACK 有 | `probe_graph_support.json`、`wiring_diag.txt` |
| **运动级** | **未取得**：`/cmd_vel` 全程 0；`mux` 报 `reason=track, velocity=[0,0,0,0]` | 同上 |
| **导航级** | **未取得**：无路径、无到达、BT 未终止 | 同上 |

**链路失败的两个直接读数**（下一步的入口）：
1. `/v8/3d/global/status` = `{"ok": false, "reason": "E_PLAN_NOT_READY", "map_version": 2}`；
2. `/v8/3d/global/start` 的 **Publishers: None**（global 是该话题唯一订阅者），而 graph 声明了 `carrier.pose → global.start` 边；
   同时 `/v8/3d/global/goal` 的 **Subscribers 为空**（BT 在发，global 未订）——需要核对 `topic_node` 的订阅时机与 remap 落点。

### 10.4 已确认的能力（只按实现）

- `cmd3d_mux`：单写者、`hold→0`、0.4 s 超时→`0 + E_CMD3D_STALE`、`frame!=map→E_CMD3D_FRAME`；**无 speed_cap、无多源优先级**。
- 实跑观测到 `track` 与 `E_CMD3D_STALE` 两种真实状态；`track` 时速度为零是因为上游命令本身为零（无路径），**不能算运动验收**。

### 10.5 环境清理

收尾：`rosnode kill -a` + 按 world 路径杀本次 gzserver ⇒ 剩余节点仅 `/rosout`，gzserver 计数 0，`/cmd_vel` 与 `/cmd_vel_raw` 的发布者均为 0（`final_cleanup3.txt`）。

---

## 11. E_PLAN_NOT_READY 定因与链路打通（第四轮）

### 11.1 真实 ROS 图证据（不是 YAML 推断）

| 项 | 实测 |
| --- | --- |
| global 订阅表（`rosnode info`） | `/v8/3d/global/goal`、`/v8/3d/global/start`、`/v8_support_map`、`/clock` |
| global remap（渲染件） | 只有 `/v8/3d/global/goal`（恒等）与 `/v8_support_map`（恒等）；**start 没有 remap** |
| `/v8/3d/global/start` | Publishers: **None**，Subscribers: global —— 起点没人发 |
| `/v8/local/pose` | 发布者 carrier；订阅者 bt、local（**global 不在其中**） |
| `/v8/3d/global/goal` | 发布者 bt；订阅者 global ✅ |
| 图里 `global.start` | `external: {to: global.start, source: operator}` —— 是**外部输入**，不是 carrier 的边 |

### 11.2 定因（逐行源码）

`planner/topic_node.py:89-92` 的 `replan()`：`state['map'] is None or state['start'] is None or state['goal'] is None`
→ 发 `E_PLAN_NOT_READY`。本次实测：map 已到（`map_version: 2`）、goal 已到、**start 缺失** ⇒ 恒不发规划。
`map_version=2` 只是地图 JSON 的版本号，**不等于**规划就绪；就绪还要求 start 与 goal 同时存在。

⇒ 不是 remap 错、也不是订阅时机错，而是**操作员没有提供声明式外部输入（起点）**。这与 FAR 图的差别正在这里：
FAR 图显式写了 `carrier.pose → global.start` 的边（`graph_3d_lite3_far.yaml:206`），support 图没有。

### 11.3 最小修复

**不改规划器、不伪造 ready、不注入假目标**：探针按图契约补发**起点**（`--start-topic`，默认 `/v8/3d/global/start`，latch 发布），
并顺带把 `path/status/local_goal` 话题参数化（此前写死 `/global_path`，而本图发 `/v8/3d/global/path`，导致误判"缺 path"）。

### 11.4 修复后实测（同一 master 11313，隔离仿真）

| 验收项 | 结果 | 证据 |
| --- | --- | --- |
| global 收到目标与起点 | ✅ | probe `start_sent`、`goal_sent`、`planner_goal` |
| 地图就绪 | ✅ | `/v8_support_map` 有发布者；`map_version: 2` |
| global 产出 path/status | ✅ | `/v8/3d/global/status` = `{"ok": true, "points": 16, "expansions": 97, "goal": [1.5,0,0]}` |
| BT 收到 path/status | ✅（间接但明确） | BT 由 `WaitPath` 转 `FollowPath` 并下发局部目标（`local_goal` 事件） |
| local 收到执行目标 | ✅ | probe `local_goal` = (1.5, 0.0) |
| 取消 ACK | ✅ | `cancel_ack` = `1:cancelled` |
| cmd3d_mux 状态 | ✅（真实读数） | `{"reason": "track", "velocity": [0,0,0,0], "input_frame": "map"}`；另一轮 `E_CMD3D_STALE` |

### 11.5 分级结论

| 等级 | 结论 |
| --- | --- |
| **启动级** | **PASS**：7/7（gazebo、map、global、local、mux3d、bt、carrier） |
| **链路级** | **PASS**：目标→规划→路径→BT→局部目标→取消回执 全部有实际读数 |
| **运动级** | **FAIL**：`/cmd_vel` 全程 0、`/v8/local/pose` 恒为 (0, -3.06)；BT `root=RUNNING` 且 `event=localization_ok` |
| **导航级** | **NOT RUN**（无运动即无到达） |

**运动级下一步**：`localization_ok` 有发布者（carrier）但 BT 仍停在 `localization_ok` 节点，需核对 carrier 实际发布的 Bool 值
与 local 的同类门控；这不是"没接线"，是**门控输入的值/时序**问题。

---

## 12. 运动级 FAIL 定因（第五轮，只定位不接线）

### 12.1 首个阻断组件：local（`v8.fw.local_native`），条件是**障碍心跳缺失**

| 证据 | 读数 |
| --- | --- |
| `ros_node.py:157` 注释与实现 | `Missing localization or obstacle heartbeat is unsafe, including at startup.`；`state` 里 `obstacles_at=-1e100`、初始 `last_error=E_LOCAL_NOT_READY` |
| `/v8/local/obstacles` | **Publishers: None**；Subscribers: `/v8/3d/local/local` |
| 渲染 launch 的 local 节点 | 只有 cancel/goal/localization/pose/map 的 remap，**没有 obstacles** |
| 该图组件清单 | `map, global, bt, local, mux3d, carrier` —— **没有 perception/obstacles 生产者** |

⇒ local 按**设计**在缺障碍心跳时不下发速度（fail-closed），这是安全语义，不是 bug。
**运动失败的第一个阻断点就在这一层**，与 mux、Gazebo、规划都无关。

### 12.2 三层输出对照（证明零速来源在 local）

| 层 | 实测 |
| --- | --- |
| local 输入 | pose=(0,−3.06,0) frame `map` stamp 25.4 s 新鲜；`localization_ok`=**true**（18 s 内 425 条）；goal 已到；**obstacles 缺失** |
| local 输出 `/v8/local/cmd_vel_3d` | 823 帧，**全 0**，frame `map` |
| mux `/cmd_vel` | `{"reason":"track","velocity":[0,0,0,0],"input_frame":"map"}` —— 忠实透传 0 |
| Gazebo 载具 | pose 恒为 (0,−3.06,0)，无位移 |

⇒ 属于「local 自己输出零速」，**不是** mux 阻止，也**不是** Gazebo 不动。

### 12.3 起点与位姿一致性（外部输入不伪造）

| 项 | 值 |
| --- | --- |
| carrier 实测位姿 | (0.000, −3.060, 0.000)，frame `map` |
| 探针发送的 start | (0.000, −3.060)，由**当前位姿生成**（非固定常量） |
| 规划路径首点 | (0.150, −3.150)，6 点，frame `map`；与 start 差 0.15/0.09 m（规划器起点吸附量级） |

### 12.4 A/B 分离测试

| 组 | 做法 | 读数 |
| --- | --- | --- |
| **A 正常执行** | 发 start+goal，**14 s 窗口内不取消** | `cmd_vel_3d` 823 帧全 0；`cmd_vel` 834 帧全 0；pose 不变；mux `track` |
| **B 取消** | 发 goal → 3 s 后 `2:all:probe-B` | **100 ms** 内收到 `2:cancelled`；`cmd_vel` 345 帧全 0；节点顺序有时间戳 |

⇒ A 组窗口内速度恒零，才是"运动失败"的判据；B 组只证明取消链正常（ACK 及时）。

### 12.5 本轮未做修复（受"不改 graph 接线"约束）

两条候选，都需授权，且**都不能靠伪造**：

| 选项 | 做法 | 风险 |
| --- | --- | --- |
| A（推荐） | 给 Gazebo support 图补 `perception`（`v8.fw.dynamic_obstacles`）实例与 `perception.obstacles → local.obstacles` 边 | 动 graph 接线；需确认赛道的感知配置在仿真域可用 |
| B | 若该图设计上**不打算**有障碍源，则应在 local 侧提供"无感知域"的显式开关（配置项），默认仍 fail-closed | 动算法契约，需评审 |

**不做的事**：不写死 `localization_ok=true`、不关 stale 检查、不跳 hold、不伪造速度、不注入 Gazebo 位姿。

### 12.6 分级（本轮维持）

| 等级 | 结论 |
| --- | --- |
| 启动级 | **PASS** 7/7 |
| 链路级 | **PASS**（path 6 点、局部目标、取消 ACK 均有读数） |
| 运动级 | **FAIL**（首个阻断＝local 缺 obstacles 心跳） |
| 导航级 | **NOT RUN** |

---

## 13. 方案 A 前置判定：**环境 INVALID（Gazebo 无真实点云）**

用户批准「给 Gazebo support 图补 `v8.fw.dynamic_obstacles` + 边」，并附三条硬条件。按条件 1 先验证"Gazebo 是否真提供点云"：

| 检查 | 读数 |
| --- | --- |
| 运行中话题总数 | 36 |
| 含 `points` / `cloud` / `lidar` / `scan` 的话题 | **（none）**——`/rslidar_points`、`/velodyne_points`、`/livox/lidar`、`/cloud_registered` 全部不存在 |
| 世界模型 | `ground_plane`、`perimeter_north/south`、`obs_1` …、`app_carrier`（`rosservice call /gazebo/get_world_properties`） |
| 传感器定义 | 只在 `lite3_planar.urdf.xacro` 与 `Mid360_simulation_plugin/.../model.sdf` 里；本图使用的 `app_carrier` 模型**未带激光** |
| 世界插件 | `m1_world.world` 内无 `ray/gpu_ray/lidar/topicName` |

⇒ **本环境不存在任何真实点云源**。按硬条件「若 Gazebo 没有可用的真实点云，应记录环境 INVALID，不能制造运动 PASS」：

- **不添加** `perception` 实例与 `perception.obstacles → local.obstacles` 边：加了也没有数据，只会让图看起来"接了感知"，属误导。
- **不动** local 的 fail-closed：它现在的零速正是"感知缺失必须拒绝运动"的正确行为（§12 A 组 823 帧全零即该反例的实证）。
- 也**未**新增无感知开关（按指示不选 B）。

### 13.1 继续 A 方案需要的前置（需另行授权，且都会改仿真资产）

| 步骤 | 内容 | 说明 |
| --- | --- | --- |
| P1 | 让 `app_carrier` 带激光：spawn 带传感器的模型，或给 `m1_world.world` 加 `ray/gpu_ray` 插件并把话题对齐到 `dynamic_obstacles` 的 `cloud` 端口（默认 `/cloud_registered`） | 改仿真资产/SDF，不是接线 |
| P2 | 确认点云**持续**更新（`rostopic hz` 有稳定频率、`header.frame_id` 与 `map` 一致） | 硬条件 1 |
| P3 | 给 `v8.fw.dynamic_obstacles` 补顶层 shim（manifest 现写 `framework/nodes/dynamic_obstacles_node.py`，属 F4 同类） | 否则 roslaunch 起不来 |
| P4 | 再往图里加实例与边，然后跑：正常→非零速度/位姿变化；**杀感知→必须回零**；取消→ACK；到达 | 硬条件 1—3 |

### 13.2 本轮分级（维持，未强行升级）

| 等级 | 结论 | 依据 |
| --- | --- | --- |
| 启动级 | **PASS** 7/7 | 前轮 |
| 链路级 | **PASS** | 前轮（path 6 点、局部目标、取消 ACK） |
| 运动级 | **INVALID**（原 FAIL 已定因：local 缺障碍心跳；补感知的**环境前置**不成立） | §12 + §13 |
| 导航级 | **NOT RUN** | 依赖运动级 |

---

## 14. P1～P4 连续执行（第六轮）

### 14.1 P1 传感器：**PASS**

给 `gazebo_carrier3d_node.py` 的内联 SDF 加了 Gazebo 原生 ray 传感器（`libgazebo_ros_block_laser.so` → **真实 PointCloud2**）：

- 参数：`update_rate=10`、水平 360 线、垂直 8 层、量程 0.15–12 m、`gaussianNoise=0.005`；
- `frameName = app_carrier/link`（**真实 link 名**，不是把 frame 写成 map）；
- 载体模型名、位姿来源、控制接口与既有导航链未改。

| 读数 | 值 |
| --- | --- |
| `/cloud_registered` 频率 | **10.03 Hz**（两次采样：9.967 / 10.033） |
| `frame_id` | `app_carrier/link` |
| TF `map → app_carrier/link` | 存在，平移 (0.000, −3.060, 0)，与 carrier pose 一致 |

### 14.2 P2 点云与 TF：**PASS（部分项保持未测）**

- 点云连续更新、时间戳随 `/clock`（gazebo 由 `empty_world.launch` 提供 `/clock`，节点订阅 `/clock`）；
- TF 链：`map → app_carrier/link`（由 carrier 节点按自身 pose 广播，与 pose 同源）；
- **空场景差异**：ROI 内无障碍时 `dynamic_obstacles` 输出 `[]`（合法空检测）；
- 未构造障碍场景（本轮无新增障碍物需求，留待后续）。

### 14.3 P3 感知组件与接线：**PASS**

| 项 | 处置 |
| --- | --- |
| manifest | `v8.fw.dynamic_obstacles` 的 `type` 由 `framework/nodes/dynamic_obstacles_node.py` 改为 `run_dynamic_obstacles.py`（F4 同类） |
| shim | 新增 `v8_platform/scripts/run_dynamic_obstacles.py`；该模块**无 main()**，入口是 `DynamicObstaclesNode().spin()`，节点名由 `__name:=perception` 覆盖 |
| package index | 补 `run_dynamic_obstacles.py` 裸名条目 |
| graph | `graph_3d_gazebo_bt.yaml` 新增 `perception` 实例（`cloud_topic: /cloud_registered` 等）、边 `carrier.pose→perception.pose`、`perception.obstacles→local.obstacles`、外部输入 `perception.cloud ← lidar`；**并删除**旧的 `local.obstacles` 外部声明（否则 `E_GRAPH_INPUT_DUP`） |

| 读数 | 值 |
| --- | --- |
| 节点数 | **8/8**（含 `/v8/3d/perception/perception`） |
| `/v8/local/obstacles` | **10.0 Hz**，内容 `[]`（空检测，来自真实点云处理，非独立假心跳） |
| `/v8/3d/perception/status` | 曾报 `no_cloud`，随后按 10 Hz 稳定发布 |

**未改** local 的 fail-closed、未关闭新鲜度检查。

### 14.4 P4-A 正常导航：**部分 PASS（运动命令已出，位姿未推进）**

| 验收项 | 结果 | 证据 |
| --- | --- | --- |
| BT 收到目标 | ✅ | `goal_sent` |
| global 生成路径 | ✅ | `path` 6 点，首点 (0.15, −3.15) |
| BT 收到路径 | ✅ | 由 WaitPath 转 FollowPath |
| local 收到目标/位姿/地图/新鲜障碍 | ✅ | `local_goal`、pose、map、obstacles 10 Hz |
| **local 输出非零速度** | ✅ | `cmd_vel_3d` 由全 0 变为非零（此前 823 帧全 0） |
| **cmd3d_mux 输出非零** | ✅ | `{"reason":"track","velocity":[0.597, 0.0597, 0, 0],"input_frame":"map"}` |
| **Gazebo 载体位姿变化** | ❌ | 20 s 内 `/v8/local/pose` 恒为 (0, −3.06) |
| 到达 / BT 终止 / 结束零速 | ❌ | 无位移即无到达 |

**新阻断（下一轮入口）**：carrier 订阅的是 `/v8/local/cmd_vel_3d`（TwistStamped，图内 remap 结果），
而非 mux 输出 `/cmd_vel`；需要确认它的运动积分是否读这个端口、以及 `snap_m`/地图贴面是否把位移夹回原处。
这是**运动链最后一环**，不是感知问题。

### 14.5 P4-B/C/D：**NOT RUN**（依赖 A 的位姿推进）

### 14.6 分级

| 等级 | 结论 |
| --- | --- |
| 启动级 | **PASS 8/8** |
| 链路级 | **PASS** |
| 运动级 | **部分 PASS**：非零命令已出；位姿未推进 ⇒ 不能记运动 PASS |
| 导航级 | **NOT RUN** |

---

## 15. 运动级打通与 P4 全项（第七轮）

### 15.1 新根因：渲染器把 mux 的输出当成它的输入

`cmd3d_mux` 的端口 `cmd`：**输入**侧读 `~cmd_topic`、**输出**侧读 `~output_topic`。渲染器统一按
`<port>_topic` 推导，于是 `mux3d.cmd → carrier.cmd` 这条边给载体生成了

```
<remap from="/cmd_vel" to="/v8/local/cmd_vel_3d"/>
```

载体声明的是 `Twist`、而 `/v8/local/cmd_vel_3d` 是 `TwistStamped` ⇒ **消息被静默丢弃**，速度恒为 0。
这正是上一轮"命令非零但车不动"的原因。

**修复**：`render.py` 新增 `PORT_PARAM_OVERRIDES`（端口×方向 → 真实参数名），remap 推导按两端**各自方向**取参数：

| 组件 | 端口 | 方向 | 参数 |
| --- | --- | --- | --- |
| `v8.fw.cmd3d_mux` | `cmd` | outputs | `output_topic` |
| `v8.fw.local_native_raw` | `cmd` | outputs | `command_topic` |
| `v8.fw.local_native_raw` | `cmd_twist` | outputs | `twist_topic` |

修后载体 remap 变恒等 `/cmd_vel → /cmd_vel`。回归：`test_graph_wiring.py::test_mux_output_not_confused_with_its_input`
（断言载体 remap 指向 mux 输出、且不含 mux 输入话题）。

### 15.2 P4-A 正常导航：**PASS**

| 验收项 | 读数 |
| --- | --- |
| 起点 / 目标 | (0.000, −3.060) → (1.200, −3.060)，frame `map` |
| 路径 | 6 点，首点 (0.150, −3.150) |
| **非零速度** | **96 帧**，`vx = 0.597 m/s`（`t=2.85 → 10.62`） |
| **位姿变化** | (0.000, −3.060) → **(0.934, −3.033)** |
| **到达** | 判据 0.35 m：实测偏差 **0.27 m** ⇒ 到达成立，用时约 **8 s** |
| 结束零速 | 到达后 `/cmd_vel` 采样为 0；后续目标期间 mux 仍报 `track` 0 |

> 诚实说明：**BT 对该到达目标的最终成功状态未捕获**（后一次 `goal_seq=4` 因断感知而 `outcome=failed`，
> 覆盖了状态快照）。到达是以位姿判据确认的，不是以 BT 状态确认的。

### 15.3 P4-B 断感知：**PASS**

`rosnode kill /v8/3d/perception/perception` 后 3 s 再发新目标：

- `/cmd_vel` 全部为 **0**（未执行新目标），位姿停在 (1.700, −3.060) 不再推进；
- mux 报 `{"reason":"track","velocity":[0,0,0,0]}` —— 是 local 侧 fail-closed 拒绝下发，**未改任何安全开关**。

### 15.4 P4-C 取消：**PASS**

运动中发 `7:all:p4-C`：**60 ms** 内收到 `7:cancelled`，之后 `/cmd_vel` 全 0；BT 状态里
`cancel_ack: cancelled`、`cancel_origin: operator`。

### 15.5 P4-D mux 行为：**部分 PASS（只记实现具备的）**

| 项 | 结果 | 读数 |
| --- | --- | --- |
| hold 归零 | ✅ | `{"reason":"hold","velocity":[0,0,0,0],"hold":true}` |
| 输入超时归零 | ✅ | `{"reason":"E_CMD3D_STALE","velocity":[0,0,0,0]}`（早前观测） |
| 单写者 | ✅ | plan `command_owner=mux3d`；`/cmd_vel` 仅 mux 发布 |
| frame 校验 | **NOT RUN** | 未构造非 map 帧输入 |
| 非法值拒绝 | **NOT RUN** | 未构造 NaN/长度非法输入 |
| speed_cap / 多源优先级 | **不存在** | 实现里没有，不记 PASS |

### 15.6 分级

| 等级 | 结论 |
| --- | --- |
| 启动级 | **PASS 8/8** |
| 链路级 | **PASS** |
| 运动级 | **PASS**（非零速度 + 位姿推进 + 到达 0.27 m + 结束零速） |
| 导航级 | **PASS（位姿判据）**；BT 终态未捕获，记一条待补 |

### 15.7 未解决 / 待补

1. BT 对到达目标的终态证据（下轮用「到达即抓 status」的取样点补）；
2. mux 的 frame 校验与非法值拒绝未构造用例；
3. 障碍场景（ROI 内有障碍时的检出与绕行）未跑。

