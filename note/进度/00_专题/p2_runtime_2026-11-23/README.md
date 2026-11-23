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

