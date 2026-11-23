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
