# BT-ABC · 编排闭环与标准接口（2026-11-14）

> 任务：`T-20261114-03`（qoder）。范围：A 组 BT 编排语义、B 组标准组件接口、C 组子项目配套；D（真机/M20）与正式判据不在本片。
> 本目录只放文档与数据；代码真身在 `ws/src/` 与三仓。运行日志 `.runtime/` 不入库。

## 1. 窗口

| 窗口 | 结果 | 读数 |
| --- | --- | --- |
| `ABC-upstairs-r1` | 失败（保留） | `E_ORCH_READY_TIMEOUT: 未就绪: carrier`；根因：`catkin_install_python` relay 的 `sys.path[0]=devel/lib/v8_simulation`，`import carrier_kinematics` 在 `rospy.init_node` 前失败；修复 `d394bd1` |
| `ABC-upstairs-r2` | **ok=true** | 六项 checks 全 true；57 poses，末点 `(-5.9,1.1,1.6)`；`local reason=arrived`；`carrier_final=(-5.736089848500734,1.1234157188905518,1.6)`；markers 40；preflight rc=0 |

命令（`ros1-slam-sim-viz`，`/opt/ros1-slam`）：

```bash
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /opt/ros1-slam/sim_ws/devel/setup.bash
V8_FORMAL_ALIAS_ROOT=/home/ros1-slam \
python3 -B scripts/sim/two_floor_nav.py --case upstairs --execute --orchestrator bt \
  --window-id ABC-upstairs-r2 \
  --out note/进度/v8/M5/26-11-14/evidence/BT-ABC/upstairs-r2/summary.json \
  --port 11659 --timeout-s 120
```

## 2. A 组读数

- **A1 取消（真实执行端）**：BT + `local_planner/ros_node.py`，发 `7:all:real-ack` → BT 状态
  `{event:cancelled, outcome:cancelled, cancel_requested:false, cancel_ack:cancelled, cancel_scope:all}`。
  负控：无 ack 时 `event/outcome=cancel_unacked`，不静默成功。旧 `Stop` 的零速不作为取消。
- **A2 恢复**：两目标任务、每目标 `path=1.0;retry=1`、`recovery_mode=retry`。日志四次
  `global path timeout after 1.0 s`；终态
  `{event:retry_exhausted, outcome:failed, current_seq:4, goal_seq:4, queue_depth:0, attempts:1, goal_index:2, goals_total:2}`
  —— 重试 1 次、耗尽后跳过并进入第 2 个目标。
- **A3 流程可配置**：定位丢失（`/v8/local/localization_ok=false`）+ 目标 →
  `{event:recovered_skip, outcome:failed, goal_seq:1, goal_index:1, goals_total:1, localization_ok:false, localization_check:true}`；
  每目标 `path/arrival/dwell` 由 mission wire 进入 BT（路径超时 1.0 s 已实测）。
- **A4 多目标/巡检**：`app_runner --app inspection_bt.yaml` 干跑 `ok=true`，3 个有序目标
  `upper_landing/upper_north/upper_south`，`resolved.goals` 3；BT 侧 FIFO 队列按序消费。
- **A5 执行管理**：`reached/failed/cancelled/cancel_unacked` 四态；取消 ack、失败原因、恢复动作在 status；
  安全 mux 的 hold/看门狗仍独立停车。
- **A6 观测**：`/v8/bt/status` 新增 `cancel_requested/cancel_scope/cancel_ack/attempts/recovery_mode/goal_index/goals_total/localization_ok/localization_check`；
  控制台显示任务进度/尝试/回执；关 UI 不影响 BT tick。

## 3. B 组读数

- **B1 接口**：BT 端口 `goal_policy/cancel/cancel_ack/localization_ok/mission_goal` 与输出 `cancel_cmd`；
  local 端口 `cancel/cancel_ack`；三张 BT 图（仿真、P2 场景、真机离线）均 `cli check ok`。
- **B2 换算法正反控**：`test_bt_swap_contract.py` 用同端口 alt 规划器 → 图 check OK 且 BT 实例逐字节不变；
  改 `path` 类型 → `E_GRAPH_*` 拒绝。`port_truth.json` 重采 38 话题（新端点 live 观测；晚启动
  `carrier/status` 沿用 11-12 窗口），`test_port_conformance` 6/6 OK。
- **B3 能力声明**：manifest.v3 `capability` 校验 `cancellation/feedback_modes/timeout_s`；
  `test_manifest_v3` 8/8、`test_manifest_v2` 6/6；BT 声明 `cancellation=cooperative`、
  `feedback_modes=[pose,ack,signal]`。

## 4. C 组读数

- **C1**：见窗口 r2（ok=true，六 checks）。
- **C2**：`tasks/inspection.json` + `apps/inspection_bt.yaml` + `navigation_bt` 契约；
  `lite3_gazebo_sim/test/test_inspection_task.py` 2/2；P2 `make test` 15+2 OK。
- **C3**：控制台新增「取消 3D 任务」→ `/v8/bt/cancel`；BT 任务/尝试/恢复/回执/定位显示；
  `cancelled/cancel_unacked` 作为终态失败；`make test` RC=0，`check_console.sh` ALL PASS。
- **C4**：`sim` 改为 headless（不发布 11313/11345），`sim-viz` 独占；`check_compose_ports.py` PASS；
  两容器并行 Up。

## 5. 发布物与提交

| 仓 | HEAD | 发布物 | sha256 |
| --- | --- | --- | --- |
| P1 | 本目录提交 | 见 `target_workspace` | — |
| P2 | `d394bd1` | `ros1-slam-simulation-0.1.0.tar.gz`（2,414,199 B，7 包） | `4fe3e74e9d12fcf520d523f23ae3fb2334d71841ccbb8f9f82b1adda3f32947f` |
| P3 | `c1848e3` | `ros1-slam-console-0.1.0.tar.gz`（540,343 B） | `3a6e138e68d18b2539c205dfbcb252dd4355398cc63cf6fca6634b8c742a84d6` |

## 6. 回归门

- P1 framework **303 OK（10 skip）**；解耦 9/9；端口 6/6；target_workspace 27/27；
  contracts 8+6；`check_bt` host+XML OK；compose PASS；project standards rc=0。
- P2 15+2；P3 5/17/4/2 + BT 配置 + mission events；`check_console.sh` ALL PASS。

## 7. 判读边界

- 仿真为 declared 几何 + Gazebo 运动学载体；不签四足物理/真机/formal。
- 真机/M20 未触碰；`real_robot` 仍只 lite3（L2）。
- 逐目标 `dwell/arrival` 的完整多目标到达窗口未跑；A4 到位判据由单目标窗口 + 多目标队列/重试/定位门读到，
  多目标端到端列入下个作业窗。
- `sim` 与 `sim-viz` 共用 devel 卷：一次只编一套，运行可并存。
