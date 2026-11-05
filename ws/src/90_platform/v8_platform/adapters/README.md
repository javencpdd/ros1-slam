# adapters · 接线、损失声明与定位恢复

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录兼有不依赖 ROS 的契约/状态机与 ROS 包装，不再整体标为“不 import ROS”。声明级接线和运行恢复是两种证据范围。

## 消息匹配与信息损失

[io_match.py](io_match.py) 的 `wiring(store, ids)` 将每个非外部输入与选中组件输出按消息类型匹配；传感器、任务输入和心跳等外部类型须在白名单中。缺生产者报 `E_ADAPTER_NO_SOURCE`。报告列出可能 providers/external，不会修改真实 ROS 订阅。

[declarations.py](declarations.py) 的 `losses` 给每个适配器列 kind/description/domain/evidence，未登记拒绝；空列表表示显式声明无损。组合样例会执行 compose+wiring+lock。hdl/lite3、DWA/TEB 的声明级接线同形不证明运行精度或运动能力等价。SCAN 抽稀、搜索平面与 2D 控制器的高度损失见[接缝探针](../probe_mode3/README.md)。

从仓根运行一个完整登记组合的接线检查：

```bash
python3 -B ws/src/90_platform/v8_platform/adapters/io_match.py wiring \
  ws/src/90_platform/v8_platform/registry/data/manifests \
  --select=v8.odometry.faster_lio,v8.global_localization.lite3,v8.map_server.apt,v8.representation.costmap_layers,v8.local_planner.scan_planner,v8.global_planner.apt,v8.controller.dwa_adapter,v8.safety_arbiter.lite3
```

CLI rc 0 为报告、rc 5 为拒绝、rc 9 为用法错误。

## 跟踪与租约

| 入口 | 作用 |
| --- | --- |
| [localization_state.py](localization_state.py) | 时间新鲜度、跳变、稳定样本与递增 lease generation；失效/relocalize 撤销租约，旧轨迹不能复活；CLI 可作为命令拒绝门 |
| [recovery_supervisor.py](recovery_supervisor.py) | TRACKING→RELOCALIZING→RECOVERING→新租约；候选、时限与失败状态机；bootstrap 仍需观测与租约 |
| [a6_recovery_node.py](a6_recovery_node.py) | ROS odom/health/candidate/lease 接口、重定位服务调用及 backend/status/command_allowed 输出；lease 发布为 latched |
| [tracking_monitor.py](tracking_monitor.py) | 参考对实际三向偏差、连续拒绝锁存 hold；实际停车仍需接安全链 |
| [follower_node.py](follower_node.py) | 轨迹执行的 ROS 实验适配，须受定位/租约与安全仲裁约束 |

## NDT 候选与绝对源共识

[ndt_recovery_bridge.py](ndt_recovery_bridge.py) 把 NDT `~relocalized` 事件、健康/新鲜 odom 和 fitness 转成 candidate；分数为 `1/(1+fitness/fitness_max)`，不是经定标的定位概率。

[consensus_gate.py](consensus_gate.py) 将原始候选与绝对定位候选比较，距离在 `--radius` 内且两者不过期才转发；转发使用共识源位姿。缺源、过期或不一致不放行。A6 wrapper 可用 `--require-candidate-source bbs_consensus --max-agreement-m 0.5` 要求来源与一致性字段；该实验数值不能擅自当作新冻结门值。

默认接线为 `/a6/recovery/candidate_raw` + `/a6/consensus/candidate` → `/a6/recovery/candidate`，生产 graph 必须显式接入。BBS 共识不是第二个真实传感器。

R1114 记录了超出局部搜索范围仍恢复租约的 NDT-only fail-open；R1115 共识门在该探针中拒绝候选。短时局部失锁结果与限制见[进度](../../../../../note/进度/v8/README.md)及 [NDT 说明](../../v8_ndt_odom/README.md)。NDT 仍 monitor-only，任意 kidnapping、连续导航后端切换和真机没有由本目录获得认证。

## 测试入口

```bash
python3 -B ws/src/90_platform/v8_platform/adapters/test_io_match.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_declarations.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_localization_state.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_recovery_supervisor.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_ndt_recovery_bridge.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_consensus_gate.py
python3 -B ws/src/90_platform/v8_platform/adapters/test_tracking_monitor.py
```

`a6_native_recovery_live_smoke.py` / `a6_ndt_recovery_live_smoke.py` 会操作 Gazebo/ROS 活体，需使用隔离作业窗。此 README 未重新运行这些实验。
