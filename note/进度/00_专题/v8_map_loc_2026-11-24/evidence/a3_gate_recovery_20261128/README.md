# A3 定位闸门关断/恢复与 GT 正常（2026-11-28）

## 结论

在 M2 隔离场景、9/9 节点、单目标已受理的同一任务链上：

| 阶段 | local | BT | GT | cmd |
|---|---|---|---|---|
| 关闸前 | `E_LOCAL_SUPPORT`（z=-0.0489） | `localization_ok=true`，`goal_seq=1/current_seq=1` | z=0.1100 | 0 |
| 关闸 3 s | `stop=true, E_LOCAL_POSE_STALE, stop_latched=true` | `event=localization_lost, localization_ok=false`，队列 0 | z=0.1100 仍正常 | 0 |
| 恢复 4 s | `stop=true, E_LOCAL_LOCALIZATION_LOST, stop_latched=true` | `event=localization_ok, localization_ok=true`，`goal_seq=1/current_seq=1`，队列 0 | — | 0 |

**判定：**

- EST 失效但 GT 正常：符合预期。闸门关断后 loc/局部规划立即判失效，GT 话题仍发 z=0.1100。
- 恢复不自动续跑：符合预期。loc 心跳恢复为 true，但 local 保持 `stop_latched` 的 `E_LOCAL_LOCALIZATION_LOST`，BT 没有新目标、没有新路径、`/cmd_vel=0`。
- 本项是在起点支撑阻断（A5 未闭合）的任务上做的，**没有非零速度**；它证明的是“失效锁存与恢复不自动续跑”，不等于完整运动任务的重放验收。

## 文件

- `goal_once.json`：目标受理凭据（`accepted=true`）
- `before_status.yaml` / `before_bt.yaml`：关闸前
- `gate_off.log`：服务返回
- `off_status.yaml` / `off_bt.yaml` / `off_gt.yaml` / `off_cmd.yaml`：关闸后
- `gate_on.log`：恢复服务返回
- `recovered_status.yaml` / `recovered_bt.yaml` / `recovered_cmd.yaml` / `recovered_loc_ok.yaml`：恢复后
- `nav_graph/`：本轮图、plan、readiness、log

## 复跑

```bash
# 底座：M2 + GT 隔离（见 m2_t3_contact_20261128/README.md）
# 图：graph_3d_m2_lio_bt.yaml，NO_GOAL 模式起 9 节点
# 发一条目标后：
rosservice call /v8/3d/lio_gate/lio_gate/set_enabled "{data: false}"
# 3 s 后采样 /v8/local/status、/v8/bt/status、/sim/odom_gt、/cmd_vel
rosservice call /v8/3d/lio_gate/lio_gate/set_enabled "{data: true}"
# 4 s 后再采样
```

## 未闭合

- GT TF 独立关断、GT 话题独立关断：仍缺“raw GT + eval relay”或插件开关；本切片未做。
- FAR/感知运行时不消费 GT：仍只有静态装配/源码证明，需 FAR 全图窗口做 `rosnode info` 与 TF 来源归因。
- 恢复不自动续跑尚未在“已有非零速度”的任务上复测。
