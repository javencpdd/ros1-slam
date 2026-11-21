# lite3_task · SMACH 任务编排

> 最新更新时间：2026-11-06 15:17:09 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包以 `config/task_primitives.yaml` 注册原语，由 `scripts/task_smach_min.py` 装配 SMACH 状态机。
启动入口为 `launch/50_task_smach_min.launch`，支持 `chain`、`navigate`、`scan` 模式。

- `chain`：GET_PATH → EXE_PATH，消费 MBF action 的结果；可按恢复策略重试、重规划或清代价图。
- `navigate`：使用 MBF MoveBase action，outcome 经 `lite3_navigation/config/recovery_policy.yaml` 查表并检查预算。
- `scan`：发布 SCAN 目标，消费 `/lite3/scan/arrived` 与 `/lite3/scan/arrival_dist`；到达成功或等待超时返回。

SCAN 分支没有上游失败事件，不替其制造失败/恢复语义；“失败只有超时”仅指这条分支。
MBF 分支还会对 action server 缺席、空路径、返回的失败 outcome、恢复预算耗尽等返回 `aborted`。

验证入口为 `verify/v_task_smach.sh`、`v_task_scan_primitive.sh` 与 `v_scan_task_live.sh`。
任务状态机不代替独立到达评价器，也不直接绕过 M6 发布 `/cmd_vel`。
