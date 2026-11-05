# lite3_supervisor · 心跳监管与 hold

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

`launch/90_supervisor.launch` 启动 `scripts/health_monitor.py`，按
`config/health_watch.yaml` 的 mode/watch 表订阅模块 `std_msgs/Empty` 心跳。

监管器对启动宽限和稳态失联使用不同截止时间，失联时发布 latched
`/lite3/supervisor/hold`（`std_msgs/Bool`）让 M6 输出零速，自身不发布 `/cmd_vel`。
当前默认允许连续稳定观察后自动恢复；`auto_recover` 和 `recover_beats` 可按 mode 配置。
这里的 recover beats 是看门狗观察拍数，不是对定位精度的复核。

源码只执行失联/恢复状态与 hold 发布；配置中的 `on_loss` 文字不等于已经实现进程关停。
任务进程生命周期与资源回收需要独立编排器/runner 负责。
参见[工作区结构](../../../README.md)与现行停止链证据；不把单段 hold 证明当作全链停车预算。
