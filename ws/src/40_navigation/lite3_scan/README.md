# lite3_scan · SCAN 装配与到达观测

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包提供 SCAN 上游五包的自有 launch、参数外化、消息适配与验证入口；
`ws/src/30_3rd/` 中上游源码保持基线，当前包无算法编译目标。

| 入口 | 实际职责 |
| --- | --- |
| `launch/70_scan.launch` | include 上游 `plan_manage/launch/advanced_param.xml` 并装配自有参数/到达观测 |
| `launch/lio_relay_sim.launch` | 仿真 LIO/TF 中继 |
| `scripts/scan_arrival_monitor.py` | 我方请求目标的 J1 到达与距离观测，不为上游 FSM 合成失败事件 |
| `verify/v_scan_p1.sh`…`v_scan_p4.sh`、`v_scan_p3_triple.sh` | 占据/目标规划/跟踪/重规划与三次运行验证入口 |
| `verify/v_scan_launch_expand.sh`、`v_scan_relay_equiv.sh`、`v_tf_edge_conflict.sh`、`v_scan_o1.sh` | launch、基线、TF 和到达接口合同 |

调整 SCAN 行为应使用本包参数、relay 与编排，不修改上游五包。
命令进入 M6 的 `scan_planner` 源，保持 `/cmd_vel` 唯一仲裁输出。
引用 P4 等历史结果须同时说明载体代际与实际构建件；文件数量或目录在位不是运行验收。
