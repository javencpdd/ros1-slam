# lite3_navigation · 成熟算法执行后端

> 最新更新时间：2026-11-21 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包为 BT 编排提供 `move_base` 单目标执行后端，复用 GlobalPlanner、TEB 和现有高程/STVL/SOB 代价图，不承载算法 C++ 源码。后端自动恢复关闭，任务顺序与恢复归 `lite3_bt`；旧 MBF 入口只留作历史对照。

| 入口 | 实际职责 |
| --- | --- |
| `launch/navigation_executor.launch` | 当前后端：move_base + 原规划器/代价图；候选速度送 action/lease 守卫，再经安全 mux；M4 心跳不等价于就绪证明 |
| `launch/mbf_navigation.launch` | 历史 MBF 对照入口；当前 bringup 主入口已不再 include |
| `config/mbf_navigation.yaml`、`planner_teb.yaml` | 规划器/控制器选择与 TEB 参数 |
| `config/costmap_common.yaml`、`global_costmap_params.yaml`、`local_costmap_params.yaml` | 当前代价图基础配置 |
| `config/recovery_policy.yaml` | MBF outcome、处置与恢复预算；供任务层读取 |
| `verify/dwa_recheck.py`、`v_dwa_recheck.sh`、`v_dwr_exitclean_selftest.sh` | DWA 复核入口 |

`config/variants/{night,outdoor_open,crowd}.yaml` 在本包主 launch 中没有加载行，不能因文件存在
声称对应运行配置已生效。`launch/cmd_vel_raw_relay.launch` 的 relay 节点仍整体注释，
是历史占位；新主链使用 `navigation_executor.launch` 和 BT action 守卫，不依赖此 relay。

本包只输出 `/v8/navigation/cmd_vel_raw_candidate`，经 BT action/租约守卫到 `/cmd_vel_raw`，再由 M6 mux 输出 `/cmd_vel`。TEB 的 MBF 接口包依赖仍保留，不将“无 MBF 服务器”写成“无 MBF 依赖”。
二维代价图中的高程障碍标记不等于三维全局/局部规划，v8 3D 栈另见 v8_platform。
