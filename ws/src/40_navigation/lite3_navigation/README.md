# lite3_navigation · MBF 平面导航装配

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包装配 MBF 的规划器、控制器、代价图与恢复参数，不承载算法 C++ 源码。

| 入口 | 实际职责 |
| --- | --- |
| `launch/mbf_navigation.launch` | 启动 `mbf_costmap_nav`，装入 MBF/TEB/全局规划/代价图基础配置，发布 M4 心跳；控制器 `cmd_vel` remap 为 `cmd_vel_raw` |
| `config/mbf_navigation.yaml`、`planner_teb.yaml` | 规划器/控制器选择与 TEB 参数 |
| `config/costmap_common.yaml`、`global_costmap_params.yaml`、`local_costmap_params.yaml` | 当前代价图基础配置 |
| `config/recovery_policy.yaml` | MBF outcome、处置与恢复预算；供任务层读取 |
| `verify/dwa_recheck.py`、`v_dwa_recheck.sh`、`v_dwr_exitclean_selftest.sh` | DWA 复核入口 |

`config/variants/{night,outdoor_open,crowd}.yaml` 在本包主 launch 中没有加载行，不能因文件存在
声称对应运行配置已生效。`launch/cmd_vel_raw_relay.launch` 的 relay 节点仍整体注释，
是待配置占位；当前主链直接使用 `mbf_navigation.launch` 的 remap，不依赖它转发。

速度必须经 M6 mux 输出 `/cmd_vel`；本包输出是 `/cmd_vel_raw`。
二维代价图中的高程障碍标记不等于三维全局/局部规划，v8 3D 栈另见 v8_platform。
