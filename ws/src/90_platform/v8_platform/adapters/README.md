# v8 适配器契约面 + A6 恢复（V8-07 首切片 R647；A6 R1071—R1106）

> 2026-10-26 起。M2 首件首切片：**消息级 io 匹配**（V8-05 流程3 留白闭合）＋外部输入白名单＋
> 接线报告（composer 结果 adapters 槽位的前置）＋后端交换证据（A6 面设计层）。
> 消费注册表与组合层；不 import ROS。仿真运行面按 25 号窗纪律另批。

## 1. 交付内容

| 件 | 作用 | 边界 |
| --- | --- | --- |
| [io_match.py](io_match.py) | `msg_type`（『类型（注释）』解析）、`is_external`（传感器 sensor_msgs/*、任务级 /initial_path 等、平台心跳 liveness/health、静态地图文件——显式枚举不静默吞缺口）、`wiring(store, ids)`（每个非外部输入找到生产者，缺源＝E_ADAPTER_NO_SOURCE fail-closed；报告含 providers/external）；CLI `io_match.py wiring DIR --select=ids` rc 0/5/9 | 接线＝可行接线报告（谁可供谁），具体订阅关系归运行装配 |
| [test_io_match.py](test_io_match.py) | 6 项：解析、八件套真实组合全闭合（dwa 全局路径←GlobalPlanner、scan 参考路径←GlobalPlanner、scan 点云←faster_lio 配准云、body_pose←faster_lio）、缺全局规划器负例（nav_msgs/Path 无源）、**hdl⇄lite3 换后端接线同形＋锁面可区分**、CLI 三态 | — |

## 2. 伴随修正（消费暴露的三处不精，均有据）

1. **新入册 v8.global_planner.apt**（第 13 份 manifest＋provenance 第 13 条）：apt GlobalPlanner（NavFn 纯 2D、实栈 /planners=[GlobalPlanner] 历史证据）——六件套缺全局路径生产者由消息级匹配暴露，入册后闭合。
2. **io 词汇修正**（3 份 manifest）：costmap 层输出改 `costmap_2d/Costmap（层插件…聚合进 costmap 服务）`、dwa/teb 局部代价图输入同型——原字符串非消息型，匹配层无法消费。
3. **composer 运动链语义修正**：MOTION_ROLES 收窄为 {controller, local_planner}——global_planner 产出地图级路径不执行运动，kinematics=not_applicable 合法（apt 全局规划器入册暴露）。17 项测试复跑无回退。
4. **teb kinematics 定案边界**（R642 挂账处置）：宿主 ws 无 teb/dwa 配置件（R616 十六件引用在容器面），differential 声明的佐证＝历史证据「TEB 2D 刚体假设」与实栈无横向速度迹象；**定案须容器面读配置，留运行批**（manifest 注记已更新）。

## 3. 后端交换证据（A6 面「同任务接口换后端」设计层）

- 定位面 hdl⇄lite3、控制器面 dwa⇄teb：**接线结构同形**（组件名归一后一致）＋**锁摘要可区分**——同任务接口换后端在组合层成立；运行面等价性归仿真（V8-13）。

## 4. A6 恢复与 NDT 重定位（R1071—R1115）

| 件 | 作用 |
| --- | --- |
| `localization_state.py` | 时效/跳变/稳定样本/lease generation 门；失效或 relocalize 撤销当前租约；旧轨迹不复活；CLI 可做 `cmd_vel` 拒绝门 |
| `recovery_supervisor.py` | TRACKING→RELOCALIZING→候选→RECOVERING→新 lease→TRACKING；启动 bootstrap 只在 see observation 且有 lease 后收候选；deadline FAILED |
| `a6_recovery_node.py` | ROS 包装：relocalize 服务、candidate JSON、backend/lease/status/command_allowed；支持 Empty/Trigger 服务与 backend 名 |
| `ndt_recovery_bridge.py` | NDT `~relocalized` 事件→candidate；`score=1/(1+fitness/fitness_max)`；事件后 freshness 重试，不丢已认证位姿 |
| `a6_native_recovery_live_smoke.py` / `a6_ndt_recovery_live_smoke.py` | Gazebo BBS/NDT 活体恢复；GT 只做后验门；空扫描 fail-closed 负控 |
| `candidate_consensus.py`、`tracking_monitor.py`、`declarations.py`、`io_match.py` | 候选/跟踪/损失声明/消息级接线 |
| `consensus_policy.py` / `consensus_gate.py` | NDT 候选与 BBS 绝对源共识在 0.5 m 内才放行并输出共识位姿；无/过期/不一致 fail-closed |

- R1106 静态失锁最终 XY **.052642 m**、候选 **.057400 m**，lease 1→2；失锁期 gate `.1→0`；空扫描 FAILED、lease 2→2。运动候选停在 prior `.247788 m`，不作认证。
- R1114：`/lease` 改为 latched，修掉 gate 晚订阅错过初始 lease 的启动竞态；静态复跑 **.048655 / .044988 m**，0.3 m 局部劫持两跑终态 **.043754 / .060690 m**，候选仍停在 prior（**.242—.250 m**），空扫描负控 FAILED、lease 2→2。0.6 m 探针（超出 0.4 m 搜索半径）候选/终态偏 **.5421/.5225 m**，A6 仍 `TRACKING`、gate 报 `HEALTHY`——系统级 fail-open 已登记，未修。
- R1115：加 BBS 三分辨率绝对源共识门，NDT 候选与共识差 >0.5 m 即拒绝；A6 可要求 `source=bbs_consensus`、`agreement<=0.5`。活体：静态 **.057751 m**（共识 [1.5,0]）、0.3 m **.053147 m**（共识 [1.9,0]）；0.6 m 共识 [2.2,0] 与 NDT 候选差 **.647609** → A6 FAILED、lease 1、无 reset。堵的是 R1114 的 fail-open；BBS 非真实第二传感器，生产图需显式接入。
- 测试：A6/共识定向 **30/30**；`test_localization_state` 6、`test_recovery_supervisor` 9、`test_ndt_recovery_bridge` 3、`test_consensus_gate` 4。
- 边界：NDT 仍 monitor-only；四足/真机/长时 kidnapped 未认证；A6 仍 PARTIAL。

## 5. 运行方式

```bash
python3 -B io_match.py wiring ../registry/data/manifests --select=v8.controller.dwa_adapter,...  # rc0 报告
python3 -B test_io_match.py   # 6 项
```
