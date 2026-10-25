# v8 适配器契约面（V8-07 首切片，R647）

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

## 4. 运行方式

```bash
python3 -B io_match.py wiring ../registry/data/manifests --select=v8.controller.dwa_adapter,...  # rc0 报告
python3 -B test_io_match.py   # 6 项
```
