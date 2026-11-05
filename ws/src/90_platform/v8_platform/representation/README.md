# representation · 支撑图、证据、版本与增量

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录提供支撑图数据结构和建图后处理，以及地图服务、ROS 适配和语义桥。核心算法与 ROS 节点已分文件实现，不能把整个目录仍描述为“运行时尚未接入”。

## 支撑与观测

| 实现 | 当前职责 |
| --- | --- |
| [support_map.py](support_map.py) | HeightColumn 保留同 XY 的多高度；EvidenceGrid 区分 free/occupied/unknown；build_support_map 校验出图；suggest_edges 只给咨询候选 |
| [elevation.py](elevation.py) | 点云格桶、z 聚类、连通成面与小簇过滤；提取结果保留真实格集 |
| [map_service.py](map_service.py) | MapService 版本/内容身份、证据、查询、原子增量、快照；MapStore 多图文件与锁 |
| [map_cache.py](map_cache.py) | 按 surface_id 聚合操作、生成/合并 map_delta；ack 推进 base |
| [map_bundle.py](map_bundle.py) / [map_router.py](map_router.py) | 带校验的地图交付包与多图路由 |
| [semantic_map.py](semantic_map.py) | semantic_map/v1、语义 catalog；support/obstacle bridge 转入相应规划输入 |
| [occupancy_adapter.py](occupancy_adapter.py) / [open_data_import.py](open_data_import.py) | PGM/占据栅格适配与公开地图、world、数据集导入/来源校验 |

支撑面表示可接触几何；free 必须来自正向观测，缺证为 unknown。开顶净空查询可为 None，不能冒充测得无穷净空。相邻面不会仅因距离近自动连边，必须显式确认 edges。

地图 extent 是支撑足迹并集外包框换算的格数。契约要求从首面能到达全部面；不连通可通行域按分量拆为不同 map_id。多面不能压成“最高点”，同 XY 消歧由[选面器](../framework/surface_resolver.py)处理。

## v1/v2/v3 与服务边界

`build_support_map` 未带 cells/obstacles 时生成 v1；显式 obstacles 生成 v2；面带 cells 时生成 v3。cells 用绝对格号的 RLE 编码，`contains_xy` 有 cells 时按真实格集判断，无 cells 才回退 bbox；`select_support` 在容差内取当前位置下方最高支撑。

障碍版地图需要消费端显式启用 `allow_obstacles=True`，见[契约](../contracts/README.md)。全局/原生局部规划和轨迹检查器已启用；MapService 的 `_validate` 仍走缺省入口，不能据生成器支持 v2/v3 推断该服务也接收带障碍图。不同 ROS 地图节点的协议范围须分别核对。

MapService 以内容 digest、map_id/version 防止回退和无变化升版；propose_delta 按 delta_id 幂等，旧 base、同 id 异内容或非法结果拒绝，成功后原子升版并通知监听者。快照保存地图、证据、history 与 applied_deltas，临时文件/fsync/原子替换；恢复核对内容身份。MapStore 支持显式 acquire/release，文件锁是调用方生命周期责任。

## ROS 查询和更新入口

- [map_node.py](map_node.py)：Trigger 查询；参数 `~map_json`、可选 `~evidence_json`、`~query_points`。每次重新读取并返回 map_id/version/digest 与各点全部层、支撑、净空和观测状态。evidence 格式为 `[{"xyz":[0.25,0.25,0],"state":"free"}]`，未列点为 unknown。
- [delta_map_node.py](delta_map_node.py)：订阅 JSON `map_delta/v1`，发布版本化 `/v8_support_map`、`/v8_map_status`，`~query` 返回快照；`~state_path` 用于重启恢复。
- [map_cache_node.py](map_cache_node.py)、[map_merge_node.py](map_merge_node.py) 与 [map_delta_relay.py](map_delta_relay.py)：typed v8_msgs 的 cache/merge/relay。需先构建 v8_msgs，不能把旧 graph_3d_cache 测试替身当成真实 typed 服务。
- multi_map、semantic_map、semantic_support_bridge、semantic_obstacle_bridge 节点提供相应适配，组件清单和连线见 [framework](../framework/README.md)。

## 建图与定向测试

PCD 入口是[map_from_pcd.py](../framework/map_from_pcd.py)，更完整用法见[framework](../framework/README.md)和[Lite3 配置](../configs/lite3/README.md)。Gazebo 传感 survey 使用 `regression/gazebo3d_mapping.py`；使用 GT 位姿完成配准的地图不能称为独立定位建图证明。

从仓根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/representation/test_support_map.py
python3 -B ws/src/90_platform/v8_platform/representation/test_elevation.py
python3 -B ws/src/90_platform/v8_platform/representation/test_map_service.py
python3 -B ws/src/90_platform/v8_platform/representation/test_map_delta.py
python3 -B ws/src/90_platform/v8_platform/representation/test_map_cache.py
python3 -B ws/src/90_platform/v8_platform/representation/test_semantic_map.py
```

真实 ROS smoke 脚本会创建节点，按[作业单](../../../../../note/计划书v8/25_v8自动调度作业单.md)执行。本文没有重跑历史 survey，不新增 A3 或真机验收结论。
