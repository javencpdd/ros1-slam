# contracts · 组件、地图、机器人与任务契约

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录提供 JSON schema、标准库校验器和正反例。校验通过表示输入满足契约，不表示后端已运行或硬件能力已验证。设计依据见[组件契约与组合架构](../../../../../note/计划书v8/22_组件契约与组合架构.md)。

## 契约入口

| 文件 | 内容与消费者边界 |
| --- | --- |
| [manifest.v1.json](schema/manifest.v1.json) | 单组件身份、角色、I/O、能力、所有权、证据；兼容可选 source/adapter_entry/param_schema 与 not_applicable 运动学 |
| [manifest.v2.json](schema/manifest.v2.json) | 命名 ports、dimensions 与 launch；`normalize_manifest` 为旧注册表/组合器派生 io；[framework](../framework/README.md) 消费命名端口 |
| [robot.v1.json](schema/robot.v1.json) | 参考点、包络、运动学、能力、制动；数值来源与证据级别 |
| [map.v1.json](schema/map.v1.json) | 单层 occupancy_grid 或单/多层 support_map |
| [map.v2.json](schema/map.v2.json) | support_map 与显式 `obstacles` 三维盒 |
| [map.v3.json](schema/map.v3.json) | 支撑面可携带 RLE `cells` 真实格集，并可带障碍 |
| [task.v1.json](schema/task.v1.json) | 起终点、支撑面消歧、预算与时限；`check_pair` 校验地图身份/版本/坐标系 |
| [map_delta.v1.json](schema/map_delta.v1.json) | 基于指定 base_version 的 upsert/remove 操作 |
| [validate.py](validate.py) / [fixtures](fixtures) | mini JSON-Schema 子集、跨字段语义和正反例；不支持的 schema 写法显式拒绝 |

## 地图与任务语义

- occupancy_grid 恰好一层 occupancy，禁 surfaces，`multiple_surfaces_per_xy=false`。
- support_map 使用 surfaces 和显式 edges。单面必须声明 `multiple_surfaces_per_xy=false`；多面声明为 true 时至少两面且至少一对 XY 足迹重叠。不能把多面图压平。
- surface_id 唯一，足迹非退化。跨面边不能自指或引用幽灵面；边两端须 supported。flat/slope/step 参数按类型互斥；step 高差不能超过声明上限。单向边按方向参与从首面的可达性闭包；不可达分量须拆为不同地图。
- `cell_states` 恰为 free/occupied/unknown；未知不能声明为自由。有支撑面不等于有空闲观测。
- support_surface 目标须有 z 和消歧策略；explicit_hint 须带 surface_hint；plane_xy 目标 z 为 null 且不带 hint。起点 z 必须明确。
- 任务预算须小于 harness deadline，容差/预算须有非空来源；布尔不能冒充数值，NaN/Inf、空串与未知字段按 schema 拒绝。

`map.v3` 的 cells 格号与 `representation/support_map.py` 配套；支持 cells 的消费者按真实格集判归属，旧无格集图回退足迹。不能仅凭 schema 版本宣称全部 ROS 节点已支持每项语义。

## 显式障碍与版本选择

障碍是地图 frame 内、以米计的轴对齐实体盒：

```json
{"obstacle_id":"deck_box","min":[1.0,-0.2,0.06],"max":[1.4,0.2,0.5]}
```

三轴须 min < max、ID 唯一。空 obstacles 只说明无声明体积，不产生空闲观测；内容变化须更新 map_version。

Python `validate('map', value)` 缺省拒绝 v2；v3 若携带 obstacles 也须显式 `allow_obstacles=True`，v3 无 obstacles 可走缺省入口。CLI map 校验会显式开启该能力。当前全局规划器、原生局部规划器与轨迹检查器均显式开启障碍校验；MapService 等其他消费者须逐入口核对，不能沿用“旧局部与监督器一律拒 v2”的旧结论。

## 能力与所有权

robot 能力/制动值为 declared 时必须 `composition_use=prohibited`；tested 值须带报告。manifest 证据高于 declared 时 reports 不得为空。控制器必须声明停止方法和命令话题；跨组件 TF、命令冲突由 [composer](../composer/README.md) 与 [framework](../framework/README.md) 检查。

常见理由码：`E_SCHEMA_*`（形状/字段/有限性）、`E_VERSION`、`E_MANIFEST_*`、`E_ROBOT_*`、`E_MAP_*`、`E_TASK_*`、`E_PAIR_*`。精确定义以 `validate.py` 和对应负例为准。

## 使用与定向测试

从仓根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/contracts/validate.py map \
  ws/src/90_platform/v8_platform/contracts/fixtures/map_ml_two_surfaces.json
python3 -B ws/src/90_platform/v8_platform/contracts/test_contracts.py
python3 -B ws/src/90_platform/v8_platform/contracts/test_manifest_v2.py
```

CLI：rc 0 合规、rc 5 拒绝并给理由码、rc 9 用法或前置错误。样例与单测只验证表达和拒绝语义，运行与正式验收另查[进度](../../../../../note/进度/v8/README.md)。
