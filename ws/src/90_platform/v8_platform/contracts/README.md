# v8 契约（V8-02 首批切片，R641 v1.1 增补）

> 2026-10-25 起，R627 初版，R628／R630／R631 按三轮独立复核修订；R641 按真人勾选做 v1.1 增补（可选 source／adapter_entry／param_schema 字段＋capability.kinematics 增 not_applicable，向后兼容，A1 冻结面以 v1.1 增量记档）。本目录是**设计契约与可执行样例**，不是已接入组件，也不代表任何后端通过验收。
> 上游语义见 [22 组件契约与组合架构](../22_组件契约与组合架构.md)、[24 验收](../24_仿真验收与迁移方案.md)。

## 1. 交付内容与边界

| 件 | 作用 | 边界 |
| --- | --- | --- |
| [schema/manifest.v1.json](schema/manifest.v1.json) | 组件清单：身份、角色、I/O、能力、所有权、证据 | 只描述单组件；跨组件冲突判定归 V8-05 |
| [schema/robot.v1.json](schema/robot.v1.json) | 机器人配置：参考点、包络、运动学、能力与制动 | 数值须带来源与证据级别；未验证值不得用于组合 |
| [schema/map.v1.json](schema/map.v1.json) | 地图：二维占据栅格与多层支撑面 | 只定义携带语义，不含建图算法；高程算法归 V8-08 |
| [schema/task.v1.json](schema/task.v1.json) | 导航任务：起终点、支撑面消歧、预算与时限 | 只定义请求语义；取消与结果类型归 V8-06 |
| [validate.py](validate.py) | mini JSON-Schema 子集＋跨字段语义＋任务/地图配对检查 | 子集外写法直接报错，不做「部分校验当通过」 |
| [fixtures/](fixtures) | 9 正例、52 反例及逐例理由码 | 全部为设计样例，不含实测地图或传感器数据 |
| [test_contracts.py](test_contracts.py) | 61 例逐例断言，加 5 项反控（单向边合法、双环拒绝、配对、校验器自身） | 证明表达与拒绝语义，不证明运行能力 |

## 2. 二维与多层的表达差异

| 语义 | 二维（kind=occupancy_grid） | 多层（kind=support_map） |
| --- | --- | --- |
| 主体 | layers 恰好一层 occupancy | surfaces 至少两个支撑面，禁 layers |
| 同 XY 多层 | multiple_surfaces_per_xy 必须 false | 必须 true，且**至少两面 XY 足迹相交**（E_MAP_ML_NO_OVERLAP） |
| 连通性 | 由栅格通行性表达 | 只能由 edges 表达；从首面做可达性闭包，全部面必须可达 |
| 高度 | goal_reference=plane_xy，目标 z 必须为 null | support_surface，目标 z 必填 |
| 未知格 | 只能 unknown_not_traversable | 同上，禁用 treat_unknown_as_free |

补充规则：surface_id 唯一；足迹必须非退化；边不得自指或指向不存在的面；边两端都须 traversability=supported；flat 边两侧高差不得超过 0.05 m 且不得携带坡度/台阶参数；step 边高差不得超过其 max_step_height_m（无容差，超限即拒）且不得携带坡度参数；slope 边必须给 max_slope_deg 且不得携带台阶参数；bidirectional 参与可达性（单向 A→B 合法，两个互不相连的环非法）。

## 3. 不留空白语义的做法

- 能力数值与制动参数带 value/unit/source/level/composition_use；level=declared 时 composition_use 必须 prohibited（E_ROBOT_UNSOURCED）；level=tested 时必须给 report 路径（E_ROBOT_EVIDENCE）。
- 容差与预算必须带非空 source；字符串 strip 后为空一律 E_SCHEMA_BLANK；数值必须有限，NaN/Inf 报 E_SCHEMA_NONFINITE。
- 预算必须严格小于 harness_deadline_s（E_TASK_BUDGET）。
- 证据级别高于 declared 时 reports 不得为空（E_MANIFEST_EVIDENCE）。
- 地图类型与维度须一致（E_MANIFEST_DIM）；控制角色必须声明停止方式与命令话题（E_MANIFEST_STOP／E_MANIFEST_CONTROL）。
- 起终点 x/y 必须为数值（E_TASK_XY_NULL）；数组长度按 minItems／maxItems 精确约束（机器人参考点 xyz 定长 3、多边形顶点内层定长 2）。
- 足迹参数与类型互斥：circle 不带 points、polygon 不带 radius_m（E_ROBOT_FOOTPRINT）；配对检查对缺字段地图以 E_PAIR_* 理由码失败，不裸抛异常。
- v1.1（R641，真人勾选增补）：manifest 可选 `source`（uri/license 必填、commit 可选）、`adapter_entry`、`param_schema`（非空白字符串）；`capability.kinematics` 增 `not_applicable` 供静态组件（odometry/定位/表征等）诚实落地；旧 manifest 不带新字段仍合法。补遗（R642，同依据）：`io.map_schema` 增 `not_applicable`（无地图组件专用，dimensions 仍 2/3 描述自身空间语义、E_MANIFEST_DIM 配对不变）；`ownership.namespace` pattern 允许裸根 `/`（`^(/|(/[a-z0-9_]+)+)$`）。

## 4. 理由码

| 码 | 含义 |
| --- | --- |
| E_SCHEMA_UNKNOWN_FIELD / TYPE / REQUIRED / ENUM / PATTERN / MINITEMS / MAXITEMS / UNIQUE / MINIMUM / MAXIMUM / BLANK / NONFINITE | 形状、类型、缺失、空串与数值有限性 |
| E_SCHEMA_UNSUPPORTED | schema 或实例用到子集外写法（拒绝静默通过） |
| E_VERSION | schema_version 严格不等（布尔与整数不互通） |
| E_MANIFEST_EVIDENCE / DIM / STOP / CONTROL | 证据矛盾、维度与地图类型矛盾、控制职责缺停止或话题 |
| E_OWNERSHIP_TF_DUP | 重复或自指 TF 边 |
| E_ROBOT_UNSOURCED / EVIDENCE / FOOTPRINT | 未验证值被允许组合、tested 无报告、包络缺参数或携带与类型矛盾的参数 |
| E_MAP_2D_SURFACES / ML_COLLAPSE / ML_COUNT / ML_NO_OVERLAP / ML_SAME_Z / FOOTPRINT_DEGENERATE | 二维带面、多层压平、面数不足、无同 XY 重叠、重叠面同高、足迹零面积或反向 |
| E_MAP_FLOOR_ID / UNKNOWN_AS_FREE / CELL_STATES / LAYERS / LAYERS_MISSING / SURFACES_MISSING | 楼层号代替连通性、未知当自由、状态集或层数不合、按类型缺主体 |
| E_MAP_SURFACE_DUP / EDGE_TARGET / EDGE_PARAM / EDGE_TRAVERSABILITY / EDGE_Z / DISCONNECTED | 面重名、边非法、缺或多带参数（与 kind 互斥）、边任一端不可通行、高差错配（step 不得超声明上限）、不可达 |
| E_TASK_MISSING_Z / PLANE_Z / XY_NULL / SURFACE_AMBIGUOUS / DISAMBIGUATION / BUDGET | 多层缺 z、平面目标带 z、x/y 为空、消歧不足或误用、预算不小于时限 |
| E_PAIR_MAP_ID / PAIR_MAP_VERSION / PAIR_SURFACE / PAIR_MAP_KIND / PAIR_FRAME | 任务与地图不一致：id、版本、支撑面、地图类型、坐标系 |

## 5. 代码分层

1. 契约层（本目录）：只依赖标准库，不 import ROS、不读写运行目录。
2. 领域层（V8-04/05）：注册表、组合求解、锁文件；只消费契约层。
3. 适配层（V8-06/07）：ROS 适配器、进程与话题所有权。
4. 编排与评价（V8-06/11）：启动顺序、健康、独立评价器；不得反向依赖适配层内部实现。

## 6. 旧规则差异（v7/v6 → v8）

| 旧做法 | v8 要求 | 理由 |
| --- | --- | --- |
| 用 nav_msgs/Path 当全局路线 | support_route/v1 携带 surface_id 与地图版本 | 只交 Path 丢楼层、参考点与有效范围 |
| 同 XY 取最高点当三维地图 | support_map 保留多面并要求显式边与足迹重叠 | 压平后无法区分上下层与连通性 |
| 未知格按自由处理 | 只能 unknown_not_traversable | 未知变自由会制造假可通行 |
| 目标只给 x/y，取最近层 | 必须给 z 与消歧策略 | 猜最近层会到达错误楼层 |
| 用固定睡眠等就绪 | 就绪须由可观测条件建立（V8-06） | 固定等待不是证据 |
| 多节点直发最终 cmd_vel | 只发带来源的建议，由仲裁输出唯一命令 | 控制权冲突无法归因 |
| 规划预算可大于测试等待时限 | 启动前拒绝该配置 | 否则把未收齐结果误判为算法失败 |

## 7. 运行方式

```bash
python3 -B validate.py map fixtures/map_ml_two_surfaces.json     # rc0 合规；rc5 有理由码；rc9 用法/前置
python3 -B test_contracts.py                                     # 61 例＋5 项反控
```

## 8. 本批未覆盖

GlobalRoute／LocalTrajectory／MotionCommand／PoseEstimate／SceneProfile 的消息级契约、后端替换与兼容报告归 V8-04/05/06/07/09/10/11；本目录不预写注册表或求解器代码，A1 的正式结论须独立复核后另记。
