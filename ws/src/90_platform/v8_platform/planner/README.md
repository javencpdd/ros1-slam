# planner · 支撑图全局规划与 ROS 接口

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[support_planner.py](support_planner.py) 在“支撑面 × XY 栅格”上做有界 A*；面内走八邻域，跨面只走显式 edges，遵守单向性。它生成路线与分段/转层/展开数/长度诊断，不输出底盘命令。

## Python API

```python
route = plan(mapping, start_xyz, goal_xyz, budget=20000, cancel=None,
             min_clearance_m=None, robot_radius_m=0.35, robot_height_m=0.436,
             goal_surface_id=None, start_surface_id=None)
```

数值仅展示调用形状，机器人包络须由调用方给依据。budget 是最大展开数，不是秒数；耗尽报 `E_PLAN_BUDGET_EXCEEDED`，不能记为几何无路。cancel 为轮询谓词，触发报 `E_PLAN_CANCELLED`。输入先过[地图契约](../contracts/README.md)，坏图报 `E_PLAN_MAP_INVALID`。

起终点必须是有限 xyz，z 为支撑高度。可用 surface_id 指定支撑面；未指定时在包含 XY 的面中按 z 消歧。上层交互遇到同 XY 多面可由[选面器](../framework/surface_resolver.py)显式拒绝歧义，不能据底层最近 z 策略代替用户选面。

min_clearance_m 可筛除低顶格和面内连接段；起终点低顶报 `E_PLAN_CLEARANCE`，无安全连接报 `E_PLAN_NO_ROUTE`。未知不能当自由，地图合法性/连通性与规划可达性分别判断。

## 障碍与真实格集

规划器显式启用障碍校验，接受 map v1/v2/v3；v3 的 cells 用于支撑格集。非空 obstacles 要求显式、有限的水平半径和正高度，否则 `E_PLAN_INPUT`。参考点为支撑面，包络由支撑 z 向上挤出；XY 使用包住圆的保守正方形。侧面相切判碰撞，从上方接触实体顶面不算穿入。

起终点包络碰撞报 `E_PLAN_OBSTACLE`，全部连接被挡报 `E_PLAN_NO_ROUTE`。v2 路线对实际起终点到格中心连接、面内线段和显式转层门户逐段核验，并保留精确端点；其精确连接/多门户分支目前限定 schema_version=2，v3 升级不能自动继承该分支的保证。

支撑图的面内高度仍为常数，slope/step 边表达离散转层关系，不是连续坡面或四足接触模型。搜索是离散且有界的，不承诺连续空间全局完备。

## ROS 运行接口

- [ros_node.py](ros_node.py)：参数式地图/start/goal，提供 GetPlan/GetPath 包装及状态；路径与拒绝共用同一核心 planner。
- [topic_node.py](topic_node.py)：JSON String 地图、PoseStamped 起点/目标；Path 输出和 JSON 状态。新输入触发 generation 与取消，旧线程结果不得覆盖新请求；Trigger 可请求取消。
- topic_node 缺省地图 `/v8_support_map`、起终点 `/v8/global/start` 与 `/v8/global/goal`、输出 `/global_path`；实际 [app graph](../framework/fixtures/graphs) 覆盖为 `/v8/3d/global/*`。pose frame 须与 `~frame` 一致。
- robot_radius_m/robot_height_m/min_clearance_m/surface_hint 从私有参数进入核心策略。Path 消息本身不携完整 surface_id/map_version，详细身份仍要看状态/原始路线。

[mesh_support.py](mesh_support.py) 从声明网格提取支撑面，[scene_map.py](scene_map.py) 处理声明场景输入；它们不等于真实雷达建图。传感 PCD 管线见[framework](../framework/README.md)。

## 定向测试与回归

从仓根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/planner/test_support_planner.py
python3 -B ws/src/90_platform/v8_platform/planner/test_obstacles.py
python3 -B ws/src/90_platform/v8_platform/planner/test_ros_node.py
python3 -B ws/src/90_platform/v8_platform/planner/test_topic_node.py
python3 -B ws/src/90_platform/v8_platform/regression/global_obstacles.py \
  --out /tmp/global-obstacles.json
```

global_obstacles 使用 SC02/SC05/SC06 的无遮挡/改线/封路派生请求，原场景真值和冻结判据不改。`regression/a4_runtime.py clearance` 需要已有隔离 ROS master；SC09 有安全绕路与“目标在低顶下”的派生请求须分别报数。全局路径→局部→MuJoCo 的历史结果属于声明图/单策略实验，且部分路径只预计算一次，不能称在线全局重规划或真机验收。当前状态统一查[进度](../../../../../note/进度/v8/README.md)。
