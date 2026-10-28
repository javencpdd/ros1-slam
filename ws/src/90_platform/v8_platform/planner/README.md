# v8 支撑图全局规划（V8-09，R652：有界 A* 设计层）

> 2026-10-26 起。M3 次件：有界 A* 支撑图全局规划设计层——面内通行＋**只走显式边转层**
> ＋预算/取消/路径诊断（分段＝V8-03「切单调段」主路线的规划层供给）。消费 map.v1
> （representation 生产者构造）；不 import ROS。运行面与 A4 正式判据归仿真矩阵。

## 1. 交付内容

| 件 | 作用 | A4 落点 |
| --- | --- | --- |
| [support_planner.py](support_planner.py) | `plan(map, start, goal, budget, cancel)`：图模型＝(surface, 格) 节点、面内 8 邻（supported 面）、**跨面只走显式 edges**（bidirectional=false 单向、桥＝足迹最近格对）；预算＝最大展开数（超限 E_PLAN_BUDGET_EXCEEDED，拒绝≠无路）；取消谓词（E_PLAN_CANCELLED）；地图先过契约（E_PLAN_MAP_INVALID）；目标消歧＝同 XY 多层取最近 z（无面＝E_PLAN_GOAL_INVALID）；诊断＝路径/分段/转层/展开数/长度 | 正例有支撑且连续安全（逐点落面断言）；负例各有理由码 |
| [test_support_planner.py](test_support_planner.py) | 8 项：正例（地面→坡→夹层，分段与转层、逐点落面）、同面短径、目标无面、**单向边无路**（map 合法但 mezz 回不去）、预算耗尽、取消、非法图拒、**语义发现断言** | — |

## 2. 语义发现（R652 登记）

**map.v1 结构性容不下 unsupported 面**：单连通（全部面从首面可达）＋「不可通行面不得持边」（E_MAP_EDGE_TRAVERSABILITY 双向禁）⇒ unsupported 面在任何合法 support_map 中不可达 ⇒ 不可能成图——`traversability` 枚举的 `unsupported` 值在 map 层为**死值**（测试以断言锁定）。候选处置（后续裁定）：map.v1 语义修订草案或把 unsupported 语义移到证据层/规划层——本批只登记不改已签契约。

## 3. 对抗面（代行复验）

z 消歧取最近面、同输入确定性（逐字段一致）、夹层内直达不转层、充足预算不误拒、**1600 节点大图可解**（exp=250 有界）——全 PASS；两次实现缺陷留痕（_result 长度计算误用 surface_id 当格坐标、断连通用例又踩双向边陷阱）。

## 4. 运行方式

```bash
python3 -B test_support_planner.py   # 8 项
```

## 5. 显式净空策略与运行负例

`plan(..., min_clearance_m=...)` 会在展开前筛掉净空不足的格，并检查面内连接线段，
避免细低顶落在格中心之间时被跨过。起点或目标净空不足返回 `E_PLAN_CLEARANCE`；
安全图上无路仍返回 `E_PLAN_NO_ROUTE`。未提供策略时保留原有调用语义。
ROS 节点通过私有参数 `~min_clearance_m` 接收该值，状态的 `policy` 字段回显。

`regression/a4_runtime.py clearance` 从冻结定标读取净空值，不改场景定义：
官方 SC09 低顶有限且存在安全绕路，应产出检查器可接受的路径；派生请求只把目标
放到同一地图的低顶下，应由规划器直接拒绝。两者分别记录，不把评价器拒绝计为
规划器拒绝，也不把派生请求称为官方 SC09 无路。

```bash
PYTHONPATH=../regression:$PYTHONPATH python3 -B -m a4_runtime clearance --out /tmp/a4-clearance.json
```

命令需要已经启动的 ROS master。所得证据覆盖规划服务接口，未包含底盘执行。

## 6. 障碍体积输入

全局规划器同时接收 `map/v1` 与显式声明 `obstacles` 的 `map/v2`。
`build_support_map(..., obstacles=[...])` 生产 v2；不传该参数时保留 v1。
输入形状与消费者版本边界见 [契约说明](../contracts/README.md#9-显式障碍体积mapv2)。

```python
route = plan(mapping, start, goal, robot_radius_m=0.5, robot_height_m=0.5)
```

非空障碍要求显式给出有限的水平半径和正高度，否则返回 `E_PLAN_INPUT`。
这里的数字只是调用示例，须由调用方按实际几何给出，不是 Lite3 能力定标。
参考点为支撑高度；检查支撑 z 到 z+height 的竖直包络，XY 使用包住圆足迹的
保守正方形。侧面相切判碰撞，站在实体顶面时仅接触顶面不算穿入。

起终点包络碰撞返回 `E_PLAN_OBSTACLE`；无安全栅格连接或全部路线被挡返回
`E_PLAN_NO_ROUTE`。格中心、连续面内边、对角边、显式转层边、实际起终点到格中心
的连接均核验。转层保留原支撑几何的所有等距最近门户；障碍不能把门户挪到更远处
制造新桥。v2 返回精确起终点连接并保留浮点精度，避免输出舍入后穿箱。
搜索仍是离散有界图，不承诺连续空间全局完备或四足跨阶能力。

ROS 的初始发布、GetPlan 与 GetPath 共用 `~robot_radius_m`、`~robot_height_m`；
拒绝时路径为空，状态回显障碍版本、数量和包络参数。

```bash
python3 -B test_obstacles.py
python3 -B ../regression/global_obstacles.py --out /tmp/global-obstacles.json
```

回归为 SC02／SC05／SC06 的派生请求，各含无遮挡、箱体改线、封路三例。
SC02／SC06 的封路体贯穿地面与夹层，因只放阶面小箱可从下层绕到箱后；
SC06 改线可合法使用 `ground→mezz→ground`。原场景真值与冻结判据不改。
独立评价器采用六面相交检查，旧无障碍路线须在新箱体场景下被判碰撞，防止假改线。
