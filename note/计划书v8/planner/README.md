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
