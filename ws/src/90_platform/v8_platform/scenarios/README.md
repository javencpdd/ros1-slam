# scenarios · SC01—SC14 与独立评价

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[scenarios.py](scenarios.py) 的 `SCENARIOS` 已为 SC01—SC14 提供地图构造器、起终点、真值预期和 kind。`scenario(name)` 返回地图、task、truth 与 planner_budget；SC04—SC14 已不是空槽位。

| 场景 | 定义 | kind |
| --- | --- | --- |
| SC01 / SC02 / SC03 | 平地直达 / 坡道爬升 / 双层转层回程 | planning |
| SC04 / SC05 / SC06 | 短转层 / 同高天桥 / 同 XY 双层列 | planning |
| SC07 / SC08 | 断桥反向无路 / 单向边正向可达 | planning |
| SC09 / SC10 | 低顶净空 / 预算耗尽 | planning |
| SC11 | 跟踪偏差撤销 | tracking |
| SC12 | 多任务隔离 | orchestration |
| SC13 | 地图升版与旧版拒判 | versioning |
| SC14 | planning 子矩阵汇总 | matrix |

运行矩阵应按 kind 分派，不能把 tracking、orchestration、versioning 或 matrix 场景都当普通规划到达测试。

`evaluate(scenario_def, trajectory, use_truth_map=None)` 只消费场景真值与被测输出：调用独立轨迹检查、重算必须途经面与终点偏差；不修复输出，不向规划器回流真值。SC09 的有限低顶存在绕路；派生的“目标放在低顶下”请求须与原场景分别记录。SC07 无路、SC10 预算耗尽也须保留各自拒绝原因。

从仓根运行：

```bash
python3 -B ws/src/90_platform/v8_platform/scenarios/test_scenarios.py
```

测试核对全场景构造、kind、规划正例与悬空/错终点负控。场景使用声明地图；本目录不等同真实感知、Gazebo 或四足接触场景库，运行证据和验收见[进度入口](../../../../../note/进度/v8/README.md)。
