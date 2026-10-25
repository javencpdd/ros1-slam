# v8 轨迹监督（V8-10 首切片，R653）

> 2026-10-26 起。M4 首件首切片：**独立轨迹检查器**设计层——22 号「独立轨迹检查器核验
> 支撑、净空、时间有效性、地图版本」；被拒轨迹不算规划成功（A5 面）。
> **独立性＝对策**：V8-03 实证 SCAN 优化器兜底会掩盖 A* 失败并产出未验证几何轨迹，
> 本检查器不信任任何被检链的自评。消费 map.v1 与 V8-09 规划器输出；不 import ROS。

## 1. 交付内容

| 件 | 作用 | 理由码 |
| --- | --- | --- |
| [trajectory_checker.py](trajectory_checker.py) | `check(map, trajectory, z_tol, min_clearance, max_step, time_budget, expected_map_version)`：逐点支撑（悬空即拒）／净空（HeightColumn 复算、开顶不误报）／连续安全（跨面须显式边·方向敏感；同面步距上限）／时间（单调＋预算窗）／地图版本（消费者旧版不得判）；违例逐条 (code, index, detail) | E_TRAJ_UNSUPPORTED／CLEARANCE／TRANSITION／DISCONTINUITY／TIME／MAP_STALE／MAP_INVALID／EMPTY |
| [test_trajectory_checker.py](test_trajectory_checker.py) | 7 项：规划器输出全链通过、V8-03 悬空场景拒（恰为悬空段）、单向边/瞬移/净空/时间/守卫 | — |

## 2. 对策实证（V8-03 数据闭环）

R633 烟测实测的悬空轨迹点（坡脚 x≈0.29 处 z=0.598，真实坡面体心 0.30）穿检＝**逐点 E_TRAJ_UNSUPPORTED**——「优化器兜底掩盖 A* 失败」产出的那类轨迹，正是本检查器要拦的。

## 3. 未覆盖（V8-10 后续切片）

运动学上限核验（robot.v1 profile 接线，declared 值 composition_use=prohibited 铁则面）；失效撤销（被拒→V8-06 停止链联动）；跟踪面；SCAN 适配或备选裁决（消费 V8-03 四前提）。A5 正式判据归仿真矩阵。

## 4. 运行方式

```bash
python3 -B test_trajectory_checker.py   # 7 项
```
