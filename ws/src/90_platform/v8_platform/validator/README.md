# validator · 独立轨迹、运动学与跟踪监督

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录检查被测链输出，不修复轨迹。检查通过只代表对应地图、时间、能力与容差条件满足，不能替代载体接触或真机验收。

| 实现 | 当前接口与范围 |
| --- | --- |
| [trajectory_checker.py](trajectory_checker.py) | `check(map_value, trajectory, z_tol, min_clearance, max_step, time_budget, expected_map_version)`：支撑、净空、显式有向转层、步距/时间、地图版本和障碍碰撞；返回 ok 与逐项 violations |
| [enforcement.py](enforcement.py) | `check_kinematics` 检查 robot profile 台阶/坡度/制动能力；`on_rejection` 生成停止动作；`RejectionTracker` 连续拒绝达到上限时升级停止 |
| [tracking.py](tracking.py) | `check_tracking` 比较参考与实际位置的横向/纵向/垂向偏差；`overrun` 检测参考耗尽后继续执行 |

轨迹检查器显式使用 `allow_obstacles=True`，接受相应 map v1/v2/v3 契约；v3 有 cells 时支撑/净空归属按真实格集，旧图回退足迹。障碍碰撞报 `E_TRAJ_OBSTACLE`。旧版 README 的“跟踪尚未实现”和“监督器默认拒绝全部 v2”已不适用。

轨迹跨面须有合法方向的显式边；未知/悬空、净空不足、旧地图版本、非单调时间或超预算均拒绝。常见码为 `E_TRAJ_UNSUPPORTED/CLEARANCE/TRANSITION/DISCONTINUITY/TIME/MAP_STALE/MAP_INVALID/EMPTY/OBSTACLE`。空闲观测与支撑面声明是不同信息。

运动能力为 declared+prohibited 时拒绝以它判定可执行（`E_TRAJ_CAPABILITY_UNVERIFIED`）；tested 能力仍须报告。调用方负责把否决连接到实际停止链，单独调用检查函数不会自行停车。[framework/safety.py](../framework/safety.py) 的 `enforce` 提供编排层停止联动，实时命令仍需 mux/看门狗。

跟踪配准优先级为显式 `ref_index`、时间戳、XY 最近；最近点分配含转层 z 消歧。偏差超限报 `E_TRACK_DEVIATION`，参考耗尽报 `E_TRACK_OVERRUN`。跟踪监督比对估计位姿和参考；真值由独立评价器消费，不混入定位闭环。

## 定向测试

从仓根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/validator/test_trajectory_checker.py
python3 -B ws/src/90_platform/v8_platform/validator/test_enforcement.py
python3 -B ws/src/90_platform/v8_platform/validator/test_tracking.py
```

SCAN 接缝风险见[数学探针](../probe_mode3/README.md)，完整运行与 A5/A7 状态见[进度](../../../../../note/进度/v8/README.md)。
