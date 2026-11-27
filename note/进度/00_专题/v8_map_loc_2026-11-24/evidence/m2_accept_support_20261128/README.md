# M2 Support/FAR 四例运行验收（2026-11-28，BLOCKED）

## 结论

**0 PASS / 1 BLOCKED / 3 NOT_RUN**。未假绿、未改门值、未用 GT 或虚假输入。

Support M2 正常导航在**局部起点支撑关联**处阻断，两次独立复现读数一致：

| 量 | 第一次（`nav_graph/`） | 干净复现（`clean_20261128/`） |
|---|---|---|
| `/v8` 节点 | 9/9 | 9/9 |
| BT 路径 | 14 点 | 14 点 |
| local 目标 | — | `(4.85, 2.45, 0.0)`（契约字段） |
| local 起点 z | `0.0598` | `0.0597` |
| 阈值 | `1e-06` | `1e-06` |
| 候选支撑 | `0` | `0` |
| `/cmd_vel` | 零 | 零 |
| 结论 | `E_LOCAL_SUPPORT` | `E_LOCAL_SUPPORT` |

干净复现的 `/v8/local/status` 原文（`clean_20261128/local_status.yaml`）：

```text
stop=true reason=E_LOCAL_SUPPORT
detail=E_LOCAL_SUPPORT: start requires a matching z support
       | z=0.0597 z_tol=1e-06 候选=0 歧义=False 地图=1
       | start=(5.203940445, 2.465831694, 0.059717872)
         goal=(4.85, 2.45, 0.0)
```

同一时刻：

- `/v8/local/pose` = `(5.175854, 2.468616, 0.061571)`
- `/sim/odom_gt` = `(5.200000, 2.450000, 0.110000)`
- `/cmd_vel.linear` = `(0, 0, 0)`
- `/v8/bt/status`：`path_state=ready`、`path_points=14`、`follow_path=SUCCESS`、`wait_arrival=RUNNING`、`localization_ok=true`
- 定位年龄 `0.281 s`，地图版本 1，不是 TF 断链、不是时钟过期、不是 GT 混入。

GT 与估计位姿差约 6.1 cm（三维），与 A4 118 帧残差分布（mean −0.05416，abs_max 0.06103）一致。**这是估计误差进入局部支撑关联的结果。**

## 为什么停

用户明确：`support_tol_m` 与安全判据不放宽，`0.08 m` 未批准正式使用。此处 `support_assoc_tol_m=1e-6`，`0.0597 m` 无法关联到 `z_reference=0.0` 的地面。把容差放宽到 0.08 或把 z 偏置写死，都会放宽或掩盖真实估计误差，属于 L2，未擅自执行。

因此：

- FAR M2 图未启动四例（同一 `local_native` 前置，必然同样阻断，不重复浪费跑批）。
- 障碍/定位失效/取消三项未跑（依赖正常导航先取得非零运动）。
- A3 的 GT 闸门负控（关闸、缺边、双父、双发布者、停钟）已独立 PASS/FAIL 留证；GT TF/话题独立关断仍需全图任务窗。

## 运行现场

- 容器：`ros1-slam-sim`，M2 隔离模式：`M2_ISOLATE_GT=1`，地图/出生点 `(5.2, 2.45)`。
- 地图：`data/maps/generated/v8_m1_hdl_v1.json`；`z_reference=0.0`。
- 图：`graph_3d_m2_lio_bt.yaml`，9 个 `/v8` 节点（`lio_gate/loc/map/perception/global/bt/path_goal/local/mux3d`）。
- 第一次跑批：`nav_graph/graph.log` 记录 BT 目标 `3.9,2.45,0`（14 点），p4 探针在写出 `nav_probe.json` 前因 ROS shutdown 退出，**没有导航 PASS/FAIL**；随后人工收图。`nav_timeline.tsv` 是并发采样，不作为独立判据。
- 干净复现：`clean_20261128/` 只起装配并单发一条目标；所有读数来自该目录文件，未复用旧进程状态。
- 收图后确认 `rosnode list` 中 `/v8` 为 0，底座 `faster_lio` 仍约 10 Hz 发布 `/Odometry`，未被本次收图杀死。

## 需要 L2 裁决的候选（不自行勾选）

1. **甲：仅在 M2 仿真验收使用 `support_assoc_tol_m=0.08`**，并在证据中标注 scoped/experimental，不进入 formal/真机。理由：实测 abs_max 0.061<0.08；代价是放宽局部起点关联，不等于放宽碰撞/净空。
2. **乙：保留 1e-6，改 LIO 高度/重力对齐或定位质量门**。理由：不许用残差拟合补偿；需改核心定位算法或新增地面约束，属算法/真机参数决策。
3. **丙：保留 1e-6，改为 XY/面关联后再用接触点 z，不把估计 z 当起点支撑判据**。理由：避免以 z 误差判“无支撑”，但改变了起点支撑语义，需判据面签字。
4. **丁：补一版独立高度估计/外部高程输入**，局部只用 XY+接触面高度。工作量最大，但不动安全容差。

**推荐先做乙的只读诊断、再在甲/丙之间裁决**：乙不改变门值，只证明误差来源；甲的 scoped 容差是当前唯一能让四例继续跑的最小改动；丙语义改动最大，不建议先动。

## 未动项

- 未改 `support_tol_m`、`support_assoc_tol_m`、`cmd_vel` 安全参数、FAR 核心搜索、`local_native` 几何判据。
- 未启用风险权重。
- 未上真机；未向真机发送任何帧。
