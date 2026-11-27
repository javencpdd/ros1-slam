# M2 Support/FAR 四例运行验收（2026-11-28，BLOCKED）

## 结论

**0 PASS / 1 BLOCKED / 3 NOT_RUN**。未假绿、未改门值、未用 GT 或虚假输入。

Support M2（`graph_3d_m2_lio_bt.yaml`，9/9 v8 节点）第一项正常导航在**局部起点支撑关联**处阻断：

```
E_LOCAL_SUPPORT:
start requires a matching z support
| z=0.0598 z_tol=1e-06 候选=0 歧义=False 地图=1
| start=(5.204181708, 2.471238632, 0.059844797)
  goal=(4.85, 2.45, 0.0) now=60.835 obs=3
```

同时：

- `/v8/local/status`: `stop=true`, `reason=E_LOCAL_SUPPORT`, `input_ages.pose=0.141`, `map_version=1`
- `/v8/bt/status`: `path_state=ready`, `path_points=14`, `goal_seq=1`, `follow_path=SUCCESS`, `wait_arrival=RUNNING`
- `/cmd_vel.linear=0`
- `/v8/local/pose` = `(5.17797, 2.47354, 0.05832)`；GT = `(5.20000, 2.45000, 0.11000)`

GT 与估计位姿差约 6.1 cm（三维），与 A4 的 118 帧残差分布（mean −0.05416，abs_max 0.06103）一致。**这是估计误差进入局部支撑关联的结果，不是 TF 断链、不是 GT 混入、不是时钟过期。**

## 为什么停

用户明确：`support_tol_m` 与安全判据不放宽，`0.08 m` 未批准正式使用。此处 `support_assoc_tol_m=1e-6`，0.0598 m 无法关联到 `z_reference=0.0` 的地面。把容差放宽到 0.08 或把 z 偏置写死，都会放宽或掩盖真实估计误差，属于 L2，未擅自执行。

因此：

- FAR M2 图未启动四例（同一 `local_native` 前置，必然同样阻断，不重复浪费跑批）。
- 障碍/定位失效/取消三项未跑（依赖正常导航先取得非零运动）。
- A3 的 GT 闸门负控（关闸、缺边、双父、双发布者、停钟）已独立 PASS/FAIL 留证；GT TF/话题独立关断仍需全图任务窗。

## 运行现场

- 容器：`ros1-slam-sim`，M2 隔离模式：`M2_ISOLATE_GT=1 M2_MAP_SPAWN_X=5.2 M2_MAP_SPAWN_Y=2.45`
- 地图：`data/maps/generated/v8_m1_hdl_v1.json`；`z_reference=0.0`
- 图：`graph_3d_m2_lio_bt.yaml`，启动 9 个 `/v8` 节点（`lio_gate/loc/map/perception/global/bt/path_goal/local/mux3d`）
- `nav_graph/graph.log`：loc `READY`，`source=faster_lio`
- `nav_setup.log`：`nodes_ready_after=2x2s n=9`
- `run.log`：正常导航项开始后停在 `[nav] v8_nodes=9`
- 跑批在定位到第一处真实阻断后人工终止，未等待 140 s 超时，也未把超时读成 FAIL。

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
