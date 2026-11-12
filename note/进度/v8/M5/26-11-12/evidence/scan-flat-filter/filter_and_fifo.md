# 平地地面分割过滤：目标截断修复与两目标 FIFO

- 时间：2026-11-12 03:28—03:49（CST）
- 容器：`ros1-slam-sim-viz`；master：`http://roscore:11312`（控制台域）
- 代码：`ws/src/40_navigation/lite3_scan/launch/scan_nav_sim.launch`（flat 默认
  `ground_filter=true`、`cloud_topic=/LIO/clouds_lidar_ground_filtered`、
  节点 `scan_cloud_ground_segment.py`，`ground_plane_distance=0.08`）

## 现象（未过滤，profile:=flat 旧默认）

控制台连发两目标（goal A (-1.5,2.0)、goal B (1.0,-1.5)），A 正常到达；
B 在已观测区域，SCAN 反复截短终点：

    [ WARN] [..., 71.300000000]: [global target] Target [1.00, -1.50, -0.27] is occupied; use backward collision-free point [0.78, -1.20, -0.30].
    [ WARN] [..., 76.281000000]: [global target] Target [0.78, -1.20, -0.30] is occupied; use backward collision-free point [-0.02, -0.83, -0.37].
    [ WARN] [..., 78.530000000]: [global target] Target [-0.02, -0.83, -0.37] is occupied; use backward collision-free point [0.11, -0.96, -0.35].
    [ WARN] [..., 81.383000000]: [global target] Target [0.11, -0.96, -0.35] is occupied; use backward collision-free point [0.28, -1.43, -0.28].
    [ WARN] [..., 83.641000000]: [global target] Target [0.28, -1.43, -0.28] is occupied; use backward collision-free point [0.40, -1.49, -0.21].
    [ WARN] [..., 85.696000000]: [global target] Target [0.40, -1.49, -0.21] is occupied; use backward collision-free point [0.41, -1.55, -0.20].
    [ WARN] [..., 151.300000000]: [bt] arrival timeout after 176.4 s (budget 176.3 s)
    [ WARN] [..., 151.300000000]: [bt] recovery: stop and invalidate path

机器人停在 (0.37,-1.51)，BT 按原目标判到达失败；`/v8/bt/arrived` 始终 false。

## 修复后（平面分割过滤开）

同链、同两目标（各自单发复验）：

    goal (-1.5,2.0)：reached，停点 (-1.502,1.989)，截断告警 0 条
    goal (1.0,-1.5)：reached，停点 (1.015,-1.524)，截断告警 0 条

连发（FIFO，goal 间隔 2 s）：

    t=6s   goal_seq=1 current_seq=1 outcome=""
    t=18s  goal_seq=2 current_seq=2 queue_depth=0        （A 已完成，B 接管）
    t=42s  goal_seq=2 current_seq=2 outcome_seq=2 outcome=reached x=1.008 y=-1.528
    t=48..108s outcome=reached，终点 (1.013,-1.527)
    截断告警 0 条

## 平地 P4 门禁复验（新默认）

`SCAN_BT_PROFILE=flat SCAN_BT_PROBE=.../p4_bt_probe.py bash v_scan_bt.sh`

    verdict=PASS  reason="BT outcome=reached after dynamic replan"
    goal=[-2.754,1.547]  dist_end=0.07 m  dt_sim=16.97 s（≤120）
    clearance: min_dist_to_injected=0.492（≥0.40）  min_dist_to_any_occ_3d=0.304（≥0.10）
    physical.robot_min_to_injected=0.876 m  restore.ok=true  replanned=True
    原始 2D 最小距离 0.017（只记录，判据用 3D）

证据：`evidence/scan-p4-bt/flat_bt_p4_groundfilter.json` + `.harness.log`。

## 斜坡 P4 目标相关性（同一轮，非本次改动回归）

单入口 `profile:=ramp` 的 P4 探针随机取到 goal (2.668,-1.172)、注入 obs_5（60%）：

    clearance: min_dist_to_injected=0.226（<0.40）  min_dist_to_any_occ_3d=0.028（<0.10）
    arrive: dist_end=1.687 m  dt_sim=139.88 s（>120）
    bt.outcome=""  scan_arrived=false  → verdict=FAIL

斜坡参数本次未变（`ground_filter=true`/filtered cloud/`min_z=0.0` 均与旧默认等价）；
原通过目标 (1.5,0) 在新 launch 上复验仍 PASS（reached，wall ~12 s，截断 0）。
即斜坡门禁**按目标**成立，斜向目标 + 贴路线注入仍不过——沿 §10 的 scoped 口径。
证据：`evidence/scan-p4-bt/ramp_bt_p4_goal2p67_fail.json` + `.harness.log`。

## 真机预检脚本

新增 `verify/real_preflight.py`（只读）：节点在位 / `planner_mode=signal` /
`arrived=false` / `/cmd_vel` ≥10 Hz 且全零 / `/global_path/status` 在线；
`verify/test_real_preflight.py` 纯解析单测 4/4 PASS。用法：

    python3 ws/src/40_navigation/lite3_scan/verify/real_preflight.py --window 4
