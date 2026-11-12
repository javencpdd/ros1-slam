# scan-p4-astar-fsm —— 斜坡 A* 模式 P4 失效层定位窗（2026-11-12，#27）

目的：给诊断文档 §13「A* 模式坡上 P4 replanned=False」补 FSM 触发源证据，
钉死失效卡在哪一层。链＝v_scan_bt.sh 非 profile 路径（roscore 11313 →
m2_gazebo → m2_mapping → 60_safety → 70_scan → scan_bt_nav），sim-viz 容器。

## 配置（与 §13 同族，参数现值随读）

- world=ramp_scan10.world（10° 平滑坡），spawn x=0/z=0.88/yaw=π（坡上出生）
- **navi_mode=1（A*）**，ground_filter=true，节点=70_scan 默认
  scan_cloud_ground_segment.py（平面分割 distance=0.08＋column=0.10 列保留）
- 探针 p4_bt_probe.py：P4_GOAL_XY=1.5,0、P4_INJECT_FRAC=0.5、P4_INJECT_OFFSET=0.25

## 判定与读数

`SCAN-BT 结论 = FAIL`（probe reason=BT outcome=failed）。首规正常
（path 120 点、bspline start_time 17.098、末点 [1.493,0]）；注入 (0.76,0.26)
看见 **21** 点；**replanned=True 但新轨迹退化**：start_time 18.778、n_ctrl=6、
**first=last=[0.578,0.015]（零长度）**。avoid 0.305<0.40、物理 0.33≤0.40、
A23 3D 0.344≥0.10、dist_end 1.439 m、scan_arrived=false、restore ok。

## FSM 归因（本窗核心新增）

227 迁移：replan 触发源 **SAFETY=4／FSM（周期）=46／TRIG=0**；
EMERGENCY_STOP=2 **全 SAFETY**（sim 18.778 与 30.623，scan.log 两条
"Suddenly discovered obstacles. emergency stop!" WARN 逐字对应）。
`a star error`=**606** 条；"Unable to refresh global trajectory"=0 条。

**关键对应**：traj_after.start_time=18.778 与第一次急停 WARN 的 sim 时刻逐字相同
⇒ 探针看见的「新轨迹身份」是**急停零长度轨迹**，不是避让重规。

## 结论（修正 §13 的表述层）

1. A* 模式坡上 P4 的失效**不是「没有触发」**：安全与周期重规都触发了
   （4+46 次），而是**重规尝试产不出可行避让轨迹**（606 条 a star error），
   安全层兜底急停，机器人停在 0.578 m（距目标 1.439 m）。
2. §13 的 replanned=False 与本窗 replanned=True（退化）是同一失效的两种表现，
   随点云/几何抖动（看见 44 vs 21 点）；两窗皆 FAIL，定性不变。
3. 探针身份判据分不出急停轨迹 ⇒ 已加诊断标记 `traj_after_degenerate`
   （bs_degenerate，只诊断不入判定）；本窗数据验证：after=True／before=False。
4. 剩余修复面＝上游 A*/rebound 在坡面+箱体的重规语义（30_3rd 基线），
   属诊断文档 §4 的 L2 选项，壳侧无在不改被测算法前提下的修法。

## 件清单

probe.json（INFO+JSON 同 stdout）／harness.log／fsm_transitions.log／
fsm_summary.json（现版解析器重算）／scan.log.gz（含 606 条 a star error 与
两条急停 WARN 的原始出处）。