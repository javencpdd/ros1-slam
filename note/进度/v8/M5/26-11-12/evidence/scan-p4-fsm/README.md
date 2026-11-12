# scan-p4-fsm —— P4 取证面补 FSM 迁移的活体验证窗（2026-11-12）

链：v_scan_p4.sh 原骨架（roscore 11313 → m2_gazebo → m2_mapping → 60_safety →
70_scan navi_mode=1 → p4_replan.py），sim-viz 容器，平地 m1_world，注入 obs_5
沿首规 60% 处（-1.73, 1.03, z=0.4）。负载起前 2.68/72 核，端口起前 0。

## 判定

`P4 重规划结论 = PASS`：看见 85 点 ⇒ avoid 0.68≥0.40 ⇒ 物理 0.66>0.40 ⇒
到达 0.143≤0.35；restored=True（delta_m=0.0/qdelta=0.0）；世界件 sha 起前＝退后；
样张 sha 起前＝退后；teardown 残留=空、11313/11312 监听=0。
WITNESS=INVALID（容器内无 systemd，六 unit 覆盖面快照 12 行≠6）⇒ 本窗**不作零触碰声称**，
与 §4 同口径。

## FSM 迁移诊断（本窗新增面，只诊断不入判定）

- 原始迁移行：`fsm_transitions.log`（593 行，含 state 打印行；555 条迁移）。
- `fsm_summary.json`＝**收口时用现版 fsm_log.py 从原始行重算**（p4.json 内嵌 fsm
  字段是窗内旧版解析器所写，缺 emergency_by_source；两份同源于同一原始日志，
  重算不是改读数）。
- 读数：replan 触发源 FSM（周期）=290／TRIG=2／SAFETY=0；EMERGENCY_STOP=8，
  **全部 SAFETY 触发**，每次紧跟 `[FSM]: EMERGENCY_STOP → GEN_NEW_TRAJ` 恢复。
- 解读（只对本窗）：注入箱的避让由**周期重规**完成（障碍进占据图后下一轮周期重规
  让开），安全碰撞检查没有以 REPLAN_TRAJ 形式介入；但安全检查在本窗触发过 8 次
  急停并都恢复——P4 门值仍全过（物理最近 0.66 m）。急停成因未定论：FSM cout 无时间戳，
  只能排序不能对时，注入时刻与急停的先后关系无法从本包直接指认 ⇒ 留作后续窗口
  观察项（给 scan.log 行加时间戳或 rosbag /rosout 才能对时）。

## 件清单

| 件 | 内容 |
|---|---|
| harness.log | harness 全程（含红线预检/还原/终检） |
| p4.json | 判据行＋JSON 同走 stdout（R220 在册形状，首行中文非合法 JSON） |
| fsm_transitions.log | 从 scan.log 抽出的 FSM 迁移/state 行（grep 形见 v_scan_p4.sh） |
| fsm_summary.json | 现版解析器对原始行的重算摘要 |
| scan.log.gz | 70_scan roslaunch 全量 stdout（gzip；FSM 行的原始出处） |
| world_*.sha / pcd_*.sha | 世界件与样张起前/退后 sha256 |

代码真身：`ws/src/40_navigation/lite3_scan/verify/fsm_log.py`、`test_fsm_log.py`
（离线 11/11 PASS，含三反向例）、`v_scan_p4.sh`/`v_scan_bt.sh` 抽取段、
`p4_replan.py`/`p4_bt_probe.py` 诊断挂点（判据链未动）。