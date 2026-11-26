# M2 stageA（2026-11-27 02:59）

判定：**FAIL**（9 pass / 2 fail）

| 判据 | 结果 | 读数 |
|---|---|---|
| A 闭环走完 | FAIL | phase_fwd_back_TIMEOUT |
| A③ 漂移 | FAIL | drift=27.170 m / gt_drift=27.170 m（>0.5） |
| A① 点云 | PASS | 9.6 Hz |
| A② IMU | PASS | 200 Hz |
| A④ 飞点 | PASS | 0.0000 |
| A0 RTF | PASS | 0.422 |
| 位姿源 | PASS | /Odometry=faster_lio_online, /sim/odom_gt=gazebo |

★关键观察★：drift 与 gt_drift **完全相等（27.170）** ⇒ 位姿跟踪没跑偏，是**机器人本身没回到起点**（回程段超时）；
且 mw_wall=187.7 s 只换来 mw_sim=79.1 s（RTF 0.422）——回程预算按仿真时间给，慢速仿真下容易耗尽。
