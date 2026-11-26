# FAR 四项完整回归（2026-11-27 02:2x，含定位收敛闸门）

| 项 | 判定 | 关键读数 |
|---|---|---|
| 1 正常导航 | PASS | gt_arrived=true, gt_to_goal=0.078 m（此前 0.1907 FAIL） |
| 2 障碍安全（离线 5 场景） | PASS | A/A2/B/C/D 全 PASS；最小可通间隙 0.650 ≥ 需求 0.613 |
| 2b 运动障碍（在线） | PASS | 见 obstacle_inmotion.json |
| 3 定位失效 | PASS | stop_delay=0.701 s, coast=0.1102 m, 恢复后不动, 新任务可解锁 |
| 4 取消 | PASS | ack=1, stop_delay=0.081 s, coast=0.0317 m, 重放被拒/新任务接受 |

定位闸门读数：est_gt_offset 0.058 / 0.050 m（限 0.15）
