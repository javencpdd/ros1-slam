# R1044 首次 18-run（已被本批 goal-block 修复取代）

> 更新时间：2026-11-04（v8 现行口径校对）｜ 权威口径：计划书 v8（note/计划书v8/）；本文件如与 v8 冲突，以 v8 为准

这是局部规划器修复前的 18-run 矩阵：18/18 PASS，但目标被障碍占据时 A* 返回
best partial progress，机器人一路逼近到障碍前约 0.2 m 才拒绝。读数：

- 0.2：stop max 4.906 s、path max 0.521151 m、forward max 0.441118 m、min gap 0.195385 m；
- 0.4：stop max 2.882 s、path max 0.477821 m、forward max 0.400810 m、min gap 0.159047 m；
- 18 run 仅 10 条互异结果。

当时提议 6.4 s / 0.68 m，被独立复核第一轮否决（0.68 m 越过可用净空；0.2 实速未达；
静态瞬移入感；无同批 kinematic 负控）。本目录只留 summary/manifest 与原始 full sha256；
完整 raw 在容器 `/tmp/r1043_matrix2`（未入仓）。随后 R1044 增加 goal-block 分支并重跑，
新读数与 scoped 冻结见上级目录。
