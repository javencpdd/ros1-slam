# R1044 独立复核回执摘要（非真人签署）

两轮均由独立子代理只看原始 JSON/代码做增量复核；以下是回执要点摘要，原始对话未存档。

## 第一轮（对旧 18-run 与 6.4 s / 0.68 m 提议）
- 原始数据可复算：18/18 PASS、checks 全真、latch=E_LOCAL_NO_ROUTE；
  最坏 r03=4.906 s / 0.521151 m cumulative / 0.441118 m forward；0.4 组最坏 2.882 s / 0.478 m；
  触发均对应 spawn 后首个带点云 hit；无碰撞，最小 gap r12=0.159 m。
- 判 **FAIL**：0.68 m 路径预算越过可用净空（r15 首云 base→障碍前缘 0.846 m、
  AABB 前端 0.28 m、可用前移 0.566 m；0.68/1.10≈0.618 m 超 0.052 m）；
  0.2 组触发前 0.5 s 平均体速仅 0.119—0.162；18 run 仅 10 条互异；
  静态瞬移入感、无同批 kinematic 负控；代码/证据未封存。
- 建议：formal #151 不冻结；pilot 可保留 6.4 s、distance≤0.60 m，并补独立重复与触发距离扫描。

## 第二轮（对 planner 修复、新 18-run 与 3.6 s / 0.20 m 提议）
- planner 修复判 **PASS**：旧行为对目标被障碍占据返回 best partial progress，
  机器人逼近到约 0.2 m 才拒；新行为首个 hit 即 E_LOCAL_NO_ROUTE，gap 0.454—0.613 m；
  移动障碍不被该检查误伤（test_execution 仍过）。
- 数据：18/18 PASS、13 checks 全真、trigger 与 spawn 后首个带点云 hit 全等；
  最坏 stop 2.744 s、cum 0.153347 m、forward 0.104878 m、min gap 0.454 m；
  0.2 组触发前 0.5 s 均速 0.119—0.162（cmd 0.2），0.4 组 0.426—0.434；18 run 仅 10 条不同轨迹。
- 判 **formal 冻结 FAIL / scoped 可行**：3.6/0.20 算数正确
  （2.744×1.30=3.567→3.6；0.15335×1.30=0.1994→0.20），0.20<0.454 净空安全；
  但仍是 in-process、单载体、静态瞬移、无 ROS 中间件/真机、0.2 实速未达 0.2；
  sample_n 应按互异数记 10，scope 必须写明命令速度与实际均速。
- 建议：formal 两键不冻结；另立 scoped 键，或补 ROS 全链/运动障碍/kinematic 负控后再议 formal。

本批采纳：formal 两键保持 pending；新增 scoped 两键 3.6 s / 0.20 m、sample_n=10，
scope 写明 in-process / goal_blocked / 命令 0.2/0.4；不声称 formal #151 闭合。
