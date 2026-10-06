# 采图脚本族（scripts/mapping/）

采图建图流程的固化脚本集——每次采图只需在 tmux 里按序跑几个脚本，替代手敲长命令。

## 快速流程（五步）

```bash
# tmux 内（ros1-slam 身份）: sudo -u ros1-slam tmux new-session -A -s p3b
bash m_check.sh                 # ① 体检（11 项；阻塞项须处置）
bash m_clean.sh                 # ② 清场（上一场残留）
bash m_start.sh                 # ③ 三窗启动（core/drv/slam 自动建窗）
#    → m_verify.sh 验频率 → 达标后开始走位（静置30s→外圈→反向→内部补线→回起点）
bash m_pose.sh                  # ④ 回起点时记坐标
bash m_stop.sh                  # ⑤ 停机（先停 SLAM 等落盘日志！）→ ls data/maps
```

## 脚本清单

| 脚本 | 作用 | 关键超参 |
|---|---|---|
| `m_env.sh` | 公共环境（勿单独跑） | — |
| `m_check.sh` | 开工体检 11 项 | `$1`=雷达 IP（默认 .201） |
| `m_clean.sh` | 残留清场 | — |
| `m_start.sh` | 三窗一键启动（tmux） | `SESSION=p3b` 会话名 |
| `m_run_core/drv/slam.sh` | 各窗内容（m_start 调用，勿单独跑） | — |
| `m_verify.sh` | 频率联锁验证 | `ODOM=1` 加验 /Odometry |
| `m_pose.sh` | 当前位姿单帧 | `N`=帧数 |
| `m_viz.sh` | 可视化伴侣（快照+狗模型） | — |
| `m_stop.sh` | 一键停机（保落盘顺序） | `WAIT`=落盘等待上限（默认 45s） |

## Foxglove

- 连接：ROS 1 → `http://192.168.1.103:11312`，ROS_HOSTNAME=笔记本局域网 IP
- 订阅：`/map_snapshot`（30s 全图快照，主判据）+ 狗模型自动出现 + `/cloud_registered`（可加 Decay Time 看单帧）
- 红线：**只订阅不发布**

## 安全与铁律

- `m_stop.sh` 已固化停序（SLAM 先停等落盘 → 驱动 → master）；**SLAM 窗日志没打完绝不关 tmux 窗**
- 采图全程不发任何 cmd_vel；ros2qnx 禁启
- 走位要领：静置 30s → 慢速 ≤0.5m/s → 外圈+反向+内部补线 → 回起点记坐标
- 产物三件套校验与 G3 复验：采完喊团队（或看 note/进度/P3b/现场作业单.md）
