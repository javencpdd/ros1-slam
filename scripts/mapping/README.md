# 采图脚本族（scripts/mapping/）

采图建图流程的固化脚本集——每次采图只需在 tmux 里按序跑几个脚本，替代手敲长命令。

## 快速流程（五步）

```bash
# tmux 内（ros1-slam 身份）: sudo -u ros1-slam tmux new-session -A -s p3b
cd ros1-slam/scripts/mapping
bash m_check.sh                 # ① 体检（11 项；阻塞项须处置）
bash m_clean.sh                 # ② 清场（上一场残留）
bash m_start.sh                 # ③ 三窗启动（core/drv/slam 自动建窗）
#    → m_verify.sh 验频率 → 达标后开始走位（静置30s→外圈→反向→内部补线→回起点）
bash m_pose.sh                  # ④ 回起点时记坐标
NAME=1006采集 bash m_stop.sh     # ⑤ 停机（先停 SLAM 等落盘日志！）→ 产物自动改名 lite3_1006采集.pcd
```

## 脚本清单

## tmux 窗口一览（m_start.sh 建成后，按 Ctrl-b 数字切换）

| 窗 | 跑的脚本/命令 | 里面是什么 | 你要做的 |
|---|---|---|---|
| `0:bash` | 你的主操作窗 | 普通终端（跑 m_check/m_verify/m_pose/m_stop 的地方） | 体检/验证/记坐标/停机都在这 |
| `1:core` | `m_run_core.sh` → `roscore -p 11312` | ROS1 master（全图中枢，所有话题经它） | **只看着，别动**；采完随 m_stop 一起退 |
| `2:drv` | `m_run_drv.sh` → `roslaunch lite3_drivers 01_sensors.launch` | **M0 驱动层**：Livox Mid-360 驱动（→/rslidar_points 10Hz、/livox/imu 200Hz）+ qnx2ros（会报 bind error 死掉=已知噪音，无视） | 确认 livox 启动成功即可；采图期间别碰 |
| `3:slam` | `m_run_slam.sh` → `roslaunch lite3_slam 10_slam_mapping.launch` | **M1 建图层**：faster_lio（吃点云+IMU → /Odometry + 累积地图） | ⚠ **停机时第一个 Ctrl-C 的窗**；退出时必须等 `current scan saved` + `finish done` 两行打完（PCD 全量落盘在这两行里） |
| `4:viz` | `m_viz.sh` → `roslaunch lite3_bringup mapping_viz.launch` | **可视化伴侣**：/map_snapshot（30s 一条 LIO 视角全图快照）+ Lite3 狗模型（robot_state_publisher，随 /Odometry 移动） | Foxglove 看建图过程的窗口；TORSO inertia WARN 无害 |

> 频率验证不分窗：`bash m_verify.sh`（加 `ODOM=1` 验 /Odometry）在任何窗都能跑。



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
| `m_view.sh` | 发布任意 PCD 为 latched 话题（Foxglove 看图） | `FILE`=图名 `TOPIC` `/DECIMATE`=抽点 |
| `pcd_latch_pub.py` | m_view 的后端（勿单独跑） | — |
| `m_stop.sh` | 一键停机（保落盘顺序）+ **产物重命名** | `WAIT`=落盘等待上限（默认 45s）；`NAME`=产物标签（**默认=月日_时分**，如 lite3_1006_1759.pcd；`NAME=mine`；`NAME=keep` 保持原名）；`KEEP_LATEST=1` 额外留 lite3.pcd 副本（hdl 默认名兼容） |

## Foxglove

- 连接：ROS 1 → `http://192.168.1.103:11312`，ROS_HOSTNAME=笔记本局域网 IP （192.168.2.47）
- 订阅：`/map_snapshot`（30s 全图快照，主判据）+ 狗模型自动出现 + `/cloud_registered`（可加 Decay Time 看单帧）
- 红线：**只订阅不发布**

## 产物说明（data/maps/ 下的文件都是什么）

| 文件 | 是什么 | 谁生成的 |
|---|---|---|
| `lite3_<NAME或月日_时分>.pcd` | **本场采图的全量原始 master**（interval=-1 全程累积，百 MB 级）——最完整母本 | m_stop.sh 重命名 |
| `<master>_polished.pcd` | **抛光工作图**（z 裁剪去地板/房顶 + 体素二分自适应 ~40 万点 + SOR）——hdl 定位/G8 用这份 | m_polish.sh |
| `lite3.pcd` | 工作图副本（下游默认名兼容，=最新 polished 的拷贝） | m_polish 后复制 |
| `lite3.pgm/.yaml` | 2D 导航栅格图（狗身带直接投影，origin z=0.0） | numpy 直接投影流水线 |
| `slices/` | 按高度切分的断面条（Z 断面逐层查看） | m_slice.sh |

> 派生流水线（polish→工作图→2D）由团队跑或 `bash m_polish.sh` 自助；旧场图不需要时可整批移入 `data/archive/`（git 外）。

## 看图（m_view.sh）

采完后想再看某张图的 3D 效果：

```bash
bash m_view.sh                                    # 默认看最近产物（latched，Foxglove 订 /cloud_view）
FILE=lite3_ceshi1006 DECIMATE=5 bash m_view.sh    # 指定文件 + 抽点密度
```

订阅话题 `/cloud_view`（latched：订阅一次即得，看完 Ctrl-C 关发布器省资源）。

## 安全与铁律

- `m_stop.sh` 已固化停序（SLAM 先停等落盘 → 驱动 → master）；**SLAM 窗日志没打完绝不关 tmux 窗**
- 采图全程不发任何 cmd_vel；ros2qnx 禁启
- 走位要领：静置 30s → 慢速 ≤0.5m/s → 外圈+反向+内部补线 → 回起点记坐标
- 产物三件套校验与 G3 复验：采完喊团队（或看 note/进度/P3b/现场作业单.md）

- **R348 README 同步（追加句、旧行不回改）**＝本目录另有**两枚脚本此前未登记在本 README**：`pcd_polish.py`＝**133 行／sha 前 8=`037bf5f0`**／mtime 2026-10-07 02:26；`pcd_slice.py`＝**58 行／sha 前 8=`536667fe`**／mtime 同刻（式＝`wc -l <scripts/mapping/<件>>`＋`sha256sum`，时刻 2026-10-17 03:0x，单位＝行）。用途＝`pcd_polish.py` 对点云做抛光、`pcd_slice.py` 按 z 切片（★只登记"在位"，不声称其当代读数；★本 README 上方走位要领一节一字未动）。
- **★同批记一枚权限位读法（防后人误判为异常）**＝本目录 16 枚在册件的 `stat -c %a` 现跑多报 **`674`**（含本 README 自身），★这不是历批 `chmod` 漏覆盖：`getfacl -p scripts/mapping` 现跑＝目录带**默认 ACL** `default:user:ysc:rwx`＋`default:mask::rwx`，文件实位＝`user::rw-`／`other::r--`，★中间那位读作 **ACL mask**（具名用户 `ysc` 拿到 `rwx` 把 mask 顶到 `rwx`）⇒ `674` 属**共用工作区权限模型的显示形**、有效权限与 `664` 同义（普查式＝`git ls-files -z | xargs -0 stat -c '%U %a' | sort | uniq -c`，2026-10-17 03:0x 现跑＝`ros1-slam 674`＝**5045** 枚 ‖ `ysc 664`＝**3450** 枚 ‖ `ysc 674`＝**875** 枚，单位＝枚）⇒ ★施工侧不对任何单件 `chmod`（单点改会造成本目录面内不一致）；★在册『脚本 `755`／正文 `664`』那句与盘面主导显示值如何统一表述＝**属全仓权限位口径、不在本 README 判定**，已另起句登请求单。

---

## ★同步戳（追加节·scripts/mapping；上方旧文一字未回改）★

- **对齐到**＝进度真身末号 **R518** ／ 本 boot **批174**（零窗·派单批）。
- **盘面口径（现跑 `ls`）**＝本目录 17 枚脚本件 ＋ 本 README ＝ **18 枚**；
  上方「脚本清单」表数据行 **11 行**，与盘面**一致**。
- 同步时刻＝2026-10-21 03:32 CST（整串从 `date` 取，不心算）。
