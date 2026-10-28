# Foxglove 观察（viz.sh 路线 B）——布局件与连接口径

> 对应 `docker/viz.sh` 头部注释里的**路线 B**：不需要 X / GPU，任意机器的浏览器打开
> <https://foxglove.dev> 连 `rosbridge`（默认 9090）即可旁观仿真。
> 适合没有桌面、或不想折腾 X 认证（`viz.sh check` 里 `OpenGL renderer` 取不到值时）。

## 1. 连接参数（据在册件现读，不凭记忆）

| 项 | 值 | 出处 |
|---|---|---|
| rosbridge 端口 | `9090` | `docker/viz.sh` 头部注释路线 B |
| 建图侧 ROS_MASTER_URI | `http://127.0.0.1:11312` | `scripts/mapping/m_viz.sh` 头注释「Foxglove 连 11312」 |
| 仿真/判据侧 ROS_MASTER_URI | `http://127.0.0.1:11313` | `docker/viz.sh` 第 36 行 `OBSERVE_URI` 缺省值 |
| 固定参考系 | `camera_init` | `lite3_bringup/launch/mapping_viz.launch` 头注释 |
| 全图快照话题 | `/map_snapshot`（30 s 一条、latched） | 同上 + `scripts/map_snapshot_pub.py` 第 23 行 `frame_id = "camera_init"` |
| 机器人模型 | `robot_description` ← `lite3_bringup/urdf/Lite3_prim.urdf` | `mapping_viz.launch` 第 5 行 |
| 位姿跟随 | `/Odometry` | `mapping_viz.launch` 头注释 |
| 里程计 frame | `camera_init`（快照）／`map`→`base_link`（`sim_pose_tf.py`） | `map_snapshot_pub.py:23`、`sim_pose_tf.py:61` |

启动伴侣（采图三窗之外的第 4 窗）：

```bash
bash /home/ros1-slam/scripts/mapping/m_viz.sh
# 等价于 roslaunch lite3_bringup mapping_viz.launch
```

## 2. 导入布局件

`ros1_slam_sim_layout.json` 是**手写**的（不是 Foxglove Studio 导出的），按 Studio 的
`version: 1` 布局 schema 组织，只含一个 3D 面板（`/map_snapshot` 点云 ＋ URDF ＋ TF ＋ `/Odometry`），
相机 `distance=12 / target=[2.5,0,0] / phi=45°`——与 `docker/rviz_sim_map.rviz` 同一套取景
（依据：`m1_world.world` 的 perimeter 是 9 m 边界盒，场地约 9 m × 9 m）。

导入：`Foxglove Studio → Layouts → Import layout` 选本文件。

> **★若导入失败（schema 版本漂移是常态，尤其 Studio 大版本升级后）★** → 走第 3 节手工步骤，
> 效果等价；并把 Studio 里摆好的布局 `Export layout` 覆盖写回本文件，让仓内这份回到"官方导出"形态。

## 3. 手工步骤（兜底，永远可用）

1. 打开 <https://foxglove.dev> → `Open connection` → `Rosbridge (ROS 1)` → `ws://<主机 IP>:9090`。
2. `Add panel` → **3D**：
   - 右上角 `Settings` → `Frame` → `Follow` 取消，固定参考系选 `camera_init`；
   - `Topics` 里打开 `/map_snapshot`（`Color by = z`）、`/tf`、`/Odometry`；
   - 相机：Distance `12`、Target `[2.5, 0, 0]`、Phi `45°`。
3. `Add panel` → **URDF  viewer** 或直接在 3D 面板里勾 `Robot model`（读 `/robot_description`）。
4. （可选）`Add panel` → **Publish**：填 `geometry_msgs/PoseStamped` → 话题 `/move_base_simple/goal`
   （据 `lite3_scan/launch/70_scan.launch` 第 51 行）即可从网页端点目标点。
5. （可选）导航态另开一个 3D 面板，固定参考系改 `map`（据 M3 窗参数回读 `global_frame=map`），
   订 `/x2_g6/costmap/costmap` 与 `/x2_g6/costmap/stvl_obstacle_layer/voxel_grid`。

## 4. 与 rviz 路线的分工

| 场景 | 选谁 |
|---|---|
| 本机有桌面 / 已挂 DISPLAY，要最高帧率 | 路线 A，`viz.sh rviz-raw` / `rviz-map` / `rviz-nav` |
| 远程机、无 X、或 X 认证折腾不通 | **路线 B（本目录）** |
| 无 X 又必须跑 GUI（gzclient/rviz） | 路线 C，`viz.sh` 容器内 Xvfb + VNC |

---

## ★同步戳（追加节·docker/foxglove；上方旧文一字未回改）★

- **对齐到**＝进度真身末号 **R518** ／ 本 boot **批174**（零窗·派单批）。
- 本目录＝`viz.sh` **路线 B** 的布局件与连接口径；**红线＝只订阅不发布**。
- 同步时刻＝2026-10-21 03:32 CST。

## ★追加（2026-10-29，R858）：起终点控制与场景热切换

- 连接：`Rosbridge (ROS 1)` → `ws://<主机>:9090`。本镜像已装 `rosbridge_server`；无 `foxglove_bridge`（后者是 ROS2）。
- **终点**：Publish 面板发 `geometry_msgs/PoseStamped` → `/move_base_simple/goal`（需 `flat2d_stack` 的 move_base）。
- **起点**：先起 `ws/src/95_simulation/lite3_gazebo_sim/scripts/sim_pose_bridge.py`，再发 `geometry_msgs/PoseStamped` → `/sim/set_start`（或 `PoseWithCovarianceStamped` → `/initialpose`）；状态回 `/sim/set_start/status`。桥只接受 frame 为空/`map`/`odom`。
- **场景热切换**：`sim_scene.py --scene stairs_easy` / `--scene wall_h045` / `--clear`；只换场景模型，不重启世界。静态地图不随动，正式导航需重启对应世界。
- 实测读数见 `note/进度/v8/M5/26-10-29/R858.md`。
