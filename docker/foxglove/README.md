# Foxglove与ROS1观察连接

> 最新更新时间：2026-11-06 15:33:00 CST。本轮核对rosbridge配置、话题生产者与布局JSON，未启动服务或验证客户端导入。

本目录的 [ros1_slam_sim_layout.json](ros1_slam_sim_layout.json) 是手写布局模板。浏览器客户端需要兼容 **ROS1 rosbridge WebSocket**；`ws://<可达地址>:9090` 是桥的端点，ROS master的11312/11313不是浏览器端口。不要把该桥选成使用另一协议的“Foxglove WebSocket”连接。

## 网页端 Foxglove 模式（2026-11-12）

控制台页（`subprojects/console/console/ui/console.html`，agent `:8780/console.html`）的「可视化后端」可切到 `Foxglove（关本页重载）`：
切换时页面会停服务端体素流（agent `/api/stacks/voxel_stream/stop`）、停浏览器
实时点云/3D 渲染与重载监视订阅（`/tf` 等，只留 `/v8/bt/status`），并显示连接
指引（ws 地址 + 本目录布局模板链接）。**rosbridge 必须保持运行**——Foxglove 经
它连接（选 Rosbridge (ROS 1)，不是 Foxglove WebSocket；本镜像未装
`foxglove_bridge`）。实测读数见
`note/进度/v8/M5/26-11-12/evidence/scan-console/foxglove_mode.md`。

**SCAN 布局**：[ros1_slam_scan_layout.json](ros1_slam_scan_layout.json) 面向
SCAN 栈：3D 面板含 `/cloud_registered`（建图点云）、`/global_path`、
`/v8/bt/stage_markers`（编排阶段 3D 文本标记）、TF、`/Odometry`；右栏
Raw Messages 看 `/v8/bt/status`。阶段标记由 `scan_bt_nav` 里的
`bt_stage_marker.py` 发布（只读，跟随机器人位姿显示当前阶段与 seq/event）。
完整树结构仍用 Groot2（ZMQ 5555）。

## 连接域先对齐

| 运行图 | ROS master | 桥的要求 |
| --- | --- | --- |
| Compose开发域 | 默认 `http://roscore:11312` | extras的独立rosbridge默认面向这一域，节点地址和动态端口须能互访 |
| `sim` / `sim-viz` | `http://127.0.0.1:11313` | 节点通告loopback，应把桥运行在同一个容器/网络命名空间 |
| v8 3D app | 默认独立11537，按app实际配置 | 桥需连接该app运行的ROS图，不能沿用另一个11313域 |
| 真机采图 | 本机现场11312配置，IP现查 | 真机运行另有许可，不能照抄旧机IP或仿真loopback配置 |

当前 `docker-compose.yml` 中独立的 `rosbridge` 缺省连开发 `roscore`。仅把它的 `ROS1_MASTER_URI` 设成 `127.0.0.1:11313`，会指到**桥容器自身**；即便改成sim容器IP，其他节点通告的loopback也仍不可达。按当前[执行队列](../../note/计划书v8.1/23_v8.1任务队列.md)，rosbridge基线修复已延后，本文不宣称跨容器观察已经修复。

已明确选择单容器仿真观察时，可在同一个sim容器内另开终端运行桥（以下示例只针对本次独占的sim域）：

```bash
cd /home/jack/ysc-code/ros1-slam/docker
docker compose exec sim bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  roslaunch rosbridge_server rosbridge_websocket.launch'
```

`sim` 的Compose端口表没有发布9090，宿主LAN地址的9090不会因此自动可用。Linux宿主可查询容器的实际bridge地址，并在可达时连接其9090；远程客户端需要显式隧道或经核准的端口配置。不要另起独立桥抢宿主9090后误认为接到了同一个图。

```bash
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ros1-slam-sim
```

该例的端到端网络和客户端兼容性本轮未实测。桌面观察的可用入口见 [Docker指南](../README.md)。

## 固定帧与话题

| 数据链 | 固定帧 | 主要话题/模型来源 |
| --- | --- | --- |
| LIO采图快照 | `camera_init` | `mapping_viz.launch` 启动包内 `map_snapshot_pub.py`，累积 `/cloud_registered` 并发布latched `/map_snapshot`；`robot_description`来自`Lite3_prim.urdf`，模型经TF随LIO位姿 |
| 仿真GT采集 | `odom` | `mapping_recorder.py` 发布 `/mapping/cloud`、`/mapping/status`，位姿源 `/sim/odom_gt`；不能当成LIO地图 |
| 2D导航 | `map` | `/map`、实际MBF costmap话题和TF；旧 `x2_g6` 探针命名空间不能套用所有栈 |
| v8 3D导航 | `map` | 当前graph的 `/v8/3d/...` markers/path等，先确认桥连接的是11537域 |

LIO快照生产者真身是 [lite3_bringup/scripts/map_snapshot_pub.py](../../ws/src/70_bringup/lite3_bringup/scripts/map_snapshot_pub.py)，不是已不存在的 `scripts/map_snapshot_pub.py`；伴侣launch见 [mapping_viz.launch](../../ws/src/70_bringup/lite3_bringup/launch/mapping_viz.launch)。真机包装脚本 `m_viz.sh` 不应直接套到Gazebo GT记录器链。

## 布局

布局JSON声明 `version: 1`，以3D面板显示 `/map_snapshot`、TF、里程计与模型；它不是客户端正式导出的兼容性证明。客户端版本可能改变导入格式和菜单，导入失败时按相同固定帧/话题手工配置，并把经过实际导出验证的新布局另行维护；本次不覆盖原JSON。

观察时只订阅。网页发目标、改起点或热切场景会改变仿真状态，属于独立操作，不列入只读观察步骤。2D/3D交互操作及地图前置见 [Docker指南§4.6.1—4.8](../README.md)，真机不由此取得运动授权。

## 2026-11-27 实测通过：在运行中的 sim-viz 容器内起桥（不重启验收容器）

### 步骤
```bash
# 1) 在**正在运行**的 sim-viz 容器里起桥，连本容器的 11313 域
docker exec -d ros1-slam-sim-viz bash -lc 'source /opt/ros/noetic/setup.bash; \
  source /opt/ros1-slam/ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313; \
  roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 address:=0.0.0.0'

# 2) 浏览器连接（二选一）
#    容器 IP 直连：ws://172.20.0.5:9090
#    SSH 隧道：ssh -L 19090:172.20.0.5:9090 <宿主>  → ws://127.0.0.1:19090
```

### 四件核验（缺一不算通）
```bash
docker exec ros1-slam-sim-viz python3 /opt/ros1-slam/scripts/verify/foxglove_ws_check.py \
  ws://127.0.0.1:9090 --topic /v8/local/pose
```
实测读数：
```
HANDSHAKE OK
TOPICS n=99 m2_ok=True          # 桥必须能看到 /v8/local/pose、/v8/bt/status、/v8/local/status
DATA OK n=3 topic=/v8/local/pose
VERDICT PASS
```
即：握手 101、话题域正确（99 条，含 M2 关键话题）、真实数据可订阅。
仅"9090 在监听"不算通过——原有 `ros1-slam-rosbridge` 容器连的是 **11312** 开发域，
`/rosapi/topics` 里只有 4 条，接上去是空面板。

### 边界
- 只读：不发 goal / initialpose / cmd_vel，不调改状态的服务；允许 `rosapi` 查询。
- 不重启验收容器；11312 域的桥保持原样。
- 观察开销需记录（RTF、`/rslidar_points` 激活与感知负载），且**不改变正式判据**。

### 在 Foxglove 客户端里怎么点（桌面版，中文界面）
1. 右上角 **「打开连接」** → 选 **Rosbridge (ROS 1)** → 地址填 `ws://172.20.0.5:9090` → 连接。
   - 连不上时先 `ping 172.20.0.5`；跨机就用 `ssh -L 19090:172.20.0.5:9090 <宿主>`，再连 `ws://127.0.0.1:19090`。
2. 左侧 **「布局」** → 右上 **「⋯」→ 导入** → 选本目录下的 json：
   - `ros1_slam_sim_loc_layout.json`  定位诊断（3D + local status 原始消息）
   - `ros1_slam_sim_nav_layout.json`  导航诊断（全局/局部路径、障碍、地形）
   - `ros1_slam_sim_task_layout.json` 任务生命周期（BT status + cancel_ack）
3. 3D 面板里把 **Fixed frame** 设为 `map`，否则点云与轨迹会因缺 TF 不显示。
4. 三类布局由 `scripts/ops/gen_foxglove_layouts.py` 生成，话题名按实际图核过，改图后重跑该脚本即可。

## 本目录每个文件是干什么的

| 文件 | 作用 | 什么时候用 |
|---|---|---|
| `README.md` | 本说明：连接方式、导入步骤、每个文件的作用、核验口径 | 第一次用、或换了机器/网段时 |
| `ros1_slam_sim_loc_layout.json` | **定位诊断布局**：3D（pose / TF / 点云）+ Raw Messages 看 `/v8/local/status` | 怀疑位姿不对、z 偏差、定位失效时 |
| `ros1_slam_sim_nav_layout.json` | **导航诊断布局**：3D（全局路径 / gait 路径 / 局部轨迹 / 障碍 / 地形）+ local status | 车不动、走偏、避障异常时 |
| `ros1_slam_sim_task_layout.json` | **任务生命周期布局**：`/v8/bt/status` + `/v8/local/cancel_ack` | 目标不生效、取消没确认、任务序列异常时 |
| `ros1_slam_sim_layout.json` | 旧版**通用仿真**布局（SCAN 时代的单 3D + BT 原始消息） | 只做兼容保留；新排查优先用上面三份 |
| `ros1_slam_scan_layout.json` | 旧版**建图/编排阶段**布局（`/cloud_registered`、`/global_path`、`/v8/bt/stage_markers`） | 建图（Stage A）阶段用，不是导航排查 |

### 三份新布局怎么来、怎么改
- 生成器：`scripts/ops/gen_foxglove_layouts.py`（话题名按**实际图**核过，不按默认猜）。
- 改图后重跑生成器即可，别手改 JSON：
  `python3 scripts/ops/gen_foxglove_layouts.py`
- 格式要点（踩过的坑）：顶层必须是 `configById / globalVariables / userNodes / playbackConfig / layout`；
  `configById` 的**值直接是面板配置**，不能再包 `{id,title,config}`，否则客户端报「未知的面板类型」；
  面板类型由 key 前缀表达（`3D!` / `RawMessages!`）；`layout` 是 JSON **字符串**。

### 布局能显示什么、不能显示什么
- `String` 类型的 JSON（`/v8/local/status`、`/v8/bt/status`）用 **Raw Messages** 面板读原文，
  不会自动变成曲线或 3D 图元——需要曲线时另配 Plot 面板并解析字段，属待办。
- 3D 面板只画**原生消息类型**（PointCloud2 / Path / PoseStamped / TF）。
- 因此"轨迹/障碍/任务数据能看见"必须用对应面板单独验，不能只在 3D 面板里加了话题就说显示成功。
