# Foxglove与ROS1观察连接

> 最新更新时间：2026-11-06 15:33:00 CST。本轮核对rosbridge配置、话题生产者与布局JSON，未启动服务或验证客户端导入。

本目录的 [ros1_slam_sim_layout.json](ros1_slam_sim_layout.json) 是手写布局模板。浏览器客户端需要兼容 **ROS1 rosbridge WebSocket**；`ws://<可达地址>:9090` 是桥的端点，ROS master的11312/11313不是浏览器端口。不要把该桥选成使用另一协议的“Foxglove WebSocket”连接。

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
