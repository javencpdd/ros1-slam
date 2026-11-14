# 3D 任务台（P3 可视化/操作面）

> 2026-11-13 从 `web/` 迁出：页面与静态资源在 `scripts/console/ui/`，由宿主 agent 在
> `:8780` 独立提供；官网 `web/` 只留文档。生成物 `ui/assets/data/`、`ui/assets/maps/`
> 不入库，刷新/生成后即生效。

## 启动

```bash
bash scripts/console/start_agent.sh --start
# 本机：http://127.0.0.1:8780/console.html
# 局域网：http://<host>:8780/console.html（agent 默认 0.0.0.0，信任 LAN）
```

`agent.py` 同时提供 `/api/health`、`/api/stacks`、`/api/whoami`、`/api/voxels/*`、`/api/maps/*`，
并静态服务 `ui/`；页面每 4 s 重试 `/api/stacks`，agent 重启后通常无需刷新。

## 栈与权限

- 作业栈按 [`stacks.json`](stacks.json) 白名单启停（12 栈）；停 world 栈按依赖逆序级联停止，
  `stack_state` 同时校验 pid 组与 `pid_match` 命令身份，避免 PID 复用误报。
- `agent.py` 启动时执行 `validate_stacks()`：容器名必须属于允许集合，命令禁止 shell 元字符；
  启停动作写 `/tmp/v8-console/audit.log`。agent 不挂 Docker socket。
- `/api/whoami` 按来源 IP 判定本机/远程：回环、宿主自身 IP、`LITE3_LOCAL_CIDRS`（默认仅
  `127.0.0.0/8,::1/128`）为本地，其余为远程；远程端把“本机桌面”卡片灰化，`local_only` 栈在
  agent 侧也返回 403。只应在可信局域网使用，没有额外鉴权。
- 若只要本机访问：`LITE3_AGENT_BIND=127.0.0.1 bash scripts/console/start_agent.sh --start`。

## 页面能力

- **地图库/任务**：读取 `ui/assets/data/maps.json`；2D 栅格/点云顶视图 + 3D 点云预览；可订阅
  实时 `/cloud_registered` 或 `/rslidar_points`；叠加路径/TF 位姿/MBF 状态/costmap；多目标状态机
  按到点或 MBF SUCCEEDED 推进并记录逐目标耗时/路径/终点误差；2D 发 `/move_base_simple/goal` +
  `/initialpose`；3D 目标发 `/v8/bt/goal`（BT→goal_gate→`/v8/3d/global/goal`），起点发
  `/v8/3d/global/start`；2D 可 cancel，3D 无独立 cancel（用急停/作业单）。
- **3D 场景**：加载 `ui/assets/data/worlds/*.json` 的实体 box（可含 DAE 三角网格），在同一
  canvas 切换“点云/实体/叠加”；实体来自 SDF 精确几何，未解析的 mesh/模型库 include 在生成
  统计里列出，不在前端猜测。
- **World registry**：[`world_registry.json`](world_registry.json) 是 world 文件、SHA256、场景
  标签、地图绑定和可启动栈绑定的唯一主源；`scripts/ops/build_world_scenes.py`、
  `scripts/ops/build_maps_index.py` 从它生成 `worlds.json` 与 `maps.json` 的 `world_id`，
  `scripts/ops/test_world_registry.py` 校验三面一致。
- **视图口径**：主画布始终是所选地图的 2D 俯视预览；“3D 全链”只切换目标发布接口，不会把
  主图变成 3D。点云/实体世界用“打开 3D 场景”查看。
- **监控**：rosbridge/rosapi 状态、`/cmd_vel`、`/v8/3d/global/path`、`/v8/local/trajectory`、
  `/v8/local/pose`、`/tf` 频率与 TF 对、节点/话题列表、BT 阶段条（`/v8/bt/status`）。
- **遥控**：只向安全 mux 的 `/lite3/teleop/app_cmd_vel` 发 Twist（deadman 0.5 s）；需先启
  `safety_mux` 栈，mux 以 20 Hz 仲裁到 `/cmd_vel`。急停持续向 `/lite3/safety/estop_cmd_vel`
  发零速；**不直接写 `/cmd_vel`**。
- **服务端体素**：`voxel_stream` 栈订阅 `/cloud_registered`（优先）与 `/rslidar_points`，
  `leaf=0.10 m` 下采样，原子写 `data/console/voxels/latest.bin` + `meta.json`；浏览器按二进制
  拉取（16 B/点，含 RGBA）。
- **可视化双模**：页面可切“服务端体素/本页 Foxglove 指引”；Foxglove 经 rosbridge `:9090`
  连接（Rosbridge ROS1，不是 Foxglove WebSocket），布局模板见 [`docker/foxglove/`](../../docker/foxglove/)。
- **地图生命周期**：刷新/重栅格化/隐藏/恢复/编辑栅格；`mapping_save` 停止时保存 PCD 到
  `data/maps/imported/`（agent 起前备份、停后恢复样张，GAP-5）。
- **Gazebo world 选择**：`sim_m1` 可选 `m1_world`/`m1_world_v2`/`demo_two_floor`/`open`；
  `stairstudy` 可选 9 个 `stair_lab_*`；NIST 用 `sim_nist`（仅世界）。同一时间只起一个
  Gazebo world，换 world 先停当前栈。`demo_two_floor.world` 不是 `two_floor_nav` 的
  `open/blocked` 已知地图导航 app；后者入口是 `scripts/sim/two_floor_nav.py --execute`，
  不在网页白名单。

## 验证

```bash
bash scripts/console/check_console.sh
python3 scripts/console/test_agent_contract.py
node scripts/console/test_mission_events.js
python3 scripts/ops/test_world_registry.py
```

控制台只启停白名单 demo 栈，不启停 formal/EVT/真机，也不代批作业窗；Gazebo/导航正式启动
仍走 `docker/` 作业单与 `runtime_lab.py`。
