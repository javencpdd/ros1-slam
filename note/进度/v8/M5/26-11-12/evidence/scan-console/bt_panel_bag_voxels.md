# 控制台：BT 阶段面板、bag 录制、web 实时建图（2026-11-12）

- 容器：`ros1-slam-sim-viz`；master `http://roscore:11312`；页面 `http://127.0.0.1/web/console.html`
- 栈：`scan_flat`（SCAN 全链）、`voxel_stream`（服务端体素）、`bag_record`（新增）

## 1. BT 编排阶段面板（web 新增）

改动：`web/console.html`（监控卡增 `BT 阶段` kv + `#bt-stage`）、
`web/assets/console/console.css`（`.bt-node` 状态色）、`console.js`
（`renderBtStage()` 订阅 `/v8/bt/status` 的 `nodes`）、
`mission_events.js`（`parseStatus` 补 `nodes/planner_mode/arrival_signal`）。

阶段条顺序：等目标 → 请求路径 → 等路径 → 跟随路径 → 等到点 → 到点停留 → 停止
（另列 失败恢复）；RUNNING 高亮，SUCCESS 绿、FAILURE 红、IDLE 灰。

浏览器实测（scan_flat 运行中，页面经 rosbridge 9090→11312）：

    idle:  字段 "等目标 ｜ seq 0/0 ｜ signal ｜ arrival_reset"
    nav:   chips 中「等到点」bt-running；字段 "等到点 ｜ seq 2/2 ｜ signal ｜ path_ready"
    到点后: "等目标 ｜ seq 1/1 ｜ signal ｜ arrived ｜ stop"

单测：`node scripts/console/test_mission_events.js` → ALL PASS（新增
`bt nodes parsed and malformed dropped`、`planner mode and arrival flag parsed`）。

## 2. bag 录制栈（新增 `bag_record`）

- `scripts/console/stacks.json`：`bag_record`（master 11312，SIGINT 收尾）。
- 话题：`/rslidar_points /livox/imu /tf /tf_static /Odometry /LIO/odom_vehicle
  /planning/bspline /global_path /scan_planner/goal /scan_planner/cmd_vel
  /cmd_vel /v8/bt/status`；`--lz4 --split --size=512 --max-splits=8`。
- 输出：`data/bags/nav_<时间>_<卷>.bag`（目录由仓内 `data/bags/.gitkeep` 保证；
  `data/bags/*` 已入 .gitignore，不入库）。
- 实测（录 20 s 墙钟）：13,921,620 B 袋、`duration 12.1 s`、5225 条、
  lz4 17.48%（75.2 MB 原始）、类型含 PointCloud2/Imu/Odometry/Twist。
  停止走 agent 的 SIGINT→TERM→KILL，索引完整可 `rosbag info`。
- 已知：文件属主 root（容器写盘）；回放 `rosbag play --clock <bag>` 后按
  建图/导航链复跑；`/cloud_registered` 为派生量且大，默认不录。

## 3. web 实时建图（既有链路，实测复核）

- `voxel_stream` 栈订阅 `/cloud_registered`（回退 `/rslidar_points`），
  leaf 0.1 m，原子写 `data/console/voxels/{latest.bin,meta.json}`；
  agent `/api/voxels/{meta,latest}` 供页面拉取。
- 实测 meta：`source=/cloud_registered, frame=camera_init, count=471,
  bbox≈[-33,-29,-10, 34,20,9]`，latest.bin 7,589 B，0.4 s 内更新。
- 页面 3D 场景「数据源＝服务端体素」：显示 `服务端 407 点`、`leaf 0.1 m ·
  0.4 s · /cloud_registered`，与 m1 实体世界 8 box 叠加。

## 4. 验证用栈与清理

`scan_flat`、`voxel_stream` 起停均走 agent；验证后停止，容器无残留。
浏览器截图见本次会话（页面监控卡“BT 阶段”、3D 场景“服务端 407 点”）。
