# 网页 Foxglove 双模（2026-11-12）

- 页面：`http://127.0.0.1/web/console.html` 新增「可视化后端」卡片：
  `网页内建`（默认）/ `Foxglove（关本页重载）`，选择存入 localStorage。
- Foxglove 模式做的事（切换时）：
  1. 停服务端体素流：`POST /api/stacks/voxel_stream/stop`（实测由 running→stopped）；
  2. 停浏览器实时点云订阅与服务端体素轮询；
  3. 隐藏 3D/点云块（`#cloud-wrap`）；
  4. 退订重载监视话题（`/tf`、`/cmd_vel`、位姿、costmap 等），**只留
     `/v8/bt/status`** 驱动 BT 阶段条；
  5. 显示连接指引：`ws://<host>:9090`、Foxglove 里选 **Rosbridge (ROS 1)**
     （不是 Foxglove WebSocket）、布局模板链接
     `/docker/foxglove/ros1_slam_sim_layout.json`（HTTP 200）、域一致性提醒。
- 切回网页内建：恢复 3D 块并重新订阅（体素流需到作业栈手动重启）。

## 实测读数（浏览器）

    切前: mode=web, guide hidden=true, cloud hidden=false
    切后: mode=foxglove, guide hidden=false, cloud hidden=true,
          ws=ws://127.0.0.1:9090,
          log="已切到 Foxglove：停服务端体素流/实时点云/3D 与重载订阅
               （保留 /v8/bt/status）；rosbridge 保持给 Foxglove 用"
    agent: voxel_stream running → stopped（由页面触发）
    刷新后: mode=foxglove 保持、guide 可见、cloud 隐藏；/tf 实测为 "—"
            （重载订阅被跳过）；BT 阶段条仍更新（等目标 ｜ seq 0/0 ｜ signal）
    切回: guide hidden=true, cloud hidden=false,
          log="已切回网页内建可视化；体素流如需请到作业栈重启 voxel_stream"

单测：`node scripts/console/test_mission_events.js` ALL PASS（新增
`console.html has viz backend switch and Foxglove guide`、
`console.js stops web-side load when Foxglove mode is chosen`）。

## 边界

- **rosbridge 不能关**：Foxglove 走的就是它（`ws://<host>:9090`）；本镜像
  未装 `foxglove_bridge`。要改用 Foxglove WebSocket 需另装桥，届时才可停 rosbridge。
- 桥必须连到目标 ROS 图：当前连控制台域 11312；真机换桥的 master，见
  `docker/foxglove/README.md`。
- 布局模板是手写件（`/map_snapshot` + TF + `/Odometry`）；SCAN 栈需手动加
  `/cloud_registered`、`/global_path`、`/planning/bspline`。
- 本页在 Foxglove 模式下仍负责：发目标、BT 阶段、作业栈、监控、遥控/急停。

## 编排阶段 3D 标记 + SCAN 布局（2026-11-12 补）

- 新增 `lite3_bt/scripts/bt_stage_marker.py`（只读）：订阅 `/v8/bt/status` +
  `/v8/local/pose`，发布 `/v8/bt/stage_markers`（MarkerArray，TEXT_VIEW_FACING，
  1.5 s lifetime，跟随机器人上方 1.6/1.18 m）。挂在 `scan_bt_nav.launch`。
- 实测（scan_flat，goal (-1.5,2.0)，逐 4 s 采样两条文本）：

      idle:  "WAIT_GOAL"      ｜ "seq 0/0 running arrival_reset"
      nav:   "WAIT_ARRIVAL"   ｜ "seq 1/1 ready path_ready"
      到点:  "DWELL_AT_GOAL"  ｜ "seq 1/1 ready reached"
      收口:  "WAIT_GOAL"      ｜ "seq 1/1 ready reached"

- 新布局 `docker/foxglove/ros1_slam_scan_layout.json`：3D 面板含
  `/cloud_registered`、`/global_path`、`/v8/bt/stage_markers`、`/tf`、
  `/Odometry`；右栏 Raw Messages 看 `/v8/bt/status`。
- 单测 `lite3_bt/test/test_bt_stage_marker.py` 5/5 PASS。
- 边界：Foxglove 里树的分支结构仍不可见（Raw Messages 是 JSON）；
  完整树形图用 Groot2（ZMQ 5555）。
