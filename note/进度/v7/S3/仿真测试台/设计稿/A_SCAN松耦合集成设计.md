# A. SCAN-Planner 松耦合集成设计（v1.0 终稿，小张 2026-10-07）

## A1. 进程架构与话题契约（松耦合 = 只碰 3 条线）

SCAN 为**独立进程**，与 mbf 导航栈零代码耦合，仅通过话题对接：

| 方向 | 话题 | 类型 | 来源/去向 | 说明 |
|---|---|---|---|---|
| 入 | `/Odometry` | nav_msgs/Odometry | faster_lio | pose+twist，frame camera_init/body |
| 入 | `/cloud_registered_body` | sensor_msgs/PointCloud2 | faster_lio | 机体系稠密点云（lio_relay.launch 已验证该配对） |
| 入 | `/scan_planner/goal` | geometry_msgs/PoseStamped | 壳包 goal 分流 | SCAN 原 navi_mode=1 用 `/move_base_simple/goal`，**必须 remap 成私有名**（防与 mbf 双消费） |
| 出 | `/scan_trajectory`（bspline 采样 Path）+ `/position_cmd`/`cmd_vel` | — | SCAN→壳包跟踪层 | SCAN 原生输出；cmd_vel 由 closed_loop_controller 发 |
| 出 | `/cmd_vel`（经 mux） | geometry_msgs/Twist | 跟踪层→safe_cmd_vel_mux | mux 新增源 `scan_planner` |

内部链：`/cloud_registered_body → grid_map（概率占据+ESDF 膨胀）→ path_searching(A*)+bspline_opt → scan_replan_fsm（定时重规划）→ 轨迹`。ESDF 全部由 SCAN plan_env 内部从点云增量建，**不依赖 mbf 2D costmap、不依赖底图 PCD**（真机也不需要底图，这是它与 G8 定位栈解耦的关键）。

## A2. 轨迹跟踪层（关键缺口的三个方案）

| 方案 | 内容 | 取舍 |
|---|---|---|
| **T1 内置闭环（主选）** | SCAN 自带 `closed_loop_controller.cpp`（P 反馈跟踪→cmd_vel），run.launch `controller_mode=closed_loop` 已默认；`.bak-maxvel075/.bak-vy050` 显示 max_vel/vy 已调参史 | 零新代码。风险：参数按 Go2 仿真调的，Lite3 planar_move 下 max_vel/acc 需重调（壳包 yaml 覆写，不改源） |
| T2 自研 traj_server（备选） | bspline 采样点列 + pure pursuit → cmd_vel，壳包 scripts/traj_pure_pursuit.py（约 200 行） | 仅当 T1 跟踪发散（打滑/速度跳变 dev>0.35m）时启动，预登记不预施工 |
| T3 交 teb 跟踪（仅对比用） | SCAN 轨迹降维 nav_msgs/Path → mbf ExePath(teb) | teb 基于 2D costmap 重优化，丢失 3D 避障语义；只作 A/B 对比数据，**不作主路线** |

## A3. goal 接口与模式分流

- SCAN 进程 goal 话题 remap → `/scan_planner/goal`；mbf 维持 SMACH GetPath/ExePath 自有链路。
- 壳包 `scan_goal_relay.py`（≤60 行）：按参数 `nav_mode∈{mbf,scan}` 决定把 `/move_base_simple/goal`（RViz/任务层统一入口）转发给 `/scan_planner/goal` 或透传给 mbf 任务原语；默认 mbf（向后兼容）。
- M5/任务层：task_primitives.yaml 新增原语 `scan_navigate`（pub goal → 订阅 SCAN FSM reached/failed 状态）——**占位设计，施工在 SCAN Gazebo 集成完成后**。

## A4. 与 mbf 的分工与切换（C9 场景驱动）

- **分工**：2D 平地常规（maze2d/室外开阔/走廊）= mbf（GlobalPlanner/navfn + teb）；3D 复杂地形（maze3d/perlin3d 坡、真机台阶坡道）= SCAN 全链。SCAN **不进 mbf 插件**（mbf costmap 2D，3D 语义必丢；nav_core 适配只换入口不换代价地图，无意义）。
- **切换**：C9 variants 新增第 4 档 `scan_3d.yaml`（场景→跟踪器速度上限、ESDF 分辨率、mux 源使能）；mux 按 variants 档位使能 `scan_planner` 源（优先级低于 m6 安全源、与 teb 源互斥使能）。
- **回退**：scan_3d 档下 SCAN FSM failed 超时（10s 无轨迹）→ relay 自动发 2D 投影 goal 给 mbf 兜底（占位，先人工）。

## A5. 逻辑闭环图（全数据流）

```
Gazebo 雷达点云(仿真) / Livox(Mid360 真机)
   → faster_lio → /Odometry + /cloud_registered_body
      → SCAN 独立进程：grid_map 占据/ESDF → A*+bspline → scan_replan_fsm
         → /scan_trajectory → closed_loop_controller（跟踪）
            → cmd_vel → safe_cmd_vel_mux(源=scan) → M6 → /cmd_vel
               → Gazebo planar_move 伪狗运动（真机：M6→底盘）→ 传感器再采集（闭环）
RViz 2D Nav Goal / SMACH scan_navigate → /scan_planner/goal（分流后）
```

## A6. /home/test/scan_planner 复用清单（只读约束，双目录制）

| 类别 | 件 | 处置 |
|---|---|---|
| 原样拷 ws/src/30_3rd/ | planner/{plan_env,bspline_opt,path_searching,plan_manage,traj_utils}、simulator/Utils | 第三方包原样复制；先 /tmp 试编译预检（隔离红线） |
| 已在 ws 不重复拷 | mockamap（矩阵已实证在装 5 launch） | 复用 |
| 拷入壳包后改造 | run.launch → `lite3_scan/launch/70_scan.launch`：is_real_world 逻辑保留，sim 侧 body/cloud 源从 quad_0/pcl_render_node 换成 faster_lio 系；goal remap；robot_description 段删（我们已有 Lite3_prim.urdf） | 全部改动进壳包 |
| 直接拷 | lio_relay.launch（faster_lio 话题适配已验证）、真机安全网 remap 逻辑（/scan_planner/cmd_vel） | 逐字保留 |
| **不适用** | go2_kinematic_sim、go2_gait_publisher（Go2 四足运动学仿真——我们用 Gazebo planar_move 伪狗）；local_sensing/pcl_render_node（SCAN 自带深度仿真——我们用 Gazebo 真传感器链）；sensor_type=depth 全分支（Lite3 无深度相机）；ROS2 transfer 注释 | 不拷，壳包 launch 不引用 |
| 需核验 | advanced_param 中 max_vel/vy 参数基线值 → 迁出为壳包 scan_params.yaml | 参数外化 |

## A7. Gazebo 仿真验证方案（SCAN 进 mockamap 世界，不依赖真狗）

1. **环境**：95_sim.launch 起 maze2d 基线图 → 先回归；再 maze3d/perlin3d（SCAN 的主场）。
2. **链路**：Gazebo 仿真雷达 → faster_lio（M2 已闭环 teb 0.179m 基线不动）→ SCAN → T1 跟踪 → planar_move。SCAN 自带仿真闭环（smoke.sh）此前已验证，本次验证的是**换真传感器链后的松耦合闭环**。
3. **执行序**：P1 感知入（SCAN 订 faster_lio 双话题，rviz 看占据图收敛）→ P2 静态规划（发 goal，轨迹出且不穿墙）→ P3 动态闭环（跟踪走完）→ P4 重规划（丢 1 块动态障碍）。
4. **验证判据**：① grid_map 30s 内收敛且占据图与 world 一致（rviz 截图+点数）；② goal 后 5s 内 FSM 出轨迹、路径无穿墙（采样点查占据）；③ 到达判据 FSM reached，终点 odom 偏差 ≤0.35m（对齐 teb 基线门）；④ 全程 dt≤120s；⑤ 3 连全过（沿用「三连任一未过即停」红线）。
