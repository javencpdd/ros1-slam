# Lite3 3D app · 配置模板与现场边界

> 最新更新时间：2026-11-06 14:30:30 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../../note/进度/README.md)为准。

本目录保存 Lite3 现场装配模板。它说明当前仓内配置，不表示机器人此刻开/关机，也不新增真机运动许可或验收。既有静态/容器冒烟和现场记录见[进度入口](../../../../../../note/进度/v8/README.md)。

## 文件与本机资产

| 文件 | 当前内容 |
| --- | --- |
| [lite3_real_3d.app.yaml](lite3_real_3d.app.yaml) | navigation_app/v1；map=lite3_real@4，backend=lite3，carrier=TF，引用本机 registry/task |
| [graph_3d_lite3.yaml](graph_3d_lite3.yaml) | map、global、path_goal、local、ultrasonic_guard、mux、perception、gait_guard、carrier 共9实例 |
| [local_native.yaml](local_native.yaml) | 局部搜索、包络/能力、心跳、话题与墙钟兜底参数 |

模板要求每机使用被忽略的 `lite3_real_3d.local.app.yaml`；地图/registry/task 在 `data/maps/real_lite3_20261103/`，不入 Git，各主机可能不同。app 与 graph 显式声明 `execution_mode: real_robot`、`robot_id: lite3`；旧本机覆盖若缺这两字段，真实入口会拒绝，不会从 backend 或 --real 推断。复制 app 后按本机资产改 map_id/version/路径，不能照旧指标推断文件现状。统一 registry 模板见[registry.template.json](../../../../../../data/maps/registry.template.json)。

shell 环境由 `scripts/env.sh` 导出 ROS1_SLAM_ROOT，可读本机 local.env；Python [paths.py](../../framework/paths.py) 优先用已导出的环境变量，再从仓目录标识/git 回退推导仓根，不直接加载 shell env 文件。仓库位置不能硬编码为某台机器路径。

## 当前数据与命令接线

```text
map → global Path → path_goal（末点）→ local goal
TF/odom → carrier pose/localization ────→ local
点云 → perception obstacles ──────────→ local
local plain Twist → /cmd_vel_raw → safe_cmd_vel_mux → /cmd_vel
前超声 → speed_cap ───────────────────→ mux
map + global Path + carrier pose → gait_guard → /v8/chassis/gait
```

perception 已使用 dynamic_obstacles，订阅 `/rslidar_points`，结合位姿/TF做 ROI、分箱与跟踪；不是旧的固定空障碍心跳。`blind_timeout_s=2.0` 时无新点云停止障碍心跳，随后局部输入时效门归零；空表/成功订阅也不能证明所有障碍已被看见。

生产 mux 是唯一 `/cmd_vel` 写者；local 的 `cmd_vel_3d` 仍为map三维接口，plain Twist 进入 navigation 源 `/cmd_vel_raw`，按有效位姿yaw把 map XY 显式转到同一base_link原点的航向平面。非法姿态/源时间/速度或非零竖直请求使本周期两路归零；yaw=π/2时 map(1,0)→body(0,-1)已有离线适配器正反控。航向平面不是完整6D底盘控制；carrier仍未补支撑高度偏移，base_link/支撑高度与现场运动语义尚须独立核实，不能称为已完整真机适配。

carrier保留TF/odom的原非零源stamp；frame/child必须匹配配置map/base_link，topic输入若是odom坐标直接拒绝，不改标签冒充变换。零/旧/未来/倒退源戳及停钟时缺少新源推进均使健康false，不重播无效旧pose。TF查询不等待ROS时间超时，20Hz节拍按单调墙钟；源年龄和墙钟进展年龄任一达到既有stale_s=0.5即失效。local再独立核源时间与既有input_max_age_s=1.0；阶段仅做代码/离线验证，不表示现场TF链或停止距离已验。

## 配置值与来源边界

local_native.yaml 当前速度上限0.5 m/s、半径0.35 m、高0.436 m、台阶0.15 m、坡度40°、clearance0.10 m、support_tol0.05 m。这些是配置采用值；手册几何/声明能力不能代替装载后实测，也不是 tested/allowed robot profile 的自动证明。

[生产mux配置](../../../../60_safety/lite3_safety/config/cmd_vel_mux.yaml) 当前恒20 Hz，线性上限0.5 m/s、角速度1.0 rad/s；cap required=true，cap过期退到0.1 m/s。它没有把超声缺失解释为正常全速，也不是所有失效均停车：超声缺失的现行策略是低速限制。

图中超声前方 index=0：≤0.3 m cap0，≤0.8 m cap0.1，≥1.2 m恢复0.5（迟滞），过期按 fail_cap 限到0.1。gait_guard 前视1.5 m、进入1.0 m/退出1.5 m，超出声明台阶上限拒绝；它产生步态请求，不证明底盘已执行。

[mode_navigation.launch](../../../../70_bringup/lite3_bringup/launch/mode_navigation.launch) 的 enable_motion 缺省false；只在显式打开时启动 ros2qnx。ros2qnx 节点自身 guard 缺省关闭，而该 launch 的 chassis_guard 缺省true并传入节点，两者不可混为“所有入口默认开/关”。现场服务和厂商链行为须以目标机原件与授权窗为准。

## 离线建图与任务生成

PCD→support_map 的工具真身为[map_from_pcd.py](../../framework/map_from_pcd.py)。v3 的 cells 保留实际格集，避免 bbox 空角冒充接触；障碍层可选 surfaces/full/both，z-align 与近地面吸收会改变地图内容，必须生成新版本并更新 registry/task。

下面是仓根的本机示例，输出写到 /tmp，不回写现有地图：

```bash
python3 -B ws/src/90_platform/v8_platform/framework/map_from_pcd.py \
  --pcd data/maps/real_lite3_20261103/lite3.pcd --map-id lite3_real --version 4 \
  --resolution 0.15 --voxel 0.03 --z-min -0.30 --z-max 2.50 \
  --surface-z-min -0.20 --surface-z-max 0.15 --obstacle-min-z 0.15 \
  --min-cells 4 --min-side 0.25 --min-area 0.50 --xy-gap 0.20 \
  --obstacle-mode full --obstacle-cell 0.20 --obstacle-min-points 5 \
  --obstacle-band '0.02,2.50' --z-align --absorb-near-floor 0.08 --goal-max 1.5 \
  --out /tmp/lite3_real_v4.json
```

重新指定起点而不重建地图可用 `--task-only --map /absolute/map.json --start-x X --start-y Y --goal-max 1.5 --out /tmp/task.json`，map_id/version须与输入一致。`--write-app`/`--register` 会生成 app 或修改本机登记，按目标文件显式使用。静态图导出、任务选点或局部自报arrived不能替代实际载体到达。

## 预检与运行入口

```bash
python3 -B ws/src/90_platform/v8_platform/framework/cli.py check \
  ws/src/90_platform/v8_platform/framework/fixtures/manifests \
  ws/src/90_platform/v8_platform/configs/lite3/graph_3d_lite3.yaml
python3 -B ws/src/90_platform/v8_platform/framework/app_runner.py \
  --app ws/src/90_platform/v8_platform/configs/lite3/lite3_real_3d.local.app.yaml
```

第二条是 dry-run，要求本机 app/地图齐备。Lite3 模板不允许用 `--execute --no-gazebo` 当作仿真图；容器仿真须选择显式 simulation app/载体。已授权现场使用 `--execute --real` 沿用现场 master，Gazebo/GT/错误机器人与未知后端会在任何进程前拒绝。启动前必须核唯一写者、hold、位姿时效、障碍/超声心跳与运动桥边界，实际运行按[作业单](../../../../../../note/计划书v8/25_v8自动调度作业单.md)。本次完成速度坐标与TF源时效的代码/离线守卫验证，未连接机器人；现场TF、支撑参考点、设备协议与停止时距继续待目标环境验证。
