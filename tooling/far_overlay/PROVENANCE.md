# FAR overlay 来源与依赖登记（2026-11-25）

> 本目录是**第三方件的独立 overlay**：只建 `far_planner` 与 `visibility_graph_msg`，
> **不进** `ws/src` 的 catkin 闭包（`tooling/build/profiles.json` 的 roots 里没有它，
> 也没有任何包 depend 它），因此对 support 后端零影响。

## 1. 上游与版本锁定

| 项 | 值 |
| --- | --- |
| 上游 | `MichaelFYang/far_planner`（分支 `melodic-noetic`） |
| 锁定 sha | `2799b696`（出处：`ws/src/90_platform/v8_platform/launch/far_stack.launch` 头注，2026-11-22 R4② 立） |
| 本地形态 | 无 `.git` 的源码副本（overlay 部署），故**改用内容指纹锁版本** |
| 内容指纹 | `src/` 下 42 个文件、`find … | sort | xargs sha256sum | sha256sum` ＝ `f57ad057a39876fbd8035e3d8074376f1bd9dd509bbfc023395e92cd0f814c0e` |
| 获取日期 | 2026-11-24（自 `/home/jack/far_overlay/src` 收进仓内） |

复核：`cd tooling/far_overlay && find src -type f | sort | xargs sha256sum | sha256sum`

## 2. 包与依赖

| 包 | 作用 | 依赖 |
| --- | --- | --- |
| `far_planner` | 3D 全局后端（可见图） | roscpp／std_msgs／sensor_msgs／visibility_graph_msg／pcl_ros |
| `visibility_graph_msg` | 可见图消息 | std_msgs／geometry_msgs |

★更正一处口径★：任务书写的是 "far_planner + far_planner_msgs"，但**上游没有 `far_planner_msgs`
这个包**（`find` 零命中，`package.xml` 里也没有该依赖）；真正的消息依赖是 `visibility_graph_msg`。
本 overlay 收的正是这两个包。

## 3. 独立构建

```bash
bash tooling/far_overlay/build.sh          # 内部 catkin_make --extend <ws>/devel
```

★`--extend` 不是可选项★：不加它，overlay 生成的 `setup.sh` 会把 `ROS_PACKAGE_PATH`
覆盖成「overlay + /opt/ros」，ws 的包全部消失；实测表现为 `rospack find v8_platform` not found、
`roslaunch v8_platform far_stack.launch` 报 "neither a launch file..."。

产物（git 忽略）：`tooling/far_overlay/{build,devel,install,logs}`。

## 4. 装配读数（2026-11-25 实测）

```bash
source /opt/ros/noetic/setup.bash
source <ws>/devel/setup.bash
source tooling/far_overlay/devel/setup.bash
export ROS_PACKAGE_PATH=<ws>/.profiles/x86_sim/src:tooling/far_overlay/src:$ROS_PACKAGE_PATH
roscore -p 11399 & roslaunch v8_platform far_stack.launch
```

| 项 | 读数 |
| --- | --- |
| 构建 | `catkin_make` rc=0，`devel/lib/far_planner/far_planner` 1.19 MB，error 计数 0 |
| 节点 | `/far_planner`、`/v8_global_far` 均起 |
| 订阅面 | `/odom_world`(Odometry)、`/goal_point`(PointStamped)、`/scan_cloud`(PointCloud2) 等，与 far_stack.launch 头注的六话题契约一致 |
| 日志 | `**************** STATIC ENV PLANNING ****************` ⇒ 地形/可见图初始化进入规划态 |

## 5. P2 验收结论

| 组合 | 结论 | 依据 |
| --- | --- | --- |
| FAR 装配（包可用性＋接线） | **PASS** | 上表：节点起、订阅面与契约逐条对上 |
| FAR 完整 P2 路径验收（trad_far / graph Gazebo+FAR） | **INVALID** | 缺 Gazebo 点云与 odom 前置：FAR 需要持续 `/odom_world`＋`/terrain_cloud`＋`/scan_cloud` 才能出航点，本容器未起 Gazebo，`/global_path` 无产出。**不据此声称 FAR 规划能力成立** |

复跑（待 Gazebo 前置就绪）：

```bash
roscore -p 11399 &
roslaunch lite3_gazebo_sim m1_gazebo.launch          # 点云 + odom
roslaunch v8_platform far_stack.launch               # FAR 后端
python3 scripts/verify/p2_runtime_probe.py --combo trad_far \
  --evidence-dir <ev> --goal 1.0 0.0 --duration 20 --planner-goal-topic /v8/global/goal
```
