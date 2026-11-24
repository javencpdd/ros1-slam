# FAR overlay 来源与依赖登记（2026-11-25）

> 本目录是**第三方件的独立 overlay**：只建 `far_planner` 与 `visibility_graph_msg`，
> **不进** `ws/src` 的 catkin 闭包（`tooling/build/profiles.json` 的 roots 里没有它，
> 也没有任何包 depend 它），因此对 support 后端零影响。

## 1. 上游与版本锁定（2026-11-25 补齐）

| 项 | 值 |
| --- | --- |
| 上游 | `MichaelFYang/far_planner`，分支 `melodic-noetic` |
| 完整 SHA | **`2799b6964c141cacd1c32a14b19bc7abffbe0e52`**（2025-03-31，作者 Ji Zhang） |
| 获取方式 | `curl codeload.github.com/MichaelFYang/far_planner/tar.gz/2799b696…`，解包后按包收入 |
| 内容指纹 | `src/` 42 件：`f57ad057a39876fbd8035e3d8074376f1bd9dd509bbfc023395e92cd0f814c0e` |
| 许可证 | `BSD`（`far_planner/package.xml` 的 `<license>`） |
| 版权 | `Copyright (C) 2021 Fan Yang - All rights reserved`（`src/*.cpp` 头注）；维护者 `Fan Yang <fanyang2@alumni.cmu.edu>` |
| ★许可文件缺失如实登记★ | 上游仓库**没有 LICENSE 文件**（根目录只有 `.gitignore`/`img`/`README.md`/`src`）；许可声明仅见于 `package.xml` 与源码头注。本仓不代拟许可证全文 |

### 1.1 源码指纹验证（可复跑）

```bash
curl -sSL https://codeload.github.com/MichaelFYang/far_planner/tar.gz/2799b6964c141cacd1c32a14b19bc7abffbe0e52 -o /tmp/up.tar.gz
tar -xzf /tmp/up.tar.gz -C /tmp && cd /tmp/far_planner-2799b6964c141cacd1c32a14b19bc7abffbe0e52
# 与本仓 tooling/far_overlay/src/<pkg> 逐文件 sha256 对表
```

实测结果（2026-11-25）：`far_planner` **38/38 件字节相同**、`visibility_graph_msg` **4/4 件字节相同**，
无「仅上游」、无「仅本地」——即仓内副本与固定 SHA 的上游**逐字节一致**。

### 1.2 上游还有哪些包（本 overlay 未收，理由）

上游 `src/` 下共 6 个包：`far_planner`、`visibility_graph_msg`、`boundary_handler`、`graph_decoder`、
`goalpoint_rviz_plugin`、`teleop_rviz_plugin`。本 overlay 只收前两个：
`far_planner/package.xml` 的依赖里没有后四个（后四个是可视化/遥控/边界处理的配套件），
`far_stack.launch` 也不起它们。**未收 ≠ 遗漏**，是按依赖闭合裁剪。

## 2. 包与依赖

| 包 | 作用 | 依赖（取自 package.xml） |
| --- | --- | --- |
| `far_planner` | 3D 全局后端（可见图） | roscpp／std_msgs／sensor_msgs／visibility_graph_msg／pcl_ros |
| `visibility_graph_msg` | 可见图消息 | std_msgs／geometry_msgs |

★更正一处口径★：任务书写的是 "far_planner + far_planner_msgs"，但**上游没有 `far_planner_msgs`
这个包**（`find` 零命中，`package.xml` 里也没有该依赖）；真正的消息依赖是 `visibility_graph_msg`。

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

## 5. 运行期环境接缝（2026-11-25 实测，必须照做）

独立 overlay 的 `setup.bash` 会**覆盖**两处路径，导致 ws 侧节点或 FAR 节点找不到：

| 症状 | 原因 | 正解 |
| --- | --- | --- |
| `rospack find v8_platform` not found、roslaunch 说 far_stack.launch 不是 launch 文件 | overlay setup 把 `ROS_PACKAGE_PATH` 覆盖成「overlay + /opt/ros」 | 补 `ROS_PACKAGE_PATH=<ws>/.profiles/x86_sim/src:<overlay>/src:$ROS_PACKAGE_PATH` |
| `Cannot locate node of type [bt_console]` | `CMAKE_PREFIX_PATH` 同样被覆盖，ws 的 C++ 可执行找不到 | 补 `CMAKE_PREFIX_PATH=<ws>/devel:<ws>/.profiles/x86_sim/devel:$CMAKE_PREFIX_PATH` |
| `multiple files named [far_stack.launch] in package [v8_platform]` | 同一条路径在 `ROS_PACKAGE_PATH` 里出现两次 | 不要把 profile src 重复 prepend |

固定顺序：

```bash
source /opt/ros/noetic/setup.bash
source <ws>/devel/setup.bash
source tooling/far_overlay/devel/setup.bash          # 会覆盖上面两处
export ROS_PACKAGE_PATH=<ws>/.profiles/x86_sim/src:tooling/far_overlay/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=<ws>/devel:<ws>/.profiles/x86_sim/devel:$CMAKE_PREFIX_PATH
```

## 6. P2 验收结论（2026-11-25 实测）

环境：`roscore` + `roslaunch lite3_gazebo_sim m1_gazebo.launch`（点云 10 Hz、TF map→base_link 通）
+ 渲染后的 `graph_3d_gazebo_far.yaml`（14 节点全起，含 `/far_planner`）。

| 检查项 | 判定 | 读数／证据 |
| --- | --- | --- |
| 包可用性＋装配接线 | **PASS** | `/far_planner`、`/v8_global_far` 均起；FAR 订阅 `/odom_world`／`/goal_point`／`/scan_cloud` |
| FAR 规划在动 | **PASS** | 日志：`Adding Goal to V-Graph`、`Path Search Time: 0.187ms`、`global vertices: 35` |
| 有效路径 | **PASS** | 探针 path 事件 `{"n": 2, "end_x": 1.0, "end_y": 0.0}`：两点契约、终点＝FAR 航点 |
| 取消 | **PASS** | `cancel_sent=1`、`cancel_ack=1` |
| 运动（非零速度） | **INVALID** | `/v8/local/cmd_vel_3d` 的 twist 恒为 0；mux 状态 `reason=track`、`hold=false` ⇒ **不是 mux 挡的**，是 local 本身输出零；探针 `nonzero_cmd_samples=0` |
| BT 到达 | **INVALID** | 车未动，无位移可言（依附上一行） |
| 障碍安全（运动中） | **INVALID** | 无运动，安全行为不可观测 |
| 定位失效停止 | **INVALID** | 运行中栈退出（`/clock` 中断），未能取得失效前后对照 |

★不据上表声称 FAR 具备 3D 导航能力★：只证明「包可用、接线对、图在规划、路径两点有效、取消闭环」。
运动与到达的阻断点在 `local` 侧（输出恒零），与本 overlay 无关，属 v8 local_native 侧待查项。
