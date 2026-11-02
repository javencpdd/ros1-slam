# NDT 定位监测器

本包将散落在临时工作区的自研 NDT 包迁入源码面，默认且仅允许
`output_role=monitor`，不承担连续定位输出。R787 的 LIO 单后端冻结范围与
0.22 m 门值保持原样；本包未获两类后端精度可切换验收。

## 来源与变更

来源根目录：`/home/jack/temp/ndt_priv/src/v8_ndt_odom/`，只读复制，原件未修改。

| 来源相对路径 | 原始 sha256 | 仓内处理 |
| --- | --- | --- |
| `src/ndt_odom_node.cpp` | `9c6b0963f12264f9f99d7ff363e3a5ebf2a12008fb03a5fcb8bbaa88ba6cd903` | 修复预测历史、拒帧新鲜度，新增显式 reset 与 monitor 边界 |
| `src/ndt_score_probe.cpp` | `fbb4d460a68834980eaff388e3bacd601e92036bee2de9d76a0623e77d991aaf` | 原样迁入，仅供测量，不代表 score 门值已定标 |
| `CMakeLists.txt` | `c86e4c052be4f5ae8d869699e992b538d551ef444866497e1f18a7c41eb61348` | 依赖改为 catkin 导出，移除临时区/固定构建目录硬编码，新增独立变换测试 |
| `package.xml` | `dd78497dccf653104d71a6ce3c5937b0677c7d7ada1f59b6f13b632f579c1c02` | 声明 std_msgs/std_srvs/ndt_omp 与 monitor 用途 |

迁入原件共 14,473 字节，属于项目自研代码；`ndt_omp` 第三方包未修改。

旧实现每帧末先更新当前位姿，再令 previous 等于 current，所以下帧预测的相对
变换恒等。`PoseHistory` 在提交当前位姿之前保留上一已接受位姿，只有接受帧
推进历史与时间戳。两帧 x=0、0.1 的预测应为 x=0.2，而非 x=0.1。

拒绝配准时发布 `~healthy=false`，不再以新雷达时间戳重发旧 Odometry。
接受帧的协方差未定标，以大对角线显式保守表示，不再用全零暗示确定性。
`~reset`（std_srvs/Trigger）撤销监测结果、清历史、清拒绝计数，从配置初值
重新开始；开启 `global_search` 时允许下帧重新进入搜索。网格全局搜索仍无
单次调用硬时限，不能据本改动声称已满足“2 秒内全局重定位”。

## 接口与验证

`map_path` 必须显式传入，输入由 `~cloud_topic` 指定（缺省 `/rslidar_points`）。
所有输出位于节点私有命名空间：`~odom`、`~score`、`~debug`、`~healthy`。
本节点不发布 TF，不接管现有 LIO 的连续输出。旧 `/ndt_odom` 订阅者须显式
重映射，不会通过迁入自动替换现有定位源。

独立的两帧变换、拒绝帧不更新、新旧时戳与 reset 测试不依赖 ROS 或 NDT：

```bash
g++ -std=c++14 -I/usr/include/eigen3 -Iinclude test/history_test.cpp -o /tmp/ndt_history_test
/tmp/ndt_history_test
```

ROS 构建使用私有 catkin 工作区并消费已安装的 Noetic/ndt_omp 导出，不复制
第三方源码，不复用历史私有 `build/devel`。真实点云精度、失效监视正控及全局
重锁时限需要另外取目标环境证据；不得把这里的单测作为两后端精度验收。

## A6 重定位接口（R1106／R1114）

新增三个接口，仍只允许 `output_role=monitor`：

- `~relocalize`（`std_srvs/Trigger`）：请求一次**有界**短时失锁重定位。服务只负责
  置位，不阻塞等待搜索完成；成功后 `~relocalized=true`，失败/预算耗尽保持 false。
- `~initialpose`（`geometry_msgs/PoseWithCovarianceStamped`）：把外部候选位姿作为
  局部跟踪种子；种子本身不算健康观测，必须等下一帧配准被接受。
- `~relocalized`（`std_msgs/Bool`）：`false` = 尚未在本次请求后验收，`true` =
  已发布一帧由重定位搜索直接产生的观测。

重定位语义（刻意保守）：

1. 搜索中心＝最后一次接受的监控位姿，范围由 `relocalize_radius_m`（默认 .5）、
   `relocalize_xy_step_m`（.2）、`relocalize_yaw_span_deg`（10）、
   `relocalize_yaw_step_deg`（5）定义；**不**做全图全局搜索。
2. `relocalize_budget_s`（8 s）内必须跑完整张网格；预算耗尽即失败，不接受未跑完的
   当前最优。`relocalize_score_max` 默认沿用 `score_max`，可单独收紧。
3. 候选先按 `relocalize_score_max` 过滤，再按到中心的位移（加小权重 yaw）取**最近**
   的可接受假设，而不是取全场最低 fitness；结果还必须落在
   `relocalize_max_displacement_m`（默认 radius＋step）与 yaw 界内。理由是 NDT 在
   走廊场景存在更低 fitness 的错误盆地（R902—R905），单看 fitness 会选错。
4. 搜索成功后直接发布该搜索位姿为观测，不再用局部跟踪器二次精修；避免精修把
   已验证的候选拉进更差的盆地。搜索失败则撤销健康、拒绝后续局部观测直到新的
   `relocalize`/`initialpose`。

边界：该接口只覆盖**短时失锁、先验附近的静态重定位**；不做 kidnapped/长时失联的
全图重定位，也不把本监控器提升为连续导航后端。R1106 的 Gazebo 活体只在静态位姿
验证了“失锁→有界搜索→A6 候选→`~initialpose`→新 lease→租约门恢复”（最终 XY
`.052642 m`、候选 `.057400 m`；失锁期 gate `.1→0`；空扫描 FAILED、lease 2→2）。
R1114 复跑静态 **.048655 / .044988 m**，0.3 m 传送后两跑终态 **.043754 / .060690 m**
（候选仍停在 prior `.242—.250 m`，靠恢复后跟踪回正）；**0.6 m 传送超出 0.4 m
搜索半径时候选/终态偏 `.5421/.5225 m`，A6 仍恢复 lease、gate 报 HEALTHY——系统级
fail-open 已登记、未修**。运动中的连续跟踪与任意 kidnapping 未通过，需独立运动先验
或恢复后一致性检查（新的安全决策）。
