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
