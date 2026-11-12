# v8_simulation

> 最新更新时间：2026-11-12。用途与机器边界见[工程标准](../../../../note/架构/工程标准.md)。

本包声明 v8 的仿真运行依赖并提供显式入口，只供 `x86_sim` 使用。公共算法包 `v8_platform` 的普通依赖不含 Gazebo；SCAN 的 Gazebo 回归依赖保留为测试依赖。目标构建入口和隔离目录见[构建说明](../../../../tooling/build/README.md)。

| 入口 | 实现来源 | 范围 |
| --- | --- | --- |
| `scripts/gazebo_carrier3d_node.py` | 本包自持（`scripts/carrier_kinematics.py`） | SetModelState 静态方盒运动学载体，无接触动力学 |
| `scripts/gazebo_truth.py` | 本包自持 | Gazebo 空世界和独立真值通道 |
| `scripts/mujoco_truth.py` | 本包自持 | 仓内 drop_world 的落体探针真值 |
| `launch/verification_gazebo.launch` | 本地入口 | Gazebo 空世界和独立真值通道 |
| `launch/verification_mujoco.launch` | 本地入口 | 仓内 drop_world 的落体探针真值 |

活动 carrier 和 verification manifest 指向本包，组件 ID、端口和参数语义保持。本包实现自持，**不依赖 `v8_platform` 源码**；`v8_platform` 通过组件 manifest 以 `v8_simulation` 入口调用本包（装配期引用，不复制实现）。`v8_platform/launch/verification_*.launch` 保留为平台侧验证入口，节点包名指向本包。

入口实现（含 `carrier_kinematics` 纯几何函数）已物理迁入本包 `scripts/`，不再经 `rospkg` 查找 `v8_platform` 源码；本包在 source/devel 与 install 两种工作区均可独立运行。`v8_platform` 不再保留仿真入口源码副本。

Gazebo/ROS 依赖在 `package.xml` 中声明；MuJoCo 入口还需仿真环境提供兼容的 Python `mujoco` 运行时及 XML/模型资产。该外部 Python 依赖当前不由 catkin manifest 或本包自动安装，应按[框架说明](../../90_platform/v8_platform/framework/README.md)和目标环境清单准备、记录版本。仓内没有训练器；落体探针不证明四足接触、策略训练或真机能力。

离线入口测试：

```bash
python3 -B -m unittest discover -s ws/src/95_simulation/v8_simulation/test -p 'test_*.py'
```

该测试验证薄入口的模块路径、ROS 参数与退出行为，不启动 ROS/Gazebo/MuJoCo。实际构建和仿真运行仍须在对应 Noetic 环境按作业单验证；本次没有新增运行或真机验收结论。
