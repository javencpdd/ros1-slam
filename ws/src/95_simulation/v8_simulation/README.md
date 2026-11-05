# v8_simulation

> 最新更新时间：2026-11-06。用途与机器边界见[工程标准](../../../../note/架构/工程标准.md)。

本包声明 v8 的仿真运行依赖并提供显式入口，只供 `x86_sim` 使用。公共算法包 `v8_platform` 的普通依赖不含 Gazebo；SCAN 的 Gazebo 回归依赖保留为测试依赖。目标构建入口和隔离目录见[构建说明](../../../../tooling/build/README.md)。

| 入口 | 实现来源 | 范围 |
| --- | --- | --- |
| `gazebo_carrier3d_node.py` | `v8_platform/framework/gazebo_carrier3d_node.py` | SetModelState 静态方盒运动学载体，无接触动力学 |
| `launch/verification_gazebo.launch` | `gazebo_truth.py` → `v8_platform/gazebo_truth.py` | Gazebo 空世界和独立真值通道 |
| `launch/verification_mujoco.launch` | `mujoco_truth.py` → `v8_platform/mujoco_truth.py` | 仓内 drop_world 的落体探针真值 |

活动 carrier 和 verification manifest 已指向本包，组件 ID、端口和参数语义保持。`v8_platform` 内原有同名 launch、源码和定标夹具继续作为仿真测试/兼容入口使用；应在已具备全部仿真依赖的测试环境执行，不属于机器人运行入口。

当前薄入口通过 `rospkg` 查找 `v8_platform` 源码，保留 ROS 命令参数及兄弟模块导入；不复制算法实现，也不移动离线点云工具复用的纯几何函数。`v8_platform` 尚不安装其 Python 源码，因此本切片支持 source/devel 工作空间；只有 install 产物而没有原源码时会明确报错。后续物理拆包和 install-only 支持须独立实施与验证。

Gazebo/ROS 依赖在 `package.xml` 中声明；MuJoCo 入口还需仿真环境提供兼容的 Python `mujoco` 运行时及 XML/模型资产。该外部 Python 依赖当前不由 catkin manifest 或本包自动安装，应按[框架说明](../../90_platform/v8_platform/framework/README.md)和目标环境清单准备、记录版本。仓内没有训练器；落体探针不证明四足接触、策略训练或真机能力。

离线入口测试：

```bash
python3 -B -m unittest discover -s ws/src/95_simulation/v8_simulation/test -p 'test_*.py'
```

该测试验证薄入口的模块路径、ROS 参数与退出行为，不启动 ROS/Gazebo/MuJoCo。实际构建和仿真运行仍须在对应 Noetic 环境按作业单验证；本次没有新增运行或真机验收结论。
