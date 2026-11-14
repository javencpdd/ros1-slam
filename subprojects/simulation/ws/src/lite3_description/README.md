# lite3_description · Lite3 几何描述

> 最新更新时间：2026-11-06 15:17:09 CST。现行规划见仓库[计划书 v8.1](../../../../note/计划书v8.1/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包保存 Lite3 的 URDF 和四种 STL 网格，供机器人模型展示与描述引用。

| 文件 | 内容 |
| --- | --- |
| `urdf/Lite3.urdf` | 躯干和四足模型，12 个 revolute 关节 |
| `meshes/{torso,hip,thigh,shank}.STL` | 四类原始网格 |

描述件本身没有步态控制器或接触状态估计。Gazebo M1/M2 的实际载体是
`lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro`；它与本包带腿视觉模型不是同一个执行对象。
MuJoCo 接触物理 runner 使用独立外部 MJCF/mesh 资产，见
[`dynamics`](../lite3_gazebo_sim/dynamics/README.md)，也不能靠 URDF 关节数认定爬阶能力。

URDF/网格按其来源规则保持原件。运行和验收结论见进度证据，不在本目录重复积累。
