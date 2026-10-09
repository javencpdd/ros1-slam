# lite3_description（DESC 机体描述 URDF/mesh）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `95_simulation` ｜ 角色 `DESC 机体描述（URDF/mesh）` ｜ 厂商机体几何描述的沙箱副本：URDF＋STL，供 xacro 引用，不含控制器。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 带腿描述 | `urdf/Lite3.urdf`（12 个 revolute 关节） |
| mesh | `meshes/{torso,thigh,shank,hip}.STL`（共 4 件） |

**红线/契约**：**注意**：仿真闭环当前**不**用它驱动——真用的是 `95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro`（刚体）。带腿＝越障位置⑤的前提，缺控制器与接触标定（补丁 X7）。

## 说明

本 README 是新建件（文档批次）：包内此前无 README，内容以上表为准；进度与证据不在本目录，见 `note/进度/`。
