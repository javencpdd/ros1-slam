# 三仓物理拆分迁移设计（2026-11-14）

> 目标（真人 2026-11-14 指令）：仿真 / 可视化成为**独立子项目**——物理分仓、各自独立 catkin 工作区、独立发布物。P1 组件化导航平台仍为主仓与集成/账本层。
> 现状依据：[current-2026-11-13.md](current-2026-11-13.md)；解耦守卫 `ws/src/90_platform/v8_platform/verify/test_decoupling.py`；耦合面实测 119 文件引用 `95_simulation`。
> **历史策略**：两个新仓采用**新历史 + 来源提交说明**（首提交记录来源仓/路径/HEAD），P1 历史与 `note/` 的 code_ref 链**不改写、不丢失**。不做 subtree/filter-repo 重写。

## 1. 目标拓扑

```
/home/jack/ysc-code/
├── ros1-slam/                     # P1 平台仓（主）：平台 + 集成/验证 + 账本
│   ├── ws/                        #   P1 独立 catkin 工作区
│   ├── docker/ scripts/ note/ data/ configs/ tooling/ web/
├── ros1-slam-simulation/          # P2 仿真仓（新）
│   ├── ws/{src,build,devel,install}   # P2 独立 catkin 工作区
│   ├── ws/src/{v8_simulation,lite3_gazebo_sim,lite3_description,
│   │           local_sensing,map_generator,mockamap,Mid360_simulation_plugin}
│   ├── config/ tools/             # 原 95_simulation/config、tools
│   └── Makefile scripts/ dist/    # 独立构建与发布物
└── ros1-slam-console/             # P3 可视化仓（新，host-only）
    ├── console/{agent.py,ui,stacks.json,world_registry.json,voxel_stream.py}
    ├── console/tests/             # 只读契约/体素/任务事件
    ├── ops/{build_maps_index.py,build_world_scenes.py}
    └── Makefile dist/
```

## 2. 文件归属

| 归属 | 内容 | 不持有 |
| --- | --- | --- |
| P1 | `ws/src/{00_drivers,10_slam,20_localization,30_3rd,30_perception,40_navigation,50_task,60_safety,70_bringup,90_platform,90_supervisor}`、`docker/`、`scripts/{agent,build,verify,ops,sim,mapping}`、`note/`、`data/`、`configs/`、`tooling/`、`web/` | 仿真包、任务台 UI |
| P2 | `ws/src/95_simulation` 的 7 个包 + `config/` + `tools/` | 平台装配、BT、控制台 |
| P3 | `scripts/console/` 全部 + 两个前端资产生成器 | ROS 写入、formal/真机启停 |

数据：`data/maps`、`data/bags`、`data/console` 留在 P1（host-only 数据根）；P3 经 `V8_REPO_ROOT` 只读/写入既有子目录。`web/` 留在 P1（文档面），本轮不动（含 codex 未提交改动）。

## 3. 跨仓契约

1. **P1→P2 装配期**（不变）：manifest `launch.package: v8_simulation`、app `simulation_declaration`。
2. **运行时发现**：sim/sim-viz 容器挂载 `../../ros1-slam-simulation/ws → /opt/ros1-slam/sim_ws`；入口脚本 source `/opt/ros1-slam/sim_ws/devel/setup.bash`。`roslaunch <pkg>` 经 `ROS_PACKAGE_PATH` 解析，包名不变。
3. **P2→P1**：仅 `exec_depend`（`lite3_drivers`/`faster_lio`/`lite3_slam` 等），构建期不需要 P1 源码；运行期由 P1 安装空间提供。场景脚本 `scene.py` 的 `PLATFORM` 改为 `V8_PLATFORM_ROOT` 环境变量（默认同级 `../ros1-slam/ws/src/90_platform/v8_platform`），属集成期依赖。
4. **P3→运行域**：HTTP/rosbridge/docker exec 只读；仓根用 `V8_REPO_ROOT`（默认同级 `../ros1-slam`），容器根用 `V8_CONTAINER_ROOT`（默认 `/opt/ros1-slam`），foxglove 目录与数据根可覆盖。不改任何 ROS 写入路径。
5. **集成/守卫**：`scripts/sim/two_floor_nav.py`、`scripts/verify/`、`verify/test_decoupling.py` 留在 P1，用 `V8_SIM_ROOT`/`V8_CONSOLE_ROOT` 定位子项目；守卫改为断言**跨仓 import/包依赖为零**，而不是断言同仓目录存在。

## 4. 受影响路径（迁移时逐项核）

- 容器内绝对世界路径：`scripts/console/stacks.json` 的 `/opt/ros1-slam/ws/src/95_simulation/...` → `/opt/ros1-slam/sim_ws/src/...`。
- 宿主测试/工具：`verify/test_decoupling.py`、`scripts/console/test_agent_contract.py`、`scripts/ops/build_world_scenes.py`、`scripts/build/target_workspace.py`（x86_sim 目标需合并 P2 源根）、`framework/tests/test_sim_backends.py`、`framework/tests/test_framework.py`、`registry/data/provenance_core.json`。
- 文档：`README.md`、`ws/README.md`、`docker/README.md`、`framework/README.md`、`lite3_gazebo_sim/README.md`（随仓搬走）。

## 5. 迁移切片

| 片 | 交付 | 验收 |
| --- | --- | --- |
| S0 | 本设计 + 任务卡 `T-20261114-02` | 设计入库；来源 HEAD 记录 |
| S1 | P2 仓落地（复制 + 独立 `ws` + Makefile + README + `.gitignore` + 首提交） | 容器内 `catkin_make` 通过；`v8_simulation` 单测过 |
| S2 | P2 接线：compose 挂 `sim_ws`、入口 source、`stacks.json` 世界路径、`scene.py` env | `roslaunch lite3_gazebo_sim ...` 与 two_floor_nav 解析到 P2 |
| S3 | P3 仓落地（console + 两个生成器 + env 根 + Makefile + 首提交） | `check_console.sh`、agent 契约、体素/任务事件测试过 |
| S4 | P1 移除 `95_simulation`/`scripts/console`，更新守卫/工具/文档 | P1 平台单测 + 解耦守卫 + import 守卫过 |
| S5 | 独立发布物：P2 install tarball、P3 发行包，含 manifest+sha256 | `dist/` 产物可解包、sha256 校验过 |
| S6 | 集成回归：scoped BT（仿真）读 P1→P2 装配、P3 只读 | `DECOUPLE-upstairs` 等价摘要 ok=true |
| S7 | 证据、note 登记、bridge 收口 | `note/进度/v8/M5/26-11-14/evidence/REPOS/` |

## 6. 风险与回滚

- **旧路径断链**：历史 note/README 引用 `ws/src/95_simulation` 不回改；活引用按上表更新。
- **镜像构建**：Dockerfile 仍从 P1 `ws/src` 构建，不含 P2；P2 在 `sim_ws` 独立构建，不依赖镜像层。首机构建顺序写入 P2 README。
- **缓存错配**：P1 命名 devel 卷可能残留旧 P2 目标；迁移后首轮 `catkin_make` 重建并记录。
- **回滚**：P1 删除前保留 `git revert` 点；两个新仓独立，删除即可回退。P1 历史未改写，`code_ref` 不受影响。
- **并发**：codex `T-20261113-03` 在 review，未提交 `web/index.html` 不碰；`web/` 不迁。
