# SUBREPOS · 三仓落回主仓 subprojects（2026-11-14 第二切片）

> 任务：`T-20261114-04`（qoder）。用户决策：**合并到 `subprojects/`、`git subtree` 保留历史、仅本地提交**。
> 本目录只放文档与数据；代码真身仍在 P1 `ws/src`、`subprojects/simulation`、`subprojects/console`。

## 1. 拓扑

| 子项目 | 主仓内路径 | 工作区 | 历史 |
| --- | --- | --- | --- |
| P2 `simulation` | `subprojects/simulation` | `ws/`（独立 catkin） | 同级仓 `d394bd1` 经 `git subtree` 并入 |
| P3 `console` | `subprojects/console` | host-only 根 | 同级仓 `c1848e3` 经 `git subtree` 并入 |

- 主仓 `origin`：`git@github.com:javencpdd/ros1-slam.git`，是唯一远端同步面；**本切片未 push**。
- 原同级仓已移到仓外 `/home/jack/ysc-code/backup-20261114-subprojects/`（含各自 `.git` 与未跟踪产物），可回滚。
- P2/P3 的 `ws/build`、`ws/devel`、`ws/install`、`dist` 从原仓复制到新路径（忽略规则内），容器 `sim_ws` 直接可用。

## 2. Git 历史

```text
a74d9b2d Add 'subprojects/simulation/' from commit 'd394bd11...'
5220227f Add 'subprojects/console/'    from commit 'c1848e35...'
e36562b0 merge: 三仓合并到主仓 subprojects（subtree 保留 P2/P3 历史）
```

`git log --oneline` 在 `subprojects/...` 下可追溯到 P2/P3 的提交；主仓 `c28acddf` 及之前历史不变。

## 3. 路径更新

- `docker/docker-compose.yml`：`../subprojects/simulation/ws → /opt/ros1-slam/sim_ws`；`../subprojects/simulation → /opt/ros1-slam/sim_repo`。
- `docker/Dockerfile` 的 `GAZEBO_MODEL_PATH`/`RESOURCE_PATH` 改 `sim_ws`；`.dockerignore` 排除 `subprojects/console` 与 P2 构建/发布物。
- `V8_SIM_ROOT` / `V8_CONSOLE_ROOT` 默认指向 `subprojects`；console 的 `paths.py`、`check_console.sh`、`start_agent.sh`、`ops/*`、`test_bt_config` 更新。
- `target_workspace.py` 的 x86_sim 默认源与 `HOST_ONLY_PATHS` 改 `subprojects/console`；`two_floor_nav`/`scene.py`/`test_support_cells` 的宿主根改 `subprojects/simulation`。
- 活文档 `README`/`ws/README`/`docker/README`/`note/架构/项目架构与目录.md`/`current-2026-11-14.md` 同步；`three-repos-migration` 保留原设计并加第二切片指针。

## 4. 验证

| 项 | 读数 |
| --- | --- |
| P1 framework | 303 OK（10 skip） |
| P2 | 15+2 OK（含 install-space、support-cells 对齐、inspection） |
| P3 | 5/17/4/2 OK + BT 配置 + mission events |
| 解耦/端口/target/contracts | 9 / 6 / 27 / 8+6 OK |
| console/check_bt/compose/standards | ALL PASS / TREE OK / PASS / rc=0 |
| 容器 | `sim`、`sim-viz` Up；`rospack find v8_simulation`→`/opt/ros1-slam/sim_ws/src/v8_simulation` |
| 窗口 `ABC-upstairs-r3` | **ok=true**；六 checks；57 poses；末点 `(-5.9,1.1,1.6)`；carrier `(-5.747618277068962,1.121768894023993,1.6)`；markers 40 |

## 5. 发布物

- P2：`subprojects/simulation/dist/ros1-slam-simulation-0.1.0.tar.gz`，2,414,199 B，sha256 `4fe3e74e…a3f32947f`，源 head `d394bd1`。
- P3：`subprojects/console/dist/ros1-slam-console-0.1.0.tar.gz`，540,343 B，sha256 `3a6e138e…c742a84d6`，源 head `c1848e3`。
- 发布物在 `.gitignore` 内，不入主仓索引；主仓远端同步的是**可编辑源码**，发布包按需 `make release` 重建。

## 6. 回滚

1. `git revert -m 1 e36562b0`（撤销子树合并）或 revert 本切片的结构提交；
2. 把 `backup-20261114-subprojects/{ros1-slam-simulation,ros1-slam-console}` 移回 `/home/jack/ysc-code/`；
3. 恢复 `docker-compose.yml` 旧挂载并重建容器。

## 7. 边界

- 未 push；远端同步由用户执行。
- 旧同级仓仅作备份，不是活副本；勿在备份里改代码。
- 三仓不再是独立 git 仓；独立 **工作区**、**发布物**、**测试**仍保留。
