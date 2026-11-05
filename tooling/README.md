# 工具资产、技能镜像与恢复

> 最新更新时间：2026-11-05 11:49:54 CST。本轮核对仓内脚本和清单覆盖；未恢复技能、未改系统服务。

本目录保存项目维护的工具资产。运行环境、源码和真实地图各有自己的交付方式，不能用2026年10月旧机的容量/包数快照描述当前主机。

| 入口 | 作用与限制 |
| --- | --- |
| `skill/software-company/` | 多角色协作工作流与角色定义 |
| [skill/senior-developer/SKILL.md](skill/senior-developer/SKILL.md) | 单阶段施工工作流及通用开发子技能 |
| `skill/note/` | 技能使用说明 |
| `skill/window-scheduler/` | 历史窗口调度技能；当前调度不运行后台 watch |
| [skill/RESTORE.sh](skill/RESTORE.sh) | 将四个技能目录复制到**执行账户**的 `$HOME/.zcode/skill`，为 `$HOME/.qoder/skills` 建链接 |
| `systemd/` | 已登记的生产服务单元镜像；用于核对来源，不自动安装/启停 |
| [_forbid_str_exempt.tsv](_forbid_str_exempt.tsv) | `tooling/` 的显式字符串豁免登记，不能扩展到其他目录 |
| [_build_tooling_manifests.sh](_build_tooling_manifests.sh) | `skill` 子清单及 `top` 顶层清单的校验与整体重建 |

现行项目规则先读[跨工具协作契约](../note/09_跨工具协作契约.md)，路线与排程见[计划入口](../note/计划书v8/README.md)及[执行队列](../note/计划书v8/23_v8任务队列.md)。技能中历史阶段、固定主机路径和旧待办不覆盖现行指针。

## 校验与恢复

在仓库根只读核对：

```bash
bash tooling/_build_tooling_manifests.sh --target skill --check
bash tooling/_build_tooling_manifests.sh --target top --check
```

两份清单覆盖不同：`skill/_sha256_manifest.txt` 覆盖技能文件；顶层 `_sha256_manifest.txt` 覆盖本 README、恢复脚本、豁免表、窗口技能和已登记的 systemd 单元。修改被覆盖文件后，旧 digest 不再表示当前内容；清单的重新生成须遵循生成件既有生命周期，记录旧新摘要和校验结果；不能手改摘要掩盖差异，也不能仅凭旧同步戳称“全过”。本轮保留原清单，README 已有的摘要差异照实列入核对记录，不冒充一次完整恢复校验。

需要恢复到本账户工具路径时，先确认前述校验和目标账户，再执行：

```bash
bash tooling/skill/RESTORE.sh
```

脚本先打印校验 OK 数，然后备份已有技能根并复制文件、设置权限、为尚不存在的 Qoder 路径建链接。**它不是 fail-closed 恢复器**：脚本只启用 `set -u`，清单有坏项并不保证阻止后续复制；已有链接或目录也不会自动纠正。应先独立校验，恢复后人工核对路径与链接。它不配置 Codex/WorkBuddy 技能、不安装 MCP、不恢复 ROS，也不恢复 `/etc/systemd/system/`。

## 外部依赖

| 依赖 | 本仓能提供什么 | 目标环境仍须具备什么 |
| --- | --- | --- |
| ROS1 Noetic 与系统库 | [Docker 环境](../docker/README.md)、工作区源码与依赖说明 | x86 Docker 或目标机原生 ROS、可用编译工具链 |
| MCP、插件与会话工具 | 必要的项目说明和协作契约 | 会话宿主提供的连接与权限；不能从旧机“不存在配置文件”推断新机同样如此 |
| 系统服务 | `systemd/` 镜像和来源登记 | 管理员根据实际主机逐项核对；本文不授权安装或启停生产服务 |
| 模型、地图、bag 与外部仿真资产 | registry 模板、路径约定、证据索引 | 各机真实文件、来源、许可与数据校验；不从其他主机默认名直接覆盖 |

来源与变更以 `note/` 登记为准。生产侧目录、第三方原件、冻结清单和历史证据遵守各自边界；恢复工具资产不等于授权反向同步或真机运行。
