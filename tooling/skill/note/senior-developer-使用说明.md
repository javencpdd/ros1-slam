# senior-developer 使用说明（v3 适配版）

> 适配基准：21 号《ros1-slam 隔离部署计划书 v4》
> （`/home/ros1-slam/note/计划书v4/21_ros1-slam隔离部署计划书-v4.md`；P0–P2 已按 v3 执行完毕冻结，v4 作用于 P3a 及以后）
> 适配日期：2026-10-04　原始定义备份：`.orig-backup/senior-developer.md.orig`

---

## 一、这是什么

| | 适配前 | 适配后 |
|---|---|---|
| 来源 | WorkBuddy 专家市场「senior-developer（吴八哥）」 | 同一人设，领域重铸 |
| 定位 | 通用高级全栈开发工程师（Web 后端/前端） | **ROS1 Noetic / Jetson 高级部署工程师**，专职执行 21 号 v4 计划书 |
| 工作流 | 任务分解 → 增量「写→验→报」→ 交付清单 | **定位 Phase → 读对应章节 → 施工 → Gate 校验 → 落盘 note/**，逐阶段推进 |
| 质量闸 | 代码自检清单 | **计划书 Gate G0–G9 + v3_isolation.sh + 真机安全专项** |
| 错误恢复 | 三次失败暂停 | 三次失败暂停 + **§14 回滚矩阵对号处置** |

核心不变的原则：交付为王、小步快跑、言出必行、简洁高效——只是「交付物」从代码变成了**过了 Gate 的部署状态**。

## 二、v3 适配映射

| 原概念 | v3 落点 |
|---|---|
| 任务分解（≤10 行计划） | 开工三步：定位阶段（§4 DAG）→ 读对应章节 → 报「做什么/过哪个 Gate/怎么验」 |
| 功能模块 | 部署阶段 P0–P7 与配置项 C0–C10 |
| 自动验证（lint/build/test） | Gate 校验命令（§4.3、§11.3 验证矩阵）+ `v3_isolation.sh`（8 项） |
| 交付清单 | Phase 收尾状态 + Gate 证据 + note/进度/<阶段>/（进度文档.md + 证据/）落盘 |
| 硬编码红线 | **7 条红线速查**（见角色文件）：ros2qnx 禁令、/home/ysc 隔离、不复制 build/devel、env.sh 唯一入口、第三方包一字不改（唯一例外 laser_mapping.cc:751）、顺序不能倒、/cmd_vel 唯一 publisher |

## 三、怎么触发

ZCode 会在对话命中以下话题时自动加载（也可 `/senior-developer <任务>` 强制加载）：

- 提到 **ros1-slam、21 号计划书、沙箱部署**：如「按计划书把 P2 复制跑完」「ros1-slam 那边编译挂了帮我看下」
- 提到 **Gate / 配置重构**：如「C1 的 yaml 转换稿落一下」「G8 一直不过怎么排查」
- 提到 **采图/建图/编译部署**：如「明天去现场采图，流程帮我过一遍」

示例 prompt：

1. 「读一下 21 号计划书，告诉我现在可以直接做哪个 Phase，为什么」
2. 「执行 P2 的文件复制和路径改写，完成后跑隔离自检」
3. 「编译批 3 报 OOM 了，按计划书怎么处理」

## 四、文件结构

```
senior-developer/
├── SKILL.md                        # 入口与路由（v3 新增，ZCode 发现用）
├── agents/senior-developer.md      # ★ 角色定义（v3 已重铸）
├── skills/fullstack-dev/           # 通用子技能，未改动
├── skills/frontend-dev/            # 通用子技能，未改动
├── skills/browser-use/             # 通用子技能，未改动
├── MANIFEST.md / MIGRATION.md      # 原包清单，未改动
└── avatars/ .codebuddy-plugin/     # 原包资源，未改动
```

## 五、ZCode 发现路径说明

本包实际位于 `~/.zcode/skill/senior-developer/`（**单数 skill**，ZCode 不扫描）。
已在 `~/.agents/skills/senior-developer` 建立符号链接接入发现路径。
若 skill 不触发，依次检查：

```bash
ls -l ~/.agents/skills/senior-developer/SKILL.md   # 链接与入口存在
head -5 ~/.agents/skills/senior-developer/SKILL.md # frontmatter 完整（name/description）
```

恢复链接：`ln -sfn ~/.zcode/skill/senior-developer ~/.agents/skills/senior-developer`

## 六、红线速查（施工时随身）

1. `ros2qnx` 仅真机 mode，其余模式出现即事故
2. `/home/ros1-slam` 内禁 `/home/ysc`（note 来源记录、opt/ref 除外）
3. 永不复制 build/devel/.git
4. 环境入口只用 `source scripts/env.sh`
5. 第三方包一字不改（唯一例外：`laser_mapping.cc:751` 存盘路径，已改写并登记 diff）
6. 顺序不能倒：G3→G8→G9 不全不碰真定位/对比实验/第二速度源/SCAN-PCT
7. `/cmd_vel` publisher 只能是 `safe_cmd_vel_mux`；M4 只出 `/cmd_vel_raw`
8. 每完成一个阶段/配置项必须落 `note/进度/<阶段>/`（进度文档.md + 证据/）并更新 00_总览；不留档视为未完成

## 七、回滚

角色定义与 SKILL.md 均可从 `.orig-backup/`（本 skill 目录上一级）恢复原始版本；
适配不影响 MANIFEST.md 记录的原始包完整性基线。
