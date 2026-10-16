---
description: ros1-slam 跨工具协作（zcode ⇄ qoder）任务交接纪律
---

# ros1-slam · 跨工具协作纪律

> 本规则没有 `paths` frontmatter，因此在项目内**始终生效**。
> 若只想限定到某些目录，可加：`paths: - src/**`（Qoder 官方支持的写法）。

本项目 `/home/ros1-slam` 由 **zcode 与 qoder 交替驱动**。两者的自动记忆互不通用，
交接只能依赖仓库内的 `.ai/` 黑板文件。

## 每次会话开始必须执行

```bash
cd /home/ros1-slam
python3 .ai/bridge.py resume --as qoder
```

`--as` 必须换成自己的身份：`zcode` / `qoder` / `human`。
拿到 `resume` 输出后，**按它列出的顺序把文件读一遍**，然后 `claim` 认领。

**没有读到 `.ai/tasks/<ID>.md` 任务卡原文之前，禁止修改任何代码。**

## 每次会话结束必须执行

```bash
python3 .ai/bridge.py note    T-20261016-01 "本次成果 + 关键结论"
python3 .ai/bridge.py handoff T-20261016-01 --to <zcode|qoder|human> \
  --summary "下一步做什么 / 坑在哪 / 别碰什么"
python3 .ai/bridge.py check
```

## 红线

1. 同一时刻只跟进 `HANDOFF.md` 里标记为 current 的那**一个**任务。
2. 只改任务卡「作用域」白名单内的文件，越界重构一律禁止。
3. `TASKS.md` / `HANDOFF.md` / `state.json` 由 `bridge.py` 生成，**禁止手改**。
   唯一可手写追加的是任务卡里的「工作笔记」。
4. `HANDOFF.md` 顶部的 `epoch` 小于 `bridge.py json` 输出的值 = 读到旧快照，重跑 `resume`。
5. 涉及上机运行的参数（外参、TF 树、雷达配置）修改前**必须先向用户确认**。
6. ROS1 项目，不要擅自引入 ROS2 写法。

完整契约见项目根目录 `AGENTS.md`。
