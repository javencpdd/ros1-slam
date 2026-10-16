# ros1-slam 跨 Agent 协作黑板 — 使用说明

**解决的问题：** 在 `192.168.1.103` 的 `/home/ros1-slam` 项目上，zcode 和 qoder 交替干活时
能不能无缝接力，且任务不串台。

---

## 一、为什么这么设计

| 手段 | 结论 |
|---|---|
| 直接同步 zcode / qoder 的私有记忆文件 | ✘ 格式闭源（sqlite / 压缩 jsonl）+ 厂商随时变更 + 有写坏库的风险 |
| memU（14.5k star, Apache-2.0） | △ 定位正合适，但官方 host 矩阵不覆盖 zcode / qoder，需自写 adapter + 自备 embedding |
| MCP 记忆服务 | △ 两边都支持 MCP，但为一个 ROS 项目引入 DB 依赖过重 |
| **共享黑板 + bridge.py** ✔ | 零依赖、纯文本、可 git diff、人和 LLM 都能读、换任何工具都不失效 |

**关键突破口：** zcode 和 qoder 都会读工作区的 `AGENTS.md`。这是唯一可靠公共面。

> ⚠️ zcode **不展开 `@import`**（官网明确），所以所有必读规则都**内联**在 `AGENTS.md` 里，
> 长篇内容靠「指路让 Agent 自己去读」，而不是塞进去。

---

## 二、部署到 103

在本机（能 ssh 到 103 的机器上）执行：

```bash
cd ~/ros1-slam-agent-bridge          # 本目录
bash install.sh user@192.168.1.103 /home/ros1-slam
```

`install.sh` 会做：

1. 把 `AGENTS.md` 放到 `/home/ros1-slam/AGENTS.md`（已存在则先备份成 `AGENTS.md.bak-<时间戳>`）
2. 把 `.ai/` 整体放到 `/home/ros1-slam/.ai/`
3. 把 `.qoder/rules/` 放到 `/home/ros1-slam/.qoder/rules/`
4. 远端跑 `python3 .ai/bridge.py init` 生成 `state.json` / `TASKS.md` / `HANDOFF.md`

已有 103 上的 `.ai/` 数据不会被覆盖（脚本会跳过已存在的 `.ai/state.json`）。

---

## 三、日常工作流

### 场景 A：qoder 干了一半，换 zcode 接手

```bash
# 在 zcode 里，第一句话直接说：
读取 AGENTS.md，执行 cd /home/ros1-slam && python3 .ai/bridge.py resume --as zcode，然后按指示继续。
```

`resume` 会吐出：当前任务是谁、要去读哪几个文件、作用域限定、上一棒的交接摘要、最近 Journal。

### 场景 B：zcode 干完，交回给 qoder

```bash
cd /home/ros1-slam
python3 .ai/bridge.py note    T-20261016-01 "已完成 xxx，验证命令 catkin_make 通过"
python3 .ai/bridge.py handoff T-20261016-01 --to qoder \
  --summary "下一步跑 xxx.bag 回归并对比轨迹误差；坑：g2o 别升级；别碰 params.yaml"
```

### 场景 C：查看全局 / 排查串台

```bash
python3 .ai/bridge.py status
python3 .ai/bridge.py check          # 会报：多个 in_progress、任务卡缺失、scope 未限定、烂尾任务
```

---

## 四、目录结构

```
/home/ros1-slam/
├── AGENTS.md                        ← 两个工具共同读取的入口（精简，靠 AGENTS.md 引用）
├── .ai/
│   ├── bridge.py                    ← 状态机 CLI，唯一的状态写入者
│   ├── state.json                   ← 单一真相源（机器读写）
│   ├── TASKS.md                     ← 自动生成，任务账本
│   ├── HANDOFF.md                   ← 自动生成，当前交接游标
│   ├── JOURNAL.md                   ← 只追加的操作流水
│   ├── knowledge.md                 ← 跨工具长期知识（手写沉淀）
│   └── tasks/
│       ├── _template.md
│       └── T-YYYYMMDD-NN.md         ← 每个任务一张卡，含全部历史
└── .qoder/rules/
    └── ros1-slam-collab.md          ← qoder 侧常驻提醒（带 alwaysApply）
```

---

## 五、防串台的三道保险

1. **单 current 游标**：全局只有 `current_task` 一个，`resume` 只吐它。
2. **scope 白名单**：每个任务卡写明允许改动的文件目录，越界按违规处理。
3. **epoch 版本号**：任何状态迁移 +1；Agent 读到旧 epoch 会被要求重跑 `resume`。
   外加 `check` 命令主动检测「多个 in_progress」「任务卡缺失」「N 天未更新的烂尾任务」。

> AGENTS.md 类指令的遵守率天然是**概率性**的（<200 行、指令冲突会随机取舍）。
> 所以**状态迁移务必走 bridge.py**，让脚本而不是 LLM 来保证一致性。

---

## 六、可选增强

- **zcode 全局提醒**：在 `~/.zcode/AGENTS.md`（用户全局）追加一句
  「进入 /home/ros1-slam 工作区时，先跑 `python3 .ai/bridge.py resume --as zcode`」，
  省得每次手动说。
- **git 版本化**：把 `AGENTS.md` 和 `.ai/` 提交进 git（`.gitignore` 里排除 `.ai/.state.lock`），
  交接记录就有了 diff 历史。
- **文件监听自动拉起**（进阶）：参考 Markdown Blackboard 的 watcher 模式，
  用 `nodemon --watch .ai/HANDOFF.md` 在交接时自动唤起另一个 CLI。
  通常没必要——你手动切工具已经够快。

---

## 七、参考

- zcode 官方：项目指令文件 AGENTS.md <https://zcode.z.ai/cn/docs/agents>
- qoder 官方：静态记忆与 Rules <https://docs.qoder.com/zh/cli/memory>
- 黑板 vs监督者 模式：<https://www.agentpatternscatalog.org/patterns/blackboard>
- Agent 交接四模式（Handoff Envelope / Relay Ledger / 显式 ACK / 上下文压缩）：
  <https://askpatrick.co/guides/agent-handoff-patterns>
- memU（日后要跨机/语义检索再升级）：<https://github.com/NevaMind-AI/memU>
