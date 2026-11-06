# Agent 协作与跨主机任务交接

> 最新更新时间：2026-11-06 22:42:50 CST。Python 标准库、Git、本机文件锁；Python 3.8+。工程标准真源：[工程标准](../../note/架构/工程标准.md)，进场规则：[跨工具协作契约](../../note/09_跨工具协作契约.md)。

[bridge.py](bridge.py) 是随 Git 分发的命令本体，[crosshost.py](crosshost.py) 负责单任务交接。[流程教程](../../note/tools/跨Agent协作黑板-使用教程.md)说明同工作区换工具和跨主机接续；[交付记录](../../note/进度/00_跨主机任务交接_2026-11-06.md)记录验证范围。

## 本机进场

在项目 Git 根运行；新机先取得包含本工具的已提交代码。`--root` 是全局参数，须放在子命令之前，省略时由工具所在目录推导。不需要 source ROS 环境。

```bash
python3 scripts/agent/bridge.py bootstrap --platform x86_sim --as codex
python3 .ai/bridge.py resume --as codex
python3 .ai/bridge.py check
```

机器人机明确选 `--platform lite3` 或 `--platform m20`。平台只标识交接目标，不安装、构建或启动 ROS/仿真/训练，不代表 M20 适配已完成。运行环境、地图、SDK、网络和目标构建产物单独准备；旧 `scripts/env.sh` 的 devel 入口不由本工具修正。

bootstrap 保留现有任务、epoch、任务卡和已有契约指针，生成本机 `.ai/bridge.py` 薄入口、host UUID/platform 与私有 inbox/outbox。重复执行保持身份与状态。已有 host 不允许隐式改平台。已知旧 runner 自动按 SHA 备份到 `.ai/legacy/` 再替换；未知定制 runner 默认拒绝，审查后可明确加 `--replace-legacy` 保留备份迁移。已有 AGENTS 缺少共同指针时拒绝覆盖。bootstrap 中断时从 canonical 入口重跑，不依赖可能未写完的薄入口。

`.ai/` 与 AGENTS 继续被 Git 忽略。不要提交、打包整库或跨机复制 host 身份。旧 `init/new/claim/note/handoff/done/status/ls/check/resume/knowledge/mem/json` 保持兼容；不对已有工作区重跑 `init --force`。

## 源机到目标机

以下是两个终端上的命令，文件路径代表操作者准备的普通 JSON 文件。票据、交接包在可信通道中传递，工具没有 SSH、远程执行、自动拉代码或自动 checkout 功能。

目标机先 bootstrap，输出本机票据到已存在的外部目录：

```bash
python3 .ai/bridge.py host --out /tmp/target-host.json --as codex
```

将票据交给源机。源机确认当前任务、阶段提交和目标端 HEAD 一致，使用实际任务 ID：

```bash
TASK_ID=T-YYYYMMDD-NN
python3 .ai/bridge.py export "$TASK_ID" \
  --destination /tmp/target-host.json --to codex --as codex \
  --summary '剩余步骤、已试失败方案、代码提交、目标环境未验项与边界' \
  --evidence '提交号:证据路径及摘要；接收端获取方式' \
  --ttl-hours 24 --out /tmp/task-handoff.json
```

首次 export 要求选中当前开放任务、现任 Agent、非空明确 scope、依赖引用 `blocked_by` 已解决，以及 scope 内 Git 跟踪/非忽略未跟踪文件干净。路径按字面目录前缀检查，含中文、空格和方括号也不作为 Git 通配符；路径逃逸和软链拒绝。范围外脏文件保留，不打入包。代码工具必须与本机 HEAD 中的版本一致，目标票据也绑定该提交。

导出先持久化私有 outbox 和源端 `transferred` 状态，清空源游标，再公开 JSON 包。源端该任务的 claim/note/handoff/done 和清除移交记录的 init 被拒绝；`transferred` 不等于 done，也不自动执行其他旧任务。

将包交给目标机，检查并接收：

```bash
python3 .ai/bridge.py inspect /tmp/task-handoff.json --show-card --as codex
python3 .ai/bridge.py import /tmp/task-handoff.json --as codex
python3 .ai/bridge.py resume --as codex
python3 .ai/bridge.py check
```

inspect 只校验包结构/完整性并展示接收身份、提交、scope 和有效期，**不核对本机是否满足导入条件**；可以离线查看过期包。import 核本机 UUID/platform、接收 Agent、精确 HEAD、有效期与状态；不自动修环境。目标有当前任务或执行中任务时拒绝；源 ID 冲突则分配本地 ID 并返回映射，保留已有任务及孤立任务卡。导入默认为 `review`；核来源、授权、scope、实际代码/资产和证据后，用返回的本地 ID 显式 claim 再施工。包本身不增加设备动作许可。

## 重试、续期与返回

- **公开输出失败**：源保持 transferred；使用同一任务 ID 再 export，只需 `--out` 与原身份，重发私有 outbox 的同一 core。输出已存在且相同可重试，内容不同拒绝覆盖。
- **过期**：源执行下面的 `export --renew`，发出新文件。保持原 task/package UUID、hop、接收身份及代码提交，只增加租约版本/时间。源 HEAD 已推进也不改变包绑定的原提交；目标首次接收仍须该原提交。
- **重复导入**：核已有回执和私有 inbox 后只读返回映射，不 bump、不重新激活任务；合法接续写笔记、done、转出或 HEAD 推进后仍如此。同 package ID 的 core 变化拒绝。
- **返回原机**：接收机完成阶段提交后，对原机新票据 export；原机 import 接续同一 task UUID。初版只支持直接 peer 回流且 hop 连续；A→B→C→A 不自动解除 A 的隔离，旧包重放不解除 B 的隔离。

```bash
python3 .ai/bridge.py export "$TASK_ID" --renew --ttl-hours 24 \
  --out /tmp/task-handoff-renewed.json --as codex
```

不要因 TTL 过期在源端手动取消移交或恢复执行。离线机制不知道目标是否已收到旧包；续期仍保持源隔离，目标若已收只返回原回执。

## 中断恢复

普通事务异常恢复受限文件的原内容；修改时间可能变化。硬中断留下 `.ai/transfer.pending.json`，常规状态写入和 resume/check 拒绝继续；保留现场，从 canonical 入口运行：

```bash
python3 scripts/agent/bridge.py recover --as codex
python3 .ai/bridge.py check
```

recover 校验私有快照摘要及固定路径白名单，恢复尚未公开的事务；损坏快照拒绝，不能通过它撤销已公开的移交。公开包之前 pending 必须已清除，因此输出失败只能重发。源端硬中断可能留下未引用的私有 outbox，保留排查，不据此启动任务。工具不承诺突然断电时多个文件的整体原子性，也不是分布式锁；手改状态、旧 runner 或其他程序不受其约束。

## 格式、边界与验证

票据格式 `agent_host/v1`，交接包 `agent_handoff/v1`。包只载所选任务元信息、原卡文本、摘要、证据索引和主机/提交/链/UTC 租约；不载其他任务、代码文件、patch、地图、构建缓存或整个状态库，不执行文本。**原卡与摘要是自由文本，可能已有敏感内容；工具不做自动脱敏，导出前须检查。** SHA-256 只校验完整性，不能认证发送者，须信任传输来源和本机文件系统。租约要求两端 UTC 正确。

工具资源限额：包 1 MiB；原卡 256 KiB；摘要 64 KiB；scope 1–128 项、每项至多 512 字节；证据至多 128 项、每项 4096 字节。TTL 为 1–168 小时，默认 24。超限、未知字段/版本、重复 JSON 键、非法类型/时间、软链或越界路径拒绝；不截断记录求通过。每次往返保留来源上下文，过长任务卡同样按限额拒绝。

```bash
python3 -B scripts/agent/test_crosshost.py
```

测试仅在 owned 临时 Git 仓库中运行真实 CLI 和故障注入，不操作当前 `.ai`、ROS、SSH、Docker或机器人。完整性、格式、冲突、幂等、过期/续期、直接/多跳回流、scope、锁失败与事务恢复各有正反例；实际异机传输、跨账户 ACL 和设备环境另验。
