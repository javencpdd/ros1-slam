# 跨 Agent 协作黑板 · 使用教程（zcode / qoder / WorkBuddy 接力）

创建：2026-10-17 ｜ 更新：2026-10-17（补本机实战实录）
适用工作区：`/home/jack/ysc-code/ros1-slam`（本机，开发主场）
状态：已部署并实测，`check` 通过（epoch=24）

---

## 一、三家记忆是怎么互通的

同一台机器上有 **zcode、qoder、WorkBuddy** 在 ros1-slam 上交替干活。它们各有一套「自动记忆」，
但**三家互不相通、且大多不可导出**：

| 工具 | 私有记忆位置 | 能给别的工具用吗 |
|---|---|---|
| zcode | 本地 sqlite | ✘ 官方明确「不能查看或清除」，不进 Git |
| qoder | `~/.qoder/projects/<工作区slug>/memory/` | ✘ 只对 qoder 可见，且要 `QODER_MEMORY=1` 才开 |
| WorkBuddy | `<项目>/.workbuddy/memory/MEMORY.md`（项目级）<br>`~/.workbuddy/MEMORY.md`（用户级） | ✘ 另两家不读 |

**唯一公共面是 `<项目根>/AGENTS.md`** —— 三个工具都原生读它。WorkBuddy 的官方文档原文：

> 进行代码开发时，WorkBuddy 支持读取项目工作目录下的 `.codebuddy/` 项目级配置……
> **兼容现有配置：与 `CODEBUDDY.md`、`.codebuddy/CODEBUDDY.md`、`AGENTS.md` 及用户级 `~/.codebuddy/` 配置兼容，可并存使用**

所以做法**不是**打通三家私有记忆，而是**在仓库里放一份纯文本共享状态（黑板）**，
三家各自用自己的机制去读它：

```
   zcode ──── 私有 sqlite（不可导出）───┐
                                        │
   /AGENTS.md  ◄── 三家唯一公共面 ───────┤
   （共读入口）                          │
   qoder ──── ~/.qoder/projects/ ───────┤
                                        │
   workbuddy ─ .workbuddy/memory/ ──────┘
                     │
                     ▼
             .ai/  ← 共享黑板（唯一真相源）
```

**`.ai/` 不是新账本**，只管「此刻谁在干哪一件、上一棒留了什么话」。
项目事实的唯一权威仍然是 `README.md` 导航表、`ws/README.md`、`note/` 与计划书
—— **有冲突一律以 `note/` 为准**。

---

## 二、几个文件各是什么

| 文件 | 谁写 | 作用 |
|---|---|---|
| `AGENTS.md` | 人 | 三工具共读入口。硬规则**全部内联**（zcode 不展开 `@import`），长内容指向 `.ai/` 让 Agent 自己去读 |
| `.ai/bridge.py` | 人 | 状态机 CLI，**唯一的状态写入者**。所有状态迁移必须走它 |
| `.ai/state.json` | bridge.py | 机器读的单一真相源 |
| `.ai/TASKS.md` | bridge.py 自动生成 | 任务账本（人类/Agent 可读） |
| `.ai/HANDOFF.md` | bridge.py 自动生成 | **当前交接游标**：进行到哪、上一棒留了什么话 |
| `.ai/JOURNAL.md` | bridge.py 追加 | 只追加的操作流水，回答「最近发生了什么」 |
| `.ai/tasks/T-*.md` | bridge.py + 人 | 每张任务卡：元信息 / 目标 / 非目标 / 已尝试 / 阻塞 / 验收 / 工作笔记 / 交接记录 |

> `TASKS.md` / `HANDOFF.md` / 任务卡的「元信息」段都是 `state.json` 的**渲染产物**，禁止手改
> ——手写会立刻分叉。唯一可手写追加的是任务卡里的「工作笔记」与「交接记录」正文。

---

## 三、实战：qoder ⇄ workbuddy 三棒接力（本机实录）

下面是一次**真实跑过**的接力，输出原样摘自 2026-10-17 04:29–04:30。
任务卡全文在 `.ai/tasks/T-20261017-02.md`，可以 `cat` 出来对照。

**场景**：`ws/src/40_navigation/lite3_scan` 的 scan 时间戳与 `/livox/imu` 差约 12ms，
分三段干——qoder 定位、workbuddy 改、qoder 回归收尾。

### 第 1 棒：qoder 开工

在 qoder 里**第一句话**就说这句（`--as` 换成你自己）：

```
读取 AGENTS.md，执行 cd /home/jack/ysc-code/ros1-slam && python3 .ai/bridge.py resume --as qoder
```

输出（没有进行中的任务时）：

```
========================================================================
RESUME PACKET · epoch=11 · 生成于 2026-10-17 04:29:23 · 请求方=qoder
========================================================================
当前无进行中任务。用 `python3 .ai/bridge.py new "标题" --as qoder` 开一个。
```

于是开任务、认领、干活、交接：

```bash
ID=$(python3 .ai/bridge.py new "核对 lite3_scan 的 scan 时间戳与 /livox/imu 对齐" \
       --as qoder --scope ws/src/40_navigation/lite3_scan)      # 作用域白名单
python3 .ai/bridge.py claim "$ID" --as qoder --summary "从总览读到该模块的 scan 时间戳疑点，开工排查"

# ……qoder 实际干活（改代码、跑验证）……

python3 .ai/bridge.py note "$ID" \
  "实测 scanRegistration.cpp:88 处时间戳落后 imu 约 12ms；已定位到没用 header.stamp 而是 ros::Time::now()" \
  --as qoder

python3 .ai/bridge.py handoff "$ID" --to workbuddy --summary \
  "下一步：把该处改成用 header.stamp，并跑一次 v_scan_p1.sh 验证。坑：同文件 L102 还有一处同类写法但先别动（不在本次范围）。别碰：ws/src/40_navigation/lite3_scan/launch/ 下的任何文件" \
  --as qoder
```

```
claimed T-20261017-02 -> qoder
noted
handed off T-20261017-02 -> workbuddy (epoch=15)
```

> **注意 handoff 摘要的三段式**：下一步具体做什么 ／ 坑在哪 ／ 别碰什么。
> 缺任何一段，下一棒就要重新摸索。

### 第 2 棒：workbuddy 接手（关键在这里）

在 WorkBuddy 里同样第一句话：

```
读取 AGENTS.md，执行 cd /home/jack/ysc-code/ros1-slam && python3 .ai/bridge.py resume --as workbuddy
```

这次输出完全不同——**它读到了 qoder 留的全部上下文**：

```
========================================================================
RESUME PACKET · epoch=15 · 生成于 2026-10-17 04:29:36 · 请求方=workbuddy
========================================================================
▶ 当前任务：T-20261017-02  核对 lite3_scan 的 scan 时间戳与 /livox/imu 对齐
   状态=blocked  现任=workbuddy  最后更新：2026-10-17 04:29:26

强制阅读顺序：
  1) cat /home/jack/ysc-code/ros1-slam/AGENTS.md
  2) cat /home/jack/ysc-code/ros1-slam/.ai/HANDOFF.md
  3) cat /home/jack/ysc-code/ros1-slam/.ai/tasks/T-20261017-02.md   ← 最重要，含全部历史结论
  4) tail -30 /home/jack/ysc-code/ros1-slam/.ai/JOURNAL.md

本次作用域（之外的文件不要动）：
  - ws/src/40_navigation/lite3_scan

上一棒交接摘要：
  > 下一步：把该处改成用 header.stamp，并跑一次 v_scan_p1.sh 验证。坑：同文件 L102 还有一处同类写法但先别动（不在本次范围）。别碰：ws/src/40_navigation/lite3_scan/launch/ 下的任何文件

任务卡最近记录：
  ## 工作笔记（按时间追加）

  ### 2026-10-17 04:29:25 · qoder 认领
  从总览读到该模块的 scan 时间戳疑点，开工排查

  ### 2026-10-17 04:29:26 · qoder
  实测 scanRegistration.cpp:88 处时间戳落后 imu 约 12ms；已定位到没用 header.stamp 而是 ros::Time::now()

  ## 交接记录

  ### 2026-10-17 04:29:26 · qoder 交给 workbuddy
  摘要：下一步：把该处改成用 header.stamp，并跑一次 v_scan_p1.sh 验证。坑：同文件 L102 还有一处同类写法但先别动（不在本次范围）。别碰：ws/src/40_navigation/lite3_scan/launch/ 下的任何文件
```

**workbuddy 不需要重新排查**——qoder 已经测出「12ms、`ros::Time::now()`、L102 别动」。
它直接改 + 验证，然后交接回 qoder：

```bash
python3 .ai/bridge.py claim T-20261017-02 --as workbuddy --summary "读到上一棒结论，直接改 header.stamp"
python3 .ai/bridge.py note T-20261017-02 \
  "已把 scanRegistration.cpp:88 改为用 header.stamp；v_scan_p1.sh 六项环境前置全 PASS，判据 0 失败" \
  --as workbuddy
python3 .ai/bridge.py handoff T-20261017-02 --to qoder --summary \
  "下一步：跑一次真机 bag 回归，对比轨迹误差是否降到 12ms 以内。坑：改完后 TF 里 base_link→laser 的 timestamp 也跟着变了，若有节点硬编码 now() 会重新引入偏移。别碰：ws/src/10_slam/faster_lio/（第三方包，一字不改）" \
  --as workbuddy
```

```
claimed T-20261017-02 -> workbuddy
noted
handed off T-20261017-02 -> qoder (epoch=18)
```

### 第 3 棒：qoder 收尾

```bash
python3 .ai/bridge.py resume --as qoder          # 又读到 workbuddy 的交接
python3 .ai/bridge.py claim T-20261017-02 --as qoder --summary "接手跑真机 bag 回归"
python3 .ai/bridge.py note T-20261017-02 \
  "bag 回归完成：轨迹误差从 12ms 降到 1.3ms；验证命令 v_scan_p1.sh 全 PASS" --as qoder
python3 .ai/bridge.py done T-20261017-02 --summary \
  "scan 时间戳改用 header.stamp 后，与 /livox/imu 对齐，bag 回归轨迹误差 12ms→1.3ms。遗留：同文件 L102 还有一处同类写法，另开任务处理"
python3 .ai/bridge.py check
```

```
done T-20261017-02
✔ 一致，epoch=21
```

### 最终状态与任务卡

```
$ python3 .ai/bridge.py status
epoch=21  current=-
ID               STATUS       OWNER    TITLE
------------------------------------------------------------------------------
 T-20261016-01   done         human    验证任务：打通 zcode/qoder 接力链路
 T-20261017-01   done         human    冒烟：三工具身份与跨机守卫
 T-20261017-02   done         human    核对 lite3_scan 的 scan 时间戳与 /livox/imu 对齐【教程示例】
```

任务卡 `.ai/tasks/T-20261017-02.md` 里能看到完整链条：三次认领、三条笔记、两次交接、一次完成，
**每一条都带时间戳和责任工具**。这就是「无缝切换」的实际样子。

### 这个例子里值得学的四点

1. **`--scope` 一开始就设**：qoder 只被允许碰 `lite3_scan`，所以它不会顺手改到 `faster_lio`。
2. **交接摘要写「别碰什么」**：workbuddy 因此知道 `launch/` 和第三方包是雷区。
3. **每棒都 `claim`**：任务卡上始终只有一个人在 `in_progress`，`check` 才不会报串台。
4. **qoder 的 12ms 结论没丢**：workbuddy 不用重新量一遍——这是接力省下来的主要成本。

---

## 四、命令速查

```bash
python3 .ai/bridge.py resume --as qoder                        # 换工具后第一件事
python3 .ai/bridge.py new "标题" --as qoder --scope ws/src/xx   # 开新任务
python3 .ai/bridge.py claim <ID> --as qoder                     # 认领
python3 .ai/bridge.py note <ID> "..."                           # 记笔记
python3 .ai/bridge.py handoff <ID> --to workbuddy --summary "..."  # 交接
python3 .ai/bridge.py done <ID> --summary "..."                 # 完成
python3 .ai/bridge.py status                                    # 任务表
python3 .ai/bridge.py check                                     # 一致性自检
python3 .ai/bridge.py knowledge "..."                           # 沉淀工具协作经验
python3 .ai/bridge.py json                                      # 给脚本读的状态快照
```

`--as` 合法值：`zcode` / `qoder` / `workbuddy` / `human` / `unknown`。

**省事技巧**：`export AI_BRIDGE_AGENT=qoder`，之后所有命令可省略 `--as`。

---

## 五、任务为什么不串（三道保险）

单靠 AGENTS.md 里的嘱咐是不够的——**指令遵守率是概率性的**（超 200 行明显下降，冲突指令会被随机取舍）。
所以一致性由 `bridge.py` 保证，不靠大模型自觉：

1. **单一 current 游标** —— 全局只有一个进行中的任务，`resume` 只吐它。想做别的事，先 `handoff` / `done`。
2. **作用域白名单** —— 每张任务卡写明本次允许改的路径，**不在列表里的文件不要动**，哪怕看起来该顺手重构。
3. **epoch 版本号** —— 任何状态迁移 +1。`HANDOFF.md` 顶部写着当前 epoch，
   读到的值小于 `bridge.py json` 的输出 = 拿到旧快照，重跑 `resume`。

外加 `check` 主动检测：**多个 `in_progress`** / 任务卡缺失 / 没写 scope / N 天没更新的烂尾任务。

---

## 六、跨机器同步（备用，当前用不到）

> 本机开发用不到这一节。保留是因为将来若要把 `/home/ros1-slam`（103 真机）也接进来，
> 两个工作区 clone 自同一个 GitHub 仓库，把 `AGENTS.md` + `.ai/` + `.qoder/` 提交进仓库后，
> `git push/pull` 就是同步通道。

### 6.1 日常同步

```bash
# 收工前（在哪台机器都一样）
cd <项目根>
git add AGENTS.md .ai .qoder
git commit -m "协作黑板：<本次任务简述>"
git push

# 开工前
git pull
python3 .ai/bridge.py check     # 必须做，见 6.2
```

`.ai/.gitignore` 已排除 `.ai/.state.lock`（进程锁，不进仓库）。

### 6.2 两个必踩的坑（已内置防护）

**坑 1：渲染产物与状态源脱节。**
`TASKS.md`/`HANDOFF.md` 是 `state.json` 的渲染产物。若 git 合并只带回渲染产物、却把
`state.json` 回退成旧的，下次一跑 CLI 就会**用旧状态覆盖别人的交接卡**。

防护：`bridge.py` 检测到 `HANDOFF.md` 的 epoch 大于 `state.json` 的 epoch 时，
**写命令（new/claim/note/handoff/done/knowledge）直接拒绝执行**，只读命令放行但告警；
`check` 把它列为硬错误。看到这个报错时：

```bash
python3 .ai/bridge.py check          # 会告诉你哪一侧 epoch 更大
# 确认以哪侧为准后，把另一侧的 state.json 换成它的，再重跑
```

**坑 2：`state.json` 里写着另一台机器的绝对路径。**
`resume` 会打印这些路径，指错地方。已做自愈：发现 `root` 与实际根目录不同会自动改正并记进 JOURNAL。

### 6.3 首次在第二台机器部署

**务必把第一台的 `.ai/` 状态拷过来**，不要重新 `init`——否则 epoch 和任务 ID 从 1 开始，
两边历史直接断裂。

```bash
# 例：从 103 拉状态到本机
ssh ysc@192.168.1.103 'cd /home/ros1-slam && tar czf - --exclude=.ai/.state.lock --exclude=.ai/__pycache__ .ai' \
  | tar xzf - -C /home/jack/ysc-code/ros1-slam
```

---

## 七、踩坑速查

| 现象 | 原因 | 处置 |
|---|---|---|
| `invalid choice: 'workbuddy'` | 旧版 bridge.py 身份枚举没有 workbuddy | 更新 `bridge.py`（现版本已含） |
| `检测到状态脱节，已阻止写入` | `state.json` 是旧快照（见 6.2 坑 1） | 确认以哪侧为准，同步 `state.json` 后重试 |
| `找不到 .ai/state.json` | 还没初始化 | `python3 .ai/bridge.py init` |
| `ERROR: 没有任务 \`T-xxx\`` | 任务 ID 打错 | `python3 .ai/bridge.py status` 看真实 ID |
| `check` 报「没有限定 scope」 | 新建任务时没给 `--scope` | 补上，如 `--scope ws/src/40_navigation` |
| 任务卡头部显示 `状态：todo` 但实际已 done | 旧版 bridge.py 不在状态迁移时刷新卡头 | 更新 `bridge.py`（2026-10-17 04:31 起已修） |
| 写文件后 `ros1-slam` 账号说写不动 | 属主漂移成 `ysc:ysc`（见 `note/02_问题与坑.md` 坑17） | `setfacl -m u:ros1-slam:rwx <文件>`；目录还要 `-d -m`。`bridge.py` 已内置，但你自己用 Edit / tee 写文件时要手动补 |
| Agent 还是整篇读 `note/进度/00_总览.md` | 那个文件 800KB，会撑爆上下文 | 强制区段抽取：`awk '/^## Gate 状态/,/^## 已完成阶段/' note/进度/00_总览.md` |

---

## 八、和既有纪律的边界（重要，别越界）

1. **进度真身只有一个**：`note/进度/00_总览.md`（时间线口径）。**别信任何摘要里的旧状态**，
   包括 `.ai/HANDOFF.md` 里的——它只是「最近一次交接时」的快照。
2. **不要整读巨型 note 文档**（`00_总览.md` 805KB、`04_配置改动diff.md` 1.3MB），一律先定位再抽取。
3. **A6 单向流**：绝不反向写入开发侧对照库；`/home/ysc` 只读，要用先复制到本目录。
4. **第三方包原样复制、一字不改**（faster-lio 存盘路径是唯一许可例外，须登记 `note/04`）。
5. **三态通道** PASS / FAIL / INVALID，INVALID 不入串（见 `note/06_三态通道普查.md`）。
6. **P8 计划书同步纪律**：动到话状体 / 判据载体要按 `note/05` F 段回写。
7. **不要在仓库根目录建「任务名」文件**。历史遗留了一批 0 字节垃圾文件
   （`Gate`、`timeout`、`v3`、`ros1-slam`、`lite3_drivers，FAILED`、`**产出**：部署架构师`）。
   进度写进 `note/` 或任务卡，不要建空文件占位。
8. **落盘属主（坑17）**：以 `ysc` 落盘会让属主漂移成 `ysc:ysc`，导致 `ros1-slam` 账号落到
   `other::r--` 写不动。每次编辑后立即 `setfacl -m u:ros1-slam:rwx <文件>`。
9. **拿不准的进裁定单**：`note/进度/00_裁定请求单_2026-10-09.md`，别自己拍。

---

## 九、长期结论写哪儿（两个地方，别混）

| 内容类型 | 写哪里 | 命令 / 格式 |
|---|---|---|
| **项目级结论**（环境坑、参数、判据、标定） | `note/02_问题与坑.md` | 按「坑N 登记制」：标题带 `（新登）`/`（销坑）`/`（未决·回设计侧）` 前缀 + 登记日 + 源于什么 + 症状/根因/处置 + 是否需更新计划书 + 登记人 |
| **工具协作本身的经验**（哪个工具什么脾气、交接怎么写才有效） | `.ai/knowledge.md` | `python3 .ai/bridge.py knowledge "…"` |

前者的读者是四个人 + 计划书，后者的读者是下一个 AI 工具。**不要混**。

---

## 十、为什么不用别的方案

| 方案 | 结论 |
|---|---|
| 互导三家私有记忆 | ✘ 格式闭源 + 厂商随时变更 + 有写坏库的风险 |
| memU（14.5k star, Apache-2.0） | △ 定位正合适，但官方 host 矩阵不覆盖 zcode / qoder，需自写 adapter + 自备 embedding provider |
| MCP 记忆服务（pgvector 类） | △ 两家都支持 MCP，但为一个 ROS 沙箱引入 DB 依赖过重 |
| **共享黑板 + bridge.py** ✔ | 零依赖、纯文本、可 git diff、人和 LLM 都能读、换任何工具都不失效 |

---

## 附：当前状态

```bash
python3 .ai/bridge.py status     # 任务表 + 当前 epoch
python3 .ai/bridge.py check      # 一致性自检
cat .ai/tasks/T-20261017-02.md   # 第三节那个实战例子的完整任务卡
```

跨机器防护的实现细节在 `.ai/bridge.py` 的 docstring 里（`_sync_root` / `epoch_drift`）。

---

## 附二：可选增强与参考外链（★R579／批231 从仓根 README.md 归纳过来★）

> 来历：仓根 `README.md` 的**第一—七节**原本标题是「跨 Agent 协作黑板 — 使用说明」，与本教程正文重复。
> 真人 2026-10-23 明令＝**根 README 应当是对项目整体的介绍，黑板这块归到 `note/tools/`**。
> 本附二把根 README 里**本教程正文没有的**两节原样归纳过来；★上方旧文一字未回改★；有冲突一律以本教程正文为准。

### 附二·一 可选增强

- **zcode 全局提醒**：在 `~/.zcode/AGENTS.md`（用户全局）追加一句
  「进入工作区时，先跑 `python3 .ai/bridge.py resume --as zcode`」，省得每次手动说。
- **git 版本化**：把 `AGENTS.md` 和 `.ai/` 提交进 git（`.ai/.gitignore` 已排除 `.ai/.state.lock`），
  交接记录就有了 diff 历史（做法见第六节 6.1）。
- **文件监听自动拉起**（进阶）：参考 Markdown Blackboard 的 watcher 模式，
  用 `nodemon --watch .ai/HANDOFF.md` 在交接时自动唤起另一个 CLI。
  通常没必要——手动切工具已经够快。

### 附二·二 参考外链

- zcode 官方：项目指令文件 AGENTS.md <https://zcode.z.ai/cn/docs/agents>
- qoder 官方：静态记忆与 Rules <https://docs.qoder.com/zh/cli/memory>
- 黑板 vs 监督者模式：<https://www.agentpatternscatalog.org/patterns/blackboard>
- Agent 交接四模式（Handoff Envelope / Relay Ledger / 显式 ACK / 上下文压缩）：
  <https://askpatrick.co/guides/agent-handoff-patterns>
- memU（日后要跨机／语义检索再升级）：<https://github.com/NevaMind-AI/memU>

### 附二·三 早期部署脚本 `install.sh` 在哪

仓根 `install.sh` 是**早期**的远端部署脚本（用法 `bash install.sh <user@host> <目标项目根>`），
把 `AGENTS.md`／`.ai/`／`.qoder/rules/` 推到远端主机。

★当前推荐走第六节 6.3 的 `ssh + tar` 法★——更直接，且第六节 6.2 坑 1 讲过：
`install.sh` 会跳过已存在的 `.ai/state.json`，跨机首次部署时容易留下「两边 epoch 从 1 开始」的断裂。
`install.sh` **保留仅作历史入口，未删、未改**。
