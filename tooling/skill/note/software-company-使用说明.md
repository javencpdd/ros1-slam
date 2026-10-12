# software-company 使用说明（v3 适配版）

> 适配基准：21 号《ros1-slam 隔离部署计划书 v4》正文 ＋ 已落册补丁 v4.1／v4.2（v4.3 草案待裁）；现行版本以 `note/进度/00_总览.md` 抬头与 `note/00_README.md` 逐批追加为准
> （`/home/ros1-slam/note/计划书v4/21_ros1-slam隔离部署计划书-v4.md`；P0–P2 已按 v3 执行完毕冻结，v4 作用于 P3a 及以后）
> 适配日期：2026-10-04　原始定义备份：`../.orig-backup/software-*.md.orig`

---

## 一、这是什么

| | 适配前 | 适配后 |
|---|---|---|
| 来源 | WorkBuddy「软件开发团队」五人专家包 | 同一套人设与 SOP 骨架，领域重铸 |
| 团队使命 | 从需求到测试通过的软件开发 | **按 21 号 v4 计划书完成 ros1-slam 沙箱的隔离部署** |
| 工作流 | PRD → 架构 → 编码 → 测试 | **前置核对 → 架构产出 → 施工 → Gate 验收**，按 §4 DAG 逐 Phase 流转 |
| 质量机制 | 工程师 IS_PASS 审查 + QA 两轮测试 | **Gate G0–G9 + v3_isolation.sh + 真机安全一票否决** |

核心公式（主理人原话保留）：**部署 = SOP(团队)**；「Gate 不过不前进」取代「测试不过不交付」。

## 二、五角色适配映射

| 成员 | 适配前 | 适配后 | 对应计划书 |
|---|---|---|---|
| 佳文 | Team Lead，SOP 编排 | **主理人**：工作流路由（快速/返工/标准SOP/单项）、成员调度（ZCode Agent 机制）、Gate 铁腕、真机异常冻结权 | §4 DAG、§4.3 |
| 小王 | 产品经理（PRD/竞品） | **计划与风险管家**：Q1–Q7 待确认跟踪、S1–S8 sudo 清单、施工窗口双查、note/ 进度文档与登记维护（进度/00_总览、02/03/04）、变更协调 | §5 P0、§15、P0.4/P0.5 |
| 小张 | 架构师（设计+任务分解） | **部署架构师**：目录骨架与 P1.5 脚手架、C0 配置种子、C1 yaml 转换稿、双目录制守护、diff 登记表 | §3、§6 P1.5、§8 |
| 小明 | 工程师（批量写代码） | **施工工程师**：执行 P0–P2/P5 命令序列、§7.2a 存盘路径改写、分批编译+swap、权限时序、§14 回滚执行 | §5–§7、§10、§14 |
| 小美 | QA（测试+智能路由） | **验收工程师**：Gate 校验表对号执行、真机安全专项（ros2qnx 一票否决）、失败按「谁的锅」路由、最多 2 轮 | §11、§4.3、§13 |

## 三、怎么触发

ZCode 自动触发话题：**ros1-slam 全量部署、多阶段推进、团队协作施工、Gate 失败返工、需要设计/施工/验收相互制衡**的场景。
也可 `/software-company <任务>` 强制加载。

示例 prompt：

1. 「组队按 21 号计划书把 P0–P2 做完，每个 Gate 都要留证据」
2. 「G2 隔离自检挂了，团队排查一下」（→ 返工路径）
3. 「只要验收：当前部署状态出一份 Gate 状态报告」（→ 单项工作流）

**与 senior-developer 怎么选**：明确单阶段任务/单点排查用 senior-developer（轻）；
全量部署、多阶段推进、要制衡和留痕用本团队（重）。

## 四、协作机制的两处平台适配

原包为 WorkBuddy Agent-Team 机制（TeamCreate + `subagent_type: software-architect` 直派）。
ZCode 下由主理人适配为：

1. **成员 spawn**：Agent 工具（`general-purpose`），prompt 固定含「你是<成员名>，先读
   `/home/ysc/.zcode/skill/software-company/agents/<文件名>`」+ 本次任务上下文
2. **消息中转**：成员经最终回传把产出交回主理人，主理人汇总转下一棒（原 SendMessage 直连语义由主理人中转等价实现）

铁律不变：只有主理人建团队；成员产出必须由成员自己给；跨成员信息必经主理人；禁止代写。

## 五、文件结构

```
software-company/
├── SKILL.md                              # 入口与路由（v3 新增，ZCode 发现用）
├── agents/主理人-佳文.md          # ★ 主理人（v4 现行）
├── agents/计划与风险管家-小王.md    # ★ 计划与风险管家（v4 现行）
├── agents/部署架构师-小张.md          # ★ 部署架构师（v4 现行）
├── agents/施工工程师-小明.md           # ★ 施工工程师（v4 现行）
├── agents/验收工程师-小美.md        # ★ 验收工程师（v4 现行）
├── MANIFEST.md / MIGRATION.md            # 原包清单与迁移指南，未改动
└── .codebuddy-plugin/ .downloaded_at     # 原包元信息，未改动
```

## 六、ZCode 发现路径说明

本包实际位于 `~/.zcode/skill/software-company/`（单数 skill，ZCode 不扫描）。
已在 `~/.agents/skills/software-company` 建立符号链接接入发现路径。
不触发时检查：

```bash
ls -l ~/.agents/skills/software-company/SKILL.md
head -5 ~/.agents/skills/software-company/SKILL.md
```

恢复链接：`ln -sfn ~/.zcode/skill/software-company ~/.agents/skills/software-company`

## 七、全团队红线（写进主理人与各成员定义）

1. **真机安全**：`ros2qnx` 仅真机 mode；stub/采图/回放模式出现即冻结上报（V1/R13）
2. **隔离**：`/home/ros1-slam` 内禁 `/home/ysc` 字符串（note 来源记录与 opt/ref 除外）；永不复制 build/devel/.git；永不删除 `/home/ysc/lite_cog/`
3. **顺序**：G3（地图）→G8（避障）→G9（mux）不全，不碰真定位/对比实验/第二速度源/SCAN-PCT
4. **证据**：一切 PASS/FAIL 必须附实际命令输出
5. **落档**：每完成一个阶段/任务必须按「note/ 文档架构」（SKILL.md）在 `note/进度/<阶段>/` 落进度文档与证据（2026-10-04 重组后生效）

## 八、回滚

五个 agent 定义与 SKILL.md 均可从 `../.orig-backup/` 恢复；MANIFEST.md 记录的原始包
完整性基线（字节数/SHA256）不受本次适配影响——如需回到「原厂 Web 开发团队」，从备份
拷回即可。
