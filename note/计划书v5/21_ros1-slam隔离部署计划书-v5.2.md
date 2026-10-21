# 21 · 103 主机 `ros1-slam` 用户：模块化架构隔离部署计划书（****v5.2****）

> **状态：计划书（Plan）。v4 = v3 的设计模式修订版**——设计渊源：ROS 2 navigation2 联邦式架构对标分析的**八个已验证设计决策**，ROS 1 落地映射见 **§2.6**。**P0–P2 已按 v3 执行完毕（2026-10-04，G0/G1/G1.5/G2 全 PASS，见 `note/进度/00_总览.md`），正文相应章节冻结注记（N9）；v4 变更仅作用于 P3a 及以后。本文不执行**：不创建目录、不复制文件、不修改 103 上任何内容；**S9 仅登记为 sudo 清单项**（§5 P0.4 / §15 Q9），不把"安装"写成已完成。
> 基线来自 2026-10-04 对 `lite3-f20-1-103`（192.168.1.103，即本机）的二次实测 + **P3a.0 导航栈组件只读实测**（`/tmp/P3a0_mbf组件实测.log`，690 行——**mbf 消息字段名、插件 type 字符串、参数名的唯一权威来源，引用以它为准**）。
> v3 骨架（阶段 DAG、Gate、双目录制、回滚矩阵、风险表）全部保留；v4 落位 `note/计划书v4/`（新目录；v3 原文件一字不动，N10）。
> **v5 ＝ v4 的综合统一收编版**（主理人 2026-10-12 07:44 明文「请将目前的计划书综合统一落册，并作为第五版计划书（v5）」）：收编 **v4 正文（984 行，含 §16.1–§16.11）＋ v4.7 补丁稿 Z1–Z14**（＝主理人 2026-10-12 的 24 个勾位裁定）。**v4 与 v4.1／v4.2／v4.3／v4.4／v4.5／v4.6／v4.7 一律转历史件、原文一字不动**；v5 只改需要改的格子，未涉及者与 v4 逐字相同。
> **状态三值（文内每条裁定必带其一，缺标记即读作未落）**：`已生效`＝正文所述与盘上一致、可当场复跑；`已裁待施工`＝主理人已明文但施工未落地（**不得读成已决事实**）；`未决`＝仍等明文。
> **v5.2 ＝ v5.1 ＋ v5.1.1 补丁稿（W1–W5）的收编版**（主理人 2026-10-12 19:18 两轮明文：A34 三条裁定①甲／②甲／③乙，以及"暂停授权不入技能包、改入计划书"的更正）。**v5.1 与 v5／v4 系、两份补丁稿一律转历史件、原文一字不动**；现行真身＝本件。**新增 §0.11（工作暂停授权与上报）为计划书条文**；§0.8 末两行给出 A34 与暂停授权的落地态。**技能包内不再有任何暂停授权口径**（已还原）。
> **v5.1 ＝ v5 按主理人评审优化表（R01–R44）的修订收编版**（主理人 2026-10-12 08:30 明文"根据该优化表，将 v5 计划书修改为 v5.1 并落定"）。**v5 与 v4 系全部转历史件、原文一字不动**；**现行真身＝本件**。**先读 §0.6**（生效序：§0.6–§0.10 ＞ §16 各格 ＞ 一期正文 §1–§15）；本件对一期正文**只逐字改写了 5 处**（详见 §0.6 部分落地声明），其余以 §0.7 逐条处置表＋§0.8 有效态速查**替代**或**登记待施工**——凡标"待施工"者盘上未落地，不得读成已修。

---


---

## §0.6 v5.1 修订依据与生效序（**本章为 v5.1 的入口，优先级最高**）

> **来历**：主理人 2026-10-12 明文「我对 v5 计划书加了一个评审优化表，你根据该优化表，将 v5 计划书修改为 v5.1 并落定」。
> **唯一修订依据**＝`note/计划书v5/21_ros1-slam隔离部署计划书-v5_评审优化表.md`（114 行，**R01–R44 共 44 项**，其中标高优先 **28 项**；含依据索引 B40–B47、只读抽查 E1–E8/W1）。
> **生效序（写死，治 R36 的"两句话争夺优先级"）**：① 本节 §0.6–§0.10 ＞ ② §16 各格 ＞ ③ 一期正文 §1–§15。凡一期正文条目被列入 §0.7"已失效／已替代"栏者，**不得再据以施工**；一期正文的原句保留在文内**只作历史与设计脉络**，不作操作依据。
> **部分落地声明（不假装全改）**：本批**逐字改写正文的只有 5 处**（§1 A11／§1 关键事实行／§8 C1 launch 形态／§16.4 A25 两句的次序与指代／§17 状态行——都是"照抄即失败或指代已错"级）；其余 39 项**或**收进 §0.8 有效态速查（以新表替代旧句、不改写旧句），**或**在 §0.7 标为「待施工」并绑定批次，**或**标「需主理人补明文」。**未被本轮"验证"过的事项一律不写成已修**（Y6／规矩⑨）。
> **核实纪律**：§0.7／§0.8 里的每条现值都是**本会话只读抽查所得**（命令与结果见 §0.10），不是照抄评审表；**未做**编译、未起 ROS 节点、未起活体窗、未做故障注入——所以凡涉及"运行态／验收通过"的判断，本节只写"文件值确认，不证明已加载"（评审表 E2 自己的口径，照用）。


## §0.11 工作暂停授权与上报（**v5.2 新章；本条是计划书条文，不在技能包**）

> **来历**：主理人 2026-10-12 明文「在当前终端 foxy 残留多，或在负载大的情况下，我允许你暂停工作」；随后 19:2x 更正「**不应写入 skills，应写入计划书 v5**」⇒ 施工侧已把技能包那两处撤回（`software-company/SKILL.md` 134→**120 行**、`agents/主理人-佳文.md` 101→**97 行**，用改前逐字备份 `~/.zcode/skill/.orig-backup/pre_r225_20261012_1133/` 整体还原；**同批唯一的例外**＝呈报格式②段"复选框逐条＋每条写意义与代价"，那属另一条明文，保留）。本条自此为**在册规矩**，落点＝计划书（不另立技能包口径）。

**授权（可不经逐次请示直接停手＝暂停，不是改流程）**，两种情况其一成立即可：

| 情况 | 判据（当场可核） |
|---|---|
| **① 环境混装（foxy 残留多）** | 走完 `scripts/env.sh` 之后，`ROS_PACKAGE_PATH`／`PYTHONPATH`／`LD_LIBRARY_PATH`／`CMAKE_PREFIX_PATH` 中任一仍可扫到 `/opt/ros/foxy` 或 `/home/ysc` 生产路径段；或 `source` 时回显 `ROS_DISTRO was set to 'foxy'` 警告 |
| **② 负载过大** | 1 分钟 `loadavg` 高于活体窗门槛（现值 **4**）；或榜首高耗进程**属主不明／属他人常驻基线**（非本会话所起） |

**停手时必须同时做三件事**：
1. **当场留证**——`date '+%F %H:%M'` ＋ `uptime` ＋ `ps -eo pid,user,pcpu,etime,args --sort=-pcpu | head`，负载类**必须点名榜首 PID／属主／命令行**（不点名＝没有归因；本会话实测教训：曾把编辑器索引器的 126% CPU 误读成"环境本来就这么忙"）；
2. **上报主理人**——一句话说清是哪一种情况、停了哪件事、要他做什么（停服务／关窗口／给一次明文）；
3. **账面停在诚实状态**——该批只报"施工完成／未收口"，**不得把暂停写成完成**，不得拿"没跑成"的读数当结论（`FORCE=1` 一律禁用，O-5；硬闯同禁）。

**边界（写死）**：本授权只覆盖"**停**"，不覆盖"**改**"——门值／判据／门禁串组成／配置／他人进程与服务／他人志文全部仍需主理人明文；需要放宽门槛或改环境，走请求单（A15／#54／#16／#12 那族）。
**与在册其他条目的关系**：本条是 §5 P0 前置段"施工窗口双查"的**续件**（双查决定"能不能开"，本条决定"开着开着发现不该开就停"），也是 A29 那条"出现大负载源⇒暂停并上报"的**一般化**（原句只覆盖仿真测试，现覆盖一切活体作业）。

## §0.7 R01–R44 逐条处置表（评审表 → 我的核实 → 处置）

| # | 核实结果（现值／出处） | 处置 | 落点／批次 |
|---|---|---|---|
| **R01** | 成立且更严重：`Mid-360` 在 v5 正文出现 **0 次**、`livox` 仅 1 次、`lslidar` 11 次、`2368` 3 次、`rslidar_points` 8 次；驱动实存 `ws/src/40_navigation`… 实为 `ws/src/00_drivers/livox_ros_driver2` | **已替代**：§0.8 首三行为现行有效态；一期 §1/§2/§7.5 的 C16／UDP 2368／lslidar 链降为**遗留对照，不供实机选用** | §0.8；不改写旧句 |
| **R02** | 成立：`S9` 在 v5 出现 **23 处**，§1 A11 原句写"需 S9…**不批则整体回退 move_base**"；E4 复核 mbf 头文件确含 planner/controller/recovery 三对加载器 | **已改正文**：A11 整行重写为"S9 不作前置（v4.1 X3 已裁不装不卸）；已装 0.3.0 只登记存在；回退仅由真实联调阻塞触发" | §1 A11（逐字改写） |
| **R03** | 成立：v5 §8 C1 那行确写 `type="mapping_online"`；盘上 `ws/src/10_slam/faster_lio/app/CMakeLists.txt:1` ＝ `add_executable(run_mapping_online ...)` | **已改正文**：统一为 `run_mapping_online`＋附"可执行存在性检查"一句 | §8 C1（逐字改写） |
| **R04** | 成立（继承自 v3/v4）：值 `lidar_type: 4` 对、解释错（应为 C16 处理分支，不是 Velodyne 分支） | **已改正文**：§1 关键事实行补"枚举归属＝C16 处理分支；硬件身份与处理分支不得混同（v4.1 X4）" | §1（逐字改写） |
| **R05** | 成立：v5 复制清单无 Livox 驱动与 SDK2 两项；`/usr/local/lib/liblivox_lidar_sdk_static.a` 实测在位（28,395,498 字节，root，2025-09-16） | **已登记待施工**：§0.8 记驱动与 SDK 实存路径＋"缺则构建安装"条件本次不触发；补进 §7 复制清单属施工批 | §0.8；施工单 **T-R05** |
| **R06** | 成立：`ROS_EDITION` 在 v5 正文 **0 次**；盘上 `ws/src/00_drivers/livox_ros_driver2/CMakeLists.txt:2` ＝ `if(ROS_EDITION STREQUAL "ROS1")` | **已登记待施工**：Livox 构建必须显式 `-DROS_EDITION=ROS1` 并留 CMakeCache 值／产物核对；§0.8 已写入有效态 | §0.8；施工单 **T-R06** |
| **R07** | 成立：现役 `faster_lio_outdoor.yaml:3` ＝ `imu_topic: /livox/imu`（注明 Q12 因 43897 被 jetson2motion 占用而切）；v5 正文仍写 `/imu/data` 4 处 | **已替代**：§0.8 记 IMU 有效态＋工厂初始外参配套；历史主选另注 | §0.8 |
| **R08** | 成立：v5 Q 表仍留 Q9 开项、无 Q12；R20–R23 未入风险表 | **已登记待施工**：合并"当前 Q/R 表"属结构重排，列入 **T-R08**（需保持每项绑验证阶段与证据） | 施工单 **T-R08** |
| **R09** | 成立：v5 无 §11.6 双层回归入口（只有一句"正式回归工具"） | **已登记待施工**：规划层 `mode_nav_sim`＋`sim_nav_smoke`／物理层 Gazebo·M2 两层入口与触发规则，列入 **T-R09** | 施工单 **T-R09** |
| **R10** | 成立：M2 支线步骤与定量判据未并入；"P6 3.5~5.5h"覆盖不了 | **已登记待施工**：**T-R10**（含"历史 M1③ FAIL 不得被总体 PASS 掩盖"的读法约束） | 施工单 **T-R10** |
| **R11** | 成立（表述级）：三层能力（规划／物理／执行接触）未分列 | **已替代**：§0.8 末段给三层能力边界一句；旧句保留作历史 | §0.8 |
| **R12** | 成立：开／关窗红线在 v5 分散、未成表 | **已登记待施工**：**T-R12**＝单一检查表（11313 sim 域、保护 11312/11314/11346、本 boot 现取、样张 sha 前后同、退净、`FORCE=1` 禁用）；门禁侧现值见 §0.8 | 施工单 **T-R12** |
| **R13** | 成立：安全前置只挂 P6/P7，未挂 P3b 与"任何驱动/运动桥启动" | **已登记待施工**：**T-R13** | 施工单 **T-R13** |
| **R14** | 成立：§17 原句"未执行：P3a 及以后全部阶段（本文不执行；S9 待 Q9 批准，未装）"与在册 Gate 现状冲突 | **已改正文**：§17 该行重写为三档记录（历史设计冻结／已施工完成（引 `00_总览`，**引用≠本次重跑**）／已裁待施工）＋ Q 表现值 | §17（逐字改写） |
| **R15** | 成立：历史命令与当前步骤未隔开，照抄重跑有覆盖现役配置风险（现役 yaml 已含 Q6/Q12/C13 的改动） | **已替代**：§0.6 生效序＋§0.8 有效态即"续施工入口"；一期 §5–§7 命令只用于**从零重建**，续施工一律走增量核验 | §0.6／§0.8 |
| **R16** | 成立（中优先）："同 v2／继承 19、20 号"缺精确锚点 | **待施工**：**T-R16** 依赖文档清单（路径＋版本/sha＋锚点＋用途） | 施工单 **T-R16** |
| **R17** | 成立（中优先）：SDK 副本 555 只读＋ros1-slam 无 sudo 却给 `sudo make install` ⇒ 字面流程不可执行 | **待施工**：**T-R17** 改 out-of-source＋构建/安装身份分离 | 施工单 **T-R17** |
| **R18** | 成立：现役配置已启 `self_mask_box_on: 1`＋盒参数，而正文仍写"第三方源码只改存盘路径"；C13 已改过 fork 源码（R137） | **已替代**：§0.8 记"零源码改动"自 R137 起不成立（`so=147dc8cc`）；例外表列入 **T-R18** | §0.8；施工单 **T-R18** |
| **R19** | 成立（高优先）：隔离检查以 ROS_PACKAGE_PATH 字符串为主，未全核 PYTHONPATH／LD_LIBRARY_PATH／CMAKE_PREFIX_PATH | **待施工**：**T-R19**（在干净 Noetic 会话记录变量全表＋rospack/import/库解析路径；不以"删文本"证明隔离） | 施工单 **T-R19** |
| **R20** | 部分成立：`/tmp` 本次由 ext4 根盘承载（`df` 现测），与"必为 tmpfs"不同；swap 固定路径缺挂载/容量前置 | **待施工**：**T-R20**；**评审表"不宣称 /tmp 当前为 tmpfs"这一条我照用**（与本会话早上 jack 本机 /tmp 是 100M tmpfs 不冲突——不同主机，见 §0.10） | 施工单 **T-R20** |
| **R21** | 成立：现役 `pcd_save.interval: -1`（第 29 行，Q6 翻案说明在位），正文仍写 10 | **已改正文**（关键事实行一并）＋§0.8 记有效态；10 降为历史值 | §1／§0.8 |
| **R22** | 成立：`find ws -name 11_postprocess.launch` **零命中**；在位者为 `ws/src/10_slam/pcd2grid/launch/pcd2grid.launch` | **已登记失效**：§0.8 标该入口不存在；合法替换入口需版本化并验收 → **T-R22** | §0.8；施工单 **T-R22** |
| **R23** | 成立（中优先）：G3 三件套校验缺批次/sha/字段/连通性/恢复断言 | **待施工**：**T-R23** | 施工单 **T-R23** |
| **R24** | 成立：现役 `mbf_navigation.launch:8` 已是发布节点内 `<remap from="cmd_vel" to="cmd_vel_raw"/>`；`cmd_vel_raw_relay.launch` 在位但按 Q11 未确认前 node 行为注释 | **已替代**：§0.8 记"唯一写者"以 M6 判据为准，relay 不作旁路 | §0.8 |
| **R25** | 成立（中优先）：回退称"纯配置切换"但含 action 契约/客户端接线差异 | **待施工**：**T-R25** | 施工单 **T-R25** |
| **R26** | 成立：recovery 退空兜底与 G3a"三列表非空"互斥 | **待施工**：**T-R26**（按已选形态判定，不能同时要求空与非空） | 施工单 **T-R26** |
| **R27** | 成立：SMACH 骨架 execute 恒返 succeeded、retry 无计数示例、clear 未真调 | **待施工**：**T-R27**（标"静态契约原型"＋现役实现版本指针＋四路径验收） | 施工单 **T-R27** |
| **R28** | 成立：自检 [5][6][9] 属文本级、证不了运行态唯一性与发布归属 | **待施工**：**T-R28**（补语义审查＋运行态发布者核对＋正反用例；**不私自降低旧门**） | 施工单 **T-R28** |
| **R29** | 成立：独立 1Hz 心跳不能证明 mbf/LIO 活着 | **待施工**：**T-R29**（心跳绑受监督进程/数据新鲜度/action 响应；补"算法死亡、心跳仍在"反例） | 施工单 **T-R29** |
| **R30** | 成立（中优先）：正文 SUP 骨架缺 ready 首跳/宽限/hold 复位，现役文件已有这些 | **待施工**：**T-R30**（用现役版本指针替旧全文） | 施工单 **T-R30** |
| **R31** | 成立（中优先）：`rospy.Timer` 随 ROS 时间 ⇒ 仿真时钟停滞时"3 秒超时"≠墙钟 3 秒（评审表标为源码推导、未做注入） | **待施工**：**T-R31**；**记为"源码推导、未故障注入"，不得写成已证** | 施工单 **T-R31** |
| **R32** | 成立：撤销污染档后 v5 未列四类 INVALID 前置单一判读表 | **待施工**：**T-R32**＝四类（扰动入世界／占据图可见／goal 被 FSM 消费／首规产出）逐项字段＋退出码＋分母；**照 Z1 保留超幅值记 FAIL** | 施工单 **T-R32**（并入 #111 同批） |
| **R33** | 成立且**处置方向明确**：混合 stdout 是已裁取舍、不是缺陷；但缺消费者读取契约 | **待施工**：**T-R33**＝新形状加版本标识＋判据/字段提取入口，覆盖所有 `json.loads` 类消费者；**历史分件按原格式读、R220 后按新格式读，不覆写旧留底、不擅自恢复分件** | 施工单 **T-R33** |
| **R34** | 成立：A23 缺单位/坐标系/等号边界/NaN/非法 env 值/最终判定式 | **待施工**：**T-R34**＝env→参数→字段→布尔判定单一定义链＋等值两侧用例＋撤例；**并入 #111，落地前判定链不动** | 施工单 **T-R34**＝**#111** |
| **R35** | 成立：七件／八件／VCC 也入门 → 最终组成有歧义 | **已替代**：§0.8 给**三态 manifest**（当前在册＝七件；获准目标＝**九件**（账构＋VCC 各 +1）；已落地＝七件）⇒ **"八件"不是终值** | §0.8；#110／#112 |
| **R36** | 成立，且**我在 v5 里新造了一例**：Z4 新句插在旧句"＝待裁"**之前**、句中还写"**上句**属裁定前状态"⇒ 指代反了 | **已改正文**：v5.1 内把两句次序调正、指代改对；此缺陷另起句入 `02`（坑60/62 同族） | §16.4（逐字改写）＋坑册 |
| **R37** | 成立：`未决＝无` 只在"24 勾位"范围内成立 | **已改正文**（§17 行）＋§0.8 末段列工程未决清单（O-11／C5·C6·C11 定档／阈值量纲／真机几何／恢复路线／#103 验证） | §17／§0.8 |
| **R38** | 成立：Y4 三条还原纪律未入 v5 正文 | **待施工**：**T-R38**＝"开窗前读数为来源→写回→回读一致→跨窗先查，非默认停报"＋`before/set/server/client/restored` 五项缺一即该窗 INVALID 不进分母；**#103 仍标返工待验** | 施工单 **T-R38**＝**#103**／**#117** |
| **R39** | 成立（中优先）：责任映射表被压缩成名册一句 | **待施工**：**T-R39** | 施工单 **T-R39** |
| **R40** | 成立（中优先）：1/6 历史分布、11 样本两阈、Z1 后新分母并列未标版本 | **待施工**：**T-R40**＝报数附样本集／单位／取证件 sha／判据版本／有效与无效分母 | 施工单 **T-R40** |
| **R41** | 成立：X30 均匀外扩限制、腿部包络、观测距离窗、X19 零标注边界未入 | **待施工**：**T-R41**；**0~0.20m 离线上界不得写成真机推荐值**（与 §16.10 现格一致） | 施工单 **T-R41** |
| **R42** | 成立：H 段三条"已裁待施工"，续航/温度/见证的单次合测编排不完整 | **待施工**：**T-R42**＝独立真机作业单（H2 INVALID 记法／H3 温度前后测／H5 本 boot 快照＋覆盖面＝4；**量具来源≠实测值，Q13 待读数**） | 施工单 **T-R42**＝**#115**／#16 |
| **R43** | 成立：`/tmp/P3a0_mbf组件实测.log` 现测**不存在**；归档件 `note/进度/v7/S0/计划书v4升级/证据/P3a0_mbf组件实测.log` 在位（27,370 字节，10-04 18:47） | **已替代**：§0.10 定为唯一权威证据入口规则；短名对照表列入 **T-R43** | §0.10；施工单 **T-R43** |
| **R44** | 成立：一期 DAG 与 #110/#111/#112/#116 及四扇窗无整合排程；旧 28.99 易被当现值 | **已替代**：§0.9 给出六步排程与批次映射；**每窗现取负载、不沿用旧值；远角不因 sim 例外开放** | §0.9 |

## §0.8 现行有效态速查（**照抄即用的那一页**；全部为本会话只读实测）

| 维度 | 现行有效值（核实方式） | 与一期正文的关系 |
|---|---|---|
| 传感器身份 | **Livox Mid-360**（v4.1 X1/X2 翻案在册） | 一期 §1/§2 的 C16／lslidar／UDP 2368 链＝**遗留对照** |
| 驱动与构建开关 | `ws/src/00_drivers/livox_ros_driver2`，`CMakeLists.txt:2` ＝ `if(ROS_EDITION STREQUAL "ROS1")` ⇒ **必须显式 `-DROS_EDITION=ROS1`** | 正文 0 处提及 ⇒ 见 T-R06 |
| SDK | `/usr/local/lib/liblivox_lidar_sdk_static.a` **在位**（28,395,498 B，root，2025-09-16） | S10 的"缺则构建安装"条件**本次不触发** |
| IMU 源 | `faster_lio_outdoor.yaml:3` ＝ `imu_topic: /livox/imu`（Q12 因 43897 被 jetson2motion 占用而切） | 正文的 `/imu/data` ＝历史主选 |
| 地图存盘 | `pcd_save.interval: -1`（第 29 行；Q6 翻案：`10`＝每 10 帧写盘后清空 ⇒ 截断） | 正文的 `interval=10` 已失效 |
| 近场过滤 | `blind: 0.2`（第 32 行，C13＝乙；语义＝**兜底小球**，`1.0` 口径作废）＋`self_mask_box_on: 1`＋躯干盒 `[0,0,-0.38,0.21,0.155,0.11]`＋`margin: 0.05`＋`drop_invalid: true` | **"零源码改动"自 R137 起不成立**（被测件面 `so=147dc8cc`）；`0.2` 罩不住躯干/腿 ⇒ **不得写成自遮挡保护到位** |
| 建图可执行名 | `run_mapping_online`（`faster_lio/app/CMakeLists.txt:1` 实测） | 正文 `type="mapping_online"` 已就地改正＋要求存在性检查 |
| 导航 launch 实际在位 | `ws/src/40_navigation/lite3_navigation/launch/{mbf_navigation.launch, cmd_vel_raw_relay.launch, sink, stub}`；`lite3_scan/launch/{70_scan.launch, lio_relay_sim.launch}` | 正文出现的 `40_navigation.launch` 该名**不存在** |
| 速度出口 | `mbf_navigation.launch:8` ＝ 发布节点内 `<remap from="cmd_vel" to="cmd_vel_raw"/>`；relay 件在位但 node 行按 Q11 未确认前注释 | 唯一写者判据仍以 **M6** 为准，不用"转发到 raw"替代"移除旁路写者" |
| 后处理入口 | **`11_postprocess.launch` 盘上不存在**；在位的是 `ws/src/10_slam/pcd2grid/launch/pcd2grid.launch` | 该入口标失效，替换须版本化并验收（T-R22） |
| 门禁串三态 | **当前在册＝七件**（SYN／GTC／ISO／DC／`v_py_static` 范围甲／`v_evidence_reachability`／`v_verdict_field_parity`）；**获准目标＝九件**（Z3 账构＋Z5 VCC 各 +1，均未落地）；**已落地＝七件** | 治 R35：**不得把"八件"当终值**，也不得自称已入 |
| 取证形状 | Z1／Z2 已生效：判据行与 JSON 同走 stdout、**整份非合法 JSON**；`dump()` 保单一真值；超幅值系框失真轮次记 `FAIL` 并进 n=3 分母 | 消费者读取契约＝T-R33（未落地前不得改读法惯例） |
| Gate 现状（引 `note/进度/00_总览.md`） | G0／G1／G1.5／G2／G3a／G3／G4／G5／G6／G7 **PASS（在册记录）**；**G8／G9 未到**；P7 放行≠终验 | **引用在册记录≠本次重跑验收**（本会话未起任何活体窗） |
| 三层仿真能力边界 | 规划层＝可验决策/几何/代价/规划；物理层＝历史曾支持 LIO 结论（限相应轨迹与姿态条件）；执行/接触层＝**仿真不覆盖**（刚体盒无腿、无近地回波） | 治 R11：hdl 未覆盖 ≠ 算法失败；踏步/落脚/摩擦归真机 |
| **A34 三条（v5.2 收编）** | ①甲＝SDK 构建 out-of-source＋安装身份分离（**待施工 #120**）；②甲＝ISO [6] 运行态核对＋G6"杀算法保心跳"植入用例（**要起窗、待 #117 家族**）；③乙＝ISO 新增第 **[10]** 项全变量与库解析核对（**待施工 #121**；**存量面实测 11:34：四变量 foxy／`/home/ysc` 残留 0／0／0／0、`ROS_DISTRO=noetic` ⇒ 入门不会一片红、不需先建豁免表**，落地前须在主理人交互终端复测一次） | 三条均为**已裁待施工**，本件正文未改任何判据／门值／门禁串；详见 `22_…v5.1.1…` 补丁稿 W1–W3 |
| **暂停授权（v5.2 起）** | 见 **§0.11**：混装或高负载可**直接停手＋上报＋账面停在未收口**；只授权"停"、不授权"改"。**19:16 现跑首次行使**：loadavg **14.93**、榜首为 `run_mapping_online` 118%／`uvicorn` 92.1%／`ros_bridge_node` 69.7%／`whoopsie` 69.3%／`livox_ros_driver2` 48.6%（**均属他人常驻基线，本会话链进程 0、11311–11319 零监听**）⇒ 活体窗全部停摆 |
| 工程未决清单 | **本轮 24 勾位＝无待补明文**；工程未决另有：O-11 载体保真度、C5／C6／C11 定档、X5 阈值量纲、真机几何（H4①＋Q13）、恢复路线（R25／R26）、#103 返工验证 | 治 R37：**"裁定齐"≠"缺口齐"** |

## §0.9 整合排程（治 R44；评审表建议顺序 → 批次映射）

1. **事实与恢复点对账**（零窗）：§0.7 里 T-R05／R06／R18／R22／R43 的来源·sha·例外表补齐 → 批次 **#115／#116**＋新施工单 **T-R05·T-R06·T-R18·T-R22·T-R43**。
2. **隔离与静态可执行性**（零窗）：纯净 Noetic 会话记录变量全表＋rospack/import/库解析（T-R19）；开/关窗单一红线表（T-R12，含 `FORCE=1` 禁用与端口分配）；可执行名/入口/存在性检查（已就地改）。
3. **v5.1 增量与门禁 manifest**：**#110**（±40／账构入串三参数／行首自测锁／ISO＋GTC 双向例）→ **#112**（VCC 入门四件套＋全局同判＋撤前置）；**七件串在成员逐个验收通过前仍按现行清单执行**；#103 的 before/restore/readback 须在下一扇窗之前成可执行件。
4. **两层仿真回归**（T-R09／T-R10）：先以现有证据判定哪些已完成，只补差项；每窗现取负载与归属，**保留全部 FAIL／INVALID、不反复取样直到出现 PASS**。
5. **关闭 P4 关键缺口**（**#111**＝T-R32＋T-R34；**#117**＝sim 面取 outcome＋扫 `P4_SEED`＋扩 goal＋StaticLayer 四类补测）：报数带版本·分母·字段；P4 达成条件满足后才进真机排程。
6. **G8→G9 真机合测**（T-R42＝**#115** 的 H 段三条＋现场几何）：温度/电量/断链/见证/退净全留证；2.5D·多层·PCT 按已定恢复条件另排，不挤进关键路径。

## §0.10 本批核实凭据与权威证据口径（治 R43）

- **本批只读实测**（逐条可复跑）：`grep -c` 对 v5 正文的形状计数（S9=23／`Mid-360`=0／`livox`=1／`lslidar`=11／`2368`=3／`rslidar_points`=8／`ROS_EDITION`=0／`mapping_online`=2／`11_postprocess`=1／`40_navigation.launch`=1／`待裁`=7／`未决`=3）；`ws/src/10_slam/faster_lio/app/CMakeLists.txt:1`；`ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml` 第 3／29／32／58—61 行现值；`ws/src/00_drivers/livox_ros_driver2/CMakeLists.txt:2`；`/usr/local/lib/liblivox_lidar_sdk_static.a` 大小与属主；`mbf_navigation.launch:8` 的 remap；`find ws -name 11_postprocess.launch` 零命中；`/tmp/P3a0_mbf组件实测.log` 不存在、归档件 27,370 B 在位。
- **权威证据口径**：正文凡引 P3a.0 组件实测，一律指向 **`note/进度/v7/S0/计划书v4升级/证据/P3a0_mbf组件实测.log`**（在位）；`/tmp` 下的同名副本只作**历史缓存位置**，不作引用源（已不存在）。
- **评审表快照**：原 v5 快照 1000 行，SHA-256 `7191dc9a5fcc06435e6f267ebc256c9472101e286d110885a5bb16214c01c02f`（评审表自述，**我未独立复核该 sha**——v5 现件行数 1000 与之一致，sha 待我本批算入留底后再对）。
- **未做**：编译、起节点、起活体窗、故障注入、真机验收。故本表不宣布任何验收通过；文件值确认 ≠ 节点已加载（评审表 E2 口径照用）。

## v3 变更日志（相对 v2，按严重程度排序）—— v4 原样保留，只增不改

| 级别 | # | 变更 | 原因（实测证据） |
|---|---|---|---|
| 🔴 安全 | **V1** | **`message_transformer` 拆分启动**：`ros2qnx`（订阅 `/cmd_vel` 直发 QNX 底盘，`ros2qnx.cpp:112`）只在真机 mode 启动；stub/mapping/replay mode **严禁**。M0 一期只起 `qnx2ros`（产 `/imu/data`，`qnx2ros.cpp:53`） | v2 的 `mode_nav_stub` 起完整 M0 会把 stub 假速度真发到底盘 |
| 🔴 事实 | **V2** | **faster-lio 存盘路径硬编码写穿隔离**：`laser_mapping.cc:751` 硬编码 `/home/ysc/lite_cog/system/map/lite3.pcd`，复制后建图会直接写回 ysc。处置：复制后立即改写为 `/home/ros1-slam/data/maps/`，并作为**唯一许可的第三方包源码改动**登记 diff | 实测源码；v2 隔离检查只扫 `lite3_*` 自研壳，扫不到第三方包内部——盲区已用自检 [8] 补上 |
| 🔴 事实 | **V3** | **`faster_lio_online.yaml` 是 ROS2 格式**（`/faster_lio_online: ros__parameters:`）。v2 清单第 16 条"ROS1 格式可直接用"为误判。v3 给出完整 ROS1 转换版（§C1） | 实测文件头部；这正是 v2 自己 R7 警告的坑 |
| 🔴 事实 | **V4** | **`lidar_type: 4`（Velodyne 分支）**，非 v2 所说"用 2"。全 yaml 见 §C1，勿"修正" | `faster_lio_online.yaml` 实测 + `pointcloud_preprocess.cc:20` switch 对照 |
| 🔴 事实 | **V5** | **M0→M1 话题断链**：faster-lio 订阅 `lid_topic: /rslidar_points_filtered`，但该话题全库**无生产者**（驱动只发 `rslidar_points`）。处置：转换 yaml 时改为 `/rslidar_points` | 全库 grep 零命中 + `lslidar_driver.cpp:159` |
| 🟠 流程 | **V6** | **P3 时序修正**：建图需要已编译的工作区，v2 却把"采第一张图"放在编译（P5）之前。拆为 P3a（纯写配置）与 P3b（P5 后现场采图），G3 移到 P3b | 逻辑硬伤 |
| 🟠 流程 | **V7** | **新增 P1.5 自研壳脚手架**：批 5 要编译 8 个 `lite3_*` 包，但 v2 从无一步生成它们的 package.xml/CMakeLists | 逻辑硬伤 |
| 🟠 流程 | **V8** | **新增 C0 配置种子**：C2/C3 要修改的 `planner_teb.yaml`/`costmap_common.yaml`，v2 从未提供底稿来源。P3a 第一步从 `opt/ref/` 拷入自研壳再改 | 逻辑硬伤 |
| 🟠 流程 | **V9** | **权限块时序修正**：`chown -R` 移到 P2 复制完成之后（v2 标注"Phase 1 执行"，复制还没发生，白做） | 逻辑硬伤 |
| 🟠 脚本 | **V10** | **`v2_isolation.sh` 重写为修复版（8 项）**：修 [2] `grep|head && FAIL=1` 恒真 bug、[5] 管道子 shell 丢 FAIL bug；新增 [8] 全 ws 扫描（补 V2 盲区）；`env.sh` 加 devel 存在性守卫 | v2 脚本 [2] 永远 FAIL、[5] 违规检测不到 |
| 🟡 冲突 | **V11** | **ACL 与 chmod 冲突处置**：v2 §3.3 的 `chmod 750/755/644` 会把本机已有的 `user:ysc:rwx` ACL mask 压成 r-x，静默废掉 ysc 写权限。P0 先 `setfacl -b -R` 清干净，按 v2 纯属主模型走 | 2026-10-04 上一轮操作给 `/home/ros1-slam` 设了默认 ACL |
| 🟢 优化 | **V12** | **不复制、不编译 `teb_local_planner`/`move_base`/`map_server`**：三者系统已装（`/opt/ros/noetic/share/`，teb 版本 0.9.1 与 lite_cog 源码一致）。删 v2 复制清单 6/11/12 + 编译批 4。二期真要改 teb 源码再走"复制+编译+遮蔽"流程 | 实测系统包齐全 |
| 🟢 优化 | **V13** | **R11 降级、Q2 关闭**：`message_transformer` 的 include 实测全部自包含（自家 `protocol.h`/`sensor_logger.h`）+ 标准 ROS 头，catkin 标准包依赖，无隐藏 SDK | 实测源码 |
| 🟢 优化 | **V14** | **编译前临时 swap**：本机 zram 仅 2×571 MB，`-j3` 编 `fast_gicp`+`faster_lio` 仍有 OOM 余量风险。P5 前加 4G swapfile，编完即撤 | 实测 `swapon --show` |
| 🟢 优化 | **V15** | 小项：路径改写白名单补 `*.rviz`/`*.cfg`；sudo 每阶段保持会话；`location_record.py` 实际在 `pipeline_tracking/tools/`（v2 误写 scripts/） | 实测 |

---

## v4 变更日志（相对 v3，N1–N10）

| 级别 | # | 变更（内容 + 落点） | 原因（实测证据） |
|---|---|---|---|
| 🔴 架构 | **N1** | **M4 服务层从单体 move_base 升级为 mbf_costmap_nav 0.4.0 联邦式**（Nav2 联邦式架构的 ROS 1 现成实现）。实测依据：teb 插件描述符**双注册**（同时声明 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController`，可被 mbf 原生加载）；四 action Result 均带 `uint32 outcome + string message` 错误码；Goal 均带按名点单字段（GetPath: `planner`，ExePath: `controller`，Recovery: `behavior`，MoveBase 另有 `recovery_behaviors: string[]`）。**前置 S9**：`apt install ros-noetic-nav-core-adapter`（0.3.0，官方源）——仅全局规划器（global_planner/navfn 只注册了 `nav_core::BaseGlobalPlanner`）需要适配；**用户不批 S9 则整体回退 move_base 单体形态**（回退决策表见 §8：move_base 形态下 C11/C12/C13 保留，错误码退化为 move_base actionlib 三态 outcome，C11 降级为简化映射）。落点：§1.1（A11/A12）、§1.2（P3a.0 实测行）、§2.6、§8（C0 重写 + 回退表）、§15（Q9/Q10/Q11）、§5 P0.4（S9） | P3a.0 只读实测（小明，2026-10-04，690 行 log，裁决可行） |
| 🟠 规约 | **N2** | 新增 **§2.6 导航栈设计规约**：八决策映射表 + 硬规则——lite3_navigation 壳包只许 launch/config/scripts/verify（**policy 层**），算法一律系统插件（**mechanism 层**），壳包内禁出现第三方算法源码；隔离自检新增 **[9]**：`find /home/ros1-slam/ws/src/40_navigation -name "*.cpp" -o -name "*.cc" -o -name "*.h"` 期望 0 命中。落点：§2.6、§11.2（[9]）、§8 G3a | 用户拍板 + P3a.0（teb/global_planner 均为系统插件） |
| 🟠 诊断式恢复 | **N3** | 新增 **C11** `lite3_navigation/config/recovery_policy.yaml`——恢复原子动作注册 + 错误码→处置分级映射（局部重试 / 重规划 / 清图重规划 / 中止上报），错误码分类**以实测 log 中 GetPathResult/ExePathResult/RecoveryResult 的 outcome 常量表为准**（GetPath 15 档 SUCCESS=0…MAP_ERROR=62）；**C5 升级**：TaskTransfer.py 副本从盲目 `MAX_RETRY=3` 改为读 recovery_policy.yaml 分级重试，登记 04_配置改动diff.md。落点：§8 C11/C5、§3 | P3a.0（四 Result 错误码常量表实测） |
| 🟠 词汇库契约 | **N4** | 新增 **C12** `lite3_task/config/task_primitives.yaml` 任务原语注册表 v1——navigate_to_pose / get_path / exe_path / recover / wait / clear_costmaps 六原语，每原语 = {action/service 目标， 类型， 关键 outcome 语义}；一期 M5 用系统 **SMACH 2.5.2**（实测可 import/实例化）写最小顺序状态机验证契约；py_trees 列二期（外部依赖 R19）。clear_costmaps 原语的服务挂载点实测未覆盖 → 标注"联调确认"（并入 Q11）。落点：§8 C12、§15 Q10/Q11、§16 | 用户拍板（契约先行）+ P3a.0（smach 2.5.2 ✅） |
| 🟠 lifecycle 模拟 | **N5** | 新增 **C13**——lite3_supervisor（SUP，90_supervisor 壳包，v3 从未赋职责）正式定职责：按 mode 的有序启动清单、模块 1Hz 心跳（liveness topic）监控、失联处置（通知 M6 立即零速 → 降级关停本 mode 模块）；一期 scope = health_watch.yaml（watch 清单：模块名/心跳话题/超时）+ 最小监控脚本骨架。**红线写死：SUP 绝不直接写 /cmd_vel，/cmd_vel 唯一写者永远是 M6 safe_cmd_vel_mux**（入 §2.6 规约与自检 [6] 语义）。落点：§8 C13、§2.6、§11.2 [6]、§12 mode 表、§13 R18 | Nav2 决策 7（lifecycle+bond）的 ROS 1 模拟 |
| 🟠 纵深管道 | **N6** | C7/C8 内容保留，补管道规约：`controller(M4) → /cmd_vel_raw → safe_cmd_vel_mux(M6) → /cmd_vel → ros2qnx（仅真机 mode_navigation）`；collision_guard（多边形安全区急停）登记二期 §16。落点：§2、§2.6、§8 C7/C8、§16 | Nav2 决策 8（cmd_vel 纵深防御） |
| 🟡 吸收坑1/Q8 | **N7** | P0.5 双查正则扩充——传感器侧 `lslidar\|faster_lio\|hdl_\|roscore\|rosmaster` + 运动链路侧 `ros2qnx\|message_transformer\|lite3_ros_bridge\|jetson2motion\|jetson2app\|transfer_launch`（合并正则见 §5 P0.5）；**Q8 硬停止线正文化**：root 侧 foxy 栈（jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥在跑期间，P6/P7 沙箱起节点（stub）前必须重跑 P0.5 双查**并与用户确认处置**。落点：§5 P0.5、§11/§12 前置、§15 Q8 | 坑1/坑3（`note/02_问题与坑.md`；实机 ps 4 行长期在跑） |
| 🟡 Gate 增强 | **N8** | **G3a 增两条**：`rosparam load mbf_navigation.yaml` 无报错且 `planners/controllers/recovery_behaviors` 三列表非空；插件 type 字符串与 P3a.0 实测 log 中的插件描述符逐一比对一致。**G5/G6 增两条**：mode_nav_stub 下 `rostopic list` 可见 mbf 三 action 话题族；SUP 心跳话题按 health_watch.yaml 清单全部存活。**G4 增**：`rospack find mbf_costmap_nav` 指向 /opt/ros/noetic（证明系统版，同 teb 的 A9 校验）。落点：§8 G3a、§10 G4、§11.3–11.5 | mbf 形态的可验收性 |
| 🟡 冻结注记 | **N9** | §5/§6/§7（P0/P1/P2）头部标注"✅ 已按 v3 执行完毕（2026-10-04）·内容冻结，仅 ①P0.5 正则扩充（N7）与 ②S9 新增 ③坑2/坑4 执行注记对后续阶段生效"；P0–P2 其余正文原样保留不重写事实（Gate 结果、日期以 `note/进度/00_总览.md` 为准）。落点：§5/§6/§7 头部 | 00_总览.md：G0/G1/G1.5/G2 全 PASS（2026-10-04） |
| 🟢 台账与工期 | **N10** | v4 落位 `note/计划书v4/`（新目录；v3 原文件一字不动，旧版保留）；§13 风险表增 **R17**（mbf 参数/底稿格式差异——已实测给出转换表，残余风险=costmap 挂载命名空间未实测）/ **R18**（SUP 误碰 cmd_vel——规约+自检 [6] 防线）/ **R19**（py_trees 外部依赖，二期评估）；§15 工期调整（P3a 1–2h→2–2.5h 含 C11–C13；P6 +0.5h SUP 联调；S9 用户侧 5 min）；§16 二期入口增补四条。落点：§13/§15/§16、文档头部 | 台账一致性 |

---

## §0 三份文档的分工

| 文档 | 回答的问题 | 本文怎么用 |
|---|---|---|
| **19 号** | 场景选择、SCAN/PCT 归位、施工次序 | 提供次序（W1–W7）与验收判据（A1–A5 / B1–B5） |
| **20 号** | 模块切分、接口契约、launch 组织 | 提供骨架（M0–M6、`/cmd_vel_raw`、三层 launch、variants、Stub/Sink） |
| **21 号（本文 v4）** | 具体怎么在 `ros1-slam` 家目录建起来 | 目录树 + 属主权限 + 分阶段命令 + 校验点 + 回滚 + 风险 + 导航栈设计规约（§2.6） |

> **`ros1-slam` 是手术台，不是新家**（v2 §0.1 结论保留）：`ysc/lite_cog` 原样作为对照组与回退基准，单向复制（A6）。

---

## §1 假设与基线（v3 全部经二次实测；v4 增补经 P3a.0 只读实测）

### 1.1 假设

| # | 假设 | 依据 / v3·v4 变更 |
|---|---|---|
| **A1** | ROS = **Noetic**（`/opt/ros/noetic`），不碰 foxy | 实测双装（foxy + noetic），本文只用 noetic |
| **A2** | 20 号 M0–M6 七模块；一期实装 M0/M1/M2/M6 + M4 stub，M3/M5 占位 | 同 v2；v4 由 A11/A12 细化 |
| **A3** | 传感器 = Leishen C16（192.168.1.201）+ 狗体 IMU（`/imu/data`） | 实测 `lslidar_c16.launch`；`/imu/data` 由 **`qnx2ros`** 发布（`qnx2ros.cpp:53`） |
| **A4** | 网络 192.168.1.0/24，有出网 | 实测 |
| **A5** | 执行者 = `ysc`（有 sudo）代劳；`ros1-slam` 无 sudo、当前无法登录 | 实测；Q1 |
| **A6** | 隔离单向：`ysc/lite_cog` → `ros1-slam`，**绝不反向** | v3 新增强调：V2 写穿路径是唯一已发现的反向通道，已处置 |
| **A7** | 一期不引入 SCAN/PCT 源码，只预留 `opt/research/` | 同 v2 |
| **A8** | 与 ysc 侧不同时抢 LiDAR UDP（2368）**及底盘运动链路** | v3：P0.5 扩为"查传感器 + 运动链路"（V1）；v4：正则再扩充（N7） |
| **A9** | v3 新增：teb/move_base/map_server/mbf 用系统已装版本，不复制不编译 | V12；v4 P3a.0 复核：mbf 栈 0.4.0 全系系统包 ✅ |
| **A10** | v3 新增：`message_transformer` 四节点中，一期只允许 `qnx2ros`（+可选 `sensor_checker`）；`ros2qnx`/`nx2app` 仅真机 mode | 实测 `CMakeLists.txt` add_executable 清单 + `ros2qnx.cpp:112` |
| **A11**（**v5.1 修订，治 R02**）：M4 服务层 = **mbf_costmap_nav 0.4.0 联邦式**；全局规划器加载**不需要 S9**——v4.1 X3 已裁「不装不卸、Q9 关闭」，实测复核＝`/opt/ros/noetic/include/mbf_costmap_nav/costmap_navigation_server.h` 自带 planner／controller／recovery 三对加载器（mbf 与 nav_core 各一）。**本机 0.3.0 已装 ⇒ 只登记存在、不作前置**；回退**仅由真实联调阻塞触发**，不再有"未批 S9→回退"这条伪前置（§8 回退决策表同此读法） | 原句（需 S9、不批则回退）**〔历史，由本行替代〕**；依据＝v4.1 X3＋本会话只读抽查 E4 |
| **A12** | **v4 新增**：M5 一期 = **SMACH 2.5.2**（系统版）最小顺序状态机验证 C12 契约（细化 A2 的"M5 占位"）；py_trees 列二期（R19） | P3a.0：smach/smach_ros 2.5.2 import + StateMachine 实例化 OK（Q10 关闭） |

- **载体保真度（v4.5 收编，X7／X21）**：现仿真载体是**刚体躯干盒、无腿、无近场地面回波**（URDF 仅 fixed 关节；r<1.2m 非零回波的世界 z 全在 0.37–0.80m）⇒ 能验决策／几何／代价／规划四层，**不能验**踏步／落脚／摩擦／连续台阶。实证＝G 稿 §0／§3＋R82 探针。
- **挂账 O-11＝未验**：`set_model_state` 写不住机体偏航（30/60/90° ⇒ 回读 14.8/28.6/40.5°）、`/LIO/odom_vehicle` 朝向恒 0 ⇒ 现仿真**不能表达非零机体朝向**，凡依赖朝向的判据只能真机验（实证＝R71 证据 `p4_body_frame_probe_20261009_203307/`）。**是否进路线图＝待裁**。

### 1.2 实测基线（2026-10-04 二次复核 + P3a.0 只读实测）

| 项 | 实测值 | 对计划的影响 |
|---|---|---|
| `ros1-slam` 用户 | `uid=1001 gid=1003`，**不在 sudo/video/i2c/gpio/plugdev** | P0.2 补组 |
| **`/home/ros1-slam` ACL** | `user:ysc:rwx` + **default ACL 已设**（2026-10-04 上一轮操作） | ⚠️ 与 §3.3 chmod 冲突 → P0 先 `setfacl -b`（V11） |
| SSH 登录 ros1-slam | ❌ Permission denied | 全程 `sudo -u` 代劳 → `as_ros1slam.sh` |
| 磁盘 / 系统 / 资源 | **43 GB 可用**（70G/117G）；Ubuntu 20.04.6 + gcc 9.4.0；6 核 / 6.7 GiB / **zram 2×571 MB** | 预算见 §1.6；⚠️ 编译加临时 swap（V14） |
| ROS 与库版本 | noetic；`tf2 0.7.7`、`pcl_ros 1.7.4`；PCL/Eigen/Boost/OpenCV/Ceres = 1.10.0 / 3.3.7 / 1.71 / 4.2.0 / 1.14.0 | 同 v2 |
| **系统已装导航栈** | `move_base` `move_base_flex` `mbf_costmap_nav` **`teb_local_planner`(0.9.1)** `map_server` `octomap_ros` `move_base_sequence` 全部 ✅ | **A9/V12：不复制不编译** |
| **导航栈组件实测（P3a.0，2026-10-04）** | mbf_msgs / mbf_abstract_nav / mbf_costmap_nav **0.4.0** ✅（服务器本体 `/opt/ros/noetic/lib/mbf_costmap_nav/mbf_costmap_nav`）；`nav_core_adapter` ❌ 未装（apt 官方源候选 **0.3.0**）→ S9；smach/smach_ros **2.5.2** ✅；teb 0.9.1 **双注册**；move_base 1.17.3 对照 ✅；**mbf_costmap_nav 不带样例 yaml/launch**；插件注册参数实测 `planners`/`controllers`/`recovery_behaviors`（`[{name, type}]` 元组列表） | **N1/N2/N4 落地依据；字段权威 = `/tmp/P3a0_mbf组件实测.log`** |
| 32 个关键 ROS 包 | ✅ 全装（P4 仅核对） | P4 仅核对 |
| `py_trees` / `twist_mux` | ❌ 均未装 | 编排用 SMACH（v4：A12 定案）；M6 自研 mux |
| **logrotate** | ✅ **已装并启用**（2026-10-04 修复） | 磁盘风险项消除 |
| 相机与 LiDAR | `/dev/video*` 666 + plugdev ACL（ros1-slam 可直读）；C16 以太网 UDP 2368/2369，驱动发布相对名 `rslidar_points`（`lslidar_driver.cpp:159`，launch 参数 `pointcloud_topic`）；当前 `ps aux` 无 lslidar/faster_lio/hdl/roscore（施工窗口干净） | — |
| **root 侧常驻进程（坑1/坑3）与施工窗口** | foxy 栈 `transfer_launch`/`jetson2app`/`jetson2motion`（jetson2motion 订阅 `/cmd_vel` 直发底盘）+ ysc monitor 桥 `lite3_ros_bridge`，ps 合计 4 行长期在跑；沙箱相关进程（lslidar/faster_lio/hdl/roscore）当前无 | **Q8 硬停止线（N7）**：P6/P7 起节点前重跑 P0.5 双查并与用户确认处置 |
| **faster-lio 关键事实**（**v5.1 修订，治 R04／R21／R07**）：`lidar_type: 4` 的枚举归属＝**C16 处理分支**（**不是 Velodyne 分支**——旧解释错在此，值本身对）；话题面现役＝**Livox Mid-360**（`imu_topic: /livox/imu`，`faster_lio_outdoor.yaml:3` 实测；`/rslidar_points*` 一族属**遗留对照**）；存盘＝`pcd_save.interval: -1`（第 29 行实测；**`10` 已作废**：每 10 帧写盘后清空 ⇒ 地图截断）；存盘路径硬编码两处（**坑2：751 与 862**）；参数经**全局 NodeHandle** 读取 ⇒ 同命名空间只能跑一套 | 原句〔历史，由本行替代〕；现值＝本会话只读抽查（§0.10），**文件值确认不等于节点已加载** |
| **message_transformer** | catkin ROS1 包，4 节点：`qnx2ros`（→`/imu/data` 200Hz）、`ros2qnx`（`cmd_vel`→底盘）、`nx2app`（joy）、`sensor_checker`；include 全自包含 | A10、V1、V13 |

### 1.4 继承自 19 号的五个硬事实（不变）

F1 TEB 未避障（`include_costmap_obstacles: false`）；F2 STVL 三坑（`clearing:false`/`min_z:0.2`/`obstacle_range:3.0`）；F3 `dyn_reconfigure.py` 失效且压速 0.1；F4 `system/map/` 为空；F5 顺序不能倒（补图→修避障→mux→稳 D2→SCAN→PCT；**G3→G8→G9 顺序不能倒**）。

### 1.6 磁盘预算（v3 微调；v4 无变化）

源码 ≈ 0.19 GB（少了 teb/move_base/map_server）；`build/` 3–6 GB（少了批 4，略降）；`devel/` 0.5 GB；
地图 + bag < 0.3 GB + 0.5 GB/趟（规则不变：bags 只留 3 趟）；**合计 ≈ 5–6.5 GB，余量 ≈ 36 GB ✅**。
> v4 注记：S9（nav_core_adapter）为 apt 系统包，装在 `/opt/ros`，**不占 ws 磁盘预算**。

---

## §2 目标形态（v3 修订点 + v4 修订点）

架构图、模块编号、`/cmd_vel_raw` 契约、`variants/`、Stub/Sink 全部继承 v2 §2。**v3 两处修订 + v4 一处修订**：

1. **M0 的 `message_transformer` 启动拆分**（V1）：

```xml
<!-- lite3_drivers/launch/01_sensors.launch —— 一期唯一允许的形态 -->
<node pkg="message_transformer" type="qnx2ros" name="qnx2ros" output="screen"/>
<!-- ⚠️ ros2qnx（订阅 /cmd_vel 直发 QNX 底盘）在此 launch 中绝对禁止出现；
     它只允许出现在 mode_navigation（真机导航）中，且启动前必须过 P0.5 运动链路检查。
     nx2app（joy 桥）、sensor_checker 同理，一期不启动。 -->
```

2. **M1 话题契约**（V5）：C16 驱动发布 `rslidar_points`（相对名，launch 起在根命名空间即 `/rslidar_points`）；faster-lio 配置的 `lid_topic` 一律写 **`/rslidar_points`**。`/rslidar_points_filtered` 话题名从沙箱词汇表中删除（ysc 侧该话题的真实生产者待查，见 Q7，不影响本文）。

3. **（v4，N6）cmd_vel 纵深管道显式化**：
   `controller(M4) ──► /cmd_vel_raw ──► safe_cmd_vel_mux(M6) ──► /cmd_vel ──► ros2qnx（仅真机 mode_navigation）`。
   设计规约与两条红线见 **§2.6**；collision_guard（多边形安全区急停，管道第五环）登记二期 §16。

### 2.6 ★ 导航栈设计规约（v4 新增，N2）

> 设计渊源：ROS 2 navigation2 联邦式架构对标分析的**八个已验证设计决策**，逐一映射到 ROS 1 沙箱落地物；本节是 M4/M5/SUP 的**红线级规约**，自检脚本（§11.2）与 Gate（§8/§10）承接执行。

| # | Nav2 设计决策（已验证） | ROS 1 落地物（本沙箱） | 计划书落点 |
|---|---|---|---|
| 1 | policy / mechanism 分离 | lite3_navigation 壳包只含 launch/config/scripts（policy 层）；算法 = 系统插件 teb / global_planner / mbf（mechanism 层） | §2.6 / §8 C0 / 自检 [9] |
| 2 | 一切皆 action（可抢占、可取消） | mbf GetPath / ExePath / Recovery 独立 actionlib 服务器 + M5 原语封装（C12） | §8 C11/C12 |
| 3 | 窄插件契约 | nav_core / mbf_costmap_core 接口收敛；壳包零算法源码 | §2.6 / 自检 [9] |
| 4 | 插件单步化、服务器持环 | mbf 服务器定频驱动 teb（`controller_frequency`，机制内置） | 无需施工（说明行） |
| 5 | 运行期按名点单 | goal 内 `planner` / `controller` / `behavior` 字符串字段（P3a.0 实测） | §8 C11/C12 |
| 6 | 恢复一等公民 + 诊断式恢复 | recovery_policy.yaml 错误码→处置分级映射（policy 资产，C11） | §8 C11/C5 |
| 7 | lifecycle + bond | SUP 有序启动 + 1Hz 心跳 + 失联降级（ROS 1 模拟，C13） | §8 C13 / §12 |
| 8 | cmd_vel 纵深防御管道 | cmd_vel_raw → mux → cmd_vel → ros2qnx（仅真机）；collision_guard 二期 | §8 C7/C8 / §16 |

**硬规则（红线级，违反 = Gate 不过）**：**a) `/cmd_vel` 唯一写者 = M6 `safe_cmd_vel_mux`**——SUP（及任何其他模块）只能"通知 M6 零速"（`/lite3/supervisor/hold`，C8/C13），**绝不直接写 /cmd_vel**，自检 [6] 承接（lite3_supervisor 在扫描范围内）；
**b) 壳包禁算法源码**——`lite3_*` 壳包（尤其 lite3_navigation）内禁出现第三方算法 `.cpp/.cc/.h`，自检 [9] 承接（40_navigation 下期望 0 命中）；A9 例外流程不变：二期改 teb 源码走"复制 teb 源码 → 编译 → devel 遮蔽系统版 → diff 登记"（§16）。

---

## §3 目录结构

完整目录树继承 v2 §3.1，v3 修订四处 + v4 修订一处：

1. **笔记目录定名为 `note/`**（用户指定；v2 写的是 `notes/`，全文已统一）。
2. **`40_navigation/` 下不再出现 `teb_local_planner`、`move_base`、`navigation` 三个目录**（A9）——只有自研壳 `lite3_navigation/`。
3. `10_slam/map_server` 目录**删除**（系统已装；`rosrun map_server map_saver` 直接可用）。
4. 新增 **`opt/ref/` → 自研壳的"配置种子"语义**（见 C0）。
5. **（v4，N3/N4/N5）壳包职责注记**——新增配置文件清单落位：`lite3_navigation/config/` 增 `mbf_navigation.yaml`（C0）与 `recovery_policy.yaml`（C11）；`lite3_task/config/` 增 `task_primitives.yaml`（C12）+ `scripts/task_smach_min.py`；`lite3_supervisor/config/` 增 `health_watch.yaml`（C13）+ `scripts/health_monitor.py`。M3（lite3_perception）一期仍占位不变。

### 3.2 双目录制 v3 修订条款

> **第三方包"原样复制、一个字不改"原则增加唯一例外（V2）**：
> `faster_lio/src/laser_mapping.cc:751` 的存盘路径硬编码写穿隔离边界，**必须**在复制后立即改写为
> `/home/ros1-slam/data/maps/`。这是全文唯一许可的第三方包源码改动，必须登记进
> `note/04_配置改动diff.md`（旧值/新值/理由/日期），且自检 [8] 会把它纳入永久监控。
> 判断标准写死：**凡影响隔离边界（读写 `/home/ysc`）的第三方源码行为，隔离规则优先于"一字不改"。**
> v4 注记（§2.6 硬规则 b）：本条款管第三方包；自研壳方向的反向约束（壳包内禁算法源码）由自检 [9] 承接。

### 3.3 属主与权限（v3 时序修正）

权限模型表继承 v2（`/home/ros1-slam` 750、目录 755、文件 644、脚本 755、`opt/ref/` 555）。**v3 修正执行时序与前置（V9/V11）**：
⓪ **P0.6（复制之前）**`sudo setfacl -bR /home/ros1-slam`（清掉 2026-10-04 设的 ysc ACL，避免与后续 chmod 的 mask 冲突；此后一切访问走 `sudo -u` / `sudo cp`）→ ① **P2 复制完成后**`sudo chown -R ros1-slam:ros1-slam /home/ros1-slam` → ② 紧随 ① 按权限模型 chmod（v2 原 ② ③ ④ 不变）→ ⑤（可选，P2 末）重建 ysc 便利通道：`sudo setfacl -Rm u:ysc:rX /home/ros1-slam && sudo setfacl -Rdm u:ysc:rwx /home/ros1-slam`（**必须在 ② 之后**；rX 不破坏脚本 755，与 v2 的 mask 顾虑相容）。

---

## §4 阶段总览与依赖（v4 修订 DAG：P0–P2 已执行冻结；P3a.0 ✅ 已完成；S9 为新前置）

```
Phase 0  环境准备（登录/组/ACL清理/磁盘/施工窗口）[需 root] ── Gate G0：可用 + ACL 已清              ✅ PASS 2026-10-04
   ▼
Phase 1  目录初始化 + catkin 初始化 ── Gate G1：骨架齐全                                            ✅ PASS
   ├─ P1.5 ★ 自研壳脚手架生成（lite3_* 包骨架）[v3 新增 V7] ── Gate G1.5：package.xml 就位          ✅ PASS
   ▼
Phase 2  外部文件复制 + faster-lio 存盘路径改写 + 路径扫描替换 ── Gate G2：v3_isolation.sh 退出码 0  ✅ PASS
   ├─ P2 末：统一 chown + chmod（v3 时序 V9）
   ▼
P3a.0 ★ 导航栈组件只读实测（v4 新增节点，2026-10-04 完成；产出 /tmp/P3a0_mbf组件实测.log）           ✅
   ▼
[S9] ★ nav_core_adapter 安装（用户 sudo，Q9；不批 → §8 回退决策表切 move_base 形态）                 [v4 新增，未执行]
   ▼
Phase 3a ★ 配置重构（C0–C10 + v4 新增 C11–C13；含 mbf_navigation.yaml 转换稿）[纯写文件，不依赖编译]
   │  Gate G3a：rosparam 可加载 + 三列表非空 + type 串比对 + diff 文档齐全 [v4 增强 N8]
   ▼
Phase 4  依赖核对（实测已满足，10 分钟；v4：含 S9 后 nav_core_adapter 命中核对）
   ▼
Phase 5  编译构建（v3 五批 → 实为四批 V12；先加临时 swap V14）── Gate G4：devel/setup.bash 存在 + ROS_PACKAGE_PATH 无 ysc + teb/mbf 系统版核验 [v4 增强 N8]
   ▼
Phase 3b ★ 现场采图（原 C1 执行部分，依赖 P5 编译产物）[v3 时序修正 V6] ── Gate G3：data/maps/ 三件齐
   ▼
Phase 6  分层验证（Stub/Sink 逐模块；G5–G9）
   │  ★ 前置（v4 N7/Q8 硬停止线）：重跑 P0.5 双查 + 与用户确认处置；G5/G6 判据增强（N8）
   ▼
Phase 7  场景组合启动（modes/）── ★ 前置（N7/Q8 硬停止线）：同上
   ▼
Phase 8  回滚（任意阶段可执行）
```

---

### 4.3 施工值与能力包线（**v4.5 收编新建本节**；v4 此前无 §4.3，落点由"§4.3"改指此处；**门值一字未改**）
- **X5「标定优先」＝按 Y2 落，写作三段**：①**事实**＝厂商高程包闭源（63 文件、`.cpp/.h`＝0）⇒ 其阈值量纲**不可核**；②**处置**＝§4.3 的 `slope>25°`／`step>0.15m` 标为「**我方 spike3d 设计稿自定、无实测锚**」（出处＝`spike3d_elevation_costmap_design.md` §4.3 代价映射行），厂商标称只作**上界对照**：通用挡 楼梯 ≤12cm／斜坡 ≤30°、抓地挡 斜坡 ≤40°、高踏步挡 楼梯 ≤15cm，且**身体高度分正常／匍匐两挡**（抄录在册 `03`）⇒ 15cm 是**最宽松挡**，真机默认若为通用挡则真实包线 12cm；③**禁止**＝不得把任一厂商常数当"已标定"引用。
- **0.839／0.577 的两种解释＝并列未决，不选边**：解释一 tan 两挡上限；解释二"随栅格分辨率改写的无量纲梯度"。**本轮实测把解释二削薄**：12 件 yaml 里 `slope_method` **全为 1**、`0.577` **从不作取值**（grep 命中 0 处），且 0.839 同现于 `resolution` 0.03 与 0.04 两件 ⇒ **tan 对应降为巧合级旁证**；「阈值须写成 `slope_max(resolution)`」这条原则**降为待定标**（**未验**，非已解决）。
- **待裁（不代答）**：**C11** 按哪一挡定包线或授权真机实测默认挡；**C5／C6** 向厂商索取能力表并把"可感知最小距离／可感知下限"两行入问询；**A25／A23** 见 §16.4。

- **v5 状态补记（已裁待施工）**：A23 的 0.10m 已裁纳入判定、来源＝env 覆盖 `P4_OCC_TOL`（详见 §16.4 的 Z6 格，施工落地点＝#111）；**本节 X5 的"阈值随分辨率给值"仍＝待定标、无实测锚**，0.839／0.577 两种解释照旧并列不选边。
## §5 Phase 0：环境准备

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G0 PASS**，见 `note/进度/00_总览.md`）。正文冻结不改写；对后续阶段生效的仅有 **①P0.5 正则扩充（N7，P6/P7 重查适用）②S9 新增（P0.4）③P0.7 坑4 施工规约注记**。

### P0.1 登录可用（同 v2：A 设密码 + B 分发公钥，推荐都做）

### P0.2 补充用户组（同 v2：`plugdev,video` 必、`i2c,gpio` 选、**sudo 不加**）
实测 `gpio/i2c/plugdev/video` 四个组都存在，一条 `usermod` 即可。

### P0.3 系统依赖清单（同 v2，全部 ✅；CUDA 11.4 < 11.7 硬阻塞 PCT 不变）

### P0.4 需要 sudo 的操作清单（继承 v2 S1–S6，另加；v4 增 S9）

| # | 操作 | Phase |
|---|---|---|
| S7 | `setfacl -bR`（V11）+ P2 末 chown/chmod | P0/P2 |
| S8 | 临时 swap 增删（V14） | P5 前/后 |
| **S9** | **`sudo apt install ros-noetic-nav-core-adapter`（0.3.0，官方源 packages.ros.org；仅装系统包，不进 ws、不改第三方源码）** | **P3a 前（Q9：用户批准并自行执行；不批 → §8 回退决策表 move_base 形态）** |

### P0.5 施工窗口确认（v3 扩展 V1；v4 正则再扩充 N7 —— 对 P6/P7 重查生效）

```bash
ps aux | grep -E "lslidar|faster_lio|hdl_|roscore|rosmaster" | grep -v grep   # 传感器侧，期望空
ps aux | grep -E "ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer_launch" | grep -v grep  # ★ 运动链路侧（v4 扩充 N7），期望空
```

> 第二条是 v3 新增：若 ysc 侧 monitor/桥接栈在跑，`/cmd_vel` 链路活跃，stub 模式的任何手滑 pub
> 都可能真驱动机器狗。**真机 mode（含 ros2qnx）启动前必须重查本条。**
> **v4 注记（N7，吸收坑1）**：v3 原运动链路正则漏掉 root 侧 foxy 栈三进程（`transfer_launch`/`jetson2app`/`jetson2motion`——jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥 `lite3_ros_bridge`，实机 ps 实测 4 行长期在跑（P0 执行时已按扩充正则重查登记，见 `note/01_部署记录.md` P0.5 节与 `02_问题与坑.md` 坑1）。**两侧合并等价正则**：`lslidar|faster_lio|hdl_|roscore|rosmaster|ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer_launch`。P0 执行事实：判定 = 登记 Q8 后继续，P0–P2 零节点启动。
> **Q8 硬停止线（N7，正文化）**：root 侧 foxy 栈与 ysc monitor 桥在跑期间，**P6/P7 沙箱起节点（stub）前必须重跑本节双查并与用户确认处置**（§11/§12 前置、§15 Q8）。

### P0.6 ★ 清理 ACL（v3 新增，V11）

```bash
sudo setfacl -bR /home/ros1-slam
getfacl /home/ros1-slam | grep -c "user:ysc"   # 期望 0
```

### P0.7 sudo 会话技巧

每阶段开一个持权 shell（`sudo -v` 刷新 15 分钟票据；或 `sudo -i` 常驻），避免几十条 `sudo -u` 命令反复输密码。

> **坑4 执行注记（v4 收编，对 P3a 及以后生效）**：①sudo 时间戳按会话记账，**跨 Bash 会话必须重新刷新凭据**（每施工块开头 `printf '%s\n' "$PW" | sudo -S -p '' -v`）；②**证据采集用"重定向到文件再 cat"**，不在管道子 shell 内裸 sudo（P1 实测报「a terminal is required」；P3a.0 证据暂存 /tmp 亦因此，见 §8.0）。

---

## §6 Phase 1：目录初始化

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G1 / G1.5 双 PASS**，见 `note/进度/00_总览.md`）。
> 正文冻结；`gen_shell_pkgs.sh` 全文见 v3 §P1.5（已执行，产物即当前沙箱 9 个 lite3_* 壳，此处不再重复代码块）。

P1.1 建骨架 / P1.2 `catkin_init_workspace` 同 v2（目录树按 §3 修订：无 teb/move_base/map_server 子目录，笔记目录为 `note/`）。

### P1.5 ★ 自研壳脚手架生成（v3 新增，V7）

生成 9 个 `lite3_*` 壳包骨架（package.xml + CMakeLists + launch/stub/sink/config/scripts/verify + README），
包位清单（已执行，与沙箱现状一致）：

```bash
ROOT=/home/ros1-slam/ws/src
# 00_drivers/lite3_drivers=M0  10_slam/lite3_slam=M1  20_localization/lite3_localization=M2  30_perception/lite3_perception=M3
# 40_navigation/lite3_navigation=M4  50_task/lite3_task=M5  60_safety/lite3_safety=M6  70_bringup/lite3_bringup=TOP  90_supervisor/lite3_supervisor=SUP
```

**Gate G1.5**：`find /home/ros1-slam/ws/src -name package.xml | grep -c lite3_` → **9**（至少 8）→ **PASS（2026-10-04）**。
M5（lite3_task）同样生成，虽为一期占位，但空壳可编译，省去白名单特判。

> v4 注记：lite3_task / lite3_supervisor 两壳在 v4 中正式赋职责（C12/C13，§8）；骨架不变。

### P1.6 `.bashrc` 追加（v3 落实，V10 关联）

```bash
sudo -u ros1-slam tee -a /home/ros1-slam/.bashrc <<'EOF'

# ==== ros1-slam sandbox env ====
source /home/ros1-slam/scripts/env.sh 2>/dev/null || true
EOF
```

（`env.sh` 在 P6.1 才写全，此处先容错；自检 [3] 负责把关。）

### P1.3 `as_ros1slam.sh` / P1.4 `note/00_README.md` 三条硬规则：同 v2。

---

## §7 Phase 2：外部文件复制与路径改写

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G2 PASS**，见 `note/进度/00_总览.md`）。正文冻结不改写；**坑2 执行注记**：faster-lio 存盘路径硬编码实测**两处**（`laser_mapping.cc:751 与 862`），§7.2a 原文只写 751，实际执行以 sed 全局替换一次覆盖两处（文件内 /home/ysc 计数=0，`04_配置改动diff.md` 第 1 条两处行号并载，G2 证据独立抽验两行均为改写后路径）；后续若重做复制，按两处执行。

### 7.1 复制清单（v3 修订版）

| # | 来源 | 目标 | v3 变更 |
|---|---|---|---|
| 1–2 | `…/drivers/leishen_ws/src/lslidar_driver` + `lslidar_msgs` | `ws/src/00_drivers/`（同名子目录） | 不变 |
| 3 | `…/transfer/src/message_transformer` | `ws/src/00_drivers/message_transformer` | 不变（★ M0 必需） |
| 4 | `…/slam/src/faster-lio` | `ws/src/10_slam/faster_lio` | 不变（排除 .git）；**复制后立即执行 §7.2a 存盘路径改写** |
| 5 | `…/slam/src/pcd2grid` | `ws/src/10_slam/pcd2grid` | 不变 |
| ~~6~~ | ~~`…/slam/src/map_server`~~ | ~~删除~~ | **🆕 不复制**：系统已装（A9） |
| 7–10 | `…/nav/src/hdl_localization` + `hdl_global_localization` + `ndt_omp` + `fast_gicp` | `ws/src/20_localization/…` | 不变 |
| ~~11~~ | ~~`…/nav/src/teb_local_planner`~~ | ~~删除~~ | **🆕 不复制**：系统已装 0.9.1，与源码版本一致（V12） |
| ~~12~~ | ~~`…/nav/src/move_base`~~ | ~~删除~~ | **🆕 不复制**：系统已装 |
| 13 | `…/nav/src/navigation/config/` | `opt/ref/navigation_params_orig/` | 不变（只读参照；navigation 包**任何部分都不进 ws**） |
| 14 | `…/voa/src/deeprobotics_local_height_map`+`libraries` | `ws/src/30_perception/` | 不变 |
| 15 | `…/pipeline/src/pipeline_tracking` | `ws/src/50_task/pipeline_tracking` | 不变；C4 排除 `scripts/dyn_reconfigure.py`；**`location_record.py` 在 `tools/` 子目录**（v2 误写 scripts/） |
| 16 | `/home/ysc/faster_lio_online.yaml` | `ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml` | **🆕 ROS2→ROS1 转换后才落位**（转换稿见 §C1，原文件留档 `opt/ref/faster_lio_ros2_orig.yaml`） |
| 17 | `/home/ysc/c16__lslidar_driver_node.yaml` | `opt/ref/lidar_c16_orig.yaml` | 不变（ROS2 格式只作参照） |
| ~~18~~ | `realsense2_camera` | — | 🆕 明确**二期才复制**，一期清单中删除 |

### 7.2 复制命令：同 v2（`copy_pkg` tar 管道，排除 `.git/build/devel`；**永不删 `/home/ysc/lite_cog/`**）。

### 7.2a ★ faster-lio 存盘路径改写（v3 新增，V2，复制后立即执行；坑2：实测两处 751/862）

```bash
F=/home/ros1-slam/ws/src/10_slam/faster_lio/src/laser_mapping.cc
sudo grep -n '/home/ysc/lite_cog/system/map' "$F"     # 实测命中两处：751 与 862（坑2）
sudo sed -i 's#/home/ysc/lite_cog/system/map/lite3#/home/ros1-slam/data/maps/lite3#g' "$F"
sudo grep -n '/home/ros1-slam/data/maps' "$F"         # 复核改写成功（两处）
echo "- faster_lio/src/laser_mapping.cc:751/862 存盘路径 /home/ysc/... → /home/ros1-slam/data/maps/（隔离改写，唯一许可的第三方源码改动）$(date +%F)" \
  | sudo tee -a /home/ros1-slam/note/04_配置改动diff.md
```

> v4 注记：命令保持 v3 的 sed 全局替换形态（天然覆盖两处）；执行事实见章首坑2 注记。

### 7.3 绝不复制 build/devel/.git：同 v2（R1/R2 机制与翻车案例保留）。

### 7.4 路径改写（v3 白名单扩充，V15）

```bash
cd /home/ros1-slam
# 扫描留档（白名单 *.launch/*.yaml/*.xml/*.sh/*.py/*.rviz/*.cfg/CMakeLists.txt/*.cpp/*.h/*.cc）
grep -rn "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.xml" \
     --include="*.sh" --include="*.py" --include="*.rviz" --include="*.cfg" \
     --include="CMakeLists.txt" --include="*.cpp" --include="*.h" --include="*.cc" \
     ws/src 2>/dev/null | tee note/02_问题与坑.md.scan
# 替换（仅 ws/src；notes 与 opt/ref 保持原样以存来源）
grep -rl "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.sh" \
     --include="*.py" --include="*.rviz" --include="*.cc" --include="*.cpp" \
     ws/src 2>/dev/null | xargs -r sudo sed -i \
  's#/home/ysc/lite_cog/system/map#/home/ros1-slam/data/maps#g; s#/home/ysc#/home/ros1-slam#g'
grep -rn "/home/ysc" ws/src 2>/dev/null | wc -l   # 复扫：期望 0（laser_mapping.cc 已在 7.2a 处理）
```

> v3 注：`--include="*.cc"` 是必要的——faster-lio 源文件是 `.cc` 后缀，v2 白名单漏了它，而 751 行这类硬编码恰在 `.cc` 里。
> v4 注记：P2 实际执行"按 map→data/maps 先行 + 全量替换"两步（快照 `02_问题与坑.md.scan`：替换前命中 4 行/3 文件，复扫=0），与上式语义一致。

### 7.5 C16 参数（v3 无变更，保留）：以 `lslidar_c16.launch` 默认值为准（msop 2368 / difop **2369** / frame_id `rslidar` / max_range 150 / min_range 0.30），`c16__lslidar_driver_node.yaml`（ROS2 格式、difop 56401）仅留档不用。

---

## §8 Phase 3a：配置重构（纯写文件，编译前完成）—— v4 重写章节（mbf 主路线）

> 本章为 v4 改动最大章节：8.0 收编 P3a.0 实测；C0 重写为 mbf 形态；C1–C4/C6/C9/C10 不变；C5 升级（N3）；C7/C8 补管道规约（N6）；新增 C11/C12/C13（N3/N4/N5）与回退决策表（N1）；G3a 增强（N8）。前置：S9（Q9 批准后安装；不批 → 本章末回退决策表）。

### 8.0 ★ P3a.0 导航栈组件只读实测（✅ 2026-10-04 完成，收编 N1）

执行者：小明，只读实测（未起节点、未 roscore、未 apt install、未写 ws/opt、未触碰 `/home/ysc/lite_cog/`）。原始证据：`note/进度/v7/S0/计划书v4升级/证据/P3a0_mbf组件实测.log`（690 行；P3a.0 时因 sudo 受限先暂存 /tmp，**已按坑4 规约归档至证据目录**，/tmp 留同内容副本）。**本节及全文一切 mbf 字段名/参数名/插件 type 串以该 log 为唯一权威。**

**组件清单裁决表**：

| 组件 | 实测结果 | 处置 |
|---|---|---|
| mbf_msgs / mbf_abstract_nav / mbf_costmap_nav | **0.4.0** ✅（rospack + dpkg 双证；服务器本体 `/opt/ros/noetic/lib/mbf_costmap_nav/mbf_costmap_nav`） | **N1 主路线成立** |
| move_base_flex / executive_smach（metapackage） | dpkg 均已装（0.4.0 / 2.5.2）；`rospack find` 拒识（package.xml 在但无 build_type，异常原样记录） | 不影响：服务器在 mbf_costmap_nav，metapackage 仅依赖聚合 |
| nav_core_adapter | ❌ 未装；apt 候选 **0.3.0-2focal.20250521.021323**（官方源） | **S9（Q9 用户 sudo）** |
| smach / smach_ros | **2.5.2** ✅ import + StateMachine 实例化 OK（P3a.0 python3 实测） | C12 一期成立（**Q10 关闭**） |
| teb_local_planner | 0.9.1，**双注册**（plugin xml 同时声明 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController`） | mbf 原生可载，**无需适配** |
| move_base | 1.17.3 ✅ | 回退形态对照（回退决策表） |

**关键实测事实（C0/C11/C12 的依据）**：① mbf_costmap_nav **不带样例 yaml/launch**（find 空）→ 配置全自研；② 插件列表参数名 = **`planners`/`controllers`/`recovery_behaviors`**，格式 `[{name, type}]` 元组列表，默认取各列表第一个（`strings libmbf_abstract_server.so` 原文）；③ 改名参数（`move_base_legacy_relay.py:93/102/104`）：`max_planning_retries→planner_max_retries`、`max_controller_retries→controller_max_retries`、`recovery_behavior_enabled→recovery_enabled`；④ 同名保留：`controller_frequency`/`planner_frequency`/`oscillation_timeout`/`oscillation_distance`（patience 两项原样透传）；⑤ Goal 按名点单字段：GetPathGoal.`planner`、ExePathGoal.`controller`、RecoveryGoal.`behavior`、MoveBaseGoal 另有 `recovery_behaviors: string[]`；⑥ Result 均带 `uint32 outcome + string message`，常量表见 C11（15/19/10/11 档）；⑦ costmap 三底稿结构原样沿用，**挂载命名空间未实测（Q11）**；⑧ mbf_msgs 另有 CheckPath/CheckPose/CheckPoint 三个 srv（验证用）；⑨ ★**防踩坑**：action 消息是 **GetPathAction/ExePathAction 族**——`rosmsg show mbf_msgs/GetPath` 会报错，正确查询 `rosmsg show mbf_msgs/GetPathGoal`（Result/Feedback 同理），action 定义在 `action/GetPath.action`。

### C0 — ★ 配置种子（v4 重写为 mbf 形态，N1/N8）

v3 基线保留：P3a 第一步从 `opt/ref/` 拷入底稿再改。v4 修订：**六底稿 → 五件原样拷 + 一件（move_base_params）转换**。

```bash
REF=/home/ros1-slam/opt/ref/navigation_params_orig
NAV=/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config
sudo mkdir -p "$NAV/variants"
sudo cp "$REF/common_costmap_params.yaml"     "$NAV/costmap_common.yaml"          # C3 目标（P3a.0 实测文件名）
sudo cp "$REF/global_costmap_params.yaml"     "$NAV/global_costmap_params.yaml"
sudo cp "$REF/local_costmap_params.yaml"      "$NAV/local_costmap_params.yaml"
sudo cp "$REF/global_planner_params.yaml"     "$NAV/global_planner_params.yaml"
sudo cp "$REF/teb_local_planner_params.yaml"  "$NAV/planner_teb.yaml"             # C2 目标
sudo cp "$REF/move_base_params.yaml"          "$NAV/move_base_params.yaml"        # 回退底稿：仅存档不加载（回退决策表）
```

**六底稿归宿与转换规则**（依据 P3a.0 log）：

| v3 C0 六底稿 | v4 归宿 | 转换规则 |
|---|---|---|
| `move_base_params.yaml` | **`mbf_navigation.yaml`（新写，全文见下）** | `base_global_planner`/`base_local_planner` 单字符串 → **planners/controllers 注册列表**（`[{name, type}]`，默认取第一个）；recovery_behaviors 注册表来自 C11；改名三项；同名项原样；`make_plan_clear_costmap`/`make_plan_add_unreachable_goal` 为 move_base 特有**不迁移**（Q11 确认 GlobalPlanner 参数等价物） |
| `teb_local_planner_params.yaml` | `planner_teb.yaml`（原样拷，C2 目标） | 顶层键 `TebLocalPlannerROS:` 不变；teb 参数在 mbf 下**同名生效**（同一插件二进制） |
| `common_costmap_params.yaml` | `costmap_common.yaml`（原样拷，C3 目标） | 结构沿用；挂载 ns 见 launch 注记（Q11） |
| `global_costmap_params.yaml` / `local_costmap_params.yaml` | 原样拷 | 结构沿用；挂载 ns（Q11） |
| `global_planner_params.yaml` | 原样拷 | 顶层键 `GlobalPlanner:` 不变 |

**`lite3_navigation/config/mbf_navigation.yaml` 完整草案**：

```yaml
# mbf_navigation.yaml —— M4 服务器主配置（mbf_costmap_nav 0.4.0 联邦式，v4 C0）
# 转换自：opt/ref/navigation_params_orig/move_base_params.yaml（mbf 不带样例 yaml/launch，实测 find 空 → 全自研）
# 字段权威：/tmp/P3a0_mbf组件实测.log（P3a.0，2026-10-04）——参数名禁止凭记忆增删
# ---- 插件注册表：[{name, type}] 元组列表（strings libmbf_abstract_server.so 实测；默认取各列表第一个，log 原文）----
planners:
  - name: "GlobalPlanner"
    type: "global_planner/GlobalPlanner"   # 串源=原版 move_base_params.yaml base_global_planner 实测值；
                                           # 该插件仅注册 nav_core::BaseGlobalPlanner（rospack plugins 实测）→ 加载须经 S9 nav_core_adapter（Q9）
controllers:
  - name: "TebLocalPlannerROS"
    type: "teb_local_planner/TebLocalPlannerROS"   # 串源=teb_local_planner_plugin.xml 双注册实测（mbf_costmap_core::CostmapController）→ mbf 原生可载
recovery_behaviors:    # 注册表来自 C11 recovery_policy.yaml；type 串与可加载性见 Q11（P3a.0 log 无 recovery 插件描述符）
  - name: "clear_costmaps"
    type: "clear_costmap_recovery/ClearCostmapRecovery"
    # ★ Q11 联调首查：log 仅含包名 clear_costmap_recovery 与 ccr_plugin.xml 路径，未含 class 串；
    #   且该插件只注册 nav_core 命名空间，mbf 侧加载路径（是否同样需 adapter）未实测。
    #   兜底：加载失败则本列表退空 + recovery_enabled 保持 false，恢复全走 M5 层 C11 分级（G3a 该子项随 Q11 决议修订）。
# ---- 改名参数（move_base_legacy_relay.py:93/102/104 实测映射）----
planner_max_retries: -1       # ← 底稿 max_planning_retries: -1（六底稿中唯一在稿的改名项）
recovery_enabled: false       # ← 底稿未含（mbf 默认开）。★ v4 设计裁决：mbf 内置自动恢复链关闭，恢复编排收口 M5（C11 分级映射），
                              #   避免双重恢复；注册表保留供 M5 显式按名调用 Recovery 原语（C12 recover）。
# controller_max_retries（← max_controller_retries）：底稿未含该参数，mbf 默认值生效，不虚设。
# ---- 同名保留参数（底稿原值原样迁移，P3a.0 实测同名）----
planner_frequency: 0.0
controller_frequency: 10.0
planner_patience: 3.0
controller_patience: 3.0
oscillation_timeout: 0.0
oscillation_distance: 0.1
# ---- move_base 特有参数 make_plan_clear_costmap / make_plan_add_unreachable_goal 不迁移（GlobalPlanner 等价物 Q11）----
# ---- costmap 三底稿结构原样沿用，launch 按命名空间分别加载；挂载命名空间未实测（Q11）：P6 下 rosparam get
#      复核 ~global_costmap / ~local_costmap 后回填固化；底稿子键以 opt/ref 实际为准 ----
```

**launch 主线草案**（`lite3_navigation/launch/40_navigation.launch`；rviz 默认关 = C10）：

```xml
<launch>
  <arg name="rviz" default="false"/>
  <!-- M4 主线：mbf_costmap_nav 联邦式服务器（v4 N1）。rosparam 在 node 标签内 → 参数落私有命名空间；
       各文件挂载 ns 为 v4 设计稿，P3a.0 未实测 → Q11（P6 下 rosparam get 复核后固化/修正）。 -->
  <node pkg="mbf_costmap_nav" type="mbf_costmap_nav" name="move_base_flex" output="screen">
    <rosparam command="load" file="$(find lite3_navigation)/config/mbf_navigation.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/planner_teb.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/global_planner_params.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/costmap_common.yaml"        ns="global_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/global_costmap_params.yaml" ns="global_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/costmap_common.yaml"        ns="local_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/local_costmap_params.yaml"  ns="local_costmap"/>
  </node>
  <!-- C7：/cmd_vel_raw relay（源话题名 Q11 联调确认后固化） -->
</launch>
```

### C1 — ★★ 补地图前置：faster-lio 配置转换（V2/V3/V4/V5 落地，19 号 W1）—— v4 不变

转换稿**全文以 v3 §C1 为准**（ROS2→ROS1 完整转换版，v3 原文一字不动保留于 `note/计划书v3/`，N10；落位 `/home/ros1-slam/ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml`，勿再"修正"）。四个关键裁决（不可改）：**V3** 源文件是 ROS2 格式，v3 已给出 ROS1 转换版（顶层直接是参数组）；**V5** `lid_topic: /rslidar_points`（原 `/rslidar_points_filtered` 全库无生产者；imu_topic `/imu/data` 由 qnx2ros 发布）；**V4** `lidar_type: 4`（VelodyneHandler 分支，`pointcloud_preprocess.cc:20`），绝不可"修正"为 2；**Q6** `pcd_save.interval: 10`（原值 0 时 `laser_mapping.cc:747` 条件恒假可能永不落盘，先按 10 处理）。

**launch 加载形态（不变）**：`lite3_slam/launch/10_slam_mapping.launch` 中 `<rosparam command="load" file="$(find lite3_slam)/config/faster_lio_outdoor.yaml"/>` 必须放在 `<node pkg="faster_lio" type="run_mapping_online" name="faster_lio_online"/>` **标签之外**——`run_mapping_online.cc:25` 用全局 NodeHandle 读全局命名空间（副作用：参数占全局命名空间，同一 roscore 只能跑一套 faster-lio，可接受）。**（v5.1 治 R03）**照抄旧句 `type="mapping_online"` 会**启动失败**——本会话实测 `ws/src/10_slam/faster_lio/app/CMakeLists.txt:1` ＝ `add_executable(run_mapping_online run_mapping_online.cc)`；任何建图入口执行前先跑**存在性检查**（`rospack find faster_lio`＋`ls $(catkin_find ...)`/`rosnode` 启动返回码），失败即停手不猜。**建图执行流程**：tmux 版脚本 `tmux_mapping.sh`（W4）挪到 Phase 3b（V6）执行，产物校验见 §10 P5.4。

### C2 — TEB 避障（W2）—— v4 不变，补 mbf 同名注记

对 `planner_teb.yaml`（C0 拷入的底稿）设
`include_costmap_obstacles: true`、`enable_homotopy_class_planning: false`、`max_vel_x: 0.4`、`planning_horizon: 4.0`、`dt_ref: 0.4`。
**v4 注记**：teb 参数在 mbf 下**同名生效**（同一插件二进制 0.9.1，C0 launch 经私有命名空间加载 `TebLocalPlannerROS:` 段，挂载位 Q11）。校验 G3b（`teb_markers` 含 `PolygonObstacles` ≥1）同 v2。

### C3 — STVL 三坑（W3）—— v4 不变，补 mbf 同名注记

`costmap_common.yaml`（底稿同上）改
`clearing: true`、`obstacle_range: 8.0`、`voxel_decay: 10.0`、`voxel_size: 0.10`；`min_z` 先 `tf_echo base_link rslidar` 体检再定（C16 几何依据同 v2：ring7 打 10 cm 台阶上限 8.6 m；Q3）。
**v4 注记**：costmap 参数名在 mbf 下同名生效（costmap 底稿结构原样沿用，挂载 ns Q11）。

### C4 — 不复制 `dyn_reconfigure.py`（W5）：复制后 `sudo rm -f …/pipeline_tracking/scripts/dyn_reconfigure.py` 并记录；运行期校验 `max_vel_x ≥ 0.4` 同 v2。

### C5 — `TaskTransfer.py` 限次重试（W6）—— v4 升级（N3）

v3 基线：改副本 `MAX_RETRY = 3`（盲目三次）。
**v4 升级**：副本改为读取 `lite3_navigation/config/recovery_policy.yaml`（C11），按错误码**处置分级**重试——`retry_local` 级按 `max_retries` 限次；`replan` / `clear_and_replan` 级升级处置；`abort_report` 级立即停止并上报。登记 `note/04_配置改动diff.md`（旧值/新值/理由/日期）。
（TaskTransfer.py 任务级失败语义与 mbf action outcome 的对接在 P6 mode_nav_stub 联调中验证。）

### C6 — 航点目录（W7）：同 v2；**修正**：录航点工具路径为 `…/pipeline_tracking/tools/location_record.py`（PyQt5，需 `ssh -X` 或现场 HDMI，退化方案手写 JSON 不变）。

### C7 — `/cmd_vel_raw` 改名落地（v3 基础 + v4 管道规约 N6）

v3 基线：topic_tools relay；**M4 输出绝不叫 /cmd_vel**。
**v4 管道规约（N6）**：`controller(M4) → /cmd_vel_raw → safe_cmd_vel_mux(M6) → /cmd_vel → ros2qnx（仅真机 mode_navigation）`。
★ mbf 形态下 relay **源话题 = mbf 服务器的速度输出话题**（P3a.0 未覆盖该话题名 → **Q11 联调确认后固化** launch 行）；确认前 C7 不落 launch 行，G3a 不校验本项。（回退形态下源 = move_base `~cmd_vel`，v3 原形态。）

### C8 — M6 自研 `safe_cmd_vel_mux.py`（v3 基础 + v4 N5/N6）

v3 基线：六源优先级、超时淘汰、**无源发零速**、恒 20 Hz；Lite3 底盘 250 ms 超时不能断。
**v3 运行链路事实保留**：M6 输出 `/cmd_vel` 的下游消费者是 `ros2qnx`（`ros2qnx.cpp:112`）——所以 **stub 模式下 `/cmd_vel` 只到 mux 为止**，绝不启动 ros2qnx（V1）。
**v4 增补**：① 管道规约（N6）：`/cmd_vel_raw → mux → /cmd_vel → ros2qnx（仅真机）`；② **监督保持输入（C13 配套）**：订阅 `/lite3/supervisor/hold`（std_msgs/Bool），true 时无视各源直接输出零速并保持直到 false——SUP 是该话题唯一发布者（红线：SUP 绝不直接写 /cmd_vel，§2.6 硬规则 a）；③ 登记口径：mux 速度源数量仍为六（hold 是控制输入，不是速度源）；增补项登记 diff。

### C9 — `variants/` 三档：同 v2（outdoor_open / crowd / night）。

### C10 — rviz 默认关闭：同 v2（实测 `local_rslidar_imu.launch:74` rviz `required="true"`，SSH 无 X 会拖死整个 launch——判断正确，保留）。

### C11 — ★★ 诊断式恢复策略（v4 新增，N3）

**文件**：`/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/recovery_policy.yaml`（policy 资产）——恢复原子动作注册 + 错误码→处置分级映射（**retry_local 局部重试 / replan 重规划 / clear_and_replan 清图重规划 / abort_report 中止上报**）。
**消费者**：lite3_task SMACH 状态机（C12）、TaskTransfer.py 副本（C5 升级版）；错误码常量**全部摘自 P3a.0 实测 log 的 outcome 常量表，禁止凭记忆增删**。

```yaml
# =============================================================================
# recovery_policy.yaml —— M4/M5 诊断式恢复策略（v4 新增 C11，N3）
# 错误码权威：/tmp/P3a0_mbf组件实测.log（P3a.0，2026-10-04）；outcome_tables 禁止手改
# =============================================================================
# ---- 恢复原子动作注册表（由 C12 原语组合实现；键名 = M5 处置分级命名空间，边界见下行澄清）----
# ★ 命名空间边界：本表键（retry_local 等）= M5 处置分级键（C11 策略命名空间），≠ mbf RecoveryGoal.behavior 点单值（后者仅限 C0 recovery_behaviors 注册表 name，当前仅 clear_costmaps；键→点单值是否/如何映射由 SMACH 原语实现层决定）
recovery_actions:
  retry_local:      {primitive: exe_path,       max_retries: 2, backoff_s: 1.0}   # 局部重试：同参数重发 ExePath
  replan:           {primitive: get_path,       max_retries: 2, backoff_s: 1.0}   # 重规划：GetPath → ExePath
  clear_and_replan: {primitive: clear_costmaps, then: [get_path, exe_path], max_retries: 1, backoff_s: 2.0}   # clear_costmaps 服务挂载点 Q11 联调确认（C12）
  abort_report:     {primitive: wait, notify: [M6, SUP]}   # 中止上报：停止发运动目标（零速由 M6 无源超时兜底，C8）+ 上报
  wait:             {primitive: wait}                      # 等待（退避载体）
# ---- 权威参考：outcome 常量表（P3a.0 log 原样摘录，禁止手改）----
outcome_tables:
  get_path:   {SUCCESS: 0, FAILURE: 50, CANCELED: 51, INVALID_START: 52, INVALID_GOAL: 53, NO_PATH_FOUND: 54, PAT_EXCEEDED: 55, EMPTY_PATH: 56, TF_ERROR: 57, NOT_INITIALIZED: 58, INVALID_PLUGIN: 59, INTERNAL_ERROR: 60, OUT_OF_MAP: 61, MAP_ERROR: 62, STOPPED: 63}
  exe_path:   {SUCCESS: 0, FAILURE: 100, CANCELED: 101, NO_VALID_CMD: 102, PAT_EXCEEDED: 103, COLLISION: 104, OSCILLATION: 105, ROBOT_STUCK: 106, MISSED_GOAL: 107, MISSED_PATH: 108, BLOCKED_PATH: 109, INVALID_PATH: 110, TF_ERROR: 111, NOT_INITIALIZED: 112, INVALID_PLUGIN: 113, INTERNAL_ERROR: 114, OUT_OF_MAP: 115, MAP_ERROR: 116, STOPPED: 117}
  recovery:   {SUCCESS: 0, FAILURE: 150, CANCELED: 151, PAT_EXCEEDED: 152, TF_ERROR: 153, NOT_INITIALIZED: 154, INVALID_PLUGIN: 155, INTERNAL_ERROR: 156, STOPPED: 157, IMPASSABLE: 158}
  move_base:  {SUCCESS: 0, FAILURE: 10, CANCELED: 11, COLLISION: 12, OSCILLATION: 13, START_BLOCKED: 14, GOAL_BLOCKED: 15, TF_ERROR: 16, INTERNAL_ERROR: 17, PLAN_FAILURE: 50, CTRL_FAILURE: 100}
# ---- 错误码 → 处置分级映射（键 = outcome_tables 常量名；SUCCESS=成功路径，不列即不走恢复）----
disposition:   # default 覆盖 FAILURE/NOT_INITIALIZED/INVALID_PLUGIN/INTERNAL_ERROR；move_base 表服务整链语义（navigate_to_pose / 回退形态共用）
  get_path:  {CANCELED: abort_report, INVALID_GOAL: abort_report, PAT_EXCEEDED: abort_report, OUT_OF_MAP: abort_report, STOPPED: abort_report,
              NO_PATH_FOUND: clear_and_replan, MAP_ERROR: clear_and_replan, EMPTY_PATH: replan, TF_ERROR: retry_local, default: replan}
  exe_path:  {CANCELED: abort_report, PAT_EXCEEDED: abort_report, INVALID_PATH: abort_report, OUT_OF_MAP: abort_report, STOPPED: abort_report,
              OSCILLATION: clear_and_replan, ROBOT_STUCK: clear_and_replan, MAP_ERROR: clear_and_replan,
              MISSED_GOAL: replan, MISSED_PATH: replan, BLOCKED_PATH: replan, NO_VALID_CMD: retry_local, COLLISION: retry_local, TF_ERROR: retry_local, default: replan}
  recovery:  {IMPASSABLE: abort_report, CANCELED: abort_report, STOPPED: abort_report, PAT_EXCEEDED: abort_report, TF_ERROR: retry_local, default: retry_local}
  move_base: {CANCELED: abort_report, COLLISION: clear_and_replan, OSCILLATION: clear_and_replan, START_BLOCKED: clear_and_replan,
              GOAL_BLOCKED: clear_and_replan, PLAN_FAILURE: clear_and_replan, TF_ERROR: retry_local, CTRL_FAILURE: retry_local, default: abort_report}
```

> 登记口径：C11 为新文件，登记 `04_配置改动diff.md`（新增行，理由 = N3）；C0 `recovery_behaviors` 列表来自本注册表（当前仅 clear_costmaps 占位条目，Q11）。

### C12 — ★★ 任务原语注册表（v4 新增，N4：契约先行 + SMACH 简版）

**文件**：`/home/ros1-slam/ws/src/50_task/lite3_task/config/task_primitives.yaml`（policy 资产，注册表 v1）。
**六原语**：navigate_to_pose / get_path / exe_path / recover / wait / clear_costmaps，每原语 = {action/service 目标， 类型， 关键 outcome 语义}；M5 编排层（一期 SMACH 2.5.2，二期 py_trees）**只依赖本注册表**，不直接耦合 mbf_msgs 细节，错误码语义引用 C11。

```yaml
# task_primitives.yaml —— M5 任务原语注册表 v1（v4 新增 C12，N4）
# 目标权威：action 类型与 Goal 字段名 = P3a.0 实测 log；action/service 话题名未实测 → Q11 回填
version: 1
primitives:
  navigate_to_pose:
    target: "action:mbf_msgs/MoveBaseAction"     # action 话题名随 mbf 服务器节点名，Q11 联调确认
    goal_fields: [target_pose, planner, controller, recovery_behaviors]   # MoveBaseGoal 实测字段；点单值取 C0 注册表 name
    outcome_semantics: "recovery_policy.yaml#outcome_tables.move_base"
  get_path:
    target: "action:mbf_msgs/GetPathAction"
    goal_fields: [use_start_pose, start_pose, target_pose, tolerance, planner, concurrency_slot]
    by_name_field: planner                       # 按名点单（§2.6 决策 5）
    outcome_semantics: "recovery_policy.yaml#outcome_tables.get_path"
  exe_path:
    target: "action:mbf_msgs/ExePathAction"
    goal_fields: [path, controller, concurrency_slot, tolerance_from_action, dist_tolerance, angle_tolerance]
    by_name_field: controller
    outcome_semantics: "recovery_policy.yaml#outcome_tables.exe_path"
  recover:
    target: "action:mbf_msgs/RecoveryAction"
    goal_fields: [behavior, concurrency_slot]
    by_name_field: behavior                      # behavior 点单值仅限 C0 recovery_behaviors 注册表 name（当前仅 clear_costmaps）
    outcome_semantics: "recovery_policy.yaml#outcome_tables.recovery"
    # ★ 命名空间边界：C11 recovery_actions 键（retry_local 等）为 M5 处置分级键，非点单值；处置键→behavior 点单值的映射由 SMACH 原语实现层决定（P6 接线按此实现，按 C11 键字面点单会被拒）
  wait:
    target: "local:timer"                        # 无 ROS 目标：本地定时器（params 默认 duration_s: 1.0）
    outcome_semantics: "succeeded/aborted 二态"
  clear_costmaps:
    target: "service:挂载点联调确认"              # ★ 实测未覆盖：mbf 侧 clear_costmaps 服务名/类型 → Q11（并入）；succeeded/aborted 二态
```

**一期 SMACH 最小验证状态机**（`/home/ros1-slam/ws/src/50_task/lite3_task/scripts/task_smach_min.py`）：
用系统 SMACH 2.5.2（P3a.0 实测可 import/实例化）写最小**顺序**状态机验证 C12 契约；transitions 键 = C11 处置分级；
**不接真实 action client**（话题名 Q11 回填后于 P6 接线）。

```python
#!/usr/bin/env python3
# lite3_task 一期最小顺序状态机（C12，N4）：验证 task_primitives.yaml 契约
import rospy, yaml
import smach

class PrimitiveState(smach.State):
    """读 task_primitives.yaml 原语契约；一期返回骨架 outcome，P6 换真实 action client（经 C11 disposition 映射）。"""
    def __init__(self, name, registry, outcomes):
        self.name, self.spec = name, registry['primitives'][name]
        smach.State.__init__(self, outcomes=outcomes)
    def execute(self, ud):
        rospy.loginfo('[M5] 原语 %s（target=%s）', self.name, self.spec['target'])
        return 'succeeded'

def main():
    rospy.init_node('lite3_task_smach_min')
    with open(rospy.get_param('~task_primitives')) as f:
        registry = yaml.safe_load(f)
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
        # transitions 键 = C11 处置分级：retry_local→retry / replan→replan / clear_and_replan→clear / abort_report→aborted
        smach.StateMachine.add('WAIT', PrimitiveState('wait', registry, ['succeeded', 'aborted']),
                               transitions={'succeeded': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('GET_PATH', PrimitiveState('get_path', registry, ['succeeded', 'retry', 'aborted']),
                               transitions={'succeeded': 'EXE_PATH', 'retry': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('EXE_PATH', PrimitiveState('exe_path', registry,
                                     ['succeeded', 'retry', 'replan', 'clear', 'aborted']),
                               transitions={'succeeded': 'succeeded', 'retry': 'EXE_PATH',
                                            'replan': 'GET_PATH', 'clear': 'GET_PATH', 'aborted': 'aborted'})
    rospy.loginfo('task outcome: %s', sm.execute())

if __name__ == '__main__':
    main()
```

> **py_trees 列二期**（外部依赖 R19：未装）；二期将同一注册表对接 py_trees/BT（§16）。

### C13 — ★ lite3_supervisor（SUP）职责定义（v4 新增，N5）

`90_supervisor/lite3_supervisor` 壳包 v3 从未赋职责。v4 定职责（Nav2 决策 7 lifecycle+bond 的 ROS 1 模拟）：

1. **按 mode 的有序启动清单**：SUP 在各 mode 顶层 launch 中**第一个启动**（§12 mode 表 SUP 列），按 health_watch.yaml 的 mode 组等待各模块 liveness 首跳后记 ready。2. **模块 1Hz 心跳监控**：各模块 launch 内置一行 rostopic 心跳（policy 层零代码），如 M4：`<node pkg="rostopic" type="rostopic" name="liveness_m4" args="pub -r 1 /lite3/liveness/m4_navigation std_msgs/Empty"/>`
3. **失联处置**：连续 `timeout_s` 无心跳 → ①发布 `/lite3/supervisor/hold=true`（M6 立即零速，C8）→ ②降级关停本 mode 模块（一期 scope：log fatal + 列出待停模块清单；实际关停接线在 P7 联调）。

**一期 scope（最小可用）**：`config/health_watch.yaml` + `scripts/health_monitor.py` 骨架，全文如下。

**红线（写死，入 §2.6 硬规则 a 与自检 [6] 语义）**：**SUP 绝不直接写 /cmd_vel**——`/cmd_vel` 唯一写者永远是 M6 `safe_cmd_vel_mux`；SUP 的全部输出仅有 `/lite3/supervisor/hold`（Bool）与日志。

```yaml
# health_watch.yaml —— SUP 一期 watch 清单（v4 新增 C13，N5）
# 位置：/home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/config/health_watch.yaml
# 红线：SUP 绝不写 /cmd_vel（唯一写者 = M6 safe_cmd_vel_mux，§2.6-a / 自检 [6]）
defaults: {timeout_s: 3.0}    # 1 Hz 心跳，连续 3 s 失联即触发处置
modes:
  mode_mapping:               # 本 mode 无 M6 → 失联不涉及零速通知（on_loss: degraded_shutdown）
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M1_slam, topic: /lite3/liveness/m1_slam}]
  mode_localize:
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M2_localization, topic: /lite3/liveness/m2_localization}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
  mode_nav_stub:
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M4_navigation, topic: /lite3/liveness/m4_navigation}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
  mode_navigation:            # 二期
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M2_localization, topic: /lite3/liveness/m2_localization}, {module: M4_navigation, topic: /lite3/liveness/m4_navigation}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
```

```python
#!/usr/bin/env python3
# /home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/scripts/health_monitor.py
# SUP 一期最小监控骨架（C13，N5）。红线：本脚本绝不发布任何 cmd_vel 类话题（唯一输出 = hold）。
import rospy, yaml, threading
from std_msgs.msg import Bool, Empty

class HealthMonitor:
    def __init__(self):
        mode = rospy.get_param('~mode', 'mode_nav_stub')
        with open(rospy.get_param('~health_watch')) as f:
            cfg = yaml.safe_load(f)
        self.mode_cfg = cfg['modes'][mode]
        self.timeout = rospy.Duration(self.mode_cfg.get('timeout_s', cfg['defaults']['timeout_s']))
        self.last_seen, self.lock = {}, threading.Lock()
        self.hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)  # 唯一输出，绝无 /cmd_vel
        for w in self.mode_cfg['watch']:
            self.last_seen[w['module']] = rospy.Time.now()
            rospy.Subscriber(w['topic'], Empty, self._beat, callback_args=w['module'])
        rospy.Timer(rospy.Duration(1.0), self._check)

    def _beat(self, _msg, module):
        with self.lock:
            self.last_seen[module] = rospy.Time.now()

    def _check(self, _event):
        now = rospy.Time.now()
        lost = [m for m, t in self.last_seen.items() if now - t > self.timeout]
        if lost:
            rospy.logfatal('[SUP] 心跳失联: %s —— hold=true（通知 M6 立即零速）；待降级关停 = 本 mode watch 全表', ','.join(lost))
            self.hold_pub.publish(Bool(data=True))

if __name__ == '__main__':
    rospy.init_node('lite3_supervisor')
    HealthMonitor()
    rospy.spin()
```

### 回退决策表（mbf 联邦式 → move_base 单体形态，v4 新增，N1）

**触发条件**：① Q9 用户不批 S9；② P6/P7 联调发现 mbf 阻塞（costmap 挂载 / 插件加载类问题短期不可解）——经主理人裁决切回。

| 项 | mbf 联邦式（主路线） | move_base 单体（回退形态） |
|---|---|---|
| C0 配置 | `mbf_navigation.yaml`（注册列表 + 改名参数） | C0 六底稿原样（`move_base_params.yaml` 直接用，已在 C0 存档）+ recovery_behaviors 参数清单（move_base 原生参数） |
| 全局规划器 | 经 S9 adapter 加载（global_planner/navfn 仅注册 nav_core） | move_base 原生加载（原生 nav_core，无 adapter） |
| 错误码 | 四 action 细粒度 outcome（15/19/10/11 档） | move_base actionlib **三态 outcome**（成功/抢占/中止） |
| C11 | 全量分级映射（outcome_tables 四表） | **降级为简化映射**：仅 retry_local（限次）+ abort_report 两级；replan/clear_and_replan 并入 retry_local 超限后人工介入 |
| C12 原语 | 六原语全量 | navigate_to_pose 照常（action 类型换回 move_base 标准 `move_base_msgs/MoveBaseAction`）；get_path/exe_path/recover 原语一期冻结（注册表保留，标注禁用） |
| C13 SUP | 不受影响 | **不受影响**（SUP 只依赖心跳与 hold，不感知导航形态） |
| C7 relay 源 | mbf 速度输出话题（Q11） | move_base `~cmd_vel`（v3 原形态） |

> 回退是**纯配置切换**（两套 yaml 均在 P3a 产出/存档，切启动 launch 即可），无施工回滚（§14）。

**Gate G3a（v3 基线 + v4 增强 N8）**：① `rosparam load` 上述 yaml 无报错（含 **`mbf_navigation.yaml`**）；
② `planners` / `controllers` / `recovery_behaviors` **三列表非空**（rosparam get 核对）；
③ **插件 type 字符串与 P3a.0 实测 log 中的插件描述符逐一比对一致**（planners/controllers 硬比对：teb 双注册描述符 + 原版 move_base_params 的 global_planner 串；recovery 条目 log 无描述符 → Q11 联调回填后补比对，不作为 G3a 硬判据）；
④ `note/04_配置改动diff.md` 覆盖 C2/C3/C4/C5/§7.2a + **C0（转换稿）/C5 升级/C11–C13 新增** 全部改动。

---

## §9 Phase 4：依赖核对

流程同 v2（先 source 再 import 的教训保留）。v3 结论：**R11 解除、Q2 关闭**（V13）——`message_transformer` 的 include 实测仅 `../include/protocol.h`、`../include/sensor_logger.h`、`message_transformer/ComplexCMD.h`、`message_transformer/SimpleCMD.h` 与标准 ROS 头；package.xml 依赖全为标准 catkin 包，批 5 编译它基本无悬念。

**v4 增补（N1/N8）**：若 Q9 已批且 S9 已装，P4 增一行核对——`rospack find nav_core_adapter` 应命中 **`/opt/ros/noetic/share/nav_core_adapter`**（dpkg ros-noetic-nav-core-adapter 0.3.0）；并补取证 `rospack plugins --attrib=plugin nav_core_adapter`（P3a.0 时未装，装后补录，供 C0 planners 加载路径佐证）。

## §10 Phase 5：编译构建

### P5.1 分批白名单（v3 修订，V12）

```bash
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;lslidar_driver"                                  # 批 1：M0 驱动
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="faster_lio;pcd2grid"                                          # 批 2：M1 建图
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="ndt_omp;fast_gicp;hdl_localization;hdl_global_localization"   # 批 3：M2 定位（最耗时：fast_gicp）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="message_transformer;lite3_drivers;lite3_slam;lite3_localization;lite3_perception;lite3_navigation;lite3_task;lite3_safety;lite3_bringup;lite3_supervisor"  # 批 4：全部自研壳 + message_transformer（v3：原批 4 teb/move_base 删除，壳包提前一次编完）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES=""                                                             # 批 5：清空白名单，全量收尾
```

> v4 注记：mbf_costmap_nav / nav_core_adapter（S9 后）均为系统 apt 包，**零编译零复制**（A9/A11）。

### P5.0 临时 swap（v3 新增，V14，编译前执行、P5 完成后撤除）

```bash
sudo fallocate -l 4G /tmp/build_swap && sudo chmod 600 /tmp/build_swap   # P5 前
sudo mkswap /tmp/build_swap && sudo swapon /tmp/build_swap
# …… 编译 ……（OOM 处置：-j3 → -j2）
sudo swapoff /tmp/build_swap && sudo rm -f /tmp/build_swap               # P5 完成后撤除
```

OOM 处置同 v2（`-j3`→`-j2`）；§P5.3 常见失败表继承 v2，
其中"livox 头文件"一行按 V4 修订：**`lidar_type=4` 走 Velodyne 分支，与 v2 的"C16 用 2"说法相反**，但结论一致——都不需要 Livox。

### P5.2 Gate G4：同 v2（devel/setup.bash 存在、`rospack find` 三例、`ROS_PACKAGE_PATH` 无 ysc）。v3 追加：`rospack find teb_local_planner` 应指向 **`/opt/ros/noetic/share/teb_local_planner`**（证明在用系统版，A9 生效）。**v4 追加（N8）**：`rospack find mbf_costmap_nav` 应指向 **`/opt/ros/noetic/share/mbf_costmap_nav`**（证明 mbf 在用系统版而非 ws 复制件，同 teb 的 A9 校验）。

### P5.4 ★ PCD 产物回收（v3 新增，衔接 V2）

路径已在 §7.2a 改写为 `/home/ros1-slam/data/maps/lite3.pcd`，**现场采图（Phase 3b）结束后**：

```bash
ls -lh /home/ros1-slam/data/maps/lite3.pcd            # 期望 >0 且随建图增长
roslaunch lite3_slam 11_postprocess.launch             # pcd2grid: PCD → /map
rosrun map_server map_saver -f /home/ros1-slam/data/maps/lite3
ls -l /home/ros1-slam/data/maps/                       # Gate G3：.pcd/.yaml/.pgm 三件齐
```

---

## §11 Phase 6：分层验证

> ★ **v4 前置（N7/Q8 硬停止线）**：本章任何起节点（stub）步骤之前，必须**重跑 §5 P0.5 双查（扩充正则）并与用户确认处置**——root 侧 foxy 栈（transfer_launch/jetson2app/jetson2motion，jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥 lite3_ros_bridge 长期在跑（Q8，§15）。

### 11.1 `scripts/env.sh`（v3 加守卫，V10）—— v4 不变

```bash
#!/usr/bin/env bash
export ROS_MASTER_URI=http://192.168.1.103:11312
export ROS_IP=192.168.1.103
source /opt/ros/noetic/setup.bash
# ★ v3 守卫：devel 不存在时明确报错而不是喷一屏 source 失败
[ -f /home/ros1-slam/ws/devel/setup.bash ] || { echo "[env.sh] ws/devel 不存在——先完成编译（P5）"; return 1 2>/dev/null || exit 1; }
source /home/ros1-slam/ws/devel/setup.bash
export ROS_HOME=/home/ros1-slam/.ros
export ROS_LOG_DIR=/home/ros1-slam/logs
export ROS_PACKAGE_PATH=$(echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -v "/home/ysc" | paste -sd:)
```

### 11.2 ★ 隔离自检 `scripts/verify/v3_isolation.sh`（v3 修复版 V10 + V2 盲区；v4 增补 [9]，N2）——文件名保持 `v3_isolation.sh` 不变（G2 证据链已引用该名）；[1]–[8] 语义与 v3 完全一致，P6 执行时整脚本按下面全文重新落盘（含 [9]）。

```bash
#!/usr/bin/env bash
FAIL=0
TMP=$(mktemp)
say() { echo "$1"; }
say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc" && { echo "  违规"; FAIL=1; }
say "[2] 自研壳中是否含 /home/ysc"
grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null && FAIL=1
say "[3] .bashrc 是否 source 了其他 devel"
grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null && { echo "  违规"; FAIL=1; }
say "[4] rospack 解析路径是否都在 /home/ros1-slam（teb 例外：系统包）"
for p in faster_lio hdl_localization lite3_bringup lite3_safety; do
  rospack find "$p" 2>/dev/null | grep -q "/home/ros1-slam" || { echo "  BAD: $p"; FAIL=1; }
done
say "[5] ★ 模块间是否互相 include launch（20 号 K3）—— v3 修复子 shell 丢 FAIL bug"
: > "$TMP"
for L in /home/ros1-slam/ws/src/*/lite3_*/launch/*.launch; do
  [ -e "$L" ] || continue
  SELF=$(basename "$(dirname "$(dirname "$L")")")
  grep -o 'find lite3_[a-z_]*' "$L" 2>/dev/null | sed 's/find //;s/)//' | sort -u | \
  while read -r n; do [ "$n" != "$SELF" ] && echo "$L -> $n" >> "$TMP"; done
done
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug；v4 语义注记：lite3_supervisor 亦在扫描范围（SUP 红线：绝不写 /cmd_vel，N5/C13）"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" > "$TMP" || true
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }
say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null && FAIL=1
say "[9] ★★ 40_navigation 壳包禁算法源码（v4 新增，N2/§2.6 硬规则 b：policy 层零 .cpp/.cc/.h）"
HITS=$(find /home/ros1-slam/ws/src/40_navigation \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" \) 2>/dev/null)
[ -n "$HITS" ] && { echo "  命中算法源码文件："; echo "$HITS"; FAIL=1; }
rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
exit $FAIL
```

### 11.3–11.5 验证矩阵、命令序列、验收判据

继承 v2，v3 修订四处（下表 1–4）；v4 增补（N7/N8，下表 5–7）：

1. **M0 拆分校验（V1）**：G5 判据追加一条——`rosnode list | grep ros2qnx` 期望**为空**（stub/mapping/localize 模式下出现即为事故）。
2. **M1 判据（V5/V2）**：`rostopic hz /Odometry` ≥10 Hz 的前置是 faster-lio 真的吃到点云——直接验 `rostopic hz /rslidar_points` ≥9 Hz（M0 已过 G5）再起 M1；PCD 落盘位置以 §P5.4 为准。
3. **M4 判据**：单位阵 TF + `teb_markers PolygonObstacles` 非空不变；**`rostopic info /cmd_vel` publisher 只能是 `safe_cmd_vel_mux`**。
4. A1–A5 / 20 号十条判据表整体保留。
5. **G5 判据追加（mbf action 族，N8）**：mode_nav_stub 下 `rostopic list` 可见 mbf 三 action 话题族（GetPath / ExePath / Recovery；精确话题名随 mbf 服务器节点名，Q11 联调回填后固化到判据表）。
6. **G6 判据追加（SUP 心跳，N8）**：SUP 心跳话题按 `health_watch.yaml` 当前 mode 清单**全部存活**（逐条 `rostopic list` 可见且 `rostopic hz` ≈ 1 Hz）。
7. **外部进程检查提示（N7）**：mode_nav_stub 下 ros2qnx 禁令保留（判据 1）之外，P0.5 扩充正则同时覆盖 `jetson2motion` 类 root 侧外部进程——起节点前双查命中即停下走 Q8 确认，不得"先起再说"。

## §12 Phase 7：场景组合（modes/）

> ★ **v4 前置（N7/Q8 硬停止线）**：与 §11 相同——P7 起任何 mode 前，重跑 P0.5 双查（扩充正则）并与用户确认处置；真机 mode（含 ros2qnx）启动前强制重查运动链路侧。

三层 launch 与铁律同 v2（20 号 K2/K3 承接：/cmd_vel 唯一写者 M6、模块间不互相 include）；**v4 铁律补一句（N5/C13）**：SUP 在每个 mode 顶层 launch 中第一个启动，且 SUP 仅有 `/lite3/supervisor/hold` 输出，绝不写 /cmd_vel。

mode 表 v3 修订 + v4 SUP 列：

| mode | 包含 | SUP 启动位与 watch 清单（v4） | v3/v4 变更 |
|---|---|---|---|
| `mode_mapping` | M0(驱动+**qnx2ros**) + M1 | SUP 最先起；watch = `health_watch.yaml#modes.mode_mapping`（M0/M1） | v3：明确 M0 仅 qnx2ros；v4：+SUP |
| `mode_localize` | M0(同上) + M2 + M6 | SUP 最先起；watch = `…mode_localize`（M0/M2/M6） | 同上 |
| `mode_nav_stub` ★ | M0(**仅驱动+qnx2ros**，不含 ros2qnx) + M4(**mbf 联邦式服务器**，stub TF) + M6 | SUP 最先起；watch = `…mode_nav_stub`（M0/M4/M6） | **V1：真机运动桥严禁入列（原样）**；v4：M4 升级 mbf（N1），G5/G6 增 mbf action 族 + SUP 心跳判据（N8） |
| `mode_navigation`（二期） | 真定位 + 真导航 + **此时才允许 ros2qnx** | SUP 最先起；watch 全链 | 启动前强制重跑 P0.5 运动链路检查（v4：扩充正则 + Q8 用户确认） |

## §13 跨目录风险表（v3 修订版 + v4 增补 N10）

R1–R10、R12 继承 v2；v3 修订与新增、v4 新增如下：

| # | 风险 | v3 变更 / v4 处置 |
|---|---|---|
| R11 | message_transformer 隐藏依赖 | **v3 降级为低**：实测 include 自包含（V13），保留"批 4 单独编"的处置 |
| **R13** 🆕v3 | **`ros2qnx` 写穿隔离 = 真机失控** | stub/mapping/replay mode 出现 ros2qnx 即事故。防线：§2 launch 禁入 + G5 校验 `rosnode list` + P0.5 双查 |
| **R14** 🆕v3 | **ACL 与 chmod 互相破坏** | P0.6 `setfacl -bR` 先清（V11）；P2 末如需 ysc 便利通道，ACL 在 chmod 之后重建 |
| **R15** 🆕v3 | **话题断链**（`/rslidar_points_filtered` 无生产者类问题） | 教训：复制来的 yaml 里的**每个 topic 参数**都要与上游 launch 的实际发布名对账（本文已对账：imu_topic ✅ / lid_topic 已修 ✅ / pointcloud_topic ✅） |
| **R16** 🆕v3 | **第三方包内硬编码路径**（`laser_mapping.cc:751` 类） | 复制后立即 §7.2a 改写 + 自检 [8] 永久监控；登记 diff |
| **R17** 🆕v4 | **mbf 参数/底稿格式差异**（move_base→mbf 迁移错漏） | 已实测给出转换表（§8 C0：注册列表格式、改名三项、make_plan_* 不迁移）；**残余风险 = costmap 挂载命名空间未实测（Q11）**——P6 mode_nav_stub 首日 `rosparam get` 复核后固化 |
| **R18** 🆕v4 | **SUP 误碰 /cmd_vel（零速通知变直控）** | 规约级防线：§2.6 硬规则 a（SUP 只能通知 M6 零速）+ 自检 [6]（lite3_supervisor 在扫描范围）；SUP 输出仅 `/lite3/supervisor/hold` |
| **R19** 🆕v4 | **py_trees 外部依赖（二期）** | 一期不用（SMACH 2.5.2 系统版实测可用，Q10 关闭）；py_trees 未装，二期评估装法与版本后再入计划（§16） |

## §14 回滚：同 v2。补充一行：faster-lio 源码改写若出问题 → 按 §7.2a 的 diff 记录逆向恢复（或整包重复制 + 重做改写，注意坑2：751 与 862 两处）。

**v4 补一行（N1）**：v4 配置回退 = 按 §8 回退决策表切 move_base 形态（**纯配置，无施工回滚**；C13 SUP 不受影响）。

## §15 工期与待确认项

| Phase | 内容 | v4 修订后预估（N10） |
|---|---|---|
| **P0–P2** | 用户/组/ACL/窗口 + 目录/脚手架/bashrc + 复制/改写/权限 | 15+20+40 min（✅ 已执行 2026-10-04） |
| **S9** | **nav_core_adapter 安装（用户 sudo）** | **5 min（Q9 批准后；不批 → §8 回退决策表）** |
| P3a | 配置重构（C0–C10 + **C11–C13**，含 P3a.0 收编与 mbf_navigation.yaml） | **2–2.5 h**（v3 估 1–2 h） |
| P4 | 依赖核对 | 10 min |
| P5 | 编译（四批，无 teb/move_base/map_server）+ swap | 1–2 h |
| P3b | 现场采图 + PCD 回收 | 1–2 h（要现场走一趟） |
| P6 | 分层验证（含 **SUP 联调 +0.5 h**、Q11 mbf 复核） | **3.5–5.5 h**（v3 估 3–5 h） |
| P7 | 组合启动 | 1 h |
| **合计** | | **约 2 个工作日**（不含采图往返；P0–P2 与 P3a.0 已完成不计入剩余工期） |

| # | 待确认 | 状态 |
|---|---|---|
| Q1 | ros1-slam 能否登录 | 开（P0.1 解决） |
| ~~Q2~~ | ~~message_transformer 隐藏依赖~~ | **✅ 关闭（V13：自包含）** |
| Q3 | C16 安装倾角（定 `min_z`） | 开（P3a 先验 10 min） |
| Q4 | `/Odometry` 话题名与频率（二期 SCAN 输入） | 开 |
| Q5 | `location_record.py`（`tools/` 下）图形环境 | 开（ssh -X / HDMI / 手写 JSON） |
| **Q6** 🆕 | `pcd_save.interval=0` 的落盘语义（走查显示 747 行条件恒假可能永不落盘） | 开；v3 已先按 `interval: 10` 处理，现场验证 |
| **Q7** 🆕 | ysc 侧 `/rslidar_points_filtered` 的真实生产者（可能是未复制的过滤节点，或该 yaml 从未在 ysc 侧生效） | 开；沙箱已按 `/rslidar_points` 裁决，不影响本文 |
| **Q8** 🆕 | **root 侧 foxy 栈（transfer_launch/jetson2app/jetson2motion，jetson2motion 订阅 /cmd_vel 直发底盘）+ ysc monitor 桥（lite3_ros_bridge）长期在跑**（坑3） | **开；硬停止线（N7）：P6/P7 起节点（stub）前重跑 P0.5 双查并与用户确认处置（§11/§12 前置）** |
| **Q9** 🆕 | **S9：`sudo apt install ros-noetic-nav-core-adapter`（0.3.0，官方源）由用户执行** | **开；不批 → §8 回退决策表 move_base 单体形态** |
| ~~**Q10**~~ | ~~smach 可用性~~ | **✅ 关闭（P3a.0 实测：smach/smach_ros 2.5.2 import + StateMachine 实例化 OK）** |
| **Q11** 🆕 | **mbf 联调确认包**：① costmap 三底稿挂载命名空间（~global_costmap/~local_costmap 复核）；② clear_costmaps 服务挂载点/类型；③ recovery_behaviors 注册表 type 串与 mbf 侧可加载性（是否同样需 adapter）；④ mbf action 话题族精确名（随节点名）与 C7 relay 源话题名（mbf 速度输出）；⑤ make_plan_* 的 GlobalPlanner 参数等价物 | **开；P6（mode_nav_stub 联调）关闭后回填固化** |

## §16 二期与路线（**v4.5／v4.6 收编**：由一节扩为 §16.1–§16.11；每节只写「结论一句＋实证指针＋状态」，不搬长文）

> **收编来源**＝v4.3（X1–X31）＋v4.4（X1–X4）＋v4.5 落册候选稿（Part 1／4／5／6／7）＋v4.6（Y1–Y6）。**生效时刻＝2026-10-12 主理人明文「按此落册」**。
> **写法约束（Y6，本批明文）**：本节凡「已解决／已完成／已修／达成／验证通过」均须同段给实证（证据目录名／件与字段／批号三者之一）；给不出的**一律写"未验｜待验｜部分完成"**。

### §16.1 假设与载体边界（承 X7、X21、X15、X16、X23–X30 的载体侧）
- **结论**：现仿真载体是**刚体躯干盒、无腿、无近场地面回波** ⇒ 能验决策／几何／代价／规划四层，**不能验**踏步／落脚／摩擦／连续台阶。实证＝G 稿 §0／§3 ＋ R82 探针。
- **挂账 O-11（载体保真度缺口）＝未验**：`set_model_state` 写不住机体偏航（指令 30/60/90° ⇒ 回读 14.8/28.6/40.5°）且 `/LIO/odom_vehicle` 朝向恒 0 ⇒ 现仿真**不能表达非零机体朝向**。实证＝R71 证据 `p4_body_frame_probe_20261009_203307/`。**是否进路线图＝待裁（小美＋主理人）**。

### §16.2 M5 仿真验证方案与结果（代际纪律）
- **状态＝部分完成**：M5.2 三跑已在**新 so `147dc8cc`** 上重取（三 goal 到达偏差 0.080／0.012／0.008／0.016m 级）；旧 so `f1a7c449` 之前的读数**不得与新代混引**（X10 引用规则，07 档"被测件面注记"承载）。B3 的 `dt≤60s` 时基口径已随批3 结案（根除时基）。
- **未验项**：M5.2 是否覆盖 goal 多样性以外的场景＝未验（现行只有两 goal 形态）。

### §16.3 全局任务优先级总排序（**状态刷新，均为在册证据**）
- #3 的 A7 **P1／P2 判据已达成**（实证＝R141 三连读数包 `aa_A7P4三连读数包_交小美判.md` 与 `v_scan_p1.sh`／`v_scan_p2.sh` 留底）；**P4 未达成**（合并有效样本 6 跑＝1 PASS／5 FAIL，实证＝`r119_p4_second_clause/rejudge_two_thresholds.txt`，分母 11）⇒ 表述为"上游具备避让能力但**非恒定（1/6）**"。
- **批1＝未称完成**（①②⑩ 已销 3/11、③–⑨⑪ 未做）；批2 收口材料已交待判；批4 件面完成、活体并入 G8；#6 预检完成、本体待双闸；#8 仅设计落稿；#7 未立项；批3（坑25）已结案（实证＝对应进度文档与证据目录）。
- **真机 Gate 前置（X17 已裁＝C3 案 B）**＝"仿真闭环集合到位 **且 P4 达成**"⇒ **P4 是关键路径最前的未达项**。

### §16.4 算法测试矩阵、判据与门值现状
- **已定档（有实证）**：P4 避让第二式门值 **0.40**＝箱半宽，语义"轨迹不得进入箱体、擦边算过"（A5 已裁 2026-10-10；实证＝`r119_p4_second_clause/rejudge_two_thresholds.txt` 两档对照：avoid@0.60 过 1／@0.40 过 2，**分母＝有读数样本 11**）。
- **A23 ＝待裁**：`min_dist_to_any_occ` 尚未纳入判定；施工侧推荐 0.10m（＝`OCC_TOL`，沿用 `OCC_TOL` 常量）。**两条限定必读**：①"最小 0.181"须读作"判 PASS 者最小 0.181"（全枚举最小 0.004m 属当时判 FAIL 的窗，实证＝`r191_number_audit/g2_clearance_full_census.txt`）；②**在册的 `OCC_TOL` 今天从不进判定链**（`grep` 实证：取证件内只有常量定义与 `gate_occ` 打印两处；判定链用的是 `collide = robot_min <= 0.40` 与 `avoided = traj_min >= AVOID_DIST`）⇒ 若纳入，须按 `05` C2 的形状做**真实 FAIL 一次**的对照（离线合成穿墙轨迹判红＋撤后判绿＋两面不同）。
- **A25 ＝待裁**：VCC 勘误窗宽**六档**实测在案（±20／±40／±60／±100／±200／整行；实证＝`r163_vcc_census_first_run/fixup_window_sweep.txt` 与现役面重放 `r206_material_refresh/a25_window_sweep_today_face.txt`）⇒ **留名须带四元组**（件版本／面／窗／列名），因当年 `RED` 那列今天整列叫 `豁免红`。
- **A25 已裁定（v5，Z4）＝已裁待施工**：VCC 勘误窗缺省 **±60 → ±40**，并把**四元组留名（件版本／扫描面／窗宽／列名）升为在册规矩**——此即 A32① 被追认的范围，G 段其余部分维持未追认。盘上现状＝件缺省仍 ±60、志文面基线按 ±60 建（RED 1／AMBER 43 等十项）；施工落地点＝#110（改缺省后须另存新基线、旧基线不覆写；按在册六档表 ±40 那档＝RED 4／AMBER 31／勘误句 66／OK 423／改档引用数 99）。上句"A25＝待裁"属裁定前状态、不回改。
- **A23 已裁定（v5，Z6）＝已裁待施工**：`min_dist_to_any_occ` **纳入判定**、门值 **0.10m**、来源＝**env 覆盖 `P4_OCC_TOL`（缺省 0.10、单一真身在代码）**，并写入**升级条件**（若出现"同一批跨多张不同分辨率图反复扫"，先请示再升 yaml／launch，不自扩）；阳性对照同批做（离线合成一条明知穿墙的轨迹要求判红、撤掉判绿、**两面必须不同**，不得以同判据历史命中代替）；报数形状＝**两列并报** `avoid@0.40`／`breach@0.10`，不合成单值。盘上现状＝判定链**仍不引用**该值（该常量只被定义与打印）⇒ 名实不符由本格了结；施工落地点＝#111。
- **X31／A23 之外**：未明文前**判定链不动**（施工侧不自落门值）。

### §16.5 Wave1–3、G9 预案与远角红线的一次性豁免（v4.4 X1）
- **红线现状**：远角 4 对**禁止复跑**；v4.4 曾给"一次性诊断豁免"（仅 1 对 × 至多 2 次**有效被试读数**），**该豁免随 R174 窗口关闭即失效**，此后任何远角复跑**均需另一次明文**。
- **不变式（写死）**：①不改写 perlin3d 标准档"**裁撤**"定性；②不豁免第 2／3／4 对、不做 8 对全量；③扩面须另一次明文。
- **A27 已裁定（v5，Z7）＝已生效（口径级）**：凡"至多 N 次尝试"一律按**独占活体窗数**计 ⇒ R174 那族已用 **7 窗**、**远角额度已耗尽**，此后任何远角复跑**均需逐次明文**；"有效读数 5 条（回话 2＝3.2s／60.2s，未回话 3＝60.0／30.0／90.0）"作为**另一单位并列保留、不换算**。上句"未裁"属裁定前状态、不回改。
- **A27 计数口径＝未裁**（施工侧不自选）：两种读法的数字都有实证——按"有效读数"＝尝试 5 条（回话 2 条＝3.2s／60.2s，同出 attempt1；未回话 3 条＝60.0／30.0／90.0）；按"开窗次数"＝**7 窗**（缺 6 号）⇒ 采后者则豁免额度**已超出**。实证＝`r205_role_reviews/50_r174逐窗清点.txt`。
- **A30 已裁定（v5，Z8）＝已生效（文字级）**：本节"等不到 result"的结论加一条**作废护栏**——**若服务器在放宽 patience 后能出 result，则本方向表述作废**、按新读数改写；护栏只覆盖"该对本图本 boot"，不外推为普适。
- **禁引**：`r174` attempt1 的 verdict 行与同件字段互斥（字段 `result_returned=true`／`wall_s=60.2` 而文字写"仍无 result ⇒ 升为已证"）⇒ **该行不得作为任何结论的出处**，更正另起句在册（`02` ㉕、`04` R205）。

### §16.6 工具与门禁（**v4.6 Y1：正式串六件→七件**）
- **正式串＝七件**：SYN／GTC／ISO／DC／`v_py_static`（范围甲 50 件）／`v_evidence_reachability`／**`v_verdict_field_parity`（结论行 vs 字段值并排核）**。第七件入门实证＝主理人 2026-10-11 明文＋自测 **PASS=9**（含三态：全已知须 0／清表须拦住／缺表须 rc=9 不判）＋当代读数 `已知=7、新增=0`（留底 `r211_gate_entry_first_run/`、`r212_parity_mutation_probe/`）。
- **候补盘子＝六件、全部未入**；申请包口径仍是"五件一次签"（实证＝`u` 稿 §9／§14、`ac` 稿 §8）。**VCC 已装 `#12-甲`「已定性伪红」豁免档**并自报 `[面条件]`／`[面缺件]`，自测 `PASS=11 FAIL=0`。
- **已裁待施工（v5，Z3／Z5／Z4）——三条都已明文、盘上均未落地，施工批须同批改本节本格并附复跑读数**：①**Z3 账构件已裁入串**（七件→八件），成闸三参数＝判定域下限 **R140**／FAIL 集＝**C2（缺条目）＋C4（滞后）**／滞后阈值 **2 批**（设 0 会把每个纯文档批报成滞后＝系统性假红），C1 档头账实与 C3 多号**只报不判**；参数取自施工侧推荐、标"待追认"；②**Z5 VCC 已裁入门**，语义照第七件四件套（档位＋存量表带出处＋门用面写死＋**缺表 rc=9 不判**、新增才拦存量逐条带出处），并**要求"同一目标:行全局同判"**、**直接撤掉"待移交单清零"前置**；③**Z4 窗宽缺省 ±40**（见 §16.4）。落地点＝#110／#112。
- **取证形状现况（v5，Z1／Z2）＝已生效**：A24「测量污染」档与 #14「两份输出分件」**已同时撤销**——取证件 401→**378 行**（现跑 `POLLUTION_DIST` 出现次数=**0**）、harness 174→**158 行**（**同批撤销同向自证段 20 行**：撤分件后只剩一份，留着等于每窗恒定 rc=9 假阳性）；**保留 `dump()` 的单一真值**（机读字段一律由判据行反推＝坑62 的修法，不在 #14 范围）。实证＝离线自测 **4/4 全过**（四档：判据行同流／字段同向／整份不可 json.loads）＋SYN rc=0＋PYST rc=0（未定义名 0）＋反扫改前后各 rc=0（零行号引用）；留底 `r220_revert_a24_p14/`。**对价照实**：`p4.json` 首行为中文结论句、整份不是合法 JSON；`dist_end`＞100m 的系框失真轮次重新落 `FAIL|扰动后未到达`、重新计入 n=3 分母（实例＝R120 的 5094.464m）。
- **入门通用三问（`ac` 稿 §14）**：档位定了吗／存量表带不带出处·删行会怎样／门用面写死了吗·缺表时怎么走 —— **答不齐不得进门**。

### §16.7 仿真定位（裁决句，沿用 v4.2）
- 仿真＝**正式回归工具、不是 Gate 替代**（不变，背书引用）。

### §16.8 多地图世界与公开 world 路线
- 多地图世界与 `95_sim.launch` arg 化路线不变。
- **代价映射约束（Y2 降档后写法）**：`trinary_costmap` 开启时源图只出三值 ⇒ **"写多少读多少"不成立**，但这是**可关的量化后果、不是 costmap 的天性**（实证＝`r59_w3_precheck_20261009_0522/probe/cost3.txt` 内 `a[20*40+20]=50` 读回 0；开关见在册 `costmap_common.yaml` static_layer 块）。**A 档叠加问题＝未证实可叠加**（见 §16.9）。

### §16.9 二期路线图 SCAN→2.5D→多层→PCT（**X1／X2 按 Y2 落，均为"未验"**）
- **C12 已裁**＝2.5D／高程方向**暂时搁置、放着存档即可**；**不得因搁置清理载体**（`30_perception/deeprobotics_local_height_map/` 的 `CATKIN_IGNORE` 与骨架原位保留，`.cpp/.h`＝0 属上游缺件）。下落点与恢复入口**单点写在 `ws/README.md` §6.3**。
- **X1（2.5D 与多层的关系）＝事实句已定、路线句未定**：事实＝单张 2.5D 表达不了多层（**per-map** 层面），**未证**"每层各挂一张单高度图（per-system）不可行"；路线选择**挂 C12 恢复＋#46** 之后。主理人已裁 **Z1 第四解法 (iv) 不做**（其载体正是 2.5D 高程图 ⇒ 只作存档事实、不进路线图、不写"已解决"）。实证／待补＝`F_多层导航起工设计` §2、`m_3d多层` §0。
- **X2（双 StaticLayer 组合）＝未证实可叠加**，非"被推翻"（Y2 降档；实证缺口＝探针面未开 `use_maximum`／`first_map_only`、四格对照只有"换序"一格留有原始读数）。**恢复该方向前的待补测四格**：①在册参数面（`use_maximum: true`＋`first_map_only: true`＋三量化开关）下的双实例对照；②两开关四组合；③与现役同插件构成（static＋stvl＋sob 三插件）；④**每格留原始件**（`rostopic echo` 落盘＋`data:` 计数断言）——**四格未跑齐前，本节不得写"已解决"**。
- **已裁待施工（v5，Z14）＝四扇窗全部待命**：X2 的**待补测四格＝排仿真短窗**（非远角、不碰红线）；0-a 取 outcome 码改走**非远角 sim 面**，主理人另给"这个特殊情况窗数多给 2 次"的例外，**范围经 6 号裁确认＝只 sim 面**；#103 取证件"还原"返工**并入同一扇窗**（状态仍＝纪律已立、返工未验）；J1 判 P4 达成前**先扫 `P4_SEED`**；J3 保留并加做 goal 多样性。门槛＝1 分钟 loadavg ≤4，而 04:41 现跑 lite＝**28.99**（榜首 `whoopsie` 83.7%、`is-active`＝active）⇒ **未起窗、未取任何读数**。
- **路线纪律（X26）**：先解 P4 的几何／配置重叠（C10、C9）再谈 3D 与多层。

### §16.10 楼梯与可通行性四方向（**v4.3 的落点原指"新增 §16.10"，本节即其新建**）
- **框架（X3）**：一条链五位置（表征／可通行判据／代价地图／规划／执行）；四方向落点分层，**选型＝方向一先做、方向四做先行子集、方向三后置独立立项、方向二与一合流**。
- **方向四先行子集＝部分完成（有实证）**：八实体场景集＋参数化 `stair_lab_*.world`（**不被任何在册 launch 引用**、`m1_world.world` 未动）＋预检件 `stairstudy/{fixtures.py,precheck.py}`（**T0–T9 十闸 rc=0**，实证＝R64 留底）＋标注工具（C 混合：人只改源标注 yaml，机器产物只读）。**三条红线写在代码里**（未标注不塌陷成可走、同格语义冲突直接红、生成物 sha 不符即红）。
- **第四通道＝未验**：周期性**在单帧剖面上读不出来**（`peri_amp` 四档为 0，唯一非零格可见长度 0.30m＜两个踏步、属自相关伪峰禁区，已被自检 T9 拦下）⇒ 须拆成"单帧三特征＋跨帧累积周期性"两段（X15-①）。
- **仿真图案边界（X15-②／X16）＝已实证**：竖向 50 线摊 ±90°、间隔 3.67°，楼梯立面 ±15.5° 带内只有 **8 条线**（真机口径 32 线）⇒ 三只楼梯 `surf` 全面反向 ⇒ **拿现仿真验第四通道＝验一件不存在的能力**；可判距离窗碎且非单调（实证＝R64／R65 留底）。**未裁前不改 xacro 竖向参数**（改则 R64／R65 读数作废、须整批重取）。
- **近场与 blind（X23／X24／X28／X30，门值一律未改）**：注入体实为 **0.8³、心 z=0.40**；存活点距原点 ≥0.98–1.09m ⇒ 主因＝`blind` 的**距离剔除**（生效位置＝`pointcloud_preprocess.cc` 第 208 行 **C16Handler** 分支，条件 `lidar_type: 4`；第 187 行属本链不走的 VelodyneHandler——X29 更正）。**硬约束（X28-⑤）＝已实证**：该球顺手删掉每帧 64.5%–72.5% 的 `(0,0,0)` 占位点 ⇒ **只换成盒＝总点数 ×5.84**（47943→280000）⇒ 正确形状＝「机体长方体 ∪ **显式剔除占位点**，且剔零必须独立一条」。

### §16.11 计划书同步、互检与引用纪律（**P8 全文收编＋v4.6 Y3／Y4／Y6**）
- **程序令 P8（X12）**：①新增／撤销／改级别任务，②改判据或门值（**含门禁串组成变更**），③变更能力边界或技术路线 ⇒ **必须同批产出补丁稿**并在 `note/00_README.md` 登记一行；只写进 `04`／进度文档而不动计划书＝**流程未完成**（可报"施工完成"、不得报"阶段收口"）。**违例自证在册**：v4.3 草案 2026-10-05 立、10-09 才补齐，期间 R52–R62 十二批方向性决定未同步。
- **P8 第③类样板（v4.4 X3）**：撤销／变更在册红线的合规动作＝**同批补丁稿＋README 登记＋04／总览同批落账**，且补丁必须自带**不变式**（写明"不改什么、不授权什么"），否则一次豁免会被后批读成长期开路。
- **互检与"旧定义必须消失"（X6）**：同一判据两处在册定义 ⇒ 必须有互检断言；搬家的收尾判据＝**旧定义从盘上消失**；缓做条款必须挂在 X 结案的清单上（实例：三档 variants 前置 Q11 结案五天无人回收）。已落为闸：`v_deferred_clause_scan.sh`（正式串第四件，rc=1＝在册已知态）。
- **引用代际规则（X10）**：代际轴＝SM∪LIB 提交数 ⇒ 只改被测件面**不产生新代际行**，须连载体代际＋被测件面 sha 一起引用，两代证据不可混引（现行＝**H14 ＋ so=147dc8cc**）。
- **引用格式三条**（v4.3 X29 ＋ A32 纪律 ＋ **Y3 新立第③条**）：①代码实现位置＝**行号＋所属函数名＋enum／分支条件三件齐**并附执行路径证据；②志文引用用**锚文本**、行号须带版本限定；③**凡下"已证／未证／被实证推翻／不可终止／坐实／确证"级结论，同句须给出字段名＝字段值**（条文＝`05` 追加八；与第七件互成对偶——件筛"引用了字段名的矛盾"，本条保证"该被筛到的句子都留了痕迹"）。**③尚未配机器闸＝待裁**。
- **写法约束（Y6，本批明文）**：凡「已解决｜已收口｜已完成｜已修｜达成｜验证通过」**必须同段给实证**（证据目录名／件与字段／批号至少其一）；给不出只能写"未验｜待验｜部分完成"。**历史状态句按本规复查**：批1 全文**不得称"批1 完成"**；返工项类一律写"纪律已立、待验"（`05` 追加九）。**来历（本会话三次实证）**：过期状态句被后批当事实引用——X22"候补闸未进门禁串"、"M 稿 §0 已分清"、"三档／五件"等四个错数（详见 `02` ㉓㉔㉕ 与 `04` R205／R206）。
- **追认状态（v5，Z9／Z10）＝已生效（纪律级）**：A31 **三条逐条追认**（`05` §十二 运行时写参数回执三取＋对照件须在面内／§十二补 判据必须能失败＋行数须注明计数口径／R186 三条硬要求＝未变异同果·关判据·两面必须不同）；A32 **只追⑥⑦⑧**（时刻只认 `date` 与 git、"只报数"的格子要问两问、凡计数必须写明单位与件版本），①的范围由 Z4 的四元组留名覆盖、G 段其余维持未追认，②③④⑤各挂在 A22／A25／A31／#12 上不重复开单；⑨＝主理人已明文立（下强结论须同句给字段名＝字段值）。
- **呈报格式（v5，Z13）＝已生效，真身在技能包**：凡请求裁定一律按五段——①现在的实况（带字段名＝字段值／读数／批号与原件落点）②选项与后果 ③连带影响与**不裁的后果**④"请勾"一行 ⑤末尾一次性可勾清单；**呈报轮＝只读轮**（未给明文前不写台账、不动判据／门值／门禁串、不 commit）。件位置＝`software-company/SKILL.md`（94→120 行，R219）；②段样式已裁为**复选框逐条＋每条写意义与代价**，回写落地＝#116。
- **开工动作（v5，Z12）＝已裁待施工**：①每批开工一次**写测试**（写不进先报"工具层不可写"、不得继续算读数）；②起活体窗前做**负载归因**（不止读 loadavg，须点名榜首 PID／属主／命令行）；③窗期间出现大负载源 ⇒ **暂停仿真并上报**。来源＝本会话两次实测（`/tmp` 100M tmpfs 写满致全工具 ENOSPC；vscode-server 索引器 126% CPU 使 lite loadavg 达 35.66）。落地点＝#116。
- **行政与真机前置（v5，Z11）＝部分已生效**：A33＝适用全部名册角色（已生效，在册句不回改）；真机 H 段**只追 H2（断电断链一律记 INVALID）／H3（0~40℃ 且开窗前后各记一次现场温度）／H5（现取本 boot 红线基准、见证覆盖面＝4）**为正式作业单，H1／H4 标现场条件；量具＝主理人自述"用工业量角尺测高度和角度"⇒ H4① 的读数来源已定、**待实测值到位**才能把 Q13 俯仰精测从"前提未实测"升档。
- **名册前置的读法（Y5／A33 甲）**：名册上"需小美复验／需小张复核"类前置＝**施工侧自起只读子代理出具意见＋主理人签字**，意见不自动等于裁定；F 段原文不回改、按此读。

## §17 计划书状态
- ✅ **v5 收编生效（2026-10-12 07:44，主理人明文"综合统一落册、作为第五版计划书"）**：v4 正文＋v4.7 补丁稿 Z1–Z14 并入本件。**已生效**＝Z1／Z2（取证件与 harness 撤回归，实证与留底在案）、Z7／Z8（计数口径与作废护栏）、Z9／Z10（A31 三条与 A32⑥⑦⑧ 追认）、Z11 的 A33 部分、Z13（呈报格式，真身在技能包）；**已裁待施工**＝Z3（八件串＋三参数）、Z4（±40＋四元组落件）、Z5（VCC 入门）、Z6（A23 纳入）、Z11 的 H 段三条、Z12（开工三动作）、Z14（四扇窗）；**未决**＝无。


- ✅ 环境二次实测（新增：ACL 现状、swap、系统导航栈、message_transformer 节点与 include、faster-lio 参数命名空间与存盘路径、驱动话题名、lidar_type）
- ✅ 真机安全缺口已闭合（V1：qnx2ros/ros2qnx 拆分 + G5 校验 + P0.5 双查）
- ✅ 隔离体系补盲（V2 + 自检 [8]：第三方包内硬编码路径纳入监控）
- ✅ 四处事实错误修正（V3/V4/V5 + location_record 路径）
- ✅ 流程硬伤修复（V6 时序、V7 脚手架、V8 配置种子、V9 权限时序、V10 脚本 bug）
- ✅ ACL 冲突处置（V11）；✅ 编译优化落地（V12/V14：少三个包 + 临时 swap）
- ✅ **P0–P2 已按 v3 执行完毕（2026-10-04：G0/G1/G1.5/G2 全 PASS，见 `note/进度/00_总览.md`；正文冻结注记 N9）**
- ✅ **P3a.0 导航栈组件只读实测完成（2026-10-04，690 行 log；N1 事实基础，Q10 随之关闭）**
- ✅ **N1–N10 设计决策收编（M4 升级 mbf_costmap_nav 联邦式 + 契约先行/SMACH 简版 + SUP 职责 + 八决策规约 §2.6）**
- **v5.1 修订（治 R14／R37）**：把"设计冻结"“工程完成"“正式验收""增量待施工"四件事**分开记录**——① **历史设计冻结**：§5–§7（P0–P2）正文只作从零重建脚本，续施工不得整段重跑；② **已施工完成（在册记录）**：P3a／P4／P5 与 P3b／G3 及后续 M2／M5／SCAN 系列有进度与证据留位，引 `note/进度/00_总览.md`——**引用在册记录不等于本会话重跑验收**；③ **正式验收**：G0–G7 PASS、**G8／G9 未到**（P4 达成仍是真机前置的关键未达项）；④ **增量待施工**：§0.7 标"待施工"的 24 项 T-R 系列＋v5 遗留的 Z3／Z4／Z5／Z6／Z12／Z14。S9 **不作前置**（R02）；Q 表现值＝Q9／Q12 关闭（翻案在册）、Q13 俯仰精测**待现场读数**、其余待确认项照 §15 逐条追。


> **收编生效记录（v4.5／v4.6，2026-10-12 主理人明文「按此落册」）**：v4.3（X1–X31）＋v4.4（X1–X4）＋v4.5 落册候选稿（Part 1／4／5／6／7）＋v4.6（Y1–Y6）**已收编进本正文**；四份原稿转为历史件、原文不回改，状态见 `note/00_README.md` 生效行。本节其余各条为四份稿的落点重写，**完成态一律带实证（Y6）**。
