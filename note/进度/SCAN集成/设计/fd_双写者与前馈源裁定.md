更新时间: 2026-10-08 08:34

# A7 P3 硬前置 F-D 设计 payload · 仿真域 /Odometry 双写者 与 SCAN 前馈源（保真度）裁定

- 作者：小张（部署架构师）。性质：**只做设计，不施工**。本档涉及的三个待办（F-D、J3、坑25 修法）实现状态一律 **未施工**；本档不产生任何 ws/ 与 scripts/ 改动，不产生任何 git 提交。
- 派单来源：note/进度/SCAN集成/进度文档.md:488-493（主理人裁决 R6）＋ note/04_配置改动diff.md:299（R1–R6 答复批）＋ note/进度/SCAN集成/设计/o1_到达信号与联锁设计.md:47（事实 D，F-D 出处）＋ note/进度/00_总览.md:121 ＋ note/进度/M2/进度文档.md:587/606 与 note/02_问题与坑.md:214（坑25 修法与 J3、F-D 同排 P3 起点）。
- 取证纪律：本轮全只读。**未启动任何 ROS 节点、未跑任何 roslaunch/仿真、未占任何端口**；生产 11312 零接触，隔离域 11313/11345 未触碰（现况为空，主理人随时可复跑）。所有行号均为本轮实开文件核对所得。正文零 markdown 反引号（02 坑18，note/02_问题与坑.md:151-160），路径与标识符一律裸词或「」。

---

## 0. 结论速览（六句）

1. **推荐修法**＝候选 A 的最小面形态：只改 urdf/lite3_planar.urdf.xacro:140 一行，把自研驱动插件的里程计话题改名为「/sim/odom_gt」；**odometryFrame=odom（:141）与 robotBaseFrame=base_link（:142）一律不动**，插件继续发 odom→base_link 的真值 TF。仿真域 /Odometry 自此唯一写者＝faster_lio（与真机同名同契约）。零重编译（xacro 是 launch 期现读件）。
2. **P3 保真度口径我裁向「SCAN 的前馈源必须是 LIO 估计」**：P3 仿真域必须起 faster_lio 并令其为 /Odometry 唯一写者；Gazebo 世界位真值只作对照臂（/sim/odom_gt ＋ 真值 TF），**不得进入任何「算法可用」的证据链**。
3. **不选候选 B（remap faster_lio 的 /Odometry）**的理由：它把一个在册的真机契约名（计划书v4.1:282 的 R15 对账行、P3b 现场作业单:82/92/93/98/99、P6/进度文档.md:19 的 G7-④）在仿真域换掉，sim 与真机的同一判据裂成两套话题名——这正是主理人 R2 对 M6 明确拒绝的形态；若走源码改名还额外破 00_README.md:21-22 与 04_配置改动diff.md:2 限定的唯一许可第三方改动面（laser_mapping.cc 只许 :751/:862 两行存盘路径）并触发重编译链。
4. **不选候选 C（lio_relay 显式指定 odom_in）作为主修**的理由：它不消除双写者，只把冲突推迟为「谁订阅到的是谁」；而 stageA 的历史证据已经证明那个「谁」在跑与跑之间是漂移的（§1.3 一）。C 的价值是**加固件**：把 relay 的输入源从「缺省恰好对」变成「写死的绑定＋发布者身份断言」，与 A 同批做。
5. **排程我裁两批**：批1 ＝ F-D（xacro 一行）＋ C 的显式化（壳包 70_scan.launch:28 加 arg）＋ J3（60_safety 的 cmd_vel_mux.yaml 表尾一行），三者改动面互不重叠、判据通道不同、零重编译 ⇒ 只重取一次 5 连；批2 ＝ 坑25（lite3_planar_drive.cpp:112/:127 打戳统一到 SimTime ＋ .so 重编译），**必须单独一批**，因为同批会把「重编译证据链」与「现读件」两条归因轴拧成一股，破「一次只动一个病因变量」。我接受多跑一次 5 连的成本，换归因纯度（理由见 §4）。
6. **是否触及真机契约：否**。/Odometry 在真机侧继续是 faster_lio 的输出名，本档对它零改动；改名只发生在 95_simulation（仿真域自有件）与 lite3_scan 壳包。真机相关一律写为「设计预留、待主理人裁定」（§5 末、§8），**不写施工步骤**。

---

## 1. 盘上独立复核（行号以本轮实见为准）

### 1.1 派单事实复核为真

| # | 派单事实 | 盘上实证（文件:行） | 结论 |
|---|---|---|---|
| 1 | faster_lio 以绝对名发 /Odometry | ws/src/10_slam/faster_lio/src/laser_mapping.cc:265（同段 :262-264 为三条 cloud、:266 为 /path）；句柄为全局 NodeHandle（app/run_mapping_online.cc:24-28），发布调用点 :343-345（online 分支无条件发），消息头 frame_id/child 硬编码 camera_init/body（:684-685），戳取 lidar_end_time_（:686）；TF 广播 camera_init→body 在 :700-710，帧名走参数（:67-68，值源 lite3_slam/config/faster_lio_outdoor.yaml:47-48） | 真 |
| 2 | 插件按 xacro 的 SDF 键发 /Odometry | ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140（odometryTopic=/Odometry；:141 odometryFrame=odom、:142 robotBaseFrame=base_link、:143 driveMode=dynamic、:144 cmdTimeout=10.0、:145 odometryRate=50）；插件 ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp:28 读键、:36 advertise(odom_topic_,10)、:109-123 取 WorldPose 组消息并 publish、:124-128 同批发 odom→base_link TF | 真 |
| 3 | lio_relay 把同一 odom_in 双路 relay | ws/src/30_3rd/plan_manage/launch/lio_relay.launch:24（odom_in 缺省 /Odometry）、:31-33（→/LIO/odom_vehicle）、:35-38（→/LIO/odom_imu）、:42-43（cloud）、:55-56（静态 world→camera_init）、:57-58（静态 body→base_link）；SCAN 侧消费：ws/src/40_navigation/lite3_scan/launch/70_scan.launch:24（全局 /body_pose_topic=/LIO/odom_vehicle）、:39-40（arg 注入）、:70-75（观测件 pose_topic=/LIO/odom_vehicle）；closed_loop_controller 与 FSM 同源（o1 设计件:67 已核） | 真 |
| 4 | 双写会把乱粥原样灌进 SCAN 位姿源 | relay 是 topic_tools 纯转发（lio_relay.launch:13-16 自述），对上游消息来源数不设防；/Odometry 上两条流 frame_id 不同（camera_init/body 与 odom/base_link），relay 不做过滤 ⇒ 交织原样下行 | 真（机理成立） |

### 1.2 对派单口径的三条更正（不采「上游记忆」，逐条回原件）

- **更正一（最重要）：「M2 的 stageA/stageB 不起 faster_lio」只对了一半——stageA 与 stageC 起，只有 stageB 不起。** 实证：ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh:78 起 m2_gazebo.launch、**:83 同窗起 m2_mapping.launch**（该件 :11 = pkg faster_lio / type run_mapping_online / name faster_lio_online，:7 装载的是生产 yaml lite3_slam/config/faster_lio_outdoor.yaml），stageC 同型在 :402 与 :405；而 stageB 的入口 :177 = lite3_bringup/mode_sim_teb.launch（:15→m2_teb.launch:13→m2_gazebo.launch），全链无 faster_lio 节点。⇒ **F-D 不是「P3 才会发生」的未来风险：在已验收的 M2-1/M2-4（stageA）与 M2-3 建图段（stageC）里，/Odometry 一直是双写状态。** 判据本体就是那个混合流上的读数（m2_smoke.sh:109 订阅、:125 断言、:126 起点取自它、:144 drift 取自它、:161 门 ≤0.5）。
- **更正二（行号）：台账与设计件引「xacro:141 odometryTopic=/Odometry」，盘上 odometryTopic 在 :140，:141 是 odometryFrame。** 见 note/进度/SCAN集成/进度文档.md:489、note/04_配置改动diff.md:299、o1 设计件:47（同引 :141）。按「历史记录不回改」原则，本档只作向前指向的更正声明，更正条目的登记权在主理人/小王（我不动既有 note）。
- **更正三（机理精度）：「P1/P2 因静止未暴露」结论成立，但原因不是「P1/P2 没双写」。** SCAN P1/P2 的起链本身就同时起 gazebo 与 faster_lio：ws/src/40_navigation/lite3_scan/verify/v_scan_p1.sh:4（链注释）、:79、:88、:90（wait_topic /Odometry）、:96；v_scan_p2.sh:65、:72、:74、:78。⇒ 未暴露的真实原因是**静止窗内两源近同值**（前置静止门 |Δp|<0.05m/30s，见 o1 设计件:47 与派单）。这条直接决定 §6 的判据必须写成「**源身份可证**」，而不是「数值看起来合理」。

### 1.3 三组决定性盘上实证

一、**混源签名已存在于历史证据里**（同一跑内 drift 与 gt_drift 互不相等 vs 逐位相等，两种形态都在）：

| 跑 | 证据文件 | drift | gt_drift | 形态 |
|---|---|---|---|---|
| stageA_M2R2 run2 | note/进度/M2/证据/log/stageA_M2R2/run2_stageA.txt:5 | 0.475 | 0.483 | 不等 |
| stageA_M2R2 run1/run3 | 同目录 run1:5 / run3:5 | 0.489/0.489 | 0.489/0.489 | 逐位相等 |
| M2R3 stageA | note/进度/SCAN集成/证据/log/raw/m2r3_stageA.log:7 | 0.468 | 0.485 | 不等 |
| M2R5 stageA | note/进度/SCAN集成/证据/log/raw/m2r5_stageA.log:7 | 0.490 | 0.490 | 逐位相等 |
| R1run1（IMU 键名缺失那次） | note/进度/M2/证据/log/stageA/stageA_R1run1_105540_INVALID-RTF0.278_IMU1Hz-updateRate键名缺失.txt:5 | 0.229 | 0.250 | 不等 |

⇒ harness 的 S['odom'] 是「最后到达的那条」，源身份随跑漂移（插件 50Hz 对 LIO 约 10Hz ⇒ 概率上真值常胜，但非必然）。另有一条同族旧案已在验收里被主理人补强：GazeboM1/进度文档.md:83 明记「**/Odometry 实为 planar_move 输出，非 faster_lio 自身**」，当时靠 traj.txt CSV 旁证才把 LIO 精度补齐。

二、**「真值自证」的判据空转已被抓过一次，且被小美追认降格**：

- stageB 五连的 dev_odom 与 dev_gt **逐位相同**：note/进度/SCAN集成/证据/log/raw/m2r5_fresh1n.log:19（0.172/0.172）、fresh2n.log:23（0.166/0.166）、fresh3n.log:27（0.176/0.176）、fresh4n.log:19（0.170/0.170）、fresh5n.log:23（0.173/0.173）。机读点：m2_smoke.sh:193 取 /Odometry、:379 算 dev_odom、:380 算 dev_gt——两者在 stageB 里同源（WorldPose）。⇒ M2-2 的 0.35m 门今天测的是「teb 把真值位姿开到目标没有」，**结构上不含任何估计层误差**。
- 同一形态的 evo APE 已被降格：note/进度/仿真测试台/进度文档.md:167 记「planar_move 里程计 vs WorldPose 毫米级」，小美在 :211 注记① 明判「**本次=planar_move odom vs WorldPose 的工具链演示，非定位算法精度数据**」。⇒ F-D 的要害不是话题名打架，而是**判据可能测不到它声称测的东西**；这条先例比任何论证都硬。

三、**契约在册面**（谁拥有 /Odometry 这个名字）：

- 设计侧把它写成 faster_lio 的输出：note/进度/仿真测试台/设计稿/A_SCAN松耦合集成设计.md:9（入向话题表）、:41（链路图 faster_lio → /Odometry）、:63（P3 链路「Gazebo 仿真雷达 → faster_lio → SCAN」）；note/进度/仿真测试台/设计稿/C_全局任务优先级总排序.md:11（A7 行前置明写「faster_lio 仿真链（已有）」）。
- 计划书把它列为真机对账名：note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md:282（R15：上游真实发布名 /Odometry ← laser_mapping.cc）；note/计划书v4/21_ros1-slam隔离部署计划书-v4.md:829（M1 判据 rostopic hz /Odometry ≥10Hz）、:889（Q4「/Odometry 话题名与频率（二期 SCAN 输入）」状态=开）。
- 另有第三写者潜伏在别的仿真 mode：ws/src/70_bringup/lite3_bringup/scripts/sim_pose_tf.py:30（~odom_topic 缺省 /Odometry）、:39 publish，其头注 :11 自述「话题名**沿用 faster_lio 输出名**，估计层按坑14 排除后该名无冲突」；装载点 ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch:107。它与插件不同 mode（mode_nav_sim 走 95_sim 渲染链、无 Gazebo，见该件 :8/:16/:75），但**「该名无冲突」是以「估计层不在场」为前提的**，P3 把估计层拉回来，该前提即失效 ⇒ §6 的 FD-4 就是钉这一条。

### 1.4 两条连带发现（同族、必须登记）

- **FD-G：lio_relay 的静态 body→base_link 会在 P3 窗里再造 base_link 双父（M2R4 复核节 ③ 的同型缺陷）。** 机理：lio_relay.launch:57-58 无条件发静态 body→base_link；插件 lite3_planar_drive.cpp:127-128 动态发 odom→base_link；两者同窗（v_scan_p1.sh:79/88/96 的形态）⇒ base_link 在 tf2 图有两条入边。这正是小美 M2R4 判定成立的前置③（note/进度/M2/进度文档.md:368、04_配置改动diff.md:289）与 U1 修正要根除的形态。P1/P2 之所以没炸：SCAN 不消费 TF（lio_relay.launch:52-53 自述「规划器不消费 TF，无影响」），窗内也无 mbf。**该件的约束**：lio_relay.launch 属 30_3rd 基线件（note/进度/SCAN集成/证据/scan_src_baseline_49.sha256:29，sha 1d0aa3f4…），且完整性判据是两条件（内容全等＋find 计数=49，v_scan_o1.sh:50-53、04:298）⇒ 改它=失基线事件，须主理人裁＋重发基线＋03/04 登记。处置选项与代价见 §8 R-FD-3。
- **FD-H：hdl 的输出话题名与 harness 期望名不一致（名字契约漂移，同属 F-D 家族）。** hdl nodelet 实发 /odom（ws/src/20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp:68），而 ws/src/95_simulation/lite3_gazebo_sim/launch/m2_hdl.launch:6 注「输出 /hdl_odom」、m2_smoke.sh:450/464/485 的 stageC 门控等的就是 /hdl_odom，承载件 ws/src/20_localization/lite3_localization/launch/hdl_localization.launch:75-111 内无该 remap。⇒ 即使 hdl 健康，gateC 的 hdl_odom 键也恒 False。与本切片判据无涉（stageC 已由 M2-R2C 裁决终止、转真机 G8），但它是 F-D 的同一条教训的第二实例：**名字不能靠注释与记忆，必须对账上游发布点**（R15 口径）。列为 G8 前置核对项，本档不动。

---

## 2. 候选修法（四条，各给落点／影响半径／风险／可机判判据）

### 候选 A（推荐）· 插件让位：改 xacro 的 odometryTopic 名

- **落点（唯一实质行）**：urdf/lite3_planar.urdf.xacro:140 —— /Odometry → /sim/odom_gt。头注 :5 与 :132-135 的自述行随之更新（注释行，不属被测件语义）。**明确不动**：:141 odometryFrame=odom、:142 robotBaseFrame=base_link、:143 driveMode、:144 cmdTimeout、:145 odometryRate；插件源码 lite3_planar_drive.cpp 零改动、.so 零重编译（xacro 系 launch 期现读件，先例 04_配置改动diff.md:289「零重编译」）。
- **为什么改话题名而不改帧名**（本候选的关键取舍）：odom→base_link 这条边是 M2R5/U1 结构修正的产物，被 E3 签名（m2_smoke.sh:333-355，期望 base_link_parents 恰 [odom]）、G5c1 阻断门（:324-326）、m2_tf_fresh.py 的 map→odom 段（m2_teb.launch:22-33）以及 costmap 的 robot_base_frame=base_link（config/costmap_common.yaml:5）共同锚定。帧名一改，这些刚回填的判据全部失去可比性；话题名一改，它们的读数通道（TF）一个字节都不变。
- **影响半径（仿真域读 /Odometry 的件，逐条实见）**：m2_smoke.sh:109/125/126/144/161（stageA A③）、:193/379（stageB dev_odom）、:415（stageC）；m1_smoke.sh:86/121/178（M1 台，同引该 xacro，见 m1_gazebo.launch:23）；v_scan_p1.sh:90、v_scan_p2.sh:74（wait_topic /Odometry）；p2_goal_plan.py:155-158（**注释与代码都假定读到的就是 camera_init 系的 LIO**）；rviz 面 faster_lio/rviz_cfg/loam_livox.rviz:220。生产域（10_slam/20_localization/40_navigation/60_safety/70_bringup 的 mode 件）零命中 ⇒ 不外溢。
- **风险**：
  1. **判据语义变更**（不是数值抖动）：A③「drift ≤0.5」自此由「混合流读数」变成「LIO 估计的真实回原点误差」。历史观察带 0.475–0.497（04_配置改动diff.md:306 按小美 R12 补全的带）距门 0.5 只剩 3mm 量级 ⇒ **重取有破门风险**。处置写死：任一跑破门即停手上报，门值调整权在小美/主理人，施工方不得自改阈值（note/进度/M2/进度文档.md:313「自降我设的判据——定权在验收而非施工/主理人」的教训已在案）。
  2. stageB 若 P3 也把 faster_lio 拉进窗（保真度口径要求），dev_odom 将从「真值自证」变为「估计＋控制复合误差」，0.35m 门也可能破——同上一条处置。
  3. 「顺手把插件 odom 关掉」是禁手：关掉发布同时会断掉 odom→base_link 的 TF（同一函数体 lite3_planar_drive.cpp:104-128），mbf/costmap 位姿源即失效（见候选 D1）。
- **Gate 重取清单（这是被测件变更，必须写清；口径依据＝小美 M2R4 复核节 L376「被测件即变 ⇒ 5 连必须重新起算，旧跑不得与新配置混计」＋我自己 m2r5 设计件 §7 L210-214 的回归面）**：
  - 必重取①：**stageB fresh 臂 5 连全绿**（U4 口径：G5/G5b/G5c1/G5c2/press_late_G5/G6 全 PASS，其中 ≥1 次加压档——取龄点 sim ≥60s 或同跑两取龄点 sim 差 ≥40s；每跑 foxy 六 PID 起终成对快照、四口 TCP/UDP 双查、退净残留:[]、样张 sha256 未变、5 连 TF_OLD_DATA 命中 0 次）。
  - 必重取②：**stageA 复跑 ≥1 次**（A0/A①/A②/A③/A④ 全绿），且必须登记「A③ 的读数源身份已变（混合流 → LIO 单源），改前改后数值不得混计，改前带 0.475–0.497 降为历史观察值、不作阈值」。
  - 必重取③：**SCAN 侧 P1/P2 各重跑 1 次**——判据本体不变，但 v_scan_p1.sh:90、v_scan_p2.sh:74、p2_goal_plan.py:157 读到的东西变了 ⇒ 既有 6/6 与 7/7 的证据**不可复用**（静止窗内两源近同值，预期风险低，但仍须重取一次并留 FD-1 的身份断言）。
  - 必重取④：结构等价对照 1 次（V6 形态，随首轮留证不另起窗）——因为动的是同一个 <plugin> 块；依据 02 坑21（Gazebo fixed 树合并、body 名=root link 名）与 04:290 的 V6 实测纪律：body_names/geom_names/joint_names/落地 z 与姿态 改前改后对照。本改动只换 SDF 内的一个字符串键，不碰 link/joint/inertial ⇒ 预期全等；若 body 面出现任何增减即停手（该判据的字面与成员名跟随 root 的例外口径按小美 R13 的更正版读）。
  - 必重取⑤：共同门禁 scripts/verify/v3_isolation.sh 全量 [1]-[9] 复跑，要求净新增命中 0 且可逐条归因（跑前必须 source ws/devel/setup.bash，教训在案）。预判：[5] 零命中（:15-24 只查跨 lite3_* 的 launch include，95_simulation 不在 lite3_* 命名内）、[6] 零命中（:25-28 只扫 lite3_*/launch/ 的 cmd_vel 字面，且不动 launch）、[8] 零命中（新话题名不含 /home/ysc）。
  - 不重取／不适用：stageC（hdl 仿真已被 M2-R2C 裁决终止转真机 G8，00_总览.md:106）；静态臂阴性对照 2 次（m2r5 设计件 V7，依小美 L401-404 不重跑、不计入 5 连）；M1 台复跑（已结案，且其判据③ 的 26.411 FAIL 与修复史都是历史事实，只作登记不作复跑，见 GazeboM1/进度文档.md:83/:92）。**若走 A：.so 不重编译 ⇒ 无需单包重链批与全量清白批**（同 m2r5 设计件 L214 口径）；若有人改为「动插件源码」，二者皆需，回归面再加一层。
  - 登记义务：04 台账一条（旧值/新值/理由/日期）＋ 首次编辑**之前**从 git 对象导出改前基线件（05_复验作业单 B2 与 R6 纪律）＋ before 留底 sha256。
- **可机判判据**：表 FD 的 FD-0、FD-1、FD-3、FD-4、FD-5、FD-7（§6）。

### 候选 B（判不推荐）· 10_slam 壳包侧 remap faster_lio 的 /Odometry，或 remap 消费侧

- **落点（若要成立需在两处）**：ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:11 的 node 内、以及 ws/src/10_slam/lite3_slam/launch/10_slam_mapping_sim.launch:10 的 node 内加 remap。**不可**改 laser_mapping.cc:265：00_README.md:21-22 与 04_配置改动diff.md:2 把该文件的可改面**限定**为 :751 与 :862 两行存盘路径（V2 隔离豁免），改名越界；且它是源码件 ⇒ 触发重编译与 devel 产物 sha 变化。另注：对全局句柄 advertise 的**绝对名**做 remap，其生效性我在盘上找不到本仓先例，属**未核实**（§7 一），选它就要先花一次实跑验生效性——这本身就是一笔施工成本。
- **下游按绝对名 /Odometry 消费的在册清单**（全仓 grep 实证，含 harness 与 note）：lio_relay.launch:24（sim 与真机同件）、m2_smoke.sh:109/193/415、m1_smoke.sh:86/121、v_scan_p1.sh:90、v_scan_p2.sh:74、p2_goal_plan.py:157、loam_livox.rviz:220、sim_pose_tf.py:30（mode_nav_sim 域的同名单写者，头注 :11 明写该名是从 faster_lio 借的）、mapping_viz.launch:4（真机采图 Foxglove 跟随）、P3b/现场作业单.md:82/92/93/98/99（真机采图观察窗 rostopic hz /Odometry 与回原点判据）、P6/进度文档.md:19（G7-④ /Odometry ≥10Hz）、00_总览.md:86（P7 活体 Odometry 10.036Hz）、计划书v4:829、计划书v4.1:282（R15 对账行）。hdl 侧不消费 /Odometry（它自产 /odom，apps/hdl_localization_nodelet.cpp:68；且 hdl_localization.launch:89 enable_robot_odometry_prediction=false）；mbf/costmap 侧盘上无 odom 话题键（costmap_common.yaml:4-5 只有 global_frame=map 与 robot_base_frame=base_link）⇒ 它吃 TF 不吃该话题（活体面未核，§7 五）。
- **会不会打断已验收链路**：会，且打断方式最坏——真机侧的 /Odometry 是**人工与判据共用的锚**（G7-④ 频率门、P3b 采图窗、R15 对账行）。sim 改名而真机不改 ⇒ 同一判据两套话题名（主理人 R2 明拒的形态）；sim 与真机一起改 ⇒ 真机契约变更，R15 对账行要重发、G7-④ 判据文本要改、采图作业单要改（三处都是已结案/在册件）。两条都不比候选 A 便宜。
- **风险**：新增一条「只为 B 自养」的对账判据（见下），并把源身份问题从「写者唯一」降级为「读者自己知道读的是谁」；另外 10_slam_mapping_sim.launch 与 m2_mapping.launch 是两处，任一处漏改即复发。
- **若仍选 B 的机判判据**：FD-1 反向版（/Odometry 的 frame 集合恰 {odom} ⇒ 判 FAIL）＋ 一条新增「sim/真机起链话题名逐字对账」静态断言。我的评估：这条断言之所以需要存在，正是因为 B 制造了双口径——**为修法自带一条看护判据，是不该付的钱**。

### 候选 C（必要但不充分，作 A 的加固件）· lio_relay 的 odom_in 显式指定

- **落点**：ws/src/40_navigation/lite3_scan/launch/70_scan.launch:28（现为裸 include）改为 include 内带 arg：odom_in 显式写死（值＝/Odometry，即 LIO）。**不动** 30_3rd 基线件 lio_relay.launch:24（保持该件 sha 1d0aa3f4 与 49 件计数）。
- **为什么它不是主修**：两条 relay（:32-33 与 :37-38）取的是**同一个** odom_in，把输入源指定给谁都不消除 /Odometry 上的双写；若 odom_in 仍=/Odometry 而双写仍在，SCAN 位姿源依旧交织——正如派单所点明的，它只是把冲突推迟为「谁订阅到的是谁」，而 §1.3 一 证明那个「谁」逐跑漂移。若 odom_in 指 /sim/odom_gt，则等于选了 §3 的反面口径（SCAN 吃真值），必须在进度文档显式声明保真度边界。
- **它的真价值（与 A 同批做的理由）**：把「缺省值恰好是对的」升级为「写死的绑定＋可机判的发布者身份」，并在壳包里给 P3 编排层一个显式口子（未来若要 TOP 层按档位选源，改这一个 arg 即可，不动第三方件）。
- **机判**：静态 grep 70_scan.launch 的 odom_in 显式行恰 1 命中；活体 FD-0＋FD-3（relay 出口的消息头帧名＝LIO 帧名）。

### 候选 D（其它形态，各判）

- **D1 停发插件 odom 或连 TF 一起停**：判**禁止**。odom→base_link 的 TF 与话题在同一函数体内（lite3_planar_drive.cpp:104-128），停发即断 mbf/costmap 的位姿来源（costmap_common.yaml:4-5 走 TF）与 G5c1/E3 的观测对象（m2_smoke.sh:324/:333-355），把一条话题改名问题升级成 TF 拓扑事故。
- **D2 双写保留、由消费者按 header.frame_id 过滤选源**：判**不可用作主修**。它把「源选择」下放到每一个读者，而 SCAN 侧读者全是第三方基线件（lio_relay、closed_loop_controller、scan_replan_fsm）不可改 ⇒ 只能给我方件加过滤，双写对新读者仍然生效。**可保留为过渡观测件**：在 F-D 落地前后各取一次 frame 分布，作为「混源确实存在」的阳性对照（FD-2）。
- **D3 反面保真度口径（sim 用真值当里程计）的形态**：落点＝P3 起链去掉 faster_lio（即不跑 m2_mapping.launch）＋在 SCAN集成/进度文档.md 与 00_总览.md 的边界注记里把 P3 到达判据改称「跟踪/规划收敛判据（不含估计层）」。只有主理人可选它（§8 R-FD-2），代价见 §3 末段。

### 比较表（一张看清）

| 候选 | 实质动作 | 是否被测件 | 是否重编译 | 是否消除双写 | 是否动第三方基线 | 是否外溢真机契约 | 判据语义影响 | 裁定 |
|---|---|---|---|---|---|---|---|---|
| A | 插件 odom 话题改名（xacro:140 单行） | 是（坑21 家族） | 否 | **是** | 否 | 否 | A③/dev 读数源变干净（重取即可） | **推荐** |
| B | faster_lio 话题改名/重映射 | 视路径（launch 层=否；源码层=是＋越 00_README 豁免界） | 源码路径=是 | 是（但换个名字仍双写与否要看改哪侧） | 否（launch 层） | **是**（/Odometry 是在册真机锚） | 双口径，需自养对账判据 | 不推荐 |
| C | relay 输入源显式化 | 否（壳包自有件） | 否 | **否**（只推迟冲突） | 否（走 include arg，不改基线件） | 否 | 无 | 与 A 同批做加固 |
| D1 | 停插件 odom/TF | 是 | 是（改源码）或否（改 xacro 关掉） | 名义上是 | 否 | 否 | **打断 TF 与 M4** | 禁止 |
| D2 | 消费侧按帧过滤 | 否 | 否 | **否** | 是（若改 SCAN 件）／否（只改我方件则无效） | 否 | 掩盖而非修 | 只作过渡观测 |

---

## 3. P3 仿真保真度口径（裁定，不留给施工方）

**裁定：P3 的 SCAN 前馈位姿源必须是 faster_lio 的 LIO 估计；仿真域必须起 faster_lio 且它是 /Odometry 的唯一写者；Gazebo 世界位真值改走 /sim/odom_gt ＋ 真值 TF，只作对照臂，不得进入「算法可用」的证据链。**

依据（四条，均可回原件）：

1. 设计本意就是 LIO 在环：A_SCAN松耦合集成设计.md:9（入向契约表写 faster_lio 与 frame camera_init/body）、:41（链路图 faster_lio → /Odometry）、:63（P3 链路「Gazebo 仿真雷达 → faster_lio → SCAN → T1 跟踪 → planar_move」）、:64（执行序 P1/P2/P3/P4）；C_全局任务优先级总排序.md:11（A7 的前置列「faster_lio 仿真链（已有）」）。
2. 真值当里程计的判据空转已被抓过两次：stageB 五连 dev_odom≡dev_gt（§1.3 二）＋ evo APE 被小美降格为「工具链演示、非定位算法精度数据」（仿真测试台/进度文档.md:167/:211）＋ M1 时代主理人补强 (a)「/Odometry 实为 planar_move 输出」（GazeboM1/进度文档.md:83）。
3. 真机没有真值里程计可用：真机链上只有 faster_lio 与 hdl（apps/hdl_localization_nodelet.cpp:68 发 /odom），G8/G9 的 SCAN 前馈必然来自 LIO 类估计（计划书v4.1:282 R15、计划书v4:829）。仿真用真值过关、真机用估计上场 ⇒ P3 的绿不是 G8 的可迁移证据，正是主理人在派单里点名的「把仿真通过当算法可用」。
4. sim/真机同契约的可维护性：让仿真域唯一的「非生产件」（95_simulation 插件）让位，生产件（10_slam）零改动，比反过来便宜且不产生双口径。

**代价（写清，不隐藏）**：

- (i) 一次被测件变更批的 Gate 重取（§2 候选 A 的清单一到五）。
- (ii) A③（drift ≤0.5）与 stageB（dev ≤0.35）两道门可能因「口径变干净」而破门：改前 A③ 的历史带 0.475–0.497 已贴门；改后读数含 LIO 自身误差，方向不可预判（stageB 未跑，值不可知，§7 六）。处置写死：破门即停手上报，门值调整权在小美/主理人；施工方与我均不得以「口径变了」为由自降阈值——新带另立、旧带标注为「混合源时代」。
- (iii) M2 域需要一条**显式边界登记**（不是施工、是口径）：stageB 的 M2-2 判据在 F-D 之后若仍不起 faster_lio，其 dev_odom 仍是真值自证 ⇒ 必须登记「M2-2 测的是控制/规划收敛，不含估计层精度；LIO 精度证据归 stageA 与专门的 APE 件」。这条登记的落笔权在小美/主理人，我在 §8 R-FD-1 提请。
- (iv) 多一条对照话题（/sim/odom_gt）的带宽与维护成本，以及 harness 里要新增「对照臂不参与 PASS/FAIL」的纪律（防它被误当判据读数）。

**反面口径（接受 sim 用真值当里程计）的代价**：P3 判据③（到达偏差 ≤0.35m，A_SCAN松耦合集成设计.md:65）结构性测不到 LIO 误差；A7 的绿在 G8 面前必须降级为「规划与跟踪在完美位姿下可走通」；且 P3 起链要**去掉** faster_lio（与 A7 设计稿 :63 与 C 文 :11 的链路图冲突，需同时改设计稿）。若主理人取此边，则我要求把这条边界以原话写进 SCAN集成/进度文档.md 的 P3 节与 00_总览.md 的边界注记，禁止裸引「P3 PASS」。

---

## 4. 排程：F-D × J3 × 坑25 的批次裁定

三件事的改动面（均为本轮实见的行号）：

| 事 | 改动落点 | 被测件？ | 需重编译？ | 影响的判据通道 |
|---|---|---|---|---|
| F-D（候选 A＋C） | lite3_planar.urdf.xacro:140 单行；lite3_scan/launch/70_scan.launch:28 加 arg | **是**（xacro 属 URDF/SDF 装配参数，坑21 家族＋M2R4 L376 口径） | 否（launch 期现读件） | /Odometry 的**消息身份**（A③、dev_odom、SCAN 前馈） |
| J3（主理人已裁：方向认可、推迟到 P3 起点施工） | 60_safety/lite3_safety/config/cmd_vel_mux.yaml:11-16 表尾加一行 scan_planner 源 ＋ 三处「六源」文案（cmd_vel_mux.yaml:3/:7、safe_cmd_vel_mux.py:6/:10/:31）；零 launch 改动（o1 设计件 §4.2 β） | 否（非物理被测件；属 M6 安全件配置，条文级口径由主理人 R1 已认可） | 否 | /cmd_vel 的**写者归属**（C-1..C-4） |
| 坑25（02:206-215；修法未擅做，待批） | lite3_planar_drive.cpp:112 与 :127 打戳由 ros::Time::now() 改为 :61 的 SimTime | **是**（插件源码＝被测件） | **是**（须重编译 liblite3_planar_drive.so） | TF 与 odom 的**时间基**（G5/G5b/G5c1/G5c2、E3、TF_REPEATED_DATA 出现率） |

**裁定：两批。**

- **批1（P3 起点，一次重取）**＝F-D（A＋C）＋ J3。理由：三者改动行号集合互不重叠、分属「消息身份／写者归属」两条不同判据通道、都零重编译（⇒ 一批里 .so 与 devel 产物的 sha 必须起终不变，这条本身就是归因看护）；且主理人 R2 已裁「sim 域 SCAN 闭环必须带 M6」，P3 的起链拓扑本来就要重排一次（m2_teb 现行 relay 拓扑要换，o1 设计件 C-5 已给出 TOP 编排件建议 lite3_bringup/mode_sim_scan.launch）——F-D 的断言与 J3 的 C-1..C-8 在同一次拓扑重排上一起跑，省一次起链而不损归因。**J3 不增加 5 连重取义务**（它不是被测件变更），也不得被写成 5 连破门的可疑因（看护手段＝FD-7 的 diff 行数与 sha 断言）。
- **批2（批1 全绿后再排）**＝坑25。它必须单独一批，理由见下条判据②与③。

**同批许可判据（三条全满足才可同批）**：

1. 改动面行号集合互不重叠，且落在**不同判据通道**（一者动话题身份、一者动速度归属）；
2. 批内**全部是现读件**（xacro/launch/yaml），零重编译 ⇒ 可用一条 sha256 断言钉住构建态（.so、ws/devel 产物起终不变）；
3. 任一件的判据不消费另一件改过的东西（F-D 的判据读 header 帧名与发布者数，J3 的判据读 /cmd_vel 的 Publishers 集合）。

**必须分批判据（任一命中即分批）**：

1. 批内同时含现读件与**重编译件** ⇒ 「构建态证据」与「配置态证据」两条归因轴同批双变，一条 sha 断言钉不住，破「一次只动一个病因变量」（小美既有要求，见 02 坑21/坑25 与 M2R4/M2R5 的批次史）；
2. 后动者的判据读数依赖先动者的口径 ⇒ 必须**先动口径者**。本例正是这一条：坑25 的观测对象是 TF 新鲜度与「同戳重复」的出现率（M2R5-O2 挂账：旧七跑 0 命中、新五连 3/5 命中、M2R6 三跑 4/2/0 行，02:213 明记出现率差异**未核实**），而 F-D 会把 stageA/stageB 的消息序列与采样窗整体换源。若同批，则「TF_REPEATED_DATA 消失」无法归因——它可能因 SimTime 修法而消，也可能因换源后 harness 的取数时序而变，O-2 的再开条件（任一次 Δ_age >1.0s 或 pose 类 ERROR 命中）就此失去可判性。
3. 任一件需要保留**独立回退口**（阴性对照）⇒ 不得同批。坑25 的对照形态是「打戳改回 now 即复现同戳」，F-D 的对照形态是 D2 的「改前 frame 分布阳性对照」；两批混一起做对照就得连坐回退。

**重取成本 vs 归因纯度——我的取舍**：接受批2 重复一次 5 连的成本（约两次批窗）。理由：批1 改的是「判据测的是谁」这一口径级事实，若把机理级修法混进来，一旦 5 连红就得整批回退（含把 xacro 一起翻回去），代价远高于多跑一次连击；并且批1 的绿会成为批2 的**新基线**（TF_REPEATED_DATA 的出现率要在单源前提下重新统计，这正好是 02:213 那条「未核实」唯一能被回答的方式）。

**批次先后顺序（写死）**：批1 内先落 F-D（xacro:140 ＋ 70_scan.launch:28），同窗落 J3（mux yaml），随后一次跑满 §2-A 重取清单①–⑤＋FD 表；批1 全绿并归档后，批2 落坑25（改 :112/:127 ＋ 单包重编译），再重取 stageB 5 连（**不需再动 SCAN 侧 P1/P2 判据**，因为坑25 不改话题身份），并把 M2R5-O2 挂账按新前提重算一次出现率（可判性由批1 分离出来才成立）。

---

## 5. 帧契约与生产域边界

| 名 | 生产/真机侧是谁的契约（实见出处） | 本设计动不动它 |
|---|---|---|
| 话题 /Odometry | faster_lio 的输出名：laser_mapping.cc:265；在册对账行 计划书v4.1:282（R15）；在册判据锚 计划书v4:829（M1 频率门）、P6/进度文档.md:19（G7-④ ≥10Hz）、00_总览.md:86（P7 活体 10.036Hz）、P3b/现场作业单.md:82/92/93/98/99（真机采图观察窗与回原点判据）、70_bringup/launch/mapping_viz.launch:4（Foxglove 跟随） | **不动**（真机侧继续由 faster_lio 持有；仿真侧让它成为唯一写者） |
| 帧 camera_init → body | faster_lio 的消息头硬编码（laser_mapping.cc:684-685），TF 帧名走参数（:67-68）＋生产 yaml lite3_slam/config/faster_lio_outdoor.yaml:47-48；lio_relay.launch:55-56 用恒等静态边把 world 接到 camera_init；:57-58 用恒等静态边把 body 接到 base_link（可视化连通） | 不动（FD-G 的静态边问题另案，见 §8 R-FD-3） |
| 帧 odom → base_link ＋ 话题（改后 /sim/odom_gt） | 自研驱动插件独有：lite3_planar_drive.cpp:29-30（缺省帧名）、:112-113（消息头）、:127-128（TF）；装配值 xacro:141-142。**仅存在于仿真域**（真机无此插件；mode_nav_sim 域的对应件是 sim_pose_tf.py，它发 map→base_link，:55-57） | 话题名动（:140 单行）；**帧名不动**（保护 U1/E3/G5c1/costmap 的锚） |
| mbf / costmap 的位姿来源 | config/costmap_common.yaml:4-5 只设 global_frame=map、robot_base_frame=base_link；盘上无 odom_frame_id、无 odometry_topic 键 ⇒ 依 TF 取位姿（活体面未核实，§7 五）。⇒ 插件话题改名对 M4 无外溢；若改帧名则有（故 A 明确不改帧） | 不动 |
| hdl 的输出 | apps/hdl_localization_nodelet.cpp:68 发 /odom（**不是 /Odometry**）；lite3_localization/launch/hdl_localization.launch:38/:80/:89-90 定 odom_child_frame_id=base_link、robot odom 预测关断 ⇒ 真机 localize 模式下 /Odometry 仍只有 faster_lio 一个写者（盘上未见第二发者） | 不动；FD-H 的 /odom 与 /hdl_odom 名义漂移列 G8 前置核对 |

**结论：仿真域改名不外溢成真机契约变更（否）。** 看护手段＝FD-5 的静态断言：批1 前后，10_slam / 20_localization / 40_navigation / 60_safety / 70_bringup 五模块内「/Odometry」字面命中集合逐字相同（允许变的位置只有 95_simulation 与 lite3_scan 壳包）。这条把「不外溢」从叙述变成可检项。

**真机侧一律只写设计预留、待主理人裁定（本档不写任何施工步骤）**：

- 预留1：真机 P3/G8 起链是否同样上 FD-0（/Odometry 的 Publishers 恰 1）与 FD-3（前馈源身份可证）两条断言。盘上结构证据显示真机侧 /Odometry 天然单写（仅 faster_lio），但**未经活体核**（§7 四/七）。
- 预留2：真机侧若将来需要「与仿真同名不同源」的对照位姿（例如 hdl 的 /odom），其命名要与 /sim/odom_gt 形成可判差异，避免把 sim 专用名带进真机链。
- 预留3：FD-H（hdl 输出 /odom 与 harness 期望 /hdl_odom 不符）作为 G8 真机定位联调的**前置核对项**，处置权与派单权在主理人，本档不动 20_localization。

---

## 6. 给小美的验收判据草案（表 FD，全部可机判）

建议落点：新验收件 scripts/verify/v_scan_fd.sh（静态臂零节点可跑，活体臂并入后续 v_scan_p3.sh 的 F-D 段）。命名与形态沿用在册先例：v_scan_o1.sh:19-20 的 pubs/subs 解析助手与 A-2「唯一发布者」写法、C-1 的「车动窗内采样 ≥3 次、间隔 ≥5s」采样纪律、o1_arrival_probe.py 的注入式无车用例（坑23 纪律：能在无车环境验的先无车验）。**本表实现状态：未施工。**

| 号 | 断言 | 取法（机判） | 抓什么 |
|---|---|---|---|
| FD-0 | **前置断言：/Odometry 的 Publishers 恰 1**（且 /sim/odom_gt 的 Publishers 恰 1） | 在起链就绪、车动窗中、车动窗末三处各取一次 rostopic info /Odometry 的 Publishers 段行数，三次都必须 =1；对 /sim/odom_gt 同。发布者的**节点名不参与判据**（插件在 rostopic info 里的节点名拼写未核实，§7 二），只判数与 FD-1 的帧名 | 双写者本体（F-D 的一句话判据）；「恰 1 但随时变 2」的时序漏检 |
| FD-1 | **源身份可由消息头证明**：/Odometry 的 header.frame_id 去重集合恰 {camera_init}、child_frame_id 恰 {body}；/sim/odom_gt 恰 {odom}/{base_link} | 各采样 N≥200 条（rosbag record -x 或 python 采样），打印 frame/child 的去重集合与每集合计数；期望等式不成立即 FAIL | relay 灌错源、B 候选式改名后的残留在者、以及「计数为 1 但那一路是错的那一路」 |
| FD-2 | 阳性对照（落地前后各一次）：F-D 落地**前**的同一窗内，/Odometry 的 frame 去重集合必须含 {camera_init, odom} 两个元素；落地后必须只剩 {camera_init} | 同 FD-1 采样法，跑两次（改前/改后），把两次的集合原样入证据 | 证明判据有分辨力（防「本来就没双写」的假绿）；对齐 G5c1「改前 7/7 HIT」的前后对照先例 |
| FD-3 | **SCAN 前馈源身份可证（本表最重要的一条）**：/LIO/odom_vehicle 必须与 /Odometry 同族、与 /sim/odom_gt 异源 | (i) /LIO/odom_vehicle 的 frame 集合恰 {camera_init}（FD-1 同法）；(ii) rostopic info /LIO/odom_vehicle 的 Publishers 恰 1；(iii) **异源证伪**：同窗对 /LIO/odom_vehicle 与 /sim/odom_gt 按最近邻时间戳配对（容差 0.1s）取 ≥10 对，若 |Δxy| 全对 < 1e-4 m ⇒ 判 FAIL「SCAN 读的是真值」；(iv) **阴性对照（拔掉一路）**：rosnode kill /faster_lio_online ⇒ /Odometry 与 /LIO/odom_vehicle 必须同时停发（rostopic hz 连续 3s 无数据）而 /sim/odom_gt 频率不变 ⇒ 证两路确系异源且 relay 直取 LIO；随后重启复跑 FD-0/FD-1。（/faster_lio_online 这一节点名拼带有在册先例：m2_smoke.sh:433 的 rosnode kill） | 「怎么证明 /LIO/odom_vehicle 里跑的确实是我要的那一路」——(iii) 抓真值冒充，(iv) 抓同源复制与 relay 指错 |
| FD-4 | 单写者守恒（跨域）：P3 的 SCAN 窗内**不得**同时存在 mode_nav_sim 域的 /Odometry 写者 | rosnode list 里 /sim_pose_tf 命中数必须 = 0（该件的 ~odom_topic 缺省就是 /Odometry，见 sim_pose_tf.py:30/:39 与其头注 :11「估计层排除后该名无冲突」的前提）；静态臂：P3 编排件的 include 树里不得同时出现 mode_nav_sim 与 m2_gazebo/m2_mapping | 第三写者把 FD-0 的「恰 1」在别的 mode 组合下悄悄打成 2；主理人 R2「由 TOP 模式选路保证不同时活跃」的前提落地为可检项 |
| FD-5 | 不外溢看护：生产五模块的 /Odometry 字面集合批前后逐字相同 | 静态：对 10_slam、20_localization、40_navigation、60_safety、70_bringup 各跑一次 grep -rn 字面 /Odometry，落件并按行排序，与批前基线 diff 必须为空；允许变的位置仅 95_simulation 与 lite3_scan | 「只改仿真」的承诺被顺手破；真机契约变更的早期信号 |
| FD-6 | TF 面未被 F-D 打穿（回归哨兵）＋ FD-G 的发作探测器 | 沿用 m2_smoke.sh:333-355 的 E3 直读：base_link_parents 恰为 ['odom']（单元素）；/tf_static 行集里**不得**出现任何指向 base_link 的边；若出现 {odom, body} 双元素 ⇒ 立即停手上报（这就是 lio_relay.launch:57-58 的静态 body→base_link 与插件动态 odom→base_link 同窗复发的签名，FD-G），**不得静默继续**；另按 U4 原条件要求 5 连内 TF_OLD_DATA 命中 0 次 | 话题改名把注意力全吸到消息面，漏掉 TF 面同源缺陷；M2R4 前置③ 同型缺陷在 P3 窗复发 |
| FD-7 | 被测件看护：本批只动 xacro 一行、零重编译 | ① before 留底件在**首次编辑之前**从 git 对象导出并记 sha（05_复验作业单 B2 与 R6 纪律）；② 改前后 xacro 的 unified diff 有效行数 =1（＋可选注释行），link/joint/inertial/sensor/plugin 其余块与 mu 块逐字节相同（V6 纪律，04:289）；③ liblite3_planar_drive.so 与 ws/devel 相关产物 sha256 起终一致（证零重编译、未偷带源码改动）；④ 30_3rd 两条件判据（sha256sum -c 49 OK 且 find 计数=49，v_scan_o1.sh:50-53）全绿 ⇒ 证 lio_relay.launch 未被改动 | 「顺手多改一行」与「以改名为名重编译被测件」；同批 J3/F-D 的归因钉（§4） |
| FD-8 | 判据语义变更登记（非机判但可核） | 04 台账一条：写明「自 F-D 批起，A③/stageB dev/SCAN 前馈的读数源身份 = LIO 单源；改前为混合源，历史带 0.475–0.497 与 dev 带 0.166–0.176 降为**混合源时代观察值**，不作阈值」；SCAN集成/进度文档.md 与 00_总览.md 的边界注记同步（§3 代价 iii 的登记，落笔权在小美/主理人） | 新数值与旧数值被混计（L376 明禁）；「口径变了所以放宽」的自降通道 |
| FD-9 | 破门即停（红线，写进验收件而非靠人记） | 重取清单①–③ 任一 FAIL ⇒ 退出码非 0 并打印「停手上报，门值调整权在小美/主理人」，不得重跑凑绿（三次失败红线在案）、不得改阈值再跑 | 把 §3 代价 (ii) 的风险变成硬闸 |

采样与证据纪律（全表通用）：活体断言不得只在起链瞬间取一次（沿用 C-1 的 ≥3 窗、间隔 ≥5s）；每断言打印原始输出行而非只打 PASS/FAIL；端口与退净按坑16 的 TCP/UDP 分查与 ps 判活（禁 kill -0），隔离域按仲裁位 11313/11345、生产 11312 零接触；roslaunch 自启 master、**禁显式 roscore**（坑24 的 I-0/I-1/I-2 三断言直接复用）。

---

## 7. 未核实清单（不推测填充）

1. 对全局句柄 advertise 的**绝对名**做 launch 级 remap 的生效性，在本仓无先例、本轮未活体核（候选 B 的前提）。
2. Gazebo 进程内插件在 rostopic info 里的发布者节点名拼写（未核实）⇒ FD-0/FD-3 刻意不依赖节点名，只依赖计数与消息头帧名。
3. faster_lio 在仿真域的实际 /Odometry 发布频率（名义 10Hz，间接观测量为 rate_pts 8.7–10.0）；未做逐跑发布者频率统计。
4. 双写从未被任何证据件直接记录过（全仓无一次 rostopic info /Odometry 的留档，实测 grep 命中的只有结构代码与本档引文）⇒ 本档的「双写」＝ launch/源码结构推定（§1.1/§1.2）＋ 历史 drift≠gt_drift 的间接签名（§1.3 一）。若要直接发布者表证据，须由施工方在一次性取证窗跑 FD-2 改前臂。
5. costmap/mbf 是否完全不吃 /Odometry 话题：config 面无相关键（costmap_common.yaml:4-5），活体面未核。
6. F-D 落地后 stageB 的 dev 与 stageA 的 A③ 新值：未跑，方向与量级均不可预判（§3 代价 ii）。
7. 真机侧 /Odometry 是否存在第二写者：盘上未见（hdl 发 /odom，qnx2ros/sensor_checker 不发里程计），但真机活体未核（§5 预留1）。
8. M2R5-O2（TF_REPEATED_DATA）出现率差异的原因：02:213 明记未核实；本档只把它作为「坑25 必须与 F-D 分批」的判据依据（§4 分批判据②），不对其成因作任何推断。

## 8. 需主理人裁定项（本档不擅自选边的事都收在这里）

- **R-FD-1（保真度口径追认）**：请追认 §3 的裁定（SCAN 前馈＝LIO 估计；真值仅作对照），并决定 §3 代价 (iii) 那条边界登记由谁落笔（stageB 的 M2-2 判据不含估计层精度，须写进进度文档与 00_总览 的边界注记）。
- **R-FD-2（反面口径的处置）**：若否决 §3，需同时批准「P3 起链去掉 faster_lio」并修订 A_SCAN松耦合集成设计.md:63 与 C 文 :11 的链路图/前置，以及把 A7 判据③ 改称「跟踪/规划收敛判据（不含估计层）」。
- **R-FD-3（FD-G 的处置路径）**：lio_relay 的静态 body→base_link 与插件 odom→base_link 同窗 ⇒ base_link 双父复发。三个选项：①P3 编排层不复用该 include、由壳包自持等价 relay（我方件，可控，但要多一条「与上游 relay 形态等价」的对账判据）；②改 30_3rd 基线件（失基线事件，须重发基线＋03/04 登记，代价最高）；③以拓扑隔离 TF 消费者为处置、只靠 FD-6 哨兵抓（最省，但把风险留在潜伏态）。我倾向①，且**无论选哪个，FD-6 的停手条都要先落地**。
- **R-FD-4（批次裁定追认）**：§4 的「两批、J3 搭 F-D、坑25 单独」与同批/分批判据；坑25 是否批准排入批2（02:214 与 M2/进度文档.md:606 都在等这句话）。
- **R-FD-5（破门预案）**：若批1 的 A③ 或 dev 因口径变干净而破门，门值/带值的调整由谁定、按什么口径定（我方的硬要求：施工侧与设计侧均不得自改阈值，只能停手上报）。
- **R-FD-6（行号更正条目的登记）**：§1.2 更正二（odometryTopic 在 :140 非 :141）与 FD-H（/odom 与 /hdl_odom）是否各立一条 04 向前指向更正，登记权在主理人/小王。

## 9. 本切片零改动自证（纪律回执）

只新建了本文件一件；ws/ 与 scripts/ 零改动、既有 note 文档零改动、零 git 写操作、零 sudo、零 ROS 节点与 roslaunch、零端口占用（11312/11313/11345 全程未触）；正文零 markdown 反引号；创建后已按 02 坑17 补 named ACL（setfacl -m u:ysc:rwx,u:ros1-slam:rwx，且后续任何编辑后须重补）。
