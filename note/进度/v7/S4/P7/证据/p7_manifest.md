# P7 场景组合启动「施工包」manifest（部署架构师-小张 · 2026-10-04）

> 蓝本：21 号 v4 §12（mode 表/三层 launch/铁律）+ §11.2（自检现行全文，沙箱已含 [9]，以 /home/ros1-slam/scripts/verify/v3_isolation.sh 现行为准）+ §11.3–11.5 判据；P6 四裁决 + 正式验收（G5 PASS / G6 PASS③部分 / G7 PARTIAL）为起点。
> 主理人前置裁决四条已全部落成本批载荷：GAP-1（M0/M1/M2 心跳行）、GAP-2（不砍第三方、自研 hdl 承载）、GAP-3（bringup mode 三件、mode_navigation 二期不建文件）、自检 [5] 修订（豁免 lite3_bringup）。
> 全部产出只写 /tmp；沙箱零改动；`ysc` 家目录路径零出现（A3 头注已按 D-V41-2 先例避让字面串）。

## 1. 产出文件清单（载荷 → 落盘目标）

| 载荷 | 落盘目标（ros1-slam 身份） | 类型 | sha256（前 16 位） |
|---|---|---|---|
| /tmp/p7_payload/A1_01_sensors.launch | ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch | 改（+心跳行） | 3ca1810197c96382 |
| /tmp/p7_payload/A2_10_slam_mapping.launch | ws/src/10_slam/lite3_slam/launch/10_slam_mapping.launch | 改（+心跳行） | b5898c81d9bc92b6 |
| /tmp/p7_payload/A3_hdl_localization.launch | ws/src/20_localization/lite3_localization/launch/hdl_localization.launch | **新建**（GAP-2/GAP-1-M2） | 3818e9d38915dc5a |
| /tmp/p7_payload/A4_mode_mapping.launch | ws/src/70_bringup/lite3_bringup/launch/mode_mapping.launch | **新建**（GAP-3） | d5586d250d27e03f |
| /tmp/p7_payload/A4_mode_localize.launch | ws/src/70_bringup/lite3_bringup/launch/mode_localize.launch | **新建**（GAP-3+A6） | 9353bd29e49b44ce |
| /tmp/p7_payload/A4_mode_nav_stub.launch | ws/src/70_bringup/lite3_bringup/launch/mode_nav_stub.launch | **新建**（GAP-3+stub TF） | 82b5c9300d4965df |
| /tmp/p7_payload/A5_v3_isolation_seg5.txt | scripts/verify/v3_isolation.sh [5] 段整段替换 | 改（[5] 修订） | 98a6534722b30b3b |
| /tmp/p7_payload/A6_map_supply_ruling.md | （裁决文档，不落 ws） | 裁决 | 7d1d57c6dd30dbf8 |
| /tmp/p7_payload/A7_registrations.md | 04 台账 6 条 + 02 坑册坑6 文案 | 登记 | f87762888bf4a046 |
| /tmp/p7_payload/B_p72_drill_plan.md | 演练计划 P7.1 落盘 + P7.2.0–P7.2.6 | 规程 | 8435f90eebd6d6f2 |
| /tmp/p7_payload/C_p7_criteria_dod.md | G 判据对照（17 条）+ DoD（10 条） | 判据 | e2468c61c1a9a088 |
| /tmp/p7_manifest.md | 本文件 | — | 见 /tmp/p7_manifest.sha256 |

> 全量 sha256：`sha256sum /tmp/p7_payload/* /tmp/p7_manifest.md > /tmp/p7_manifest.sha256`。
> 载荷内 6 个 .launch 均已 xml.etree 解析通过；零裸 cmd_vel 字样（唯一 cmd_vel 出现处含 cmd_vel_raw 子串，[6] 豁免口径）；心跳三话题与 health_watch.yaml 逐字一致；[5] 修订后对"现 ws + 新文件集"模拟扫描 = 零命中，平级互 include 探针模拟 = 必被抓。

## 2. mode 三文件结构摘要（GAP-3；起序语义 = 一窗起全部 + 各节点自带联锁）

| mode | include 顺序（SUP 恒首行） | 特有内容 | watch（health_watch 逐字） |
|---|---|---|---|
| mode_mapping | SUP(mode_mapping) → 01_sensors(M0) → 10_slam_mapping(M1) | 无 M6（hold 无消费者）；头注 GAP-5 样张红线 | M0_drivers + M1_slam |
| mode_localize | SUP(mode_localize) → 01_sensors(M0) → hdl_localization(M2,A3) → 60_safety(M6) | 无 map_server（A6 裁决 a）；43897 前置注记 | M0/M2/M6 |
| mode_nav_stub ★ | SUP(mode_nav_stub) → 01_sensors(M0) → **stub_tf_map_to_odom**（tf2_ros 静态 map→odom 恒等）→ 60_safety(M6) → mbf_navigation(M4) | ros2qnx 禁令 + 底盘速度唯一写者头注；stub TF 转正 | M0/M4/M6 |

- **一窗起全部**的理由：v4 铁律 SUP 最先起 = 文档首行；liveness 心跳为轻量 rostopic pub 秒级在位，一窗内不存在 P6.2"SUP 单独先起、模块跨窗后起"的误报主根因；livox required=true 提供整窗联锁；残余迟到竞态 → hold 误 latch → 重启 SUP 复位（P6.2 口径，判据入 B 章）。分底/上两个 launch 被否——跨窗间隔正是误报根因。
- **rviz arg** 默认 false（C10）并透传给声明了该 arg 的子 launch（supervisor/safety/mbf；sensors/slam/hdl 未声明不传，roslaunch 未声明 arg 传参会报错）。

## 3. stub TF 偏差说明（主理人可否决，A4 头注同步写明）

P6.3 B.3 的命令行 stub TF = map→base_link（P6 manifest §2 原文）。本批转正为 **map→odom 恒等**：
①P6 修正批实证 stamp=0 静态位姿被 mbf 保鲜校验拒收（"robot pose ~1.79e9s old" = now−stamp0），且 /tf_static 旧 latch 压制动态新戳（r4，需重启 M4 清）——若照搬 map→base_link 静态，P7 的 GetPath 冒烟回归将继承该缺陷路径；②map→odom 恒等 + 探针广播 odom→base_link 10 Hz 新鲜戳，链路 stamp 随动态段，冒烟零重启、可重复；③base_link→rslidar 恒等仍由驱动 base2livox 提供（G6-① 单位阵判据不受影响）；④二期真实定位接管 map→odom 时本行退役。若主理人坚持逐字复刻 map→base_link，仅改 A4 一行 + B 章探针改为"杀 stub 节点 + 重启 M4 + tfbr 直发 map→base_link"（修正批原路径）。

## 4. A6 裁决摘要（详见 A6_map_supply_ruling.md）

**选 a) mode_localize 不含 map_server**：①P6.2 无 map_server 契约全绿（hdl 用 pcd，/globalmap latched）；②样张 yaml origin z=nan 必被拒载（D-L1 实测），含入=常态死节点+噪音；裁决4 明令样张不回改、修 yaml 与重采同场；③b) 派生副本否决——绕过裁决4 通道 + 数据双源 + 违反本轮零新增硬约束；④2D /map 真实消费者是 M4 global_costmap，供给位在 mode_localize 属错位（map_server 归属 = 新决策点，移交 §9）。

## 5. 演练计划要点（B 章，P7.2）

- **P7.1** 变更批落盘（留底/静态三查/隔离静态复跑/台账落账）→ **P7.2.0** 方案 A 停栈（Q8 授权覆盖 P7，P6 A 章规程逐字复用）+ P0.5 双查 + 43897 闸门 + **GAP-5 样张备份**（三件套 sha256=P6 基线）。
- **P7.2.1** mode_mapping：8 节点在列/双心跳 1 Hz//rslidar_points≥9 Hz//Odometry≥10 Hz/SUP 静默；退净后 sha256 复核（GAP-5）。
- **P7.2.2** mode_localize：12 节点/三心跳//globalmap latched//imu/data≈200 Hz/双 TF 判据/**无 map_server 负判据**。
- **P7.2.3** mode_nav_stub：11 节点对照 watch 全存活 + 三 action 族 + ros2qnx 六查 + /cmd_vel 白名单三次 + mux 无源基态 + **GetPath 冒烟回归**（探针 = stub /map latched + odom→base_link 10 Hz fresh，代码入附录，goal 命令逐字复用 raw/修正批_4_getpath_smoke_final.txt）。
- **P7.2.4** SUP 真源验证：零代偿进程 + 60 s 零误报（P6.2 收口）→ pkill liveness_m4 → ≤3.5 s FATAL+hold=true+mux 零速 → 双重启复位（L1 语义）。
- **P7.2.5** isolation 全量：正向 ISOLATION OK（[5] 豁免生效）+ **反向探针**（lite3_slam 临时互 include 文件必被抓，验完删）。
- **P7.2.6** 退净 → foxy 恢复（A11/A12 逐字一致）→ 证据移 note/进度/v7/S4/P7/证据/。
- **停机条件**：stop 失败/残留、双查命中未处置、/cmd_vel 第二发布者、ros2qnx 出现、狗体反应、GetPath 同命令三败、livox 两次整窗退出。

## 6. DoD（P7 完成定义，10 条全文见 C 章 §二）

变更批落盘留底齐 → 三 mode 单命令起停全过 → nav_stub 全层存活+GetPath outcome=0 → SUP 真源验证（零误报+失联链通）→ isolation 全量含 [5] 正反向 → 样张/第三方 sha256 零漂移 → 真机红线四零 → foxy 恢复快照一致 → 04/02 登记 7 条落账 → 遗留显式移交（mode_navigation 二期 / map_server 归 M4 / L1 归 M5 / stub TF 偏差待裁决）。

## 7. 给小明（施工）注记

1. 执行序：P7.1（纯落盘）→ P7.2.0 → P7.2.1 → P7.2.2 → P7.2.3（含 P7.2.4 于同窗）→ P7.2.5 → P7.2.6；每 mode 独立 roscore，退净必杀 roscore（10_slam_mapping rosparam 全局污染）。
2. 全部 systemctl ysc+sudo；沙箱命令 ros1-slam + env.sh；证据 tee /tmp/p7_evidence/（汇总+raw）。
3. 改前留底三件：01_sensors/10_slam_mapping/v3_isolation.sh 的 .before_P7A*.bak；第三方 hdl launch 不触碰、演练后 sha256 对账。
4. mode 文件注释含字面串红线：launch 内不得出现裸 cmd_vel（唯一豁免=同行含 cmd_vel_raw）与 ysc 家目录绝对路径（[6]/[2]/[8]）——后续任何改注释都要重跑隔离自检。
5. P7.1 的 1.5 静态三查输出即留证（心跳话题对账逐条打印）。

## 8. 给小美（验收）提示

- 对应判据 = C 章 17 条对照表（每条给了出处/命令/判定窗）+ DoD 10 条；重点抽验：①三 mode launch 文档首行=SUP include（静态）；②心跳话题与 health_watch.yaml 逐字 diff；③[5] 正反向证据（P725_* 三件）；④样张 sha256 与 P6 基线（P61_pcd_baseline_sha256.txt）；⑤第三方 hdl launch sha256 前后一致；⑥GetPath result 原文 vs 修正批终态。
- P6 遗留口径衔接：G6-③ 由 PARTIAL 升 FULL（心跳转正）；G7-② 降级注记部分消除（M2 壳落地、定位质量仍不判）；L1/D-L1/底图重采不变仍挂账。

## 9. 本轮盘点新发现（路由主理人，不阻塞施工）

| # | 事项 | 建议 |
|---|---|---|
| N-1 | **stub TF 帧=map→odom 偏差**（§3）：与 P6.3 逐字口径不同，理由充分但属设计变更 | 请裁决：默认按 A4 落（推荐）；否决则按 §3 备选 |
| N-2 | **map_server 归属新决策点**：A6 裁决 a 后，重采修复 yaml 时 2D /map 供给位应设计在 M4 侧 | 与底图重采同场立项 |
| N-3 | lite3_localization/package.xml 无 hdl_localization/hdl_global_localization 的 exec_depend（roslaunch 不校验、现网能跑，纯规范性缺口） | 可选加固，非阻塞；如做需同步登记 04 |
| N-4 | 90_supervisor.launch 声明了 rviz arg 但无 rviz 节点（死 arg）；mode 文件透传无害 | 可选清理，归 SUP 维护批 |
| N-5 | GAP-5 语义扩大：P7 起任何 mode_mapping 演练（含未来常态使用）都需样张保护规程 | 建议把"备份→复核→恢复"固化为 mode_mapping 使用 SOP 入 00_README |

## 10. 登记项

04 台账 6 条 + 02 坑册坑6 文案全文见 /tmp/p7_payload/A7_registrations.md（小明随 P7.1 落账）；本阶段产出文件清单与本 manifest §1 一致，供 P7 进度文档头部引用。

---

## 附：P7 执行期主理人裁决注记（小明落盘时追加，2026-10-05）

- **N-1 修正**：§3/§2 的 stub TF 原案（map→odom 恒等静态 + odom→base_link 探针）**否决**；照准修正案 = 10Hz fresh-stamp 广播器 map→base_link 恒等，新增 `ws/src/70_bringup/lite3_bringup/scripts/stub_tf_broadcaster.py`（修正批终态 tfbr.py 同款逻辑，755，P7 变更批明单许可），mode_nav_stub.launch 引用之替代 static_transform_publisher。
- **N-2**：map_server 归 M4 侧（mode_navigation 二期），mode_localize 不含（A6 维持）——已注记 mode_nav_stub.launch 头注「地图供给」节，04 台账登记。
- **N-3 照准**：lite3_localization/package.xml 补 hdl_localization exec_depend（仅此一项）。
- **N-4 照准**：90_supervisor.launch 死 rviz arg 清理；联动=三 mode 文件 SUP include 去透传（roslaunch 未声明 arg 传参必报错），mode_mapping 同步去顶层 rviz arg。
