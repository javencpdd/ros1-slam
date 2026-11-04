# ros1-slam · 四足机器人 SLAM 与导航开发仓

> 更新时间：2026-11-04（v8 现行口径校对）｜ 权威口径：计划书 v8（note/计划书v8/）；本文件如与 v8 冲突，以 v8 为准

> 一句话：**机器狗（Lite3）的激光 SLAM 与自主导航开发仓**（ROS1／Noetic）——
> 源码在 `ws/`，进度·验收·证据·台账在 `note/`，门禁在 `scripts/verify/`，容器环境在 `docker/`。
>
> ★本文件是**项目门面**：只回答「这是什么、东西在哪、先读哪个」。
> ★跨 Agent 接力黑板（zcode／qoder／WorkBuddy）不在这里★，见第五节指针。

---

## 一、这仓是什么

面向四足机器人（Lite3）的**激光 SLAM 与自主导航**开发仓，已有 ROS1/Noetic、
`move_base_flex` 二维导航、SCAN 与仿真基础，以及可复跑检查和逐批台账。

**现行规划指针**：[计划书 v8](note/计划书v8/README.md)，目标是按场景与能力标签组合
2D/3D 定位、建图、全局与局部规划算法，并完成可信的三维导航仿真。
**当前状态（2026-11-02，R1115）：** 框架 F0—F8 的机制层与主要运行面已落册：真实 2D 栈由
v2 graph + Assembly 四格闭环（R981）；3D Assembly 三轮 **12/12**、起点扰动三种子 **12/12**（R1110/R1111）；
传感器建图 6/6（R1102）并验证同 XY 选面（R1108）；全局路径→MuJoCo 四足接触执行 **2/2**（R1112）；
A6 NDT 静态失锁重定位/租约门通过（R1106）；R1114 修复 lease 晚订阅竞态并跑通 0.3 m
局部失锁（候选仍停在 prior）；R1115 加 BBS 绝对源共识门，0.6 m 传送被拒（FAILED、lease 1）。
R1064 同 lab 挂起已抓栈根因并修复（R1113）。
**A1/A3/A4/A5/A6/A7 仍 PARTIAL，A2=PASS***；formal 与真机仍未签署，现有代码不等于全部目标能力。**
具体任务见 [执行队列](note/计划书v8/23_v8任务队列.md)，最新批次见 [进度入口](note/进度/README.md)。

## 一·五、框架要达成的目标

**不是再写一个算法，而是补齐可替换系统的脊柱：用契约描述能力，用 graph YAML 描述组合，
用运行时收口安全，用独立验证产生可信结论。** 目标能力：

- **地图可接入**：2D 栅格、3D 支撑/高程、语义地图；地图是版本化服务，支持增量、防回退、防膨胀。
- **机器人可接入**：Lite3、M20 等共用机器人 profile 与命令/遥测边界。
- **算法可替换**：建图、定位、全局、局部通过 manifest v2 的命名端口选择；算法内部策略留在自己的配置文件中。
- **中间件可插入任意边**：例如局部地图缓存按周期向全局地图提交 `MapDelta`，两端代码不改。
- **仿真独立验证**：MuJoCo/Gazebo 只验证组合，真值只进评价器；通行性 oracle 可查询但不冒充真机。

| 状态 | 内容 |
| --- | --- |
| **已实现（机制/假进程/回环/运行面）** | graph YAML、manifest v2、端口/边/DAG/命令唯一、实例 `config` 预检、2D/3D Path 与地图维度检查、Assembly/ProcessRunner、MapService/MapDelta + ROS provider、MapStore 多 map_id、边界 S1—S4、语义地图/障碍联动、plugin/nodelet 托管、2D v2 真实栈四格、3D Assembly 12/12×2、传感器图 6/6、同 XY 选面、A—E 应用层（app/task/map registry、surface resolver、nav3d Marker/RViz、Gazebo carrier、GT/LIO 对比）、全局→MuJoCo 四足 2/2、A6 NDT 静态 `~relocalize`/`~initialpose`/lease 门、R1064 挂起根因修复、2D/3D 交互式导航入口（MBF + RViz 2D Nav Goal；3D Publish Point 选面）、键盘建图→自建 3D 地图→交互导航（R1119，GT 位姿） |
| **未实现/未接** | 真叠层 mezzanine 场景与重传感器建图、A1/A4/A5 formal 签署、A6 任意 kidnapping/运动认证/真机、#151 formal 两键、`mode_navigation` 与 `ros2qnx` 真机运动通路、V8-X2 真机、Gazebo 四足步态（现为动态平面方盒；物理四足仅在 MuJoCo） |
| **开发中/边界** | 真叠层场景（新场景决定）、A1/A4/A5 formal 矩阵（L2）、#151 真机/稳定速度/同批负控（L2）、A6 长时失联/候选质量、V8-X1 0.6 m 阶沿鲁棒性；2D 四格 4/4、3D 12/12×2、MuJoCo 全局→四足 2/2 均仍非 formal/真机 |

框架机制说明见 [framework/README.md](ws/src/90_platform/v8_platform/framework/README.md)、
[架构导读](docs/architecture/framework-v0.md) 和 [图形化选配页示例](docs/architecture/framework-selector.html)。
**边界（2026-11-02，R1115）**：A1—A7 仍是 A2＝PASS*（窄域），A1/A3/A4/A5/A6/A7＝PARTIAL。R1112 的全局→MuJoCo 是单策略/声明图/预计算全局，不是 Gazebo 感知或真机；R1113 是测试台生命周期修复；R1114 的 A6 覆盖 0.3 m 局部失锁（候选仍 prior）；R1115 在带 BBS 绝对源共识门的活体链上闭合 0.6 m fail-open（不一致即 FAILED、lease 1），但 BBS 不是真实第二传感器、生产图需显式接入门；#151 formal 两键仍 pending，V8-X2 真机未开工。

**边界（2026-11-02，R1119）**：R1116—R1118 补齐 2D/3D 交互入口与 RViz 显示；R1119 打通键盘建图→自建 3D 地图→交互导航（GT 位姿 72,762 点、1 面支撑图、31 点路径到达）。建图是**仿真 GT 位姿 + LiDAR**，不是 LIO/SLAM；载体是运动学 `app_carrier`，不是四足物理步态/真机/formal；A1—A7 状态不变。

---

## 二、三条分工线（★本仓最要紧的一条纪律★）

| 面 | 放什么 | 不放什么 |
|---|---|---|
| `ws/` | 源码与配置，**只放代码** | 不放进度、不放台账 |
| `note/` | 进度真身／坑台账／判据表／计划书／证据目录 | 不放源码 |
| `scripts/verify/` | 门禁判据件，串跑入口 `run_gate_chain.sh` | 不放业务代码 |

> ⚠️ **进度真身只有一个＝`note/进度/00_总览.md`**。
> 换机器、换工具、换 Agent 都一样；任何摘要（含 `.ai/HANDOFF.md`）都只是「某次交接时」的快照。

---

## 三、全仓目录导航

| 顶层目录 | 是什么 | 入口 |
|---|---|---|
| `ws/` | catkin 工作区＝**源码面**（`build/` `devel/` 是编译产物、不入库） | `ws/README.md` |
| `note/` | **台账志文面**＝进度真身／坑台账／判据表／历代计划书 | `note/00_README.md`＝硬规则与现行规划指针 |
| `note/tools/` | 工具教程面（跨 Agent 协作黑板教程在此） | `note/tools/跨Agent协作黑板-使用教程.md` |
| `scripts/` | 门禁件 `scripts/verify/`（★十一件★）＋采图脚本族 `scripts/mapping/`（真机；仿真键盘建图见 `lite3_gazebo_sim`） | `scripts/mapping/README.md` |
| `docker/` | 容器开发环境（复刻 Jetson aarch64 的 Noetic 隔离沙箱）＋ `foxglove/` 布局件 | `docker/README.md` |
| `web/` | 项目官网（静态单页）：框架目标、架构、里程碑、判据现状、仿真入口 | `web/README.md` |
| `tooling/` | 项目自持清单（含 skill 逐字镜像＋ `*_sha256_manifest.txt` 见证清单） | `tooling/README.md` |
| `data/` | 底图与输出面（`maps/`＝建图三件套 .pcd/.pgm/.yaml，`waypoints/`＝航点 JSON） | `data/waypoints/README.md` |
| `opt/` | **第三方与原件区**（★原样复制、一字不改★；唯一许可例外＝faster-lio 存盘路径隔离） | `opt/upstream/`／`opt/3rdparty/` |
| `.ai/` | 跨工具接力黑板的状态目录（`bridge.py`＝唯一状态写入者） | `python3 .ai/bridge.py resume --as <工具名>` |
| `.qoder/rules/` | qoder 侧常驻提醒（带 `alwaysApply`） | `ros1-slam-collab.md` |

⚠️ `logs/` 与 `data/maps/` 下的产物**不入库**（`.gitignore` 已排除），换机只拿代码与台账即可重建。

---

## 三·五、官网介绍（`/web`）

仓库自带一份**静态单页**项目官网（零依赖、可离线打开）：定位、能力地图、架构、里程碑、
框架目标与三态现状、当前判据状态、仿真与观察入口、新算法接入方式、文档索引。

**启动方式**（必须从仓库根目录起服务，页面里的 `note/`、`docker/` 文档链接才可达）：

```bash
cd /home/jack/ysc-code/ros1-slam
python3 web/serve.py
# 浏览器打开 http://127.0.0.1:8000/web/
```

`web/serve.py` 会把服务根指到仓库根，并给文本补 `charset=utf-8`（否则中文 Markdown 会乱码）。
改端口或局域网访问：`python3 web/serve.py --port 8080 --bind 0.0.0.0`。

不需要文档链接时，也可以只服务站点目录：

```bash
python3 -m http.server -d web 8000   # 打开 http://127.0.0.1:8000/
```

局域网访问：`python3 web/serve.py --port 8000 --bind 0.0.0.0` 走 `http://<本机IP>:8000/`；
需要 80 端口又无 sudo 时，用 `bash web/publish.sh` 发布为 `http://<本机IP>/`（自动跳到 `/web/`）。

细节与更新约定见 `web/README.md`。页面数字是快照，权威仍以
[进度总览](note/进度/00_总览.md) 与 `note/计划书v8/calibration/` 现跑为准。

---

## 四、新接手先读哪三件

1. **`note/00_README.md`** —— 台账志文面的三条硬规则（进门先看这个）
2. **`note/09_跨工具协作契约.md`** —— 跨工具进场的唯一契约件（四家工具必读：zcode／qoder／WorkBuddy／Codex；第 9 节＝非 L2 自动推进常设授权）
3. **`AGENTS.md`** —— 仓根三行指针（★不入仓★，只指向上面那件）

跑门禁前的固定姿势（★顺序错＝假红★）：

```bash
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
bash scripts/verify/run_gate_chain.sh
```

---

## 五、跨 Agent 接力黑板（zcode／qoder／WorkBuddy）

**不在这里。** 三家工具怎么接力、`bridge.py` 怎么用、踩过哪些坑，全部在：

→ **`note/tools/跨Agent协作黑板-使用教程.md`**

---

## 六、本文件的变更留痕

- ★R579／批231（2026-10-23）★＝真人明令「**项目根目录下的 README 应该是对项目整体的介绍，
  跨 Agent 协作黑板这部分应该归纳到 tools 目录**」⇒ 本文件由「跨 Agent 协作黑板 — 使用说明」
  重写为**项目整体介绍**；黑板内容归位 `note/tools/跨Agent协作黑板-使用教程.md`
  （该件 2026-10-17 已存在，十节＋附，内容比旧版更全）。
- 归纳时逐节比对：根 README 旧版第一—七节中，**「可选增强」与「参考外链」两节**是教程件没有的，
  已按「内容不丢」原样并进教程件的**附二**；其余各节教程件均已覆盖。
- 同时销项一笔在册旧债：`note/04_配置改动diff.md` 第 3117 行登记的「丁类第一件」
  （仓根 README 门面与黑板并存两读，可勾形＝并存／保留指针各归各／搬到 `.ai/`），
  真人所勾＝**「保留一句指针后各归各」形，落点＝`note/tools/`**（非 `.ai/`）。
- 旧版正文一字未删：可从 git 取回，基准提交＝`fcc2634`。
- ★R979（2026-10-31）★＝新增「一·五、框架要达成的目标」：四层解耦、五项能力、
  已实现/未实现/开发中三态表；明确“机制层已成形、真实算法 v2 化与真实 launch 未闭环”；
  同步官网 `#framework` 页。A1—A7 状态不变。
- ★R1058（2026-11-01）★＝**修复后长距感知路径矩阵**：18/18 PASS，t_sense 最坏 .879 s / .1825 m / gap 1.464 m；实际净速 .181—.331（触发在加速段），**不作 formal 速度样本**（全文＝[R1058](note/进度/v8/M5/26-10-29/R1058.md)）。
- ★R1060（2026-11-01）★＝**MuJoCo 物理射线感知**：bridge45 `--sense` 用 9 条 `mj_ray` 打物理 barrier，命中后发布 map box、首帧锁定；smoke 停发虚拟 box。.28 感知即停（stop .6 s/forward .045/gap 2.14）。全树 70/0（代码 `22aab848`）。
- ★R1061（2026-11-01）★＝**物理射线传感器 18-run**：.28/.45 × 3 起点 ×3 seed，**18/18 PASS**，实际 **0.205—0.250 / 0.301—0.410 m/s**，最坏 **0.80 s / 0.1695 m / gap 1.566 m**；MuJoCo raycast 非 Gazebo 激光/真机，formal 仍 pending（全文＝[R1061](note/进度/v8/M5/26-10-29/R1061.md)）。
- ★R1062（2026-11-01）★＝**Gazebo 真实激光通道接入**：m1 `/rslidar_points` 经 rslidar 系 ROI、实测位姿＋R409 15° 安装变换成 map AABB，本地节点 **E_LOCAL_NO_ROUTE**；n=3 **3/3 rc=0、VALID**，AABB 全同 `[1.18,-0.32,0]—[1.52,0.42,0.62]`，触发→静止 **0.070—0.081 m**、GT 净空 **0.818—0.831 m**；**负项**：感知→静止仅 **0—0.0088 m**（物理停顿由分段边界主导），不成立感知触发制动距离、不动 #151 formal；全树 70/0（代码 `a3af4e9f`，全文＝[R1062](note/进度/v8/M5/26-10-29/R1062.md)）。
- ★R1063（2026-11-01）★＝**事件唤醒规划器＋Gazebo 感知触发物理停车**：`ros_node` 新增 `plan_wake`（新目标/障碍/地图/定位失效与段完成立即唤醒，规划在途 20 ms 轮询），去掉 0.2 s 节拍造成的分段停顿；箱体 x=2.0 使首次 ROI 可见落在分段中点。n=3 **3/3 rc=0、VALID**，感知时实速 **0.400 m/s**、输入后无 `segment_complete`、`E_LOCAL_NO_ROUTE` 后置；感知→静止 **0.0128—0.0176 m**、触发→静止 **0.3156—0.3187 m**、GT 净空 **1.083—1.087 m**；arrival 回归 PASS。平面载体 1.3—1.8 cm 是自身制动，不是四足/真机/#151 formal 预算；全树 70/0（代码 `23e5e9f8`，全文＝[R1063](note/进度/v8/M5/26-10-29/R1063.md)）。
- ★R1064（2026-11-01）★＝**事件唤醒后回归**：四例逐个各开 lab，sensor_freshness/localization 均 VALID；**负项**：四例同一 lab 串跑在第三例后挂起（500 s timeout、status 无消息、三个 adapter，未定位；逐例全过）。ROS45 停车链按 R1048 模板重采样 **18/18 PASS**，最坏 **0.80 s / 0.0966 m / 净空 0.627 m**，全部在 R1050 scoped **1.2 s/0.21 m** 内、路径 18 条互异；calibration 与 digest 未改，formal #151 仍 pending（代码 `23e5e9f8`，全文＝[R1064](note/进度/v8/M5/26-10-29/R1064.md)）。
- ★R1065（2026-11-01）★＝**物理射线 18-run 后重采样**：事件唤醒后按 R1061 模板重跑 `.28/.45 × 3 起点 × 3 seed`，**18/18 PASS**、路径互异；最坏 **0.90 s / 0.1684 m 路径 / 0.1342 m 前向 / 1.569 m 净空**，相对 R1061 基线仅停止窗 +0.1 s、路径/净空无实质变化；calibration 与 digest 未改，formal #151 仍 pending（MuJoCo 射线非 Gazebo/真机；代码 `23e5e9f8`，全文＝[R1065](note/进度/v8/M5/26-10-29/R1065.md)）。
- ★R1066（2026-11-01）★＝**修 landing 元数据与 faster_lio 仿真配置隔离**：CLI 的 `stairs` 元数据补 `landing_depth`，修复 landing 世界被无 landing 分支误判；重跑 R942 L12/L6/L2 与 R935，判定与 R943 重算一致、判据门值未改。faster_lio 新增 `v8.fw.faster_lio_sim` manifest，7 个模拟图切到 sim 组件，sim launch 去宿主绝对路径，新增仿真/真机配置禁止交叉加载断言；framework 30/30、全树 70/70。**影响**：R1042 双定位对照此前加载真机 yaml，A6 需按 sim 配置重跑（代码 `d656c723`，全文＝[R1066](note/进度/v8/M5/26-10-29/R1066.md)）。
- ★R1067（2026-11-01）★＝**V8-X1 确定性诊断**：新增 bridge/runtime 全 tick 轨迹、ONNX 单线程/顺序、未激活不跑策略；同 seed0 两份在 sim .02—.08 逐位相同、.10 首分歧来自本地节点命令墙钟相位，**3/3 PASS 但末态 x 17.96/24.32/14.02 仍不同，确定性未成立**；`finish-hold` 两份均失败（一份碰撞），默认姿态保持亦失败并回退。下一步固定逻辑 tick＋命令序列/握手与到顶减速/支撑保持（代码 `93025b84`，全文＝[R1067](note/进度/v8/M5/26-10-29/R1067.md)）。
- ★R1068（2026-11-01）★＝**V8-X1 命令按 sim 时间戳选择**：bridge 新增 `--command-stamp-window`（发布时钟域选最近命令），先踩漏 `clock_offset` 的全零命令，修正后 run20 PASS/run21 FAIL，同 seed 首分歧仍在 **sim .18**；选项默认 0 关闭，不改变现有链；bridge 6/6、全树 70/70。结论：时间戳选择必要但不充分，下一件固定逻辑 tick＋命令序列/握手（代码 `8c2d42a7`，全文＝[R1068](note/进度/v8/M5/26-10-29/R1068.md)）。
- ★R1069（2026-11-01）★＝**V8-X1 命令握手尝试**：节点按 `/clock` 逐 tick 生成带 `header.seq` 的命令，桥每 active tick 等新 seq；同 seed0 两份首分歧仍在 **sim .06**，run22 `E_LOCAL_CLEARANCE`、run23 PASS；旗标默认关闭，node policy 13/13、全树 70/70。结论：固定 tick/握手未成立，下一步 A6 sim 配置重跑与 GT/恢复闭环（代码 `196bac7e`，全文＝[R1069](note/进度/v8/M5/26-10-29/R1069.md)）。
- ★R1070（2026-11-01）★＝**A6 双定位 sim 配置重跑**：faster_lio 改走 sim manifest 后 30 帧 **mean 0.0013 m / max 0.0029**（旧真机 yaml 为 .0264 m），hdl 30 帧 **mean 0.3063 / max 0.3317**；pose_mux 切换 accepted、最大间隔 50.5 ms、退净。静态场景、真值只评分，**无 GT 门/重定位/失效恢复**，A6 仍 PARTIAL（代码 `d656c723`，全文＝[R1070](note/进度/v8/M5/26-10-29/R1070.md)）。
- ★R1071（2026-11-01）★＝**A6 恢复 supervisor**：新增纯状态机 `TRACKING→RELOCALIZING→候选验证→RECOVERING→新 lease/切后端→TRACKING`，超时 FAILED、旧 lease 不复活；单测 5/5、全树 **71/71**。未接 hdl `/relocalize`/候选 ROS 源、GT 门、Gazebo 活体与真机；A6 仍 PARTIAL（代码 `34bf2d1c`，全文＝[R1071](note/进度/v8/M5/26-10-29/R1071.md)）。
- ★R1072（2026-11-01）★＝**A6 恢复 ROS 接线**：新增 `a6_recovery_node`（odom/health/candidate/lease_in→`/relocalize`→候选验证→backend/lease/status/command_allowed），修 supervisor 的 RELOCALIZING→TRACKING 与 RECOVERING 等待；假服务/假候选 smoke **ok=true**（lease1 TRACKING→stale→/relocalize→候选→backend faster_lio lease2→TRACKING、allowed=true）。未接 hdl 实服务/Gazebo 活体/GT 门/真机，A6 仍 PARTIAL；supervisor 5/5、全树 71/71（代码 `c2e0b7f5`，全文＝[R1072](note/进度/v8/M5/26-10-29/R1072.md)）。
- ★R1073（2026-11-01）★＝**A7 小纵切**：SC09/seed0，四组合×truth/noisy，**8/8 PASS**；noisy 最坏 dist .247 m、净空 .312 m、注入 .587 m（truth 四格引 R981）；单场景/单 seed，不代签 A7（代码/图 `57d50bf2`，全文＝[R1073](note/进度/v8/M5/26-10-29/R1073.md)）。
- ★R1074（2026-11-01）★＝**hdl 全局定位 m1 高度带 smoke**：**另起句更正**——`use_global_localization` 是 launch 默认值可覆盖、PCD 可换仓库/仿真图、BBS 2.0—2.4 m 只是 lite3.pcd 的切片；m1 图 z 0—1.0 m，改切片 **0.05—1.0 m** 后 `set_global_map`+`query` 返回 **1 个候选**（identity），无 slice 告警。查询云由同图派生，不等于活体 Gazebo/精度；A6 仍 PARTIAL（代码 `c181753d`，全文＝[R1074](note/进度/v8/M5/26-10-29/R1074.md)）。
- ★R1075（2026-11-01）★＝**hdl 全局定位活体查询**：m1 Gazebo 真实 `/rslidar_points`→BBS query；res .5/.2 候选误差 **.50/.20 m**，但 res .1 候选 **(4.0,−.1)**、误差 **4.0 m**（service error 6.8 反而最低）⇒ 服务 error 不能当接受分；配置/路径/高度带不是阻塞，**候选重评分/一致性验证+GT 事后门**才是 A6 缺口。单帧静态、未接 `/relocalize`/lease 活体；A6 仍 PARTIAL（代码 `8998c1d6`，全文＝[R1075](note/进度/v8/M5/26-10-29/R1075.md)）。
- ★R1076（2026-11-01）★＝**A6 候选共识闸**：`CandidateConsensus` 多分辨率完全链接聚类，≥2 分辨率同簇才接受、取最细候选；活体 m1 Gazebo 真实扫描三实例 .5/.2/.1 候选 (0.5,0)/(0,-.2)/(1.0,-.1)，共识接受 **.2 m 候选**（事后误差 .2 m）、剔除孤立 1.0 m 错候选；4/4、全树 72/72。未接 hdl `/relocalize`/lease 活体，A6 仍 PARTIAL（代码 `035631a8`，全文＝[R1076](note/进度/v8/M5/26-10-29/R1076.md)）。
- ★R1077（2026-11-01）★＝**A6 活体恢复闭环**：m1 Gazebo 真实扫描→三 BBS 实例共识→`/relocalize` 适配→`a6_recovery_node`；lease1 TRACKING→失锁 RELOCALIZING→候选 (0,−.2)（.1 的 4.0 m 候选剔除）→backend faster_lio lease2→TRACKING、allowed=true；GT 误差 **.20 m**。`/relocalize` 为 smoke 适配服务非 hdl 原生，单静态/单次、无真机；A6 仍 PARTIAL（代码 `7529d172`，全文＝[R1077](note/进度/v8/M5/26-10-29/R1077.md)）。
- ★R1078（2026-11-01）★＝**A6 原生 hdl `/relocalize` 活体链**：m1 传感器系点云按 base_link 喂 stock hdl nodelet（BBS 单候选、原生 `/relocalize`）；失锁态（扫描暂停＋错误先验复位）3 位姿 GT **0.0229/0.0643/0.0650 m**，6 次服务全返回、scan matching 收敛。**负项**：跟踪态在 **0.0785 m** 好位姿上误调 `/relocalize` 跳到 (3.90,−0.76)、误差 **5.144 m**、yaw π；单候选原始扫描交叉 **0/0.8/4.59 m** ⇒ 必须先判失锁＋候选验收。3 静态/单次/无真机，A6 仍 PARTIAL（代码 `3d8953af`，全文＝[R1078](note/进度/v8/M5/26-10-29/R1078.md)）。
- ★R1079（2026-11-01）★＝**A6 原生恢复闭环**：失锁→最近 2 帧最细 BBS 候选（map data .2/grid .1/query .2）在 1.5 m 先验内取最近→A6 backend→hdl 原生 `/initialpose`（含 yaw）→新 lease；3 姿全过：`(1.5,0)` 两轮 lease 1→2→3、GT **0.0141/0.0134 m**，`(0,1)` **0.0740 m**，`(-1,0.8)` **0.0987 m**；健康期 relocalize 调用 0；空扫描负控 FAILED、allowed=false、无 reset。**发现**：粗分辨率多数派会压过正确细候选，单帧 BBS 随扫描相位翻盆；2 帧+先验+宽 z 稳定到 0.077—0.097 m。静态仿真/无真机、先验仅覆盖短时失锁，A6 仍 PARTIAL（代码 `e253392f`，全文＝[R1079](note/进度/v8/M5/26-10-29/R1079.md)）。
- ★R1080（2026-11-01）★＝**V8-X1 执行器回放确定性**：bridge 新增 `--replay-trace`，按记录逐 tick 应用 command/stepping（关闭 stall/finish/phase/stamp/lockstep），两次独立跑 500 tick **完全一致、max_abs_diff=0.0**，末态 qpos 同为 **[3.1116,−0.2358,0.2615]** ⇒ 执行器（观测历史/单线程 ONNX/MuJoCo）确定；R1067—R1069 的端到端首分歧 **.06—.18** 来自 ROS 命令生成/调度，不是策略或积分。开环固定命令、前 500 tick、未到顶/无真机，V8-X1 仍 PARTIAL（代码 `1a9d5462`，全文＝[R1080](note/进度/v8/M5/26-10-29/R1080.md)）。
- ★R1081（2026-11-01）★＝**V8-X1 命令锁步真实接线**：runtime 传 `_lockstep_command`，bridge 每 active tick 等新 `header.seq`、clock 先于 pose，node 暴露 `clock_error`；真跑 x=6.34 m、plans=2 时 **E_LOCAL_CLOCK**（pose timestamp is in the future），锁步与节点延迟耦合导致 clock 超前。**另起句更正 R1069**：当时 bridge 只解析未使用、runtime 未传节点参数，握手为空转；R1069 读数真实但握手结论不成立。单 seed/单 run/无真机，V8-X1 仍 PARTIAL（代码 `38672dc2`，全文＝[R1081](note/进度/v8/M5/26-10-29/R1081.md)）。
- ★R1082（2026-11-01）★＝**V8-X1 顶部零指令保持回放**：R1067 `s0_10` 登顶段（support .890358、z 1.1469）追加 500 tick 零 command，两次回放 **1849 tick 完全一致**（NaN/Inf 同值判定修正）；保持段 final **z .1857**、z 降 **.9612 m**、xy 漂移 **5.3478 m**，tick≈967 出现 ±Inf action ⇒ 零指令不能保持顶部，需减速/支撑感知保持或策略侧 nonfinite 处理。开环/无真机，V8-X1 仍 PARTIAL（代码 `3e798da3`，全文＝[R1082](note/进度/v8/M5/26-10-29/R1082.md)）。
- ★R1083（2026-11-01）★＝**A7 可复现双轨矩阵**：SC09，四组合（GP/Navfn × DWA/TEB）× truth/noisy × seed **0/1**；本批新增 seed1 八格，seed0 八格引 R1073/R981 ⇒ **16/16 PASS**；全 16 格最坏 dist **.247 m**、净空 **.312 m**、注入箱 **.581 m**；seed1 最坏 .241/.314/.615。仅单场景/单重复、无 LIO/NDT/真机，A7 仍 PARTIAL（代码/图 `3e798da3`，全文＝[R1083](note/进度/v8/M5/26-10-29/R1083.md)）。
- ★R1084（2026-11-01）★＝**A1 全组合矩阵（seed0）**：4 图（GP/Navfn × DWA/TEB）× SC01/SC02/SC09 × truth/noisy = **24/24 PASS**，wall 416 s；最坏 dist **.244 m**、净空 **.279 m**、注入箱 **.578 m**。原 72 格（seed0/1/2）在 6-worker 下内存/RTF 塌缩，完成 9 个 seed1/2 辅助格后停止；本批仅 seed0 组合矩阵，seed 覆盖引 R893/R1083，A1 仍 PARTIAL（代码 `a011dcbe`，全文＝[R1084](note/进度/v8/M5/26-10-29/R1084.md)）。
- ★R1085（2026-11-01）★＝**A3 多层地图运行面**：a3_runtime 同 XY 双层/unknown/净空/版本/单向 **15/15**；multi_map A/B/C 初始 [2,2,1]→delta A=3→restart/replay [3,2,1]，grid 100×100@.1、**ok=true**。仍声明夹具、无传感器/Gazebo 多层，A3 PARTIAL（代码 `3e798da3`，全文＝[R1085](note/进度/v8/M5/26-10-29/R1085.md)）。
- ★R1086（2026-11-01）★＝**A4 参考解与连续净空合并**：节点与设计层同图/预算/`min_clearance_m` 逐点比较；`reference_clearance` 20 cell，16 pass+4 expected、**unexpected=0、same_as_design=20**；SC09 在 .2 m 净空下走安全绕行并通过。仍 support-graph/非 Gazebo 全链，A4 PARTIAL（代码 `280644a5`，全文＝[R1086](note/进度/v8/M5/26-10-29/R1086.md)）。
- ★R1087（2026-11-01）★＝**Gazebo 感知泛化**：障碍中心/尺寸参数化；两活体变体 A (1.8,.2)/.5×.4×.7、B (2.1,−.15)/.6×.5×.8 均 **ok=true VALID / E_LOCAL_NO_ROUTE**，感知后 **.0200/.0036 m** 停，GT 净空 **1.0577/1.1483 m**。平面 Gazebo、单变体单次、无四足/真机，A5 仍 PARTIAL（代码 `67e6b8c0`，全文＝[R1087](note/进度/v8/M5/26-10-29/R1087.md)）。
- ★R1088（2026-11-01）★＝**开放地图/世界/数据导入**：新增 `open_data_import`（PCD→map.pcd+PGM/YAML、SDF+model:// 依赖、dataset 清单、递归 verify，带源/许可/sha256 provenance）；venue PCD 185 872 点 → **119×111@.1、7 774 占据**，`map_server` 载入逐项一致；NIST world 导入后 Gazebo 生成 ground_plane/nist_stairs_120/lite3_planar；dataset verify 三层全过。占据图是投影、许可证按文件实况未声明，非传感器建图/真机，A1—A7 不变（代码 `17ac93d4`，全文＝[R1088](note/进度/v8/M5/26-10-29/R1088.md)）。
- ★R1089（2026-11-01）★＝**V8-X1 回放保真复核**：R1067 `s0_10` 自身作回放源，两次回放仍 1349 tick 一致，但回放 vs 源首分歧 **tick177/sim1.12/.0065 m**、最大 qpos 差 **347.8**，回放 tick1050 `action_nonfinite` 而源无 fault；**R1082 的零指令顶部跌落结论作废**，top finish 仍 OPEN（代码 `f5bb409d`，全文＝[R1089](note/进度/v8/M5/26-10-29/R1089.md)）。
- ★R1090（2026-11-01）★＝**A1 全组合矩阵补全**：seed1/2 48 格 **48/48 PASS**（wall 1085.9 s），与 R1084 seed0 24 格合并 ⇒ **72/72 PASS**（总 wall 1501.9 s）；全 72 格最坏 dist **.248 m**、净空 **.279 m**、注入箱 **.578 m**。仅 2D Gazebo 到达型，3D 组合未入正式集，A1 仍 PARTIAL（代码 `a011dcbe`，全文＝[R1090](note/进度/v8/M5/26-10-29/R1090.md)）。
- ★R1057（2026-11-01）★＝**感知路径障碍早拒**：修复前 7/18、.45 全碰撞；静态障碍挡目标线且本地 horizon 未达即 `E_LOCAL_NO_ROUTE`，修复后 .28 感知即停（path .04 m、gap 1.65 m）。local 20/20、全树 70/0（代码 `1ce3fd42`）。
- ★R1059（2026-11-01）★＝**短距感知路径 18-run + 同批负控**：sensor .8/1.0/1.2、seed 0/1/2、.28/.45；**18/18 PASS**，实际 **0.209—0.238 / 0.418—0.451 m/s**，最坏 **0.965 s / 0.179 m / gap 0.868 m**；kinematic 6/6 path 0。formal 仍因合成感知/无真机 Gazebo/独立样本不足 pending（全文＝[R1059](note/进度/v8/M5/26-10-29/R1059.md)）。
- ★R1056（2026-11-01）★＝**同批 kinematic 负控**：无制动载体 6/6 PASS，stop 仅 hold .5 s、path/forward **0**、gap .704—.736；动力学链 .28/.45 path .097/.190，证明制动段来源（代码 `313b3550`，全文＝[R1056](note/进度/v8/M5/26-10-29/R1056.md)）。
- ★R1054（2026-11-01）★＝**45 桥 seed 扰动/高速度**：`.45` 命令实际 **0.413 m/s**，PASS（代码 `33c76a76`）。
- ★R1055（2026-11-01）★＝**0.28/0.45 三 seed 实速矩阵**：18/18 PASS，实际 **0.204—0.231 / 0.432—0.454 m/s**，低组触发瞬时 **0.243—0.355**；t_sense 最坏 **0.864 s / 0.190 m / 净空 0.594 m**；formal 仍待负控/路径感知（全文＝[R1055](note/进度/v8/M5/26-10-29/R1055.md)）。
- ★R1052（2026-11-01）★＝**tracker 越过段目标即推进**：修低速前后来回；local 19/19、execution 16/16、全树 70/0（代码 `2d280f1e`）。
- ★R1053（2026-11-01）★＝**0.28/0.4 实速 18-run 矩阵**：实际净速 **0.208—0.239 / 0.341—0.389 m/s**、18/18 PASS、RTF 1.0；t_sense 最坏 **0.895 s / 0.136 m / 净空 0.575 m**；独立复核判 **formal 不冻结**（低组触发 2/9<0.2、高组 max .389、缺负控/seed/感知路径），formal 两键仍 pending（全文＝[R1053](note/进度/v8/M5/26-10-29/R1053.md)）。
- ★R1051（2026-11-01）★＝**0.2 实速策略探针**：45 策略在平地命令 .22 时实际 **0.204 m/s**；但 ROS 单链 0.22/res .1 仅 **0.019 m/s 净前进**（tracker 低速横向/倒退振荡），不能作 .2 实速样本。formal #151 仍 pending；只做探针，不改判据（全文＝[R1051](note/进度/v8/M5/26-10-29/R1051.md)）。
- ★R1050（2026-11-01）★＝**ROS45 停车 scoped 冻结**：新增 `stop_chain_ros45_time_s=1.2`、`stop_chain_ros45_distance_m=0.21`（scope=`ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime`、n=18）；按 t_sense 修正起算最坏 **0.8559 s / 0.1549 m**，净空 **0.582 m**。formal 两停车键仍 pending；digest `9c280489…8a28`；calibration 63/63、全树 **70/0**（代码 `b6fa94ea`，全文＝[R1050](note/进度/v8/M5/26-10-29/R1050.md)）。
- ★R1049（2026-11-01）★＝**0.2 停车失败根因**：res .05 下 0.2 规划 **E_LOCAL_BUDGET**（20000、10.4 s）；res .15 可规划（76 ms）但 45 策略在 `body_command≈0.19` 时实速≈0（低命令死区）；`E_LOCAL_POSE_STALE` 是陈旧 last_error（pose 5942 帧、frame=map）。formal #151 仍 pending；只改 smoke 诊断开关（代码 `a1724b75`，全文＝[R1049](note/进度/v8/M5/26-10-29/R1049.md)）。
- ★R1048（2026-11-01）★＝**ROS 45 策略 18-run 停车矩阵（partial）**：0.3/0.4 m/s ×3 起点×3 触发相位，**18/18 PASS**、每速 9 条互异、RTF 1.0；原起算最坏 **0.80 s / 0.114 m 路径**，按首个障碍输入修正起算最坏 **0.856 s / 0.1549 m**，净空 **0.582 m**。0.2 m/s 干净重跑后更正为 res .05 规划预算耗尽 / res .15 低命令死区（pose-stale 是陈旧 last_error）；独立复核判 **formal 不冻结**（1.1/0.15 不足，0.3/0.4 只能 scoped）。全树 **70/0**（代码 `586f79c1`+`8d3e0314`，全文＝[R1048](note/进度/v8/M5/26-10-29/R1048.md)）。
- ★R1047（2026-10-31）★＝**45 维策略 ROS 桥（partial）**：真实 `ros_node` + 45 策略桥在平地 blocked-goal 场景，0.3/0.4 m/s 均 **PASS**（stop **0.60/0.70 s**、前向 **0.026/0.078 m**、净空 **0.685/0.606 m**）；0.2 m/s 120 s 未到触发点（低速死区），**未形成 0.2/0.4 双速度样本**。formal #151 两键仍不冻结；全树 **70/0**（代码 `586f79c1`，全文＝[R1047](note/进度/v8/M5/26-10-29/R1047.md)）。
- ★R1046（2026-10-31）★＝**ROS 全链停车 pilot（partial）**：真实 `ros_node` + 270 策略桥在平地执行 blocked-goal 场景，0.3 m/s **PASS**（sensing→停 **2.30 s**、前向 **0.071 m**、净空 **0.674 m**）；0.2 m/s 60 s 未到触发点、0.4 m/s 策略跌落，**未形成有效双速度样本**。formal #151 两键仍不冻结；全树 **70/0**（代码 `69cb7a42`，全文＝[R1046](note/进度/v8/M5/26-10-29/R1046.md)）。
- ★R1045（2026-10-31）★＝**A1—A7 正式签署矩阵**：按 R930 盘点＋R1035—R1044 增量，A2＝PASS*（真进程 6/6＋Gazebo 感知触发 3/3），A1/A3/A4/A5/A6/A7＝PARTIAL；明确不声称平台首版/3D 导航/真机/#151 formal。矩阵与回执入 `evidence/R1045/`（全文＝[R1045](note/进度/v8/M5/26-10-29/R1045.md)）。
- ★R1044（2026-10-31）★＝**局部规划目标占据即拒 + MuJoCo 力矩停车 scoped 冻结**：`local_native.plan` 对目标落在活跃静态障碍膨胀盒内直接 `E_LOCAL_NO_ROUTE`，消除逼近到约 0.2 m 才拒的旧行为；18 run（0.2/0.4×3 起点×3 相位）全 PASS，sensing→stop 最坏 **2.744 s / 0.153 m**，最小净空 **0.454 m**；新增 scoped `stop_chain_mj_torque_time_s=3.6`、`stop_chain_mj_torque_distance_m=0.20`（n=10 互异）。in-process、无 ROS/真机，**formal 两停车键仍 pending**；全树 **69/0**（代码 `ed1cc260`，全文＝[R1044](note/进度/v8/M5/26-10-29/R1044.md)）。
- ★R1042（2026-11-01）★＝**双定位后端同场对比与切换**：hdl 与 faster_lio 同时在线，各 30 样本；hdl 误差 mean **0.309 m**、faster_lio **0.026 m**；pose_mux 从 hdl 切到 faster_lio 被接受、输出最大间隔 **50.5 ms**。静态场景、非零跳变切换、未重定位，不代签 A6；全树 **69/0**（代码 `6c6a85b2`，全文＝[R1042](note/进度/v8/M5/26-10-29/R1042.md)）。
- ★R1041（2026-11-01）★＝**hdl 定位打通**：所有探针缺 `/leg_odom2`，hdl 在配准后解引用空 `leg_odom2_msg` 崩溃；补 10 Hz leg odom 心跳后，原版 NDT_OMP 发布 `/odom` **10 帧/3 s（3.33 Hz）**，末位置 [0.1248,-0.0073,0]。授权下尝试的第三方补丁已全部回滚；R1027 归因更正为缺 leg odom。全树 **69/0**（代码 `58f35603`，全文＝[R1041](note/进度/v8/M5/26-10-29/R1041.md)）。
- ★R1040（2026-10-31）★＝**真实三维链**：两真实 cache→真实 merge→typed relay→`delta_map_provider`（补 shebang）→真实 3D 全局/局部规划器；flush 后地图 v2→v3，全局路径 6 点、局部 v3 arrived。F7 仍拒绝无三维仲裁器的 Assembly。preflight 12/12、全树 **69/0**（代码 `316f09ea`，全文＝[R1040](note/进度/v8/M5/26-10-29/R1040.md)）。
- ★R1039（2026-10-31）★＝**faster_lio 运动段重复性**：同场景连续两跑均 ok，平均误差 2.59/2.58 cm（差 0.01 cm），末帧误差差 2.05 cm，odom 位移差 3.5 cm；同环境两跑、非 A7 正式集，不代签（全树 **69/0**，全文＝[R1039](note/进度/v8/M5/26-10-29/R1039.md)）。
- ★R1038（2026-10-31）★＝**faster_lio 运动段误差**：m1 模型前进—转向—前进，真值只做事后评分；**70** 组采样下相对轨迹平均误差 **3.6 cm**、最大 **7.1 cm**、末帧 **6.1 cm**。A6 式旁证，不代签；全树 **69/0**（代码 `db912a8a`，全文＝[R1038](note/进度/v8/M5/26-10-29/R1038.md)）。
- ★R1037（2026-10-31）★＝**三维全局规划可取消**：`support_planner` 在净空/障碍过滤与跨面最近格对搜索中也轮询取消；`topic_node` 后台规划 + Trigger cancel。10 s 级无路搜索 cancel 后 **0.10—0.16 s** 返回 `E_PLAN_CANCELLED`，无非空 Path。adapter 4/4、planner 12/12、全树 **69/0**（代码 `c2b31f89`，全文＝[R1037](note/进度/v8/M5/26-10-29/R1037.md)）。
- ★R1036（2026-10-31）★＝**A4 运行时证据**：真实 3D 全局规划器路径经独立 `trajectory_checker` 复核（6 点、0 违例）；整墙障碍 `E_PLAN_NO_ROUTE`；`budget=1` 得 `E_PLAN_BUDGET_EXCEEDED`，无路与超预算可区分。取消分支未接适配器，不代签 A4；全树 **69/0**（代码 `a2a7352a`，全文＝[R1036](note/进度/v8/M5/26-10-29/R1036.md)）。
- ★R1035（2026-10-31）★＝**真实三维全局规划器入 v2 图**：`planner/topic_node.py` 包装真实 `support_planner`，map/start/goal 入、Path/status 出，地图更新或新目标即重规划；真实冒烟首目标 6 点、改目标 8 点、地图 v3 后按 v3 重规划。adapter 3/3、preflight 11/11、全树 **69/0**（代码 `107dca2e`，全文＝[R1035](note/进度/v8/M5/26-10-29/R1035.md)）。
- ★R1034（2026-10-31）★＝**命令源重叠交接**：新增 `v8.fw.mux_dual` 与双源配置，真实 mux 在 A 源保持时启动更高优先级 B，下一 tick 切换；B 超时后回落 A。切换与回落窗口最小输出 0.15/0.3，**无零速采样**。preflight 10/10、全树 **68/0**（代码 `a9176aa6`，全文＝[R1034](note/进度/v8/M5/26-10-29/R1034.md)）。
- ★R1033（2026-10-31）★＝**定位位姿交接**：新增 `v8.fw.pose_mux`，双路位姿切源要求目标新鲜且与最近输出差 ≤ `max_gap_m`，激活源跳变同样拒绝；真实交接切换处最大输出间隔 **52.6 ms**（20 Hz），跳变 4.95 m 被 `E_POSE_MUX_JUMP` 拒绝。pose_mux 3/3、preflight 9/9、全树 **68/0**（代码 `a7fbe38a`，全文＝[R1033](note/进度/v8/M5/26-10-29/R1033.md)）。
- ★R1032（2026-10-31）★＝**语义障碍接入规划**：`role=obstacle` 的 box 转规划器障碍 JSON，`graph_semantic_obstacle_planning` 同时接支撑面与障碍两消费者；障碍在场轨迹绕到带内最低 y=2.8，语义 v2 移除后 count 1→0、支撑图 v2 直行（带内 y=2.4）到达。obstacle 4/4、preflight 8/8、全树 **67/0**（代码 `16057c0a`，全文＝[R1032](note/进度/v8/M5/26-10-29/R1032.md)）。
- ★R1031（2026-10-31）★＝**地图包可迁移**：新增 `map_bundle`，导出 state＋地图文件＋sha256 清单，导入验哈希、复核快照并把绝对 `data_uri` 重写到新目录后重封；真实 `multi_map_node` 迁移恢复 map_a v3／map_b v2／mezz z=0.8。单测 4/4、全树 **66/0**（代码 `0caed595`，全文＝[R1031](note/进度/v8/M5/26-10-29/R1031.md)）。
- ★R1030（2026-10-31）★＝**边界 500 Hz 长跑**：回环 300 s 收发 **149 958** 帧、loss/dup/bad=0、tx/rx **499.9 Hz**、inter-arrival p99 **2.165 ms**、transport p99 **0.579 ms**、deadman stop **202.5 ms**（阈 250）。本机回环，不是真机链路；A1—A7 不变（零代码，全文＝[R1030](note/进度/v8/M5/26-10-29/R1030.md)）。
- ★R1029（2026-10-31）★＝**选配器组合级校验**：`selector.combination` 对整组替换一次执行图/装配/安全校验，CLI `--choose`／`--out-selected` 非法即 rc=5 且不写页面；合法组合 digest `8c9931d5…`，非法 2D/3D 组合返回 `E_GRAPH_DIMENSION`。test_select 5/5、全树 **65/0**（代码 `ec856a29`，全文＝[R1029](note/进度/v8/M5/26-10-29/R1029.md)）。
- ★R1028（2026-10-31）★＝**faster_lio 真实接入**：新增 `graph_faster_lio`，Assembly 真实启动 `10_slam_mapping.launch`；m1 下 `/Odometry` 23 帧、`/cloud_registered` 23 帧、5 s 新增 22 帧，末帧与 Gazebo 真值 XY 差约 0.028 m，停止退净。参数仍由 include 全局 rosparam 提供（未接实例级 YAML）；全树 **65/0**（代码 `7adc0cae`，全文＝[R1028](note/进度/v8/M5/26-10-29/R1028.md)）。
- ★R1027（2026-10-31）★＝**hdl 定位诊断（负结果）**：不 source 已构建工作区时 nodelet load 退出 255；source 后两 nodelet 在位、/globalmap 5 808 点、点云/IMU 与回调均通、参数已下发，但 lite3/m1 合成图、IMU 开关、DIRECT1/7、NDT 0.5/1.0/2.0 m 与临时重建全部 `odom_frames=0`；部分运行 -11，core RIP 在 `libhdl_localization_nodelet.so+0x487da`（Eigen PartialPivLU::compute）。未改第三方；全树 **65/0**（代码 `ba64afaf`，全文＝[R1027](note/进度/v8/M5/26-10-29/R1027.md)）。
- ★R1026（2026-10-31）★＝**语义地图接入规划**：新增 `semantic_support_bridge`（role=support_surface 的 box → map.v1 支撑面，契约校验）、ROS 节点、`graph_semantic_planning`；语义 v1→支撑图 v1→mezz z=0.6 arrived，语义 v2→支撑图 v2→z=0.8 arrived，changed=true。bridge 5/5、preflight 7/7、全树 **65/0**（代码 `1f4661b5`，全文＝[R1026](note/进度/v8/M5/26-10-29/R1026.md)）。
- ★R1025（2026-10-31）★＝**原生三维规划器入 v2 图**：新增 `v8.fw.local_native`、`graph_local_native` 与实例 YAML；运行面改由 `param_exec` 预载配置后再 exec `rosrun`，配置 `wall_watchdog_s=7.5` 真实生效、mezz z=0.6 arrived。F7 对无三维仲裁器的 Assembly 返回 `E_SAFETY_ADVICE`，未绕过。framework 29/29、runtime 13/13、preflight 6/6、全树 **64/0**（代码 `647428fa`，全文＝[R1025](note/进度/v8/M5/26-10-29/R1025.md)）。
- ★R1024（2026-10-31）★＝**地图更新驱动重规划到达**：冒烟补喂位姿/目标/障碍/
  定位心跳，goal 以闩锁一次性发布；v2 地图 mezz z=0.6 arrived，delta 抬高后 v3
  地图 z=0.8 arrived，两条轨迹不同。policy 13/13、全树 **64 套件 0 失败**；
  定位/障碍为冒烟注入，不声明物理可执行性（代码 `c439169f`，全文＝[R1024](note/进度/v8/M5/26-10-29/R1024.md)）。
- ★R1023（2026-10-31）★＝**规划器消费版本地图**：`ros_node` 新增 `~map_topic`
  （raw map 或 delta_map snapshot），地图更新清 result/递增 generation，status 暴露
  map_version；真实冒烟 delta_map v2→v3、planner status 2→3。policy 13/13、全树
  **64 套件 0 失败**；未喂 pose/goal，不声明到达（代码 `dd4b11d5`，全文＝[R1023](note/进度/v8/M5/26-10-29/R1023.md)）。
- ★R1022（2026-10-31）★＝**MapDelta 回接版本地图**：新增 `map_delta_relay`
  （typed MapDelta→JSON）与 `graph_map_cache_delta`；真实冒烟 cache ops→typed
  delta→relay→`delta_map_node`，地图 v2→v3、mezz z 0.8、停止退净；全树
  **64 套件 0 失败**。未接规划消费（代码 `99b5e948`，全文＝[R1022](note/进度/v8/M5/26-10-29/R1022.md)）。
- ★R1021（2026-10-31）★＝**A5 tick+phase 组合（负结果）**：planner tick 0.5 s +
  command phase 0.5 s 同时开启，seed0 三次 **3/3 FAIL**，均 segment_complete 停第二级
  z 0.1988；A5 仍实验性 **PARTIAL**，不再堆相位开关。零代码改动、全树
  **64 套件 0 失败**（全文＝[R1021](note/进度/v8/M5/26-10-29/R1021.md)）。
- ★R1020（2026-10-31）★＝**命令链安全 hold 门**：`hotswap.apply_instance` 增加
  pre_stop/post_start 回调；真实 mux 下旧源 0.4→hold 零速→停旧起新→释放后 0.3 m/s，
  停止退净。hotswap 12/12、全树 **64 套件 0 失败**。是安全交接、不是零中断
  （代码 `6baaf0eb`，全文＝[R1020](note/进度/v8/M5/26-10-29/R1020.md)）。
- ★R1019（2026-10-31）★＝**hdl 定位链路诊断（负结果）**：换 m1 同世界 PCD、
  补 base_link→rslidar TF、关 IMU 后，点云 187/IMU 3822 帧进入、globalmap 已收，
  `/odom` 仍 **0 帧**；同地图带 IMU 也 0 帧。停止退净、全树 **64 套件 0 失败**；
  hdl 真实定位仍未验证（代码 `6879c952`，全文＝[R1019](note/进度/v8/M5/26-10-29/R1019.md)）。
- ★R1018（2026-10-31）★＝**hdl 传感器喂入探针（负结果）**：Gazebo m1 点云 194 帧、
  IMU 3912 帧已进 hdl，两 nodelet 在位，但 45 s 内 `/odom` **0 帧**；m1 场景与
  lite3.pcd 不一致，NDT 未收敛/未发布。停止退净；V8 全树 **64 套件 0 失败**。
  不声明定位精度（代码 `942098da`，全文＝[R1018](note/进度/v8/M5/26-10-29/R1018.md)）。
- ★R1017（2026-10-31）★＝**hdl nodelet_group 真实装载**：`v8.fw.hdl_nodelet_group`
  用外部 manager 装载 `GlobalmapServerNodelet` + `HdlLocalizationNodelet`（lite3.pcd、
  NDT 参数），manager list 返回两 nodelet、停止退净；framework 29/29、registry 4/4、
  V8 全树 **64 套件 0 失败**。未喂点云/IMU，`/odom` 精度与真机未声明
  （代码 `3af52398`，全文＝[R1017](note/进度/v8/M5/26-10-29/R1017.md)）。
- ★R1016（2026-10-31）★＝**cache/merge 进 graph**：v8_msgs 增 `SupportMap`，
  `map_cache_node` 可把 base_map 整图转 pending ops；新增
  `v8.fw.map_cache_ros`/`map_merge_ros` 与 `graph_map_cache_ros`。Assembly 真实
  启动两 cache+merge，flush 后 merge 输出 ground/mezz 两面 delta，停止退净；
  framework 29/29、runtime 12/12、全树 **64 套件 0 失败**。旧 graph_3d_cache
  仍替身（代码 `b12ae982`，全文＝[R1016](note/进度/v8/M5/26-10-29/R1016.md)）。
- ★R1015（2026-10-31）★＝**A5 接触参数对照（负结果）**：profile B 三次 **0/3**
  （1 plan 后卡住/碰撞），profile C **2/3**（两次顶层、一次碰撞），均不优于默认 A
  的 2/3；首命令散差 B 0.32 s、C 0.38 s。不冻结 profile，A5 仍实验性 **PARTIAL**；
  本批零代码改动，全树 **64 套件 0 失败**（全文＝[R1015](note/进度/v8/M5/26-10-29/R1015.md)）。
- ★R1014（2026-10-31）★＝**v8_msgs 与真实 cache/merge 节点**：新增 `v8_msgs`
  （MapDelta/MapOp/SupportSurface/SurfaceEdge）；`map_cache_node` 接 JSON ops →
  typed MapDelta（flush/ack），`map_merge_node` 合并两路（同 base，后者覆盖）。
  独立工作区编译 rc=0，真实 ROS 冒烟 cache v1→v2 删除、merge 0.7→0.9、停止退净；
  V8 全树 **64 套件 0 失败**。图内 cache/merge 仍为替身（缺 SupportMap→ops 生产者）
  （代码 `f7bca16e`＋`c0295a14`，全文＝[R1014](note/进度/v8/M5/26-10-29/R1014.md)）。
- ★R1013（2026-10-31）★＝**A5 启动屏障实验（负结果）**：桥 `--require-release`
  在 goal/障碍/定位就绪后才放行 sim clock，runtime `--release-barrier` 调 release；
  seed0 三次 **2/3 PASS**，首个非零命令 0.36/0.42/0.88 s（散差 0.52 s，比 R990/R996
  更差），第三 run `E_LOCAL_COLLISION`。默认关闭，A5 维持实验性 **PARTIAL**；全树
  **64 套件 0 失败**（代码 `b3df5d5b`，全文＝[R1013](note/进度/v8/M5/26-10-29/R1013.md)）。
- ★R1012（2026-10-31）★＝**map_cache/merge 核心**：`MapCache` 按 surface_id
  聚合 upsert/remove，`make_delta` 生成合法 `map_delta/v1`（source sha=ops 摘要），
  `ack` 推进 base；`merge_deltas` 同 map_id/base 合并、后者覆盖前者。端到端接入
  `MapService` 升版（v1→v2→v3）；测试 4/4、V8 全树 **64 套件 0 失败**。
  ROS `v8_msgs` 消息层仍为替身（代码 `f4ed3b28`，全文＝[R1012](note/进度/v8/M5/26-10-29/R1012.md)）。
- ★R1011（2026-10-31）★＝**外部 nodelet manager**：manifest 新增
  `launch.kind=nodelet_group`（package=nodelet、manager、plugins）；预检校验宿主
  executable、插件包与 `plugin_classes`；渲染/运行生成 manager + 每插件一条
  `nodelet load ... --no-bond`，参数按插件名下发给 hosted nodelet。真实冒烟
  manager+Plus 参数 2.5、3.0→5.5、节点 `/v8_group/*`、停止退净；framework 28/28、
  runtime 11/11、V8 全树 **63 套件 0 失败**。remap 仍被 nodelet 忽略
  （代码 `7ca8e0a4`，全文＝[R1011](note/进度/v8/M5/26-10-29/R1011.md)）。
- ★R1010（2026-10-31）★＝**MuJoCo 验证后端**：新增 `v8.verify.mujoco_truth`，
  仓库内置最小 MJCF 落体世界，`mujoco_truth.py` 推进物理并发 `/v8/verify/truth_mj`；
  truth 不在算法端口、后端不进算法图。真实冒烟 4 帧 z 0.806→0.093，停止退净；
  verification 7/7、V8 全树 **63 套件 0 失败**。仅落体/空世界通道，未做场景评价
  （代码 `95bfb921`，全文＝[R1010](note/进度/v8/M5/26-10-29/R1010.md)）。
- ★R1009（2026-10-31）★＝**只读 provider 重叠替换**：`hotswap.apply_overlap`
  先启动替换进程，`ready_check`/READY 通过后再停旧；失败保留旧进程。`prepare`
  支持 `params` 覆盖，`runtime_name` 给替换节点独立 ROS 名；`ProcessRunner` env
  改为叠加父环境（修复状态交接丢 PATH）。真实语义 provider v1→v2、查询零失败、
  状态文件交接，hotswap 11/11、V8 全树 **63 套件 0 失败**。仅限无命令/无 TF
  实例，命令/定位链仍先停后启（代码 `7378c77d`，全文＝[R1009](note/进度/v8/M5/26-10-29/R1009.md)）。
- ★R1008（2026-10-31）★＝**语义 provider 入 v2 graph**：新增
  `v8.fw.semantic_provider`（node launch，只读、无命令/TF），`graph_semantic.yaml`
  经 check/plan/Assembly 启动，query/topic 返回 sem_a v1/1 要素，`command_owner=null`，
  停止退净；补 shebang 使 rosrun 可直接执行。framework 27/27、runtime 10/10、
  V8 全树 **63 套件 0 失败**（代码 `a8d40ac0`，全文＝[R1008](note/进度/v8/M5/26-10-29/R1008.md)）。
- ★R1007（2026-10-31）★＝**manifest 运行面补齐**：`default_command` 把
  `launch.remap` 以 `from:=to` 追加到 node/plugin 运行命令；包索引可选
  `plugin_classes` 参与预检，类不在册报 `E_LAUNCH_TYPE`。实测 `nodelet standalone`
  不消费 remap 参数（已登记边界）。runtime 9/9、V8 全树 **63 套件 0 失败**
  （代码 `6187a4e5`，全文＝[R1007](note/进度/v8/M5/26-10-29/R1007.md)）。
- ★R1006（2026-10-31）★＝**Gazebo 验证后端**：新增 `v8.verify.gazebo_truth`，
  include headless empty-world，`gazebo_truth.py` 订阅 `/gazebo/model_states` 发布
  `/v8/verify/truth`；truth 不在算法端口集合、后端不进算法图。真实后端经
  `verification.run` 启动并等到 3 帧 truth（world/ground_plane），停止退净；
  验证测试 6/6、V8 全树 **63 套件 0 失败**。仅空世界静态真值，未接 MuJoCo/场景评价
  （代码 `f93ef0af`，全文＝[R1006](note/进度/v8/M5/26-10-29/R1006.md)）。
- ★R1005（2026-10-31）★＝**plugin 托管**：manifest `launch.kind=plugin` 接
  `nodelet standalone`（宿主固定 `nodelet`，`type=pkg/Class`）；实例 id→`__name`、
  namespace→`__ns`、params→私有参数，预检查宿主/插件包/type 形。真实 ROS 冒烟
  Plus 插件 3.0+2.5→5.5，停止后退净；framework 26/26、runtime 8/8、V8 全树
  **63 套件 0 失败**。仅单进程 standalone，不支持外部 manager/bond
  （代码 `9426318e`，全文＝[R1005](note/进度/v8/M5/26-10-29/R1005.md)）。
- ★R1004（2026-10-31）★＝**语义地图独立契约**：新增 `semantic_map/v1`，四类几何
  point/line/polygon/box，校验 map_id/版本/frame/重复要素 id/几何；`SemanticCatalog`
  只读多图；`semantic_map_node` 逐图 latch `/v8/semantics/<id>`、状态与 Trigger 查询，
  重复 map_id 启动即拒绝。真实 ROS 双图冒烟通过；`semantic_map` 5/5、V8 全树
  **63 套件 0 失败**。语义层不并入 `map.v1`，无消费者/规划联动
  （代码 `b87ea556`，全文＝[R1004](note/进度/v8/M5/26-10-29/R1004.md)）。
- ★R1003（2026-10-31）★＝**2D grid 版本绑定**：`multi_map_node.grid_status` 带
  map_version/kind/frame_id，publish_all 刷新；map_c grid_status v1，三图版本 3/2/1
  重启保持。V8 全树 **62 套件 0 失败**；2D grid 仍只读，语义地图未实现
  （代码 `3192a4cb`，全文＝[R1003](note/进度/v8/M5/26-10-29/R1003.md)）。
- ★R1002（2026-10-31）★＝**2D 消费适配**：PGM（P5/P2）→`nav_msgs/OccupancyGrid`，
  multi_map_node 为 2D 图发 `/v8/maps/<id>/grid`；真实冒烟 map_c 100×100、occupied=20，
  三图版本独立/重启恢复。`occupancy_adapter` 3/3、V8 全树 **62 套件 0 失败**；语义地图/
  grid 版本联动未接（代码 `bc146f72`，全文＝[R1002](note/进度/v8/M5/26-10-29/R1002.md)）。
- ★R1001（2026-10-31）★＝**地图状态单写者锁**：`MapStore` 用 `fcntl.flock` 独占
  `state_dir/.map_store.lock`，`MapRouter(exclusive=True)`/`multi_map_node` 启动取锁、
  退出释放；第二个 provider 进程 `E_MAPSERVICE_LOCKED`。`map_router` 5/5、全树 **61 套件
  0 失败**；网络文件系统/跨机写者未覆盖（代码 `18d9f133`，全文＝[R1001](note/进度/v8/M5/26-10-29/R1001.md)）。
- ★R1000（2026-10-31）★＝**多图 schema 选择**：2D `occupancy_grid` 与 3D `support_map`
  同目录共存；`select(kind)` 可选，2D 只读（delta → `E_MAPSERVICE_UPDATE_SCHEMA`），
  3D 可更新；重载后 kind/version 保持。`map_router` 4/4、V8 全树 **61 套件 0 失败**；
  ROS 2D/语义消费适配未接（代码 `691c92d2`，全文＝[R1000](note/进度/v8/M5/26-10-29/R1000.md)）。
- ★R999（2026-10-31）★＝**多图 provider 并发串行化**：`MultiMapCore` 的 apply/snapshot/
  status 共享锁，防查询读到半更新；并发测试 3/3、真实多图冒烟仍通过、V8 全树 **61 套件
  0 失败**。锁只覆盖单节点，多进程写者仍待约束（代码 `df694fb2`，全文＝
  [R999](note/进度/v8/M5/26-10-29/R999.md)）。
- ★R998（2026-10-31）★＝**多图 ROS provider**：`multi_map_node` 按 map_id 路由 delta，
  逐图 latch `/v8/maps/<id>`、状态与 Trigger 查询；真实 ROS 冒烟 map_a v2→v3、map_b
  保持 v2，重启恢复、重放幂等。`multi_map_node` 2/2、V8 全树 **61 套件 0 失败**；
  仍本地多图、无跨图事务/跨机复制（代码 `e0a0d2a6`，全文＝[R998](note/进度/v8/M5/26-10-29/R998.md)）。
- ★R997（2026-10-31）★＝**MapRouter 多图路由**：MapDelta 按 map_id 分发到独立
  MapService；`map_a` v2 / `map_b` v1 独立、重载后保持、重放幂等；未知图/坏 JSON/
  重复图 fail-closed。`map_router` 3/3、V8 全树 **60 套件 0 失败**；ROS 多图 provider
  未接（代码 `e2fcf485`，全文＝[R997](note/进度/v8/M5/26-10-29/R997.md)）。
- ★R996（2026-10-31）★＝**A5 planner sim tick 绑定**：local planner 跨固定 sim tick
  才提交新规划；0.5 s tick 三次 **2/3 PASS**，首个非零命令散差降到 **0.10 s**，一次
  `segment_complete` 停 z .5446；未提升通过率，A5 仍 **PARTIAL**（代码 `1fe56f30`，全文＝
  [R996](note/进度/v8/M5/26-10-29/R996.md)）。
- ★R995（2026-10-31）★＝**A5 trajectory-resume 实验**：只在 trajectory/target 后恢复
  sim clock（默认关闭）→ seed0 **1/3 PASS**，首个非零命令 **0.20/0.38/0.60 s**（0.40 s 散差），
  两次停在 z .7175；桥侧三种尝试均不成立，A5 仍 **PARTIAL**（代码 `4e86439b`，全文＝
  [R995](note/进度/v8/M5/26-10-29/R995.md)）。
- ★R994（2026-10-31）★＝**A5 固定 sim 相位桶**：hold 2.0 s + 0.5 s 桶三次 **3/3 PASS**，
  但首个非零命令 2.28/2.34/2.56 s（散差 **0.28 s**）；桥侧采样仍不能锁 planner 更新相位。
  A5 仍 **PARTIAL**（代码 `6b013b09`，全文＝[R994](note/进度/v8/M5/26-10-29/R994.md)）。
- ★R993（2026-10-31）★＝**A5 相位门变体**：hold 2.0 s **3/3 PASS**（首个非零命令
  2.52/2.54/2.62 s，0.10 s 散差）；hold 1.0 s＋3 s 命令锁存反而 **1/3**（0.12 s 散差）。
  门/锁存不能替代 planner tick 绑定，A5 仍 **PARTIAL**（代码 `eb07f116`，全文＝
  [R993](note/进度/v8/M5/26-10-29/R993.md)）。
- ★R992（2026-10-31）★＝**A5 相位门 1.0 s**：门后 3/3 PASS、均到顶层，但首个非零命令
  仍在 1.16/1.32/1.20 s（0.16 s 散差）；门延长了起始零命令，未真正锁相。A5 仍 **PARTIAL**，
  不据此改判据（代码 `9cdef8c6`，全文＝[R992](note/进度/v8/M5/26-10-29/R992.md)）。
- ★R991（2026-10-31）★＝**边界 500 Hz 60 s 长时窗**：端点 life 随命令窗延长；500 Hz 请求
  连续 60 s 达 **500.4 Hz**、3036 帧 odom、位移 11.91 m、loss/bad/dup=0、停止无残留。仍为
  运动学端点，S5/真机 L2；A1—A7 不变（代码 `ae02a63b`，全文＝[R991](note/进度/v8/M5/26-10-29/R991.md)）。
- ★R990（2026-10-31）★＝**A5 分叉归因**：R987 FAIL/PASS 的首个 active applied 命令分别在
  sim 0.30/0.48 s（相位差 **0.18 s**）；同一 sim 2.08 s 的 body z 仅差 0.0002 m。说明先
  分叉的是命令到达仿真的相位，不是同一时刻策略输出；A5 仍 **PARTIAL**，下一件应做相位锁定
  重复窗而非继续累计自由调度 PASS/FAIL（代码载体 `0e15667d`，全文＝[R990](note/进度/v8/M5/26-10-29/R990.md)）。
- ★R989（2026-10-31）★＝**MapStore 多 map_id**：目录内按 map_id 原子快照、显式
  `load/ids/load_all`；`svc_map`/`other_map` 各自恢复 v2 与 digest，路径穿越与坏文件
  fail-closed；`map_service` 11/11、V8 全树 **59 套件 0 失败**。无 ROS/跨机路由，
  A1—A7 不变（代码 `0a9bb833`，全文＝[R989](note/进度/v8/M5/26-10-29/R989.md)）。
- ★R988（2026-10-31）★＝**边界 500 Hz 标定**：把 50 Hz 遥测的 3 个 ROS 发布移出命令环；
  200 Hz 请求达 **202.83 Hz**、500 Hz 请求达 **485.83 Hz**（97.2%），loss/bad/dup=0，
  位移 1.33/1.30 m，停止无残留。修复前两者都只有约 60 Hz。仍是运动学端点，S5/真机 L2；
  A1—A7 不变（代码 `82da2532`，全文＝[R988](note/进度/v8/M5/26-10-29/R988.md)）。
- ★R987（2026-10-31）★＝**A5 seed0 单线程复跑**：固定 OMP/MKL/OPENBLAS/NUMEXPR=1 后
  seed0 **2/3 PASS**（一次 4 plans 停在中间台阶，两次 9/8 plans 到顶层）；与 R986 合并
  仍分叉，数值库线程不是唯一来源；A5 仍 **PARTIAL**。无代码/判据改动，A1—A7 不变
  （代码 `b749e2df`，全文＝[R987](note/进度/v8/M5/26-10-29/R987.md)）。
- ★R986（2026-10-31）★＝**A5 seed0/seed2 复跑**：同配置 seed0 **1/2 PASS**（一次 8 plans
  到顶层，一次 1 plan 约 5.1 s 跌落），seed2 1/1 PASS；R973 的 seed0 FAIL 形态仍可复现，
  A5 仍 **PARTIAL**，不把单次 PASS 当收口。无代码/判据改动，A1—A7 不变（代码 `82872d42`，
  全文＝[R986](note/进度/v8/M5/26-10-29/R986.md)）。
- ★R985（2026-10-31）★＝**V8-X1 无 landing 多 seed**：4×0.15 m、5.5 s、v .3、seed0—4×3；
  clean **12/15**、stable **3/15**（仅 seed0）、fell 0、unstable 0；seed1/3/4 越过判据后
  未留顶层，seed2 xmax 2.59 < 2.70。未改 R977 判据/阈值；MuJoCo 单策略、无 ROS/真机，
  A1—A7 不变（代码载体 `515435ec`，全文＝[R985](note/进度/v8/M5/26-10-29/R985.md)）。
- ★R984（2026-10-31）★＝**边界适配器接主链**：真实 mux → `/cmd_vel` → `ros_bridge`
  （200 Hz 33B UDP）→ Gazebo 端点，遥测回 `/boundary/odom`；4 s 位移 0.936 m、odom 276 帧、
  loss/bad/dup=0、状态 active、停止无残留；`command_type=twist` 适配真实 mux。仍是运动学
  端点，非接触动力学/真机；A1—A7 不变。
- ★R983（2026-10-31）★＝MapService 快照持久化：`save/load` 原子写、恢复时重算
  digest 并核对 history 末行，损坏/异 map_id/回退拒绝；`delta_map_node` 增 `~state_path`，
  ROS 重启冒烟 v2→v3，node2 恢复 v3/幂等重放，再升 v4，applied_deltas=2；
  map_service 10/10、map_delta 8/8、delta_map 4/4、V8 全树 59 套件 0 失败；A1—A7 不变。
- ★R981（2026-10-31）★＝真实 2D 栈接入 v2 graph：`v8.nav2d.dwa_stack`/`teb_stack` +
  `v8.nav2d.mux` + 四张 graph，Assembly 真实启动 nav+mux；Gazebo m1/SC09/seed0 四格
  （GlobalPlanner/Navfn × DWA/TEB）4/4 PASS，参数服务器与话题信息确认实际 planner 和
  `/cmd_vel` 唯一写者。真实 launch 路径经 rospack 解析；ProcessRunner 停止时快照并清理
  roslaunch 的独立会话后代。A1—A7 仍 `NOT_VALIDATED`。

---

- 同步时刻＝2026-10-23 01:31 CST（整串从 `date` 取，不心算）。
- ★R1106（2026-11-02）★＝**A6 NDT 静态失锁重定位/租约门**：NDT `~relocalize`/`~initialpose`/`~relocalized`＋A6 bootstrap/候选门；Gazebo 静态最终 XY **.052642 m**、候选 **.057400 m**，lease 1→2，失锁期 gate `.1→0`；空扫描 FAILED、lease 2→2。运动候选停在 prior `.247788 m`，不作认证；NDT 仍 monitor-only，A6 PARTIAL（代码 `bb11009a`，全文＝[R1106](note/进度/v8/M5/26-10-29/R1106.md)）。
- ★R1107（2026-11-02）★＝**R1064 未复现＋状态看门狗**：同 lab 四例 5/5（62—65 s）；写 20 s status 看门狗，把就绪后 status 静默变为确定性错误；根因当时未定位（代码 `b1ac355d`，全文＝[R1107](note/进度/v8/M5/26-10-29/R1107.md)）。
- ★R1108（2026-11-02）★＝**A3 传感器图同 XY 选面**：`nist_sensor@2` 在 (1,−1.05) 命中 m1/m2；无 hint/z 显式 `E_SURFACE_AMBIGUOUS`，hint m2 规划走 m0→m1→m2、末 z `.371516`；该重叠是台阶邻接足迹，不是叠层楼板（代码 `17cdda62`，全文＝[R1108](note/进度/v8/M5/26-10-29/R1108.md)）。
- ★R1109（2026-11-02）★＝**Gazebo 载体 driveMode 实测更正**：URDF/插件实际 `dynamic`；beat cmd `.2→v_phy .199`、`.5→w_phy .481`，z 稳定 `.11`；当前是动态平面方盒＋接触，不是 kinematic pose sink，仍无四足步态（代码 `74740c05`，全文＝[R1109](note/进度/v8/M5/26-10-29/R1109.md)）。
- ★R1110（2026-11-02）★＝**A1 3D 第三轮 4 格**：flat/flat_alt/stairs_top/stairs_mid 4/4，最坏载体误差 **.097764 m**；与 R1099 的 8 格合并 **12/12**；仍 kinematic/声明图（代码 `9a12b37b`，全文＝[R1110](note/进度/v8/M5/26-10-29/R1110.md)）。
- ★R1111（2026-11-02）★＝**A1 3D 三种子起点扰动**：3 seed × 4 场景 **12/12**，起点 x/y ±.03 m，最坏 **.099712 m**；首轮两格负起点 argparse 缺陷已修（代码 `13d62dac`＋`1432df1f`，全文＝[R1111](note/进度/v8/M5/26-10-29/R1111.md)）。
- ★R1112（2026-11-02）★＝**3D 全局→V8-X1 四足接触执行 2/2**：support_planner 34 点、末 `(0.075,1.275,.890358)`、五级过渡；checks 全真、final map `(0.075,1.275,.8904)`、body z `1.0998/1.1010`、plans `8/7`；仍单策略/声明图/预计算全局（代码 `b9ebca9f`，全文＝[R1112](note/进度/v8/M5/26-10-29/R1112.md)）。
- ★R1113（2026-11-02）★＝**R1064 根因：reap 关闭被 reader 锁住的 stdout**：SIGUSR1 栈定位 `stop_owned→reap→proc.stdout.close()`；修复后同 lab 5/5、20/20 case VALID；加唯一 local 名/独立 Gazebo master/faulthandler（代码 `396709ce`＋`8ab300bc`，全文＝[R1113](note/进度/v8/M5/26-10-29/R1113.md)）。
- ★R1114（2026-11-02）★＝**A6 lease 晚订阅竞态＋0.3 m 局部劫持**：首次 gate 侧 `last_lease=0` 而 A6 `lease=1`，冒烟在 gate lease 1 超时；`/lease` 改 latched 后静态 **.048655 / .044988 m**、0.3 m 两跑 **.043754 / .060690 m**，候选仍停在 prior **.242—.250 m**（靠 NDT 恢复后跟踪回正），空扫描 FAILED、lease 2→2。仍是短时局部失锁，非任意 kidnapping/全图重定位、无四足/真机（代码 `5f028d86`，全文＝[R1114](note/进度/v8/M5/26-10-29/R1114.md)）。
- ★R1115（2026-11-02）★＝**BBS 绝对源共识门闭合 A6 0.6 m fail-open**：新增 `consensus_policy/gate`，NDT 候选与 3 分辨率 BBS 共识在 0.5 m 内才放行并输出 BBS 位姿；A6 可要求 `source=bbs_consensus` 且 `agreement<=0.5`。活体 m1：静态 **.057751 m**（共识 [1.5,0]）、0.3 m **.053147 m**（共识 [1.9,0]）；0.6 m 共识 [2.2,0] 与 NDT 候选差 **.647609** → A6 **FAILED**、lease 保持 1、无 reset。BBS 非真实第二传感器、0.6 m 不恢复；生产图需显式接入门；A6 仍 PARTIAL、无四足/真机（代码 `26eeb88d`，全文＝[R1115](note/进度/v8/M5/26-10-29/R1115.md)）。
- ★R1116—R1119（2026-11-02）★＝**交互与建图链**：R1116 2D/3D 交互入口；R1117 补 `world→map` 静态 TF 修 3D RViz 空白；R1118 修 `Marker Topic` 并补支撑面/Lite3 模型（2D 开 `/map`）；R1119 键盘建图（GT 位姿）→ PCD 72,762 点 → 单面 support_map → `app_runner` 交互导航，点击 `(3,1)` 得 31 点路径、载体 `(3.05,1.05)`、local arrived。仿真 GT 建图/运动学载体，不是 LIO/SLAM/真机/formal（代码 `974e5997`＋`6a4f2ece`＋`94e59972`＋`626f21d6`，全文＝[R1119](note/进度/v8/M5/26-10-29/R1119.md)）。
- ★R1120（2026-11-02）★＝**导入 NIST 世界：建图可、楼梯导航未闭环**：`m2_gazebo` 增加 world/model_path/spawn 参数，键盘采点 324,493 点（z 0—0.525），RViz 点云可见；`map_from_pcd --nist-profile` 仍出重叠碎面，自建图 `E_LOCAL_CLEARANCE`，声明图 `E_LOCAL_NO_ROUTE`；平层 m1 的 R1119 仍是端到端。下一步需模型感知地面/踏步分割与局部踏步执行（代码 `3983cb45`，全文＝[R1120](note/进度/v8/M5/26-10-29/R1120.md)）。
- ★R1121（2026-11-02）★＝**打通导入 NIST 楼梯的 3D 导航**：局部规划器把声明 step/slope 的下一踏面当可跨越边界（仅靠边、机器人尺寸可容纳；真低顶仍拒绝），检查器同步；`inflate_support_map --margin .20` 生成 `nist_stairs_120_nav@1`，消除台阶交叠死带；app 局部配置 tick 1.2 s、max_step .30 m、watchdog 90 s。交互点 step5 后全局 **39 点**、载体 `(0.075,1.275,0.890358)`、局部 `arrived`、六项 checks 全 true、到局部目标 **1.26e-6 m**。仍是 Gazebo 运动学箱体 + 声明图外扩，不是四足物理/真机/formal；R1120 的键盘建图边界不回改（代码 `c37a59a1`，全文＝[R1121](note/进度/v8/M5/26-10-29/R1121.md)）。
- ★R1122（2026-11-03）★＝**演示线 v0：纯 SLAM 流程打通**：独立 11413/11445 端口，`m1_world_v2` + faster-lio vs GT，20 s / 0.25 m/s；pairs 10，translated xy mean **9.504 mm**、max 22.660 mm，rigid yaw mean 5.278 mrad；只测 SLAM/定位，不跑导航/运动学；单次 scoped、非 formal，rich 多层 world 未导入；faster-lio PCD 已备份/恢复（代码 `8aba8dbc`，全文＝[R1122](note/进度/v8/M5/26-11-03/R1122.md)）。
- ★R1123（2026-11-03）★＝**演示线 v1：受控两层 Gazebo SLAM survey**：新增 `demo_two_floor.world`（12×8 m 底层、1.6 m 上层、10×0.16 m 楼梯、墙体/家具）与 `demo_slam_survey.py`；`set_model_state` GT survey 35 s，不跑导航/运动学。恒定 yaw 版 translated xy mean **248.7 mm**、rigid yaw **2.4 mrad**；首轮 yaw 突变版 409.9 mm／2.97 rad 为负例；LIO PCD 留证，原 `lite3.pcd` 已恢复；仅 SLAM scoped、非 formal，rich 公开多层 world 未导入（代码 `48fbf5d7`，全文＝[R1123](note/进度/v8/M5/26-11-03/R1123.md)）。

## 七、数据与校验口径（2026-11-04）

- **sha256 只给数据面/交付面**：pcd、地图、实验结果、交付包、跨机拷贝需要 `sha256`（git 管不到、传输可能截断）；**入 git 的源码/文档以 git 内容寻址为准，不单列 sha**。
- **地图各机独立**：`data/maps/*` 不入库；仓内只留 `data/maps/registry.template.json`；真机用 `configs/lite3/lite3_real_3d.local.app.yaml`（`*.local.app.yaml` 已忽略）；共享仅走 `data/maps/shared/`（`*.pcd`／`*.tar.gz`／`*.bag` 永不入）。
- **历史件变更（另起句）**：`install.sh` 与仓根 `.bashrc` 于 2026-11-04 **移出仓库**（原件与 sha 留底 `/home/ysc/deleted_20261104/`）。⇒ R579「`install.sh` 保留作历史入口、未删未改」那句**自此作历史读**（原句不回改）。
- **R862/R863 大结果**：`results-initial.json`、`results-rerun6.json.gz` 及三枚 `*.sha256` 改为**本机数据面**（不入 git；盘上保留）。
