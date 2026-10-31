# ros1-slam · 四足机器人 SLAM 与导航开发仓

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
**当前状态：框架改造 F0—F8 的机制层已落册（图、预检、装配、MapDelta、安全、热替换），
真实 2D 栈已由 v2 graph + Assembly 启动并四格闭环（R981）；定位已有 v2 清单与两张同链图（R982），但真实 hdl launch 未验；3D/其他算法仍未 v2 化或未真实 launch。
现有代码不等于已具备全部目标能力。**
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
| **已实现（机制/假进程/回环）** | graph YAML、manifest v2、端口/边/DAG/命令唯一、实例 `config` 预检、2D/3D Path 与地图维度检查、Assembly/ProcessRunner、MapService/MapDelta + ROS provider、MapService 快照持久化（R983）、MapStore 多 map_id（R989）、边界适配器主链（R984）、语义地图独立契约与只读 provider（R1004）、plugin（nodelet）托管（R1005）、Gazebo 验证后端（R1006）、launch remap 生效与 pluginlib 类校验（R1007）、语义 provider 入 v2 graph（R1008）、只读 provider 重叠替换（R1009）、MuJoCo 验证后端（R1010）、外部 nodelet manager（R1011）、map_cache/merge 核心（R1012）、v8_msgs 与真实 cache/merge 节点（R1014）、cache/merge 进 graph/Assembly（R1016）、hdl nodelet_group 真实装载（R1017）、命令链安全 hold 门（R1020）、MapDelta 回接版本地图（R1022）、规划器消费版本地图并随更新重规划到达（R1023/R1024）、原生三维规划器入 v2 图与实例配置真实下发（R1025）、语义地图转支撑面并驱动规划（R1026）、robot profile、verification 配置、safety、selector、hotswap v0.3、boundary S1—S4、**2D 真实栈 v2 graph 装配（R981，4/4）** |
| **未实现/未接** | 语义地图的障碍/标签消费者（支撑面消费者已 R1026）、定位/3D/其他算法的真实 launch（DWA/TEB/Navfn/GlobalPlanner 已通过 2D 栈接入；hdl 已用 nodelet_group 真实装载两 nodelet（R1017）；R1027 确认需 source 已构建工作区、/globalmap 与点云回调可达，但 NDT 配准无输出、部分运行在 Eigen LU 处 -11，`/odom` 仍 0（R1018/R1027），传感器输入下的定位、`/leg_odom2` 与 M2-R2C 未验证；faster_lio 仍 include；LIO/3D 未 v2 化）、M20/真机 S5、旧 graph_3d_cache 内 cache/merge 仍替身（新 graph_map_cache_ros 为真实）、命令/定位链零中断热替换（命令链安全 hold 已 R1020）、#151 停车预算 |
| **开发中** | V8-X1 0.6 m 与阶沿鲁棒性、A5 整链、边界 500 Hz 满速/长时标定、MapDelta 多 map_id/跨机持久化、selector 组合级校验；2D 后端四格 4/4，v2 graph 装配 4/4（R980/R981） |

框架机制说明见 [framework/README.md](ws/src/90_platform/v8_platform/framework/README.md)、
[架构导读](docs/architecture/framework-v0.md) 和 [图形化选配页示例](docs/architecture/framework-selector.html)。
**边界**：A1—A7 仍 `NOT_VALIDATED`；R982 只证明定位组件可在图中替换并生成装配计划，hdl 真实 launch 仍未执行（`/leg_odom2`、M2-R2C、GT 辅助 probe 不计独立运行）。

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
| `scripts/` | 门禁件 `scripts/verify/`（★十一件★）＋采图脚本族 `scripts/mapping/` | `scripts/mapping/README.md` |
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
