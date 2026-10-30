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
| **已实现（机制/假进程/回环）** | graph YAML、manifest v2、端口/边/DAG/命令唯一、实例 `config` 预检、2D/3D Path 与地图维度检查、Assembly/ProcessRunner、MapService/MapDelta + ROS provider、MapService 快照持久化（R983）、MapStore 多 map_id（R989）、边界适配器主链（R984）、robot profile、verification 配置、safety、selector、hotswap v0.3、boundary S1—S4、**2D 真实栈 v2 graph 装配（R981，4/4）** |
| **未实现/未接** | 定位/3D/其他算法的真实 launch（DWA/TEB/Navfn/GlobalPlanner 已通过 2D 栈接入；hdl 已有 v2 清单/图选择但真实启动受 `/leg_odom2` 与 M2-R2C 限制；faster_lio 仍 include；LIO/3D 未 v2 化）、plugin 托管、M20/真机 S5、语义地图、真实 map_cache/map_merge、MuJoCo/Gazebo 验证后端、零中断热替换、#151 停车预算 |
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
