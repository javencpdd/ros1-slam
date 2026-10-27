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
**当前状态：计划已落册，平台改造尚未实施；现有代码不等于已具备全部目标能力。**
具体任务见 [执行队列](note/计划书v8/23_v8任务队列.md)，最新批次见 [进度入口](note/进度/README.md)。

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
| `web/` | 项目官网（静态单页）：架构、里程碑、判据现状、仿真入口 | `web/README.md` |
| `tooling/` | 项目自持清单（含 skill 逐字镜像＋ `*_sha256_manifest.txt` 见证清单） | `tooling/README.md` |
| `data/` | 底图与输出面（`maps/`＝建图三件套 .pcd/.pgm/.yaml，`waypoints/`＝航点 JSON） | `data/waypoints/README.md` |
| `opt/` | **第三方与原件区**（★原样复制、一字不改★；唯一许可例外＝faster-lio 存盘路径隔离） | `opt/upstream/`／`opt/3rdparty/` |
| `.ai/` | 跨工具接力黑板的状态目录（`bridge.py`＝唯一状态写入者） | `python3 .ai/bridge.py resume --as <工具名>` |
| `.qoder/rules/` | qoder 侧常驻提醒（带 `alwaysApply`） | `ros1-slam-collab.md` |

⚠️ `logs/` 与 `data/maps/` 下的产物**不入库**（`.gitignore` 已排除），换机只拿代码与台账即可重建。

---

## 三·五、官网介绍（`/web`）

仓库自带一份**静态单页**项目官网（零依赖、可离线打开）：定位、能力地图、架构、里程碑、
当前判据状态、仿真与观察入口、新算法接入方式、文档索引。

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

---

- 同步时刻＝2026-10-23 01:31 CST（整串从 `date` 取，不心算）。
