# 项目介绍网页

> 最新更新时间：2026-11-10 04:30 CST。本次新增 P2/P3 独立页 `console.html`（地图库/任务台/监控/遥控，经 rosbridge 连 ROS1）与 `scripts/ops/build_maps_index.py`；09「3D 仿真测试盘面」由 `scripts/ops/collect_3d_runs.py` 生成：`scripts/ops/collect_3d_runs.py` 把 R1127、EVT-PREVIEW、R1122/R1125/R1126 与 R980/R981/R866 的证据 JSON 归一成 `assets/data/3d_runs.json`，页面按类型/批次/结果筛选并链接证据；当前 4/4 与历史窗口、SLAM、2D MBF 分行不混算。05 MuJoCo 子页此前新增「Lite3 AI 策略交互演示（宿主单机）」卡：12 个厂商 ONNX、50 m 楼梯（25 层×3 级）与 50 m 坡面（15—50°）、键盘控制、CSV 参考回放与边界说明，并附宿主启动器 `assets/demo/start_lite3_mujoco_demo.sh`。此前更新为 two_floor_nav 自动用例 4/4、候选世界表去重与演示线读数口径；复核后修正验收自述与演示线口径，把 09 演示线并入 03 作为 B 区（01—09 共 9 个仿真子页，A/B 两区以硬标签隔离口径）；再把 hero / nav3d / 08 边界的重复状态叙述收敛到 Status 权威盘面，05 MuJoCo 参数表真删减（改为指向源证据的短引用；启动命令块与口径 notice 保留）。10 个主板块与各仿真子页均标更新时间，历史批次保留“历史”标注；03 的「世界 / 场景」说明改写为「舞台 / 布景」两层口径，讲清同一个 stair_lab 文件的两种用法、短名与文件名的区别，以及静态地图不随热插拔更新；06「场景」子页「热切换 3D 场景」卡补写 scene 的用途与时机，并同步 ws 包 README 与 sim_scene.py 头注释（真人授权扩 scope）。

> 2026-11-10：页面/console 源码与静态展示图入库；`assets/data/`、`assets/maps/` 是生成物，仍由生成器产出并忽略。`web/` 不进入 `lite3`/M20 镜像。

> 2026-11-10 23:40：控制台 3D 多目标 mission 的到点判据改为消费 `/v8/bt/status` 的 BT 编排回执（`event`／`current_seq`，按序号边沿去重），可视化载体位姿只用于路径与误差统计；2D 模式仍用原位姿判到达。判定逻辑落在 `assets/console/mission_events.js`（纯函数、无 DOM），`scripts/console/test_mission_events.js` 覆盖主测／注入有效性／反控／重复边沿／接线断言，并挂进 `scripts/console/check_console.sh`。

> 2026-11-10 23:50：浏览器活体实测（Chrome CDP ＋ rosbridge ＋ 重启后的 Nav2 式 BT 栈，发两个目标）发现 `event` 会被 `dwell`／`stop` 覆盖，2 Hz 订阅永远采不到 `reached`；BT 侧新增持久 `outcome`／`outcome_seq`，控制台 `mission_events.js` 优先读持久结果、`event` 作兼容回退。阶段 A 只注入位姿不推进、阶段 B 注入路径回执后 `任务目标 2/2 发送`。完整读数见 `note/进度/v8/M5/26-11-10/BT回执活体实测.md`。

[`index.html`](index.html) 是无构建步骤的静态项目介绍页，使用本目录 `assets/` 的图片/GIF/SVG。顶部导航以 hash 切换 `main .page`，可直接分享 `/web/#status`、`/web/#nav3d` 等地址。页面只展示资料，不连接 ROS 或控制机器人。

多层 Gazebo 交互式 3D 导航测试流程位于 `index.html` 的 `#two-floor-interactive`（从 `#sim` 的交互导航标签进入）。内容覆盖离线自检、唯一窗口登记、原预检、RViz Publish Point、停止退净、结果判读及当前 4/4 自动用例边界；命令真源仍以仓内场景 README 为准。

## 宿主 MuJoCo AI 策略演示

[`assets/demo/start_lite3_mujoco_demo.sh`](assets/demo/start_lite3_mujoco_demo.sh) 是宿主侧启动器：不依赖 Docker/ROS，资产默认在仓外 `~/.cache/lite3-policy-staging-20261109`，venv 默认 `~/.cache/mujoco-demo-venv`，可用 `LITE3_DEMO_ROOT`、`LITE3_DEMO_PY`、`LITE3_DEMO_DISPLAY` 覆盖。入口在 05 MuJoCo 子页「Lite3 AI 策略交互演示」卡；用法、依赖与能力边界见 [`assets/demo/README.md`](assets/demo/README.md)。它不是 Gazebo 官方运行面，也不替代 formal 判据或真机。

全站已于 2026-11-08 同步到 v8.1 现行口径：当前结构见[项目架构](../note/架构/项目架构与目录.md)，验收与版本边界见[进度入口](../note/进度/README.md)，当前授权选件与新旧编号映射见[23号 v8.1 任务队列](../note/计划书v8.1/23_v8.1任务队列.md)，工程规则见[工程标准](../note/架构/工程标准.md)。各主板块和仿真子页均标注更新日期；带“历史”的数字不扩 scope，也不把 R1126、R1127 四窗或 two_floor_nav 演示结果视为 formal、四足物理或真机验收。

## 3D 测试盘面

`index.html` 的 09「3D 仿真测试盘面」读取 [`assets/data/3d_runs.json`](assets/data/3d_runs.json)。该文件由仓库根目录执行以下命令生成（只读证据，不跑 Gazebo）：

```bash
python3 scripts/ops/collect_3d_runs.py
```

数据源：R1127 `two_floor` 四窗与历史窗口、EVT-PREVIEW 预演、R1122/R1125/R1126 纯 SLAM 演示线、R980/R981/R866 的 2D MBF driver.json。缺项在页面显示“—”，不做推算；当前 4/4 只统计 two_floor 四窗，历史失败、SLAM 和 2D MBF 均分行列出、不并入 3D 导航口径。

## 3D 任务台（P2/P3 + P4 + 3D 场景扩展）

[`console.html`](console.html) 是独立交互页，经 rosbridge（默认 `ws://<host>:9090`）连接 ROS1。地图库由
`scripts/ops/build_maps_index.py` 从 `data/maps/` 与 `two_floor_nav` 支撑图生成（2D 栅格/PCD 顶视图 + 下采样 3D 点云样本）；
实体世界由 `scripts/ops/build_world_scenes.py` 解析 `ws/src` 下的 Gazebo SDF，输出 box/DAE 世界场景：

```bash
python3 scripts/ops/build_maps_index.py
python3 scripts/ops/build_world_scenes.py
```

- **地图库/任务**：读取 [`assets/data/maps.json`](assets/data/maps.json)；2D 栅格/点云顶视图 + 3D 点云预览；可订阅实时 `/cloud_registered`（faster-lio，0.5 s 节流）或 `/rslidar_points`；叠加路径/TF 位姿/MBF 状态/costmap；多目标状态机按到点或 MBF SUCCEEDED 推进并记录逐目标耗时/路径/终点误差；地图守卫校验 `/map`；2D 发 `/move_base_simple/goal` + `/initialpose`；3D 目标发 `/v8/bt/goal`（BT→goal_gate→`/v8/3d/global/goal`），起点发 `/v8/3d/global/start`；2D 可 cancel，3D 无独立 cancel（用急停/作业单）。
- **3D 场景**：加载 `assets/data/worlds/*.json` 的实体 box（可含 DAE 三角网格），在同一个 canvas 里切换“点云 / 实体 / 叠加”；实体来自 SDF 精确几何（m1 8 box、stair_lab 195 box、NIST 60 面），未解析的 mesh/模型库 include 在生成统计里列出，不在前端猜测。
- **World registry**：`scripts/console/world_registry.json` 是 world 文件、SHA256、场景标签、地图绑定和可启动栈绑定的唯一主源；`build_world_scenes.py`/`build_maps_index.py` 从它生成 `worlds.json` 与 `maps.json` 的 `world_id`，`test_world_registry.py` 校验三面一致。不要再在手写标签里判断 world 身份。
- **视图口径**：主画布始终是所选地图的 2D 俯视预览，用来点选目标；“3D 全链”只切换目标发布接口（控制台发 `/v8/bt/goal`，由 BT/goal_gate 落到 `/v8/3d/global/goal`），不会把主图变成 3D。点云/实体世界用“打开 3D 场景”按钮查看；`mapped_m1` 等点云图的主图仍是 2D 投影。
- **远程指南**：控制台内“远程使用指南”给出连接、起栈、选图/发目标、3D 场景、监控/遥控的步骤；本机桌面项（Gazebo/RViz/MuJoCo/上传/shell）在远程灰化锁定。
- **服务端体素**：`voxel_stream` 白名单栈在容器内订阅 `/cloud_registered`（优先）与 `/rslidar_points`，按 `leaf=0.10 m` 体素下采样，原子写 `data/console/voxels/latest.bin` + `meta.json`；agent 提供 `/api/voxels/meta` 与 `/api/voxels/latest`，浏览器切到“服务端体素”后按二进制拉取（16 B/点，含 RGBA），不再在浏览器端从 20k 原始点里抽稀。原始 `/rslidar_points` 是传感器系，与实体叠加只作示意；`/cloud_registered` 的 `camera_init` 也按非 map 帧提示。
- **行为树（BT.CPP/Groot2）**：`lite3_bt` 包使用 BehaviorTree.CPP 3.8.8，单目标树 `WaitForGoal → SendGoal → WaitArrival → Stop`；作业栈启 `bt_console`（BT + goal_gate），3D goal 写入权归该栈：地图卡“发送选中/按顺序发送全部/交给行为树”统一发 `/v8/bt/goal`，BT 未运行时 3D 发送被拒绝；状态发 `/v8/bt/status`，ZMQ 5555/5556 供本机 Groot2 1.9 监控。它是新增编排层，不替代 MBF/3D 规划器；正式/真机不启用。构建与运行见 [`../ws/src/90_platform/lite3_bt/README.md`](../ws/src/90_platform/lite3_bt/README.md)。
- **监控**：rosbridge/rosapi 状态、`/cmd_vel`、`/v8/3d/global/path`、`/v8/local/trajectory`、`/v8/local/pose`、`/tf` 频率与 TF 对、节点/话题列表。
- **遥控**：遥控面板放在地图卡内、2D 主图正下方和 3D 场景上方，与画面同列；只向安全 mux 的 `/lite3/teleop/app_cmd_vel` 发 Twist（deadman 0.5 s）；需要先启 `safety_mux` 栈，mux 以 20 Hz 仲裁到 `/cmd_vel`。demo 用 `console_demo_mux.launch`，线性上限 1.0 m/s；正式/真机仍用 `60_safety.launch`（0.5 m/s、无超声 blind 0.1 m/s）。急停持续向 `/lite3/safety/estop_cmd_vel` 发零速并取消 2D 目标；**不直接写 `/cmd_vel`**。
- **地图生命周期**：刷新/重栅格化（PCD→2D 栅格）/隐藏/恢复/编辑栅格；`mapping_save` 白名单栈停止时保存 PCD 到 `data/maps/imported/`（agent 起前备份、停后恢复样张，GAP-5）；导入点云放 `data/maps/imported/` 后刷新。
- **作业栈**：宿主 agent（`bash scripts/console/start_agent.sh`）按 `scripts/console/stacks.json` 白名单启停；停 world 栈会按依赖逆序自动级联停止依赖栈，`stack_state` 同时校验 pid 组与 `pid_match` 命令身份，避免 PID 复用误报。 `sim_m1` / `mapping_m1` / `mapping_save` / `stairstudy` / `sim_nist` / `safety_mux` / `bt_console` / `voxel_stream`；每栈独立进程组、SIGINT→TERM→KILL、日志、world 元数据；formal/EVT/真机仍走作业单。
- **Gazebo world 选择**：`sim_m1` 下拉可选 `m1_world` / `m1_world_v2` / `demo_two_floor` / `open`；`stairstudy` 下拉可选 9 个 `stair_lab_*` world；NIST 用 `sim_nist`（仅世界，不含机器人/点云/导航）。`mapping_m1`/`mapping_save`/`voxel_stream` 依赖“提供 lidar 的 world 栈”，可用 sim_m1 或 stairstudy；sim_nist 不提供 lidar，会被拒绝。同一时间只起一个 Gazebo world，换 world 先停当前栈。`demo_two_floor.world` 是两层 SLAM demo（夹层 1.5 m + 10 级 0.3 m 踏步），不是 `two_floor_nav` 的 `open/blocked` 已知地图导航 app；后者入口是 `scripts/sim/two_floor_nav.py --execute ...`，不在网页白名单。`two_floor_nav` 支撑图对应 `open.world`（10 级 0.8 m 踏步、上层 x=-8…-4），控制台 3D 场景会自动选 `open`（23 box），不要选 `demo_two_floor`（30 box）；两者不互认。
- **工程**：`scripts/console/check_console.sh` 自检；`scripts/ops/test_collect_3d_runs.py`、`scripts/ops/test_world_scenes.py`、`scripts/ops/test_world_registry.py`、`scripts/console/test_agent_contract.py`、`scripts/console/test_voxel_stream.py` 单测；`assets/console/config.json` 可覆盖 rosbridge/agent/期望频率；09 有汇总 mean±std 与运行/汇总 CSV。

边界：任务台只启停白名单 demo 栈，不启停 formal/EVT/真机，也不代批作业窗；Gazebo/导航正式启动仍走 `docker/` 作业单与 `runtime_lab.py`。

## 本机浏览

在仓库根执行：

```bash
python3 web/serve.py
# 浏览器：http://127.0.0.1:8000/web/
```

[`serve.py`](serve.py) 将服务根设为仓库根，给常见文本类型补 UTF-8，给文本响应加 `Cache-Control: no-cache`，并将 `/` 和 `/index.html` 跳转到 `/web/`。`--port` 改端口；`--bind` 默认 `127.0.0.1`，需要局域网访问时显式选择：

```bash
python3 web/serve.py --port 8080 --bind 0.0.0.0
```

这会让整个仓库的可读文件通过同一静态服务可访问。只服务 `web/` 可用 `python3 -m http.server --directory web 8000`，但页面的 `../note/`、`../docker/` 等仓内链接将不可达。直接打开 `index.html` 可看页面和本地资源；跨目录文件链接是否可用取决于浏览器策略，不能保证。

### 局域网访问控制台

从另一台机器打开 `http://<host>/web/console.html` 时，web 80 与 rosbridge 9090 已监听 `0.0.0.0`；agent 启动器默认监听 `0.0.0.0:8780`，控制台会按页面主机名自动填 agent 地址。只应在可信局域网使用：agent 虽只能启停 `scripts/console/stacks.json` 白名单 demo 栈，但没有额外鉴权。若只要本机访问，启动时设：

```bash
LITE3_AGENT_BIND=127.0.0.1 bash scripts/console/start_agent.sh --start
```

页面每 4 s 自动重试 `/api/stacks`；agent 重启后通常无需刷新页面。若仍显示未连接，先确认 `ss -ltnp | grep 8780` 显示 `0.0.0.0:8780`，再从客户端执行 `curl http://<host>:8780/api/health`。

控制台会请求 agent 的 `/api/whoami` 判定操作端：回环地址、宿主自身 IP、以及 `LITE3_LOCAL_CIDRS`（默认仅 `127.0.0.0/8,::1/128`）内的来源视为本机，其余为远程。远程端会把“本机桌面”卡片灰化锁定；`stacks.json` 中带 `local_only: true` 的栈在 agent 侧也会对远程返回 403，不只靠前端置灰。若 VPN/代理网段需要算本机，显式设置 `LITE3_LOCAL_CIDRS`，不要放宽默认值。

## Docker 发布

[`publish.sh`](publish.sh) 将仓库只读挂到 `/srv`，使用现成镜像中的 Python 运行相同服务：

```bash
bash web/publish.sh 8080
# 浏览器：http://<本机IP>:8080/
```

省略参数时宿主端口为 `80`；可用 `NAME`、`IMAGE` 环境变量覆盖默认容器名 `ros1-slam-web` 和镜像 `ros1-slam:noetic-sim`。脚本会先删除同名容器再新建，执行前确认它是本次可替换的站点容器。它不构建镜像，也不配置防火墙；图片和页面修改后刷新即可生效。

```bash
docker logs -f ros1-slam-web
docker stop ros1-slam-web
docker rm ros1-slam-web
```

本轮未执行发布或新建服务：站点由既有的 `ros1-slam-web` 容器只读挂仓提供（宿主 80 → 容器 8000），页面改动刷新即生效；已在浏览器核对 03（含演示线）、Status，2026-11-09 复核时 HTTP 返回内容与本地 `index.html` 的 sha256 一致。`web/` 当前为本机忽略目录；跨机缺少该目录时应从明确的页面交付包恢复，不据此判断源码或算法缺失。

## 维护

状态更新须同时核对 `note/` 原始记录与 `calibration/calibration.py` 输出；只有在具备相应环境并实际复核后，才更新页面数字、快照日期和 digest。说明文档日期更新不代表网页内容已重新校准。操作命令集中维护在 [Docker 指南](../docker/README.md)；新增图片保持本地资源，不引入追踪脚本。
