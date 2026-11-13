# 项目介绍网页（纯文档面）

> 2026-11-13 拆分：官网只留文档与静态展示；3D 任务台迁至 `scripts/console/ui/`，由宿主
> agent 在 `:8780` 独立提供（见 [`scripts/console/README.md`](../scripts/console/README.md)）。
> 静态服务改为文档白名单：`web/`、`docker/`、`tooling/`、根 `README.md`、`ws/src/**/*.md`；
> `note/`、源码、任务台数据一律 404。页面内指向 `../note/` 的深链已删。
>
> 2026-11-10：页面源码与静态展示图入库；`assets/data/` 是生成物，仍由生成器产出并忽略。
> 05 MuJoCo 子页新增「Lite3 AI 策略交互演示」卡（12 个厂商 ONNX、50 m 楼梯/坡面、键盘控制、
> CSV 参考回放）；09「3D 仿真测试盘面」由 `scripts/ops/collect_3d_runs.py` 生成
> `assets/data/3d_runs.json`，按类型/批次/结果筛选并链接证据，SLAM/2D MBF 分行不混算。

[`index.html`](index.html) 是无构建步骤的静态项目介绍页，使用本目录 `assets/` 的图片/GIF/SVG。顶部导航以 hash 切换 `main .page`，可直接分享 `/web/#status`、`/web/#nav3d` 等地址。页面只展示资料，不连接 ROS 或控制机器人。

多层 Gazebo 交互式 3D 导航测试流程位于 `index.html` 的 `#two-floor-interactive`（从 `#sim` 的交互导航标签进入）。内容覆盖离线自检、唯一窗口登记、原预检、RViz Publish Point、停止退净、结果判读及当前 4/4 自动用例边界；命令真源仍以仓内场景 README 为准。

## 宿主 MuJoCo AI 策略演示

[`assets/demo/start_lite3_mujoco_demo.sh`](assets/demo/start_lite3_mujoco_demo.sh) 是宿主侧启动器：不依赖 Docker/ROS，资产默认在仓外 `~/.cache/lite3-policy-staging-20261109`，venv 默认 `~/.cache/mujoco-demo-venv`，可用 `LITE3_DEMO_ROOT`、`LITE3_DEMO_PY`、`LITE3_DEMO_DISPLAY` 覆盖。入口在 05 MuJoCo 子页「Lite3 AI 策略交互演示」卡；用法、依赖与能力边界见 [`assets/demo/README.md`](assets/demo/README.md)。它不是 Gazebo 官方运行面，也不替代 formal 判据或真机。

全站已于 2026-11-08 同步到 v8.1 现行口径：当前结构见[项目架构](../note/架构/项目架构与目录.md)，验收与版本边界见[进度入口](../note/进度/README.md)，当前授权选件与新旧编号映射见[23号 v8.1 任务队列](../note/计划书v8.1/23_v8.1任务队列.md)，工程规则见[工程标准](../note/架构/工程标准.md)（以上 `../note/` 链接仅供仓内阅读，官网不服务 `note/`）。各主板块和仿真子页均标注更新日期；带“历史”的数字不扩 scope，也不把 R1126、R1127 四窗或 two_floor_nav 演示结果视为 formal、四足物理或真机验收。

## 3D 测试盘面

`index.html` 的 09「3D 仿真测试盘面」读取 [`assets/data/3d_runs.json`](assets/data/3d_runs.json)。该文件由仓库根目录执行以下命令生成（只读证据，不跑 Gazebo）：

```bash
python3 scripts/ops/collect_3d_runs.py
```

数据源：R1127 `two_floor` 四窗与历史窗口、EVT-PREVIEW 预演、R1122/R1125/R1126 纯 SLAM 演示线、R980/R981/R866 的 2D MBF driver.json。缺项在页面显示“—”，不做推算；当前 4/4 只统计 two_floor 四窗，历史失败、SLAM 和 2D MBF 均分行列出、不并入 3D 导航口径。

## 3D 任务台（已迁出）

任务台页面与静态资源在 `scripts/console/ui/`，由宿主 agent 在 `:8780` 提供；启动、栈白名单、
远程锁定、地图/世界数据生成与验证命令见 [`scripts/console/README.md`](../scripts/console/README.md)。
本服务不提供任务台，`/web/console.html` 返回 404。

## 本机浏览

在仓库根执行：

```bash
python3 web/serve.py
# 浏览器：http://127.0.0.1:8000/web/
```

[`serve.py`](serve.py) 将服务根设为仓库根，但只放行文档面：`web/`、`docker/`、`tooling/`、根 `README.md`、`ws/src/**/*.md`；其余路径（`note/`、源码、数据、任务台）返回 404。文本响应补 UTF-8 与 `Cache-Control: no-cache`，`/` 和 `/index.html` 跳转到 `/web/`。`--port` 改端口；`--bind` 默认 `127.0.0.1`，需要局域网访问时显式选择：

```bash
python3 web/serve.py --port 8080 --bind 0.0.0.0
```

直接打开 `index.html` 可看页面和本地资源；跨目录文件链接是否可用取决于浏览器策略，不能保证。

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

站点由 `ros1-slam-web` 容器只读挂仓提供（宿主 80 → 容器 8000），页面改动刷新即生效。`web/` 是 Git 跟踪的文档面（生成物 `assets/data/` 忽略）；任务台不在本服务，见 [`scripts/console/README.md`](../scripts/console/README.md)。

## 维护

状态更新须同时核对 `note/` 原始记录与 `calibration/calibration.py` 输出；只有在具备相应环境并实际复核后，才更新页面数字、快照日期和 digest。说明文档日期更新不代表网页内容已重新校准。操作命令集中维护在 [Docker 指南](../docker/README.md)；新增图片保持本地资源，不引入追踪脚本。
