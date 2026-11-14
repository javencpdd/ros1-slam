# 目标构建计划与隔离空间

> 最新更新时间：2026-11-06 04:06:11 CST。工程标准真源：[工程标准](../../note/架构/工程标准.md)。本入口仅证明声明依赖的选择与构建隔离；未经设备验证不能称为真机部署验收。

从仓库根执行：

```bash
python3 -B scripts/build/target_workspace.py plan --target lite3
python3 -B scripts/build/target_workspace.py plan --target x86_sim
python3 -B scripts/build/test_target_workspace.py
```

2026-11-14 物理分仓后，`x86_sim` 默认把同级 `ros1-slam-simulation/ws/src` 并入 source view；可用 `--extra-source DIR`（可重复）显式指定。机器人目标不并入。

`plan` 只读，宿主无需 ROS、catkin_pkg 或 PyYAML。目标与动作必须显式填写；不从 CPU、主机名或已 source 的环境猜目标。输出 JSON 包括根包、完整声明依赖闭包、manifest SHA256、隔离空间、未核验的外部依赖、忽略包和缓存冲突。闭包或路径错误返回 2；已有旧缓存只记入 `cache_conflicts`，不妨碍只读审计，也不创建、修补或清理空间。

| 目标 | 根包范围与状态 | 源码视图 | build / devel / install |
| --- | --- | --- | --- |
| `lite3` | 显式选择驱动、建图、定位、导航、SCAN、安全、任务与 v8 壳包；只取真机依赖 | `WS/.profiles/lite3/src` | `WS/.profiles/lite3/` 下分别存放 |
| `m20` | 未实现硬件协议及 ROS 适配，计划和构建均拒绝；修改 profile 的 supported 字段也不能启用 | 不生成 | 不生成 |
| `x86_sim` | 公共算法 + 同级 P2 仿真仓（默认 `../ros1-slam-simulation/ws/src`，`--extra-source` 可覆盖） | `WS/.profiles/x86_sim/src` | 保留 `WS/build`、`WS/devel`、`WS/install`，兼容现有仿真运行路径 |

`WS` 默认是本仓 `ws/`，可用 `--workspace` 指定含真实 `src/` 的工作区。`--profiles` 用于明确指定配置文件，默认 [profiles.json](profiles.json)。新增本地包或更换根包时同步维护配置名册；根包来自包内 `<name>`，不是目录名（例如 `pipeline_tracking` 的包名是 `pipeline`）。当前根表覆盖上述功能范围；旧 launch 的未声明运行依赖仍须逐项审查，不能由闭包计划反推全部部署依赖已经完整。

解析遵循 catkin 包边界：遇 `CATKIN_IGNORE` 排除整棵子树，发现包后停止向包内第三方目录递归；重复包名、所需本地包缺失或被忽略都会拒绝。闭包包含 build/build_export/buildtool/buildtool_export/exec/run/depend；test/doc 不参与，构建固定 `CATKIN_ENABLE_TESTING=OFF`。format 3 仅求值明确的 `$ROS_VERSION == 1/2`、`!=` 条件，固定 ROS1；其他条件和 group 依赖保守拒绝。不会用任意 `eval` 或宿主变量猜测。

Lite3/M20 路径拒绝仿真包（按包名/依赖判定，不再依赖 `95_simulation/` 目录前缀）、`training/`，并拒绝 Gazebo、MuJoCo 和已登记的训练依赖名称；传递依赖同样检查。另有一条 host-only 边界：`web/`、`scripts/console/`、`data/console/` 不得进入机器人 source view 或镜像上下文。`plan` 输出 `host_only` 策略，`prepare_view`/`build` 在生成视图落盘前做机器检查，`.dockerignore` 对 Docker 上下文同步排除；测试覆盖三项路径和 `lite3`/`x86_sim` 的策略差异。外部 ROS/系统依赖单列为 `NOT_CHECKED`，本工具不解析 apt 的完整传递关系，也不检查驱动 SDK、设备 ABI 或执行安装。输出的 `rosdep_check_command` 是建议命令，只指向选中的生成视图路径，显式重复 `--dependency-types` 以排除 test/doc；运行它需要视图已生成。不得改成对全量 `WS/src` 的默认 rosdep 安装，再把仿真测试依赖带回机器人。

实际构建必须在目标机器或相应构建环境显式触发：

```bash
source /opt/ros/noetic/setup.bash
python3 -B scripts/build/target_workspace.py build --target lite3 --jobs 1
# 只清理身份正确的既有本目标产物并重新构建：
python3 -B scripts/build/target_workspace.py build --target lite3 --jobs 1 --clean
```

入口要求原生 Noetic setup 与 catkin_make 可用，`ROS_DISTRO=noetic`、`ROS_VERSION=1`；`CMAKE_PREFIX_PATH` 只能是 `/opt/ros/noetic`，`ROS_PACKAGE_PATH` 每项只能是该 underlay 的 `share` 或 `stacks`。须使用干净 shell；已 source 旧仿真/机器人工作区时明确拒绝。预检失败不生成 profile 或任何输出目录。入口不透传任意 CMake 参数，不清空 whitelist，不在真实 `WS/src` 初始化 catkin 或写 `CATKIN_IGNORE`。

生成视图中只为选中包创建指向 `WS/src` 内的相对链接。Livox 上游以 `package_ROS1.xml` / `package_ROS2.xml` 区分版本；配置显式选择 ROS1 manifest。当前工作区已有 `package.xml`，须与 ROS1 件字节一致，否则拒绝，避免掩盖错误选择。上游缺 `package.xml` 时仅在生成视图创建目录壳：其他内容相对链接回原包，ROS1 XML 复制为壳内 `package.xml`；不改第三方原件。

每个 profile 有 `identity.json`，绑定目标、工作区、输出空间、根表、配置与 manifest 哈希。构建还检查 CMakeCache 的源码视图、devel/install 前缀及非空显式 whitelist，拒绝跨目标、旧全量缓存、软链输出或越界源码。配置或 manifest 变化需显式 `--clean`；它先核目标、工作区、路径和 ROS 身份，只清理本目标固定视图与三类产物，不删除原始源码。无归属旧目录、身份错误或 profile 中有未知文件时拒绝自动清理，须先保留证据并单独迁移。

仿真选择 `livox_laser_simulation` 时，在同一隔离视图内先 configure，枚举并串行构建消息生成目标，再调用 catkin_make；任一步失败即停止。不靠修改上游 CMake 掩盖消息依赖问题。命令使用 [Noetic catkin_make 官方支持的接口](https://raw.githubusercontent.com/ros/catkin/noetic-devel/bin/catkin_make)：`-C`、`--source`、`--build`、`CATKIN_DEVEL_PREFIX` 和 `CMAKE_INSTALL_PREFIX`；rosdep 参数依据[官方实现](https://raw.githubusercontent.com/ros-infrastructure/rosdep/master/src/rosdep2/main.py)。

Docker 镜像构建与容器内重编通过 [build.sh](../../docker/build.sh) / [catkin_build.sh](../../docker/catkin_build.sh) 共用本 helper，固定显式 `x86_sim`；`--plan` 无 ROS 只读，`--clean` 仍只允许清理身份正确的本目标空间。构建子 shell 使用最小环境、固定系统 PATH 与 Noetic include，然后只 source Noetic；不继承旧 overlay 或编译器参数。依赖 SDK 使用系统默认路径与 ldconfig，未提供隐式自定义环境透传。

镜像 builder/runtime 都包含工具、配置和同路径源码视图；`.dockerignore` 保留两个工具目录、排除 WS 生成物。Compose 使用新的 x86 专属 build/devel/install/profile 卷，旧全量卷保留但不自动挂接，避免新身份文件误认旧产物。`PROFILE=core`、旧 `SLIM=1` 明确拒绝；使用方式和缓存边界见 [Docker 说明](../../docker/README.md)。

本次只验证离线解析、命令/环境传递、拒绝路径和静态配置；未执行真实编译、镜像构建、ROS/仿真或硬件验证。Lite3 原生目标及 SDK/系统依赖仍需单独验证，不能把上述 Docker 配置视为机器人部署证据。
