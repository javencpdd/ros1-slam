# 目标框架 vs 当前态：可插拔地图/机器人/算法/中间件

> 日期：2026-10-30 · 目标框架来自主理人需求；当前态依据代码与配置核对
> 结论先行：**当前系统只支持其中一部分（Lite3 2D 栈、离线 3D 支撑图、MuJoCo 单链实验），
> 尚不支持“任意地图/任意机器人/任意算法 + 任意中间件插入 + 仿真通行性辅助规划”的核心框架。**

## 一、需求拆解

| # | 目标能力 | 具体要求 |
| --- | --- | --- |
| R1 | 地图可接入 | 2D 栅格、3D 支撑/高程、语义地图 |
| R2 | 移动物体可接入 | Lite3、M20 等，同一套接口 |
| R3 | 算法可替换 | 建图、定位、全局规划、局部规划 |
| R4 | 中间件可插入任意逻辑线路 | 例如全局与局部之间；局部建地图缓存、间隔更新全局地图 |
| R5 | MuJoCo 辅助规划 | 用仿真验证移动物体通行能力，反馈给全局/局部规划 |

## 二、当前支持度

图例：✅ 支持；▲ 部分/仅设计层；❌ 不支持。

| 需求 | 当前状态 | 证据/说明 |
| --- | --- | --- |
| R1 · 2D 地图 | ✅ | `map_server` + `costmap_2d`；PGM/YAML 底图在 `data/maps/` |
| R1 · 3D 地图 | ▲ | 支撑图 JSON 可离线派生；`elevation_merged_layer` 可把高程并进 costmap；`representation` 有 map.v1/v2、版本服务与查询接口，但**运行时更新流未接**（README §5 归仿真阶段） |
| R1 · 语义地图 | ❌ | 只有离线 `config/passability/passability.yaml` 与 `w3_elevation_node.py` 的 2.5D 高程；没有语义类别层、置信度、版本与查询服务 |
| R2 · Lite3 | ✅（真机 L2） | ROS1 栈 + `message_transformer`（qnx2ros/ros2qnx）；真机控制未授权 |
| R2 · M20 | ❌ | 全仓 `ws/src` 与 `note/10`、`note/11` 无 M20 代码/配置命中；需厂商 SDK/协议与真机授权 |
| R3 · 建图 | ▲ | 只有 faster-lio 一套在线建图；`pcd2grid` 离线；没有 mapping 角色 manifest，换后端靠壳包 launch/config |
| R3 · 定位 | ▲ | faster-lio 里程计或 hdl_localization 两种模式；无统一 localization 端口 |
| R3 · 全局规划 | ▲ | 2D：GlobalPlanner/Navfn（MBF 插件）；3D：支撑图 A* 为设计层＋单测＋独立 ROS 包装，未进运行链 |
| R3 · 局部规划 | ▲ | 2D：TEB/DWA（MBF 插件）；3D：SCAN 独立进程、A5 独立 runner，均非 MBF 插件 |
| R4 · 任意中间件插入 | ❌ | 运行装配是静态 launch/include/remap；没有运行时组件图/端口/路由层；v8 composer/orchestrator 只做设计校验与 SimulatedRunner |
| R5 · MuJoCo 通行性辅助 | ❌（仅实验） | MuJoCo 桥是“整条 A5→270 策略”实验链（R952/R953 合计 38/50），不是可查询的通行性服务；规划器无法调用它 |

## 三、核心缺口：缺的是“脊柱”，不是某个算法

当前系统是**两条固定链**：

1. 2D：雷达/IMU → faster-lio → costmap → MBF → mux → 载体；
2. 3D：支撑图 → A5 → MuJoCo/3D 载体。

目标框架需要的是**运行时组件图**：

```text
地图服务（2D/3D/语义，版本化）
        ↑ 查询/更新
组件 A ──端口──> 中间件 M ──端口──> 组件 B
（建图/定位/全局/局部/机器人适配器/仿真 oracle）
        ↓
安全仲裁 + 命令边界 + 证据
```

四个缺件：

1. **端口契约**：Map2D/Map3D/Semantic/Traversability/Pose/Path3D/Twist 的类型、帧、版本、时效没有统一。
2. **运行时图执行器**：能按配置把组件连起来，并在任意边上插入中间件；今天只有静态 launch。
3. **版本化地图服务**：能接受“局部缓存 → 间隔合并 → 全局地图更新”，并防回退、防膨胀、可追溯；今天是离线 JSON + 静态 costmap。
4. **通行性 oracle**：MuJoCo/Gazebo 能对“地形片 + 机器人 profile + 指令”返回可通行/风险/上限；今天是一次性整链实验。

## 四、目标架构（建议）

| 层 | 内容 | 复用/新建 |
| --- | --- | --- |
| L0 契约 | 地图（2D/3D/语义）、机器人 profile、路径/轨迹、通行性结论 | 复用 v8 contracts（map.v1/v2、robot.v1），扩 map3d/semantic/traversability |
| L1 组件 | 建图、定位、全局、局部、机器人适配器、仿真 oracle | 现有算法包＋壳包；统一输入/输出端口 |
| L2 中间件/图 | 组件图、边路由、插入/替换、速率/时效策略 | **新建**（v8 composer/orchestrator 做治理；运行执行器新建） |
| L3 地图服务 | 多表示存储、查询、版本、更新事务 | 复用 representation 的 MapService 设计，补运行时更新流 |
| L4 机器人 | RobotProfile + 命令/遥测边界 | 复用 boundary S1；补 Lite3/M20 适配器 |
| L5 安全/证据 | mux、validator、断链、门禁 | 复用 mux/validator；中间件不得绕过 |

### 中间件示例：局部缓存 → 间隔更新全局地图

目标：局部规划在运行中维护一块局部 3D 缓存，每隔 N 秒把增量合并进全局地图，全局规划下一次查询用新版本。

需要的最小机制：

1. **地图服务写接口**：`propose_delta(map_id, base_version, delta)`，服务端做版本校验、防回退、原子合并；
2. **中间件组件**：`MapCache` 订阅局部观测/位姿，维护缓存，按 `update_interval` 发布 delta；
3. **图执行器**：把 `MapCache` 插在“局部感知/局部规划 → 地图服务”这条边上，不改两端代码；
4. **策略字段**：`update_interval_s`、`stale_after_s`、`merge_policy`、`max_delta_cells`；
5. **消费端**：全局规划按 `map_version` 查询；版本落后时拒绝或降级，不静默用旧图。

今天这些都不存在；用节点发布新地图 + 重启 map_server 只能算临时拼接，没有版本与原子性。

## 五、改进路线（按依赖顺序）

| 片 | 内容 | 验收/出口 |
| --- | --- | --- |
| P0 | 冻结端口契约：Map3D/Semantic/Traversability/Path3D；地图服务运行时（查询+更新+版本） | 契约测试；服务可查询/更新；A3 运行面 |
| P1 | 运行时图执行器＋组件包装：把 faster-lio/hdl/MBF/mux 包成组件；在全局与局部之间插一个 no-op 中间件 | 同任务两种接线结果一致；插入/移除不改两端代码 |
| P2 | MapCache 中间件：局部缓存→间隔更新；版本/防回退/原子合并 | 更新前后全局规划使用不同版本；回退/冲突被拒 |
| P3 | 3D 全局/局部接入：`mbf_simple_nav` + `AbstractPlanner/AbstractController` C++ shim，或独立 3D server；2D/3D 混合配对 | GetPath→ExePath 可跑；按名点单；延迟/成功率对照 |
| P4 | 机器人 profile＋M20 适配器：RobotProfile 能力/限幅/命令协议；Lite3/M20 同边界 | Lite3 复跑；M20 需 SDK 与 L2 授权 |
| P5 | MuJoCo 通行性 oracle：地形片+profile+指令→可通行/风险/上限；缓存；规划器查询 | 与 A4 声明门对齐；仿真结果只作辅助，不冒充真机 |
| P6 | 安全与验收：validator 接所有路径，mux 唯一写者，断链/取消；A1—A7 | 门禁与运行证据 |

## 六、边界与不做的事

- **不把 MBF 当脊柱**：MBF 是 2D 后端之一；3D 走 `mbf_simple_nav`/独立 server，或作为图中的一个组件。
- **不承诺 ROS1 原生热插拔**：运行时换边需要路由节点/生命周期管理；先做“配置期插入＋重启”，再谈热替换。
- **不让中间件绕过 mux**：任何改路径/改速度的中间件都要过安全仲裁与证据。
- **不把 MuJoCo 当通行性真值**：它只能辅助；可爬高度/坡度仍需 A4 声明门与真机验证（L2）。
- **M20 未接就是未接**：没有代码与协议证据，不写“支持”。

## 七、一句话回答

**当前系统支持“Lite3 的 2D 导航 + 离线 3D 支撑图 + MuJoCo 单链实验”，
但不支持“多地图/多机器人/任意算法 + 任意中间件插入 + 仿真通行性辅助”的目标框架。
最关键的改造不是再写一个算法，而是补四件基础设施：端口契约、运行时图执行器、版本化地图服务、通行性 oracle。**
