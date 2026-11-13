# 架构附录与图源

> 最新更新时间：2026-11-13 CST。下列建模材料始于 2026-10-30；10-30/11-06 批次只核对用途和文件在位，未重跑其中实验或重绘图像。2026-11-13 新增三项目边界视图，2026-11-12 保留 SCAN＋BT 控制台全链时点视图。旧批次件不回改，只标时点。

**先读当前架构**：[项目架构与目录](../../note/架构/项目架构与目录.md)。它集中说明传统 launch 与 v8 app/graph 两条装配链、真实源码入口、两种 mux 和工作机边界。当前排程及能力状态见[执行队列](../../note/计划书v8.1/23_v8.1任务队列.md)与[进度入口](../../note/进度/README.md)。

本目录是补充资料，保留独有的设计分析、HTML 视图和图源。其旧批次读数、模块枚数、未接入判断及路线建议只适用于原建模时点，不能覆盖现行源码或恢复已取消任务。

| 补充材料 | 用途 |
| --- | --- |
| [current-2026-11-13.md](current-2026-11-13.md) | ★当前★ 三项目（P1 平台/P2 仿真/P3 可视化）边界、11-12 之后更新、本轮复核发现与处置（[证据索引](current-2026-11-13.evidence.md)） |
| [current-2026-11-12.md](current-2026-11-12.md) | 11-12 时点现状模型＋已实现/未实现功能盘点（[证据索引](current-2026-11-12.evidence.md)）；全量功能盘点仍以本件为准 |
| [current-system.md](current-system.md) | 早期当前态系统建模与命令链分析 |
| [algorithm-swap.md](algorithm-swap.md) | 建图、定位、全局、局部和中间件替换机制分析 |
| [target-framework.md](target-framework.md) | 目标框架与差距分析 |
| [framework-v0.md](framework-v0.md) | YAML 组件图、运行装配和 MapDelta 的设计说明 |
| [framework-selector.html](framework-selector.html) | 图形化选配示例；不据此证明现场组件可运行 |
| [architecture-ros1-slam.html](architecture-ros1-slam.html) | 早期建模结果的单页展示 |
| [evidence.md](evidence.md) | 建模时使用的 sourceRefs、置信度与未知项 |

| 图 | 源文件 | 派生图 |
| --- | --- | --- |
| 三项目边界与运行拓扑（11-13） | [three-projects-2026-11-13.dot](diagrams/three-projects-2026-11-13.dot) | [SVG](diagrams/three-projects-2026-11-13.svg)、[PNG](diagrams/three-projects-2026-11-13.png) |
| 系统上下文（11-12） | [system-context-2026-11-12.dot](diagrams/system-context-2026-11-12.dot) | [SVG](diagrams/system-context-2026-11-12.svg)、[PNG](diagrams/system-context-2026-11-12.png) |
| SCAN＋BT 链（11-12） | [scan-bt-chain-2026-11-12.dot](diagrams/scan-bt-chain-2026-11-12.dot) | [SVG](diagrams/scan-bt-chain-2026-11-12.svg)、[PNG](diagrams/scan-bt-chain-2026-11-12.png) |
| 系统上下文 | [system-context.dot](diagrams/system-context.dot) | [SVG](diagrams/system-context.svg)、[PNG](diagrams/system-context.png) |
| 运行与模块分层 | [system-runtime.dot](diagrams/system-runtime.dot) | [SVG](diagrams/system-runtime.svg)、[PNG](diagrams/system-runtime.png) |
| 命令与反馈链 | [command-path.dot](diagrams/command-path.dot) | [SVG](diagrams/command-path.svg)、[PNG](diagrams/command-path.png) |

在仓库根执行以下命令可重绘；需 Graphviz 与可用的中文字体（原图使用 `Noto Sans CJK SC`）：

```bash
cd docs/architecture/diagrams
for f in system-context system-runtime command-path system-context-2026-11-12 scan-bt-chain-2026-11-12 three-projects-2026-11-13; do
  dot -Tsvg "$f.dot" -o "$f.svg"
  dot -Tpng -Gdpi=110 "$f.dot" -o "$f.png"
done
```

图的绿色/橙色/蓝色/灰色/红色及实虚线表示原材料中的运行、实验、设计、待接与 L2 边界；它们不是本轮重新验收的结果。版本成果及相较上版改进见[成果总览](../../note/进度/00_总览.md)，旧批次定位见[审计索引](../../note/进度/01_批次.md)，当前任务和交接以 `.ai` 为准。
