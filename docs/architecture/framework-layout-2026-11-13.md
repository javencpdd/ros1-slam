# framework 目录迁移接续验证

> 最新更新时间：2026-11-13 22:51 CST。任务：T-20261113-03；接续起点：`107cfbd3`。同范围源码已由并行 Qoder 提交为 `0a98e2cb`，随后 `6656f64d` 取消跟踪 coverage；本会话没有重复迁移或创建这些提交。

接续时，入口、工具、运行冒烟和单测已分别迁入 `framework/nodes/`、`tools/`、`smokes/`、`tests/`，公共模块保留在根目录。保留既有迁移和暂存状态，未重复旧 Gazebo 四例，也未修改算法、地图、门值、真机授权或历史实验原件。

本轮补齐了以下遗漏：

- 场景 CLI 与独立场景测试从 `nodes` 导入迁移后的 `app_runner`。
- `run_map_file.py` 包装入口加载 `nodes/map_file_node.py`。
- 3D 回归矩阵的 smoke 路径增加 `smokes/`。
- 官网四处命令同步 `nodes/` 和 `tools/`；迁移涉及的文档补写更新时间。

新进程入口检查与节点纯导入独立执行，没有借用单测预先添加全部目录的 `sys.path`。

| 验证 | 实际结果 | 范围 |
| --- | --- | --- |
| framework 全套单测 | 296 项：286 通过、10 跳过；rc=0 | 宿主 Python，不是目标 Noetic |
| 独立场景测试 | 15 项：14 通过、1 ERROR；rc=1 | 唯一剩余错误为沙箱禁止创建 socket，原始失败保留 |
| 独立 CLI / dry-run | 6 项通过 | 帮助、应用解析、图检查、默认 blocked 离线规划；未启动 ROS |
| 节点独立纯导入 | 12/12 通过 | 干净进程、惰性 ROS 替身，不执行节点 main |
| 真实 node manifest | 23 项路径及执行位通过 | 明确排除 test double，不将声明检查当运行证明 |
| Python 3.8 语法 | 66 文件通过 | AST 兼容检查，不是 Python 3.8 实际执行 |
| 包装入口和回归矩阵 | 迁移路径检查通过 | 矩阵帮助仍依赖 generated `v8_msgs`；替身检查不冒充目标环境 |
| 三项目解耦 | 4 项通过 | P1/P2/P3 的原有源码边界保持 |
| registry | 19 项通过 | 迁移涉及的 manifest 和 provenance 检查 |
| 工程标准 | rc=0，25 个覆盖项 | 文本、指针及作用域检查 |
| 实际 ROS/Gazebo/Docker | NOT_RUN | 当前 Docker socket 拒绝访问 |

独立结论为 **源码布局及离线入口限定 PASS**。这不将场景套件的 socket ERROR 改成通过，也不新增迁移后的仿真运行或真机结论。当前源码与原始输出见[证据清单](evidence/framework-layout-2026-11-13/validation.json)，独立四件从临时目录复制保存；没有复制可执行探针代码。

本会话的暂存命令 `git add -- scripts/sim/two_floor_nav.py` 实际返回 rc=128：无法创建 `.git/index.lock`，只读文件系统。随后核对发现，并行 Qoder 已提交同一迁移及本轮修复的关键入口；独立复核的66份源码哈希与当前工作树仍全部一致。因此没有重新暂存或重复提交源码。本页、补充证据目录以及剩余两处文案仍需在允许写入 Git 元数据的环境中核对后本地提交，不 push；当前架构件已引用本页，这些补充文件需一并入库。

当前用户命名空间未映射 `ros1-slam` 的 uid 998，逐文件 named ACL 设置返回无效参数或重复溢出 UID 错误。没有删除原 ACL，也没有扩大其他用户权限；后续在真实宿主核对并补齐项目要求的 ACL。历史 EVT/BT runtime 原件和既存 `.coverage` 均保持。
