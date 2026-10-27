# v8 组合求解（V8-05，R644—R645：结构检测＋兼容比对＋锁文件）

> 2026-10-25 起。领域层组合求解器首切片：**角色依赖图与结构冲突检测**（22 号 §组合流程 步骤2＋步骤5 检测面）。
> 消费 [注册表](../registry/)（12 份核心格真实 manifest）与契约层（经注册表透传）；不 import ROS。
> 结构检测（R644）、兼容比对与确定性锁文件（R645）已交；消息级语义匹配与适配器选择归 V8-06/07。

## 1. 交付内容

| 件 | 作用 | 边界 |
| --- | --- | --- |
| [composer.py](composer.py) | `load`（manifest 目录→注册表）＋`compose(registry, ids)`：角色重复／缺失角色（任一满足集）／角色环（防御）／TF child 双 parent／TF 环／命令话题双写者——全部 fail-closed；结果携带 declared_only/tested 证据分档与命令/TF 所有权表；CLI `composer.py DIR --select ids [--provenance P]` rc 0/5/9 | 结构层检测；语义兼容归流程3 |
| compatibility() | 流程3：地图供需（生产者角色自述产出、需求方须有同类生产者，E_COMP_MAP_SUPPLY）＋运动学配对（运动链角色禁 not_applicable、规划器×控制器允许配对表，E_COMP_KINEMATICS）；报告条目随结果返回 | 消息级语义匹配归 V8-06/07，报告留 adapters 槽 |
| lock() | 流程6：确定性锁文件——compose+compat 全过后生成，digest＝规范序列化 sha256（对选择序与重复选择不变），逐组件带 version/evidence/declared_only/sha256；CLI `--lock OUT` | 拒绝报告＝CLI rc5 JSON |
| [test_composer.py](test_composer.py) | 17 项：结构检测十项（真实六件套正例、真实重复角色 dwa+teb、空选择、TF/命令注入负例）＋兼容三项（供需错配、not_applicable 运动角色、holonomic×differential 错配）＋锁三项（确定性/摘要重算/重复选择不变、CLI 锁与拒绝不留件） | 正例全部取自真实 manifest |

## 2. 语义要点

- **角色依赖图**（`ROLE_DEPS`，22 号 §1 职责表派生，「任一满足」集）：localization/mapping/representation→odometry；global_planner→{map_server, representation, mapping} 任一；controller→local_planner；trajectory_validator→local_planner；safety_arbiter→controller。local_planner 的地图依赖由 manifest 的 io 声明表达，归兼容比对切片（流程3），本层不预设。
- **建议/仲裁分层**（22 号差异表「多节点直发最终 cmd_vel」行的组合层落点）：组件声明**建议话题**（如 `/cmd_vel_advice/dwa`），`/cmd_vel` 唯一写者＝safety_arbiter——dwa/teb manifest 的 command_topics 已按此修正（R644，依据＝22 号差异表原文）。
- **证据分档随行**：结果携带 declared_only/tested 清单，供锁文件层（流程6）标注「未验证组件不得宣称支持」（22 号铁则的登记面）。
- 理由码：`E_COMP_UNKNOWN_ID`／`E_COMP_DUP_ROLE`／`E_COMP_MISSING_ROLE`／`E_COMP_TF_CYCLE`／`E_COMP_TF_DUP`／`E_COMP_CMD_DUP`。

## 3. 未覆盖（移交后续件）

适配器选择与信息损失声明（流程4，V8-06/07）；资源预算与停止链运行时校验（流程5 运行面，V8-06）；消息级 io.inputs/outputs 逐对匹配（V8-07 适配器契约）；锁内含 provenance 快照（观察项，V8-06 编排可加）。

## 4. 运行方式

```bash
python3 -B composer.py ../registry/data/manifests --select v8.odometry.faster_lio,v8.controller.dwa_adapter  # rc5 缺角色
python3 -B test_composer.py   # 17 项
```
