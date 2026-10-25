# v8 注册表（V8-04，R639 首切片＋R640 provenance 边车）

> 2026-10-25 起。本目录是**领域层的注册表实现**：加载组件 manifest、透传契约层校验、
> 提供检索与证据状态分离、挂载溯源边车。A1 已签（R638）后 M1 首件；消费 [契约层](../contracts/)，不反向依赖。
> 跨组件冲突判定、依赖图、锁文件归 V8-05/06；运行时装配归 V8-06+。

## 1. 交付内容

| 件 | 作用 | 边界 |
| --- | --- | --- |
| [registry.py](registry.py) | `Registry`：`load_dir`／`load_file`（fail-closed）、`get`／`by_role`／`by_tag`／`all`（确定性 id 序）、`evidence_separation`（声明/实测分开）、`load_provenance`／`unprovenanced`（溯源边车）；CLI `registry.py DIR [--tag\|--role\|--id] [--provenance P]` | 只消费契约层；标签只用于检索，不推组合能力 |
| [fixtures/](fixtures) | 3 个正例 manifest＋`provenance.json`（设计样例诚实标注）＋`invalid/` 负例（无效 manifest、重复 id 对、非 JSON、数组 JSON、边车未注册 id／空白字段／非 JSON） | 全部为设计样例 |
| [data/provenance_core.json](data/provenance_core.json) | **真实溯源数据件**：R616 role×后端表核心格 12 条（来源/许可/构建入口；TODO 许可与「宿主不可证」如实带原文）；待对应 manifest 落地后经 `--provenance` 挂载 | 数据抽取件，非许可结论；G1/G2/G6 冲突照引不裁 |
| [test_registry.py](test_registry.py) | 18 项：加载/检索/证据分离/负例理由码/fail-closed/加载顺序确定性/边车四态/核心数据件内部一致性/CLI | 证明加载/检索/拒绝语义，不证明运行能力 |

## 2. 语义要点

- **fail-closed**：目录加载中任一 manifest 无效即整体拒绝（`E_REG_INVALID` 携带契约层码），加载顺序按文件名字典序确定。
- **注册表级理由码**：`E_REG_DUP_ID`、`E_REG_NO_MANIFESTS`、`E_INPUT_JSON`／`E_INPUT_MISSING`。
- **证据分离**（22 号 §2）：`evidence_separation()` 返回 `declared`（仅声明）与 `tested`（contract_tested／simulation_verified／robot_verified，契约层保证非空 reports）。
- **溯源边车**：`load_provenance` 挂载 id→{source, license, build_entry}；引用未注册 id＝`E_PROV_UNKNOWN_ID`（fail-closed），字段缺失/空白＝`E_PROV_INVALID_FIELD`；`unprovenanced()` 如实暴露未覆盖组件，不静默。

## 3. manifest v1.1 修订（R641 已落地——真人勾选「增补 v1.1」，原两枚证据原文保留如下）

1. manifest.v1.json 缺 22 号 §3「至少包含……来源/许可证、适配器入口、代码版本与参数 schema」中的来源/许可证、适配器入口、参数 schema 字段（R627 起四轮复核未点到，消费真实盘点数据时现形）——当前以注册层边车补足（22 号「三者分别保存」可引为依据）。
2. manifest.v1.json 的 `capability.kinematics` 枚举（differential/holonomic/quadruped_kinematic）对非运动组件（odometry/定位/表征/地图）语义不合——真实 manifest 化时被迫给静态组件填运动学值；候选修法＝v1.1 加 `not_applicable` 或按 role 收窄必填。

## 4. 运行方式

```bash
python3 -B registry.py fixtures --provenance fixtures/provenance.json   # rc0 概要＋证据分离＋边车覆盖
python3 -B registry.py fixtures --tag indoor                           # 标签检索
python3 -B test_registry.py                                            # 18 项
```

## 5. 本批未覆盖

组合求解、兼容报告与锁定文件（V8-05）；跨组件 TF/控制权冲突（V8-05）；运行时装配与健康（V8-06）；核心格真实 manifest 化（下一切片，需逐后端读源码定 io/capability/ownership——先处置 §3 两枚修订证据）；R631 观察项 extent 语义定界（待地图域件开工）。
