# registry · 组件注册、检索与溯源

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录消费[契约层](../contracts/README.md)，加载组件 manifest、分离声明与实测证据，并挂载溯源边车。它提供注册和检索；跨组件冲突由 [composer](../composer/README.md) 检查，实际进程由 [framework](../framework/README.md) 装配。

## 实现与数据

| 入口 | 当前作用 |
| --- | --- |
| [registry.py](registry.py) | `Registry.load_dir/load_file`、`get/by_role/by_tag/all`、`evidence_separation`、`load_provenance/unprovenanced` |
| [data/manifests](data/manifests) | 已落盘的项目/后端登记；条目证据级别由内容决定，登记不等于已启动 |
| [data/provenance_core.json](data/provenance_core.json) | 来源、版本、许可、构建入口；原始冲突/TODO 如实保留，不代作许可结论 |
| [fixtures](fixtures) | 设计样例与坏输入；与 data 中登记分开 |
| [runtime.py](runtime.py) | HTTP 只读查询和组合检查服务，不启动算法进程 |

v1 manifest 自带 io；v2 manifest 由契约层从命名 ports 派生兼容 io。只有 v2 命名端口清单可进入 framework 图。注册层没有新增第二套组件定义。

## 加载与证据语义

目录按文件名字典序加载；任一 manifest 无效即整体拒绝，不跳过坏件。重复 id、无 manifest、缺文件和坏 JSON 分别给 `E_REG_DUP_ID`、`E_REG_NO_MANIFESTS`、`E_INPUT_MISSING`、`E_INPUT_JSON`；契约拒绝由 `E_REG_INVALID` 携原码。

`evidence_separation()` 将 declared 与 tested 分开；tested 仍要看具体 level 和 reports，不能将 contract_tested 自动等同 simulation_verified 或 robot_verified。tags 只用于检索。

溯源边车按 id 绑定 source/license/build_entry，未注册 id 为 `E_PROV_UNKNOWN_ID`，缺失/空白字段为 `E_PROV_INVALID_FIELD`。`unprovenanced()` 暴露未覆盖项，不静默补造来源。

## CLI 与服务

从仓根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/registry/registry.py \
  ws/src/90_platform/v8_platform/registry/data/manifests \
  --provenance ws/src/90_platform/v8_platform/registry/data/provenance_core.json
python3 -B ws/src/90_platform/v8_platform/registry/registry.py \
  ws/src/90_platform/v8_platform/registry/fixtures --tag indoor
python3 -B ws/src/90_platform/v8_platform/registry/runtime.py --port 8088
```

CLI 支持 `--tag`、`--role`、`--id`、`--provenance`。HTTP 缺省监听 127.0.0.1：`GET /components?tag=...&role=...&id=...` 交集筛选；`POST /compose` 请求体为 `{"components":["id1","id2"]}`，返回组合报告或冲突理由；响应携 registry_digest。服务使用启动时加载的目录，不热重载或执行组件。

定向测试：

```bash
python3 -B ws/src/90_platform/v8_platform/registry/test_registry.py
python3 -B ws/src/90_platform/v8_platform/registry/test_v2_manifest.py
python3 -B ws/src/90_platform/v8_platform/registry/test_runtime.py
```

这些测试覆盖加载/检索/证据/HTTP/拒绝语义；真实后端能力查[运行证据](../../../../../note/进度/v8/README.md)。
