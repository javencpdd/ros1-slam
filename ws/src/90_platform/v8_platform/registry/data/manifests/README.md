# registry/data/manifests —— registry 服务的默认组件清单

**这里不是运行装配的真源。** 运行装配读的是
[`framework/fixtures/manifests/`](../../../framework/fixtures/manifests/README.md)（49 件 `v8.fw.*`）。

## 这个目录是什么

14 件 `v8.*` 领域组件的 manifest（v1 主干），同时服务三个用途：

1. **registry 服务/CLI 的默认目录**——`registry/runtime.py` 的 `--directory` 默认值即此处；
2. **registry 层测试的夹具**——`test_registry.py` 的 `load_dir`、`test_v2_manifest.py` 的
   `V1`、`test_orchestrator.py`、`adapters/test_*.py`、`composer/test_composer.py`；
3. **regression 的组件来源**——`regression/regression.py` 的 `MDIR`。

## 与 framework 侧的关系

两套清单是**不同命名空间**，不是同一批文件的副本：

| | 本目录 | framework/fixtures/manifests |
|---|---|---|
| id 前缀 | `v8.*` | `v8.fw.*` |
| 件数 | 14 | 49 |
| 消费方 | registry 服务、registry 层测试 | `app_runner.py` 等运行装配路径 |
| schema | v1 主干 | v3 |

现读 id **交集只有 `v8.fw.carrier_lite3` 一件**（唯一的跨界组件）。
`test_registry.py::test_cross_catalog_only_shares_carrier_and_versions_agree`
把这条关系锁住了：交集不得自行扩大，且交集内同一 id 的 `schema_version` 必须两侧相同
（该件 2026-11-17 前曾出现 registry 侧 v2、framework 侧 v3 的漂移）。

## 改这里的注意事项

- **不要**把这 14 件当"样例数据"整目录删除——`runtime.py` 与 `regression.py` 会在默认路径下读它。
- 新增/修改 manifest 后，若该 id 也存在于 framework 侧，两边 `schema_version` 必须一起改，
  否则上面那条断言会红。
- 目录名里的 `data` 指"服务数据"，不表示"可丢弃的临时数据"。
