# framework/fixtures/manifests —— 运行装配的组件真源

**目录名里的 `fixtures` 不代表"可删的测试样例"。** 这里的 49 件 `v8.fw.*` manifest
是运行装配路径实际读取的组件目录，`app_runner.py`、各 smoke 与 regression 的
`config.load_components(...)` 都指向它。

## 为什么放在 `fixtures/` 下（命名由来与已知歧义）

`framework/fixtures/` 这个父目录还装有 `graphs/`、`apps/` 等**同样属于运行输入**的材料，
当初按"非源码的静态输入"整体归到了 `fixtures` 名下。后果是目录名容易被读成
"测试夹具、可整目录删除"。

这一点已在两处在册面打过补丁，改动前请先读：

- [`note/架构/项目架构与目录.md:67`](../../../../../../../note/架构/项目架构与目录.md)——
  「`fixtures` 名称不能用作删除依据」；
- [`v8_platform/README.md:26`](../../../README.md)。

2026-11-17 曾评估把本目录改名为 `framework/manifests/`（并同步把
`registry/data/manifests/` 改名为测试夹具语义），**结论是不改**：

- 活代码引用面 **40 个文件**（framework/smokes 15、framework/tests 11、planner 4、
  representation 2、regression 2、contracts 2、verify 1、registry 1、local_planner 1、
  app_runner 1）；
- `note/` 下另有 **51 个历史批次证据**（R956—R1103 等）记录着这个路径，按账本规矩不能改，
  改名后全局搜索会一半新一半旧，反而更难判断现行真源；
- CMakeLists 不 install 本目录，改名没有构建面收益。

## 与 registry 侧的关系

`registry/data/manifests/`（14 件 `v8.*`）是 registry 服务与 CLI 的默认目录，
不是本目录的副本。两套命名空间不同，现读 id 交集只有 `v8.fw.carrier_lite3` 一件。
详见 [`registry/data/manifests/README.md`](../../../registry/data/manifests/README.md)。

## 改这里的注意事项

- 新增 manifest 会使件数变化：`registry/test_v2_manifest.py` 断言件数、
  `contracts/test_manifest_v3.py` 全量扫描、`verify/test_port_conformance.py` 读同目录，
  加文件后要一起复跑（2026-11-17 就修过一次"加了 `manifest_bt_console.json` 但断言还写 48"
  的存量红）。
- 单件是否属于"测试替身"要按 manifest 的 `launch` 与 `evidence` 逐件判断，
  不能按目录整体判定。
