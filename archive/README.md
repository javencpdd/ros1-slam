# archive/

非活跃源码归档。此处内容不参与 catkin 构建、不进入任何 target 的包闭包。

## lite3_task（2026-11-21 归档）

- 原位置：`ws/src/50_task/lite3_task`（SMACH 任务编排，289 行，5 个状态类，零 launch 引用）。
- 处置：真人 2026-11-21 勾选「归档」；代码与 git 历史保留，不接线、不构建。
- 恢复：`git mv archive/lite3_task ws/src/50_task/lite3_task`，并在 `tooling/build/profiles.json` 的
  `local_packages` 与对应 target `roots` 中恢复 `lite3_task`；随后按构建纪律重建。

## 2026-11-22_lite3_costmap_layers（2026-11-22 退役）

- 原位置：`ws/src/30_perception/lite3_costmap_layers`。内容＝`ElevationMergedLayer` 一枚 costmap_2d 插件
  ＋ 仿真假高程源 `elevation_gridmap_publisher.py` ＋ `pointcloud_range_filter.py` ＋ 两份 config。
- 退役理由：真人 2026-11-22 裁定 B 链（BT＋move_base＋TEB）取消，本包失去宿主；A 链零引用。
- ★须记住的缺口★：地形「判据」在 A 链由 `local_native` 的 `map_topic=/v8_support_map` +
  `max_slope_deg=35.0` / `max_step_m=0.20` 承担，但**数据新鲜度不随之转移**——support_map 是
  `map_file_node.py` 发的 latched JSON，建图后不刷新 ⇒ 建图后新出现的地形在 A 链看不见。
  该缺口登记在 `note/12_真机迁移需求清单.md`，不得读成「功能已在 A 链实现」。
- 同步改动 9 项：

  1. `scripts/verify/v8_isolation.py`：判据改「甲2 双分支」——在位则校验内容／只在归档则校验归档副本／
     两头都有拒（防静默双份）／两头都没有拒。新增 `check_plugin_placement` 与 `check_plugin_contents`。
  2. `scripts/verify/test_v8_isolation.py`：fixture 改拷归档副本；新增 4 例（在位／双份／全缺／归档内容被改仍须拒）。
  3. `scripts/verify/pyst_face.txt`：按派生式剔除该 .py，清单数 62→61（`archive/` 不参与构建，故不再进 PYST 面）。
  4. `tooling/build/profiles.json`：删 `local_packages` 与两个 target 的引用共 3 处。
  5. `ws/src/40_navigation/lite3_navigation/config/{global,local}_costmap_params.yaml`：删插件类声明各 1 行。
  6. `ws/src/40_navigation/lite3_navigation/config/costmap_common.yaml`：删 `elevation_merged_layer:` 参数块。
  7. `ws/src/40_navigation/lite3_navigation/package.xml`：删 `exec_depend`。
  8. `ws/src/70_bringup/lite3_bringup/launch/mode_sim_teb.launch`：删 `elevation_source` 两条分支与
     `elevation_ttl_s` 两处 param（契约测试断言的 `cmd_relay=false` 与 `60_safety.launch` 均保留）。
  9. 注册表：删 `manifests/manifest_representation_costmap_layers.json` 与 `provenance_core.json` 对应条目
     （总数 14→13、tested 2→1；`test_registry.py`／`test_manifest_versions.py` 的写死计数同步）。

- 不动：`data/maps/real_lite3_20261103/env_inventory.txt:24` 是 2026-11-03 的机侧清单快照，属历史记录，不回改；
  `ws/src/80_algorithm/v8_grid/v8_grid/indexing.py:82` 只是 docstring 里的冻结来源注记。
- 恢复：`git mv archive/2026-11-22_lite3_costmap_layers ws/src/30_perception/lite3_costmap_layers`，
  再按上面 9 项逐条改回，最后跑 `python3 -B scripts/verify/v8_isolation.py --navigation <仓根>`，
  应回显 `[9-v8] approved packages=2 sources=0; plugin moved to 30_perception`。

