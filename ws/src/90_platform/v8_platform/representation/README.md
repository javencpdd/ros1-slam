# v8 多层表征（V8-08 首切片，R649）

> 2026-10-26 起。M3 首件（R649—R651 三切片，已窄口径收口）：support_map **生产者设计层**（多高度列／空闲证据／净空／extent 定界）、**版本与更新服务**（MapService：防膨胀防回退／漂移检查／证据随版管理／净空 API）、**建图算法设计层**（elevation.py：点云→格桶→z 聚类全保留→连通成面小簇拒→咨询边全链）。消费契约层 map.v1（已冻结）。传感面（真实雷达数据、滤波配准）与运行时服务归仿真阶段；不 import ROS。

## 1. 交付内容

| 件 | 作用 | A3 判据落点 |
| --- | --- | --- |
| [support_map.py](support_map.py) | `HeightColumn`（同 XY 全高度保留，**无『取最高』入口**，z 冲突 E_REP_Z_CONFLICT，净空查询开顶＝None 不冒充）；`EvidenceGrid`（free 只来自正向证据，缺证＝unknown，**无默认 free 路径**）；`suggest_edges`（几何邻近＋高差阈值**只出咨询候选**，advisory_only 标记，绝不自动成边）；`build_support_map`（显式面＋显式边→map.v1 合法图，契约层 fail-closed 透传）；`columns_of`（反查多高度列） | A3-1／A3-2／A3-3 |
| [test_support_map.py](test_support_map.py) | 9 项：列三层保留与无折叠入口、z 冲突、净空、unknown 不变 free、建议器咨询性、合法出图＋extent 格数、不连通拒（按分量拆 map 的登记裁定）、无面拒、extent 随足迹缩放 | — |

## 2. 语义裁定（本件登记）

1. **extent 定界（R631 挂账闭合）**：support_map 的 extent＝支撑面足迹并集外包框按 resolution 换算的格数（整数 ≥1）＝「证据与面已声明的 XY 区域」；无支撑面拒绝出图。
2. **不连通支撑域**：map.v1 要求单连通（全部面从首面可达）——两栋无桥塔楼这类**按连通分量拆分为多个 map 实例**（一个 map_id＝一个可通行区域），builder 对不连通输入照契约拒（E_MAP_DISCONNECTED 经 E_REP_CONTRACT 透传）。
3. **edges 只显式声明**：断桥不误连的构造保证——无任何「邻近即连边」的代码路径；suggest_edges 输出必须经上游确认才可入图。

## 3. 对抗面复验（代行，2026-10-26）

同高同足迹重叠拒（E_MAP_ML_SAME_Z）、边指幽灵面拒（E_MAP_EDGE_TARGET）、不可通行面持边拒（E_MAP_EDGE_TRAVERSABILITY）、三层列 0/0.6/1.2 全保留＋净空 0.6、建议器高差阈值边界——全 PASS（两枚探针数据错误留痕：未真设 unsupported、屋顶 3.0 超 step 上限被契约正确拒绝＝fail-closed 正证）。

## 4. 模块清单（三切片后）

| 件 | 作用 |
| --- | --- |
| [support_map.py](support_map.py) | 多高度列（无『取最高』入口）／证据网格（无默认 free）／咨询边建议／build→map.v1 fail-closed／extent 定界 |
| [map_service.py](map_service.py) | 版本与更新服务（内容身份 digest、防膨胀防回退、source.sha256＝内容身份）／消费者漂移检查／证据随版管理／净空 API |
| [elevation.py](elevation.py) | 建图算法设计层：extract_columns（z 聚类全保留）／extract_surfaces（连通成面、小簇拒且数据不丢）／mapping_pipeline（全链，咨询边不自动成图） |
| 测试 | test_support_map 9＋test_map_service 5＋test_elevation 5＝**19 项** |

## 5. 未覆盖（移交）

传感面建图（真实点云滤波/配准）、地图服务运行时（话题/更新流）——归仿真阶段（25 号窗）；A3 正式判据归 V8-13 矩阵。

## 6. ROS1 查询接口

`map_node.py` 在独立适配层提供 `std_srvs/Trigger` 查询，JSON 响应包含地图版本、
内容身份和 `query_points` 中每点的全部层、支撑、净空、观测状态。
`map_json` 指定合法 map.v1 文件；可选 `evidence_json` 的格式为
`[{"xyz":[0.25,0.25,0],"state":"free"}]`。未列出的点为 `unknown`，
有支撑面不等于已观测为空闲。每次查询重新加载文件与 `query_points` 参数。
正负小数坐标用精确足迹包含测试，不以整数截断外扩屋顶。

在已经启动的 ROS master 中运行接口证据驱动：

```bash
PYTHONPATH=../regression:$PYTHONPATH python3 -B -m a3_runtime --out /tmp/a3-runtime.json
```

该驱动使用显式声明地图夹具，验证同 XY 双层、证据未知/空闲/占据、版本更新、
单向反向无路；不构成 Gazebo 传感流建图验收。map.v1 冻结契约保持原样。

## 7. 运行时 MapDelta 消费（R976）

`delta_map_node.py` 在 `MapService` 上提供 ROS 运行时：订阅 `/map_delta`（JSON String），
原子应用 `map_delta/v1`，发布版本化 `/v8_support_map` 与 `/v8_map_status`，Trigger `~query`
返回快照。坏 JSON/旧 base/异 map_id 只记错误、不改版本；回放同一 delta_id 幂等。
真机/QNX 仍未接（L2）。
