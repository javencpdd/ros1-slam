# v8 多层表征（V8-08 首切片，R649）

> 2026-10-26 起。M3 首件首切片：support_map **生产者设计层**——多高度列、空闲证据、
> 净空查询、extent 语义定界（R631 挂账闭合）、edges 显式声明制。消费契约层 map.v1（已冻结）。
> 建图算法（传感器→面的提取）与运行时地图服务归后续切片；不 import ROS。

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

## 4. 未覆盖（后续切片）

版本与更新服务（增量更新、版本一致性服务面）、空闲证据入图（EvidenceGrid→map cell_states 的成图集成）、净空服务查询 API 化、建图算法（传感器→面提取）、运行时地图服务（25 号窗）。运行面 A3 正式验收归仿真。
