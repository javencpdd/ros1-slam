# lite3_costmap_layers —— B 线自研代价层插件包

> 批220／R566 新建（裁11＋裁12，真人 2026-10-22 六裁）。本包承载 B 线高程合并代价层的**源码**；
> `lite3_navigation` 是壳包（ISO 项[9] 红线禁 `.cpp/.cc/.h`，`v3_isolation.sh` 可执行校验），
> 所以自研层源码**不能**落进那个包——这正是另立本包的原因。

## 一句话

`ElevationMergedLayer` 订阅 `grid_map_msgs/GridMap`，把"离地高超过阈值"的格子**直接写进主代价图**，
合并语义**显式钉 `use_maximum: true`**；它与在册 `sob_layer::SobLayer` 同角色（都是 `costmap_2d::Layer`，
进 `plugins` 列表），**不另开第二层障碍层**（R59／X2 在册结论）。

## 为什么 use_maximum 是"唯一一枚安全侧开关"

两枚活体窗实测（R441—R453 预检线）：

| 在册结论 | 实测 |
| --- | --- |
| R445 | 两枚层同栈时第二枚 `use_maximum=false` ⇒ 9 枚致命格 **100→0**（非零 9→0）⇒ "S3 加 elevation 层必须显式钉 `use_maximum: true`；漏写／写 false＝安全侧反例" |
| R446 | elevation_layer **漏写**两个组合键 ⇒ 同样把那 9 枚格整片抹成 0（`distinct=[0]`）⇒ **本镜像这条链路上"漏写＝覆盖"**；且声明通路（父级 cfg 默认 1=Maximum）与行为通路相反 ⇒ 采行为路 |

本层的处置（源码里三档显式，无静默路径）：

1. **没给 `use_maximum`** ⇒ 取 `true`，并 `ROS_WARN` 点名；
2. **显式 `true`** ⇒ 取大合并（`new = max(old, new)`，永不把高代价抹低）；
3. **显式 `false`** ⇒ `ROS_ERROR` 且**拒绝写入任何代价**（宁可漏报，不覆盖）。

## 文件面

| 文件 | 作用 |
| --- | --- |
| `include/lite3_costmap_layers/elevation_merged_layer.h` | 类声明（接口＋参数＋边界注释） |
| `src/elevation_merged_layer.cpp` | 实现（`PLUGINLIB_EXPORT_CLASS` 在文件尾注册） |
| `plugins.xml` | pluginlib 描述件（源码空间放包根、安装空间装进 share） |
| `config/elevation_merged_layer.yaml` | ★示例★参数块（**未挂进任何在册配置**） |
| `CMakeLists.txt` / `package.xml` | 构建与依赖声明（C++14；依赖 costmap_2d／pluginlib／grid_map_*） |

## 怎么验它真的在链路上

挂载后看节点日志三行（沿在册 R449 的 C11 活性锁口径）：

1. `Using plugin "lite3_costmap_layers::ElevationMergedLayer"`（costmap_2d 加载插件时自印）；
2. 本层 `[<name>] 在位：… use_maximum=1(explicit=…) …`；
3. 本层 `[<name>] Received a grid map（…）`（★不能只看名册／rostopic list★——在册 R444 已证
   "名册报 None"可能是退订，节点日志的 "Received a … map" 才是主证）。

负向控制（判据必须能红）：把 `use_maximum` 显式写 `false` ⇒ 应当看到 `ERROR … 拒绝写入任何代价`，
且**任何格都不被本层写**。跑不出这条红＝闸没牙，别当已验。

## 边界（本版刻意不做）

- 不做下采样／多帧融合／可视化话题；不读任何门值文件；不改 nav 侧在册配置一字。
- 坐标系：与 costmap 全局帧不同时用 `tf_` 做**平面**变换（只取 yaw＋平移；roll/pitch 不在 2D 代价语义内）。
- `height_threshold: 0.15` 等判据值★是我方自定值、无在册实测锚★（与 X5「step>0.15m 出自我方 spike3d 设计稿」同族），
  上线前须按实验窗定档，不得当作已验门值引用。
