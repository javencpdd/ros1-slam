更新时间: 2026-10-10 00:1x

# L · faster-lio 轴向丢点的候选环节（代码级阅读，零施工、零参数改动 · 喂请求单 B10／C9）

- 作者：小明（施工侧阅读）。性质：**只读代码＋只登记候选，不点名结论**。
- 起因：R72 三层探针实测（证据 `证据/log/raw/p4_layer_probe_20261009_204744/`）——
  0.1m 见方的箱子放在机体 1.0m 处，原始 `/rslidar_points` 八方位各有 1659–2221 点，
  而 `/cloud_registered` 在 **0°/90°/180°/270° 四个轴向一律 0 点**、四个对角 4–5 点；
  `/grid_map/occupancy` 0/0/19/12 属下游与累积残留。⇒ 失因层在 faster-lio 内部，**本档只问"哪一步"**。
- 纪律：按坑47，"候选"与"结论"分开写；未经 A/B 不点名环节；未经 C9 授权不动配置副本。

## 1 在册链路与参数（实测核对，不靠注释）

| 项 | 实际值 | 来源 |
|---|---|---|
| 仿真实际加载的配置 | **`ws/src/95_simulation/config/faster_lio_sim.yaml`**（不是 `10_slam/lite3_slam/config/faster_lio_outdoor.yaml`） | `95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch`；真机/离线跑才走 C1 那份 |
| `preprocess/blind` | **0.2**（坑13-R1 差异⑥，与 render `min_raylength 0.3` 同步） | `faster_lio_sim.yaml:38` |
| `preprocess/lidar_type` | **2 = VELO32 分支**（sim_adapter 契约；真机 C1 为 4 = C16Handler） | `faster_lio_sim.yaml:39` |
| `filter_size_surf` | 0.3（当前帧体素降采样叶子） | `faster_lio_sim.yaml:19` |
| `publish/scan_bodyframe_pub_en` | **true ⇒ `/cloud_registered_body` 已经在发**（pre-voxel、post-preprocess） | `faster_lio_sim.yaml:45`、`faster_lio/src/laser_mapping.cc:263` |

### §1 之**更正**（2026-10-10 01:4x，登记后自查发现；原表不回改，本节向前指向）

**上表第一行错了。** 我按 `faster_lio_sim.yaml` 头部自己的声明（"仿真加载这份"）推断加载关系，**没有去读 launch**；
实读 `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:7`：

    <rosparam command="load" file="$(find lite3_slam)/config/faster_lio_outdoor.yaml"/>

⇒ **在册仿真链跑的是 C1／真机那份**（`faster_lio_outdoor.yaml`：`preprocess/blind: 1.0`、`lidar_type: 4`＝C16Handler），
`faster_lio_sim.yaml`（blind 0.2／lidar_type 2）只有 `10_slam/lite3_slam/launch/10_slam_mapping_sim.launch:8` 会装载，
**而我们的链不用它**。因此本档 §2 的三条候选要按新事实重排：

- **A 案（`blind`）从"可排除"升为首要嫌疑**：我原来写"`blind=0.2` ⇒ 1.0m 处的箱子不该被它吃掉"——**前提就用错了值**。
  真值 **1.0m**：注入箱心在 1.0m、**近面在 0.6m** ⇒ 大面积近面回波全在剔除半径内。
  而且 `blind` 是**距离**判据、各向同性，但**箱子有 0.8m 尺度**：轴向方位时正对的是宽面（回波集中在 0.6–1.0m ⇒ 全删），
  对角方位时正对的是棱（回波铺到 1.0m 以外 ⇒ 留下几个）⇒ **这正好能造出"四轴向全盲、四对角有回波"的对称图案**。
- **B 案（体素）与 VELO32 分支案**：VELO32 一支**不适用**（我们走 C16Handler），此前子智能体报告里关于 VELO32 的
  逐条结论**不得搬进本链**（我核过 launch 才发现这点）。
- **新线索（同属 A 案）**：`/rslidar_points` 整帧 20000 点 → `/cloud_registered_body` 2361 点（两条独立链同型 11.8%）。
  与 `blind` 剔除一致——插件把**没有命中的射线也编码成 (0,0,0) 点**（子智能体读 `livox_laser_simulation` 得到，
  我尚未自证），这些点距原点 0m、必被 `blind` 删掉；`point_filter_num` 默认 2 再砍一半 ⇒ 20000→~10000→~2361 量级吻合。

**取证纪律更正**：凡"哪份配置生效"这类事实，**只准引用 launch 的行号**，不准引用被加载文件自己的注释；
本档 §1 的错就是违反了这条（坑43 同族：把声明当实测）。R76 的 `rmin–rmax` 打印正是为这一条准备的实测。


## 2 三个候选环节（并列，未排序出唯一答案）

**A｜preprocess**：`point_filter_num_` 抽样（`src/pointcloud_preprocess.cc:57/94/186/203`，默认 2，配置里没写这个键）
＋ `blind` 最小距离剔除（VELO32 分支 `:186–188`，`range = x²+y²+z²`，与 `blind²` 比）。
**`blind=0.2` 时 1.0m 处的箱子不该被它吃掉** ⇒ A 只剩"抽样"与 VELO32 分支对合成 `time/ring` 的处理两条。

**B｜`voxel_scan_`（PCL VoxelGrid，叶子 `filter_size_surf=0.3`）**：`laser_mapping.cc:132/226` 设叶子、`:295–300` filter。
本 fork **没有** `DownsampleBodyScan`／`usingVoxel` 这类"按到机体距离取舍"的函数；PCL 的规则是
**每格输出 1 个点、位置取该格均值（centroid）**（`/usr/include/pcl-1.10/pcl/filters/voxel_grid.h:169`），
且没设 `setMinimumPointsNumberPerVoxel` ⇒ 非空格必出 1 点。
⇒ 0.1m 的小箱体与**地面/墙**点同处一格时，那 1 个代表点可能被均值拉到 0.55m 半径之外——
这与"轴向 vs 对角"的对称性**可以自洽**（同一格内的地面点数量与方位有关），是目前的**首要嫌疑**。

**C｜下游 SCAN 建图面**：`cloud>0 且 map=0` 才归它。R72 的四个轴向是 `cloud=0` ⇒ **不归 C**（但 180°/270° 的 map=19/12 属累积，另一回事）。

## 3 排除掉的猜测（写清楚，免得下次再查一遍）

- **不是发布面选点**：`/cloud_registered` 发的是整帧 `scan_down_world_`（`laser_mapping.cc:713–730`，`dense_publish_en: false`），
  逐点填充在 `:505–506` **无条件**执行；`esti_plane_threshold`／`point_selected_surf_`（`:584–600`）与 `corr_pts_`（`:607–621`）
  只影响 `/cloud_registered_effect_world`（`:264/777`）与 EKF，**删不掉 `/cloud_registered` 的点**。
- **没有"车体柱体剔除"**：`det_range_`、`cube_len_` 被读但**从不使用**（`:76–77/170–171`，`laser_mapping.h:91–92`）；
  全仓没有自遮挡/FOV 角度掩膜；`fabs(x/y/z)` 的三轴比较只出现在 `MapIncremental`（`:518–520`，**地图插入**面，不在发布帧上）。
- **体素哈希本身不会清零**：`ivox3d.h:284–285` 的 `Pos2Grid = (pt * inv_resolution_).round()`（格心在 0.3k，边界 0.15+0.3k，
  ⇒ 1.00m 落进格 3、跨度 0.75–1.05）；`AddPoints`（`ivox3d.h:257–281`、`ivox3d_node.hpp:118–121`）是 `emplace_back` **全量追加**、
  不平均不覆盖不设上限；`NEARBY18`（`ivox3d.h:218–222`）是轴对齐邻域，只影响地图查询不影响发布帧。

## 4 顺手抓到的一处**在册注释失实**（独立于 O-9，属坑43/坑6 族）

`10_slam/lite3_slam/config/faster_lio_outdoor.yaml:12` 写
`filter_size_map: 0.3  # fork未使用(c16.yaml注释证实),纯装饰键,回退减误导`——**失实**：该键在 `laser_mapping.cc:514` 与 `:518–520` 被实际使用
（决定"要不要把点加进地图"的三轴距离判据）。仿真那份 `95_simulation/config/faster_lio_sim.yaml:18` **没有**这句错注释，
只写"与 C1 现值一致"⇒ **错误认知是按值传导的**：当年"回退 `filter_size_surf` 而不提 `filter_size_map`"的判断，
建立在"它没被用"这句失实注释上。**本档不改注释**（改配置注释属 04 登记项），只登记事实。
待裁：是否单开一条小批改注释＋复跑隔离自检（零参数值改动）。

## 5 判别实验（**不授权就不做**；每条都写明要哪一次 A/B 才准点名环节）

| 实验 | 动什么 | 能分开谁 | 代价／风险 |
|---|---|---|---|
| **E0（已跑，零改动）** | 订已在发的 `/cloud_registered_body`（pre-voxel）做第四层计数 | A（preprocess 吃）vs **B（体素吃）** | 零配置改动、零门值；R74 本批 |
| E1 | 配置**副本** `filter_size_surf: 0.05` 复跑 E0 的失败方位 | B 的确证（代表点假设） | 动 `95_simulation/config/` 新增件 ⇒ **等 C9**；跑完即回退＋前后 sha |
| E2 | 副本 `point_filter_num: 1` | A 的抽样支 | 同上，等 C9 |
| E3 | 代码级一行日志（`pointcloud_preprocess.cc` 拒绝分支打 range） | A 的 blind/VELO32 支 | **动第三方副本源码** ⇒ 与"原样复制一字不改"硬规则冲突，须走 04＋翻案流程，本档**不建议** |

## 6 若 E0/E1 成立，对 P4 与在册口径的后果（预先写清，不临场改口）

1. O-9 从"上游能力不稳"改判为"**我方建图参数对近场小目标的分辨率不足**"——`SEE_MIN_PTS` 无论取多少都看不见（云里没有点），
   ⇒ 请求单 A14 的"按方位分档调阈值"一案无效（已随 R72 登记）。
2. 真机侧同参数（C1 那份 `blind: 1.0`、`filter_size_surf` 现值）会**更容易**吃近场小目标（blind 更大）
   ⇒ 这条要在 G8 真机窗口前交给小张/主理人，属于**能力包线**（喂 X5／C5），不是 P4 的销项。
3. 但**不得据仿真实测断言真机必盲**（外参/IMU/图案三者都不同，R72 已把这条边界写在证据 index §四）。

---

## 7 更正节（R83，2026-10-10 04:5x）：本稿引用的**生效行号**错了（`:187` → `:208`），机理与结论不变

**更正什么**：本稿 §1／X23／X24／H 稿都把 `blind` 的逐点剔除写成 `pointcloud_preprocess.cc:187`。
**逐函数边界实测**（`grep -n "^void PointCloudPreprocess::"` ＋ `sed` 直读，不靠注释）：

| 行 | 所属函数 | 判据文本 | 本链走不走 |
|---|---|---|---|
| `:71` | `AviaHandler`（40–84） | `… > (blind_*blind_)` 才置 valid | 否（`lidar_type` 非 1） |
| `:99` | `Oust64Handler`（85–115） | `if (range < blind_*blind_) continue;` | 否（非 3） |
| **`:187`** | **`VelodyneHandler`（116–193）** | `if (range < blind_*blind_) continue;` | **否（非 2）** |
| **`:208`** | **`C16Handler`（194–221）** | `if (range < blind_*blind_) continue;` | **是——`faster_lio_outdoor.yaml:33 lidar_type: 4` ⇒ C16** |

分发口在 `:19-38` 的 `switch (lidar_type_)`；enum 定义 `include/pointcloud_preprocess.h:87`
（`AVIA=1, VELO32=2, OUST64=3, C16=4`）。⇒ 我引的那行**代码文本几乎一样**（所以机理、"距离剔除"的定性、
以及 R80 量出的刀口数值**全都不受影响**），错的是**它属于本链不走的那个分支**。

**为什么值得单独更正而不是悄悄改**：这四处判据**长得一样**，悄悄改掉行号就会掩盖掉一件真问题——
**将来做盒过滤时，只改生效那处（`C16Handler`）就够了，但注记必须写明"只覆盖 C16 分支"**；
否则哪天 `lidar_type` 一变（例如换真机另一颗雷达或走 sim_adapter 的 VELO32 路），
盒过滤**静默不生效**、`blind` 又回来，而且没有任何门禁会响。⇒ 这条写进 **C13** 的实施约束。

**纪律加严（这是 X29 的实质内容）**：引用代码位置时**行号＋所属函数名＋enum/分支条件三件齐**，只给行号不合格。
本坑不是新错误类型，而是 R76 那条"哪份配置生效只准引用 launch 行号"的**同族延伸**——
当时我按被加载文件的自我注释推生效点、栽过一次；这次改成读源码又栽在"读了同名分支"上。
**共同点：只锚定"文本长什么样"，没锚定"这段会不会被执行"**。⇒ 凡引用实现位置，必须同时给**执行路径证据**
（这里是 `switch` 分发＋配置值＋enum 定义三件）。

**落法（按在册规矩：原文不回删，加更正标注）**：X23／X24 行内已加〔R83 更正〕；H 稿 §…处已加〔R83 更正〕；
本稿加本节；**`note/进度/00_裁定请求单` 的 M 段与 04 台账 R76／R77 两条原文不回改**（在册读数面），
只在 04 加 R83 更正条并向前指向本节。零代码、零配置、零门值改动；SM/LIB 未动 ⇒ **代际仍 H14**。
