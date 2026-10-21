# t 稿 · perlin3d 超时定位·步骤①：patience 的生效路径、55 码的权威定义，以及"两种失败形态不是一条成因"

更新时间：2026-10-10 16:05（CST）｜施工：小明｜性质：**离线读数＋在册件阅读**，零改动、零链窗、未起服务；代际仍 H14。
对应待办 **#67**（3D 全局规划红线未过，perlin3d 6/8）。步骤②（诊断副本上把 patience 提一档复跑那两对）**需主理人一句话**，本稿不越去做，只把②的前提与**限定**算清。

---

## 1. 生效路径（按完整路径与行号给，X29 纪律）

| 事实 | 出处（可 grep 回原文） |
|---|---|
| 全局规划侧的 patience **现值＝3.0**，且它是**被加载的那份** | `ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml:34 planner_patience: 3.0` |
| 加载者＝服务器 launch 的 rosparam load | `ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch:9` |
| 另有一份 `move_base_params.yaml:11 planner_patience: 3.0`，**自述是"回退底稿"**（`:1`），且**全 `ws/` 内无任何加载者**（grep `move_base_params` 于 `*.launch/*.yaml/*.sh` 只命中注释与它自身头行） | `ws/src/40_navigation/lite3_navigation/config/move_base_params.yaml:1/:11` |
| ROS 参数名三个：`planner_patience`／`controller_patience`／`recovery_patience`（不是凭记忆，是从装好的服务器库里读串） | `strings /opt/ros/noetic/lib/libmbf_costmap_server.so` |
| C++ 侧成员叫 `patience_`（ros::Duration） | `/opt/ros/noetic/include/mbf_abstract_nav/abstract_planner_execution.h:292` |

- **勘误我自己**：#67 任务描述里我写的「patience_D」是**把成员名记错**——在册头文件里它是 `patience_`（带下划线，无 D）。ROS 参数名则是 `planner_patience`。**名与层要一起给**，否则又是一次"指认了名字、没指认载体"。

## 2. `outcome=55` 到底是什么码（这决定它归谁管）

- `mbf_msgs/GetPathResult.h:111` ⇒ **`PAT_EXCEEDED = 55u`**；对照同名码：`/opt/ros/noetic/include/mbf_msgs/ExePathResult.h:126` 是 **103**、`/opt/ros/noetic/include/mbf_msgs/RecoveryResult.h:87` 是 **152**。
- ⇒ 55 属 **GetPath＝全局规划侧**被 patience 掐断，与控制器/恢复无关；文案 `mbf_utility/get_path_exception.h:22` 原文 `"Patience exceeded"`。

## 3. 读数（两份在册 CSV，原样统计）

| 批次 | 行数 | outcome 分布 | 成功例规划耗时 `dt` | 55 例的 `dt` |
|---|---|---|---|---|
| W2（`证据/log/matrix_w2/T1_perlin3d_results.csv`） | 8 | **6×0 ＋ 2×55** | **0.0395–0.1543 s** | 3.155／3.165 s |
| W3（`…/matrix_w3/T1_perlin3d_results.csv`） | 8 | **5×55 ＋ 3×−1，零成功** | —（无成功例） | **3.013–3.054 s** |
| −1 那三行 | — | message 原文 `30s 无 result`，`dt=30.001–30.002` | — | — |

## 4. 三条结论（每条都带"我不能顺带说"的限定）

1. **失败点贴着上界，不是"差一点"**：6 个成功例都在 0.16 s 内出计划，而所有 55 例都落在 **3.01–3.17 s**＝正好被 3.0 s 掐断。
   ⇒ 这告诉我们的是"这些 goal 对**从没在 3 秒内返回过**"，**不是**"它们需要 3.2 秒"。
   **限定**：因此"把 patience 提到 5 或 10 就能过"这句**没有证据**——提高上界只改变"多久放弃"，不改变"会不会返回"。
   步骤②如果要做，它的**真问题**是"再等一等到底等不等得到"，读数本身就是答案，不是判据。
2. **红线未过有两种形态，混不得**：W3 里除 5 例 patience 掐断外还有 **3 例 `−1 30s 无 result`**——那是**服务器连自己的 patience 中止都没回话**（30 s 是 harness 自己的等待上界），属服务/挂起型，**与 patience 参数无关**。
   ⇒ 所以 #67 原文里"两对失败＝patience 超时"这句**只覆盖 W2 那两行**；对 W3 必须分列，否则把一条"参数问题"的结论套到另一型上。
   **这条也修正我任务描述里的说法**：`#67` 写"不是算力不够，而是 patience_D 超时"，对 W3 而言**证据不足**。
3. **在册探针读了未被加载的那份副本**：`matrix_w3/scripts/w3_t1_probe.sh:79` 用
   `grep -n "planner_patience" …/config/move_base_params.yaml`——那件是**回退底稿、无加载者**（见 §1）。
   今天两份的值都是 3.0 ⇒ **历史读数没有数值错**，但**载体错**：若有人只改 `mbf_navigation.yaml:34`（生效那份），这行探针会照样打印 3.0 并让人以为参数没动。
   **处置**：该脚本是**已归档证据目录里的历史原件** ⇒ 按"补不覆"我不改它；若要复跑，新写件并读生效件（本条即为其依据）。

## 5. 步骤②的落笔输入（等主理人的一句话，我不自决）

- 需要她定的其实只有一句：**允许不允许在"诊断副本配置"上把 `planner_patience` 提到一档（如 10.0）复跑那 8 对**。
- 若允许，我把成本控制在一个窗内并**不碰生产配置**。副本落点有两个候选，**我不自选**（涉及新增配置件的位置）：
  ①`config/variants/` 已存在，内含 `crowd.yaml`／`night.yaml`／`outdoor_open.yaml` 三份场景变体，但**当前无 launch 引用它们**（grep `variants` 于 `lite3_navigation/launch/*.launch` 零命中）⇒ 放进去等于再增一份"无加载者的副本"，与 §1 那条坑同型；
  ②新立一份诊断专用件＋在那一次跑的 launch 里显式传参（跑完即撤），可避免第二种"改了不生效"的副本。
  无论哪式：**门值/判据一字不动**，`patience` 属**参数**不属判据（边界照 C9 类先例）；改前留底 sha 按 05 A1。
- 预期读数的三种收法（预先写清，免得跑完再找说法）：
  ① 提到 10 s 后仍 `55`（dt≈10.0）⇒ 判"全局规划对这些 goal 对**不返回**"，**参数问题排除**，二期判断维持"能力问题"；
  ② 提到 10 s 后出计划（dt 落在 3–10 s）⇒ 才是"参数问题"，红线可达性重估；
  ③ 那 3 例 `−1 30s 无 result` **不因 patience 变**（它们连中止都没回）⇒ 无论①②都要**另立一型**追（服务/挂起），不能算被②解释掉。
- 一条**我本轮不做、但有在册根据**的替代：`global_planner_params.yaml:8 allow_unknown: false` 与 `:11 default_tolerance: 0.0` 这两个字段会直接影响"远角反向对"是否被判不可达，
  它们在 parameter 服务域已实测落位（`note/04_配置改动diff.md:39`：「P6/Q11-⑤ 联调实测：`/move_base_flex/GlobalPlanner` 参数域 16 键全落位（含 `default_tolerance`/`allow_unknown`）」）。
  **自曝（本轮第三次同类，一起入 04 R95）**：这一句我初稿写的是"坑42 段 `:314` 记的就是 allow_unknown 不读未知"——回原文核对后**两处都不成立**：坑42 在 `note/02_问题与坑.md:452` 且讲的是"从现役件抽式子锚不唯一"，与 `allow_unknown` 无关；`:314` 那行是坑50 族的内容。⇒ 我只保留能核的表述（16 键落位的实测条），**"不读未知"这句语义我没有在册根据，删掉不再顺带说**。
  ⇒ 若②读数落在 §5 的 ① 型（提到 10 s 仍不返回），**下一个要查的是这两个字段而不是 patience**；本步骤内我不动它们。

## 6. 本稿不声称的事

- 不声称 patience 是根因（§4.1 明确"没证据"），也不声称它不是（要②才知道）。
- 不声称红线可达性或二期判断变更（那是 #7/#19/#20 的口径，且 #67 的 ③ 步要等 ②）。
- 不改任何在册件：`mbf_navigation.yaml`、`move_base_params.yaml`、`mbf_navigation.launch`、`w3_t1_probe.sh`（历史证据原件）全部一字未动；未编译、未起服务、未占端口。
- 代际仍 H14（SM/LIB 未动）。P8＝**无新增补丁行**（离线定位与限定，不改路线/门值/能力边界；§4.2 对我自己任务描述的修正属台账面）。
