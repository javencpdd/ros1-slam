# v 稿 · 已入库台账引用黄红名单的勘误对照表（22 项逐条：当时写法 → 核实真值 → 是否牵动读数；原文一律不回改）

更新时间：2026-10-10 18:1x（CST）｜施工：小明｜性质：**离线只读核实**——只 `sed/grep/git show` 回原文，零改动（未动任何件/配置/门值/门禁串/台账正文）、零链窗；SM/LIB 未动 ⇒ **代际仍 H14**。
取证件与它的输出：[../证据/log/raw/r102_ledger_citation_table/verify_ledger_citations.sh](../证据/log/raw/r102_ledger_citation_table/verify_ledger_citations.sh)（rc=0）＋ [verify_ledger_citations.out](../证据/log/raw/r102_ledger_citation_table/verify_ledger_citations.out)（47 条 `⇒` 实证行，无一条空取证）。
名单口径：`VCC_MAX=200 bash scripts/verify/v_citation_check.sh` 缺省扫描面（04／02／总览／SCAN集成设计稿）去重后 **RED 1 ＋ AMBER 21 = 22 项**。

---

## 0. 这张表存在的理由，以及它明确不做的事

坑58（引文先写后核）与坑59 追加二（「我知道该核」不等于「我会去核」）之后，VCC 能把台账里的引用形状扫出来，但**扫出来只是第一步**：黄标到底是"我写错了"还是"件后来变了"还是"名写得简、内容其实对"，三者处置完全不同——
第一种要勘误，第二种要立规矩，第三种**不该动文**。把它们混成一句"引用待核"，等于让下一个人重新核 22 遍。

本表做三件事：① 逐条给**核实后的真值**（原样行文本，不是我概括）；② 逐条判**是否牵动任何在册读数/结论**；③ 逐条附**能重放的命令**——你不需要相信我的任何一行结论。

不做的事：**不回改已入库台账正文**。这是在册规矩（原文＋另起勘误句），改了就抹掉"当时我是这么写的"这一层证据，也正是坑58 修法④要求保留原数的理由。
唯一的例外面已在 #87 处理完（活文档＝设计稿当场改形，因为设计稿是本轮的活件、且改动只涉引用形状不涉读数）。

---

## 1. 五个型，22 项各归其一

| 型 | 形状 | 项数 | 该怎么处置 |
|---|---|---|---|
| 甲 | 行号近距差行（≤4 行，键名与内容都在同件、只差行） | 4 | 勘误本表即可，读数不动 |
| 乙 | 转述错引（键名/编号对不上且真值远） | 3 | 其中 2 条坑58 当场已自曝并给正解，本表补"原样行文本" |
| 丙 | **只给文件名**、没给包路径／同名前缀（坑52 型） | 12 | 内容全部核实为**对**，不改文；规矩面归 X29/坑52 |
| 丁 | **版本漂移**：写下时行号是对的，被引件后来长行 | 2 | 不是错，是引用没带版本 ⇒ 立坑60 |
| 戊 | 红：路径写短被解析到同名他件 | 1 | 04:750 已在册勘误，本表补真值行文本 |

合计 4+3+12+2+1 = 22 ✓（与 VCC 名单逐项对齐）

---

## 2. 逐条对照表

「牵动判定」列只回答一个问题：**这条改了/不改，会不会让任何一条在册读数或结论变样**。全部 22 项的答案都是**否**，理由逐条写在该列。

### 甲类·行号近距差行（4 项，出处：04）

| 当时写法 | 核实真值（原样行文本） | 牵动判定 | 重放命令 |
|---|---|---|---|
| `ws/src/95_simulation/mockamap/src/maps.cpp:599`（键名 block） | `:599` = `{`；**`:600`** = `Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);` | 否——键名与语义均对，仅行号差一 | `sed -n '599,600p' ws/src/95_simulation/mockamap/src/maps.cpp` |
| `ws/src/95_simulation/config/faster_lio_sim.yaml:29`（键名 mapping.extrinsic_est_en） | `:29` = `extrinsic_T: [0.0, 0.0, 0.0]`；**`:30`** = `extrinsic_est_en: false` | 否——该键的值/注记引用无误 | `sed -n '29,30p' ws/src/95_simulation/config/faster_lio_sim.yaml` |
| 同件 `:37`（键名 preprocess.blind） | `:37` = `preprocess:`；**`:38`** = `blind: 0.2` | 否——同上 | `sed -n '37,38p' ws/src/95_simulation/config/faster_lio_sim.yaml` |
| 写成 `ws/src/40_navigation/lite3_navigation/config/global_planner_params.yaml:12` 带键名 allow_unknown | `:12` = `publish_scale: 100`；**`:8`** = `allow_unknown: false` | 否——坑58③ 已自曝，正解已在册 | `sed -n '8p;12p' ws/src/40_navigation/lite3_navigation/config/global_planner_params.yaml` |

### 乙类·转述错引（3 项）

| 出处·当时写法 | 核实真值 | 牵动判定 | 重放命令 |
|---|---|---|---|
| 04：`note/02_问题与坑.md:314` 说"坑42 记的就是 allow_unknown 不读未知" | `:314` = `- 现象：我给 m2_batch_runner.sh 开了 SM_CMD／SM_ARGS 两个口子…`（与坑42 无关）；**坑42 在 `:452`** = `## 坑42 · 「从现役件抽式子来测」的锚不唯一、又不按行取 RHS ⇒ 静默抽到别家表…`，讲的是抽式子，**不是** allow_unknown | **否，但这是全表最坏的一条**：它不只是行号错，而是**虚构出处＋语义张冠李戴**。t 稿的结论已按坑58②③ 删掉"不读未知"，因此今天在册的结论里没有依赖它 | `sed -n '314p;452p' note/02_问题与坑.md` |
| 02：`ws/src/40_navigation/lite3_scan/launch/70_scan.launch:61`（键名 remap） | `:61` = `<arg name="cy" value="256.4806823730469"/>`；**`:76`** = `<remap from="cmd_vel" to="/scan_planner/cmd_vel"/>` | 否——remap 的存在与目标名无误 | `sed -n '61p;76p' ws/src/40_navigation/lite3_scan/launch/70_scan.launch` |
| 04:237 写成短路径，被 VCC 解析到 `ws/src/CMakeLists.txt`（该件仅 71 行 ⇒ 192 越界）⇒ **RED** | 见戊类第 1 行（真所指＝lvr2 件） | 否——04:750 已在册勘误 | 见戊类 |

### 丙类·只给文件名（12 项，**内容全部核实为对**）

| 当时写法（简写） | 核实真所在＋该行原文 | 牵动判定 | 重放命令 |
|---|---|---|---|
| 写成 `costmap_navigation_server.h:236/:239`（04，只给文件名） | `/opt/ros/noetic/include/mbf_costmap_nav/costmap_navigation_server.h`；`:236` = `pluginlib::ClassLoader<mbf_costmap_core::CostmapRecovery> recovery_plugin_loader_;`；`:239` = `pluginlib::ClassLoader<nav_core::BaseLocalPlanner> nav_core_controller_plugin_loader_;` | 否——加载器三元组的引用内容对 | `sed -n '236p;239p' /opt/ros/noetic/include/mbf_costmap_nav/costmap_navigation_server.h` |
| 写成 `nodelet.cpp:68`（04／总览／fd 稿各 1，共 3 项，只给简写名） | 实名 `ws/src/20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp`；`:68` = `pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);` | 否——"第三方实发绝对名 /odom"这条核心事实成立；批4 FD-H 未改该件（在册断言同判） | `sed -n '68p' ws/src/20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp` |
| 写成 `GetPathResult.h:111`（总览，只给文件名；2 项去重后 1 计 2） | `/opt/ros/noetic/include/mbf_msgs/GetPathResult.h:111` = `PAT_EXCEEDED = 55u,` | 否——W1 口径①与 t 稿的 55 码义都以此为准，数值确认无误 | `sed -n '111p' /opt/ros/noetic/include/mbf_msgs/GetPathResult.h` |
| 写成 `ExePathResult.h:126`（04／总览，只给文件名） | 同目录 `:126` = `PAT_EXCEEDED = 103u,` | 否——C11 透传码系对账用的就是它 | `sed -n '126p' /opt/ros/noetic/include/mbf_msgs/ExePathResult.h` |
| 写成 `RecoveryResult.h:87`（04／总览，只给文件名） | 同目录 `:87` = `PAT_EXCEEDED = 152u,` | 否 | `sed -n '87p' /opt/ros/noetic/include/mbf_msgs/RecoveryResult.h` |
| 写成 `move_base_action.h:116`（总览 W1 行，口径①的锚） | `/opt/ros/noetic/include/mbf_abstract_nav/move_base_action.h:116` = `// copy outcome and message from action client result`。**必须给包名**：`mbf_msgs/MoveBaseAction.h` 是 actionlib 生成件、同名易混（我本轮就先打开了错的那件，取到 `: FalseType`） | 否——注释原文逐字吻合；配套两码表亦确认：`MoveBase.action:24 uint8 OSCILLATION = 13`、`ExePath.action:27 uint8 OSCILLATION = 105` | `sed -n '116p' /opt/ros/noetic/include/mbf_abstract_nav/move_base_action.h` ＋ `grep -n OSCILLATION /opt/ros/noetic/share/mbf_msgs/action/MoveBase.action /opt/ros/noetic/share/mbf_msgs/action/ExePath.action` |
| `fresh{2,3,4,5}n.log:23/27/19/23`（fd 稿:55，4 项） | 真名带前缀、同目录 `note/进度/SCAN集成/证据/log/raw/`：`m2r5_fresh2n.log:23` = `SMOKE teb_state=3 dt=8.5s dev_odom=0.166 dev_gt=0.166`；`3n:27`＝0.176/0.176；`4n:19`＝0.170/0.170（dt=8.6s）；`5n:23`＝0.173/0.173。**首条 `m2r5_fresh1n.log:19`＝0.172/0.172 已给全路径**，后四条是同目录省写 | 否——**五个数逐一吻合**，fd 稿"逐位相同"的读数是实的；只是省写名让 VCC 找不到件 | `sed -n '23p' note/进度/SCAN集成/证据/log/raw/m2r5_fresh2n.log`（其余同形） |

> 丙类的共性：**指认了名字、没指认载体**（坑52 的原话）。这一类**不该改文**——行文本一核就过，改了反而把"当时能简写"的语境抹掉；该改的是规矩（X29 已要求"行号＋名一起给"，本表补一条：**系统件必须给包路径，证据件必须给目录**）。

### 丁类·版本漂移（2 项，出处：fd 稿）——这一类不是错，是缺"版本"这一维

| 当时写法 | 核实 | 牵动判定 | 重放命令 |
|---|---|---|---|
| 写成 `hdl_localization.launch:89` 带键名 enable_robot_odometry_prediction=false；`:38/:80/:89-90 定 odom_child_frame_id=base_link` | 现 HEAD（124 行）`:89` = `<param name="use_imu" …>`；键名现在 `:52`（arg 默认 `false`）与 `:95`（param），`odom_child_frame_id` 在 `:38`/`:86`。**`git show dfb3ef0:…`（118 行版，M2-R2C）里 `:89` 正是 `enable_robot_odometry_prediction` 那行、`:80` 正是 `odom_child_frame_id`** ⇒ 写下时是对的，长行发生在它之后：`101→111→118→124`，**最后一跳 118→124 就是批4 FD-H 自己往该件加注记** | 否——两个语义断言按键名 grep 依然成立（prediction=false、child=base_link）；不动任何读数 | `sed -n '89p' ws/src/20_localization/lite3_localization/launch/hdl_localization.launch` ＋ `git show dfb3ef0:ws/src/20_localization/lite3_localization/launch/hdl_localization.launch \| sed -n '80p;89p'` |
| fd 稿:55 的机读点 `m2_smoke.sh:193 取 /Odometry、:379 算 dev_odom、:380 算 dev_gt` | 现 HEAD（616 行）`:193` 是 Stage B 分节注释、`:379/:380` 是 E3 打印；真值现在 **`:129` 订阅 `/sim/odom_gt`**、**`:404` 算 d**、**`:405` 算 gd**、`:406` 打印。**`git show 3c12fec:…`（M2R6，568 行）里这三行逐字就是所指** ⇒ 同为漂移；后续 M2R7／批1 行 C／R26-28／H14 四轮插行 | 否——五条读数是日志里的那五个数，与 SM 后来长行无关。**但要点名一处**：`取 /Odometry` 这个说法在**批1/F-D 行 C** 之后已改订阅名（`:129`），语义归 F-D 在册裁定；`d` 与 `gd` 至今同源（`S['odom']←/sim/odom_gt` 是插件世界位姿、`S['gt']←/gazebo/model_states` 是同一模型世界位姿），fd 稿"结构上不含估计层误差"的结论因此**更强而非更弱** | `sed -n '193p;379p;380p;404,406p' ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh` ＋ `git show 3c12fec:ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh \| sed -n '193p;379p;380p'` |

### 戊类·红（1 项）

| 当时写法（04:237） | 核实真值 | 牵动判定 | 重放命令 |
|---|---|---|---|
| 04:237 写成 `lvr2 CMakeLists.txt:192`（find_package(GSL REQUIRED) 无开关）——VCC 把短路径解析到 `ws/src/CMakeLists.txt`（**仅 71 行 ⇒ 越界红**） | `opt/upstream/lvr2/CMakeLists.txt:192` = `find_package(GSL REQUIRED)`，**逐字吻合**；W2"编译链当前不可用（缺 libgsl-dev）"的结论以此为锚，锚是实的 | 否——04:750 已在册勘误并写明真所指；本表只补行文本 | `sed -n '192p' opt/upstream/lvr2/CMakeLists.txt` ＋ `wc -l ws/src/CMakeLists.txt` |

---

## 3. 结论（四要素口径）

- **实现状态**：零改动。本表是核实件，不是整改件；22 项**没有一项需要改台账正文**（甲/乙/戊的勘误已在册或本表承载，丙内容本就对，丁当时本就对）。
- **自动化验收**：取证件 rc=0（47 条实证行、零空取证）。VCC 终态 `文档=20 单点引用=425 OK=330 RED=1 AMBER=25 勘误句=69 非单点形状=121 rc=6`，红/黄**集合与本轮开工前逐项同**（本表只承载勘误、不改台账文）；本稿自身 0 红 0 黄——第一版曾因转述错引未加在册 FIXUP 标记而被 VCC 打 2 红 6 黄，补 `写成` 标记后归零（⇒ **勘误表本身也是引用面**，它同样该过闸；这条已记入 04 R102 与 02 坑60 连带笔）。SYN/GTC/ISO/DC 与两处自造红的处置见 04 R102。
- **目标环境验收**：不适用——纯离线只读，零节点零链窗。
- **证据路径**：[../证据/log/raw/r102_ledger_citation_table/](../证据/log/raw/r102_ledger_citation_table/)（本件取证件＋输出＋index＋清单）。
- **新暴露的一格要立规矩**：丁类证明"引用正确"和"引用可用"是两件事——**行号是版本限定的，而台账引用不带版本**。⇒ 记 **坑60**（本轮新增），并给两条可机判的候选动作（**不自落，归 A15/#54/#16 那族**）：
  ① 引用**被引件在本轮/近期有改动**时，改写成"键名/锚文本 ＋ 行号"，或干脆只给 `grep -n` able 的锚文本；
  ② 需要钉历史时点的，**随引文给一个 commit**（如 `3c12fec:m2_smoke.sh:193`），因为 `git show` 能重放、而裸行号不能。

## 4. 交出去的两件（不代改）

fd 稿是小张的件、且本轮已过 #26 的收口边界，我不代改他的文。移交清单（**逐条已在 §2 丁类给出真值与重放命令**）：
1. `:89`/`:80`/`:89-90` 三处指针 → 键名现在 `:52/:95`、`:38/:86`（语义不变）。
2. `m2_smoke.sh:193/:379/:380` → 现行 `:129/:404/:405`，且"取 /Odometry"这句需按批1/F-D 行 C 改口径（现订阅 `/sim/odom_gt`）。
3. `fresh{2..5}n.log` 四处省写名 → 补 `m2r5_` 前缀或写成同目录相对全路径。
