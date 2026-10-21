更新时间: 2026-10-08 06:35

# M2R5 设计 · base_link 双父 TF 结构缺陷（U1 ＝ 小美路由项 R2）

- 作者：小张（部署架构师）。性质：**只做设计，不施工**。实现状态：**未施工**（本档不产生任何 ws/ 改动）。
- 派单来源：note/进度/v7/S2/M2/进度文档.md 「## M2R4 复核（小美，2026-10-08）」节（L332 起）——二节 ③ 成立（L368）、三节第 3 项「被测件即变、5 连必须重新起算」（L376）、四节 R2（L390）、五节候补句处置（L399–L404）。她的原文为唯一权威，本档所有引证均已回原件逐行核对。
- 简称沿用她的定义：**XAC** ＝ ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro；**PLG** ＝ 同包 src/lite3_planar_drive.cpp；**SM** ＝ 同包 verify/m2_smoke.sh；**G4/G5** ＝ note/进度/v7/S1/SCAN集成/证据/log/raw/m2r4_fresh4.log / m2r4_fresh5.log；**RSP** ＝ robot_state_publisher。

## 0. 结论速览

| 项 | 结论 |
|---|---|
| 推荐修法 | **方案 A**：XAC L32–L36 的 fixed 关节反向为 parent=base_link / child=TORSO，关节名改 base_to_torso，URDF root 变 base_link。**未施工** |
| 不选 B（插件改发 odom→TORSO） | 改动面跨 5+ 包且把 sim 契约帧改动外溢到共用配置（costmap robot_base_frame），并需重编译 .so |
| 不选 C（仅观测侧显式时刻查询） | 不动结构 ⇒ 满足不了 U5 两句结案条件；只能作为 A 之后的探针精度补强或 A 修后仍 HIT 的兜底件 |
| 是否触及生产/真机契约 | **否**（三条独立证据链见 §4） |
| spawn 高度语义 | 推理上不变（关节原点恒等 ⇒ root 与 TORSO 原点重合），但**以 §4 的三条可机批判据核，不作断言** |
| Gazebo root 变更的锚点/惯性合并副作用 | 未核实其实现（沙箱无离线 URDF→SDF 件、本轮禁启仿真）⇒ 给出 §4 的服务面判据（get_model_properties / get_link_properties / model_states z / 样张 sha256 哨兵） |
| 二次发生风险（小美 L370 末句） | **机理确认、边名否定**：hdl 不发 odom→base_link，它发 map→odom（分支 A）或 map→base_link（分支 B）——撞车面同样在 base_link 入边，且分支选择本身会被双父抖动。详见 §5 |

## 1. 我对缺陷面的独立复核（不照抄派单）

派单给的每一处我都实际打开核了，逐条对齐：

| 待核项 | 实测 | 出处 |
|---|---|---|
| TORSO 为 root 的声明 | 成立：L14 注释「官方骨架：TORSO（root，保持官方名）」、L15 link TORSO（L16–L20 inertial mass 15.0／ixx 0.3406／iyy 0.1806／izz 0.2810；L21–L25 visual box 0.42 0.31 0.22 于原点；L26–L29 collision 同尺寸同原点） | XAC L14–L29 |
| fixed 关节 torso_to_base | 成立：L32 joint name=torso_to_base type=fixed；L33 parent=TORSO；L34 child=base_link；L35 origin xyz 0 0 0 rpy 0 0 0（**恒等**，此点是 §4 spawn/惯量两问的关键） | XAC L32–L36 |
| base_link link 块 | L37–L42，含 inertial（L39 mass 0.01，L40 ixx=iyy=izz=1e-5）⇒ 反向后 root 仍带惯量（见 §2 A′ 的 WARN 讨论） | XAC L37–L42 |
| base_to_rslidar | L44–L48，parent=base_link／child=rslidar／origin z 0.38；rslidar 块 L49–L59（L58 注 D-M1-5 collision 整块删除留位） | XAC L44–L59 |
| 静态边完备性 | 全文件只有上述两个 fixed 关节 ⇒ RSP 落 /tf_static 恰两条边，与 G4 L13–L14、G5 L14–L15 的观测一一对应，无漏项 | XAC L32–L36/L44–L48；G4/G5 各行 |
| 插件动态边 | 成立：PLG L127–L128 sendTransform(tf, ros::Time::now(), odom_frame_, base_frame_)；L29 缺省 odom／L30 缺省 base_link；装配值 XAC L133 odometryFrame=odom、L134 robotBaseFrame=base_link（派单写「L133–L34」系笔误，实为 L133–L134）；L111 odom 头同基 now、L112 header=odom_frame_、L113 child=base_frame_ | PLG L29–L30/L111–L113/L127–L128；XAC L133–L134 |
| 原始话题实证 | 成立：G4 L11 hits=27 last_stamp=22.080 age=0.020（无 [ZERO]）、L12 base_link_parents=[odom] transforms_total=32、L13 static base_link->rslidar stamp=0.020、L14 static TORSO->base_link stamp=0.020；G5 L12 last_stamp=22.480 age=0.015、L13/L14/L15 同形态 ⇒ 发布侧时钟清白、动态父唯一、静态面有指向 base_link 的边 | G4 L11–L14；G5 L12–L15 |
| 历史意图（非笔误） | 成立：urdf/lite3_planar.urdf.xacro.before_M2.bak L9「base_link 为我方契约 frame，官方 root 是 TORSO → 以 fixed joint TORSO→base_link 恒等桥接，双 frame 并存」，同件 L20 记 TF 链「RSP 出 base_link→rslidar（fixed 0.38m）+ TORSO→base_link」；该件关节块在 L44–L46 | before_M2.bak L9/L20/L44–L49 |
| 关节名的下游引用 | **零引用**：全仓（ws/src + scripts + note）grep torso_to_base 只命中 xacro 本体与各 .bak 本体、无脚本/配置/代码消费关节名 ⇒ 改名安全 | 实测 grep |
| 边名的下游引用（谁在消费 base_link／TORSO） | 见 §4 全表 | 实测 grep |
| 派单路径勘误 | 派单写 ws/src/50_localization/lite3_localization —— **该路径不存在**；实为 ws/src/20_localization/lite3_localization（仓内 50 号段是 50_task）。本档一律按实测路径引 | 实测 ls ws/src |

一句话复述缺陷：**base_link 有两条入边**——RSP 的静态 TORSO→base_link（XAC L32–L36）与插件的动态 odom→base_link（PLG L127–L128）。tf2 一棵树里子帧只应有一个父，这正是 F3 L3 那条 TF_OLD_DATA（frame base_link，parent TORSO，authority /robot_state_publisher）的成因面。

## 2. 候选修法

### 方案 A（推荐）· 反挂：把桥的方向掉过来

**改动落点（唯一文件、唯一功能块）**

| 文件 | 行号 | 现态 | 目标态 |
|---|---|---|---|
| XAC | L32–L36 | joint name=torso_to_base；L33 parent=TORSO；L34 child=base_link；L35 origin 恒等 | joint name=base_to_torso；parent=base_link；child=TORSO；L35 origin 恒等**不改**（xyz 0 0 0 rpy 0 0 0 逐字保留） |
| XAC | L14 | 注释「TORSO（root，保持官方名）」 | 注释同步为「root=base_link（我方契约帧）；TORSO 为官方骨架名，恒等挂于其下」，并保留 before_M2.bak L9 意图的引用说明 |
| XAC | L3–L10 头部取舍声明 | 未提 root 变更 | 增一行「M2R5/U1：关节反向，语义等价、仅拓扑方向变」 |

不动的块（逐条点名，施工方不得顺手改）：L21–L29 几何、L37–L42 base_link inertial、L44–L48 base_to_rslidar、L49–L59 rslidar、L62–L104 雷达 sensor（L101 frameName=rslidar）、L107–L122 IMU sensor（L114 bodyName=rslidar／L115 frameName=rslidar）、L128–L139 驱动插件块（**L134 robotBaseFrame=base_link 保持 base_link**）、L141–L144 gazebo reference=TORSO 的 mu1/mu2。

**语义交代（历史意图如何保留）**：A 之后 base_link 与 TORSO **仍是双 frame 并存、仍恒等、base_link 仍是我方契约帧**——before_M2.bak L9 意图的三个成分一个都没丢，只把桥的方向从「官方 root 承我方帧」改为「我方帧承官方 root」。这与全仓既有桥接先例同构：hdl 外参以 base_link 为父（lite3_localization/launch/hdl_localization.launch L61–L62 args=… /base_link /rslidar）、 livox 驱动以 base_link 为父（livox_ros_driver2/launch_ROS1/msg_MID360s.launch L32）、SCAN 侧以 body 为父挂 base_link（30_3rd/plan_manage/launch/lio_relay.launch L57–L58 args=0 0 0 0 0 0 body base_link）⇒ 契约链里 base_link 一律作**父或叶**，从不被静态挂到别的机体帧下。A 是把 sim 的一个反例归队，不是新增约定。

**影响半径**：单文件单块；**不需重编译**（xacro 是 launch 期展开件，m1_gazebo.launch L23／m2_gazebo.launch L22 每次起窗现读；SM 的构建时间戳证据链 进度文档 L344 的「.so 未重编译」口径不受影响）。被测件仍「即变」⇒ 5 连照 L376 重新起算。TF 拓扑变化：改前静态分量 {TORSO→base_link→rslidar} 与动态边 odom→base_link 争同一子帧；改后单一连通树 odom→base_link→{TORSO, rslidar}，map→odom 由 FR（m2_tf_fresh.py，L2 L24–L28，10Hz）挂顶。camera_init→body（faster_lio 自发的两条边之一，laser_mapping.cc L700–L710）在 sim 域仍是**独立分量**，A 不改变它（stageA 无消费者需连通）。

**风险**：
1. RSP 的 KDL root-inertia WARN 改指帧名：改前多份证据件首行即「The root link TORSO has an inertia … KDL does not support a root link with an inertia」（scan_C9_stageB_run2_tebB_失败原文.log L264、gazebo.log L1、M2R2 hdlC.log L1 等）；改后 root=base_link 且其 inertial 在位（XAC L38–L41）⇒ 同一 WARN 改指名 base_link。核过仓内**无任何脚本/门控 grep 该串**（本轮 grep root link 的全部命中都在 note/进度/**/证据 日志件，ws/src 与 scripts 的 .sh/.py 零命中）⇒ 属可见文案变化，非功能变化。scripts/mapping/README.md L28 早已把该 WARN 记为「无害」。
2. Gazebo 侧 root 变更对模型锚点/合并惯量的影响：见 §4 第 3 问，我不作断言，给判据。
3. 若某处隐式假设「TORSO 是根」：核过 zero —— TORSO 在 ws/src 的非 URDF 引用只有 mapping_viz L10/L15–16（真机 viz 伴侣，独立 master 域）、m1_gazebo.launch L22 与 m2_smoke.sh L292 两条**注释**、以及 30_3rd/plan_manage/launch/advanced_param.xml L56–L58 的**几何注释**（讲碰撞盒尺寸与髋关节余量，不涉帧名；同域另件 lio_relay.launch L53 明文「规划器不消费 TF」）。
4. 合并后 link 名变化会改变 /gazebo/link_states 的条目名（若 lump 生效）：核过 harness 不消费 link 名——SM 三处 gt 取的是 /gazebo/model_states 按 model name=='lite3_planar' 过滤（SM L96、L175、L422），PLG 用 model->WorldPose()（L73、L109）⇒ 零依赖。

**验证判据怎么写**：结构面与数值面全部并入 §6（V1–V6），此处只列 A 专属的两条静态前置：
- A-1（静态展开等价）：在 source 过 noetic 的 shell 里用 rosrun xacro xacro 展开改前/改后 URDF 各一次到 /tmp，diff 必须**只有**关节块三行（joint 名、parent、child）与注释行不同；link 块、sensor 块、plugin 块、mu 块逐字节相同。工具在位性本轮已核：/usr/bin/check_urdf 存在；沙箱内**无** gz2sdf 类离线 URDF→SDF 件（find /usr /opt/ros/noetic -name 通配 z2sdf 结果为空）。
- A-2（root 判定）：对展开件跑 check_urdf（/usr/bin/check_urdf 本轮核在位），改前/改后各一次，要求输出的**根 link 名**由 TORSO 变 base_link；具体行文本以实际输出为准（本轮未实测其格式，见 §9 ⑤，不作断言）。

### 方案 A′（A 的可选加固支路，随本轮**不做**）· 反挂 ＋ 让 root 不带惯量

- 落点：在 A 之上再删 XAC L37–L42 的 base_link inertial（或按 KDL 建议加一条 world→base_link 的 dummy 锚链，即证据件里那句「you can add an extra dummy link to your URDF」的官方解法）。
- 收益：root-inertia WARN 归零（不改指帧名）。
- 风险：合并开关行为未核实（§4 第 3 问①）——若 gazebo 未把 TORSO 惯量并入 root，无惯量 link 进 ODE 要由 gazebo 补默认质量，可能引入新的 WARNING 或接触/姿态扰动，直接打 dynamic 模式（XAC L135）。
- 判据：与 A 同（§6 V6 的 (i)(ii) 两条服务面判据是 A′ 的前置门槛——只有当改后 WARN 改指帧名真的打穿了某个日志哨兵时，才值得为消 WARN 付这份物理不确定性）。
- 处置：**不推荐随本轮做**；哨兵面本轮核过为零命中 ⇒ 触发条件目前不存在。若小美/主理人认为该 WARN 属噪声，走「待主理人裁定」单独片。

### 方案 B · 插件改发 odom→TORSO（小美已判不推荐，我给理由）

**落点（每一处都是必改，缺一即树断）**

| 文件 | 行号 | 改动 |
|---|---|---|
| XAC | L134 | robotBaseFrame 由 base_link 改 TORSO |
| PLG | L127–L128 | 动态边 child 变 TORSO（经 L30 读键，实为装配值变更 ⇒ 上面那一行是必要条件） |
| PLG | L113 | /Odometry 的 child_frame_id 同步变 TORSO |
| XAC | L133 | 不动（odometryFrame 仍 odom） |
| 下游全链 | costmap_common.yaml L5（robot_base_frame）、move_base_params.yaml L7（回退底稿，L1–L2 注「仅存档不加载」）、hdl_localization.launch L38（odom_child_frame_id）、L61–L62（静态外参父帧）、SM L199/L281/L288/L310/L316–L317（判据帧名）、lite3_navigation 之外还有 20_localization/hdl_localization/launch/local_rslidar_imu.launch L9 | 若选 B 且要保持 sim/真机同契约 ⇒ 这些一起改；若只改 sim ⇒ 必须为 sim 另开一份 costmap/hdl 配置（新增分叉件） |
| 构建 | PLG 改后需重链 liblite3_planar_drive.so（先例：00_总览 L82「单包批 catkin_make -j2 白名单 lite3_gazebo_sim→.so 重链」）＋ 全量回归批清白 | 比 A 多一整层构建风险 |

**影响半径**：跨 5 个模块目录（95_simulation / 40_navigation / 20_localization / harness / 构建批），且**直接触碰生产契约**：costmap_common.yaml 与 move_base_params.yaml 是全 mode 共用的导航配置（非 sim 专用件），lite3_localization/launch/hdl_localization.launch 是 G8 真机 mode_localize 的实际承载件（mode_localize.launch L34 include 它）——把机体帧改成 TORSO 要么外溢到 G8，要么在 sim/生产之间劈成两套契约。
**风险**：真机官方 URDF 里根本没有 base_link（Lite3.urdf L9 root=TORSO；Lite3_prim.urdf L6 root=TORSO；两文件 grep base_link 命中数实测 0）——B 表面上「顺着官方 root 走」，实际是把 sim 的机体帧拉到与官方一致、而把**我方全部下游契约帧**（hdl 的 odom_child_frame_id、costmap 的 robot_base_frame、驱动 base2livox 的父帧、qnx2ros L149 的 child）留在 base_link 上 ⇒ 冲突只是从 TF 层挪到配置层。
**验证判据**：V1/V2 会变体（静态面无任何边、但 base_link 从 TF 树里彻底消失 ⇒ E3 的 base_link_parents 变空列表，V2 的静态行只剩 base_link->rslidar 一条也可能没有），且须新增「costmap/hdl 在 sim 域仍能取到 robot pose」的活体判据——即 G6 后置哨兵与 teb 的 Can not get the robot pose 类命中面重新解释。**判据成本高于 A，收益不比 A 多**。

### 方案 C · 仅观测侧显式时刻查询，不修结构

**落点**：SM L219–L226 的 _age_of，把 L220 的 buf2.lookup_transform(a, b, rospy.Time(0), rospy.Duration(2.0)) 改为显式当前时刻 rospy.get_rostime()（这正是小美 L304 的 E1 第②问、L307「若 E3 判 (a) 则 G5c1 改为显式时刻查询或删项」的落地件）。
**影响半径**：只动 harness 观测件（SM 内嵌 python ⇒ 触发 R5 两检：bash -n ＋ 抽内嵌段做 ast 解析，SM 内嵌段界 L166/L341）。零被测件改动 ⇒ 5 连不因此重新起算。
**风险（这是我不推荐它单用的全部理由）**：
1. 它改变的是**探针看到什么**，不是**图是什么**。U5 的结案两句（/tf 上 base_link 父帧唯一=odom、/tf_static 无指向 base_link 的边，进度文档 L385）是**结构断言**，C 一句都不满足 ⇒ 单用 C 解不了阻断。
2. 双父的实际后果面是 tf2 按权威丢边（F3 L3 的 TF_OLD_DATA 即该机理的一次可见命中，U4 已把「5 连 TF_OLD_DATA 须 0 次」设为条件）——C 不能把丢边概率降下来。
3. 消费端不能同步改：costmap/mbf/teb 取 TF 用的是 Time(0) 还是显式时刻，**未核实**（ws 内无 costmap_2d 源码，系统包仅头文件）⇒ 不能断言「探针显式时刻后，门绿就等于消费者也绿」。
4. 判据权归属：把 G5c1 从「结构问题的探针」改成「绕开结构问题的探针」，属验收判据语义变更，定权在小美（本档 L313 她已就「自降判据」定过口径），施工方不得自行动手。
**验证判据**：C 只能作为**辅助签名**并入 §6（见 V8：A 修后若 G5c1 仍 HIT，则按 L385 转 E1，届时 C 是 E1 的落地件，且需与「显式时刻 vs Time(0)」双打印并存一跑，才能给 tf2 语义侧定性）。

## 3. 推荐与取舍声明

**推荐方案 A**（反挂）。理由：单文件单块、零重编译、保留 before_M2.bak L9 的历史意图全部语义、与仓内三处既有桥接先例同构、能同时满足 U5 两句结构断言、影响半径可被 §4 的服务面判据完整覆盖。

- 不选 **B**：同一目标（base_link 入边唯一）要花 5 个模块目录 ＋ 一次 .so 重链 ＋ 全量回归批去换，而且它把冲突从 TF 层挪到配置层（真机官方 URDF 无 base_link 这一事实使「改 TORSO」并不等于「更贴近官方」），并**触碰生产共用配置**——costmap 与 hdl 的机体帧是全 mode 共用契约件，不是 sim 私有件。
- 不选 **C 单用**：C 不动被测件，因此永远交不出 U5 要的两句结构签名；它的作用域是「让探针不再误报」，而 ③ 之所以成立恰在「结构未修」这一事实面上（L368）。
- 不选 **A′**：收益（消一条无人 grep 的 WARN）与代价（在无离线 URDF→SDF 件可证合并语义的前提下改动 inertial 块）不对称；留作「WARN 打穿哨兵」时的后手。

**取舍的诚实边界**：A 修的是**结构缺陷**，不是 G5c1 的**成因证明**。双父与 G5c1 的因果尚未证实（小美 L362 ③ 同记「双父是否即 G5c1 之因未证实」）。故 A 落地后 G5c1 仍可能 HIT——那不等于 A 失败，也不等于可以回填 M2-2；届时按 L385 走 E1 定 tf2 语义/缓存侧，另立异常（§6 V8）。

## 4. 帧契约边界核查（全仓实测，非照抄派单）

搜索面：grep base_link / TORSO / body 覆盖 ws/src（排除 build/devel）、scripts、note。**域标注**：SIM ＝ 仿真域专用件（只在 11313/11345 隔离域起）；PROD ＝ 生产/真机契约件（mode_* 链或 G8 实投）；SHARED ＝ 两域共用同一份配置；DEAD ＝ 在仓但不入任何链。

| # | 文件:行 | 帧/内容 | 域 | 受方案 A 影响 |
|---|---|---|---|---|
| 1 | XAC L14–L15/L32–L36/L44–L48/L133–L134/L141–L144 | root=TORSO；TORSO→base_link；base_link→rslidar 0.38；odometryFrame=odom／robotBaseFrame=base_link；reference=TORSO 的 mu1/mu2 0.1 | SIM | **即被改件**（只改 L32–L36 ＋ 注释） |
| 2 | PLG L29–L30/L111–L113/L127–L128 | odom_frame_ 缺省 odom、base_frame_ 缺省 base_link；/Odometry 与 TF 皆以 now 为戳 | SIM | A 零改动（B 才改） |
| 3 | launch/m1_gazebo.launch L22/L23/L24/L25–L26；launch/m2_gazebo.launch L8/L22/L23/L24–L25 | 注释「z=0.15：TORSO box 底面 -0.11，落地间隙 0.04m」；xacro 现读展开；RSP 起；spawn_model -z 0.15；GAZEBO_PLUGIN_PATH | SIM | 文件**不得改**（R2 限定改动面只 XAC/PLG）；语义见本节第 3 问 |
| 4 | launch/m2_teb.launch L6/L22/L23–L29/L30–L33；scripts/m2_tf_fresh.py L4/L27 | 注释「map→odom 恒等（static）＋ odom→base_link」；tf_source 缺省 fresh；10Hz 广播 map→odom 走 /tf（非 /tf_static） | SIM | 间接受益（拓扑变连通）；文件零改动 |
| 5 | launch/m1_mapping.launch／m2_mapping.launch L7/L11 | 载 faster_lio_outdoor.yaml，起 run_mapping_online | SIM | 无（该域 faster_lio 发 camera_init→body，与 base_link 无连边，改前改后皆独立分量） |
| 6 | launch/m2_hdl.launch L6/L12–L14；lite3_bringup/launch/mode_sim_hdl.launch L12–L23 | 注「输出 /hdl_odom（map 系）＋ TF map→base_link」；include hdl 承载件 | SIM | 间接（hdl 分支择定，见 §5） |
| 7 | verify/m2_smoke.sh L199/L219–L226/L228–L238/L281/L282/L288/L289/L296–L325/L326–L327/L343–L355/L517；L141–L152 | 判据帧名全为 map／odom／base_link（无 TORSO）；E3 结构探针；gate 字典；样张 sha256 红线；stageA A0–A④ | SIM | 判据文本零改动（U2/U3 属小明另片） |
| 8 | verify/m1_smoke.sh L97 | 同 gt 取数方式（model_states 按 model 名过滤，不涉 link 名） | SIM | 无 |
| 9 | lite3_bringup/launch/mapping_viz.launch L5/L10/L12–L14/L15–L16 | robot_description ＝ Lite3_prim.urdf；RSP 20Hz；static_transform_publisher 发 **body TORSO** 恒等 | PROD（采图第 4 窗，scripts/mapping/README.md L28 记「TORSO inertia WARN 无害」） | 文件零改动；**同窗合跑禁忌**见 §5 R-d |
| 10 | lite3_bringup/urdf/Lite3_prim.urdf L6（root TORSO；全文 grep base_link ＝ 0 命中，实测计数） | 真机可视化骨架（mesh→box 替身，运动学逐字保留，见其 L3 自述） | PROD | 无（**不含 base_link** ⇒ 桥接不外溢） |
| 11 | lite3_description/urdf/Lite3.urdf L9（root TORSO；grep base_link ＝ 0，实测计数） | 官方上游描述件 | PROD（上游原样） | 无 |
| 12 | lite3_bringup/launch/mode_localize.launch L31–L34／mode_mapping.launch L30–L33 | 生产 mode 组合：**全文无 robot_description／无 RSP 节点**（本轮 grep robot_description 命中面只有 #3 与 #9） | PROD | 无（A 触不到真机链） |
| 13 | lite3_bringup/launch/mode_nav_stub.launch L34/L38–L40/L50–L51；scripts/stub_tf_broadcaster.py L5/L13–L16 | map→base_link 恒等 fresh-stamp 广播；明文「真定位接管时本节点退役（先停本节点再起定位，**避免 base_link 双父**）」 | PROD（stub 二期） | 无——但这是本仓**既有的单父教条先例**，A 与它同向 |
| 14 | lite3_bringup/launch/mode_nav_sim.launch L32–L34/L105；scripts/sim_pose_tf.py | 旧 sim 域（/quad_0/body_pose 真值）发 map→base_link ＋ base_link→rslidar，注「map→base_link 写者唯一」 | SIM（另一支，无 TORSO） | 无——又一单父教条先例 |
| 15 | 00_drivers/message_transformer/src/qnx2ros.cpp L146–L153/L154/L219/L254 | 组 odom→base_link 的 leg_odom TransformStamped（L148 parent odom／L149 child base_link），但 **L154 sendTransform 为注释态**；L219 imu 头 frame=base_link | PROD（真机遥测） | 无（§5 用于判「真机现状」） |
| 16 | livox_ros_driver2/launch_ROS1/msg_MID360s.launch L31（注释）／L32 base2livox：/base_link → /rslidar 恒等 | 生产雷达静态外参（由 lite3_drivers/launch/01_sensors.launch L5/L7 引用） | PROD | 无 |
| 17 | hdl_localization/apps/hdl_localization_nodelet.cpp L54/L420/L439–L442/L446–L449/L455/L459 | odom_child_frame_id 缺省 base_link；L420 以「能否查得 odom→base_link」择分支；分支 A 发 **map→odom**；分支 B 发 **map→base_link** | PROD（G8 实投算法件） | 间接：A 使该择定不再被双父抖动（§5） |
| 18 | lite3_localization/launch/hdl_localization.launch L31–L35/L38/L52/L53/L61–L62/L117 | odom_child_frame_id=base_link；robot_odom_frame_id=odom；enable_robot_odometry_prediction=false；静态 /base_link→/rslidar；liveness_m2 | PROD（G8 承载件；mode_sim_hdl 只是 include 它） | **零改动**；帧参数由本件自带，不依赖 sim URDF root |
| 19 | hdl_localization/launch/local_rslidar_imu.launch L9/L18；hdl_localization/launch/hdl_localization.launch L17/L46 | 第三方原件（lite3 承载件头注 L7–L11 明「原样保留、永不 include」） | DEAD | 无 |
| 20 | 40_navigation/lite3_navigation/config/costmap_common.yaml L4/L5/L11/L17/L47/L51–L56 | global_frame=map／robot_base_frame=base_link／transform_tolerance=1.0／footprint／sensor_frame=rslidar | **SHARED**（sim 与真机同一份；mbf_navigation.launch 挂载） | 无（A 保留 base_link 于原位置 ⇒ 消费方零感知） |
| 21 | 40_navigation/lite3_navigation/config/move_base_params.yaml L1–L2/L7/L8 | 同名两参数，但本件标注「回退底稿，仅存档不加载」 | SHARED（备档） | 无 |
| 22 | 40_navigation/lite3_scan/launch/70_scan.launch L6/L27 | 注「sim 侧 body/cloud 源换成 faster_lio 系」「world/camera_init 与 body/base_link 恒等 TF」 | SHARED（SCAN 片） | 无 |
| 23 | 30_3rd/plan_manage/launch/lio_relay.launch L46–L58（L55–L56 world→camera_init，L57–L58 **body→base_link**）；advanced_param.xml L56–L58（TORSO 碰撞盒数字注释）／L63–L64 body_height | SCAN 规划域桥接：base_link 作**子帧**，恒等挂 body 下；L53 注「规划器不消费 TF」 | PROD/SCAN | 无——第三处「base_link 作子/叶、且桥向它侧」先例；与 A 同构 |
| 24 | 10_slam/lite3_slam/config/faster_lio_outdoor.yaml L44/L47/L48；faster_lio/src/laser_mapping.cc L67/L160/L684–L685/L700–L710/L772 | tf_world_frame=camera_init／tf_imu_frame=body；LIO 广播 camera_init→body；点云体系统 L772 | PROD＋SIM（同一 yaml 两处用） | 无（body 链独立于 base_link/TORSO 桥） |
| 25 | 50_task/pipeline_tracking/tools/location_record.py L79、location_record_en.py L79、scripts/Task.py L47 | 查 map→base_link（任务层定位） | PROD | 无（SHARED 消费方，靠 base_link 名字存活——这正是不能选 B 的理由） |
| 26 | 00_drivers/lslidar_driver/launch/lslidar_c16.launch L73（base_link→rslidar 0.16 0 0.45 1.57） | 遗留雷达件 | DEAD（01_sensors.launch L11 明文「保留不入链」） | 无 |
| 27 | 95_simulation/Mid360_simulation_plugin/livox_laser_simulation/models/sensors_only/model.sdf L72 与 rviz/test_pattern.rviz L87（Fixed Frame: base_link） | 上游插件示例件 | DEAD／示例（lite3_gazebo_sim/package.xml L5 注：仅复制其 livox_laser_simulation **源码**入链，模型目录未入链） | 无 |
| 28 | 30_perception/deeprobotics_local_height_map/launch/height_map_safety.launch L29/L40/L73/L80/L99 等、two_height_map_safety.launch L32/L79/L82、height_map_lite.launch L29/L59 | 上游相机/雷达外参（/base_link 作父发多条相机边；L80 base_link→rslidar 0.32 0 0.14；L99 base2urdf dummy） | DEAD／上游（全仓 launch 无任何 include 命中该包，实测） | 无。**观察项**（不属 U1）：其 L80 的 0.32/0.14 与 sim XAC L47 的 0.38 及 #16/#18 的恒等值互不一致，属上游件遗留外参分叉，登记待裁 |

### 三个必须明确回答的问题

**问 1：改 XAC 会不会波及 G8 真机定位（hdl）？——不会。** 三条独立证据：① 该 xacro 的唯一消费点是 m1_gazebo.launch L23 与 m2_gazebo.launch L22（全仓 grep lite3_planar 实测，另两处是 .bak 与文档）；② 真机 mode_localize（L31–L34）与 mode_mapping（L30–L33）既不起 RSP、也不载 robot_description（本轮 grep 命中面见上表 #9/#12），真机 TF 的 base_link 完全由 #16 驱动静态边与 #17 hdl 动态边撑起；③ 真机两份官方 URDF（#10/#11）根本不含 base_link（grep -c 实测 0）⇒ TORSO→base_link 这座桥是 **sim-xacro 独有件**，反挂它不会改变真机任何一帧的父/子关系。hdl 侧参数（#18 的 L38/L53、#17 的 L54）自带缺省 base_link，与 sim 的 URDF root 无耦合。

**问 2：会不会改变已归档的 M2-1／M2-4 判据？——判据口径不变，数值须重取。** M2-1/M2-4 的判据本体是 SM L141–L152 的 A（闭环走完 out/turn/back 全 done）、A③（drift ≤0.5）、A①（点云 ≥5Hz）、A②（IMU ≥100Hz）、A④（飞点 <1%）、A0（运动窗 RTF ≥0.25 前置，M2R2 主理人裁决 0.3→0.25，SM L25/L148–L152）；它们消费的是话题名与数值，**不含 TORSO、不含 URDF root**，且 /Odometry 的 child 由 XAC L134 装配值决定（A 明确保留 base_link）⇒ 文本与阈值零变更。但按 L376「被测件即变 ⇒ 5 连重新起算」，stageA 三连同 stageB 五连都要重取（回归面 §7）。归档件里的历史数值（00_总览 L103 的 drift 0.489/0.475/0.489、进度文档 L347 的 dt/dev 区间）只作**观察带**，不设新阈值。

**问 3：会不会改变 spawn 高度语义？root 变更在 Gazebo 里是否改变模型锚点/惯性合并行为？**

- spawn 语义（推理）：m1_gazebo.launch L25–L26／m2_gazebo.launch L24–L25 的 spawn args 是模型级平移（-x 0 -y 0 -z 0.15），施加在模型根上；L22 注释的「TORSO box 底面 -0.11」由 XAC L22 的 visual/collision origin 恒等 ＋ L23/L28 的 box 尺寸 0.22（半高 0.11）决定，而 L35 的关节原点也是恒等 ⇒ base_link 与 TORSO 两帧**原点重合**，换 root 不换相对位置 ⇒ z=0.15 时底面仍在 0.04。此为几何推理，**以下列判据核，不以推理结案**。
- 惯性合并（**未核实其实现**）：Gazebo 对 fixed 关节的 lump 行为、lump 后 body 命名规则、以及 <gazebo reference=TORSO> 的 mu 落到哪个 body，本轮**无法静态核实**——沙箱内无离线 URDF→SDF 件（find 通配 z2sdf ＝ 空；/usr/bin 只有 gz-11.11.0 系列与 check_urdf，且 gz model --info 需连 Gazebo master），本轮又禁启仿真。故一律转为**可核判据**（V6，全部是既有 stageB/stageA 跑内的零额外成本取样）：
  - **锚点判据**：spawn 后首帧 /gazebo/model_states 中 name=lite3_planar 的 pose.position.z ＝ 0.15（±1e-3）；静置 ≥10 sim-s 后该 z 与「改前同场景基线快照」差 ≤5e-3（基线快照须在**首次编辑之前**取一次，与 R6 的留底时序纪律一致）；同时 /Odometry 的 pose z（PLG L116 直取 WorldPose z）改前改后同值（±1e-3）。
  - **合并判据①（拓扑）**：/gazebo/get_model_properties(model_name=lite3_planar) 的 canonical_body_name／body_names／geom_names／joint_names 四字段，改前应记为基线；改后**只允许** canonical_body_name 由 lite3_planar::TORSO 变 lite3_planar::base_link，**集合成员与个数不得增减**。任一增减 ⇒ root 变更确实改了合并行为 ⇒ 立即停手上报，不得进入 5 连。（服务与字段本轮已核：/opt/ros/noetic/share/gazebo_msgs/srv/GetModelProperties.srv 响应含此四项。）
  - **合并判据②（数值）**：/gazebo/get_link_properties 改后取 link_name=lite3_planar::base_link、改前取 link_name=lite3_planar::TORSO，比较 mass 与 ixx/ixy/ixz/iyy/iyz/izz 共 7 值 ＋ com：要求相对差 ≤1e-6、com 差 ≤1e-4（关节原点恒等 ⇒ 平行轴项应为零）。字段已核：GetLinkProperties.srv 响应即含 com/mass/ixx…izz。（该服务**不返回摩擦系数** ⇒ mu 只能走下面的行为代理。）
  - **摩擦代理判据**：mu1/mu2（XAC L141–L144）若因合并落空，最先响的必然是 dynamic 模式（XAC L135）的接触行为 ⇒ 以 stageA 的 A（三相位全 done）＋ A③（drift ≤0.5）＋ A②/A①（传感频率不因 RTF 变化而欠供）＋ stageB 的 dev/dt 落在改前五跑带（dt 8.4–8.6s、dev_odom 0.162–0.193，出处 进度文档 L347 与 G4 L16／G5 L17）作代理；同时把新五连逐值原样登记（**不新增阈值、不改判据本体**，避免重演 L313 的自降判据问题）。
  - **几何等价哨兵（零成本、已在位）**：样张 lite3.pcd sha256 未变（SM L517；改前五跑均 [PASS]，G4 L21／G5 L22）——碰撞盒位置/尺寸任何变化都会经建图点云反映到该 sha。口径：与改前基线**全等**，不作近似。
- 一句话结论：**设计意图是「几何、惯量、摩擦、锚点四者皆不变，只变 TF 拓扑方向」**，而这四件事各自都有上面这条可机判的等价判据；判据不过 ⇒ 停手回到本档 §2 重议，不得放宽。

## 5. 二次发生风险（对小美 L370 末句的确认／否定）＋ 对 G8/hdl 的设计预留

她的原句：「M2-3 接真实定位后 hdl 侧同样要发 odom→base_link，与 TORSO 静态父帧的权威冲突会二次发生」。**逐字面：否定；机理面：确认，且比她的写法更强。**

- 否定的是**边名**。hdl 不发 odom→base_link。原件：apps/hdl_localization_nodelet.cpp L420 以「tf_buffer.canTransform(robot_odom_frame_id, odom_child_frame_id)」择分支；分支 A（L436–L442）发的是 **map→odom**（header.frame_id=map L439、child_frame_id=robot_odom_frame_id L440）；分支 B（L444–L449）发的是 **map→base_link**（L446 map、L447 child=odom_child_frame_id）。G8sim/进度文档.md L83 的预核口径同样是「输出 /odom（map 系，z 强制 0）＋tf map→base_link」，与分支 B 一致。lite3 承载件里 enable_robot_odometry_prediction=false（#18 L52）、robot_odom_frame_id=odom（L53），但**分支择定与该开关无关**（L420 只看 TF 可查性）——这点容易误读，写死在此。
- 确认的是**冲突面**：base_link 只要挂着一条静态机体桥（TORSO→base_link），就与 hdl 的两条候选入边之一撞车——走分支 B 时撞的是 **map→base_link**，性质与 odom→base_link 完全相同（同一子帧两条入边）。所以「二次发生」成立，只是**边名要更正**。
- 还确认了一条她没写的、更硬的后果：**双父会让 hdl 在分支间跳变**。L420 的判据依赖 odom→base_link 这条边在 TF 缓冲里查得到；而双父正是让这条边按权威互斥/按时刻被丢的成因面（F3 L3 的 TF_OLD_DATA 是一次可见命中，U4 已要求 5 连 0 次）。一旦择定抖动，sim/真机就会交替出现 map→odom 与 map→base_link 两种 TF 拓扑，下游共用件（#20 costmap 的 robot_base_frame=base_link、#25 任务层查 map→base_link）跟着抖。**结论：U1 的结构修正不只是让 G5c1 好看，它是 hdl 分支择定的前置稳定性条件。**
- 真机现状（只作事实登记，不含任何操作步骤）：真机链上 base_link 的写者只有 #16（驱动静态 base_link→rslidar，msg_MID360s.launch L32）与 #18 L61–L62（同一条边的第二写者）；odom→base_link 在 #15 里被组装但 L154 的 sendTransform 为**注释态** ⇒ 真机当前无 odom→base_link 写者 ⇒ hdl 走分支 B；真机两份官方 URDF（#10/#11）不含 base_link ⇒ **TORSO→base_link 这座桥不会随真机链出厂**，故 A（反挂）对 G8 是零波及，反向也说明「G8 二次发生」的条件不是 A 造成的，而是「谁往 base_link 上挂静态机体桥」。

**设计预留（对 G8/hdl，一律不得写操作步骤，标注待主理人裁定）**

| 编号 | 预留内容 | 状态 |
|---|---|---|
| R-a | 把「任一 master 内 base_link 的入边唯一，且不得为静态机体桥」升为全仓 TF 契约条款（sim／真机同则）。先例三处已在仓：#13 mode_nav_stub L38–L40 与 stub_tf_broadcaster L13–L15（先停 stub 再起定位，明文「避免 base_link 双父」）、#14 mode_nav_sim L34（「map→base_link 写者唯一」）、#23 lio_relay L57–L58（桥的方向是 body→base_link，与 A 同构） | 建议入 02 坑册／04 台账，**待主理人裁定**（我不改 K，遵 R8/R9 同规） |
| R-b | G8 期 hdl 走分支 A 还是分支 B 的**择定权与口径**（走 A 需存在 odom→base_link 写者；仓内唯一现成候选是 #15 那段注释态代码） | **待主理人裁定**；本档不写真机操作步骤 |
| R-c | base_link→rslidar 同边双写者（#16 与 #18 L61–L62）——非双父，属重复权威静态发布；是否留一删一 | **待主理人裁定**（另案，不混入 U1） |
| R-d | #9 mapping_viz（body→TORSO ＋ RSP 载 Lite3_prim.urdf）与 sim 载荷不得同 master：A 之后 sim 会发 base_link→TORSO，若与 #9 同窗则 **TORSO 变双父**（body 与 base_link 各一）——这是 A 引入的**新发生面**，必须登记。现状：sim 走 11313/11345 隔离域（SM L21），#9 属真机采图第 4 窗（scripts/mapping/README.md L28），本轮未见同窗先例 | **待主理人裁定**隔离口径是否加一条「viz 与 sim 载荷互斥」门禁 |
| R-e | 把 V1／V2 两句结构签名固化为常驻哨兵（现 E3 只在 stageB 门控时刻跑一次，SM L325 单次调用） | **由小美判**（判据增删权在验收） |

## 6. 给小美的验收判据草案（可机判；U5 结案面）

先给**修后 E3 应看到的签名**（逐行模板，行文本以 SM L313–L322 的打印格式为准）：

    SMOKE E3 tf odom->base_link hits=<N> last_stamp=<t> age=<a>
    SMOKE E3 base_link_parents=['odom'] transforms_total=<M>
    SMOKE E3 static base_link->rslidar stamp=0.020
    SMOKE E3 static base_link->TORSO stamp=0.020

要点：静态行集里**不再出现任何指向 base_link 的边**，改为出现 base_link->TORSO（父=base_link、子=TORSO）；第一行不得带 [ZERO] 后缀（该后缀由 SM L315 在戳等于零时刻时附加）；hits 与 <M>（transforms_total）预期与改前同量级（改前 G4 L11/L12：hits=27、total=32；因静态边不经 /tf，修后 /tf 侧计数不应变化——若 total 突增，说明静态边被挪进了 /tf，属异常）。

| 编号 | 判据（机判式） | 覆盖点 | 原件锚 |
|---|---|---|---|
| V1 | 行 SMOKE E3 base_link_parents= 的值恰为单元素列表 odom（grep 计数该形态 ＝ 1，且无任何含 TORSO 的多元素形态） | U5 第一句「/tf 上 base_link 父帧唯一=odom」 | SM L316–L317；G4 L12／G5 L13 |
| V2 | E3 static 行集中 grep -- "->base_link " 命中数 ＝ 0；且静态行集恰为 base_link->rslidar 与 base_link->TORSO 两行（枚举完备） | U5 第二句「/tf_static 无指向 base_link 的边」＋完备性 | SM L318–L322；G4 L13–L14／G5 L14–L15；XAC L32–L36/L44–L48 |
| V3 | E3 第一行 age ≤ 0.050 且 last_stamp 与该行 sim_now 同量级（对照改前 G4 L11 的 22.080/0.020、G5 L12 的 22.480/0.015），[ZERO] 后缀 0 命中 | 结构修正不得把发布侧改坏 | G4 L11–L14；G5 L12–L15 |
| V4 | G5c1 依 U2 回补进 gate 字典（SM L326–L327 须含该键）后，fresh 臂 5 连该项全 PASS；g5_sample 调用形态（SM L288）与 need=3／lim=1.0（L228）、零戳与首帧戳否决（L222–L225）**一律不得放宽**；判定行仍 5 PASS/0 FAIL、rc=0 | L385「G5c1 作为 gate 项在 5 连全 PASS」；L375 | SM L219–L238/L288/L326–L327；进度文档 L375/L385 |
| V5 | 5 连 grep TF_OLD_DATA 命中 ＝ 0（对照历史 F3 L3 的一次） | U4 已设；结构修正后的直接可观察后果 | 进度文档 L349/L384；m2r3_fresh3.log L3 |
| V6 | 结构等价四判（§4 问 3 的锚点判据、合并判据①②、摩擦代理判据）＋样张 sha256 全等，任一不过即停手 | 证明「root 变更只动拓扑、不动物理」不是断言而是核得 | SM L96/L175/L422/L517；GetModelProperties.srv／GetLinkProperties.srv 字段 |
| V7 | 静态臂阴性对照**不重跑**（依 L404 原文：M2R3 已完成的 2 次 static 对照不重跑、不计入 5 连）；也不得因「修后对照仍 FAIL」记回归 | 防止把她的既有裁决跑成新账 | 进度文档 L401–L404 |
| V8 | 若 V1/V2 成立而 G5c1 仍 HIT：按 L385 转 E1 三分支离线探针（Time(0)／显式 now／buffer 原始条目），本档 §2 方案 C 即 E1 的落地件；届时**不得回填 M2-2、不得由施工方自行降级**，另立异常 | L385 后半句的预置出口 | 进度文档 L304/L307/L385 |

## 7. 回归范围声明（修后必须重跑的 Gate）

依 L376 与 L401：**被测件即变 ⇒ 5 连重新起算，旧五跑不得与新配置混计**。具体：

- **必重跑**：① stageB fresh 臂 **5 连全绿**（U4 口径：其中 ≥1 次按她替换后的新加压档——取龄点 sim ≥60s 或同跑两取龄点 sim 差 ≥40s；每跑 foxy 六 PID before/after 成对快照、端口 TCP/UDP 双查、退净 残留:[]、样张 sha256 未变、G6 goal 后 pose 类 ERROR=0）；② **stageA 复跑**（M2-1/M2-4 判据 A0–A④；因 root 变更属被测件变更，物理面的 A③/A②/A① 必须重取）；③ **U3 的 60s 大档延时 1 次**（证明 120s 预算不被吃、raw/adjusted 两个 t 打印不失真）；④ V6 的结构等价四判随首轮留证一次（不额外起窗）。
- **建议随批次做一次（静态、零节点）**：共同门禁 scripts/verify/v3_isolation.sh 全量 [1]–[9] 取一次 EXIT=0——[8]/[5]/[2] 的扫描面覆盖全 ws/src，改 XAC 注释即在该扫描面内（低风险项，非阻断）。
- **不重跑／不适用**：stageC（M2-3 hdl 仿真已被 M2-R2C 裁决终止转真机 G8，00_总览 L106）；静态臂阴性对照 2 次（V7）；若选 A ⇒ **.so 不重编译**（xacro 是 launch 期现读件），故不需要单包重链批与全量清白批；若改选 B ⇒ 二者皆需，回归面再加一层（先例 00_总览 L82）。
- **状态口径**：M2-2 维持 PARTIAL（阻断中）直至 U1–U5 全绿；L328 旧候补句继续不生效，终稿走 L404 的候补 v2。

## 8. 施工面交接要点（仅仿真域；不含真机操作）

1. 改动面**只许 XAC**（A 方案：L32–L36 关节块 ＋ L14 及 L3–L10 注释），PLG 与任何 launch/config/SM 零改动；不许顺手调判据（R2 原文 L390）。
2. 留底纪律（R6）：.bak 必须在**首次编辑之前**生成并当场入清单；同时导出改前基线件（check_urdf 输出、get_model_properties／get_link_properties 快照、model_states z、样张 sha）——这些是 V6 的对照组，事后补不得。
3. 若同轮动 SM（U2/U3 属小明）：改后必做两检才可复跑（R5）：bash -n ＋ 抽取每段内嵌 python 做语法解析（内嵌段界 SM L166/L341）。
4. 落盘纪律：文档/台账追加一律带引号 heredoc，正文禁 markdown 反引号（02 L151–L159 坑18）；编辑后**每次**立即补 named ACL（02 L141–L149 坑17）：setfacl -m u:ysc:rwx 与 -m u:ros1-slam:rwx 本文件，目录加 -d 两条 default。
5. 端口与隔离：11313/11345 为本次隔离域（SM L21），生产 11312 零监听；SM L8 头注仍写 GAZEBO_MASTER_URI=11355（与 L21 的 11345 不一致，系 P10 仲裁后注释未随更）——属注释面残留，路由小明，与本设计无关、不阻断。
6. 回退：A 的回退＝还原该关节块（含关节名），零重编译、零回滚 .so；与 XAC L9 既有回退口径（换 filename 即回 M1 形态）互不干扰。
7. 停手条件：V6 任一条不过、或 V1/V2 不过、或 stageA/stageB 任一判据 FAIL ⇒ 按三次失败红线停报，回到本档 §2 重议，不得以「主判据已绿」替代结构修正（L385 末句原文有效）。

## 9. 未核实清单（不推测填充）

① Gazebo 11 的 fixed-joint 合并（lump）实现细节：是否合并、合并后 body 命名规则、以及 <gazebo reference=TORSO> 的 mu 归属哪个 body —— 无离线件可证（沙箱 find z2sdf 为空），且本轮禁启仿真 ⇒ 以 V6 判据替代。② GetLinkProperties.srv 不返回摩擦系数 ⇒ 摩擦只能行为代理。③ 消费端（costmap_2d／mbf／teb）取 TF 用的是 Time(0) 还是显式时刻 —— ws 内无 costmap_2d 源码，系统包仅头文件，未核实；故 C 方案不能宣称与消费端同构。④ 双父与 G5c1 的**因果**未证（她 L362 ③ 同记）；本设计消除结构缺陷，不承诺 G5c1 转绿。⑤ check_urdf 输出的确切行文本未在本轮实测（我未对展开件跑它，避免落盘临时件），判据写「根 link 名由 TORSO 变 base_link」以实际输出为准。⑥ 22/27/28 行号取自本轮 grep 命中，个别上游件是否参与构建未逐一核（已在表内标 DEAD／示例并注明依据）。⑦ Mid360_simulation_plugin 与 lite3_gazebo_sim 的插件复制关系仅据 package.xml L5 的自述。

## 10. 执行合规（本档）

- 零 ws/ 改动、零 data/ 改动、零他档改动、零 git 写、零 sudo、零节点、零仿真：本轮只 Read/Grep/sed -n/cat -n 只读代码与证据件（XAC、PLG、m1/m2_gazebo、m2_teb、m2_hdl、mode_*（7 件）、Lite3_prim.urdf、Lite3.urdf、hdl nodelet、lite3_localization/hdl launch、costmap_common、move_base_params、qnx2ros、sim_adapter、01_sensors、msg_MID360s、lslidar_c16、lio_relay、advanced_param.xml、height_map 三件、m2_tf_fresh.py、m2_smoke.sh、m1_smoke.sh、before_M2/before_M2R1 .bak、G4/G5/F3 证据件、00_总览、进度文档 L290–L408、02 坑17/18、mapping/README.md、GazeboM1/进度文档 L74、G8sim/进度文档 L83、gazebo_msgs srv 定义）。
- 唯一新增文件：本件（note/进度/v7/S2/M2/设计/m2r5_双父TF设计.md），目录为本轮新建（mkdir -p）；创建与追加皆用带引号 heredoc，正文零 markdown 反引号（坑18）。
- ACL：建目录与本件后补 setfacl -m u:ysc:rwx、-m u:ros1-slam:rwx 于文件与目录，并对新目录补两条 default（坑17）。
- 实现状态：**未施工**。本件是 U1 的设计 payload，施工与复跑归小明，判定归小美；真机面一律标「待主理人裁定」。

## 11. 向前指向更正（小张，2026-10-08，依小美 M2R5 结案复核 R13）

本件成文 2026-10-08 06:35，早于 V6 两批取样（raw 四件 mtime 实测：batch1 06:47／06:48，batch2 06:58／07:00）。依仓库惯例**上文一字不回改**，本节只做向前指向：自本节生效起，本件 §4 问 3「合并判据①（拓扑）」（L161）以及沿用其字面的 §4 问 3 末句（L165）、§6 表 V6 行「任一不过即停手」（L204）、§8 第 7 条停手条件（L225）、§9 ①（L229）一律按下面 11.1 的替代判据读。

### 11.1 判据① 的字面被实测否证；她的更正文本（逐字引）

- 她更正后的判据文本（逐字，出 note/进度/v7/S2/M2/进度文档.md L560 的 R13 条；L506 表 #31 与 L430 的〔小美 R13〕括注同值）：「**个数不变 ＋ 成员名允许且仅允许跟随 root link ＋ 数值面逐位全等 ＋ canonical_body_name 不作锚（两态皆空串）**」。
- 本件 L161 的字面「**集合成员与个数不得增减**。任一增减 ⇒ root 变更确实改了合并行为 ⇒ 立即停手上报，不得进入 5 连」**已被实测否证**：改 URDF root 后 lumped body 名**必然**跟随 root —— body_names 由 [TORSO] 变 [base_link]（V6a2 L6 对 V6b2 L6），geom_names 由 [TORSO_collision] 变 [base_link_fixed_joint_lump__TORSO_collision]（V6a2 L7 对 V6b2 L7），而**个数两态皆为 1、未增减**；joint_names 两态皆空列表（V6a2 L8、V6b2 L8）；canonical_body_name 两态皆空串（V6a2 L5、V6b2 L5），该字段本就**不可作锚**。
- **字面被否证 ≠ 物理面受损**：改 root 使 lumped body 名随 root，是 Gazebo 对 fixed 关节做合并（lump）的**应有行为**（02 坑21 L178–L183；机理面的直接物证我本轮复核为 geom 名内嵌 fixed_joint_lump 串 V6b2 L7 ＋ 改后 URDF 仅 1 个 collision 块（实开 note/进度/v7/S1/SCAN集成/证据/m2r5_baseline/urdf_after_M2R5.urdf：L36 起，grep 计数 1）＋ 非 root 名两态皆回 link not found（V6a2 L9、V6b2 L11））。施工方未走我这条停手条、亦未申报该判据字面被否证，是她 #31 记的**程序面偏差**一条，不归物理面。
- **替代判据（可机判写法，下轮起用）**：
  1. 个数面：len(body_names)、len(geom_names)、len(joint_names) 三值改前改后逐位相等。
  2. 成员名面：body_names 的每个元素只允许两种取值 —— 与改前同名，或等于改后的 root link 名；geom_names 只允许 root_fixed_joint_lump__（改前 geom 名）这一形态；出现其他新名、或条目增删 ⇒ 停手上报。
  3. 数值面：mass 与 ixx/iyy/izz/ixy/ixz/iyz 六惯量分量 ＋ com 三分量 ＋ gravity_mode，改前取 lite3_planar::TORSO、改后取 lite3_planar::base_link，要求**逐位全等**（打印逐位对照，相等为过；本件 L162 的 ≤1e-6／≤1e-4 容差本轮实测为逐位全等，容差只可收不可放）。
  4. canonical_body_name **不作锚**：该字段两态皆空串，凡以它为基线或锚点的判据一律视为无效判据，不得据其写停手条。
- **主证锚点（batch2 四行，点名）**：note/进度/v7/S1/SCAN集成/证据/log/raw/m2r5_v6r2_pre.txt **L10、L11** 与 m2r5_v6r2_post.txt **L9、L10** —— mass=15.060000、ixx=0.347906、iyy=0.187906、izz=0.281110、ixy=ixz=iyz=0.000000、com=0.000000,0.000000,0.001262、gravity_mode=True，两态逐位相同（拓扑面另见 V6a2／V6b2 各 L5–L8）。

### 11.2 batch1／batch2 取证口径（本件无「以 batch1 为准」字样，但链上表述一并指向正确处置）

实测核对：本件 grep batch 零命中（成文早于两批取样），故 §4 与 §6 不存在「以 batch1 为准」的表述；该误记在 note/进度/v7/S2/M2/进度文档.md L430（施工方登记语），已由她以 R12 就地更正且原文保留以显形迹（L559），我不重复改。本件需向前指向的是下列三点：
- batch1（m2r5_v6_pre_batch1.txt／m2r5_v6_post_batch1.txt）**字段名当时写错、惯量面根本没读到**（V6a1 L8 model_props=ERR、L10 TORSO=ERR 报无 inertia 字段；V6b1 L8、L9 同），故它**只能作瞬态包络旁证**：z 的 min/max/span 两态全等 0.109869／0.150000／0.040131、z_mean 差 3.5e-5（V6a1 L4 对 V6b1 L4，样本 1006 对 1021），**不得用作数值主证**。
- batch2 才是数值主证（即 11.1 末那四行）；且 batch2 两态 z 四统计逐位相同（V6a2 L3 对 V6b2 L3，恒 0.110000、span 0.000000），pre/post 之间**不存在采样窗错位**（她 #30，L505）。两批同向 ⇒ 采信加强而非削弱。
- 我 §4 问3 L159 与 §9 ① L229 把 lump 整体列为「未核实其实现」，须收窄：就**是否合并**与**合并后 body 命名规则**两问，batch2 已给出实测答案（合并成立、名随 root），这两问不再算未核实；仍**未核实**的只剩 <gazebo reference=TORSO> 的 mu 归属哪个 body —— GetLinkProperties.srv 不返回摩擦系数（L162 末注仍有效），摩擦仍走行为代理，不得据本更正段宣称摩擦面已证。

### 11.3 建议给 02 坑21 加的「单模型一对实测」限定（只建议，02 归主理人／小王登记，我未改 02）

建议限定句文字（可直接附于 02 L178 标题行下或 L179 句尾）：

「限定（小张，2026-10-08，依小美 #32 ①）：本坑结论「fixed 关节树合并为单一 body、body 名 = URDF root link 名」目前仅由 lite3_planar 这**一个模型**的改前／改后**一对**实测支持（V6a2／V6b2 各一件、每态一台，n=1 对），属**单模型单对实测**，不得写成 Gazebo 通用定律；未经第二个模型（多 fixed 子帧／多 collision／含 active 关节的树）复现前，不得据此外推一般行为，也不得据此外设或放宽任何其他模型的前后对照判据。」

同段另两条建议（同样只申报、不改 02）：①02 L181 的「本轮实测的等价面」引的是 batch1 的 z 三统计 ＋ mean 差 3.5e-5，按 11.2 应补一句「数值面以 batch2 为主证（V6a2 L10/L11、V6b2 L9/L10），本行 z 统计为 batch1 瞬态包络旁证」；②L179 的报错引文串与原件不一致，见 11.4。

### 11.4 「by model name／by link name」实测核对结果（只核对与申报，我未动 02）

- 原件逐字（四件全含同串，实开 grep 命中 8 处）：「GetLinkProperties: link not found, did you forget to scope the link by model name?」——出现于 m2r5_v6r2_pre.txt L9、L12，m2r5_v6r2_post.txt L11、L12，m2r5_v6_pre_batch1.txt L9、L11，m2r5_v6_post_batch1.txt L10、L11。**原件的串是 by model name**。
- 02 L179 现状写作「link not found, did you forget to scope the link by link name」⇒ **与原件逐字不一致**（by link name 应为 by model name）。应改成的逐字串即上面那条完整句（含 GetLinkProperties: 前缀与句末问号）。
- 口径对齐：她 #32 ②（L506）与 R13（L560）记载均为「原件 by model name、02 写成 by link name」，与我的实测**一致**；本轮派单转述给我的「原件串为 by link name」与原件**相反**，属转述走样，我一律以原件为准并在此留痕。
- 该更正的执行权在台账侧（02 归主理人／小王），我只出这条核对结论。

### 11.5 影响面与合规（本段）

- 效力面：本节只更正我这一条判据的**字面**与其停手触发条件；本件 §1–§10 的其余判据（锚点判据、合并判据② 的数值比较、摩擦代理判据、几何等价哨兵，及 §6 V1–V8 各行）**不回改、不撤销**。
- 合规：零 ws/ 与 scripts/ 改动、零证据件改动、零 02 与进度文档改动（均为只读引用）、零 git 写、零 sudo、零节点、零仿真；11313／11345 未触碰，未干扰施工方在跑的 M2R6 批。追加只用带引号 heredoc、正文零 markdown 反引号（02 L151–L159 坑18）；追加后立即补 named ACL setfacl -m u:ysc:rwx,u:ros1-slam:rwx 本件（02 L141–L149 坑17）。引用行号全部实开原件核对：本件 L159/L161/L162/L165/L186/L204/L225/L229；进度文档 L430/L505/L506/L542/L559/L560；02 L178–L183；raw 四件各行；m2r5_baseline/urdf_after_M2R5.urdf L36。｜小张 2026-10-08
