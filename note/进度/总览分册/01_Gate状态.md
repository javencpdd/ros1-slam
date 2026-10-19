> 本文件＝`note/进度/00_总览.md` 人读区外迁件（R487/批137 路甲拆分，2026-10-20；原文行逐字搬迁、一字未改；在册位置以 00_总览 第 2 行下方内置索引为准）。

## Gate 状态
> （另起句·R486 登记：本节原为两列表格——表头行＝`| Gate | 状态 |`、分隔行＝`|---|---|`；批127/R477 重排版（129ad70）把本节表格改写为 ### 小节列表形态，表头行与分隔行未随表格保留。本句为补登（另起句），不是原表格行。★原文不回改★；列义以本句为准＝左列 Gate 名、右列状态（含日期与复核人）。）
### G0
- PASS（2026-10-04）
### G1
- PASS（2026-10-04）
### G1.5
- PASS（2026-10-04）
### G2
- PASS（2026-10-04）
### G3a
- **PASS**（2026-10-04 小美，独立复核）
### G4
- **PASS**（2026-10-04 小美，独立复核）
### G3
- **PASS（复验 2026-10-07，现行底图=lite3_1007 系）**（2026-10-07 小美，独立复核：三件套+raw master lite3_1007.pcd 329 万点/17.34×25.18m/189m² 声明逐项实测吻合
- ；polished 386,549 点=z 裁剪【-0.09,4.46】+0.049m 体素二分+SOR 剔 166 实测一致，头 147B+体 386549×16B+尾 0B 严丝合缝、无 POINT_STEP、无 stray 换行、NaN=0
- ；孤立点 6=0.002%、NN median 3.25cm
- ；lite3.pcd 与 polished 逐字节同哈希
- ；pgm/yaml 299×290@0.05、origin z=0.0 原生、**pgm 占据格 100% 被 polished 云支撑（导航带 z 0.2–1.5 IoU=0.97）**
- ；**外圈原生闭合围合 63.2m²、主内部域连通 93%**（v1 虚线墙膨胀桥接时代结束）
- ；**G8 工作底图确定=lite3.pcd（=lite3_1007_polished 逐字节副本），hdl map_path 建议 /home/ros1-slam/data/maps/lite3.pcd**。前史：2026-10-04 PASS（形式）样张 b467c69 → 2026-10-05 PASS（正式）v1 底图 21609 点 a223698（**已退役**），三段历史分列 P3b 进度文档）
### G5
- **PASS**（2026-10-04 小美，独立静态核验）
### G6
- **PASS（完全，L1 收口 2026-10-06）**（2026-10-04 小美：①②④⑤ PASS
- ；③ SUP 心跳为 GAP-1 临时代偿 + hold 复位需 SUP+M6 双重启=L1，PARTIAL——2026-10-06 L1 修复收口：SUP 自动复位语义（恢复观察状态机+防振荡+auto_recover 安全阀）回归 6/6，证据 [P6/证据/L1回归_六判据.log](P6/证据/L1回归_六判据.log)、04 台账 D-L1-1/2/3
- ；**2026-10-06 小美验收确认**：六判据逐字核验+payload 对账复算（bak+diff==现役）+设计符合性全过，G6-③ 转完全 PASS，报告见 [P6/进度文档.md](P6/进度文档.md) L1 正式验收节）
### G7
- **PASS（② 降级注记保留：hdl 定位质量不判，转真机 G8）· 非无条件 PASS**（2026-10-08 小美 Gate 补验收：**G7-① 由 PARTIAL 转 PASS、D-L1（GAP-4）遗留项关闭**——实起实采（roscore 11313 + 系统版 noetic map_server + 现行 lite3.yaml）
- ；负判据根除：yaml-cpp／bad conversion／exception 双通道（节点 stdout/stderr + /rosout）零命中，控制台 `Read a 299 X 290 map @ 0.050 m/cell`
- ；正判据实测 **width=299 / height=290 / resolution=0.050000 / origin=[-8.800000, -4.350000, 0.000000]**（z 非 nan）、frame_id=map、meta≡info、**len(data)=86710=299×290**，与 pgm 头 `P5 299 290`+体 86,710B=86,725B 三方逐字节闭合
- ；红线零违规（三件 sha256 起前=退后全等、foxy 五 PID+180237 `lstart` 逐位一致零扰动、11312 全程零监听、/cmd_vel 与 ros2qnx 零出现、环境三步零 foxy 混装）
- ；退净=本单进程零残留+11313 释放
- ；证据 [P6/证据/G7-1复验_D-L1_小美.log](P6/证据/G7-1复验_D-L1_小美.log) + raw/G7-1_* 五件，报告见 [P6/进度文档.md](P6/进度文档.md) G7-① 复验节。**★ 附带上报 N-G7①-1（预存在回归，非本单造成）**：`v3_isolation.sh` 全量 [1]–[9] = **ISOLATION FAILED EXIT=1（两次复现）**，仅 [5][6] 触发、命中全在 `95_simulation/lite3_gazebo_sim/launch/` 域、生产域零命中、最后一次在册全量 OK=2026-10-05 04:33、涉事 mtime（10-05 20:36～10-06 13:40）全早于本次开工、脚本本体工作树净未被改
- ；**G2 待主理人复评**，本验收未现场改脚本或改 sim launch 凑过
- ；清理项=既有孤儿 `rosout` 328580（起前即在位）。 **〔小美 R23(五)／O-3 处置，2026-10-08〕**清理项补正：本仓另有一孤儿 rosout **1373109**（属 ysc），经小美 M2R6 溯源为本仓 logs-20261008_045922 未收口跑的漏网（非 G7 在册的起前件），已于 2026-10-08 09:2x 以 ps 判在位后 kill 销项、复检零存在 ⇒ **O-3 清理面闭合**
- ；328580（属 ros1-slam、lstart 10-06 19:51:44）仍为起前既存、无 sudo 通道不动，继续挂清理项。前史：2026-10-04 小美 PARTIAL（② hdl 契约层 PASS（GAP-2 手动组合+样张地图降级注记，定位质量不判）、③ PASS
- ；① map_server 拒收 origin z=nan=D-L1 PARTIAL，修 yaml 与底图重采同场））
### G8/G9
- **G8 前置配置 PASS**（2026-10-05 小美，静态批：C3 min_z=-0.30 定案+M2 壳 TF 四 args 占位行+留底 sha256 四项对账+推导复核自洽，证据 [P8prep/证据/G8前置配置_小美核验.log](P8prep/证据/G8前置配置_小美核验.log)
- ；**G8/G9 本体仍未到**：余 Q13 俯仰精测（G8 同场残差反推）/真运动链，F5 顺序 G3→G8→G9 不可倒，明令不硬闯——非 FAIL）
### G-组合（P7）
- **PASS**（2026-10-05 小美，独立核验：三 mode 全 PASS+GetPath outcome=0+SUP 真源三段含修正批 2.89s 回归+[5] 正反向+样张/第三方零漂移+foxy 快照一致+DoD 10/10
- ；观察 O1–O5 不阻塞
- ；**P7 放行≠全栈终验，G8/G9 仍未到**）
### G-G8b（仿真测试台）
- **PASS（边界注记）**（2026-10-05 小美，独立核验：run3 13 CHECK 全 PASS 逐字复核+静态红线全过（ros2qnx 零节点引用/写者唯一/ysc·test 双 0/拷贝包零改动/payload 7 件 sha256 零改动/v3_isolation ISOLATION OK）+偏差 D-1..5 全部追认，**D-5 判据几何修正成立（traj_maze 实测去程 34.8m、三 goal 落线 0.000m、最坏走廊距 32.4m＜下限 35.6m），常数 33.6 附口径注记=沿用既有注释口径、实测以 34.8m 为准（+2 松弛覆盖，裕度 0.8m）**
- ；**边界=规划/导航层活性+正确性，非闭环非估计层（坑14），不动 G0–G9 判据原文，G8 真机正路照旧**
- ；证据 [仿真测试台/证据/](仿真测试台/证据/)，报告见 [仿真测试台/进度文档.md](仿真测试台/进度文档.md)）
### GazeboM1（仿真主线）
- **PASS（边界注记：闭环转 M2）**（2026-10-06 小美，独立核验：返工 R1 后四判据 ①13.2Hz≥5 ②264.4Hz≥100 ④出图 570 帧/522,883 点/飞点 0.0000 全 PASS、③漂移 26.411m≤0.5 **FAIL 事实保留**但 traj.txt CSV 独立抽验 faster_lio 自身轨迹与 gt 全程吻合至 1.5cm（0.06%）+四元数恒等=**非估计层漂移**，闭环未完成归 harness（planar_move odom yaw 开环积分至 π 而机体物理未转向，CSV 坐实）→划转 M2
- ；D-M1-5（rslidar collision 删除，xacro 独立展开 rslidar 真实 collision=0+sha256 四件对账）/D-M1-6（gt 探针 p.pose→p.position，diff 单行）复核 PASS
- ；隔离 11313/11312/foxy 4 行/样张 c034c906/退净全过（验收时点 ss+pgrep 实测双空）
- ；**核心命题成立：Gazebo 物理仿真可支撑 faster_lio 仿真验证（坑14 局限在物理仿真不复现）
- ；hdl 属 M2 定位接入待验**
- ；勘误 O1「No point 176→3」实为 1587→3（三件留证一致）
- ；证据 [GazeboM1/证据/](GazeboM1/证据/)，报告见 [GazeboM1/进度文档.md](GazeboM1/进度文档.md)）
### v4.2 补丁（文档批）
- **PASS**（2026-10-06 小美，独立核验：sha256 8c0651fc… 吻合+v4/v4.1 正文零改动（git diff ba33502^ ba33502 为空）+X1–X7 逐项对上 02 坑册/仿真测试台/GazeboM1/G8sim/manifest 证据+勘误 O1 四处在位（176 口径残留 0）/O4 env.sh=1/O5 .gitignore 两行+03 六仓库+一致性抽查全中（1.5cm/0.06%、13 CHECK、310×295=pgm 实测、298,350 点、样张 c034c906 实测全等）
- ；轻缺陷 d1–d3 随 M2-5 落账（入口件名 m2_gazebo≠mode_nav_gazebo/11312 双口径注记/坑14 02 侧落位）
- ；报告见 [M2/进度文档.md](M2/进度文档.md)
- ；**M2 运行时批暂停待主理人仲裁**）
### M2（物理仿真闭环主线）
- **PASS（条件注记：仿真覆盖 faster_lio✅ teb✅（功能面
- ；门控面已由 G5/G5b/G5c1/G5c2/G6 闭合） hdl❌，hdl 转真机 G8 顺带验证
- ；M2-2=**PASS（2026-10-08 由 PARTIAL 回填，小美 M2R5 结案复核第五节）**：base_link 单父修正件（U1）之后重取 stageB 正向 5 连全绿 —— teb_state=3、dt 8.5–8.6s、dev_odom 0.166–0.176 ≤0.35
- ；G5 Δ_age 实测 0.005–0.092s(sim) 连续 3 采样 ≤1.0s 且零戳／首帧戳否决未触发、G5c2 map→odom 分段同带、**G5c1 已由诊断项回补为常驻 gate 项并 5 连全 PASS**（修正前同一判据 7 跑全 HIT zero_stamp ⇒ 同判据前后对照即阴性对照，**双父即病因**得证
- ；静态臂 2 次对照依 M2R3 已完成者不重跑、不计入 5 连）
- ；G6 goal 后 pose 类 ERROR=0 五跑全 PASS
- ；取龄点1／取龄点2（若开）／goal 发出／goal 到达四点 sim 时钟皆有打印，加压档 1 跑（fresh3n）取龄点2 sim=62.44s、与取龄点1 sim 差 41.31s，且该跑 goal 段整体后移至 sim 63.05–71.59s，与另四跑的 20.5–30.5s **分属两时钟区**（不得称「同条件」）
- ；延时档门控 PASS 系**扣除观测延时后 ≤120s** 口径（注入墙钟 155.4s，t_raw=t_adj=2.4s，记账符号由 U3 修正为加回）
- ；E3 核得 /tf 上 base_link 父帧唯一为 odom、/tf_static 无指向 base_link 的边、原始 odom→base_link age 0.002–0.018s 且 [ZERO] 0 命中、5 连 TF_OLD_DATA 命中 0 次
- ；红线：五跑各 foxy 六 PID 起终成对 lstart 快照逐位一致、11312／11313 TCP 与 UDP 分查全程零监听、样张 lite3.pcd sha256 五跑未变、共同门禁 [1]–[9] ISOLATION OK
- ；stageA 于 root 变更后重取 1 跑 = 7 PASS/0 FAIL、drift 0.490 ≤0.5（观察带 0.475–**0.497** 之中带，非上沿）、RTF 0.271 ≥0.25、点云 8.9Hz、IMU 195.0Hz、飞点 0.0000
- ；根因口径「全局系 TF 时基新鲜度缺失已闭合（P6 L62 / P7 N-1 家族同款修法在仿真域复现并生效）」，附记「base_link 双父结构缺陷另案闭合，见 M2/进度文档 M2R4 复核节 U1 与 M2R5 结案复核节」。**两条观察项挂账不销（引用时加 M2R5- 前缀，与 A7 切片 O-1 同名不同物）**：M2R5-O1 退净逐跑打印缺失且一次批级检查非空（两 PID 身份永久不可回溯，记未核实非虚报）
- ；M2R5-O2 TF_REPEATED_DATA（base_link，parent odom）在新五连 3/5 命中、旧七跑 0 命中、待归因 —— 二者不阻断回填，**但 O-2 若证得推高取龄或影响消费端（任一次 >1.0s 或 pose 类 ERROR 命中），本回填自动回落 PARTIAL，无需另案**）**（2026-10-06 小美，独立核验：五批 commit 串行 2fe9976→5710384→b858a86→51e0636→dfb3ef0+关键数据原件抽验全中——PROBE H1 三方一致 48.4°/D-01 IMU 键名反转（隔离 world updateRateHZ+in-stack 1.0→197.8Hz·飞点 0.2830→0）/StageA 3 连 drift 0.475–0.489≤0.5·RTF 0.288–0.290≥0.25 门·IMU 198.4–198.6·飞点 0·转向 3/3（M2-1/M2-4 收口）/stageB teb_state=3·dt 8.6s·dev 0.179≤0.35（**M2-2 转 PARTIAL** 2026-10-08：收口证据系 n=1、未达三连
- ；C9 活体回归同 code 两跑相反——run1 PASS dev 0.181 / run2 FAIL state=4 dt=0 dev=3.000，失败链 age=21.491s vs tolerance 3s，且 gate=PASS 与之并存 ⇒ 门控未覆盖全局系 TF 新鲜度
- ；功能面多源自洽 0.179–0.183 不否定）/stageC 三败（SIGSEGV×2+Bond broken）→R2C --no-bond+/clock 门控 Bond 根除仍 segv 无 backtrace→终止裁决成立（M2-3 hdl 仿真不通过）
- ；M2-5 勘误 d1/d2/d3 落 04/02 核验通过
- ；隔离/样张 c034c906/foxy 四行/退净验收时点活体抽检全过
- ；**边界=仿真覆盖 faster_lio✅ teb✅ hdl❌，两层均不可替代 G8/G9 真机 Gate**
- ；报告见 [M2/进度文档.md](M2/进度文档.md)〔同被测件（XAC b32f49fd／PLG a32e70ff／SO 零重编译）、判据集为超集（gate 十字键，新增 press_late_G5）之 harness 追加三跑：正向两跑 rc=0、5 PASS/0 FAIL、G5 与 G5c2 含 late 采样全 PASS（ages 集内 0.007–0.093s、单值 0.099s 出自**设计内阴性对照跑**不入正向带）、G5c1 明细 ages 全 PASS、dev_odom 0.161–0.180、dt 8.4–8.5、四口 TCP/UDP 逐跑全零、逐跑退净落件
- ；阴性对照跑 press_late_G5=False 把整跑判死（rc=1、未发 goal）〕〔小美 M2R6 待决③ 分代际句，2026-10-08
- ；F1n–F5n 原句数字不回改〕）〔小美 R29（三）落册、小明执行（2026-10-08 11:42）〕第五代（批1 改后件：SM 28a49f89／647 行 ＋ XAC ef23da2d）新五连全绿（5 跑 rc=0、判定 7 PASS/0 FAIL、E3 base_link 父帧唯一 odom、加压档 sim2=62.39 span=39.70），**但它不充 M2-2 的连击**，只算批1 的复跑（回归）证据
- ；M2-2 回填的唯一连击仍是 F1n–F5n，**禁写 10 连**。dev 带自终稿 0.166–0.176 外扩到 0.156–0.178（下沿亦破）⇒ 依「给带不给门」，0.35 门值不动、分代际列带。M2-1／M2-4（stageA 面）批1 **未取得有效跑**（rtf_motion 0.240／0.233 两跑作废，第二次跑未收口）⇒ 在册证据仍停在改前代际，取得有效 stageA 前不得声称已复取。
- ｜小美裁定、主理人落册）
### 测试矩阵 Wave1
- **PASS**（2026-10-07 小美，独立核验：A1 M5 首跑（run0 桩跑 outcome=succeeded+实链修法#1–#6 留底 49393bfd→9f661722 sha256 链复算全等+六路标 log 逐一对应）/C11 三分支（RECOVER 恰 1 次 outcome=0、59×2 预算耗尽、abort 路由）/A3 双 planner 20 次采数独立复算（GP 15.774m/0.1063s、navfn 15.285m/0.0903s、20/20 outcome=0 零穿墙）/13 CHECK 双锚点全 PASS（W1-0/W1-F，navfn 注册零行为变化）
- ；**口径①②追认**：mbf 组合 action 透传阶段码（move_base_action.h:116 源码注释「copy outcome…」+MoveBase.action 13 vs ExePath 105 双表实证，修法#4 回退分流语义零漂移）、「ExePath 到达」T1 结构性不可达（mode_nav_sim.launch 头注边界原文逐字核验「NOT 验证：闭环导航效果」，到达归 T2=M2 stageB teb 在案，非判据弱化）
- ；隔离终态活体抽检全过（11313/11312 无监听、foxy 零触碰、oscillation_timeout 终态 0.0）
- ；注记 3 件移交小明（4.8s 口径注记/55 分支补留底/5 尝试中 2 跑补留底）
- ；证据 [仿真测试台/证据/log/matrix_w1/](仿真测试台/证据/log/matrix_w1/)，报告见 [仿真测试台/进度文档.md](仿真测试台/进度文档.md)）
### 矩阵 v2 定稿
- **追认通过**（2026-10-07 小美核验＋主理人预裁决采纳：五件齐（manifest+A/B/C(D)/E(F)）
- ；3D 维 T0–T3r 自洽+X4 延伸口径互锁（仅证路径形状、真越障归真机/MuJoCo）
- ；mesh_navigation/elevation/lvr2/dwa/evo 未装 dpkg 独立实证=Wave2/W3 预检必要性成立
- ；TerrainPlanner=mbf_abstract_core 原生插件外部佐证（AbstractPlanner→MeshPlanner→TerrainPlanner 继承链）无需 S9、仍守 W3 预检前置
- ；公开 world 项+fratopa 转换路线在册
- ；M5 两处修正现场实证对齐（50_task_smach_min.launch line5 内置 param+G-G8b 冻结判据不碰、判据件改独立 v_task_smach.sh）
- ；v4.3 §16.1–16.8 增章制与 G0–G9 零冲突（§16.7 逐字引用 X4、判据原文不动）
- ；mockamap launch 恰 5 件复核）
### 测试矩阵 Wave2
- **PASS（条件注记）**（2026-10-07 小美，独立核验：只读+离线复算零节点——壳包三件终态 sha256 fd4c62a4/aba9b658/cbbb8f0b 复算全中+diff 对留底逐 hunk 合理
- ；三图 CSV 独立复算 maze20 10/10 L̄15.988m、post2d 6/6 L̄4.848m 全中
- ；teb_run1 gt/odom 1007/428 实物对账+dev 0.183 复算+**终点 (2.817,0) 与 stageB (2.82,0) 逐位重合**+平滑度 0.0018 复算吻合=对齐性硬证据
- ；evo APE 纯 python 独立复算 428/428 RMSE 2.0630mm 逐位命中
- ；DWA 败#3 原件链完整（loaded successfully+pose 5.372s old tolerance 3s→57）+teb 同栈全过对照=**裁撤追认，teb 单选定型**（非算法否定）
- ；T1r noetic 分支无 TerrainPlanner 盘上独立验证+GSL 原件=预检结论确认
- ；隔离活体抽检全过。**四项追认**：D-W2-1 追认（B 文 Wave2.3 明文透传授权）/D-W2-2 追认附注记（stageB 无时序实证，一跑两用=工具链演示级，faster_lio APE 正式件归 Wave3 用 M1 素材）/**D-W2-3 追认承重化**（独立复测 RTF≈0.27–0.29 双时间戳锚点法——04「RTF≈1.0」不成立须更正
- ；实测过 M2R2 门 0.25、不过 C 文 0.3→全矩阵统一 0.25 为有效性前提）/D-W2-4+3D 维**采纳附修正边界**（2.5D 自研 elevation→costmap 层承载 T2r 路径形状判据、零 lvr2 依赖链
- ；诚实入册=永久放弃悬挑/多层场景，mesh_navigation ROS2 列观察）。**★新发现 N1（重大）**：perlin3d 勘察选点缺陷——mockamap perlin3D y∈[−10,10]（y_length=20 中心对称源码实证）而 w2_survey free=~occ 把云外区判自由+bridge「其余=free」→goals 3/4 个（y=−19.45）在图外假自由带，正向对弧长≈直线距实证路径走在不存在地形上，**perlin3d 6/8 数据整体无效，「75% 记档」改判「勘察修法+重做」移交 Wave3 必办**（不采纳判据修订
- ；maze20/post2d 正方形模板核验无缺陷）
- ；N2 RTF 实测矛盾/N3 DWA 败#1#2 与 perlin3d 第二轮无原件/归档脚本≠执行版/N4 口径差——均注记+路由小明补登 4 项、小张勘察修法与矩阵稿 v2.1 勘误。证据 [仿真测试台/证据/log/matrix_w2/](仿真测试台/证据/log/matrix_w2/) 48 件，报告见 [仿真测试台/进度文档.md](仿真测试台/进度文档.md) Wave2 验收节）
### 测试矩阵 Wave3
- **验收 PASS（2026-10-07 小美：补登四项核验落账+A5 双 PASS 独立复核+3D spike 设计件在档
- ；perlin3d 0/8 红线停手如实记档→移交 R2 两轮封顶，终局见下行「矩阵收官」
- ；下为原施工申报照录·2026-10-07 小明，移交四件全落账）**——①补登四项+Wave1 附注三项：04「W3 补登注记」+进度文档补登块（RTF 更正/单轮留底/归档对账+harness 版本化/门集统一 5 门），留底 raw/*.before_w3.bak（0544c535/d75c33a5）
- ；②perlin3d 修法重做（N1 必办）：w2_survey.py v3 封边修法（before 7ef42135→终版 f44277a4：模板地形封边+选点连通图与 planner 语义对齐（global_costmap 无 inflation 层实证）+0.15m 选点间隙
- ；v1 逐柱覆盖/v2 保守连通两版退化实证留底）→重选 goals 全落模板盒 [−20,20]×[−10,10]、重跑 8 对 **0/8<90% 触发红线停手**（PAIR00-04 确定性 55@3.0s planner_patience+PAIR05-07 采数中 mbf SIGSEGV×2 无效采数，上限 5/8=62.5%
- ；诊断探针=W2 已知好对 7.9m 双向 0.05–0.15s 过、GP 远角对 patience 放宽 60s 后 90s 不可终止+卡死线程堵死 reconfigure 锁=段错误机理、**NavfnROS 远角对双向同样 55=非 GP 特有**、GP 11m 中程对 55——触发域=盒内迷宫路径 ≥~11m，**第 2 轮归小张**，设计输入四件齐）
- ；③A5 组合矩阵（缩水版）**双 PASS**：GlobalPlanner×teb=state3/outcome0/dt 8.5s(sim)/dev_odom 0.171m/RTF 0.283、NavfnROS×teb=8.5s/0.178m/RTF 0.275（统一 5 门同版 harness w3_t2_exec.py 970e30e1：dt_wall 修复固化+RTF 双时间戳锚点法逐跑必探+A0 门 0.25+按名点单零配置改动
- ；对齐 stageB 基线 8.6s/0.179m）
- ；④3D spike 设计确认（不实现）：/tmp/simtb_payload/spike3d_elevation_costmap_design.md（01f30b8b）=elevation_mapping_cupy 契约对照（CPU numpy 不取 GPU）+w3_elevation_node 接口表（unknown=−1 不做「其余=free」N1 教训写死）+costmap 集成两档+预检 4 项，实现前置=小张接口复核+主理人批准
- ；壳包三件零改动（fd4c62a4/aba9b658/cbbb8f0b 不变）
- ｜证据 [仿真测试台/证据/log/matrix_w3/](仿真测试台/证据/log/matrix_w3/) 34 件+对账链，施工记录见 [仿真测试台/进度文档.md](仿真测试台/进度文档.md) Wave3 节）
### 测试矩阵收官（W3-R2+全矩阵终局）
- **收官 PASS（2026-10-07 小美，矩阵终局关闭）**——W3-R2 证据 53 件 sha256 链 52/52 全 OK+68 点单逐单复读（sector1 8/8 过、其余 60/60 确定性 55@3.0s，n_selected=1/4）
- ；A 表 perlin3d 行「裁撤：planner 长程能力不足，图与栈无责」三处口径对账一致
- ；A5 双 PASS 四跑 dev 0.171–0.183m/dt 8.5s/RTF 0.275–0.283 与 stageB 基线 8.6s/0.179m 对齐（harness 970e30e1 三方一致）
- ；04 补登四条+p3b 关闭 a4ae3eb+端口表 b55d9d7 在案。**最终状态表：M5✅/全局对比✅（20/20）/局部✅（teb 定型，DWA 裁撤）/组合✅（双 PASS）/多地图✅（perlin3d 除外）/定位仿真❌（hdl segv 转真机 G8）/perlin3d 远角❌（裁撤，两轮封顶）/3D 地形规划⏸（spike 设计件 01f30b8b 待二期）**。**边界注记定稿=仿真覆盖 faster_lio✅ teb✅ hdl❌ perlin3d 远角❌**（两层仿真不可替代 G8/G9 真机 Gate，X4 不动）。勘误 2 处（wave1 T2 引用误标→实为 W3 A5
- ；「860 万点 perlin3d」口径混引→perlin3d 云实测 116,747 点）+补档 2 件（W3 T2 stdout，w3_t2_stdout_补录.sha256）随收官落账
- ；遗留=真机域 hdl+G8/G9 本体、二期 3D spike，矩阵自身零未闭合施工项
- ｜报告见 [仿真测试台/进度文档.md](仿真测试台/进度文档.md) 收官验收节
### P0 波次 M5.1（roadmap C #2）
- **验收 PASS（口径 A 收口 · 边界注记：到达归 T2）**（2026-10-07 小美正式验收·独立核验未启节点：施工面复核——Q11 对账四 action×五件套类型/话题全一致+bak↔现役 diff 独立复算仅注释级（b93ce4c2→44e1161f）/修法#1–#6 留底 sha256 链尾 9f661722 实测全等/双跑 log 关键行逐行复读（run1/run2 同签名：wait 1.00s→get_path outcome=0 0.05s→exe_path 101 看门狗 90.01s→aborted 91.8s，101 收尾=契约路由非悬挂）/v_task_smach.sh bash-n+双跑独立复跑与留档判定逐字一致（False/exit=1）/隔离终态+foxy 五 PID+180237 抽检全过/违规项无
- ；**主理人裁决口径 A 复核成立**——①边界原文一致（mode_nav_sim.launch L12「NOT 验证：闭环导航效果」逐字在案，W1 追认+矩阵收官 M5✅ 同口径）②判据本体零弱化（到达归位 T2 闭环域已四轮实证 0.171–0.183m，M5.2 SCAN 后 T2 再验到达，判据保留在案
- ；B3 判据系设计期信息差非施工偏差）③收口目标=C12 契约链可运行（Q11+GetPath+C11 路由+判据件）已全证
- ；样张前缀勘误成立（现值 aaa09362…，c034c906 系旧值，04 P0-5 注记+验收追认）
- ；C12 覆盖=wait/get_path/exe_path 实链验+recover 经 C11 注入（W1）+navigate_to_pose 契约/Q11 在册（任务级随 M5.2 覆盖）+scan_navigate 属 M5.2（前置=SCAN Gazebo 集成 C #3）
- ｜验收 log [M5/证据/log/m51/G_M51_验收_小美复核.log](M5/证据/log/m51/G_M51_验收_小美复核.log)，报告见 [M5/进度文档.md](M5/进度文档.md) M5.1 正式验收节）
- ｜施工申报（2026-10-07 小明，commit aa8a6c7）：设计稿 A–E 五件入库（sha256 全等）+Q11 头注回填+双跑 aborted 判据一/二未达按红线停手+判据件新增（13 CHECK 零触碰）+04 P0-1..P0-5
- ｜证据 [M5/证据/log/m51/](M5/证据/log/m51/)）

### P0 波次 SCAN 集成（roadmap C #3 · A7 P1+P2）
- **P1 判据① PASS(6/6) + P2 判据② PASS(7/7)
- ；共同门禁 ISOLATION FAILED 但 SCAN 净贡献 0 命中**（2026-10-08 主理人直接执行·非成员产出，因施工代理连续 5 次 API 非正常退出、为守「不留档=没完成」接手）——**§B1 拷包**：SCAN 五包原样入 ws/src/30_3rd，50 文件对基线 sha256sum -c 50/50 OK、源侧指纹 75444790c263a00c 未变、build/devel/.git 命中 0
- ；**§B2 壳包**：lite3_scan 六件（40_navigation 与 lite3_navigation 同层），70_scan.launch 逐字沿用小张草稿、scan_params.yaml 八键对 advanced_param.xml:112-119 零默认行为变化
- ；**§C 编译**：make -j2 / BUILD_EXIT=0 / 816s / 无 -w / 告警 0
- ；零回归四条主理人独立复算成立（15 个 M2 关键产物 sha256 逐个全等、CATKIN_WHITELIST 仍空、devel/lib 63→78 新增全属本次 6 包与 reconfigure .pc、ws/ tracked 零 M）
- ；★成员对账 log 第 1 条「基线产物条数=0」系空集真命题/假证据，已记为不采信
- ；**F11 预检**可用（scan_planner_node 15.6MB + closed_loop_controller 3.7MB，4m32s，-j2 足够禁 -j6）。**★两处硬缺陷已根除**：① goal remap 从未生效——<include> 不支持 remap，roslaunch 报 WARN 并静默忽略 ⇒ A1/A3 防双消费失效，修法=remap 改挂外层 <group>（第三方 advanced_param.xml 一字未动），静态实证 scan_planner_node remap 含 /move_base_simple/goal→/scan_planner/goal
- ；② 时基缺陷由便利编排件 75 的 ns="gz" 把 m2_gazebo 相对名 use_sim_time 关进 /gz/ 造成，按裁决④删除该件（§D 起链本为三终端各起、该件零消费
- ；三终端形态下缺陷本不存在）⇒ [5] 净新增归零。**判据①实测**：/grid_map/occupancy 152 帧/30s=5.07Hz、点数 4914→5331(+8.5%)、frame_id=world、包围盒 8.95×8.85m=79.2m² 非退化、落在 LIO 累积云界内 True
- ；口径边界=局部滑窗图与 LIO 累积云几何相容，**不等于**与 world 全部墙位逐格对齐（且本轮机器人静止）。**判据②实测**：react_s=0.10s（门 ≤5s）、Bspline order=3/n_ctrl=19/n_knots=23、120 采样点距最近占据 min 0.625m/median 0.811m、落入 0.10m 硬阈点数=0、订阅归属 /scan_planner/goal=[scan_planner_node] 且 /move_base_simple/goal=[]（双消费排除）。**观察 O-1（挂小张，P3 前置）**：endpoint_err=1.027m 轨迹终点越过 goal 约 1m，且 SCAN 无显式 reached 话题 ⇒ A7 判据③「FSM reached」若沿用轨迹终点会误判，P3 派单前须定到达信号与终点容差口径。**红线**：样张 lite3.pcd sha256 全程起前=退后、foxy 六 PID 启动时刻逐位不变、11312 零监听、move_base 节点数=0（F9 互斥）、ros2qnx 零、退净残留空+11313 释放。**门禁归因**：[1][2][3][4][7][8][9] 过
- ；[5]4+[6]3 全落 95_simulation/lite3_gazebo_sim（M2 预存在，最后全量 OK=10-05 04:33）→ 挂 M2 返工待办，本阶段不判 SCAN 违规亦不判 ISOLATION OK。**主理人两处自纠**：① §B1 曾报「/home/ysc 命中 0」系把 grep 输出的 1 误读为 0，真命中文件 pcd2grid_local.launch（注释记原作者本机 octomap_server 路径 + 参数硬写 /home/test/.../lite3.pcd）已由 [8] 抓出，处置=从沙箱副本删该死件（不在任何 include 链上
- ；留底 sha256 9aeebfcf…
- ；/home/test 原件未动），拷包基线 50→49 文件
- ；② 曾判「<6> 是脚本口径缺陷、建议改语义」，经 04 L31 先例（当年注释命中处置=改文案不动脚本）撤销该建议，[6] 维持文本级红线。另记 02 坑17：编辑工具走 temp+rename 会冲掉已补 named ACL ⇒ setfacl 须随每次编辑执行
- ；本机无免密 sudo，改 ysc 落盘+setfacl 补 u:ros1-slam:rwx（882 项已补，devel/lib 15/15 覆盖），残留 ws/build
- ｜devel 目录 default ACL 需 ros1-slam/root 侧一次性补。commits 39ebbd8/c5edcbf/8d50880/e5e1a45/20aabc4 + 本次 P2 收尾批
- ｜主理人登记


> 备注（2026-10-04）：**P4 依赖核对（21/21 命中全 /opt/ros）+ P5 编译构建（五批全 PASS 零 OOM + WRAP2 纯默认收尾）已完成，Gate G4 PASS**（devel 产物可解析、teb/mbf/adapter 系统版证明齐备、swap 撤净、CATKIN_IGNORE/模板修正落账完整
> ；证据 [P5/证据/G4.log](P5/证据/G4.log)，报告已追加 [P5/进度文档.md](P5/进度文档.md)）。P3a 全部施工（S1–S3，C0–C13 共 26 文件）已完成且 Gate G3a PASS（[P3a/证据/G3a.log](P3a/证据/G3a.log)）。P3a.0 mbf 组件实测证据 690 行：[P3a0_mbf组件实测.log](计划书v4升级/证据/P3a0_mbf组件实测.log)。S9（ros-noetic-nav-core-adapter 0.3.0）已装、Q9 关闭。
