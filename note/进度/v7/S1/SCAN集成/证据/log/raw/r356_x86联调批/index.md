更新时间: 2026-10-18 03:3x（`date '+%F %T'` 现跑取）｜本索引描述 R356 联调批证据包，随提交入库

# r356_x86联调批 · 索引（格③补测窗＋A1 等价基准＋预检变体）

- **批次身份**＝真人 2026-10-18 到签 R356：①格③补测窗按**联调**跑（读数/日志/bag 三样存证、**自称旁证**；窗号登记与转正随批量走、不阻塞）＋x86 侧预检变体（新件、在册 `v_window_preflight.sh` 一字不动；跑通 rc=0 即视为预检 rc=0）；③按 `au` 稿 §5.1／§6.2 出 **A1 等价基准件**（断言①登记"覆盖面变化：异机无此对象"不记通过；②③④⑤逐条现跑工件入仓）；出完按到签②由 **x86 侧独立小美实例**判定签注。
- **执行入口**＝`x86_link_driver.sh`（本目录根层；host 视角驱动，容器内经 docker exec；bash -n OK）。
- **★本窗四次尝试的准确形状（按判定侧 2026-10-18 03:51 校准）**：**第 1 次＝真失败**（批7 package.xml 回写引入的 catkin_pkg InvalidPackage ⇒ roscore 起不来；已修并登记，见 `04` R356）；**第 2—3 次＝探针整段跑完且段级合格**，失败在**驱动外壳**（容器 ENV 的 PATH 不含 `/opt/ros/noetic/bin` ⇒ 外壳未 source 时 `rosnode: command not found`，轮询空转 45s 假报"roscore 未起"，致 bag 未起、A1 窗中/退净未采；已修）；**第 4 次＝全要素齐（本包读数）**。★四次均未被当成被试失败或合格——纪律形状照守。
- **★引用勘误（判定侧发现，另起句）**：到签原文写「au §5.1」；`grep -n "5\.1"` 实查 au 稿**无此子节**（§5 后直接 §6.1）⇒ 本包按 **au §6.2＋§5 表**执行，引用按此登记。
- **★脱敏记录**：`a1_baseline/assert1_unit_coverage.txt` 第 14 行曾把沙箱 PID1 的环境串（含本会话 MCP token 字面）截入留底，已于 2026-10-18 03:5x **整行脱敏替换**（原件不入库、不引用；判定件遗留项 7）。

## 目录地图（四件套＋根层）

| 目录／件 | 内容 | 清单 |
|---|---|---|
| `x86_link_driver.sh` | 执行入口（驱动） | 根层清单收 |
| `window_container_run.log` | 容器段全量输出（tee） | 根层清单收 |
| `preflight_variant/` | 预检变体**当刻**读数（rc=0 放行行） | 1 条 |
| `x2_cell3_window/` | 格③补测窗原始件（四件齐形状＋bag） | 13 条 |
| `a1_baseline/` | A1 等价基准件（au §6.2 五断言＋退净） | 10 条 |
| `acceptance_verdict/` | 判定件（到签② x86 侧独立小美实例，非终审） | 1 条 |

## 格③补测窗读数（x2_g6 四插件段；探针在册件原样跑、rc=0）

- **探针结论行（逐字）**＝`X2 四格读数 = 段数=1｜原始件合格=1／1｜不合格=0／1｜N3 段=无`；`X2_FOURCELL VERDICT rc=0（所选 1 段原始件齐＝1／1；★段数不足 6 ⇒ 只报本段合格、★不得读作四格跑齐）`。
- **判据字段（`run.log` 解析行逐字）**＝`data_行数=1 格数=1600 判别区=[100, 100, 100, 100, 100] A独占50点=0 B独占100区=[100, 100, 100] 未知区=[0, 0, 0, 0, 0] 第二帧新置区=[0, 0, 0, 0, 0] distinct=[0, 99, 100] 254_格数=0`。
- **读法（照探针写死句）**＝判别区=[100×5]⇒A 的 lethal 保住（取大语义）；第二帧新置区=[0×5]⇒只吃首帧（first_map_only 拦在后；first_map_only=true）；A独占50点读回 0＝在册 trinary 把 50 抹成 0；`distinct=[0,99,100]` 中 99 值的来历时录未注（★留待判定侧读 `cost_x2_g6.txt` 原件）。
- **★两项照实报的限制**＝①`node_x2_g6.log`＝**0 字节**（stdout 缓冲未及落盘即被 kill；同型 `tf_base.log`／`tf_r_x2_g6.log` 亦 0）⇒ ao §2 段2 的「插件装载行数≥4」「StaticLayer_装载数=2」两字段**无源可 grep**——即 R332 已登记的两项证据缺陷之一**在乙案范围内未 remediate**（段3 stdbuf 属甲案第三段，乙案明示另排）；②`cloud_x2_g6.log`＝`X2_CLOUD first sent pts=42 frame=rslidar topic=/rslidar_points tag=x2_g6`（101 字节）＝**#250 最小修后的活体首帧成功行**（R332 的 `AttributeError` traceback 不再现）。
- **bag**＝`x2_g6_window.bag`（1,128,360 字节；话题＝/x2_g6/costmap/costmap＋costmap_updates／pmap_a／pmap_b／rslidar_points／tf／tf_static；`rosbag_record.log` 646 字节）。
- **四件齐（格④ 纪律格原形）**＝`cost_x2_g6.txt`（rostopic echo -n1，5,206 字节）＋`params_x2_g6.yaml`（3,081 字节）＋`node_x2_g6.log`（0 字节，见上）＋`run.log`（5,914 字节，含首末时间戳与退净行）。

## 预检变体当刻读数（到签①：跑通 rc=0 即视为预检 rc=0）

`PREFLIGHT VERDICT rc=0（放行：ISO_RC=0／numpy 1.19.4 在 /usr/local 下／load1=5.68≤36.00｜核归一化＝核数 72 × 每核阈值 0.5）`（ISO 走乙式包装件、self-attest 行 script sha `8e2c2c02…` 未变、ws_src_files=4580）。★在册 `v_window_preflight.sh` 一字未动；变体＝`scripts/verify/v_window_preflight_x86_variant.sh`＋`scripts/verify/_iso_alt_wrapper.sh`（syn_face 89→91 同批刷）。

## A1 等价基准件（au §6.2 五断言；工件逐条在 `a1_baseline/`）

| 断言 | 现跑读数 | 判定形状 |
|---|---|---|
| ①六 unit systemd 态 | 六枚全部 `Failed to connect to bus`（沙箱无 systemd 面） | ★登记「**覆盖面变化：异机无此六 unit 对象**」——**不记通过**（au §6.2 明示）；**等价物已补**（真人终端 2026-10-18 04:0x 现跑入册 `assert1_host_load_top.txt`：workbuddy 组 4 枚 ~177%（zygote 95.1% 单枚偏高）／gnome-shell 19%／Xorg 15%／openclaw 17.5%／Qoder 13.1%／内存余 10G／无常驻仿真负载） |
| ②foxy 残留零进程 | `pgrep -x run_mapping_online/livox_ros_driver/ros2qnx`＝空（起前 03:28:30＝退后 03:29:17 两刻同）；`/opt/ros/foxy` 不存在 | 起前＝退后同形 |
| ③样张 lite3.pcd sha256 | 起前＝退后＝`aaa09362c5f1aef93d2cbbb36f4b71f42ce32a5c98083f6c379c39cb27146f34`（与 au §4.4 锚串逐字等） | 可完整等价 |
| ④1131x 端口 TCP/UDP 分查 | host 起前 TCP=2／退后 TCP=2（★2＝容器 11313 映射面、起前＝退后同形）；UDP 均 0；container 起前 0/0；退净后 container TCP=UDP=0 | 起前＝退后同形（host 侧 2 的来由已注） |
| ⑤ros2qnx/qnx 零出现 | 窗中 `rosnode list \| grep -c qnx`＝0＋全表留底（仅 /rosout 与 bag recorder）；`pgrep -x ros2qnx`＝空 | 跨机照搬无豁免、零出现 |
| 退净形状 | `pgrep -x gzserver=0 gzclient=0 roslaunch=0 roscore=0 rosmaster=0`＋container 1131x TCP/UDP 全 0 | 与 au §6.2 原句形状同 |

## 判定（到签②：x86 侧独立小美实例）

判定件＝`acceptance_verdict/verdict_x86_acceptance.md`（117 行｜判定时刻 2026-10-18 03:51｜与施工侧零协商、非终审）。要点：**判据一**＝等价件齐但非全绿（②③⑤ 成立／① 覆盖面变化不记通过＋host 榜首无源／④ 同形等价物成立＋"零监听"原义在异机不成立／退净 container 成立·host 端口非零）；**判据二**＝段级有效读数成立、五项判据式 **3 有源·3 无源**（其中 `stvl_标注列` 全批无生产者）、停手条件未触发；**独立复验**＝三目录清单非 OK 行 0（24 行）＋`X2_PARSE_SELFTEST PASS=4 FAIL=0 rc=0`。档名＝旁证·未计分子；★不撤 §16.9 拦人句；遗留项 13 条（归属逐条见判定件 ⑥）。

## ★留底形状限制（判定侧指认）

`x2_cell3_window/run.log` 为驱动 `tee -a` 的**四次尝试累计件**（[19:07:10]／[19:18:29]／[19:23:56]／[19:28:34] 四组开跑行）；**最终读数以 [19:29:10] 行为准**；前几组同形读数只作可重复性旁证，★不得记成 3—4 个窗、不得据以推窗数。

## ★自称旁证声明（到签①原文口径）

本批全部读数**自称旁证**：联调所得、★**不计入在册判据分子**（au §8 待裁①＝A56 家族未裁，主理人未勾）；引用必须带「旁证·未计分子·x86 联调·2026-10-18」限定词；★转正与否归判定侧（本批由 x86 侧小美实例出判定记录，终审保留 Lite3 原册）；★本包**不声称** X2 已解决、不声称四格跑齐（仅格③ 一段合格）、不撤 §16.9 拦人句、不声称任何门成立。

**P8**＝计划书未定稿、现行真身＝v5.3（2D 导航线范围内终稿效力在位）；本索引只描述证据、不下判定；**推荐不是判据**。
