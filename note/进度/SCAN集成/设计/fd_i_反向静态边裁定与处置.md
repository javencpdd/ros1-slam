# FD-I 反向静态边：小张裁定与主理人处置（2026-10-08）

更新时间：见文内登记行。性质：零仿真、零节点；改动面=一个死件的 arg 缺省值与件头注释＋一条静态检查件。

## 一、事实（本轮独立复核，不只信转述）

- ws/src/30_perception/deeprobotics_local_height_map/launch/height_map_mini.launch:64
  static_transform_publisher name=base2urdf，args 末两端 = /base_link（父）与 /body（子）；
  同件 :7 的 enable_urdf 缺省 = true，:63 的 group if=enable_urdf 包住该行 ⇒ 缺省即生效。
- ws/src/30_3rd/plan_manage/launch/lio_relay.launch:57–58 的 body_to_base_link args 末两端 = body（父）base_link（子）。
  ⇒ 同两端点、反方向。两 launch 一旦同窗：TF 有向图成环，且 base_link 复现双父（与 M2R4/U1 刚闭合的病根同型，02 坑21 家族）。
- **全包穷尽扫描**（本轮新增，小美只点了那一条）：该包 launch 内所有 static_transform_publisher 的父子对里，
  以 body 为子的**只有 :64 一处**；其余为 /base_link→camera_*、base_link→rslidar、base_link→imu、/base_link→/dummy，不与本仓在册边反向。
- **死件与不可起两面**：全仓（ws/src、scripts、note、根 md）对 height_map_mini 的消费性引用 0 命中
  （唯一命中是 note/进度/REPO_INIT/证据/git_init.log 的 create mode 清单行，非消费性）；
  且该包根有 CATKIN_IGNORE（0 字节，实测存在）⇒ 不参与建沙箱编译；
  同件 :65 引用 mini 包的 urdf，而 **mini 包在本沙箱不存在**（rospack find 与 find 双查皆空）⇒ 即便有人起它，
  缺省态下 $(find mini) 亦会让 roslaunch 报错。故它不是「会静默发作」的活雷，而是「换环境即可能发作」的埋雷（真机侧带 mini 包与 height map 栈时）。
- 基线无关：该件不在 30_3rd 的 49 件清单内（deeprobotics／30_perception 在清单内零命中），
  所以动它不触发 R-FD-3「改 30_3rd 件即失基线」的限制。

## 二、小张裁定（R31 要求的裁定，形态＝保留并解除缺省，不删不改名）

1. **不删除、不 .STALE 改名**。理由：它不是弃件而是**与 lio_relay 互斥的件**——真机 G8 侧若要用局部高度图栈，
   删除或改名都会让 G8 需要重新找源；而该包已 CATKIN_IGNORE 且在沙箱不可起，缺省态不发作的条件已由处置补齐，
   无需以停用形态表达「勿用」。
2. **采 R31③ 的 (i)＋(ii) 双施**：enable_urdf 缺省 true→false（上游自己在 height_map_lite.launch:7 就是 false，
   故此改不造新语义、只是与本包其它件对齐），并在件头写明与 lio_relay 互斥、需可视化时显式传 enable_urdf:=true 且不得与 LIO 同窗。
   可逆口径＝**就地改缺省值＋注释这一种**（不并存删除留底与改名两式，依她 R31② 「二选一」）。
3. **静态防线常驻**：新增零节点检查件 ws/src/40_navigation/lite3_scan/verify/v_tf_edge_conflict.sh，
   扫全仓 launch 的反向边对与缺省态，作为 G8 前置检查表的一项；它不进门禁 v3_isolation（进门禁属判据面变更，需小美落笔）。

## 三、升阻断条件（照小美原文执行）

任一在册或在跑窗实测 base_link 出现双父，或 /tf_static 同窗共存 base_link→body 与 body→base_link 两条边
⇒ **停手上报、判 TF 结构回归，不走 INVALID 通道**。

## 四、向前指向（不回改旧句）

在册工单 #29 与前一轮台账里 FD-I 的口径是「死件，待处置／不删不改」。本节把处置定形为「保留＋缺省解除＋互斥注记＋静态检查件」，
以本节为准；旧句按纪律不回改。｜小张裁定、主理人执行、小明落件，2026-10-08

## 五、R42 承接（小美 M2R8 三条，小张定文、小明落件，2026-10-08 12:47）

**（一）件头注的自指锚（已落件）**：HMM 注记块原先写「本件 L64 的 base2urdf」——但我自己加的注记把那行推到了下方，
行号随之失真（她第 23 条核出）。现改为**以 name=base2urdf 作自指锚**并在注记里写明「用 name 定位而非行号，
因注记块自身会把行号推走」。这不属判据面变更，属注释实体的名实相符修正。

**（二）G8 前置检查表新增项的文字（小张定稿，抄入时不得改写语义）**：

> G8 开工前置第 N 项（TF 结构面）：在隔离域三步环境下跑
> ws/src/40_navigation/lite3_scan/verify/v_tf_edge_conflict.sh，要求 VERDICT 7/7 PASS 且 rc=0；
> 其中关节面两项为真机侧新增——C6a 必须认出 base_to_torso 的 child=TORSO（扫描器分辨力的阳性对照），
> C6b 要求全仓 xacro／urdf 中以 base_link 为 child 的关节块命中 0 条。
> 另核：deeprobotics_local_height_map/launch/height_map_mini.launch 的 enable_urdf 缺省仍为 false；
> 若真机侧确需该 urdf 可视化（传 enable_urdf:=true），则**不得与 30_3rd lio_relay 同窗**，
> 同窗即 base_link 双父＋TF 成环，属结构回归而非环境问题。
> 升阻断条件（原文不变）：任一在册或在跑窗实测 base_link 出现双父，或 /tf_static 同窗共存
> base_link→body 与 body→base_link 两条边 ⇒ 停手上报、判 TF 结构回归，不走 INVALID 通道。

**（三）扫描面扩到关节（裁定＝扩，已落件）**：她给的事实我复核为真——M2R4/U1 那一次双父的成因不是 launch 静态边，
而是 xacro 的 fixed 关节 torso_to_base（parent=TORSO／child=base_link）与插件动态边争同一子帧；而本防线 C1–C4
只扫 launch 的 static_transform_publisher ⇒ 关节面是**结构性盲区**，正是「U1 的病因复发却查不到」那一型。
扩面实现为 C6a／C6b 两条（v_tf_edge_conflict.sh），要点三条：①关节块跨行是常态，故按整元素匹配（同坑29 的教训，
逐行扫会静默漏）；②**先阳性对照**（必须认出 base_to_torso 的 child=TORSO）再谈零命中，否则「34 个关节块命中 0」
这类句子在空扫描面上也成立；③沙箱现况实测＝文件 3 个、关节块 34 个、以 base_link 为 child 的命中 0 条
（⇒ 单父不变式当前成立，且这一条同时是 U1 修正仍在位的原件证据，不必再靠登记句反推）。
判据未扩任何门值：C6 是**结构断言**（子帧归属唯一性），与新鲜度／漂移阈值无关，故不触发她 R-FD-5 的改门程序。｜小张定文、小明落件
