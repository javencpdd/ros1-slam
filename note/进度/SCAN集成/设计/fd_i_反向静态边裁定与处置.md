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
