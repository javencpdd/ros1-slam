#!/usr/bin/env bash
# v_w3_terrain_design_contract.sh —— roadmap C #6/#7/#8 二期的**零节点件面**：预置判据件与设计稿逐字对账（执行清单 ③.4；小明 2026-10-09）
#
# 这件**不实现任何东西**，也不改任何门值。它做三件事：
#   ① 把在册设计件（spike3d_elevation_costmap_design.md，落账 sha=01f30b8b…，commit 5867efe）的**接口表、三通道判据、
#      unknown=−1 语义、能力边界、开工前置**逐条按字面钉住 ⇒ 设计稿若被**静默改**（§9 明写要走矩阵稿 v2.1 勘误），
#      本件先红。基线不在这里手抄：随证一份 sha256 基线件（见 W0），可复算。
#   ② 盘上核实「**未开工**」是有物证的、不是我以为的：w3_elevation_node 不存在、bringup config 无 elevation yaml、
#      ws/src 无 lvr2/mesh_navigation/CuPy 路线痕迹、第三方沙箱基线 49 件双条件全等。
#      这些走 **ABSENT 档**＝「按设计尚未存在」，**不计入通过数**（05 的跑不动≠跑不过；把 ABSENT 读成绿＝坑29 那族）。
#   ③ 把在册**口径空白**原样列出（不替他/她补数）：路径长差无数值阈、T2r 三连/单跑未定、节点落哪个包未定、§8 预检 1–4 未跑、
#   ★R280 依验收工程师-小美会签（有条件签，四改逐字落）改注：本件 ② 的"未开工物证"一段在 S1／S2 落盘后
#     由 ABSENT 改为"开工后正确形态"的联合判（落点唯一＋规范位 yaml 在位），且"放行未开工"亦改判真红；
#     rc 语义 0／1／5／9 四档一字未动、--negctl 未动、本件仍不在 gate_chain.txt（入串与否归 #12／#19）。
#      以及**程序前置「小张接口复核＋主理人批准」未销 ⇒ 施工侧无权开工**。
# 阴性对照（05 C4a：新 gate 先真的红一次，且可复跑）：--negctl 拿设计稿副本改掉一处门值 ⇒ 本体必须真红。
#
# 用法：bash v_w3_terrain_design_contract.sh              对账正跑
#       bash v_w3_terrain_design_contract.sh --negctl     阴性对照（期望本体真红）
# 退出码（本件自有层，不与 G35／FDH／ISOLATION 各层互认）：0=对账全过；1=有 FAIL；9=前置不成立（设计稿/基线缺件）；
#   5=阴性对照未真红（即本件无分辨力）
set -u
ROOT=/home/ros1-slam
SELF=$(readlink -f "$0")
DOC=$ROOT/note/进度/仿真测试台/证据/log/matrix_w3/spike3d_elevation_costmap_design.md
BASE=$ROOT/note/进度/仿真测试台/证据/w3_design_baseline_01f30b8b.sha256
B3=$ROOT/note/进度/SCAN集成/证据/scan_src_baseline_49.sha256
PKG_LITE40=$ROOT/ws/src/40_navigation
# 在册只写「参数规范位 lite3_bringup/config/」（§4.3），**没写它在哪一层** ⇒ 路径按包名发现，不硬编码层号
#   （第一版我把它写成 ws/src/10_bringup/… 而实际在 70_bringup/…，于是"目录不存在"被判成 FAIL——
#    假红不是我判得严，是我把在册没写的东西当已知，见 02 坑42 追加条）
PKG_BRINGUP=$(find "$ROOT/ws/src" -maxdepth 3 -type d -name lite3_bringup 2>/dev/null | head -1)

if [ "${1:-}" = "--negctl" ]; then
  [ -f "$DOC" ] || { echo "缺设计稿：$DOC"; exit 9; }
  T=$(mktemp -d /tmp/w3neg.XXXXXX); trap 'rm -rf "$T"' EXIT
  sed 's/slope>25°/slope>30°/' "$DOC" > "$T/doc_mut.md"
  n0=$(grep -cF 'slope>25°' "$DOC"); n1=$(grep -cF 'slope>25°' "$T/doc_mut.md")
  echo "=== W3 设计对账 阴性对照：在册含 slope>25° $n0 处 ⇒ 副本含 $n1 处 ==="
  [ "$n0" -ge 1 ] && [ "$n1" = 0 ] || { echo "改写未生效 ⇒ 阴性对照空转，判 9"; exit 9; }
  W3_DOC="$T/doc_mut.md" bash "$SELF" > "$T/red.log" 2>&1; r=$?
  echo "  本体实得 rc=$r（期望 1＝有 FAIL），红行数=$(grep -c '^  \[FAIL\]' "$T/red.log")"
  grep '^  \[FAIL\]' "$T/red.log" | head -4 | sed 's/^/      /'
  if [ "$r" = 1 ] && [ "$(grep -c '^  \[FAIL\]' "$T/red.log")" -ge 1 ]; then
    echo "NEGCTL VERDICT: 门值一改本件即真红 rc=0（对账有分辨力，不是恒绿）"; exit 0
  fi
  echo "NEGCTL VERDICT: 门值改了本件仍不红 ⇒ 对账是空气，判 rc=5"; exit 5
fi

DOC=${W3_DOC:-$DOC}
[ -f "$DOC" ] || { echo "缺设计稿：$DOC ⇒ 前置不成立"; exit 9; }
NP=0; NF=0; NA=0
ok()   { NP=$((NP+1)); echo "  [PASS] $1"; }
bad()  { NF=$((NF+1)); echo "  [FAIL] $1"; }
absn() { NA=$((NA+1)); echo "  [ABSENT] $1"; }
has() { grep -qF "$1" "$DOC"; }

echo "=== W3 设计对账（设计稿 sha=$(sha256sum "$DOC" | cut -c1-12) 行数=$(wc -l < "$DOC")）｜ABSENT 不计通过数 ==="

echo "--- W0 基线可复算（不手抄 commit 号）---"
if [ -f "$BASE" ]; then
  if (cd "$ROOT" && sha256sum -c "$BASE" >/dev/null 2>&1); then ok "W0 基线件在场且 sha256sum -c 过（$(cat "$BASE" | cut -c1-12)）"
  else bad "W0 基线件在场但校验不过 ⇒ 设计稿与落账基线脱钩"; fi
else
  absn "W0 基线件不在场（$BASE）⇒ 本次首跑即为它建档，建档后该项转 PASS 由复跑证明"
fi

echo "--- W1 输入流契约逐字（§3/§4.1）---"
for pat in "/rslidar_points | sensor_msgs/PointCloud2" "/quad_0/body_pose" "/elevation_grid | nav_msgs/OccupancyGrid" "/elevation_stats"; do
  has "$pat" && ok "W1 在册原文含「$pat」" || bad "W1 缺原文「$pat」"
done

echo "--- W2 输出语义与 unknown 口径（§4.2/§4.3，N1 教训写死那一句）---"
has "−1=未知" && ok "W2 unknown=−1 在位" || bad "W2 unknown=−1 不在位"
has "未覆盖≠自由" && ok "W2 「未覆盖≠自由」的禁令在位" || bad "W2 禁令被改 ⇒ 会退回 W2 那版「其余=free」同款假设"

echo "--- W3 三通道判据门值按字面钉住（本件**不改门值**，只核在册值未被静默改）---"
for v in "slope>25°" "step>0.15m" "cost 100" "NaN 占比>30% 记 WARN" "A0 RTF 门 0.25"; do
  has "$v" && ok "W3 门值原文「$v」在位" || bad "W3 门值原文「$v」不在位 ⇒ 判据漂移，须小美定档后走勘误、我不自改"
done

echo "--- W4 能力边界与依赖链（§7＋Wave2 修正边界）---"
has "永久放弃" && ok "W4 悬挑/多层/桥洞的永久放弃仍写明" || bad "W4 放弃条款被改 ⇒ 二期范围被悄悄放大"
LVR=$(find "$ROOT/ws/src" -maxdepth 3 \( -iname '*lvr2*' -o -iname '*mesh_navi*' -o -iname '*elevation_mapping_cupy*' \) 2>/dev/null | wc -l)
[ "$LVR" = 0 ] && ok "W4 盘上零 lvr2/mesh_navigation/cupy 路线件（命中=$LVR）" || bad "W4 出现 lvr2/cupy 路线件 $LVR 个 ⇒ 违「零 lvr2 依赖链」追认边界"
# 只认**真实用法**（python import / package.xml 依赖声明），不认散文里提到这个词——
#   第一版按 `cupy|CuPy` 全文 grep，把**本件自己的注释与模式串**算成了命中（自匹配假红，坑39/40 同族）。
GPU=$( { grep -rlE '^[[:space:]]*(import|from)[[:space:]]+cupy' "$ROOT/ws/src" 2>/dev/null
         grep -rl 'cupy' --include=package.xml "$ROOT/ws/src" 2>/dev/null; } | sort -u | grep -vxF "$SELF" | wc -l)
[ "$GPU" = 0 ] && ok "W4 自研面无 cupy 的真实用法（import／package.xml 依赖，命中 $GPU）" || bad "W4 出现 cupy 真实用法 $GPU 件 ⇒ GPU 路线已被 GLFW segfault 排除，须公开回报"

echo "--- W5 开工前置的**在场性**（★R280 依验收工程师-小美会签由"未开工物证"改为"开工后正确形态"；ABSENT 档在下述两条作废）---"
HAS_REL=$(grep -c '放行 3D' "$ROOT/note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" 2>/dev/null)
if has "小张接口复核" && has "主理人批准" && [ "$HAS_REL" -ge 1 ]; then
  ok "W5 §9 批准半已销于 v5.3 §0.14（主理人明文「放行 3D」命中 $HAS_REL 处）；**小张接口复核半未销** ⇒ 本件不声称 §9 全销"
else
  bad "W5 §9 前置句被删或放行明文不在计划书里（文字命中／明文命中＝$HAS_REL）⇒ 这才是真正的红线"
fi
NODE_LANDED="$ROOT/ws/src/30_perception/lite3_perception/scripts/w3_elevation_node.py"
NODE_ALL=$(find "$ROOT/ws/src" -type f -name 'w3_elevation_node.py' 2>/dev/null | wc -l)
if [ -z "$PKG_BRINGUP" ]; then
  bad "W5 按包名找不到 lite3_bringup（在册规范位所在包不在盘上 ⇒ 对账无锚，先修这条）"
elif [ ! -d "$PKG_BRINGUP/config" ]; then
  bad "W5 §4.3 的规范位目录不存在：$PKG_BRINGUP/config"
else
  Y_SPEC=$(find "$PKG_BRINGUP/config" -name '*elevation*' 2>/dev/null | wc -l)
  # 不用 `grep -vxF <(...)`：进程替换是管道、grep 读模式文件时要 seek ⇒ 静默不生效（实测报 1 而非 0，见 02 坑18 追加例）
  Y_ELSE=$(find "$ROOT/ws/src" -name '*elevation*.yaml' 2>/dev/null | grep -v "^$PKG_BRINGUP/config/" | grep -v '/deeprobotics_local_height_map/' | wc -l)
  if [ -f "$NODE_LANDED" ] && [ "$NODE_ALL" = 1 ] && [ "$Y_SPEC" -ge 1 ]; then
    ok "W5 联合判过：落点件在位且全盘唯一（命中 $NODE_ALL）＋规范位 yaml $Y_SPEC 件（小张 R278 落点裁定形态）"
  elif [ ! -f "$NODE_LANDED" ] && [ "$NODE_ALL" -gt 0 ]; then
    bad "W5 节点落错包：裁定落点缺件而全盘命中 $NODE_ALL ⇒ 违落点裁定"
  elif [ -f "$NODE_LANDED" ] && { [ "$Y_SPEC" = 0 ] || [ "$NODE_ALL" -gt 1 ]; }; then
    bad "W5 半件状态：节点在位但规范位 yaml=$Y_SPEC／全盘节点=$NODE_ALL ⇒ S1 与 S2 必须同批"
  else
    bad "W5 放行未开工：v5.3 §0.14 已明文放行而全盘节点命中 $NODE_ALL＝0 ⇒ 依会签此档由 ABSENT 改为真红"
  fi
  if [ "$Y_ELSE" -gt 0 ] && [ "$Y_SPEC" = 0 ]; then
    bad "W5 elevation yaml 不在规范位：规范位 0 而其他位 $Y_ELSE 件 ⇒ 违 §4.3 规范位"
  else
    ok "W5 elevation yaml 位置对账（规范位 $Y_SPEC／其他非第三方位 $Y_ELSE）"
  fi
fi

echo "--- W6 隔离红线兼容（未来落位不得触 [9]；现在就核一次，别等开工才发现）---"
HITS=$(find "$PKG_LITE40" \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' \) 2>/dev/null | wc -l)
[ "$HITS" = 0 ] && ok "W6 40_navigation 零算法源码（命中 $HITS，与 v3_isolation.sh [9] 同式）" || bad "W6 40_navigation 出现算法源码 $HITS 件 ⇒ [9] 会红，二期落位不能选这层"
ISO3=$(cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$B3" 2>/dev/null | grep -c ': OK$')
CNT=$(find "$ROOT/ws/src/30_3rd" -type f ! -name '*.sha256' 2>/dev/null | wc -l)
if [ "$ISO3" = 49 ] && [ "$CNT" = 49 ]; then ok "W6 第三方沙箱双条件全等（可验 $ISO3＝计数 $CNT＝49，D2 口径）"
else bad "W6 第三方基线不符（可验 $ISO3／计数 $CNT，应均 49）⇒ 二期不得在扰动过第三方的前提下起跑"; fi

echo "--- W7 §8 预检与对照锚点的**在册承载件**（跑没跑不在这件里声称）---"
for t in "grid_map_msgs" "116,747" "13 CHECK" "sim_map_bridge"; do
  has "$t" && ok "W7 §8 预检项原文含「$t」" || bad "W7 §8 预检项「$t」被删 ⇒ 前置清单不完整"
done
A1=$ROOT/ws/src/50_task/lite3_task/verify/v_task_smach.sh
if [ -f "$A1" ] && grep -q '13 CHECK' "$A1" && bash -n "$A1" 2>/dev/null; then
  ok "W7 maze2d 零行为变化的 13 CHECK 锚点件在位且语法过（$A1 行号 $(grep -n '13 CHECK' "$A1" | head -1 | cut -d: -f1)）"
else bad "W7 13 CHECK 锚点件缺失或语法不过 ⇒ §8.3 的对照物没有载体"; fi

echo "--- W8 口径空白清单（在册就是空的，我不替他/她补数；逐条须裁定后才可开工）---"
echo "  1) T2r 的「路径长差」在册只有'形状可辨'，**无数值阈** ⇒ 交小美定档"
echo "  2) T2r harness 是单跑还是三连在册未写 ⇒ 交小美（P3 已有两轴口径可参照，不自动套用）"
echo "  3) w3_elevation_node 落哪个壳包/是否新建包：原为口径空白 ⇒ 已由小张 R278 落点裁定填上（留底 21_xiaozhang_landing.txt），本条升为实断言"
XZ_LB="$ROOT/note/进度/SCAN集成/证据/log/raw/r278_3d_release/21_xiaozhang_landing.txt"
if [ -f "$XZ_LB" ] && grep -q 'w3_elevation_node.py' "$XZ_LB"; then
  ok "W8-3 落点裁定留底在位且点名 w3_elevation_node.py（$(grep -c 'w3_elevation_node.py' "$XZ_LB") 处）"
else
  bad "W8-3 落点裁定留底缺失或不含落点名 ⇒ 落点无据，S1 不得自定"
fi
echo "  4) §8 预检 1–4（grid_map_msgs 在装否／116,747 点吞吐／StaticLayer 双实例零变化／unknown 与 0 的解释差异）**未跑**"
echo "  5) 程序前置：小张接口复核＋主理人批准 **未销** ⇒ 本件面完成不等于二期开工，施工侧不越过此门"

echo "=================================================="
echo "W3 CONTRACT VERDICT: PASS=$NP FAIL=$NF ABSENT=$NA（ABSENT 不计通过）"
if [ "$NF" != 0 ]; then echo "⇒ 有 FAIL：对账不过（设计稿漂移或盘上越界），rc=1"; exit 1; fi
echo "⇒ 件面对账过；★R280 起 S1／S2 已落盘（S1 本体＋规范位 yaml），但**本件不声称任何实现正确性或活体结论**：小张接口复核半未销、#46 三态语义未销 ⇒ 判据面与达成判定仍不在本件出口"
exit 0
