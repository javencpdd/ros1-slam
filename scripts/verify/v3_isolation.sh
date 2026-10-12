#!/usr/bin/env bash
# ── --selftest（R235 新入，依据＝主理人裁「覆盖性统一＝只补 ISO＋GTC」）────────────────────
# 做法照 GTC 的在册先例 --negctl：**拿本体的临时副本改路径来注入，在册本体一行不动**（十项判据＝判据面，施工侧不自改）。
# 十例每例都必须能单独失败；**且副本基线（未注入）必须先证明是绿的**——第一版我没做这一步：
#   ①`expect` 把「放／拦」当数字比 ⇒ 七例全被当成"必须拦"⇒ 假的全 PASS；②插进副本的是字面 `$W`（副本里没有这个变量）
#   ⇒ 第 [4] 项永远 BAD ⇒ 干净副本也红 ⇒ 所有断言失去意义。两处都是**守卫自身坏掉而输出好看**，本版加"基线不绿即 exit 9"堵死。
#   ③第三例同族（R240 加 [10] 时又踩）：[10] 读的是**跑自测的这台机器**的环境 ⇒ 基线红不红取决于账户与真实路径，
#   与注入无关 ⇒ 沙箱必须自带四变量／rospack 桩／import 模块，否则 T0 假红、十例全不作数（下方 SP_*）。
if [ "${1:-}" = "--selftest" ]; then
  SELF=$(readlink -f "${BASH_SOURCE[0]}")
  W=$(mktemp -d /tmp/iso_selftest.XXXXXX)
  mkdir -p "$W/ws/src/95_simulation/lite3_gazebo_sim/launch" "$W/ws/src/95_simulation/lite3_nav/launch" \
           "$W/ws/src/70_bringup/lite3_bringup/launch" "$W/ws/src/40_navigation" \
           "$W/pkgs/iso10_fake" "$W/ws/devel/lib" "$W/ws/src/iso10_pkg"
  echo clean > "$W/ws/src/95_simulation/lite3_gazebo_sim/launch/ok.launch"
  : > "$W/pkgs/iso10_fake/__init__.py"
  X=$(mktemp -d /tmp/iso_outside.XXXXXX); mkdir -p "$X/pkgs/iso10_bad"; : > "$X/pkgs/iso10_bad/__init__.py"
  : > "$W/.bashrc"
  COPY="$W/iso_copy.sh"
  sed -e "s|/home/ros1-slam|$W|g" -e "s|rospack find \"\$p\"|echo \"$W\"|" -e "s|rospack list|echo iso10_pkg $W/ws/src/iso10_pkg|g" "$SELF" > "$COPY"
  bash -n "$COPY" || { echo "前置不成立：副本语法不过"; rm -rf "$W" "$X"; exit 9; }
  grep -q "echo \"$W\" 2>/dev/null" "$COPY" || { echo "前置不成立：副本里 [4] 的替换没生效（仍走 rospack）"; rm -rf "$W" "$X"; exit 9; }
  grep -q "echo iso10_pkg $W/ws/src/iso10_pkg 2>/dev/null" "$COPY" || { echo "前置不成立：副本里 [10] 的 rospack list 替换没生效"; rm -rf "$W" "$X"; exit 9; }
  [ -f /opt/ros/noetic/setup.bash ] && { set +u; . /opt/ros/noetic/setup.bash; . /home/ros1-slam/ws/devel/setup.bash 2>/dev/null; set -u; }
  # R240 新增 [10] 后自测面必须**自带一套沙箱环境**：副本若继续继承真实环境，[10] 会把"跑自测的这台机器"本身
  #   （运行账户 ysc、numpy 落在 /home/ysc/.local）判成越界 ⇒ T0 基线假红 ⇒ 下面全部例不作数。沙箱段全在白名单内、
  #   rospack 走桩、import 走沙箱模块 ⇒ 基线与真实环境解耦；注入例（T4/T5/T6）再把段移出白名单，验"拦得到"。
  SP_RPP="$W/ws/src:/opt/ros/noetic/share"
  SP_PYP="$W/pkgs:/opt/ros/noetic/lib/python3/dist-packages"
  SP_LD="$W/ws/devel/lib:/opt/ros/noetic/lib"
  SP_CM="$W/ws/devel:/opt/ros/noetic"
  NPASS=0; NTOT=0
  runit() { ( export ROS_PACKAGE_PATH="${1:-$SP_RPP}" PYTHONPATH="${2:-$SP_PYP}" LD_LIBRARY_PATH="$SP_LD" \
                    CMAKE_PREFIX_PATH="$SP_CM" ISO10_MODULES="${3:-iso10_fake}" ISO_ALLOW="$W:/opt/ros/noetic:/usr"
              bash "$COPY" >/tmp/iso_st.out 2>&1; echo $? ); }
  expect() {  # $1=例名 $2=期望(放|拦) $3=说明 $4=实得rc $5=（选填）拦例必须由该项自己报出：输出里命中的专属串（R44 口径＝不可指认的 rc 不采信）
    NTOT=$((NTOT+1)); HIT=""
    if [ "$2" = "放" ]; then GOOD=$([ "$4" = "0" ] && echo 1 || echo 0)
    else
      GOOD=$([ "$4" != "0" ] && echo 1 || echo 0)
      if [ -n "${5:-}" ]; then
        if grep -qF "$5" /tmp/iso_st.out; then HIT="｜指认＝命中「$5」"; else GOOD=0; HIT="｜指认＝**未命中**（红非该项报出，本例不作数）"; fi
      fi
    fi
    [ "$GOOD" = "1" ] && NPASS=$((NPASS+1))
    echo "[$([ \"$GOOD\" = \"1\" ] && echo PASS || echo FAIL)] $1 $3（期望 $2／rc$([ \"$2\" = \"放\" ] && echo =0 || echo ≠0)，实得 rc=$4）$HIT"
  }
  R0=$(runit)
  if [ "$R0" != "0" ]; then
    echo "[FAIL] T0 未变异副本基线就不绿（rc=$R0）⇒ 沙箱前置不成立，**下面所有例都不作数**"; sed -n '1,12p' /tmp/iso_st.out | sed 's/^/    ／'; rm -rf "$W" "$X"; exit 9
  fi
  expect T0 放 "未变异副本必须全绿（红只能来自注入，不能来自沙箱本身）" "$R0"
  echo '/home/ysc/lite_cog 硬编码' > "$W/ws/src/95_simulation/lite3_gazebo_sim/launch/bad.launch"
  expect T1a 拦 "壳包注入 /home/ysc 硬编码 ⇒ [2] 与 [8] 必须红" "$(runit)"
  rm -f "$W/ws/src/95_simulation/lite3_gazebo_sim/launch/bad.launch"
  expect T1b 放 "撤掉注入必须回绿（否则 T1a 的红可能来自别的因素）" "$(runit)"
  printf '<launch><include file="$(find lite3_nav)" /></launch>\n' > "$W/ws/src/95_simulation/lite3_gazebo_sim/launch/i.launch"
  expect T2 拦 "平级 lite3_* 互 include ⇒ [5] 必须红" "$(runit)"
  rm -f "$W/ws/src/95_simulation/lite3_gazebo_sim/launch/i.launch"
  printf '<launch><include file="$(find lite3_nav)" /></launch>\n' > "$W/ws/src/70_bringup/lite3_bringup/launch/t.launch"
  expect T2b 放 "TOP 编排层（lite3_bringup）include 必须仍被豁免（P7 裁决在位；误删豁免即红）" "$(runit)"
  rm -f "$W/ws/src/70_bringup/lite3_bringup/launch/t.launch"
  echo 'int main(){return 0;}' > "$W/ws/src/40_navigation/alg.cpp"
  expect T3 拦 "40_navigation 壳包混进 .cpp ⇒ [9] 必须红" "$(runit)"
  rm -f "$W/ws/src/40_navigation/alg.cpp"
  expect T4 拦 "ROS_PACKAGE_PATH 混入 /home/ysc ⇒ [1] 必须红" "$(runit "$SP_RPP:/home/ysc/fake")"
  expect T5 拦 "PYTHONPATH 混入白名单外段 ⇒ [10] 必须红" "$(runit "$SP_RPP" "$W/pkgs:/home/ysc/evil/dist-packages")" "越界=1"
  expect T5b 放 "撤掉该段必须回绿（证明 T5 的红来自注入而非沙箱本身）" "$(runit)"
  expect T6 拦 "import 来源落在白名单外（沙箱外模块）⇒ [10] 的 import 支路必须红" "$(runit "$SP_RPP" "$X/pkgs" "iso10_bad")" "⇒ 越界｜运行账户="
  echo "ISOLATION SELFTEST VERDICT: PASS=$NPASS/$NTOT（十例：T0 基线／T1a-b 注入与撤除／T2 互 include／T2b 豁免／T3 算法源码／T4 混装／T5-b [10] 变量段拦与撤／T6 [10] import 来源拦）"
  rm -rf "$W" "$X"
  [ "$NPASS" = "$NTOT" ] && exit 0 || exit 9
fi
FAIL=0
TMP=$(mktemp)
say() { echo "$1"; }
say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc" && { echo "  违规"; FAIL=1; }
say "[2] 自研壳中是否含 /home/ysc"
grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null && FAIL=1
say "[3] .bashrc 是否 source 了其他 devel"
grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null && { echo "  违规"; FAIL=1; }
say "[4] rospack 解析路径是否都在 /home/ros1-slam（teb 例外：系统包）"
for p in faster_lio hdl_localization lite3_bringup lite3_safety; do
  rospack find "$p" 2>/dev/null | grep -q "/home/ros1-slam" || { echo "  BAD: $p"; FAIL=1; }
done
say "[5] ★ 平级模块间是否互相 include launch（20 号 K3）—— P7 修订（主理人裁决，04 台账）：豁免 lite3_bringup（TOP 编排层 = include 合法主体，mode 级组合 launch include 各平级模块属 TOP 职责）；K3 语义 = 平级 lite3_* 模块禁互 include（TOP 下行不豁免）—— v3 修复子 shell 丢 FAIL bug 口径不变 —— SCAN 精度修订（主理人裁决 2026-10-08，04 台账，留底 raw/v3_isolation.sh.before_scanC5.bak）：旧实现 grep -o 'find lite3_*' 会把「rosparam file=\$(find lite3_X)/config/... 加载他包配置」也计为命中，与本段标题及 K3 语义（禁互 include **launch**）不符，实测致 m1_mapping/m2_mapping 两条纯 config 加载假阳性；新实现只匹配 <include file=\"\$(find lite3_X)\" 形态，真 include 命中数不变、反向探针必抓（见 04 登记）。已知局限：跨行书写的 <include\\n file=...> 不在扫描面内（沙箱现存 launch 均单行写法）"
: > "$TMP"
for L in /home/ros1-slam/ws/src/*/lite3_*/launch/*.launch; do
  [ -e "$L" ] || continue
  SELF=$(basename "$(dirname "$(dirname "$L")")")
  [ "$SELF" = "lite3_bringup" ] && continue  # P7 修订：TOP 编排层豁免（语义变化见上方 say 行与 04 台账）
  grep -oE '<include[^>]*file="\$\(find (lite3_[a-z_]*)\)' "$L" 2>/dev/null | sed 's/.*find //;s/)//' | sort -u | \
  while read -r n; do [ "$n" != "$SELF" ] && echo "$L -> $n" >> "$TMP"; done
done
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug；v4 语义注记：lite3_supervisor 亦在扫描范围（SUP 红线：绝不写 /cmd_vel，N5/C13）"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" > "$TMP" || true
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }
say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null && FAIL=1
say "[9] ★★ 40_navigation 壳包禁算法源码（v4 新增，N2/§2.6 硬规则 b：policy 层零 .cpp/.cc/.h）"
HITS=$(find /home/ros1-slam/ws/src/40_navigation \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" \) 2>/dev/null)
[ -n "$HITS" ] && { echo "  命中算法源码文件："; echo "$HITS"; FAIL=1; }
say "[10] ★★ 全变量与库解析来源核对（v5.3 §0.8 收编 A34③乙 ⇒ 待办 #121；四变量**全表逐段**＋rospack 全量＋python import 来源）"
ISO_ROOT="${ISO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ISO_ALLOW="${ISO_ALLOW:-$ISO_ROOT:/opt/ros/noetic:/usr:/opt/3rdparty:/etc}"
# 白名单＝**前缀**语义：段＝表内项本身或其子目录 ⇒ 算在面内。R240 首版写成"整段全等"，把 /home/ros1-slam/ws/src
#   这类合法子路径判成越界（实测四变量四行全红＋rospack 37 条假阳性）；改前缀后同一份环境实测越界=0、348 包全在面内。
IFS=':' read -ra ISO_ALLOW_ARR <<< "$ISO_ALLOW"
in_allow() {
  local s="${1%/}" p
  case "$s" in /*) : ;; *) return 1 ;; esac
  for p in "${ISO_ALLOW_ARR[@]}"; do p="${p%/}"; [ -n "$p" ] || continue
    case "$s" in "$p"|"$p"/*) return 0 ;; esac
  done
  return 1
}
V10_BAD=0
for v in ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH; do
  eval "vals=\${$v:-}"; n=0; bad=0; first=""
  IFS=':' read -ra segs <<< "$vals"
  for seg in "${segs[@]}"; do
    [ -z "$seg" ] && continue; n=$((n+1))
    if ! in_allow "$seg"; then bad=$((bad+1)); [ -z "$first" ] && first="$seg"; fi
  done
  printf '  [%s] 段数=%s 越界=%s%s\n' "$v" "$n" "$bad" "$( [ "$bad" -gt 0 ] && printf '｜首条越界=%s' "$first" )"
  [ "$bad" -gt 0 ] && V10_BAD=1
done
RL=$(rospack list 2>/dev/null | wc -l)
RB=$(rospack list 2>/dev/null | awk '{print $2}' | while read -r d; do in_allow "$d" || echo x; done | wc -l)
printf '  [rospack list] 包数=%s 解析到白名单外=%s\n' "$RL" "$RB"
[ "$RB" != "0" ] && V10_BAD=1
for m in ${ISO10_MODULES:-rospy numpy}; do
  MP=$(python3 -c "import $m; print($m.__file__)" 2>/dev/null | tail -1)
  if [ -z "${MP:-}" ]; then printf '  [import %s] 取不到来源 ⇒ 判越界｜运行账户=%s HOME=%s\n' "$m" "$(id -un)" "$HOME"; V10_BAD=1; continue; fi
  if in_allow "$(dirname "$MP")"; then printf '  [import %s] 来源=%s（在允许面内）｜运行账户=%s HOME=%s\n' "$m" "$MP" "$(id -un)" "$HOME"; else printf '  [import %s] 来源=%s ⇒ 越界｜运行账户=%s HOME=%s\n' "$m" "$MP" "$(id -un)" "$HOME"; V10_BAD=1; fi
done
[ "$V10_BAD" = "1" ] && FAIL=1
# 自证行（小美 M2R6 路由项 R25 二）：旧版只打结论、无时间与退出码 ⇒ 归档件里「谁在跑、跑出几」
#   要反推（与「跑的是哪份 harness」同族的自证缺口）。日期＋扫描面＋rc 三者入正文，跑完即自解释。
SCOPE_N=$(find /home/ros1-slam/ws/src -type f 2>/dev/null | wc -l)
rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
echo "# self-attest: at=$(date '+%Y-%m-%d %H:%M:%S') script=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}') ws_src_files=$SCOPE_N user=$(id -un) home=$HOME rc=$FAIL"
exit $FAIL
