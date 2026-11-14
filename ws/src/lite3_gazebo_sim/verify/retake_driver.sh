#!/usr/bin/env bash
# retake_driver.sh —— 批1／批2 活体复取的一键驱动（小明 2026-10-08；判据与处置口径全部引自在册裁决，本件不自设门值）
#
# 为何要一件驱动而不是手敲：本轮三件反复出问题的事，都能被一件程序灭掉——
#   （一）O-4／R19／R28 三次同型：证据落在 /tmp，会话一断或机器一重启即永久不可回溯。
#         ⇒ 本件每段跑完立刻归档两样东西：①本次新产生的 LOGDIR 文本原件（不入 lite3.pcd.bak 二进副本）；
#           ②**本件自己的 $OUT**（driver.log／各段 log／sha_inputs.txt／foxy 起终快照／batch 与 dwa 子目录）。
#           第②项是 2026-10-08 那次硬重启教出来的：H13 批的 stageA 与五连因已入仓而活了下来，
#           而只落在 /tmp/rt_b1b2 的 SCAN 段／门禁段／teb 对照现场**整批丢失**，那段结论至今无原件可指。
#         ⇒ 并加一条收口断言：归档件数必须等于 $OUT 件数，不等即判该段证据未收口（本件 rc=10）。
#   （二）O-5：运动窗 RTF 抖动已成活体批第一失效源（0.240／0.233／0.246 三读）。盲跑一次 stageA 要 7～10 分钟。
#         ⇒ 本件起跑前做负载预检（loadavg 与非本仓大户只读点名），超阈值需 FORCE=1 明示；
#           stageA 按小美 O-5 句执行「最多三次有效重试，全 INVALID 即停手上报算力面」，
#           并把 #30 那型「INVALID 吸收并存 FAIL」的读数原样打出来（rc 分档仍待她落笔，本件不自立分档）。
#   （三）R20三／R27二：加压覆盖与每跑期望 rc 属批次层断言 ⇒ 五连段直接走 m2_batch_runner.sh 并带 EXPECT_RC。
#
# 用法：OUT=/tmp/rt1 bash retake_driver.sh [段...]     段∈{pre,a,b,dwa,scan,gate,negctl,archive,all}（缺省 all）
#   环境变量：FORCE=1 越过负载预检；LOAD_MAX=4 阈值；STAGEA_MAX=3 A0 重试上限；EXPECT_RC 透传给批次驱动。
#   注：LOAD_MAX 是**工序闸门**（不跑=不浪费窗），不是判据门：它不判被测件过不过、不进任何 PASS/FAIL 计数、
#     也不入连击；阈值 4 是施工侧取的操作数，验收若认为该由读数而不是 loadavg 决定，可直接改本行不动判据面。
# 退出码：0 全绿；4 A0 三次皆 INVALID（算力面停手，非判据失败）；5 判据 FAIL（被测件面，停手上报）；10 证据未收口；
#         6 批次覆盖不足（无加压达标跑，R20三）；7 负载预检不过且未 FORCE；8 期望 rc 不符或计数不自洽（R27二/四）；
#         9 前置件缺失（门禁/归档/基线面，含「清单不在位」与「结论行不可指认」两类）。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
# ★R606／批258 核出、批259 落裁★＝原 ROOT 写死容器面路径，宿主面恒不可达；按 L1 问三裁甲改成 git 派生＋缺根即退（与 D7 件第 45 行同形）。
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=/home/ros1-slam
source /opt/ros/noetic/setup.bash
source "$ROOT/ws/devel/setup.bash"
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$ROOT/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
PKG=$ROOT/ws/src/95_simulation/lite3_gazebo_sim
PAY=/tmp/m2_payload
OUT=${OUT:-/tmp/rt_drive}
EV=${EV:-$ROOT/note/进度/v7/S1/SCAN集成/证据/log/raw}
mkdir -p "$OUT"
RD_STDOUT_IS_LOG=0
if [ -e "/proc/self/fd/1" ]; then
  _t=$(readlink /proc/self/fd/1 2>/dev/null)
  [ "$_t" = "$OUT/driver.log" ] && RD_STDOUT_IS_LOG=1
fi
export RD_STDOUT_IS_LOG
STAGEA_MAX=${STAGEA_MAX:-3}
LOAD_MAX=${LOAD_MAX:-4}
VERDICT=0
RD_OWN_LOG=1
log(){ # R50：日志由本件自己拥有并追加到 $OUT/driver.log——旧式把「谁调我怎么重定向」当成证据的一部分，
       #      19:38 那次调用把 stdout 重定向成 driver_stdout.log ⇒ 归档件里没有 VERDICT 行可指认（R48 那条指认闸当场抓住）。
       #      若调用者的 stdout 本来就指向这个文件，则只走 stdout，避免同一行写两遍。
  printf '%s\n' "$*"
  if [ "${RD_STDOUT_IS_LOG:-0}" != "1" ]; then mkdir -p "$OUT"; printf '%s\n' "$*" >> "$OUT/driver.log"; fi
}
now(){ date '+%F %T'; }
# A1（2026-10-10 主理人裁定＝乙＋丙同落）：红线基准**每批现取**，不再沿用写死的旧 boot PID 名单——
# 写死名单一重启就必然读空（O-7 病根），而旧行为只打一句"空洞成立"仍 return 0，等于把没见证当绿。
FOXY_RE='ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer'   # 在册 P0.5 的第二条正则，原样沿用
FOXY_LAST_KNOWN="115711,115792,115795,115797,115799,1331,180237"   # 上一 boot 的名单，**仅留作对照，不再当基准**
FOXY=""
WITNESS_INVALID=0
rebaseline(){ # 乙：每批起工前按 P0.5 正则现取本 boot 名单
  FOXY=$(ps -eo pid=,args= | awk -v re="$FOXY_RE" '$0 ~ re && $0 !~ /awk|grep|retake_driver/ {print $1}' | paste -sd, -)
  : > "$OUT/foxy_baseline.txt" 2>/dev/null
  [ -n "$FOXY" ] && ps -eo pid,user,lstart,args | awk -v re="$FOXY_RE" '$0 ~ re && $0 !~ /awk|grep|retake_driver/' > "$OUT/foxy_baseline.txt"
  log "  foxy 基准现取（A1 乙＝每批重取，不沿用上 boot 名单）：命中 $(grep -c '[0-9]' "$OUT/foxy_baseline.txt" 2>/dev/null || echo 0) 行"
  if [ ! -s "$OUT/foxy_baseline.txt" ]; then
    WITNESS_INVALID=1
    log "  [INVALID] 基准空集（本 boot 无 foxy/monitor 进程在位）⇒ A1 丙：本批红线面记 INVALID，**不得写「零触碰」、不得引用为达成**；环境事实随档留证"
  fi
}
snap(){ if [ -n "$FOXY" ]; then ps -o pid,user,lstart,comm --no-headers -p $FOXY > "$OUT/foxy_$1.txt" 2>/dev/null; else : > "$OUT/foxy_$1.txt"; fi
  log "  foxy[$1] 计数=$(wc -l < "$OUT/foxy_$1.txt")"; [ ! -s "$OUT/foxy_$1.txt" ] && WITNESS_INVALID=1; return 0; }

# ---- 归档：把自 t0 起新产生的 LOGDIR 文本原件拉进证据址（本段可单跑，跑前跑后都能补）----
manifest_build() { # 清单**整体重建**（R48 一）：条目＝归档址内除清单自身与临时件外的全部件，路径一律相对归档址
  #   旧式「每段 >> 追加」且 out/ 段条目缺 out/ 前缀 ⇒ 同一件在一张表里叠多份 sha，且从归档址 sha256sum -c 根本跑不通
  #   （18:23 批实测：清单 88 行对 61 件，-c 非 OK 103 行）。可验性是证据的最低要求；本改动不触任何判据门值。
  local t="$MANIFEST.new"
  ( cd "$ARCHIVE_DIR" && find . -type f ! -name '_sha256_manifest.txt' ! -name '*.new' \
      | sed 's|^\./||' | sort | xargs -d '\n' -r sha256sum ) > "$t" 2>/dev/null
  mv -f "$t" "$MANIFEST"
}

archive_out() { # 把本件的 $OUT 整批拉进证据址，三面核：件数对得上／清单条目＝件数／清单逐条可验（R48 三）
  local n m c k
  mkdir -p "$ARCHIVE_DIR/out"
  cp -rp "$OUT"/. "$ARCHIVE_DIR/out/" 2>/dev/null
  manifest_build
  n=$(find "$OUT" -type f 2>/dev/null | wc -l); m=$(find "$ARCHIVE_DIR/out" -type f 2>/dev/null | wc -l)
  c=$(find "$ARCHIVE_DIR" -type f ! -name '_sha256_manifest.txt' 2>/dev/null | wc -l)
  k=$( ( cd "$ARCHIVE_DIR" && sha256sum -c "$MANIFEST" 2>/dev/null ) | grep -c ': OK$' )
  if [ "$n" = "$m" ] && [ "$n" -ge 1 ] && [ "$c" = "$k" ] && [ "$k" -ge 1 ]; then
    log "  [archive] OUT 收口：$OUT 件数=$n ＝ 归档件数=$m ｜清单可验=$k ＝ 全件数=$c（校验 CWD＝归档址）"
  else
    OUT_UNCLOSED=1
    log "  [archive] OUT 未收口：$OUT 件数=$n，out 归档=$m，清单可验=$k，全件数=$c（四数须 OUT＝out 且 可验＝全件≥1）⇒ 该段证据不完整，末判 rc=10"
  fi
}

archive_since() { # $1=epoch 秒
  local t0="$1" new d tgt man
  new=$(find "$PAY" -maxdepth 1 -type d -name 'logs-*' -newermt "@$t0" 2>/dev/null | sort)
  [ -z "$new" ] && { log "  archive：本次无新 LOGDIR（$t0 之后）"; return 0; }
  for d in $new; do
    tgt="$ARCHIVE_DIR/$(basename "$d")"; mkdir -p "$tgt"
    find "$d" -maxdepth 1 -type f ! -name 'lite3.pcd.bak' -exec cp -p {} "$tgt/" \; 2>/dev/null
    manifest_build   # 每收一个 LOGDIR 即整体重建（R48 一）；路径自带 logs-*/ 前缀，跨目录同名件仍可判读
    log "  archived $(basename "$d")：$(ls "$tgt" | wc -l) 件，收口=$( [ -f "$d/stageB.txt" ] || [ -f "$d/stageA.txt" ] && echo 是 || echo 否 )"
  done
}

sec_pre() {
  log "===== 0) 负载与隔离域预检 $(now) ====="
  local la r1 r2 r3
  la=$(awk '{print $1}' /proc/loadavg)
  log "  loadavg(1min)=$la  LOAD_MAX=$LOAD_MAX  STAGEA_MAX=$STAGEA_MAX  FORCE=${FORCE:-未设}"
  r1=$(ss -lnt 2>/dev/null | grep -c ':11313'); r2=$(ss -lnt 2>/dev/null | grep -c ':11345'); r3=$(ss -lnt 2>/dev/null | grep -c ':11312')
  log "  在册端口监听：11313=$r1 11345=$r2 生产11312=$r3（三者必须 0／0／0，否则并发未清）"
  log "  非本仓大户点名（只读，不动他人进程）："
  ps -eo pid,user,pcpu,etime,comm --sort=-pcpu 2>/dev/null | awk 'NR>1 && $3+0>20 {printf "    pid=%s user=%s %s%% 起了=%s comm=%s\n",$1,$2,$3,$4,$5}' | head -6
  awk 'BEGIN{exit !('"$la"' > '$LOAD_MAX')}' && [ "${FORCE:-}" != "1" ] && { log "  [停手] 负载超阈值且未 FORCE=1 ⇒ 跑了也是作废跑（O-5 在册）"; return 7; }
  [ $((r1 + r2 + r3)) -eq 0 ] || { log "  [停手] 隔离域或生产口非零 ⇒ 有并发在跑，本驱动不叠加（坑24／#24 同源）"; return 9; }
  rebaseline
  snap start
  log "  指纹（载体＋被测件＋配套件）："; sha256sum "$PKG/verify/m2_smoke.sh" "$PKG/verify/m2_exitclean_lib.sh" \
    "$PKG/urdf/lite3_planar.urdf.xacro" "$PKG/src/lite3_planar_drive.cpp" "$ROOT/ws/devel/lib/liblite3_planar_drive.so" \
    "$PKG/verify/v_pose_source.sh" "$ROOT/ws/src/40_navigation/lite3_scan/launch/70_scan.launch" | tee "$OUT/sha_inputs.txt"
  return 0
}

sec_a() {
  log "===== 1) stageA：最多 $STAGEA_MAX 次有效重试（A0 门 0.25 属 M2R2 裁决，本件不改） $(now) ====="
  local i rc t0
  for i in $(seq 1 "$STAGEA_MAX"); do
    t0=$(date +%s)
    TF_SOURCE="" bash "$PKG/verify/m2_smoke.sh" stageA > "$OUT/stageA_$i.log" 2>&1; rc=$?
    log "  第 $i 次 rc=$rc | $(grep -oE 'drift=[0-9.]+ gt_drift=[0-9.]+ rate_pts=[0-9.]+ rate_imu=[0-9.]+ fly_ratio=[0-9.]+ rtf_motion=[0-9.]+' "$OUT/stageA_$i.log" | head -1)"
    log "    判定面 $(grep -cE '^\[PASS\]' "$OUT/stageA_$i.log")P/$(grep -cE '^\[FAIL\]' "$OUT/stageA_$i.log")F | $(grep -E '^SUMMARY marks' "$OUT/stageA_$i.log" | tail -1)"
    grep -E '^\[INVALID\]|^M2 SMOKE|^\[posesrc\].*VERDICT' "$OUT/stageA_$i.log" | sed 's/^/      /'
    if [ "$rc" = "0" ]; then log "  ⇒ 取得有效 stageA，M2-1／M2-4 的复取成立"; archive_since "$t0"; return 0; fi
    if [ "$rc" = "3" ]; then log "  ⇒ A0 作废（不计 FAIL、不入连击），重试"; archive_since "$t0"; continue; fi
    log "  [停手上报] rc=$rc 属判据面或跑崩，不是环境作废 ⇒ 如实入档，不重跑掩盖（并存 FAIL 明细见上）"; archive_since "$t0"; return 5
  done
  log "  [停手上报] $STAGEA_MAX 次全 INVALID ⇒ 算力面问题（O-5 末句），本批不出结论；三次读数已全部归档"; return 4
}

sec_b() {
  log "===== 2) stageB 五连（含 ≥1 次加压档，批次断言交 m2_batch_runner） $(now) ====="
  local t0 rc
  t0=$(date +%s)
  EXPECT_RC="${EXPECT_RC:-0 0 0 0 0}" BATCH_OUT="$OUT/batch" bash "$PKG/verify/m2_batch_runner.sh" r1: r2: r3:62 r4: r5: > "$OUT/batch.log" 2>&1; rc=$?
  log "  batch rc=$rc"; grep -E '^\s+\[|VERDICT|rc=' "$OUT/batch.log" | tail -22 | sed 's/^/    /'
  archive_since "$t0"; return "$rc"
}

sec_dwa() {
  log "===== 3) DWA 三态复取（0 判据过／5 判据不过／6 INVALID，不得把 rtf 折进 verdict） $(now) ====="
  local rc
  DWA_OUT="$OUT/dwa" bash "$ROOT/ws/src/40_navigation/lite3_navigation/verify/v_dwa_recheck.sh" 3 > "$OUT/dwa.log" 2>&1; rc=$?
  log "  dwa rc=$rc"; grep -E 'DWA RECHECK|汇总|INCOMPLETE|rtf=' "$OUT/dwa.log" | tail -8 | sed 's/^/    /'
  # 件间 rc 映射（DWR 自有层 0/4/5/6/7 → 本件层，她裁定 1 丙条要求逐件写表）：
  #   DWR 0 → 本件 0；DWR 5 → 本件 5（判据不过，停手）；DWR 7 → 本件 5（并存停手，等同需人工）；
  #   DWR 6 或 4 → 本件 4（作废/不完整集，算力或前置类，不判 DWA 不可用也不称复现）；其余未知 → 9 前置缺失
  case "$rc" in
    0) return 0;;
    5|7) log "  （DWR rc=$rc ⇒ 判据面或并存停手，本批不作通过结论）"; return 5;;
    6|4) log "  （DWR rc=$rc ⇒ 作废或不完整集：不判 DWA 不可用，也不称复现）"; return 4;;
    *) log "  [停手] DWR 返回未知 rc=$rc ⇒ 件间映射不含此值，属前置缺失，不默认绿"; return 9;;
  esac
}

sec_scan() {
  log "===== 4) SCAN P1／P2（批2 接线的活体面） $(now) ====="
  local rc1 rc2
  bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_p1.sh" > "$OUT/p1.log" 2>&1; rc1=$?
  bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_p2.sh" > "$OUT/p2.log" 2>&1; rc2=$?
  log "  p1 rc=$rc1 p2 rc=$rc2"
  # 回显口径修正：P1/P2 的结论行不是行首 [PASS] 而是「P1 判据① 结论 = PASS」「判定: PASS|全部 7 项通过」，
  #   旧 grep 匹配不到 ⇒ 驱动的关键读数回显长期空着（不是件在假绿，是我这层看不见；两者都必须纠）。
  grep -E 'P1 判据① 结论|P2 判据② 结论|判定:|endpoint_err|react_s|残留进程|监听' "$OUT/p1.log" | tail -5 | sed 's/^/    p1| /'
  grep -E 'P1 判据① 结论|P2 判据② 结论|判定:|endpoint_err|react_s' "$OUT/p2.log" | tail -5 | sed 's/^/    p2| /'
  grep -qE 'P1 判据① 结论 = PASS' "$OUT/p1.log" || { log "  [停手] p1.log 里没有可指认的结论行 ⇒ 不采信 rc=$rc1（R44 新查得：rc 必须与结论行成对可核）"; return 9; }
  grep -qE 'P2 判据② 结论 = PASS' "$OUT/p2.log" || { log "  [停手] p2.log 里没有可指认的结论行 ⇒ 不采信 rc=$rc2"; return 9; }
  [ "$rc1" = "0" ] && [ "$rc2" = "0" ] && return 0
  return 5
}

sec_gate() {
  log "===== 5) 共同门禁＋基线双条件＋样张＋退净 $(now) ====="
  bash "$ROOT/scripts/verify/v3_isolation.sh" > "$OUT/gate.log" 2>&1; log "  门禁 EXIT=$?"; tail -2 "$OUT/gate.log" | sed 's/^/    /'
_BL=$ROOT/note/进度/v7/S1/SCAN集成/证据/scan_src_baseline_49.sha256   # 上一版误写 $EV
  if [ ! -f "$_BL" ]; then                                        #   却被打印成「非OK行数=1」＝一条假红灯。
    log "  [停手] 基线清单不在位（$_BL）⇒ 属前置缺失 rc=9，本件拒绝打印任何基线读数"
    return 9
  fi
  _btot=$(grep -vc '^#' "$_BL")
  _bok=$( cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$_BL" > "$OUT/base.txt" 2>&1; grep -c ': OK$' "$OUT/base.txt" )
  _bcnt=$(cd "$ROOT/ws/src/30_3rd" && find . -type f | wc -l)
  # 正向计数比对（清单条目数＝OK 数＝目录文件数），不再用「非 OK 行数」这种把「跑不动」与「跑出不符」混为一谈的口径
  if [ "$_bok" = "$_btot" ] && [ "$_bcnt" = "$_btot" ]; then
    log "  30_3rd 基线双条件 OK：清单条目=$_btot 逐条 OK=$_bok 目录文件=$_bcnt"
  else
    log "  [停手上报] 30_3rd 基线双条件不过：清单条目=$_btot 逐条 OK=$_bok 目录文件=$_bcnt（三者应相等）⇒ 对照组完整性红线，本批不作结论"
    return 9
  fi
  sha256sum "$ROOT/data/maps/lite3.pcd" > "$OUT/sample_sha.txt"
  rosnode kill -a >/dev/null 2>&1; sleep 3
  pkill -f gzserver >/dev/null 2>&1; pkill -f run_mapping_online >/dev/null 2>&1
  pkill -f "roslaunch.*mode_sim_" >/dev/null 2>&1; pkill -f "roslaunch.*m2_" >/dev/null 2>&1
  pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; pkill -f move_base_flex >/dev/null 2>&1; sleep 5
  log "  退净：残留=[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex|run_mapping_online|rosout' | tr '\n' ' ')] 端口11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11345=$(ss -lnt 2>/dev/null | grep -c ':11345')"
  log "  E3 双父签名（各跑 stageB 的 base_link 父帧集合，须只有 odom）："
  grep -hoE "SMOKE E3 .*base_link_parents=\[[^]]*\]" "$OUT"/batch/r*.log 2>/dev/null | sort | uniq -c | sed 's/^/    /'
  snap end
  # 三种形态必须分开说，否则要么假绿要么假报警（本轮 dry-run 实测到假报警一次）：
  #   ①缺基准（只跑了 gate 段没跑 pre 段）⇒ 属前置缺失，绝不得指控他人进程被扰动；
  #   ②两侧皆空（名单里的 PID 全不在位，如重启后换号）⇒ 断言空洞成立，红线见证失效（O-7），不得引用为达成；
  #   ③两侧都有行且整行（pid＋user＋lstart＋comm）不等 ⇒ 才是真的扰动。
  if [ ! -f "$OUT/foxy_start.txt" ] || [ ! -f "$OUT/foxy_end.txt" ]; then
    log "  [停手] foxy 起终快照不全（本调用没跑 pre 段 ⇒ 无基准可比）＝前置缺失 rc=9；这不是他人进程被扰动，不得那样指控"
    return 9
  fi
  _ns=$(wc -l < "$OUT/foxy_start.txt"); _ne=$(wc -l < "$OUT/foxy_end.txt")
  if [ "$_ns" = "0" ] && [ "$_ne" = "0" ]; then
    WITNESS_INVALID=1
    log "  [INVALID] foxy 快照两侧皆空（A1 丙）⇒ 红线见证失效，本批**不得写「零触碰」、不得引用为红线达成**（O-7）；空集原件随 OUT 归档可看；末判按 rc=11 出"
  elif diff "$OUT/foxy_start.txt" "$OUT/foxy_end.txt" > /dev/null 2>&1; then
    log "  foxy 起终成对：整行（含 lstart）逐位一致（$_ns 行）"
  else
    log "  [停手上报] foxy 起终整行不一致 ⇒ 他人进程被扰动，本批不作结论"
    diff "$OUT/foxy_start.txt" "$OUT/foxy_end.txt" | head -4 | sed "s/^/    /"
  fi
  return 0
}

sec_negctl() { # R41（二）活体现场面：批内含一位非 0 期望，且阴性跑必须**按预期原因** FAIL（小明 2026-10-08）
  log "===== 7) 活体阴性对照批（tf_source:=static 走 mode_sim_teb 的 [M2R3] 复验口子；本件不设门值） $(now) ====="
  local t0 nspc=(r1:0 r2:62 r3:0) nexp="${NEGCTL_EXPECT_RC:-0 0 1}" nstatic="${NEGCTL_STATIC_RUN:-3}" rc nsub bad r3log key p1 p2
  t0=$(date +%s)
  echo 0 > "$OUT/negctl_cnt"
  env NEGCTL_CNT="$OUT/negctl_cnt" NEGCTL_STATIC_RUN="$nstatic" BATCH_OUT="$OUT/negctl" SLEEP_BETWEEN=4 EXPECT_RC="$nexp" \
      SM_CMD=bash SM_ARGS="$PKG/verify/m2_negctl_tf_runner.sh" \
      bash "$PKG/verify/m2_batch_runner.sh" "${nspc[@]}" > "$OUT/negctl.log" 2>&1
  rc=$?
  # 闸一（R49 那型的活体 Equivalent）：被试指认——每跑必须有 [subject] 行，且零次回落到现役 SM；不过＝本段前置不成立，不作结论
  nsub=$(grep -c '\[subject\] 本跑被试＝' "$OUT/negctl.log"); bad=$(grep -c '被试＝bash verify/m2_smoke.sh' "$OUT/negctl.log")
  log "  [subject] 被试行=$nsub（应＝跑数 ${#nspc[@]}）回落到现役 SM=$bad（应 0）"
  if [ "$nsub" != "${#nspc[@]}" ] || [ "$bad" != "0" ]; then
    log "  ⇒ 被试指认不过 ⇒ 本段不作结论（跑的可能不是阴性对照，rc=9 前置不成立）"; archive_since "$t0"; return 9
  fi
  # 闸二：阴性跑必须按预期键 FAIL（G5c2 全局系 TF 时基新鲜度）；换了别的键就不是 M2R3 那口子该给的读数
  r3log="$OUT/negctl/$(printf '%s' "${nspc[$((nstatic-1))]}" | cut -d: -f1).log"
  key=$(grep -c 'B: 门控未过（键=tf_fresh_mapodom_G5c2' "$r3log" 2>/dev/null); [ -z "$key" ] && key=0
  log "  阴性跑原件：$r3log 内 G5c2 门控未过行=$key（应 ≥1）"
  if [ "$key" = "0" ]; then
    log "  ⇒ 阴性跑未按预期原因 FAIL ⇒ 判该段不可引用（口子失效或另有扰动，rc=5）"; archive_since "$t0"; return 5
  fi
  # 闸三：另两跑必须真 PASS（阴性对照混进负载型失败＝整批读数不可引用为分辨力证明）
  p1=$(grep -c 'M2 SMOKE: PASS' "$OUT/negctl/r1.log" 2>/dev/null); p2=$(grep -c 'M2 SMOKE: PASS' "$OUT/negctl/r2.log" 2>/dev/null)
  log "  阳性对照面：r1 PASS 行=$p1 r2（加压档）PASS 行=$p2（均应 1）"
  if [ "$p1" != "1" ] || [ "$p2" != "1" ]; then
    log "  ⇒ 同批阳性跑未达标 ⇒ 阴性对照失去意义（可能负载型），本段不作结论 rc=5"; archive_since "$t0"; return 5
  fi
  log "  批次层：$(grep -E 'BATCH VERDICT|逐跑：|\[expect\]' "$OUT/negctl.log" | tr '\n' ' | ')"
  if [ "$rc" != "${NEGCTL_EXPECT_BR_RC:-0}" ]; then
    log "  ⇒ BR 批级 rc=$rc 与预期 ${NEGCTL_EXPECT_BR_RC:-0} 不符（期望位没被逐位消化）rc=8"; archive_since "$t0"; return 8
  fi
  log "  ⇒ R41（二）现场面成立：批内含一位非 0 期望且被逐位消化，且阴性跑按 G5c2 键 FAIL"
  archive_since "$t0"
  return 0
}

SECS=("${@:-all}")
[[ " ${SECS[*]} " == *" all "* ]] && SECS=(pre a b dwa scan gate)
ARCHIVE_DIR="$EV/retake_$(date +%Y%m%d_%H%M%S)"; MANIFEST="$ARCHIVE_DIR/_sha256_manifest.txt"
OUT_UNCLOSED=0
mkdir -p "$ARCHIVE_DIR"; log "归档址：$ARCHIVE_DIR"; log "  清单不列自身；校验口径：cd 归档址 && sha256sum -c _sha256_manifest.txt（旧句写仓根 CWD 不实，R48 一）"
for s in "${SECS[@]}"; do
  case "$s" in
    pre) sec_pre; rc=$?;; a) sec_a; rc=$?;; b) sec_b; rc=$?;;
    dwa) sec_dwa; rc=$?;; scan) sec_scan; rc=$?;; gate) sec_gate; rc=$?;;
    negctl) sec_negctl; rc=$?;;
    archive) archive_since "$(date -d '12 hours ago' +%s)"; rc=$?;;
    *) log "未知段 $s"; rc=9;;
  esac
  archive_out
  [ "$rc" = "0" ] || { VERDICT=$rc; log "-- 段 $s 止于 rc=$rc ⇒ 后续段不再跑（失败即停手，不往后凑绿）"; break; }
done
if [ "$WITNESS_INVALID" = "1" ] && [ "$VERDICT" = "0" ]; then VERDICT=11; fi   # A1 丙：空集强制 INVALID，不当绿
if [ "$OUT_UNCLOSED" = "1" ] && [ "$VERDICT" = "0" ]; then VERDICT=10; fi
log "===== RETAKE VERDICT: rc=$VERDICT $(now) ====="
log "  0=全绿 4=A0 三次皆 INVALID（算力面）5=判据 FAIL 6=批次覆盖不足 7=负载预检不过 8=期望/计数面 9=前置缺失 10=证据未收口 11=红线见证空集 INVALID（A1 丙，2026-10-10 主理人裁定新增档）（OUT 归档数不符）"
log "  rc 空间属本件自有层（与 DWR／BR／SM 各层不互认），引用时必须连件名＋代际（她裁定 1 丙条／D5）"
archive_out   # R48 二：VERDICT 行打完再收一次——旧序里归档副本永远没有本批自己的 VERDICT 行（18:23 批实测归档件 66 行／活体件 68 行）
_v=$(grep -c 'RETAKE VERDICT' "$ARCHIVE_DIR/out/driver.log" 2>/dev/null); [ -z "$_v" ] && _v=0
log "  [archive] 终局指认：归档件 out/driver.log 内含 RETAKE VERDICT 行=$_v（应=1）；本行及其后的收口行按定义进不了自己的副本，属可指认的固定残差"
if [ "$_v" != "1" ]; then
  OUT_UNCLOSED=1; [ "$VERDICT" = "0" ] && VERDICT=10
  log "  ⇒ 终局指认不过 ⇒ 本批证据不可自证收口，末判改按 rc=$VERDICT"
fi
log "  归档件数=$(find "$ARCHIVE_DIR" -type f ! -name '_sha256_manifest.txt' | wc -l)（不含清单自身） 清单=$MANIFEST"
log "===== RETAKE VERDICT(终): rc=$VERDICT $(now) ====="
exit "$VERDICT"
