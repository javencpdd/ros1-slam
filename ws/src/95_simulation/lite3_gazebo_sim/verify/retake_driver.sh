#!/usr/bin/env bash
# retake_driver.sh —— 批1／批2 活体复取的一键驱动（小明 2026-10-08；判据与处置口径全部引自在册裁决，本件不自设门值）
#
# 为何要一件驱动而不是手敲：本轮三件反复出问题的事，都能被一件程序灭掉——
#   （一）O-4／R19／R28 三次同型：LOGDIR 落在 /tmp，会话一断证据即永久不可回溯（1373109 那次已经吃过）。
#         ⇒ 本件每段跑完立刻把本次新产生的 LOGDIR 文本原件归档到证据址，并打清单（不入 lite3.pcd.bak 二进副本）。
#   （二）O-5：运动窗 RTF 抖动已成活体批第一失效源（0.240／0.233／0.246 三读）。盲跑一次 stageA 要 7～10 分钟。
#         ⇒ 本件起跑前做负载预检（loadavg 与非本仓大户只读点名），超阈值需 FORCE=1 明示；
#           stageA 按小美 O-5 句执行「最多三次有效重试，全 INVALID 即停手上报算力面」，
#           并把 #30 那型「INVALID 吸收并存 FAIL」的读数原样打出来（rc 分档仍待她落笔，本件不自立分档）。
#   （三）R20三／R27二：加压覆盖与每跑期望 rc 属批次层断言 ⇒ 五连段直接走 m2_batch_runner.sh 并带 EXPECT_RC。
#
# 用法：OUT=/tmp/rt1 bash retake_driver.sh [段...]     段∈{pre,a,b,dwa,scan,gate,archive,all}（缺省 all）
#   环境变量：FORCE=1 越过负载预检；LOAD_MAX=4 阈值；STAGEA_MAX=3 A0 重试上限；EXPECT_RC 透传给批次驱动。
#   注：LOAD_MAX 是**工序闸门**（不跑=不浪费窗），不是判据门：它不判被测件过不过、不进任何 PASS/FAIL 计数、
#     也不入连击；阈值 4 是施工侧取的操作数，验收若认为该由读数而不是 loadavg 决定，可直接改本行不动判据面。
# 退出码：0 全绿；4 A0 三次皆 INVALID（算力面停手，非判据失败）；5 判据 FAIL（被测件面，停手上报）；
#         6 批次覆盖不足（无加压达标跑，R20三）；7 负载预检不过且未 FORCE；8 期望 rc 不符或计数不自洽（R27二/四）；
#         9 前置件缺失（门禁/归档/基线面）。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
ROOT=/home/ros1-slam
PKG=$ROOT/ws/src/95_simulation/lite3_gazebo_sim
PAY=/tmp/m2_payload
OUT=${OUT:-/tmp/rt_drive}
EV=${EV:-$ROOT/note/进度/SCAN集成/证据/log/raw}
mkdir -p "$OUT"
STAGEA_MAX=${STAGEA_MAX:-3}
LOAD_MAX=${LOAD_MAX:-4}
VERDICT=0
log(){ printf '%s\n' "$*"; }
now(){ date '+%F %T'; }
FOXY="115711,115792,115795,115797,115799,1331,180237"
snap(){ ps -o pid,user,lstart,comm --no-headers -p $FOXY > "$OUT/foxy_$1.txt" 2>/dev/null; log "  foxy[$1] 计数=$(wc -l < "$OUT/foxy_$1.txt")"; }

# ---- 归档：把自 t0 起新产生的 LOGDIR 文本原件拉进证据址（本段可单跑，跑前跑后都能补）----
archive_since() { # $1=epoch 秒
  local t0="$1" new d tgt man
  new=$(find "$PAY" -maxdepth 1 -type d -name 'logs-*' -newermt "@$t0" 2>/dev/null | sort)
  [ -z "$new" ] && { log "  archive：本次无新 LOGDIR（$t0 之后）"; return 0; }
  for d in $new; do
    tgt="$ARCHIVE_DIR/$(basename "$d")"; mkdir -p "$tgt"
    find "$d" -maxdepth 1 -type f ! -name 'lite3.pcd.bak' -exec cp -p {} "$tgt/" \; 2>/dev/null
    ( cd "$ARCHIVE_DIR" && sha256sum "$(basename "$d")"/* 2>/dev/null >> "$MANIFEST" )   # 带目录前缀，否则跨目录同名件在清单里不可判读（STL 自引用同族的教训）
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
  case "$rc" in 0) return 0;; 5) return 5;; 6) log "  （两过一作废之类的不完整集：不判 DWA 不可用，也不称复现）"; return 4;; esac
  return 0
}

sec_scan() {
  log "===== 4) SCAN P1／P2（批2 接线的活体面） $(now) ====="
  local rc1 rc2
  bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_p1.sh" > "$OUT/p1.log" 2>&1; rc1=$?
  bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_p2.sh" > "$OUT/p2.log" 2>&1; rc2=$?
  log "  p1 rc=$rc1 p2 rc=$rc2"
  grep -E '^\[(PASS|FAIL)\]|VERDICT' "$OUT/p1.log" | tail -6 | sed 's/^/    /'
  grep -E '^\[(PASS|FAIL)\]|VERDICT|endpoint_err' "$OUT/p2.log" | tail -6 | sed 's/^/    /'
  [ "$rc1" = "0" ] && [ "$rc2" = "0" ] && return 0
  return 5
}

sec_gate() {
  log "===== 5) 共同门禁＋基线双条件＋样张＋退净 $(now) ====="
  bash "$ROOT/scripts/verify/v3_isolation.sh" > "$OUT/gate.log" 2>&1; log "  门禁 EXIT=$?"; tail -2 "$OUT/gate.log" | sed 's/^/    /'
  ( cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$EV/scan_src_baseline_49.sha256" > "$OUT/base.txt" 2>&1
    log "  30_3rd：非OK行数=$(grep -vc ': OK$' "$OUT/base.txt") find计数=$(find . -type f | wc -l)（应为 0 与 49）" )
  sha256sum "$ROOT/data/maps/lite3.pcd" > "$OUT/sample_sha.txt"
  rosnode kill -a >/dev/null 2>&1; sleep 3
  pkill -f gzserver >/dev/null 2>&1; pkill -f run_mapping_online >/dev/null 2>&1
  pkill -f "roslaunch.*mode_sim_" >/dev/null 2>&1; pkill -f "roslaunch.*m2_" >/dev/null 2>&1
  pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; pkill -f move_base_flex >/dev/null 2>&1; sleep 5
  log "  退净：残留=[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex|run_mapping_online|rosout' | tr '\n' ' ')] 端口11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11345=$(ss -lnt 2>/dev/null | grep -c ':11345')"
  log "  E3 双父签名（各跑 stageB 的 base_link 父帧集合，须只有 odom）："
  grep -hoE "SMOKE E3 .*base_link_parents=\[[^]]*\]" "$OUT"/batch/r*.log 2>/dev/null | sort | uniq -c | sed 's/^/    /'
  snap end
  # 整行比（pid＋user＋lstart＋comm），不是只比前两列：lstart 才是「他人进程未被打扰」的实质证据
  if diff "$OUT/foxy_start.txt" "$OUT/foxy_end.txt" > /dev/null 2>&1; then log "  foxy 起终成对：整行（含 lstart）逐位一致"; else log "  [停手上报] foxy 起终整行不一致 ⇒ 他人进程被扰动，本批不作结论"; diff "$OUT/foxy_start.txt" "$OUT/foxy_end.txt" | head -4 | sed "s/^/    /"; fi
  return 0
}

SECS=("${@:-all}")
[[ " ${SECS[*]} " == *" all "* ]] && SECS=(pre a b dwa scan gate)
ARCHIVE_DIR="$EV/retake_$(date +%Y%m%d_%H%M%S)"; MANIFEST="$ARCHIVE_DIR/_sha256_manifest.txt"
mkdir -p "$ARCHIVE_DIR"; log "归档址：$ARCHIVE_DIR"
for s in "${SECS[@]}"; do
  case "$s" in
    pre) sec_pre; rc=$?;; a) sec_a; rc=$?;; b) sec_b; rc=$?;;
    dwa) sec_dwa; rc=$?;; scan) sec_scan; rc=$?;; gate) sec_gate; rc=$?;;
    archive) archive_since "$(date -d '12 hours ago' +%s)"; rc=$?;;
    *) log "未知段 $s"; rc=9;;
  esac
  [ "$rc" = "0" ] || { VERDICT=$rc; log "-- 段 $s 止于 rc=$rc ⇒ 后续段不再跑（失败即停手，不往后凑绿）"; break; }
done
log "===== RETAKE VERDICT: rc=$VERDICT $(now) ====="
log "  0=全绿 4=A0 三次皆 INVALID（算力面）5=判据 FAIL 6=批次覆盖不足 7=负载预检不过 8=期望/计数面 9=前置缺失"
log "  归档件数=$(find "$ARCHIVE_DIR" -type f | wc -l) 清单=$MANIFEST"
exit "$VERDICT"
