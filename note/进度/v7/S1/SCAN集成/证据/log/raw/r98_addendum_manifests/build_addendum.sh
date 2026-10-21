#!/usr/bin/env bash
# build_addendum.sh —— 给**被在册文字引用过**的历史证据目录旁挂补建清单（R98；姿势照 R48「补不覆」）
#
# 为什么只做"被引用过"的：补建清单只作"自今日起的完整性锚点、不证明历史未改"（04 R97 已写这句）。
#   对无人引用的目录做这件事＝纯台账噪声；若验收要求全量，跑 `bash build_addendum.sh --all` 即可（口子留在件里）。
# 条目形状：一律带 "<目录名>/" 前缀、**基准＝父目录（raw/）**——
#   这是候选巡检件 v_evidence_reachability.sh 的父基准规则所要求的（它只有在"每条都从父目录跑通"时才接受父基准，
#   只放宽不校验会被父层同名件骗成假绿，见 04 R97）。
# 用法：bash build_addendum.sh [--all]      缺省＝只补被引用者
# 性质：只在各目标目录内**新增** `_manifest_recheck.txt` 一件；不覆盖任何原件；零链窗、无 sudo。
# 退出码：0 完成（含"跳过若干"）｜9 前置缺失（工作目录不在位／sha256sum 不可用）
set -u
RAW=/home/ros1-slam/note/进度/SCAN集成/证据/log/raw
cd "$RAW" || { echo "前置缺失：$RAW 不在位"; exit 9; }
command -v sha256sum >/dev/null 2>&1 || { echo "前置缺失：sha256sum 不可用"; exit 9; }
MODE="${1:-cited}"
OUT=/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r98_addendum_manifests
mkdir -p "$OUT"
LOG="$OUT/build_addendum.log"
: > "$LOG"

# 目标＝候选巡检件 P3a-2/P3b-3 两类"已知态"名单（20 个约定前缺清单 ＋ 5 个旧式不可验）
TARGETS="b1_gen5_logs b1_out2 b1_retake b3_configure_pre batch2_wire env_free2_20261008 env_free3_20261008_2241
env_free_20261008 fd2_pre_baseline h14_zero_node_batch j3_mux_row m2_tmp_payload_logs m2r6_exitclean
m2r7_batch m2r7_batch_logs r26_32_zero_node_batch r26_selftest r31_fd_i r42_joint_surface r46_syntax_rescan
retake_20261008_143433 retake_20261008_144159 retake_20261008_144235 retake_20261008_145931
retake_20261008_182329 retake_20261008_182329_addendum"

built=0; skipped=0; already=0; broken=0
for d in $TARGETS; do
  [ -d "$d" ] || { echo "$d 缺目录 ⇒ 跳过" >> "$LOG"; skipped=$((skipped + 1)); continue; }
  if [ -f "$d/_sha256_manifest.txt" ]; then
    # 已有正名清单时**不覆盖它**；但如果它从本目录跑不通（旧式缺 out/ 前缀那一族），
    #   就在旁边补一份可验的 recheck——这正是 R48 对 18:23 批做过的事，同一姿势推广到 14:xx 那四批。
    ok_here=$( sha256sum -c "$d/_sha256_manifest.txt" 2>/dev/null | grep -c ': OK' )
    if [ "$ok_here" -gt 0 ]; then
      echo "$d 已有正名清单且本目录可验（OK=$ok_here）⇒ 完全不补" >> "$LOG"; already=$((already + 1)); continue
    fi
    echo "$d 正名清单不可验（OK=$ok_here）⇒ 旁边补一份 recheck，原件一字不动" >> "$LOG"
  fi
  if [ -f "$d/_manifest_recheck.txt" ]; then
    echo "$d 已有 _manifest_recheck.txt ⇒ **跳过不覆盖**（补不覆；首跑就是没这条闸才把 R48 的 addendum 清单写坏）" >> "$LOG"
    already=$((already + 1)); continue
  fi
  if [ "$MODE" != "--all" ]; then
    n=$(grep -rl -- "$d" /home/ros1-slam/note/04_配置改动diff.md /home/ros1-slam/note/进度/00_总览.md \
        /home/ros1-slam/note/进度/*/进度文档.md /home/ros1-slam/note/进度/*/设计/*.md 2>/dev/null | wc -l)
    if [ "$n" -eq 0 ]; then
      echo "$d 在册引用=0 ⇒ 本轮不补（避免台账噪声；--all 可强补）" >> "$LOG"; skipped=$((skipped + 1)); continue
    fi
    ref="引用=$n"
  else
    ref="引用=未查(--all)"
  fi
  # 基准＝RAW 本身（首跑误写成 `cd ..` ⇒ 找不到件、18 个目录全出空清单，当场被自己的不等检查打红）
  #   条目自带 "<目录名>/" 前缀 ⇒ 复核命令是 `cd raw && sha256sum -c <dir>/_manifest_recheck.txt`，
  #   与 04:405 那条 addendum 在册复核命令同形，也正是 VAR 父基准规则所要求的形状。
  ( find "$d" -type f ! -name "_manifest_recheck.txt" -print | sort | while read -r f; do sha256sum "$f"; done ) > "$OUT/.tmp_$d"
  {
    echo "# 补建清单（R98，$(date '+%F %T')）：本目录原无 _sha256_manifest.txt（早于 R48 的清单约定）或被旧式条目所害"
    echo "# 校验 CWD＝**父目录 raw/**：cd 父目录 && sha256sum -c $d/_manifest_recheck.txt"
    echo '# 只作"自今日起的完整性锚点"，不证明历史未改；目录内原件与任何原清单一字未动（补不覆，照 R48）'
    cat "$OUT/.tmp_$d"
  } > "$d/_manifest_recheck.txt"
  rm -f "$OUT/.tmp_$d"
  tot=$(grep -vc '^#' "$d/_manifest_recheck.txt")
  chk=$( sha256sum -c "$d/_manifest_recheck.txt" 2>/dev/null | grep -c ': OK' )
  if [ "$tot" = "0" ]; then
    echo "$d **空清单**⇒ 删掉自己的产出并停手（空清单不是证据，坑29）" >> "$LOG"
    rm -f "$d/_manifest_recheck.txt"; broken=$((broken + 1)); continue
  elif [ "$tot" != "$chk" ] || [ "$chk" -lt 1 ]; then
    echo "$d **不等**：条目=$tot OK=$chk ⇒ 人工看（不删不改，留着暴露）" >> "$LOG"; broken=$((broken + 1))
  else
    echo "$d 旁挂完成（$ref）：条目=$tot OK=$chk" >> "$LOG"
  fi
  chmod 664 "$d/_manifest_recheck.txt"
  setfacl -m u:ros1-slam:rw- "$d/_manifest_recheck.txt" 2>/dev/null || true
  built=$((built + 1))
done
{ echo "----"; echo "合计：处理 $built 个目录（其中不等 $broken）、已有正名不动 $already、跳过 $skipped；模式=$MODE"; } >> "$LOG"
chmod 664 "$LOG"; setfacl -m u:ros1-slam:rw- "$LOG" 2>/dev/null || true
tail -3 "$LOG"
[ "$broken" -gt 0 ] && { echo "BUILD VERDICT rc=5（有目录补建后校验不等，见 log）"; exit 5; }
echo "BUILD VERDICT rc=0"; exit 0
