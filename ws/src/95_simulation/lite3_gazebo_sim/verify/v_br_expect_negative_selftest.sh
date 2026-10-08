#!/usr/bin/env bash
# v_br_expect_negative_selftest.sh (BRS) —— m2_batch_runner 批次层四档 rc 的零节点阴性对照自测
#
# 为何存在（小美 M2R8 第 11 条：「他自述 fixture 级已验期望表逻辑，我在证据目录未核实（无对应件）⇒ 记欠件，随 R41 一并补」；
#   以及 R41（二）「EXPECT_RC 表逐位显式写出，阴性对照位也要写——这是『新门能把 rc 打成非 0』的现场证据面」）：
#   活体批里五跑的期望恒为 0，BR 的 6/7/8/9 这四档**在活体批里永远打不到**；不造桩就没有现场证据，只有静态推断。
#   本件把被试换成打点桩（走 R49 新加的 SM_CMD／SM_ARGS 口子），逐档打真红，并核「取一序 8>7>6」与「三面逐条都打不隐没」。
#
# 纪律：桩只打 BR 要读的那几行格式（[PASS]／[FAIL]／SUMMARY marks／gate_press_late）；
#   被测对象是**现役 m2_batch_runner.sh 本体**（bash 直调，不复制实现，STS 同一条纪律）；
#   门值 60／35／40、rc 语义、取一序一个都没动，也不碰现役 m2_smoke.sh。
#
# 用法：bash v_br_expect_negative_selftest.sh     退出码 0=全过；5=有 FAIL（本件自有 rc 层，不与 BR／SM／RD 各层互认）
set -u
ROOT=/home/ros1-slam
BR=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/verify/m2_batch_runner.sh
STUB=/tmp/brs_stub_$$.sh
NP=0; NF=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
[ -f "$BR" ] || { echo "[FAIL] 现役 BR 件不在位：$BR"; echo "BRS VERDICT: rc=5"; exit 5; }

# 桩：按 STUB_PLAN 的分号段决定第 n 跑的行为；段格式 rc:pass行首数:fail行首数:打不打SUMMARY:press 态
cat > "$STUB" <<'STUBEOF'
#!/usr/bin/env bash
CNTF="${STUB_CNT:-/tmp/brs_cnt}"
n=$(cat "$CNTF" 2>/dev/null || echo 0); n=$((n+1)); echo "$n" > "$CNTF"
seg=$(echo "${STUB_PLAN}" | cut -d';' -f"$n")
[ -z "$seg" ] && seg=$(echo "${STUB_PLAN}" | cut -d';' -f"1")
IFS=':' read -r rc np nf sum press <<<"$seg"
i=0; while [ "$i" -lt "$np" ]; do echo "[PASS] B: 桩判定 $i"; i=$((i+1)); done
i=0; while [ "$i" -lt "$nf" ]; do echo "[FAIL] B: 桩失败 $i"; i=$((i+1)); done
[ "$press" = "late" ]    && echo "SMOKE gate_press_late=PASS branch=abs+span (reached=True sim2=62.40 span=41.20 判据=(sim2>=60 且 span>=35) 或 span>=40)"
[ "$press" = "unreach" ] && echo "SMOKE gate_press_late=FAIL branch=unreached (reached=False sim2=0.00 span=0.00)"
[ "$sum" = "1" ] && echo "SUMMARY marks pass=${STUB_SUMPASS:-9} fail=$nf"
[ "$sum" = "2" ] && echo "SUMMARY marks pass=$np fail=$nf"
echo "SMOKE teb_state=3 dt=8.5s dev_odom=0.170 dev_gt=0.170"
[ "$rc" = "0" ] && echo "M2 SMOKE: PASS" || echo "M2 SMOKE: FAIL"
exit "$rc"
STUBEOF
# sum=1 ⇒ 自报数取 STUB_SUMPASS（可与行首数不等，用来造「计数不自洽」）；sum=2 ⇒ 自报与行首同数
export STUB_SUMPASS="${STUB_SUMPASS:-9}"

runbr(){ # $1=工作目录 $2=桩计划 $3=EXPECT_RC（空串=不设期望表） $4..=跑规格（缺省 r1:0 r2:62 r3:0）
  local wd="$1" plan="$2" exp="$3"; shift 3
  [ $# -eq 0 ] && set -- r1:0 r2:62 r3:0
  rm -rf "$wd"; mkdir -p "$wd"; echo 0 > "$wd/cnt"
  # R49 三：这里必须用 env 前缀式。**曾写成的「export 一行＋换行续写变量」是错的**——export 命令在换行处即结束，
  #   第二行的 SM_CMD／SM_ARGS 只落成本 shell 变量、没进环境 ⇒ BR 看到的是缺省值 ⇒ 静默去跑现役 m2_smoke.sh 真仿真。
  ( env STUB_PLAN="$plan" STUB_CNT="$wd/cnt" BATCH_OUT="$wd/b" SLEEP_BETWEEN=0 STUB_SUMPASS="$STUB_SUMPASS" \
        SM_CMD=bash SM_ARGS="$STUB" EXPECT_RC="$exp" \
        bash "$BR" "$@" ) > "$wd/log" 2>&1
  echo "$?" > "$wd/rc"
}
rc_of(){ cat "$1/rc"; }

# C1 阳性对照：三跑全 0、含一次加压达标、计数自洽 ⇒ rc=0（正常面必须先能绿，否则后面全是假红）
runbr /tmp/brs_c1 "0:2:0:2:none;0:2:0:2:late;0:2:0:2:none" "0 0 0"
[ "$(rc_of /tmp/brs_c1)" = "0" ] && grep -q '批内含 ≥1 次加压档达标跑' /tmp/brs_c1/log \
  && grep -q '\[count\] 自洽' /tmp/brs_c1/log \
  && mark 0 "C1 阳性对照：正常批 rc=0、加压覆盖与计数自洽都在" || mark 1 "C1 阳性对照不过（rc=$(rc_of /tmp/brs_c1)）"
# C0 被试指认：批次日志里每一跑的被试行必须都点名桩件（这一条就是 19:08 那型回落的当场探测器）
NSTALL=$(grep -c '\[subject\] 本跑被试＝bash /tmp/brs_stub' /tmp/brs_c1/log)
NSREAL=$(grep -c '\[subject\] 本跑被试＝bash verify/m2_smoke.sh' /tmp/brs_c1/log)
[ "$NSTALL" = "3" ] && [ "$NSREAL" = "0" ] \
  && mark 0 "C0 被试指认：三跑全点名桩件，零次回落到现役 SM" || mark 1 "C0 被试指认不过（桩=$NSTALL 真=$NSREAL，应 3／0）"



# C2 rc=7：期望 0 而实得 3 ⇒ 期望面断；同批无加压达标跑（6 那面）也必须照打
runbr /tmp/brs_c2 "3:2:0:2:none" "0" r1:0
[ "$(rc_of /tmp/brs_c2)" = "7" ] && grep -q '\[expect\] 跑1 期望 rc=0 实得 rc=3 ⇒ BREAK' /tmp/brs_c2/log \
  && grep -q '批内无加压档达标跑' /tmp/brs_c2/log \
  && mark 0 "C2 期望不符 ⇒ rc=7，且 6 那面的 FAIL 行不被取一隐没" || mark 1 "C2 期望档不过 7（rc=$(rc_of /tmp/brs_c2)）"

# C3 阴性对照位按设计成立：实得 1、期望就写 1 ⇒ match ⇒ 批次仍 rc=0（#17(i) 要的「设计外 FAIL 与阴性 FAIL 可分」）
runbr /tmp/brs_c3 "0:2:0:2:none;1:2:0:2:late;0:2:0:2:none" "0 1 0"
[ "$(rc_of /tmp/brs_c3)" = "0" ] && grep -q '\[expect\] 跑2 期望 rc=1 实得 rc=1 ⇒ match' /tmp/brs_c3/log \
  && grep -q '每跑实得 rc 与期望表逐位相符' /tmp/brs_c3/log \
  && mark 0 "C3 阴性位（期望 1 实得 1）被逐位消化 ⇒ 批次 rc=0，这就是「非 0 期望位」的形态" \
  || mark 1 "C3 阴性位反被判红（rc=$(rc_of /tmp/brs_c3)）"

# C4 rc=8：自报 pass=9 而原件行首只有 2 ⇒ 计数不自洽（STUB_SUMPASS 造这型）
STUB_SUMPASS=9 runbr /tmp/brs_c4 "0:2:0:1:none;0:2:0:1:late;0:2:0:1:none" "0 0 0"
[ "$(rc_of /tmp/brs_c4)" = "8" ] && grep -q '不自洽：自报 pass=9 fail=0 ≠ 原件行首 pass=2 fail=0' /tmp/brs_c4/log \
  && mark 0 "C4 计数不自洽（自报 9 对原件 2）⇒ rc=8" || mark 1 "C4 计数不自洽未打 8（rc=$(rc_of /tmp/brs_c4)）"

# C5 rc=8：桩不打 SUMMARY ⇒ 判「不可自证」而不是默许
runbr /tmp/brs_c5 "0:2:0:0:none;0:2:0:0:late;0:2:0:0:none" "0 0 0"
[ "$(rc_of /tmp/brs_c5)" = "8" ] && grep -q '无 SUMMARY marks 行' /tmp/brs_c5/log \
  && mark 0 "C5 缺 SUMMARY 判不可自证 ⇒ rc=8（不是默认绿）" || mark 1 "C5 缺 SUMMARY 竟默许（rc=$(rc_of /tmp/brs_c5)）"

# C6 rc=9：期望表长度与跑数不符 ⇒ 起跑前即停，且不产出任何被试 log（作业单错不判被试）
runbr /tmp/brs_c6 "0:2:0:2:late;0:2:0:2:none" "0 0 0" r1:0 r2:62
NOLOG=$(find /tmp/brs_c6/b -name 'r*.log' 2>/dev/null | wc -l)
[ "$(rc_of /tmp/brs_c6)" = "9" ] && [ "$NOLOG" = "0" ] \
  && mark 0 "C6 长度不符 ⇒ rc=9 且被试未起跑（被试 log=$NOLOG）" || mark 1 "C6 长度档不过（rc=$(rc_of /tmp/brs_c6) 被试 log=$NOLOG）"

# C7 rc=6：三跑全 0、计数自洽，但 press 只有 unreached ⇒ 覆盖不足；unreached 注行必须照打（#17(ii) 的快扫易读错面）
runbr /tmp/brs_c7 "0:2:0:2:none;0:2:0:2:unreach;0:2:0:2:none" "0 0 0"
[ "$(rc_of /tmp/brs_c7)" = "6" ] && grep -q '加压档未达 sim 目标' /tmp/brs_c7/log \
  && mark 0 "C7 无加压达标跑 ⇒ rc=6 且 unreached 专列注行在位" || mark 1 "C7 覆盖档不过（rc=$(rc_of /tmp/brs_c7)）"

# C8 取一序 8>7>6：三面同时不过必须取 8，且其余两面的 FAIL 行都在
runbr /tmp/brs_c8 "3:2:0:1:none" "0" r1:0
[ "$(rc_of /tmp/brs_c8)" = "8" ] && grep -q '跑的 rc 与期望表不符' /tmp/brs_c8/log \
  && grep -q '计数不自洽跑数=1' /tmp/brs_c8/log && grep -q '批内无加压档达标跑' /tmp/brs_c8/log \
  && mark 0 "C8 三面同不过 ⇒ 取一序给 8，另两面 FAIL 行逐条都在" || mark 1 "C8 取一序不成立（rc=$(rc_of /tmp/brs_c8)）"

# C9 不设期望表：期望面缺席（逐跑行写 exp=未设、无 [expect] 行），其余两面照核 ⇒ rc=0
runbr /tmp/brs_c9 "0:2:0:2:late" "" r2:62
grep -q 'exp=未设' /tmp/brs_c9/log && [ "$(grep -c '\[expect\]' /tmp/brs_c9/log)" = "0" ] && [ "$(rc_of /tmp/brs_c9)" = "0" ] \
  && mark 0 "C9 未设期望表时期望面缺席 ⇒ rc=0" || mark 1 "C9 未设期望表面异常（rc=$(rc_of /tmp/brs_c9)）"

rm -f "$STUB"
echo "=================================================="
echo "BRS VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
