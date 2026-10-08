#!/usr/bin/env bash
# m2_batch_runner.sh —— M2 批次级驱动（小美 M2R6 路由项 R20 三 的落地件；R27 二／R27 四 改版，小明 2026-10-08）
# 为何存在：press_late_G5 只在单跑内判定 ⇒「本批是否真的至少做过一次加压档」不可机判（她 R9续 的残余）。
# 本轮补的两件事（皆出自她 M2R7 结案 #17）：
#   （一）旧版只统计 press 覆盖，b3 那类设计外 FAIL（rc=1）在批次层与「阴性对照应有的 FAIL」不可分（#17 i）
#         ⇒ 加每跑期望 rc 表 EXPECT_RC，实得与期望逐位比对，不符即批次判不符（R27 二）。
#   （二）判定行打印数与 mark 计数逐跑自洽（#2：表上写 6 PASS/2 FAIL 而原件实为 3/1，不可指认）
#         ⇒ 批次层核对 harness 的 SUMMARY marks 与原件行首 [PASS]／[FAIL] 计数（R27 四）。
#   branch 打印在 unreached 跑曾被快扫读成达标（#17 ii）：harness 侧已补 unreached 态，本件把它单列一行。
# 用法：EXPECT_RC="0 0 1" bash m2_batch_runner.sh b1: b2:62 b3:99999
#   期望表按跑序逐位对齐（空格分隔）；不设 EXPECT_RC = 不校验期望，只跑 press 覆盖与计数自洽两面。
# 本件自身 rc：0=批次 VERDICT PASS；6=批内无加压档达标跑（R20 三）；7=有跑的实得 rc 与期望表不符（R27 二）；
#   8=判定计数不自洽或 harness 未打 SUMMARY（R27 四）；9=期望表长度与跑数不符（作业单本身错，不判被试）。
#   多面同时不过时按 8 > 7 > 6 取一（先证据不可指认，再设计外 FAIL，再批次覆盖不足）；VERDICT 行把三面逐条都打，不因取一而隐没其余。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
B=${BATCH_OUT:-/tmp/m2_batch}
mkdir -p "$B"
snap(){ echo "### foxy $1 $(date '+%T')"; ps -o pid,user,lstart,etime,comm --no-headers \
        -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee "$B/foxy_$1.txt" | wc -l; }

EXPECTED=(${EXPECT_RC:-})
if [ "${#EXPECTED[@]}" -gt 0 ] && [ "${#EXPECTED[@]}" -ne "$#" ]; then
  echo "期望表长度=${#EXPECTED[@]} 与跑数=$# 不符 ⇒ 作业单本身错，起跑前即停（不判被试）"; exit 9
fi
idx=0; PRESS_OK=0; NPASS=0; NFAIL=0; EXP_BREAK=0; CNT_BREAK=0; SUMMARY_MISSING=0
declare -a ROWS
for spec in "$@"; do
  idx=$((idx+1)); tag=${spec%%:*}; dly=${spec#*:}
  echo "##### $tag G5_DELAY_SIM=[$dly] $(date '+%T') #####"
  snap "start_$tag"
  TF_SOURCE="" G5_DELAY_SIM="$dly" bash verify/m2_smoke.sh stageB > "$B/$tag.log" 2>&1
  rc=$?
  snap "end_$tag"
  pl=$(grep -oE "gate_press_late=[A-Z]+ branch=[a-zA-Z+]+" "$B/$tag.log" | head -1)
  br=$(echo "$pl" | grep -oE "branch=[a-zA-Z+]+" | cut -d= -f2)
  [ -n "$dly" ] && [ "$dly" != "0" ] && [ "$br" = "unreached" ] && \
    echo "  [注] 本跑加压档未达 sim 目标（branch=unreached ⇒ 走 300s 墙钟兜底，非取龄达标）"
  echo "  rc=$rc | ${pl:-press 未参与（未开档）} | $(grep -oE 'teb_state=[0-9]+ dt=[0-9.]+s dev_odom=[0-9.]+' "$B/$tag.log" | head -1)"
  grep -E "^\[exitclean\]|^\[PASS\]|^\[FAIL\]|^\[INVALID\]|^SUMMARY" "$B/$tag.log" | sed 's/^/    /'
  [ "$rc" = "0" ] && NPASS=$((NPASS+1)) || NFAIL=$((NFAIL+1))
  if [ -n "$dly" ] && [ "$dly" != "0" ] && echo "$pl" | grep -q "=PASS"; then PRESS_OK=$((PRESS_OK+1)); fi
  # ---- R27 二：期望 rc 表逐位比对 ----
  if [ "${#EXPECTED[@]}" -gt 0 ]; then
    exp=${EXPECTED[$((idx-1))]}
    if [ "$rc" = "$exp" ]; then exl=match; else exl=BREAK; EXP_BREAK=$((EXP_BREAK+1)); fi
    echo "  [expect] 跑$idx 期望 rc=$exp 实得 rc=$rc ⇒ $exl"
    ROWS+=("$tag rc=$rc exp=$exp $exl press=${br:-off}")
  else
    ROWS+=("$tag rc=$rc exp=未设 press=${br:-off}")
  fi
  # ---- R27 四：判定计数自洽（harness 自报 vs 原件行首计数；不等即该跑的 PASS/FAIL 面不可指认）----
  sm=$(grep -E '^SUMMARY marks' "$B/$tag.log" | tail -1)
  sp=$(grep -c '^\[PASS\]' "$B/$tag.log"); sf=$(grep -c '^\[FAIL\]' "$B/$tag.log")
  if [ -z "$sm" ]; then
    SUMMARY_MISSING=$((SUMMARY_MISSING+1)); echo "  [count] 该跑无 SUMMARY marks 行 ⇒ harness 代际过旧或判定段未跑到，计数面不可自证"
  else
    mpa=$(echo "$sm" | grep -oE 'pass=[0-9]+' | cut -d= -f2); mfa=$(echo "$sm" | grep -oE 'fail=[0-9]+' | cut -d= -f2)
    if [ "$mpa" = "$sp" ] && [ "$mfa" = "$sf" ]; then
      echo "  [count] 自洽：自报 pass=$mpa fail=$mfa ＝ 原件行首 pass=$sp fail=$sf"
    else
      CNT_BREAK=$((CNT_BREAK+1)); echo "  [count] 不自洽：自报 pass=${mpa:-?} fail=${mfa:-?} ≠ 原件行首 pass=$sp fail=$sf"
    fi
  fi
  sleep 4
done
echo "##### 批次级断言（R20 三 ＋ R27 二 ＋ R27 四）#####"
echo "  逐跑：${ROWS[*]}"
[ "$PRESS_OK" -ge 1 ] && echo "  [PASS] 批内含 ≥1 次加压档达标跑（实测 $PRESS_OK）" \
                      || echo "  [FAIL] 批内无加压档达标跑 ⇒ 批次判不符（R20 三）"
if [ "${#EXPECTED[@]}" -gt 0 ]; then
  [ "$EXP_BREAK" = "0" ] && echo "  [PASS] 每跑实得 rc 与期望表逐位相符（R27 二）" \
                         || echo "  [FAIL] $EXP_BREAK 跑的 rc 与期望表不符 ⇒ 设计外 FAIL 或阴性对照未 FAIL（R27 二）"
fi
if [ "$CNT_BREAK" = "0" ] && [ "$SUMMARY_MISSING" = "0" ]; then
  echo "  [PASS] 每跑判定计数自洽（R27 四）"
else
  echo "  [FAIL] 计数不自洽跑数=$CNT_BREAK，缺 SUMMARY 跑数=$SUMMARY_MISSING（R27 四）"
fi
echo "  跑况：PASS=$NPASS FAIL=$NFAIL / $# 跑"
echo "  foxy 成对快照件数=$(ls "$B" | grep -c foxy_)"
_rc=0
if [ "$CNT_BREAK" -gt 0 ] || [ "$SUMMARY_MISSING" -gt 0 ]; then _rc=8
elif [ "$EXP_BREAK" -gt 0 ]; then _rc=7
elif [ "$PRESS_OK" -lt 1 ]; then _rc=6
fi
echo "BATCH VERDICT: rc=$_rc（0=PASS 6=无加压达标跑 7=期望 rc 不符 8=计数不自洽 9=期望表长度错）"
exit $_rc
