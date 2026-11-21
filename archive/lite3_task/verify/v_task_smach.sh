#!/usr/bin/env bash
# v_task_smach.sh —— M5.1 独立判据件（矩阵 v2 追认口径：新增独立件，不碰 sim_nav_smoke.sh 13 CHECK 冻结判据）
# 用法: v_task_smach.sh <task_smach 运行 log> [dt上限秒，缺省 60]
# 判据（roadmap B 文 B3 / C 文 P0 表 #2）:
#   判据一 task outcome=succeeded
#   判据二 任务链总时长 dt≤60s（log 内「总时长」行）
#   判据三 各 phase outcome 行在册（wait/get_path/exe_path 的 [M5] 原语 log）
# 输出: 逐判据 PASS/FAIL + 终判 True/False；exit 0=True 1=False（供 CI 取用）
set -u
LOG=${1:?用法: v_task_smach.sh <log> [dt上限秒]}
DT_MAX=${2:-60}
[ -r "$LOG" ] || { echo "FAIL 判据0: log 不可读 $LOG"; echo False; exit 1; }

ok=1
# 判据一：task outcome
OUT=$(grep 'task outcome' "$LOG" | tail -1)
if grep -q "task outcome: succeeded" "$LOG"; then
  echo "PASS 判据一: $OUT"
else
  echo "FAIL 判据一: 未见 task outcome: succeeded（${OUT:-无 outcome 行}）"; ok=0
fi
# 判据二：任务链时长
DT=$(grep -oP '总时长 \K[0-9.]+(?=s)' "$LOG" | tail -1)
if [ -n "$DT" ]; then
  if awk -v a="$DT" -v b="$DT_MAX" 'BEGIN{exit !(a<=b)}'; then
    echo "PASS 判据二: dt=${DT}s ≤ ${DT_MAX}s"
  else
    echo "FAIL 判据二: dt=${DT}s > ${DT_MAX}s"; ok=0
  fi
else
  echo "FAIL 判据二: log 无「总时长」行"; ok=0
fi
# 判据三：各 phase outcome 行
MISS=""
for ph in wait get_path exe_path; do
  grep -q "\[M5\] 原语 $ph" "$LOG" || MISS="$MISS $ph"
done
if [ -z "$MISS" ]; then
  echo "PASS 判据三: wait/get_path/exe_path phase log 三连在册（$(grep -c '\[M5\] 原语' "$LOG") 行原语 log）"
else
  echo "FAIL 判据三: 缺 phase log:$MISS"; ok=0
fi
if [ "$ok" -eq 1 ]; then echo True; exit 0; else echo False; exit 1; fi
