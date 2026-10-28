#!/bin/bash
# v11 全矩阵 36 组（f64nli）：SC09 12 + SC01/02 24，逐组重启栈、复位、跑 driver
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/ws/src/90_platform/v8_platform/calibration/flat2d
OUT=/work/v11_out
LOG=/work/v11_36.log
LOCK=/work/v11_36.lock
mkdir -p "$OUT"

if [ -f "$LOCK" ] && kill -0 "$(cat $LOCK 2>/dev/null)" 2>/dev/null; then
  echo "already running pid=$(cat $LOCK)"; exit 3
fi
echo $$ > "$LOCK"
trap 'rm -f "$LOCK"' EXIT

: > "$LOG"
echo "START $(date -u +%FT%TZ) pid=$$" | tee -a "$LOG"

for man in $(ls $FLAT/runs_v11/*.json $FLAT/runs_v11_sc0102/*.json | grep -v index.sha256); do
  tag=$(basename "$man" .json)
  echo "RUN $tag $(date -u +%T)" | tee -a "$LOG"
  V5_OUT=$OUT bash /work/run_sc09_nli.sh "$man" >>"$LOG" 2>&1
  rc=$?
  if [ -f "$OUT/$tag.json" ]; then
    line=$(python3 -c "import json;d=json.load(open('$OUT/$tag.json'));print(d['verdict'],'goal='+str(d['dist_to_goal_m']),'clr='+str(d['obstacle_min_clearance_m']),'inj='+str(d.get('injection_ok')),'t='+str(d['wall_duration_s']))")
  else
    line="NO_RESULT rc=$rc"
  fi
  echo "DONE $tag rc=$rc $line" | tee -a "$LOG"
done

python3 - <<'PY' | tee -a "$LOG"
import glob, json, math
rows=[]
for f in sorted(glob.glob('/work/v11_out/*.json')):
    d=json.load(open(f)); rows.append((d['run_id'], d['verdict'], d['dist_to_goal_m'], d.get('injection_ok')))
ok=sum(1 for r in rows if r[1]=='PASS')
print('TALLY n=%d pass=%d fail=%d' % (len(rows), ok, len(rows)-ok))
# Clopper-Pearson lower 95% (one-sided) for k/n
from math import comb
def cp_lower(k,n,alpha=0.05):
    # find p s.t. P(X>=k | p)=alpha by bisection
    lo,hi=0.0,1.0
    for _ in range(80):
        p=(lo+hi)/2
        tail=sum(comb(n,i)*p**i*(1-p)**(n-i) for i in range(k,n+1))
        if tail<alpha: lo=p
        else: hi=p
    return (lo+hi)/2
if rows: print('CP lower95 = %.4f (k=%d n=%d)' % (cp_lower(ok,len(rows)), ok, len(rows)))
PY
echo "END $(date -u +%FT%TZ)" | tee -a "$LOG"
