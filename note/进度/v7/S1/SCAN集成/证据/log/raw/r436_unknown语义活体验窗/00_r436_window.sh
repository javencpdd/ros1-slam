#!/usr/bin/env bash
# 00_r436_window.sh —— ★窗号 R436★（先报后开）＝§8 预检 4 的活体验（unknown=−1 与 0 在 costmap 里的解释差异）
# 档名＝联调·冒烟级（★不是判据窗、★A59 不计分子、★桶名甲-离）；★只 kill 本窗 PID；★预检不过不起窗；★退净重采到全零才收窗（坑72 追加例＋R429 自纠二的新姿势）
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r436_unknown语义活体验窗
RUN="$R/run"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
T0=$(date +%s); PIDS=""
cleanup(){ for p in $PIDS; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT
seg_pids=""
cleanup_seg(){ for p in $seg_pids; do kill "$p" 2>/dev/null; done; seg_pids=""; }

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "== 步0b 起前见证（被试域＝容器 netns）=="
bash "$R/witness.sh" 起前 "$R/02_witness_before.txt"; tail -3 "$R/02_witness_before.txt" | sed 's/^/    /' | tee -a "$R/00_window_log.txt"

roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）=="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"

run_seg(){
  local NS="$1"
  say "== 段 $NS（★$(grep -o 'track_unknown_space: [a-z]*' "$RUN/params_$NS.yaml")）=="
  python3 "$R/pub_unknown_probe.py" "$NS" > "$RUN/pub_$NS.log" 2>&1 & seg_pids="$seg_pids $!"
  sleep 3
  rosparam load "$RUN/params_$NS.yaml" "/$NS/costmap" || { say "  ★load_FAIL"; cleanup_seg; return 9; }
  rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & seg_pids="$seg_pids $!"
  local rdy=0 j
  for j in $(seq 1 40); do rostopic info "/$NS/costmap/costmap" >/dev/null 2>&1 && { rdy=1; break; }; sleep 1; done
  say "  costmap 就绪＝$rdy（按 rostopic info 判、非固定 sleep；等待 ${j}s）"
  [ "$rdy" != 1 ] && { say "  ★未就绪 ⇒ 该段记 N3"; cleanup_seg; return 8; }
  { echo "# rostopic info 逐字（★in-band 带话题名＝R429 裁定人 (d) 的姿势）"; rostopic info "/$NS/costmap/costmap" 2>&1; } > "$RUN/topics_$NS.txt"
  timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
  say "  取样完成 data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt") 字节=$(stat -c%s "$RUN/cost_$NS.txt" 2>/dev/null)"
  cleanup_seg; sleep 3
  return 0
}
run_seg u1; A=$?
run_seg u2; B=$?

say "== 步9 退净核＋见证重采（★跑到五名与端口同时全零、★最多三次）=="
for k in 1 2 3; do
  bash "$R/witness.sh" "退后第${k}采" "$R/09_witness_after$k.txt"
  if grep -qE '五名计数=0' "$R/09_witness_after$k.txt" && ! ss -lnt 2>/dev/null | grep -qE ':(11311|11312|11313)\b'; then say "  ★第${k}采＝五名 0 且端口 0 ⇒ 退净达标"; break; fi
  say "  第${k}采未达标（$(grep -E '五名计数=' "$R/09_witness_after$k.txt" | head -1)）⇒ 只收本窗 master holder 后重采"
  for q in $(ss -lntp 2>/dev/null | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
    cl=$(tr -d '\0' < /proc/$q/cmdline 2>/dev/null)
    case "$cl" in *rosmaster*|*roscore*) kill -9 "$q" 2>/dev/null; say "    收 master holder pid=$q";; *) say "    ★跳过非 master holder pid=$q";; esac
  done; sleep 4
done
cp "$R/09_witness_after1.txt" "$R/08_witness_after1.txt" 2>/dev/null
say "  首采与末采同值＝$(diff <(grep -v '^# 见证' "$R/08_witness_after1.txt") <(grep -v '^# 见证' "$R/09_witness_after3.txt" 2>/dev/null || grep -v '^# 见证' "$R/09_witness_after2.txt") >/dev/null 2>&1 && echo True || echo False)"

say "== 步10 判读（★只读对比件、★两态并报）=="
python3 "$R/_cmp_unknown.py" "$RUN/cost_u1.txt" "$RUN/cost_u2.txt" > "$R/05_unknown_semantics.txt" 2>&1
CC=$?; say "  对比件 rc=$CC"; cat "$R/05_unknown_semantics.txt" | tee -a "$R/00_window_log.txt"

say "== 步11 清盘（清理与计数同条命令、路径全集）=="
find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -type d -name __pycache__ -prune -exec rm -rf {} + 2>/dev/null
echo "  pyc=$(find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f|wc -l) tooling=$(find /opt/ros1-slam/tooling -type f|wc -l) note=$(find /opt/ros1-slam/note -type f|wc -l)" | tee -a "$R/00_window_log.txt"
say "== 窗结束：预检 $PRC｜u1 $A｜u2 $B｜对比 $CC｜时长 $(( $(date +%s) - T0 ))s（上限 10 分钟）｜★联调·冒烟、不计分子 =="
exit 0
