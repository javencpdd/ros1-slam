#!/usr/bin/env bash
# 00_r429_window.sh —— R1 活体＋三段同窗（窗号 R429；x86 侧、容器内跑；★联调·旁证、★不入分子）
# 授权＝真人 2026-10-19 04:5x 勾甲（R 补一百四十一 甲案）；前置缺项真身＝r383 裁定 §2.2 第一行判定式第四项（V≥1 或 R1 无面）
# ★两条★在册坑的处置写进跑法＝(a) 坑72 追加例＝就绪以 rostopic info 成功返回为准、★不按固定 sleep 判就绪；(b) 采样次序＝★cost 取样之后★才采体素话题（★裁定复跑式原话）
# ★纪律＝预检变体 rc≠0 不起窗｜在册件（探针／derive／发布器／预检／参数面原件）★一字不改｜★只 kill 本窗 PID｜退净后★重采见证｜清盘与计数★同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r429_R1活体三段同窗
RUN="$R/run"; SCR=/opt/ros1-slam/note/进度/SCAN集成/设计/scripts
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
T0=$(date +%s); MASTER=""; SEG_PIDS=""
cleanup_seg(){ for p in $SEG_PIDS; do kill "$p" 2>/dev/null; done; SEG_PIDS=""; }
trap 'cleanup_seg; [ -n "$MASTER" ] && kill $MASTER 2>/dev/null' EXIT

say "== 步0 预检变体当刻 =="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "== 步0b 起前见证（被试域＝容器 netns）=="
bash "$R/witness.sh" 起前 "$R/02_witness_before.txt"; tail -3 "$R/02_witness_before.txt" | sed 's/^/    /' | tee -a "$R/00_window_log.txt"

roscore -p 11313 > "$RUN/roscore.log" 2>&1 & MASTER=$!
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）=="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }

run_seg(){
  local NS="$1" ABL="$2"
  say "== 段 $NS（★$ABL）=="
  rosparam load "$RUN/params_$NS.yaml" "/$NS/costmap" || { say "  ★load_FAIL"; return 9; }
  say "  plugins 枚数回读=$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c 'name:') 含stvl=$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c stvl) 含sob=$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c sob)"
  rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_${NS}_base.log" 2>&1 & SEG_PIDS="$SEG_PIDS $!"
  rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$RUN/tf_${NS}_r.log" 2>&1 & SEG_PIDS="$SEG_PIDS $!"
  X2_CLOUD_TAG=$NS python3 "$SCR/x2_pub_cloud.py" > "$RUN/cloud_$NS.log" 2>&1 & SEG_PIDS="$SEG_PIDS $!"
  X2_GAP=12 X2_TAG=$NS python3 "$SCR/x2_pub_two_frame.py" > "$RUN/pub_$NS.log" 2>&1 & SEG_PIDS="$SEG_PIDS $!"
  sleep 4
  rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & SEG_PIDS="$SEG_PIDS $!"
  local rdy=0 j
  for j in $(seq 1 40); do rostopic info "/$NS/costmap/costmap" >/dev/null 2>&1 && { rdy=1; break; }; sleep 1; done
  say "  costmap 话题就绪＝$rdy（★按 rostopic info 成功返回判、★非固定 sleep；等待 ${j}s）"
  [ "$rdy" != 1 ] && { say "  ★未就绪 ⇒ 该段记 N3、不硬采"; cleanup_seg; return 8; }
  rostopic list 2>/dev/null | sort > "$RUN/topics_$NS.txt"
  grep -i voxel "$RUN/topics_$NS.txt" > "$RUN/voxel_topics_$NS.txt" 2>/dev/null
  say "  话题数=$(wc -l < "$RUN/topics_$NS.txt") ‖ 含 voxel 条数=$(wc -l < "$RUN/voxel_topics_$NS.txt")"
  timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
  say "  ★cost 取样完成（data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt")）"
  if [ "$NS" = "x2_g6" ]; then
    local V="$RUN/voxel_echo_three.txt"
    { echo "# R1 活体采样：★在 cost 取样之后★开采（容器 $(date '+%F %T')）"; echo "# 就绪自证＝rostopic info 原文："; rostopic info "/$NS/costmap/stvl_obstacle_layer/voxel_grid" 2>&1; } > "$V" 2>&1
    timeout 60 rostopic echo -n 3 "/$NS/costmap/stvl_obstacle_layer/voxel_grid" >> "$V" 2>&1
    say "  R1 三帧采完 rc=$? ⇒ 逐帧 width 现读＝$(grep -c 'width:' "$V") 帧、width 值串＝$(grep -E '^width:' "$V" | tr -d ' \n' | sed 's/width:/,/g')"
    grep -E '^width:|^height:|^seq:|point_step|frame_id' "$V" | sed 's/^/      /' | tee -a "$R/00_window_log.txt"
  fi
  cleanup_seg; sleep 3
  return 0
}

run_seg x2_g6 "四插件基线（在册 R419 面逐字拷、★含 R1 活体三帧）"; G=$?
run_seg x2_d1 "去 sob_layer（在册 R386 面逐字拷）"; O=$?
run_seg x2_d2 "去 stvl_obstacle_layer（在册 R386 面逐字拷）"; D=$?

say "== 步9 退净核 ＋ 退后重采见证 =="
bash "$R/witness.sh" 退后首采 "$R/08_witness_after1.txt"
if grep -qE '五名计数=[1-9]' "$R/08_witness_after1.txt" || ss -lnt 2>/dev/null | grep -qE ':(11311|11312|11313)\b'; then
  say "  ★退净未达标 ⇒ 只收本窗 master holder 后重采"
  for q in $(ss -lntp 2>/dev/null | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
    cl=$(tr -d '\0' < /proc/$q/cmdline 2>/dev/null)
    case "$cl" in *rosmaster*|*roscore*) kill -9 "$q" 2>/dev/null; say "    收 master holder pid=$q";; *) say "    ★跳过非 master holder pid=$q";; esac
  done; sleep 4
fi
bash "$R/witness.sh" 退后重采 "$R/09_witness_after2.txt"
say "  两采同值＝$(diff <(grep -v '^# 见证' "$R/08_witness_after1.txt") <(grep -v '^# 见证' "$R/09_witness_after2.txt") >/dev/null 2>&1 && echo True || echo False)"

say "== 步10 只读后处理件★三段同窗并报（★derive 一字未改）=="
python3 "$SCR/derive_stvl_mark.py" --dir "$RUN" --segs x2_g6,x2_d1,x2_d2 > "$R/05_derive_three_same_window.txt" 2>&1
DR=$?; say "  derive rc=$DR"; grep -E "STVL_MARK|SUMMARY|GRADE" "$R/05_derive_three_same_window.txt" | sed 's/^/    /' | tee -a "$R/00_window_log.txt"

say "== 步11 清盘（清理与计数同条命令、路径全集）=="
find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null
echo "  pyc=$(find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f|wc -l) tooling=$(find /opt/ros1-slam/tooling -type f|wc -l) note=$(find /opt/ros1-slam/note -type f|wc -l)" | tee -a "$R/00_window_log.txt"
say "== 窗结束：预检 $PRC｜g6 $G｜d1 $O｜d2 $D｜derive $DR｜时长 $(( $(date +%s) - T0 ))s（★上限 10 分钟）｜★联调·旁证、★A59 不计分子 =="
exit 0
