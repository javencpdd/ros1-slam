#!/usr/bin/env bash
# 00_r422_window.sh —— ②R2 差分窗（窗号 R422；x86 侧、容器内跑）
# 目的＝把 derive_stvl_mark 的「归因＝未定源」定住：★同参数面逐字只摘掉 stvl_obstacle_layer 一枚，
#       与 R419 的 g6（四插件）段做四组＋逐格对比，取「痕迹消失／不消失」这一个二值。
# ★跑法照抄在册先例＝R419 win_driver_v3.sh 的启动序（★两帧地图发布器必须先于 costmap_2d_node，否则成本图对象不建＝v1/v2 三次尝试留名的教训）
# ★纪律：预检变体 rc≠0 ⇒ 不起窗；★只 kill 本窗 PID；★退净后必须重采见证；★探针与在册件一字不改；读数入 A59 旁证桶、★不进分子
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r422_R2差分窗
RUN="$R/run"
SCR=/opt/ros1-slam/note/进度/SCAN集成/设计/scripts
NS=x2_r422_d2
REFDIR=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r419_M3归因窗/run
mkdir -p "$RUN"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
PIDS=""; T0=$(date +%s)
cleanup(){ for p in $PIDS; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT

say "== 步0 预检变体当刻（在册预检本体一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "预检变体 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗、当场拒窗"; exit 7; }

say "== 步0b 起前见证（被试域＝容器 netns）=="
bash "$R/witness.sh" 起前 "$R/02_witness_before.txt"
cat "$R/02_witness_before.txt" | sed 's/^/    /' | tee -a "$R/00_window_log.txt" | tail -4

say "== 步1 差分参数面自证（★与 R419 g6 面只差两处＝plugins 摘 stvl ＋ 删 stvl 参数块）=="
{
  echo "# 差分面＝run/params_$NS.yaml（取 R419 params_x2_g6.yaml 逐字改）"
  echo "  行数：g6=$(wc -l < "$REFDIR/params_x2_g6.yaml") ‖ 差分面=$(wc -l < "$RUN/params_$NS.yaml")"
  echo "  plugins 现读枚数=$(grep -c '{name:' <(grep '^plugins:' "$RUN/params_$NS.yaml")) ‖ 含 stvl 字样=$(grep '^plugins:' "$RUN/params_$NS.yaml" | grep -c stvl)（★须 0）"
  echo "  全局参数前 11 行与 g6 逐字等=$(python3 -c "
import io
a=io.open('$REFDIR/params_x2_g6.yaml',encoding='utf-8').read().split(chr(10))[:11]
b=io.open('$RUN/params_$NS.yaml',encoding='utf-8').read().split(chr(10))[:11]
print('True' if a==b else 'False')")"
  echo "  sob_layer 块与文件尾逐字等=$(python3 -c "
import io
a=[l for l in io.open('$REFDIR/params_x2_g6.yaml',encoding='utf-8').read().split(chr(10))[53:] if l.strip()]
b=[l for l in io.open('$RUN/params_$NS.yaml',encoding='utf-8').read().split(chr(10))[14:] if l.strip()]
print('True' if a==b else 'False')")"
} > "$R/03_param_face.txt" 2>&1
cat "$R/03_param_face.txt" | tee -a "$R/00_window_log.txt"

say "== 步2 起本窗 master =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "  master_ready=$ok（等待 ${i}s）"
[ "$ok" != 1 ] && { say "★master 未起 ⇒ 不起窗"; exit 8; }

say "== 步3 装载差分面并回读（★只读回证，不写在册件）=="
rosparam load "$RUN/params_$NS.yaml" "/$NS/costmap" && say "  load_ok plugins 枚数=$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c 'name:')" || say "  load_FAIL"
rosparam get /$NS/costmap/plugins > "$RUN/plugins_readback.txt" 2>&1

say "== 步4 发布器先起（TF×2 ＋ 合成点云 ＋ ★两帧地图）再起 costmap 节点 =="
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"
rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$RUN/tf_r.log" 2>&1 & PIDS="$PIDS $!"
X2_CLOUD_TAG=$NS python3 "$SCR/x2_pub_cloud.py" > "$RUN/cloud_$NS.log" 2>&1 & PIDS="$PIDS $!"
X2_GAP=12 X2_TAG=$NS python3 "$SCR/x2_pub_two_frame.py" > "$RUN/pub_$NS.log" 2>&1 & PIDS="$PIDS $!"
sleep 4
rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & PIDS="$PIDS $!"
sleep 30

say "== 步5 话题与体素话题存在性（★去 stvl ⇒ 预期 stvl voxel 话题为 0 条）=="
rostopic list 2>/dev/null | sort > "$RUN/topics.txt"
say "  话题数=$(wc -l < "$RUN/topics.txt") costmap话题=$(grep -c "$NS/costmap/costmap$" "$RUN/topics.txt")"
grep -i "voxel" "$RUN/topics.txt" > "$RUN/voxel_topics.txt" 2>/dev/null
say "  含 voxel 话题数=$(wc -l < "$RUN/voxel_topics.txt")（★本段无 stvl ⇒ 须 0；R419 四插件段实测 1 条）"
{ echo "# 本段（去 stvl）含 voxel 字样话题逐字（★零条＝与 R419 的 1 条成对）"; cat "$RUN/voxel_topics.txt"; } > "$R/04_voxel_zero.txt" 2>&1

say "== 步6 采 cost 面原始件 =="
timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
say "  cost 件字节=$(stat -c%s "$RUN/cost_$NS.txt" 2>/dev/null) data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt")"

say "== 步7 只读后处理件两段并报（★derive 件一字未改）=="
python3 "$SCR/derive_stvl_mark.py" --dir "$RUN" --segs "$NS" > "$R/05_derive_d2.txt" 2>&1
D2=$?
python3 "$SCR/derive_stvl_mark.py" --dir "$REFDIR" --segs x2_g6 > "$R/06_derive_g6_ref.txt" 2>&1
D1=$?
say "  derive d2 rc=$D2 ‖ derive g6（R419 参照、同件同式复跑）rc=$D1"
grep -E "顶档|痕迹|归因|254|足印|桶名" "$R/05_derive_d2.txt" | sed 's/^/    d2‖ /' | tee -a "$R/00_window_log.txt"
grep -E "顶档|痕迹|归因|254|足印|桶名" "$R/06_derive_g6_ref.txt" | sed 's/^/    g6‖ /' | tee -a "$R/00_window_log.txt"

say "== 步8 逐格对比（足印 rows18-21 × cols29-32 ＋ 全图差异格数）=="
python3 "$R/_cmp_cells.py" > "$R/07_cell_compare.txt" 2>&1
CC=$?; say "  对比件 rc=$CC"
cat "$R/07_cell_compare.txt" | tee -a "$R/00_window_log.txt"

say "== 步9 退窗（只 kill 本窗 PID）＋退净后★重采见证 =="
cleanup; sleep 6
bash "$R/witness.sh" 退后首采 "$R/08_witness_after1.txt"
if grep -qE '五名计数=[1-9]' "$R/08_witness_after1.txt" || ss -lnt 2>/dev/null | grep -qE ':(11311|11312|11313)\b'; then
  say "  ★退净未达标 ⇒ 只收本窗 master holder 后重采"
  for q in $(ss -lntp 2>/dev/null | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
    cl=$(tr -d '\0' < /proc/$q/cmdline 2>/dev/null)
    case "$cl" in *rosmaster*|*roscore*) say "    收 master holder pid=$q"; kill -9 "$q" 2>/dev/null;; *) say "    ★跳过非 master holder pid=$q cmd=$cl";; esac
  done
  sleep 4
fi
bash "$R/witness.sh" 退后重采 "$R/09_witness_after2.txt"
say "  两采同值＝$(if diff <(grep -v '^# 见证' "$R/08_witness_after1.txt") <(grep -v '^# 见证' "$R/09_witness_after2.txt") >/dev/null 2>&1; then echo True; else echo False; fi)"

say "== 步10 清盘（清理与计数同条命令、路径写全集）=="
find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null
echo "  pyc=$(find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f|wc -l) tooling=$(find /opt/ros1-slam/tooling -type f|wc -l) note=$(find /opt/ros1-slam/note -type f|wc -l)" | tee -a "$R/00_window_log.txt"

say "== 窗结束：预检 rc=$PRC｜derive d2 rc=$D2 g6 rc=$D1｜时长 $(( $(date +%s) - T0 ))s（★上限 10 分钟）｜★档名＝联调·旁证、★A59 不计分子 =="
exit 0
