#!/usr/bin/env bash
# 00_attr_window.sh —— stvl 归因旁证窗驱动（x86 侧，容器内跑；★联调·旁证、★不入分子、★不开判据窗）
# 工单真身＝note/进度/SCAN集成/证据/log/raw/r383_clause1与M4裁定批/01_裁定_clause1与M4.md §2.2（R384 入册）
# ★纪律：预检变体 rc≠0 ⇒ 不起窗；★不 kill 非本件进程；★探针与在册件一字不改；跑完清 __pycache__（路径写全集）且与计数同条命令（坑67②）
set -u
cd /opt/ros1-slam
H=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r388_stvl归因窗第二遍
mkdir -p "$H"
set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash; set -u
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$H/00_window_log.txt"; }

say "== 步0 预检变体当刻（在册预检件一字未动，走 x86 变体）=="
bash scripts/verify/v_window_preflight_x86_variant.sh > "$H/01_preflight.txt" 2>&1
PRC=$?
say "预检变体 rc=$PRC"
if [ "$PRC" != 0 ]; then say "★预检未过 ⇒ 按纪律不起窗、当场拒窗"; exit 7; fi

say "== 步1 基线复跑段 x2_g6（在册探针 X2_SEGS 单段）＋步2 R1 旁路采样（同窗并发）=="
X2_RUN="$H" X2_SEGS="x2_g6" bash note/进度/SCAN集成/设计/scripts/probe_x2_fourcells.sh > "$H/02_probe_g6.txt" 2>&1 &
P=$!
for i in $(seq 1 45); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
if rosnode list >/dev/null 2>&1; then
  say "master 在位 ⇒ R1 旁路采样开跑"
  { echo "# rostopic list（容器 $(date '+%F %T')）"; rostopic list; } > "$H/03_topics_x2_g6.txt" 2>&1
  CAND="/x2_g6/costmap/stvl_obstacle_layer/voxel_grid"
  echo "候选话题命中计数＝$(grep -c "$CAND" "$H/03_topics_x2_g6.txt")" | tee -a "$H/00_window_log.txt"
  say "含 stvl／voxel 字样的话题行（逐字留底）："
  grep -iE 'stvl|voxel' "$H/03_topics_x2_g6.txt" | tee -a "$H/00_window_log.txt" | tee -a "$H/04_voxel_topic_hits.txt" || echo "  （无）"
  timeout 30 rostopic echo -n 1 "$CAND" > "$H/05_voxel_echo.txt" 2>&1
  say "R1 采样 rc=$?（★话题不存在则此件为空／超时，照实报、★不反推未装载）"
else
  say "★roscore 45s 未起 ⇒ 前置缺失（★不硬采样）"
fi
wait $P; WRC=$?
say "基线段探针 rc=$WRC"

say "== 步3 差分段 x2_d1（去 sob_layer、留 stvl）=="
ABLATE_SEG=x2_d1 X2_RUN="$H" bash note/进度/SCAN集成/设计/scripts/x2_ablate_stvl.sh > "$H/06_ablate_d1.txt" 2>&1
D1=$?; say "x2_d1 rc=$D1"

say "== 步4 差分段 x2_d2（去 stvl_obstacle_layer、留 sob）=="
ABLATE_SEG=x2_d2 X2_RUN="$H" bash note/进度/SCAN集成/设计/scripts/x2_ablate_stvl.sh > "$H/07_ablate_d2.txt" 2>&1
D2=$?; say "x2_d2 rc=$D2"

say "== 步5 退净核 =="
{
  echo "# 退净形状（容器 $(date '+%F %T')）"
  echo "  pgrep gzserver=$(pgrep -x gzserver|wc -l) gzclient=$(pgrep -x gzclient|wc -l) roslaunch=$(pgrep -x roslaunch|wc -l) roscore=$(pgrep -x roscore|wc -l) rosmaster=$(pgrep -x rosmaster|wc -l) costmap_2d_node=$(pgrep -x costmap_2d_node|wc -l)"
  echo "  TCP 1131x=$(ss -lnt|grep -cE ':1131[1-9]') UDP 1131x=$(ss -lnu|grep -cE ':1131[1-9]') TCP 11345/11355=$(ss -lnt|grep -cE ':1134[5]|:1135[5]')"
} > "$H/08_exitclean.txt"
cat "$H/08_exitclean.txt" | tee -a "$H/00_window_log.txt"

say "== 步6 只读复算（derive 件三段并报；★该件不改在册件、★不自判归因之外的事）=="
python3 note/进度/SCAN集成/设计/scripts/derive_stvl_mark.py --dir "$H" --segs x2_g6,x2_d1,x2_d2 > "$H/09_derive_three.txt" 2>&1
say "derive rc=$?"
cat "$H/09_derive_three.txt" | tee -a "$H/00_window_log.txt"

find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null
echo "  ws_src=$(find /opt/ros1-slam/ws/src -type f|wc -l) pyc=$(find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name '*.pyc' -not -path '*/.Trash-0/*'|wc -l)" | tee -a "$H/00_window_log.txt"
say "== 窗结束：基线段 rc=$WRC｜d1 rc=$D1｜d2 rc=$D2｜★顶档判定见 09_derive_three.txt（★不声称转正、★不入分子）=="
exit 0
