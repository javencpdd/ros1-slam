#!/usr/bin/env bash
# b1_retake.sh —— 批1（F-D）11 条复跑清单驱动（小张 fd 矩阵 §10.4 批1 清单）
# ①stageA(post_a 断言随跑) ②stageB 五连含≥1 加压 ③DWA 三连+teb 对照 ④SCAN P1 ⑤SCAN P2
# ⑥共同门禁 ⑦30_3rd 基线双条件 ⑧样张 sha ⑨foxy 起终成对
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
O=/tmp/b1_out; mkdir -p $O
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
snap(){ ps -o pid,user,lstart,comm --no-headers -p 115711,115792,115795,115797,115799,180237 > $O/foxy_$1.txt 2>/dev/null; echo "  foxy[$1] 计数=$(wc -l < $O/foxy_$1.txt)"; }
echo "===== 0) 被测件与 harness 指纹 ====="; date '+%F %T'
sha256sum urdf/lite3_planar.urdf.xacro verify/m2_smoke.sh verify/v_pose_source.sh \
  /home/ros1-slam/ws/src/40_navigation/lite3_navigation/verify/dwa_recheck.py \
  /home/ros1-slam/ws/src/40_navigation/lite3_scan/launch/70_scan.launch | tee $O/sha_inputs.txt
snap start
echo "===== 1) stageA（含 post_a 位姿源断言）====="
TF_SOURCE="" bash verify/m2_smoke.sh stageA > $O/stageA.log 2>&1; echo "  stageA rc=$?"
grep -E "^\[posesrc\]|pose_src|POSE SOURCE|drift=|rtf_motion|^\[PASS\]|^\[FAIL\]" $O/stageA.log | sed 's/^/    /' | tail -14
echo "===== 2) stageB 五连（第 3 跑加压 62s）====="
for i in 1 2 3 4 5; do
  d=""; [ $i = 3 ] && d=62
  TF_SOURCE="" G5_DELAY_SIM="$d" bash verify/m2_smoke.sh stageB > $O/sB$i.log 2>&1; rc=$?
  echo "  sB$i delay=[$d] rc=$rc | $(grep -oE 'teb_state=[0-9]+ dt=[0-9.]+s dev_odom=[0-9.]+' $O/sB$i.log | head -1) | press=$(grep -oE 'gate_press_late=[A-Z]+' $O/sB$i.log | head -1) | 判定 $(grep -cE '^\[PASS\]' $O/sB$i.log)P/$(grep -cE '^\[FAIL\]' $O/sB$i.log)F"
  grep -E "^\[exitclean\].*(命中|port)" $O/sB$i.log | sed 's/^/      /'
  sleep 3
done
echo "===== 3) DWA 三连 ＋ teb 对照（roadmap #4 撤销的可复现性）====="
cd /home/ros1-slam
export DWA_OUT=/tmp/b1_out/dwa
bash ws/src/40_navigation/lite3_navigation/verify/v_dwa_recheck.sh 3 > $O/dwa.log 2>&1; echo "  dwa rc=$?"
grep -E "loaded|rc=|VERDICT|DWA RECHECK|get_path attempt1|exe_path outcome" $O/dwa.log | sed 's/^/    /' | tail -12
echo "===== 4) SCAN P1 ====="
bash ws/src/40_navigation/lite3_scan/verify/v_scan_p1.sh > $O/p1.log 2>&1; echo "  p1 rc=$?"; grep -E "^\[(PASS|FAIL)\]|SCAN P1 VERDICT|未出" $O/p1.log | tail -8 | sed 's/^/    /'
echo "===== 5) SCAN P2 ====="
bash ws/src/40_navigation/lite3_scan/verify/v_scan_p2.sh > $O/p2.log 2>&1; echo "  p2 rc=$?"; grep -E "^\[(PASS|FAIL)\]|VERDICT|endpoint_err|react_s" $O/p2.log | tail -10 | sed 's/^/    /'
echo "===== 6) 30_3rd 基线双条件 ＋ 7) 共同门禁 ====="
(cd ws/src/30_3rd && sha256sum -c /home/ros1-slam/note/进度/SCAN集成/证据/scan_src_baseline_49.sha256 > /tmp/b1_out/base.txt 2>&1; echo "  非OK行数=$(grep -vc ': OK$' /tmp/b1_out/base.txt)  find 计数=$(find . -type f | wc -l)")
bash scripts/verify/v3_isolation.sh > $O/gate.log 2>&1; echo "  门禁 EXIT=$?"; tail -2 $O/gate.log | sed 's/^/    /'
echo "===== 8) 样张与退净 ====="
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee $O/sample_sha.txt
rosnode kill -a >/dev/null 2>&1; sleep 3; pkill -f gzserver >/dev/null 2>&1; pkill -f run_mapping_online >/dev/null 2>&1
pkill -f "roslaunch.*mode_sim" >/dev/null 2>&1; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; pkill -f move_base_flex >/dev/null 2>&1; sleep 5
echo "  残留=[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex|run_mapping_online' | tr '\n' ' ')]"
echo "  端口: 11311=$(ss -lnt 2>/dev/null | grep -c ':11311') 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313')"
snap end
echo "B1_DONE $(date '+%T')"
