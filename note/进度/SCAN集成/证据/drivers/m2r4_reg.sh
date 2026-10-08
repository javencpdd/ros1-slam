#!/usr/bin/env bash
# m2r4_reg.sh —— M2R4 补齐跑（小美 M2R3 复核条件：fresh 连正 5 次，其中 >=1 次取龄推到 sim 时钟 >=21.5s）
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
P=/tmp/m2r4_out; mkdir -p $P

snap_foxy() { # 小美批评④：回归必须有起始快照可比对
  echo "### foxy lstart $1 $(date '+%T')"
  ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null \
    | tee "$P/foxy_$1.txt"
  echo "  计数=$(wc -l < "$P/foxy_$1.txt")"
}
ports() {
  echo "  TCP 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11314=$(ss -lnt 2>/dev/null | grep -c ':11314')"
  echo "  UDP 11312=$(ss -lnu 2>/dev/null | grep -c ':11312') 11313=$(ss -lnu 2>/dev/null | grep -c ':11313') 11314=$(ss -lnu 2>/dev/null | grep -c ':11314')"
}

run() { # $1=标签 $2=TF_SOURCE $3=G5_DELAY_SIM
  local tag=$1 ts=$2 dly=$3
  echo "##### $tag  TF_SOURCE=[$ts] G5_DELAY_SIM=[$dly]  $(date '+%T') #####"
  snap_foxy "start_$tag"
  TF_SOURCE="$ts" G5_DELAY_SIM="$dly" bash verify/m2_smoke.sh stageB > "$P/$tag.log" 2>&1
  local rc=$?
  snap_foxy "end_$tag"
  echo "  rc=$rc"
  echo "  取龄行: $(grep -oE 'SMOKE (G5_DELAY_SIM|gate_tf_fresh[A-Za-z0-9_]*)=[^|]*' "$P/$tag.log" | tr '\n' '|')"
  echo "  主判据: $(grep -oE 'teb_state=[0-9]+ dt=[0-9.]+s dev_odom=[0-9.]+ dev_gt=[0-9.]+' "$P/$tag.log" | head -1)"
  echo "  诊断:   $(grep -oE 'DIAG_G5c1.*' "$P/$tag.log" | head -1)"
  echo "  E3:     $(grep -cE 'SMOKE E3' "$P/$tag.log") 行"
  grep -E 'SMOKE E3' "$P/$tag.log" | sed 's/^/    /'
  echo "  判定行: $(grep -cE '^\[PASS\]' "$P/$tag.log") PASS / $(grep -cE '^\[FAIL\]' "$P/$tag.log") FAIL"
  ports
  sleep 4
}

run fresh4 "" ""
run fresh5 "" "21.5"
echo "##### 退净核查 $(date '+%T') #####"
echo "残留:[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex' | tr '\n' ' ')]"
ports
snap_foxy "final"
echo "ALL_DONE $(date '+%T')"
