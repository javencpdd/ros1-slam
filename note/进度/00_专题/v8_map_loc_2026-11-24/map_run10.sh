set -eu
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
SAMPLE=/home/ros1-slam/data/maps/lite3.pcd
OUT=${1:-}
[ -n "$OUT" ] || { echo "FAIL: 必须显式给输出路径"; exit 2; }
case "$OUT" in "$SAMPLE"|*/lite3.pcd) echo "FAIL: 拒绝写入样张 $SAMPLE"; exit 3;; esac
[ "$(readlink -f "$(dirname "$OUT")")" = "$(readlink -f /home/ros1-slam/data/maps)" ] && { echo "FAIL: 输出目录与样张同目录"; exit 4; }
mkdir -p "$(dirname "$OUT")"; [ -w "$(dirname "$OUT")" ] || { echo "FAIL: 输出目录不可写"; exit 5; }
echo "路径自检通过: $OUT"; sha0=$(sha256sum $SAMPLE | awk '{print $1}'); rm -f "$OUT"
EV=/tmp/v8map; mkdir -p $EV; : > $EV/traj.csv
rosnode kill -a >/dev/null 2>&1 || true; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null || true; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz9.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
rosparam load $(rospack find lite3_slam)/config/faster_lio_outdoor.yaml /
rosparam set use_sim_time true
rosparam set /faster_lio_online/pcd_save/pcd_save_en true
rosparam set /faster_lio_online/pcd_save/interval -1
rosparam set /faster_lio_online/pcd_save/path "$OUT"
echo "pre_set=$(rosparam get /faster_lio_online/pcd_save/path)"
setsid rosrun faster_lio run_mapping_online __name:=faster_lio_online > $EV/lio7.log 2>&1 < /dev/null &
# 轨迹采集（真值 vs 里程计）
( for i in $(seq 1 90); do
    gt=$(timeout 1 rostopic echo -n1 /sim/odom_gt 2>/dev/null | grep -A3 position | grep -E "^ +x:|^ +y:" | awk '{print $2}' | tr '\n' ' ')
    lo=$(timeout 1 rostopic echo -n1 /Odometry 2>/dev/null | grep -A3 "position:" | grep -E "^ +x:|^ +y:" | awk '{print $2}' | tr '\n' ' ')
    echo "$(date +%s.%N) $gt | $lo" >> $EV/traj.csv
  done ) &
sleep 10
timeout 28 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.45}, angular: {z: 0.0}}" >/dev/null 2>&1
timeout 14 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.9}}" >/dev/null 2>&1
timeout 24 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.45}, angular: {z: 0.0}}" >/dev/null 2>&1
timeout 14 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.9}}" >/dev/null 2>&1
timeout 20 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.45}, angular: {z: 0.0}}" >/dev/null 2>&1
rosnode kill /faster_lio_online >/dev/null 2>&1 || true; sleep 10
echo "out=$(ls -lh $OUT 2>/dev/null | awk '{print $5}')"
sha1=$(sha256sum $SAMPLE | awk '{print $1}'); [ "$sha0" = "$sha1" ] && echo "样张未变 ✓" || echo "样张被改动 ✗"
grep -oE "saved to [^ ]+" $EV/lio7.log | tail -1
