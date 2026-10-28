#!/bin/bash
# SC09 v5 单跑：重启 sc09 栈 → 就绪门 → 复位 → driver
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/ws/src/90_platform/v8_platform/calibration/flat2d
man=$1
tag=$(basename "$man" .json)
OUT=${V5_OUT:-/work/sc09_nli}
mkdir -p $OUT

bash /work/restart_sc09nli_stack.sh >/dev/null 2>&1
timeout 150 python3 /work/wait_ready.py 140 200 200 0.05 -2.5 -5.0
RC_READY=$?
echo "V5 $tag ready_rc=$RC_READY $(date +%H:%M:%S)"
if [ $RC_READY -ne 0 ]; then
  echo "V5 $tag ABORT not_ready"
  exit 10
fi

for m in sc09_block_g0 sc09_block_g1 sc09_block_g2 sc09_block_g3; do
  timeout 5 rosservice call /gazebo/delete_model "{model_name: $m}" >/dev/null 2>&1
done
ST=$(timeout 5 python3 -c "import json;d=json.load(open('$man'));s=d.get('start',[0.0,0.0]);print('%g %g'%(s[0],s[1]))")
set -- $ST
timeout 8 rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: $1, y: $2, z: 0.112}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" >/dev/null 2>&1
sleep 4
timeout 8 rosservice call /move_base/clear_costmaps >/dev/null 2>&1
timeout 330 python3 $FLAT/driver.py "$man" "$OUT/${tag}.json" > "$OUT/${tag}.log" 2>&1
RC=$?
echo "V5 $tag rc=$RC $(tail -1 $OUT/${tag}.log | cut -c1-400)"
for m in sc09_block_g0 sc09_block_g1 sc09_block_g2 sc09_block_g3; do
  timeout 5 rosservice call /gazebo/delete_model "{model_name: $m}" >/dev/null 2>&1
done
exit $RC
