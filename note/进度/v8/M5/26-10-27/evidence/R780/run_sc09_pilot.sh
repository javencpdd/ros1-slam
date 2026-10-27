#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
OUT=/work/sc09_pilot
mkdir -p $OUT
bash /work/flat2d_stop2.sh
rosparam delete /move_base/global_costmap >/dev/null 2>&1
rosparam delete /move_base/local_costmap >/dev/null 2>&1
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
rosparam set use_sim_time true
printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /lite3/supervisor/hold\nsources:\n  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}\n' > /work/mux_flat2d.yaml
rosparam set /mux/mux_config /work/mux_flat2d.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/sc09_mux.log 2>&1 &
python3 $FLAT/scan_relay.py > /work/sc09_relay.log 2>&1 &
roslaunch $FLAT/flat2d_stack_sense.launch > /work/sc09_stack.log 2>&1 &
sleep 14
# 1) 纯静态检查：不注入时走廊是否仍为空
python3 - <<'PY'
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('dump_g', anonymous=True)
got=[]
rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, lambda m: got.append(m) if not got else None, queue_size=1)
t0=rospy.Time.now().to_sec()
while not got and rospy.Time.now().to_sec()-t0<12: rospy.sleep(0.1)
m=got[0]; info=m.info; d=m.data
ox,oy,res=info.origin.position.x,info.origin.position.y,info.resolution
lines=[]
for iy in range(int(info.height)-1,-1,-1):
    y=oy+(iy+0.5)*res
    if abs(y)>1.6: continue
    row=''
    for ix in range(0,info.width,2):
        v=d[iy*info.width+ix]
        row += '.' if v==0 else ('#' if v>=99 else '+')
    lines.append('%+06.2f %s'%(y,row))
open('/work/sc09_pilot/costmap_noblock.txt','w').write('\n'.join(lines)+'\n')
print('DUMP_OK')
PY
# 2) SC09 pilot：目标 (6.2,2.2)，行进 1.2 m 后在前方注入 0.8 m 方块
python3 - <<'PY'
import json, pathlib
man = {'run_id': 'sc09_pilot_1', 'run_class': 'pilot', 'calibration_only': False,
       'scenario_id': 'SC09', 'seed': 0, 'repetition': 0, 'goal': [6.2, 2.2],
       'start': [0.0, 0.0],
       'criteria': {'goal_position_m': 0.30, 'min_clearance_m': 0.20, 'deadline_s': 120},
       'inject': {'at_progress_m': 1.2, 'model_name': 'sc09_block',
                  'size': [0.8, 0.8, 0.8], 'pose_xy_yaw': [3.4, 2.2, 0.0]}}
pathlib.Path('/work/sc09_pilot/sc09_pilot_1.json').write_text(json.dumps(man))
PY
timeout 200 python3 $FLAT/driver.py /work/sc09_pilot/sc09_pilot_1.json /work/sc09_pilot/sc09_pilot_1_res.json > /work/sc09_pilot/sc09_pilot_1.log 2>&1
echo "SC09 rc=$? $(tail -1 /work/sc09_pilot/sc09_pilot_1.log | cut -c1-260)"
rosservice call /gazebo/delete_model '{model_name: sc09_block}' >/dev/null 2>&1
echo SC09_PILOT_DONE
