#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
OUT=/work/diag_obs
mkdir -p $OUT
bash /work/flat2d_stop2.sh
rosparam delete /move_base/global_costmap >/dev/null 2>&1
rosparam delete /move_base/local_costmap >/dev/null 2>&1
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
rosparam set /mux/mux_config /work/mux_flat2d.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/diag_mux.log 2>&1 &
python3 $FLAT/scan_relay.py > /work/diag_relay.log 2>&1 &
rosparam set use_sim_time true
roslaunch $FLAT/flat2d_stack.launch > /work/diag_stack.log 2>&1 &
sleep 14
python3 - <<'PY'
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('dump_local', anonymous=True)
def dump(tag):
    got=[]
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, lambda m: got.append(m) if not got else None, queue_size=1)
    t0=rospy.Time.now().to_sec()
    while not got and rospy.Time.now().to_sec()-t0<10: rospy.sleep(0.1)
    m=got[0]; info=m.info; d=m.data
    ox,oy,res=info.origin.position.x,info.origin.position.y,info.resolution
    lines=[]
    for iy in range(int(info.height)-1,-1,-1):
        y=oy+(iy+0.5)*res
        if abs(y-int(y))>0.2 and False: pass
        row=''
        for ix in range(0,info.width,2):
            x=ox+(ix+0.5)*res
            v=d[iy*info.width+ix]
            row += '.' if v==0 else ('#' if v>=99 else '+')
        lines.append('%+06.2f %s'%(y,row))
    open('/work/diag_obs/%s.txt'%tag,'w').write('\n'.join(lines)+'\n')
    print('dump', tag, info.width, info.height, 'origin', ox, oy)
dump('baseline')
PY
# 注入方块 (2.0,0.0) 0.6x0.6
rosservice call /gazebo/spawn_sdf_model "{model_name: 'diag_block', xml: '<sdf version=\"1.6\"><model name=\"diag_block\"><static>true</static><pose>2.0 0.0 0.3 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.6 0.6 0.6</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.6 0.6 0.6</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}, reference_frame: 'world'}" >/dev/null 2>&1
sleep 4
python3 - <<'PY'
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('dump_local2', anonymous=True)
got=[]
rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, lambda m: got.append(m) if not got else None, queue_size=1)
t0=rospy.Time.now().to_sec()
while not got and rospy.Time.now().to_sec()-t0<10: rospy.sleep(0.1)
m=got[0]; info=m.info; d=m.data
ox,oy,res=info.origin.position.x,info.origin.position.y,info.resolution
lines=[]
for iy in range(int(info.height)-1,-1,-1):
    y=oy+(iy+0.5)*res
    row=''
    for ix in range(0,info.width,2):
        v=d[iy*info.width+ix]
        row += '.' if v==0 else ('#' if v>=99 else '+')
    lines.append('%+06.2f %s'%(y,row))
open('/work/diag_obs/with_block.txt','w').write('\n'.join(lines)+'\n')
print('dump with_block ok')
PY
rosservice call /gazebo/delete_model '{model_name: diag_block}' >/dev/null 2>&1
echo DIAG_DONE
