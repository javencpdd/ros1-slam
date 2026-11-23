source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/p2/motion; mkdir -p $EV
rosnode kill -a >/dev/null 2>&1; sleep 3
for pid in $(ps -eo pid,args | awk '/gzserver/ && /lite3_gazebo_sim\/worlds/ {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 3
setsid roslaunch lite3_gazebo_sim app_world.launch gui:=false > $EV/gz6.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; rosservice list 2>/dev/null | grep -q '^/gazebo/set_model_state$' && break; done
setsid roslaunch /tmp/g2.launch > $EV/live7.log 2>&1 < /dev/null &
sleep 22
echo "nodes=$(rosnode list | grep -c .)"
timeout 3 rostopic echo -n1 /v8/local/pose 2>/dev/null | grep -E "^\s+(x|y):" | head -2
timeout 3 rostopic echo -n1 /v8/3d/perception/status | cut -c1-120
