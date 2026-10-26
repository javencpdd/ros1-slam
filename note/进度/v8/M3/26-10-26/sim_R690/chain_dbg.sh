source /opt/ros/noetic/setup.bash
[ -f /opt/ros1-slam/ws/devel/setup.bash ] && source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11345 GAZEBO_PLUGIN_PATH=/opt/ros1-slam/ws/devel/lib:/opt/ros/noetic/lib LD_LIBRARY_PATH=/opt/ros1-slam/ws/devel/lib:$LD_LIBRARY_PATH ROS_PACKAGE_PATH=/work/repo/ws/src:$ROS_PACKAGE_PATH
export PYTHONPATH=/work/adapters:/work/planner:/work/contracts:$PYTHONPATH
mkdir -p /work/ros /work/logs
roscore -p 11311 >/tmp/rc.log 2>&1 & sleep 6
gzserver --verbose -s libgazebo_ros_api_plugin.so /work/flat.world >/tmp/gz.log 2>&1 & sleep 8
xacro /work/repo/ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro > /work/robot.urdf 2>/dev/null
rosrun gazebo_ros spawn_model -urdf -file /work/robot.urdf -model lite3 -x 0 -y 0 -z 0.5 >/tmp/spawn.log 2>&1; sleep 3
printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /test/hold\nsources:\n  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}\n' > /work/mux.yaml
rosparam set /mux/mux_config /work/mux.yaml
printf 'path: [[2.0, 0.0]]\npose_topic: /test/pose\nout_topic: /follow/cmd_vel\n' > /work/f.yaml
rosparam load /work/f.yaml /follower
cat > /work/pose_bridge.py <<'P'
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
rospy.init_node('pose_bridge', disable_signals=True)
pub=rospy.Publisher('/test/pose', PoseStamped, queue_size=1)
def cb(msg):
    try: i=msg.name.index('lite3')
    except ValueError: return
    out=PoseStamped(); out.header.frame_id='map'; out.pose=msg.pose[i]; pub.publish(out)
rospy.Subscriber('/gazebo/model_states', ModelStates, cb); rospy.spin()
P
python3 /work/pose_bridge.py >/work/bridge.log 2>&1 & sleep 1
python3 /work/repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux >/work/mux.log 2>&1 & sleep 1
python3 /work/adapters/follower_node.py __name:=follower >/work/follower.log 2>&1 & sleep 1
python3 - <<'P'
import json, math, rospy, time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
rospy.init_node('watch2', disable_signals=True)
vals=[]
rospy.Subscriber('/follow/cmd_vel', Twist, lambda m: vals.append(round(m.linear.x,3)))
m=rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=15)
i=m.name.index('lite3'); x0=m.pose[i].position.x; t0=time.time()
while time.time()-t0 < 20:
    m=rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=15)
    if m.pose[i].position.x - x0 > 1.2: break
print("RESULT "+json.dumps({"dx":round(m.pose[i].position.x-x0,2),"follow_msgs":len(vals),"follow_last":vals[-3:]}))
P
echo "FOLLOW"; tail -3 /work/follower.log
echo "MUXOUT"; cat /work/mux.log | tail -8; echo "INFO"; rostopic info /cmd_vel 2>&1 | head -6
