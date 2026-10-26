source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
export PYTHONPATH=/work/adapters:/work/planner:/work/contracts:$PYTHONPATH
mkdir -p /work/ros /work/logs
roscore -p 11311 >/tmp/rc.log 2>&1 & sleep 6
cat > /work/mux.yaml <<Y
output_topic: /cmd_vel
output_rate_hz: 20
hold_topic: /test/hold
sources:
  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}
Y
rosparam set /mux/mux_config /work/mux.yaml
cat > /work/follower.yaml <<Y
path: [[1.0, 0.0], [1.0, 0.5]]
pose_topic: /test/pose
out_topic: /follow/cmd_vel
Y
rosparam load /work/follower.yaml /follower
python3 /work/adapters/follower_node.py __name:=follower >/work/node.log 2>&1 &
python3 /work/safety/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux >/work/mux.log 2>&1 &
python3 - <<'P'
import json, math, rospy, time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
rospy.init_node('driver', disable_signals=True)
pub=rospy.Publisher('/test/pose', PoseStamped, queue_size=1)
hold=rospy.Publisher('/test/hold', Bool, queue_size=1)
got=[]
rospy.Subscriber('/cmd_vel', Twist, lambda m: got.append((round(m.linear.x,3), round(m.angular.z,3))))
x=y=yaw=0.0; path=[(1.0,0.0),(1.0,0.5)]; idx=0
for step in range(400):
    p=PoseStamped(); p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    pub.publish(p); time.sleep(0.05)
    if step % 4 != 0: continue
    # 用最近一次 /cmd_vel 推进运动学
    if not got: continue
    v,w = got[-1]
    x += v*math.cos(yaw)*0.2; y += v*math.sin(yaw)*0.2; yaw += w*0.2
    if idx < len(path) and math.hypot(x-path[idx][0], y-path[idx][1]) < 0.08: idx += 1
moved = math.hypot(x, y)
hold.publish(Bool(True)); time.sleep(1.0)
tail = got[-1] if got else None
print("NODE "+open("/work/node.log").read()[-260:].replace("\n"," | "))
print("MUX "+open("/work/mux.log").read()[-260:].replace("\n"," | "))
print("RESULT "+json.dumps({"twists":len(got),"moved":round(moved,2),"waypoints":idx,
 "final_twist":tail,"zero_after_hold": tail == (0.0,0.0)}))
P
