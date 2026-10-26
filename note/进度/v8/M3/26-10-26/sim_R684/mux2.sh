source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
mkdir -p /work/ros /work/logs
roscore -p 11311 >/tmp/rc.log 2>&1 & sleep 6
cat > /work/mux2.yaml <<Y
output_topic: /cmd_vel
output_rate_hz: 20
hold_topic: /test/hold
sources:
  - {name: high, topic: /src/high, priority: 1, timeout_s: 0.5}
  - {name: low,  topic: /src/low,  priority: 2, timeout_s: 0.5}
Y
rosparam set /mux/mux_config /work/mux2.yaml
python3 /work/safety/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux >/work/mux.log 2>&1 & sleep 2
python3 - <<'P'
import json, rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
rospy.init_node('arb', disable_signals=True)
hi=rospy.Publisher('/src/high', Twist, queue_size=1); lo=rospy.Publisher('/src/low', Twist, queue_size=1)
got=[]
rospy.Subscriber('/cmd_vel', Twist, lambda m: got.append(round(m.linear.x,3)))
def ttl(x):
    m=Twist(); m.linear.x=x; return m
def sample(sec, publish_hi=True, publish_lo=False, hi_val=0.3, lo_val=0.1):
    end=time.time()+sec
    while time.time()<end:
        if publish_hi: hi.publish(ttl(hi_val))
        if publish_lo: lo.publish(ttl(lo_val))
        time.sleep(0.1)
sample(2.0, True, True, 0.3, 0.1); a=got[-3:]
sample(1.5, False, True, 0.3, 0.1); b=got[-3:]
rospy.Publisher('/test/hold', Bool, queue_size=1).publish(Bool(True)); time.sleep(0.8); c=got[-2:]
ok = all(abs(v-0.3)<1e-6 for v in a) and all(abs(v-0.1)<1e-6 for v in b) and all(v==0.0 for v in c)
print("RESULT "+json.dumps({"priority_wins":a,"fallback_to_low":b,"hold":c,"ok":ok}))
P
