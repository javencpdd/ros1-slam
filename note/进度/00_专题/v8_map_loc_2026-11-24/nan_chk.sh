set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz14.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
python3 - <<PY
import rospy, math, struct, time
from sensor_msgs.msg import PointCloud2
rospy.init_node("nanchk", anonymous=True, disable_signals=True)
box={}
rospy.Subscriber("/rslidar_points", PointCloud2, lambda m: box.update(m=m), queue_size=1)
t=time.time()
while time.time()-t<12 and "m" not in box: time.sleep(0.2)
if "m" not in box: print("NO_CLOUD"); raise SystemExit
m=box["m"]; n=m.width*m.height; step=m.point_step; data=m.data
nan=zero=0; lim=min(n,20000)
for i in range(lim):
    x,y,z=struct.unpack_from("fff", data, i*step)
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)): nan+=1
    elif x==0 and y==0 and z==0: zero+=1
print("points=%d checked=%d NaN=%d zero=%d (%.1f%%)" % (n, lim, nan, zero, 100.0*(nan+zero)/lim))
PY
