import json, rospy, actionlib, time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import GetPathAction, GetPathGoal
t0=time.time()
rospy.init_node("r676_checker", disable_signals=True)
got_clock = False
try:
    rospy.wait_for_message("/clock", Clock, timeout=40); got_clock=True
except Exception: pass
c=actionlib.SimpleActionClient("/move_base_flex/get_path", GetPathAction)
up=c.wait_for_server(rospy.Duration(30))
def pose(x,y,z):
    p=PoseStamped(); p.header.frame_id="map"
    p.pose.position.x,p.pose.position.y,p.pose.position.z=x,y,z
    p.pose.orientation.w=1.0; return p
out={"clock":got_clock,"server_up":up,"wall_s":round(time.time()-t0,1)}
if up:
    c.send_goal(GetPathGoal(target_pose=pose(1,0,0.35), tolerance=0.1, use_start_pose=True, start_pose=pose(0,0,0)))
    c.wait_for_result(rospy.Duration(30)); r=c.get_result()
    out.update(points=len(r.path.poses) if r else 0, outcome=getattr(r,"outcome",None))
print("RESULT "+json.dumps(out))
