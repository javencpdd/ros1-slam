import json, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import GetPathAction, GetPathGoal
rospy.init_node("r674_checker", disable_signals=True)
c=actionlib.SimpleActionClient("/move_base_flex/get_path", GetPathAction)
up=c.wait_for_server(rospy.Duration(20))
def pose(x,y,z):
    p=PoseStamped(); p.header.frame_id="map"
    p.pose.position.x,p.pose.position.y,p.pose.position.z=x,y,z
    p.pose.orientation.w=1.0; return p
out={"server_up": up}
if up:
    c.send_goal(GetPathGoal(target_pose=pose(1,0,0.35), tolerance=0.1, use_start_pose=True, start_pose=pose(0,0,0)))
    c.wait_for_result(rospy.Duration(20)); r=c.get_result()
    c.send_goal(GetPathGoal(target_pose=pose(99,99,9), tolerance=0.1, use_start_pose=True, start_pose=pose(0,0,0)))
    c.wait_for_result(rospy.Duration(20)); rb=c.get_result()
    out.update(ok=bool(r and r.path.poses) and bool(rb and not rb.path.poses),
               action_points=len(r.path.poses) if r else 0,
               outcome=getattr(r,"outcome",None),
               bad_outcome=getattr(rb,"outcome",None),
               bad_path_empty=bool(rb and not rb.path.poses),
               bad_message=(getattr(rb,"message","")[:48]))
json.dump(out, open("/work/check.out","w"), ensure_ascii=False)
print(json.dumps(out, ensure_ascii=False))
