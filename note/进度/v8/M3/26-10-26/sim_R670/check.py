import json, rospy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
rospy.init_node("r670_checker", disable_signals=True)
rospy.wait_for_message("/clock", Clock, timeout=30)
msg = rospy.wait_for_message("/support_route", Path, timeout=30)
rospy.wait_for_service("/make_plan", timeout=30)
def pose(x, y, z):
    p = PoseStamped(); p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
    p.pose.orientation.w = 1.0
    return p
req = GetPlanRequest(start=pose(0,0,0), goal=pose(1,0,0.35), tolerance=0.1)
resp = rospy.ServiceProxy("/make_plan", GetPlan)(req)
value = json.load(open("/work/map.json"))
boxes = {s["surface_id"]: s["footprint"] for s in value["surfaces"]}
check = lambda m: all([s for s,b in boxes.items() if b["min_x"]<=q.pose.position.x<=b["max_x"] and b["min_y"]<=q.pose.position.y<=b["max_y"]] for q in m.poses)
bad_req = GetPlanRequest(start=pose(0,0,0), goal=pose(99,99,9), tolerance=0.1)
bad_resp = rospy.ServiceProxy("/make_plan", GetPlan)(bad_req)
status = rospy.get_param("/support_route_status", {})
print(json.dumps({"ok": bool(resp.plan.poses) and check(resp.plan) and not bad_resp.plan.poses,
                  "service_points": len(resp.plan.poses), "topic_points": len(msg.poses),
                  "same_end": (resp.plan.poses[-1].pose.position.x == msg.poses[-1].pose.position.x),
                  "bad_plan_empty": not bad_resp.plan.poses, "status_after_bad": status, "bad": []}))
