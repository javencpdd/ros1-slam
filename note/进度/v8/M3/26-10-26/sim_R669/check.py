import json, rospy
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
rospy.init_node("r668_checker", disable_signals=True)
rospy.wait_for_message("/clock", Clock, timeout=30)
msg = rospy.wait_for_message("/support_route", Path, timeout=30)
value = json.load(open("/work/map.json"))
boxes = {s["surface_id"]: s["footprint"] for s in value["surfaces"]}
ok, bad = True, []
if msg.header.frame_id != "map": ok=False; bad.append("frame")
if not msg.poses: ok=False; bad.append("empty")
if msg.header.stamp.to_sec() <= 0: ok=False; bad.append("stamp_not_sim")
for pose in msg.poses:
    p = pose.pose.position
    if not [s for s,b in boxes.items() if b["min_x"]<=p.x<=b["max_x"] and b["min_y"]<=p.y<=b["max_y"]]:
        ok=False; bad.append(("off_surface", p.x, p.y))
status = rospy.get_param("/support_route_status", {})
if not status.get("ok"): ok=False; bad.append("status")
models = rospy.wait_for_message("/gazebo/model_states", __import__("gazebo_msgs.msg", fromlist=["ModelStates"]).ModelStates, timeout=30)
names = sorted(models.name)
print(json.dumps({"ok": ok, "points": len(msg.poses), "frame": msg.header.frame_id,
                  "sim_stamp": round(msg.header.stamp.to_sec(),3), "models": names,
                  "status": status, "bad": bad[:5]}))
