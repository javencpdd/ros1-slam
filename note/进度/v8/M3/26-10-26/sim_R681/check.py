import json, rospy
from nav_msgs.msg import Path
rospy.init_node("r681_checker", disable_signals=True)
msg = rospy.wait_for_message("/support_route", Path, timeout=30)
value = json.load(open("/work/map.json"))
z = {s["surface_id"]: s["z_reference"] for s in value["surfaces"]}
boxes = {s["surface_id"]: s["footprint"] for s in value["surfaces"]}
ok, bad = True, []
if not msg.poses: ok=False; bad.append("empty")
for pose in msg.poses:
    p = pose.pose.position
    hit = [s for s,b in boxes.items() if b["min_x"]<=p.x<=b["max_x"] and b["min_y"]<=p.y<=b["max_y"]]
    if not hit: ok=False; bad.append(("off_surface", round(p.x,2)))
    elif all(abs(p.z - z[s]) > 0.06 for s in hit): ok=False; bad.append(("wrong_height", round(p.z,2)))
status = rospy.get_param("/support_route_status", {})
if not status.get("ok"): ok=False; bad.append("status")
print("RESULT "+json.dumps({"ok":ok,"points":len(msg.poses),
 "segments":len(status.get("segments",[])),"transitions":len(status.get("transitions",[])),
 "length":status.get("length"),"bad":bad[:4]}))
