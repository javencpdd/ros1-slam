import json, math, rospy
rospy.init_node("r666_checker", disable_signals=True)
from nav_msgs.msg import Path
msg = rospy.wait_for_message('/support_route', Path, timeout=20)
value = json.load(open('/work/map.json'))
boxes = {s['surface_id']: s['footprint'] for s in value['surfaces']}
ok, bad = True, []
if msg.header.frame_id != 'map': ok=False; bad.append('frame')
if not msg.poses: ok=False; bad.append('empty')
for pose in msg.poses:
    p = pose.pose.position
    hit = [sid for sid,b in boxes.items()
           if b['min_x'] <= p.x <= b['max_x'] and b['min_y'] <= p.y <= b['max_y']]
    if not hit: ok=False; bad.append(('off_surface', p.x, p.y))
status = rospy.get_param('/support_route_status', {})
if not status.get('ok'): ok=False; bad.append('status')
if status.get('segments') and len(status['segments']) < 1: ok=False; bad.append('segments')
print(json.dumps({'ok': ok, 'points': len(msg.poses), 'frame': msg.header.frame_id,
                  'status': status, 'bad': bad[:5]}))
