import json, time, rospy, os, signal, subprocess
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
rospy.init_node('p4_abc', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'cmd':[],'ack':None,'lok':None,'obst':0}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,2), round(m.linear.x,3), round(m.linear.y,3))))
rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: (st.update(ack=m.data), mark('ack',data=m.data)))
rospy.Subscriber('/v8/local/localization_ok', String, lambda m: None)
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
cp=rospy.Publisher('/v8/bt/cancel',String,queue_size=1)
time.sleep(2)
p=st['pose'] or (0.0,-3.06)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); mark('start_sent', x=p[0], y=p[1]); time.sleep(0.6)
goal=(p[0]+1.2, p[1])
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=goal[0]; g.pose.position.y=goal[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal_sent', x=goal[0], y=goal[1])
# A: 等到达或 45s
arrived=None; t_end=time.time()+45
while time.time()<t_end:
    if st['pose'] and abs(st['pose'][0]-goal[0])<0.35 and abs(st['pose'][1]-goal[1])<0.35:
        arrived=st['pose']; break
    time.sleep(0.2)
mark('A_result', arrived=arrived, pose=st['pose'], n_cmd=len(st['cmd']))
nz=[c for c in st['cmd'] if abs(c[1])>1e-6 or abs(c[2])>1e-6]
mark('A_cmd', nonzero=len(nz), first=nz[:3], last=nz[-3:] if nz else [])
out={'events':ev,'ack':st['ack'],'pose':st['pose'],'arrived':arrived}
print(json.dumps(out, ensure_ascii=False))
