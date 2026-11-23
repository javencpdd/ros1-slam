import json, time, rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
rospy.init_node('p4_term', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'bt':None,'cmd':[]}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/v8/bt/status', String, lambda m: st.update(bt=m.data))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,2), round(m.linear.x,2))))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
time.sleep(2)
p=st['pose'] or (0.0,-3.06)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); time.sleep(0.6)
goal=(p[0]+1.2, p[1])
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=goal[0]; g.pose.position.y=goal[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal_sent', x=goal[0], y=goal[1])
terminal=None; end=time.time()+60
while time.time()<end:
    b=st['bt']
    if b:
        d=json.loads(b)
        if d.get('outcome'):
            terminal=d; break
    time.sleep(0.2)
time.sleep(1.5)
nz=[c for c in st['cmd'] if abs(c[1])>1e-6]
mark('terminal', outcome=(terminal or {}).get('outcome'), root=(terminal or {}).get('root'),
     pose=st['pose'], dist=round(((st['pose'][0]-goal[0])**2+(st['pose'][1]-goal[1])**2)**0.5,3) if st['pose'] else None,
     nonzero_cmd=len(nz), cmd_tail=st['cmd'][-2:])
print(json.dumps({'events':ev,'terminal':terminal,'final_pose':st['pose'],'goal':goal}, ensure_ascii=False)[:1400])
