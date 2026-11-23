import json, time, rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
rospy.init_node('nav_diag', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'bt':None,'lstat':None,'cmd':[]}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/v8/bt/status', String, lambda m: st.update(bt=m.data))
rospy.Subscriber('/v8/local/status', String, lambda m: st.update(lstat=m.data))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,1), round(m.linear.x,2))))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
time.sleep(2)
p=st['pose'] or (0.0,-3.06)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); time.sleep(0.5)
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=p[0]+1.2; g.pose.position.y=p[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal_sent', x=p[0]+1.2, y=p[1])
for i in range(16):
    time.sleep(1.0)
    l=st['lstat']; b=st['bt']
    if i%4==0:
        mark('tick', t=i+1, pose=st['pose'], cmd=st['cmd'][-1:] , lstat=(l[:110] if l else None))
nz=[c for c in st['cmd'] if abs(c[1])>1e-6]
mark('summary', nonzero=len(nz), pose=st['pose'], lstat=(st['lstat'] or '')[:200], bt=(st['bt'] or '')[:200])
print(json.dumps({'events':ev}, ensure_ascii=False)[:1800])
