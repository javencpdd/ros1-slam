import json, time, rospy, subprocess
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
rospy.init_node('cancel_blind', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'cmd':[],'ack':None,'lstat':None}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,2), round(m.linear.x,3))))
rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: (st.update(ack=m.data), mark('ack', data=m.data)))
rospy.Subscriber('/v8/local/status', String, lambda m: st.update(lstat=m.data))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
cp=rospy.Publisher('/v8/bt/cancel',String,queue_size=1)
time.sleep(1.5)
def send(dx):
    p=st['pose'] or (1.1,-3.05)
    s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
    sp.publish(s); time.sleep(0.4)
    g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=p[0]+dx; g.pose.position.y=p[1]; g.pose.orientation.w=1.0
    gp.publish(g); return (p[0]+dx, p[1])
# C: 取消
goal=send(1.5); mark('goal_cancel', x=goal[0])
time.sleep(2.0); mark('moving_before_cancel', pose=st['pose'], cmd=st['cmd'][-1:])
c=String(); c.data='9:all:p4C'; cp.publish(c); tc=time.time()-T0; mark('cancel_sent')
time.sleep(3.0)
zero_at=None
for t,v in st['cmd']:
    if t>=tc and abs(v)<1e-9: zero_at=t; break
mark('cancel_result', ack=st['ack'], zero_after=round(zero_at-tc,2) if zero_at else None, pose=st['pose'])
# B: 运动中断感知
goal2=send(1.2); mark('goal_blind', x=goal2[0])
time.sleep(2.5); mark('moving_before_kill', pose=st['pose'], cmd=st['cmd'][-1:])
tk=time.time()-T0
subprocess.run(['rosnode','kill','/v8/3d/perception/perception'], capture_output=True); mark('perception_killed')
time.sleep(4.0)
zero_at2=None
for t,v in st['cmd']:
    if t>=tk and abs(v)<1e-9: zero_at2=t; break
mark('blind_result', zero_after_kill=round(zero_at2-tk,2) if zero_at2 else None, pose=st['pose'], lstat=(st['lstat'] or '')[:120])
nz=[c for c in st['cmd'] if abs(c[1])>1e-6]
mark('summary', nonzero=len(nz))
print(json.dumps({'events':ev}, ensure_ascii=False)[:1500])
