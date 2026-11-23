import json, sys, time, rospy, subprocess
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

mode = sys.argv[1] if len(sys.argv) > 1 else 'cancel'
rospy.init_node('motion_safety_'+mode, anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)

st={'pose':None,'cmd':0.0,'last_cmd_at':0.0,'ack':None,'lstat':None,'cloud_stamp':None,'cloud_at':None,'obst_at':None}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(m.pose.position.x, m.pose.position.y)))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st.update(cmd=m.linear.x, last_cmd_at=time.time()-T0))
rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: (st.update(ack=m.data), mark('ack', data=m.data)))
rospy.Subscriber('/v8/local/status', String, lambda m: st.update(lstat=m.data))
rospy.Subscriber('/v8/local/obstacles', String, lambda m: st.update(obst_at=time.time()-T0), queue_size=5)
rospy.Subscriber('/cloud_registered', PointCloud2,
                 lambda m: st.update(cloud_stamp=m.header.stamp.to_sec(), cloud_at=time.time()-T0), queue_size=1)
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
cp=rospy.Publisher('/v8/bt/cancel',String,queue_size=1)
time.sleep(2)
p=st['pose'] or (0.0,-3.06)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'
s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); time.sleep(0.6)
goal=(p[0]+1.8, p[1])
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'
g.pose.position.x=goal[0]; g.pose.position.y=goal[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal', x=goal[0], y=goal[1])

# 先确认确实在运动：连续 3 次 |vx|>0.1，且相邻间隔 ≥0.2 s
hits=0; t_motion=None; end=time.time()+25
while time.time()<end:
    if abs(st['cmd'])>0.1 and (time.time()-T0)-st['last_cmd_at']<0.2:
        hits+=1
        if hits>=3: t_motion=time.time()-T0; break
    else:
        hits=0
    time.sleep(0.2)
if not t_motion:
    mark('no_motion', pose=st['pose'], cmd=st['cmd'])
    print(json.dumps({'mode':mode,'result':'NO_MOTION','events':ev}, ensure_ascii=False)); raise SystemExit(1)
mark('motion_confirmed', t=t_motion, cmd=st['cmd'], pose=st['pose'])
pose_before = st['pose']; t_trigger=time.time()-T0
if mode=='cancel':
    c=String(); c.data='21:all:msA'; cp.publish(c)
else:
    subprocess.run(['rosnode','kill','/v8/3d/perception/perception'], capture_output=True)
mark('trigger', mode=mode, t=t_trigger)

t_zero=None; t_stale=None; end=time.time()+12
while time.time()<end:
    if t_zero is None and abs(st['cmd'])<1e-9 and (time.time()-T0) > t_trigger:
        t_zero=time.time()-T0
    l=st['lstat'] or ''
    if t_stale is None and 'E_LOCAL_OBSTACLES_STALE' in l:
        t_stale=time.time()-T0
    if t_zero is not None and (time.time()-T0) > t_zero+2.0: break
    time.sleep(0.05)
disp = None
if pose_before and st['pose']:
    disp = round(((st['pose'][0]-pose_before[0])**2+(st['pose'][1]-pose_before[1])**2)**0.5, 3)
mark('result', ack=st['ack'], last_obst_at=st['obst_at'],
     net_stale_s=round(t_stale-st['obst_at'],2) if (t_stale and st['obst_at']) else None,
     net_zero_s=round(t_zero-st['obst_at'],2) if (t_zero and st['obst_at']) else None, t_zero=round(t_zero,2) if t_zero else None,
     latency_zero=round(t_zero-t_trigger,2) if t_zero else None,
     t_stale=round(t_stale,2) if t_stale else None,
     latency_stale=round(t_stale-t_trigger,2) if t_stale else None,
     last_cloud_at=st['cloud_at'], pose_before=[round(v,3) for v in pose_before],
     pose_after=[round(v,3) for v in st['pose']], coast_m=disp)
print(json.dumps({'mode':mode,'events':ev}, ensure_ascii=False)[:1600])
