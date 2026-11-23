import json, time, rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
rospy.init_node('p4_obst', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'obst':None,'bt':None,'cmd':[]}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/v8/local/obstacles', String, lambda m: st.update(obst=m.data[:200]))
rospy.Subscriber('/v8/bt/status', String, lambda m: st.update(bt=m.data[:260]))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append(round(m.linear.x,2)))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
time.sleep(2)
mark('obst_before', data=st['obst'])
# 在 ROI 内放一个真实障碍（Gazebo 模型，雷达会打到）
sdf=('<sdf version="1.6"><model name="p4_box"><static>true</static><link name="link">'
     '<collision name="c"><geometry><box><size>0.4 0.4 0.8</size></box></geometry></collision>'
     '<visual name="v"><geometry><box><size>0.4 0.4 0.8</size></box></geometry>'
     '<material><ambient>0.8 0.2 0.2 1</ambient></material></visual></link></model></sdf>')
try:
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5)
    spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    p=Pose(); p.position.x=0.85; p.position.y=-3.06; p.position.z=0.4; p.orientation.w=1.0
    r=spawn('p4_box', sdf, '', p, 'world'); mark('spawn', ok=bool(r.success), msg=r.status_message[:60])
except Exception as e:
    mark('spawn_fail', err=str(e)[:80])
time.sleep(2.5)
mark('obst_after_spawn', data=st['obst'])
# 发目标越过障碍
p=st['pose'] or (0.0,-3.06)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); time.sleep(0.5)
goal=(p[0]+1.8, p[1])
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=goal[0]; g.pose.position.y=goal[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal_sent', x=goal[0])
arrived=None; end=time.time()+40
while time.time()<end:
    if st['pose'] and abs(st['pose'][0]-goal[0])<0.35: arrived=st['pose']; break
    time.sleep(0.15)
mark('arrival', arrived=arrived, pose=st['pose'], bt=st['bt'])
nz=[c for c in st['cmd'] if abs(c)>1e-6]
mark('cmd_summary', nonzero=len(nz), max=max(nz) if nz else 0)
print(json.dumps({'events':ev,'bt':st['bt'],'obst':st['obst']}, ensure_ascii=False))
