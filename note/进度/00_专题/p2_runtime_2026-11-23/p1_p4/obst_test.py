import json, time, rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
rospy.init_node('obst_test', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'obst':None,'cmd':[],'lstat':None}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/v8/local/obstacles', String, lambda m: st.update(obst=m.data))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,2), round(m.linear.x,3))))
rospy.Subscriber('/v8/local/status', String, lambda m: st.update(lstat=m.data))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
time.sleep(2)
p=st['pose'] or (0.0,-3.06)
# 在正前方 0.8 m 放一个真实障碍
sdf=('<sdf version="1.6"><model name="p4_blk"><static>true</static><link name="link">'
     '<collision name="c"><geometry><box><size>0.5 0.5 0.9</size></box></geometry></collision>'
     '<visual name="v"><geometry><box><size>0.5 0.5 0.9</size></box></geometry></visual></link></model></sdf>')
rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5)
bx=Pose(); bx.position.x=p[0]+0.8; bx.position.y=p[1]; bx.position.z=0.45; bx.orientation.w=1.0
mark('spawn', ok=bool(rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)('p4_blk', sdf,'',bx,'world').success))
time.sleep(3)
mark('obstacles_after_spawn', obst=(st['obst'] or '')[:160])
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
sp.publish(s); time.sleep(0.5)
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=p[0]+2.0; g.pose.position.y=p[1]; g.pose.orientation.w=1.0
gp.publish(g); mark('goal', x=p[0]+2.0)
time.sleep(10)
mark('after_10s', pose=st['pose'], cmd=st['cmd'][-3:], lstat=(st['lstat'] or '')[:110])
nz=[c for c in st['cmd'] if abs(c[1])>1e-6]
mark('summary', nonzero=len(nz), moved=(round(st['pose'][0]-p[0],2) if st['pose'] else None))
print(json.dumps({'events':ev}, ensure_ascii=False)[:1200])
