import json, time, rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
rospy.init_node('cancel_test', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'ack':None,'cmd':[]}
rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: (st.update(ack=m.data), mark('ack', data=m.data)))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append(round(m.linear.x,3)))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
cp=rospy.Publisher('/v8/bt/cancel',String,queue_size=1)
s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.y=-3.06; s.pose.orientation.w=1.0
sp.publish(s); mark('start_sent')
g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=1.5; g.pose.position.y=-3.06; g.pose.orientation.w=1.0
gp.publish(g); mark('goal_sent')
time.sleep(3.0)
c=String(); c.data='2:all:probe-B'; cp.publish(c); mark('cancel_sent')
time.sleep(4.0)
tail=[x for x in st['cmd'][-5:]]
print(json.dumps({'events':ev,'ack':st['ack'],'cmd_tail':tail,'cmd_count':len(st['cmd']),
                  'all_zero':all(abs(x)<1e-9 for x in st['cmd']) if st['cmd'] else None}))
