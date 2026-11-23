import json, time, subprocess, rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
rospy.init_node('p4_bcd', anonymous=True, disable_signals=True)
T0=time.time(); ev=[]
def mark(k,**kw): r={'t':round(time.time()-T0,2),'ev':k}; r.update(kw); ev.append(r)
st={'pose':None,'cmd':[],'ack':None,'bt':None,'mux':None}
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m: st.update(pose=(round(m.pose.position.x,3),round(m.pose.position.y,3))))
rospy.Subscriber('/cmd_vel', Twist, lambda m: st['cmd'].append((round(time.time()-T0,2), round(m.linear.x,3), round(m.linear.y,3))))
rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: (st.update(ack=m.data), mark('ack',data=m.data)))
rospy.Subscriber('/v8/bt/status', String, lambda m: st.update(bt=m.data[:150]))
rospy.Subscriber('/v8/3d/mux/status', String, lambda m: st.update(mux=m.data[:120]))
sp=rospy.Publisher('/v8/3d/global/start',PoseStamped,queue_size=1,latch=True)
gp=rospy.Publisher('/v8/bt/goal',PoseStamped,queue_size=1)
cp=rospy.Publisher('/v8/bt/cancel',String,queue_size=1)
hold_pub=rospy.Publisher('/lite3/supervisor/hold',Bool,queue_size=1)
time.sleep(2)
def send_goal(dx, tag):
    p=st['pose'] or (0.9,-3.03)
    s=PoseStamped(); s.header.stamp=rospy.Time.now(); s.header.frame_id='map'; s.pose.position.x=p[0]; s.pose.position.y=p[1]; s.pose.orientation.w=1.0
    sp.publish(s); time.sleep(0.4)
    g=PoseStamped(); g.header.stamp=rospy.Time.now(); g.header.frame_id='map'; g.pose.position.x=p[0]+dx; g.pose.position.y=p[1]; g.pose.orientation.w=1.0
    gp.publish(g); mark('goal_'+tag, x=g.pose.position.x, y=g.pose.position.y)
def tail(n=6): return st['cmd'][-n:]
mark('start_pose', pose=st['pose'])
# --- D1: hold 归零（先测，避免后面 kill 影响）
send_goal(0.5,'hold'); time.sleep(2.5)
mark('before_hold', mux=st['mux'], cmd=tail(2))
hold_pub.publish(Bool(data=True)); time.sleep(1.2)
mark('during_hold', mux=st['mux'], cmd=tail(2))
hold_pub.publish(Bool(data=False)); time.sleep(0.5)
# --- C: 取消
send_goal(1.2,'cancel'); time.sleep(2.5)
mark('before_cancel', cmd=tail(2))
c=String(); c.data='7:all:p4-C'; cp.publish(c); mark('cancel_sent')
time.sleep(2.0); mark('after_cancel', ack=st['ack'], cmd=tail(3))
# --- B: 断感知（kill perception 节点 = 停点云处理）
mark('kill_perception_start')
subprocess.run(['rosnode','kill','/v8/3d/perception/perception'], capture_output=True)
time.sleep(3.0)
send_goal(0.8,'nopercep'); time.sleep(4.0)
mark('B_after_no_perception', cmd=tail(4), pose=st['pose'], mux=st['mux'])
print(json.dumps({'events':ev,'bt':st['bt']}, ensure_ascii=False))
