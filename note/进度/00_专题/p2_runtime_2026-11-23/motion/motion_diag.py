import json, time, threading
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

rospy.init_node('motion_diag', anonymous=True, disable_signals=True)
T0 = time.time(); ev = []
def mark(k, **kw):
    r = {'t': round(time.time()-T0, 2), 'ev': k}; r.update(kw); ev.append(r)

state = {'pose': None, 'lok': None, 'lok_stamp': None, 'lok_count': 0, 'path0': None,
         'cmd3d': [], 'cmd': [], 'mux': None, 'lstatus': None}
def on_pose(m):
    p = m.pose.position; state['pose'] = (round(p.x,3), round(p.y,3), round(p.z,3), m.header.frame_id, m.header.stamp.to_sec())
def on_lok(m):
    state['lok'] = bool(m.data); state['lok_stamp'] = time.time(); state['lok_count'] += 1
def on_path(m):
    if m.poses and state['path0'] is None:
        p = m.poses[0].pose.position
        state['path0'] = (round(p.x,3), round(p.y,3), round(p.z,3), m.header.frame_id, len(m.poses))
def on_cmd3d(m):
    t = getattr(m, 'twist', None)
    if t is not None:
        state['cmd3d'].append((round(time.time()-T0,2), getattr(m,'header',None).frame_id, round(t.linear.x,3), round(t.linear.y,3), round(t.linear.z,3)))
def on_cmd(m):
    state['cmd'].append((round(time.time()-T0,2), round(m.linear.x,3), round(m.linear.y,3), round(m.linear.z,3)))
def on_mux(m): state['mux'] = m.data[:120]
def on_lstatus(m): state['lstatus'] = m.data[:160]

rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=1)
rospy.Subscriber('/v8/local/localization_ok', Bool, on_lok, queue_size=1)
rospy.Subscriber('/v8/3d/global/path', Path, on_path, queue_size=1)
rospy.Subscriber('/v8/local/cmd_vel_3d', TwistStamped, on_cmd3d, queue_size=20)
rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=20)
rospy.Subscriber('/v8/3d/mux/status', String, on_mux, queue_size=5)
rospy.Subscriber('/v8/3d/local/status', String, on_lstatus, queue_size=5)

time.sleep(2.0)
mark('lok_value', value=state['lok'], count=state['lok_count'])
mark('pose_now', pose=state['pose'])

# 起点取当前有效位姿（图契约：start 由 operator 提供）
start_pub = rospy.Publisher('/v8/3d/global/start', PoseStamped, queue_size=1, latch=True)
goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
p = state['pose'] or (0.0, -3.06, 0.0, 'map', 0)
st = PoseStamped(); st.header.stamp = rospy.Time.now(); st.header.frame_id = 'map'
st.pose.position.x, st.pose.position.y, st.pose.position.z = p[0], p[1], 0.0
st.pose.orientation.w = 1.0
start_pub.publish(st); mark('start_sent', x=p[0], y=p[1])
time.sleep(1.0)
g = PoseStamped(); g.header.stamp = rospy.Time.now(); g.header.frame_id = 'map'
g.pose.position.x = p[0] + 1.5; g.pose.position.y = p[1]; g.pose.orientation.w = 1.0
goal_pub.publish(g); mark('goal_sent', x=g.pose.position.x, y=g.pose.position.y)

time.sleep(14.0)   # A 组：正常执行窗口，不取消
mark('window_end', pose=state['pose'], path0=state['path0'], lok=state['lok'],
     n_cmd3d=len(state['cmd3d']), n_cmd=len(state['cmd']), mux=state['mux'], lstatus=state['lstatus'])
print(json.dumps({'events': ev, 'cmd3d_head': state['cmd3d'][:6], 'cmd_head': state['cmd'][:6],
                  'pose': state['pose'], 'path0': state['path0'], 'lok': state['lok'],
                  'lok_count': state['lok_count'], 'mux': state['mux'], 'local_status': state['lstatus']},
                 ensure_ascii=False))
