"""目标传感器全链停车注入：等速直行→在正前方生成墙体→记录停车分段（全部仿真钟）。"""
import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

OUT = '/work/scan_stop_event.json'
NEED_X = 0.9
WALL_AHEAD = 1.05
WALL_HX, WALL_HY, WALL_HZ = 0.2, 4.4, 0.5

rospy.init_node('scan_stop_event', disable_signals=True)
state = {'odom': None, 'cmd': None, 'cmd_t': None}


def cb_odom(m):
    p = m.pose.pose
    state['odom'] = (m.header.stamp.to_sec(), p.position.x, p.position.y, p.position.z,
                     m.twist.twist.linear.x)


def cb_cmd(m):
    state['cmd'] = m.linear.x
    state['cmd_t'] = rospy.Time.now().to_sec()


rospy.Subscriber('/sim/odom_gt', Odometry, cb_odom, queue_size=50)
rospy.Subscriber('/cmd_vel', Twist, cb_cmd, queue_size=50)
path_pub = rospy.Publisher('/initial_path', Path, queue_size=1, latch=True)


def send_path():
    p = Path()
    p.header.frame_id = 'world'
    p.header.stamp = rospy.Time.now()
    for x in (0.5, 1.5, 3.0, 4.5):
        g = PoseStamped()
        g.header = p.header
        g.pose.position.x = x
        g.pose.position.y = 0.0
        g.pose.position.z = 0.25
        g.pose.orientation.w = 1.0
        p.poses.append(g)
    path_pub.publish(p)


def wait_odom():
    t0 = time.time()
    while state['odom'] is None and time.time() - t0 < 10:
        time.sleep(0.1)
    return state['odom'] is not None


if not wait_odom():
    print(json.dumps({'ok': False, 'reason': 'no_odom'}))
    sys.exit(0)

send_path()
t0 = time.time()
while time.time() - t0 < 25:
    send_path()
    if state['odom'] is not None and state['odom'][1] > NEED_X and state['odom'][4] > 0.2:
        break
    time.sleep(1.0)
else:
    print(json.dumps({'ok': False, 'reason': 'never_moving', 'odom': state['odom'], 'cmd': state['cmd']}))
    sys.exit(0)

od = state['odom']
wall_x = round(od[1] + WALL_AHEAD, 3)
wall_y = round(od[2], 3)
with open('/work/scan_stop_box.json', 'w') as f:
    json.dump({'x': wall_x, 'y': wall_y, 'z': 0.5, 'hx': WALL_HX, 'hy': WALL_HY, 'hz': WALL_HZ}, f)

sdf = ('<sdf version="1.6"><model name="stop_wall"><static>true</static><link name="link">'
       '<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry>'
       '<material><script><name>Gazebo/Grey</name></script></material></visual></link></model></sdf>'
       % (2 * WALL_HX, 2 * WALL_HY, 2 * WALL_HZ, 2 * WALL_HX, 2 * WALL_HY, 2 * WALL_HZ))
rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
pose = Pose()
pose.position.x = wall_x
pose.position.y = wall_y
pose.position.z = 0.5
pose.orientation.w = 1.0
t_pre = rospy.Time.now().to_sec()
res = spawn('stop_wall', sdf, '', pose, 'world')
t_post = rospy.Time.now().to_sec()

zero_t = None
rest_x = None
deadline = time.time() + 20
while time.time() < deadline:
    if state['cmd'] is not None and abs(state['cmd']) < 0.01 and zero_t is None:
        zero_t = state['cmd_t']
    if zero_t is not None:
        cur = state['odom']
        if cur is not None:
            rest_x = cur[1]
        if time.time() > deadline - 14:
            break
    time.sleep(0.05)

cur = state['odom']
res = {
    'ok': bool(res.success),
    'spawn_msg': res.status_message,
    't_pre': round(t_pre, 4),
    't_post': round(t_post, 4),
    'wall': {'x': wall_x, 'y': wall_y, 'hx': WALL_HX, 'hy': WALL_HY, 'hz': WALL_HZ},
    'odom_at_spawn': od,
    'cmd_at_spawn': state['cmd'],
    'first_zero_cmd_t': None if zero_t is None else round(zero_t, 4),
    'odom_rest': cur,
    'vx_rest': None if cur is None else cur[4],
}
with open(OUT, 'w') as f:
    json.dump(res, f)
print(json.dumps(res))
