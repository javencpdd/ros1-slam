"""SCAN 局部层时延探针：点云/位姿快照 → 轨迹 → cmd_vel（ROS 墙钟=仿真时间基）。"""
import json, sys, time
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from scan_planner.msg import Bspline
from nav_msgs.msg import Path

rospy.init_node('scan_lat_probe', disable_signals=True)
cloud_t = []
plan_t = []
cmd_t = []
pose = {'x': None, 'y': None}
start = {'x': None, 'y': None}

rospy.Subscriber('/pcl_render_node/cloud', PointCloud2, lambda m: cloud_t.append(rospy.Time.now().to_sec()), queue_size=5)
rospy.Subscriber('/planning/bspline', Bspline, lambda m: plan_t.append(rospy.Time.now().to_sec()), queue_size=20)
rospy.Subscriber('/cmd_vel', Twist, lambda m: cmd_t.append(rospy.Time.now().to_sec()), queue_size=50)

def pose_cb(m):
    p = m.pose.pose.position
    pose['x'], pose['y'] = p.x, p.y
    if start['x'] is None:
        start['x'], start['y'] = p.x, p.y

rospy.Subscriber('/quad_0/body_pose', Odometry, pose_cb, queue_size=5)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
MODE = sys.argv[3] if len(sys.argv) > 3 else 'goal'

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
out = sys.argv[2] if len(sys.argv) > 2 else '/work/scan_lat.json'

def send(x, y, z=0.25):
    g = PoseStamped()
    g.header.frame_id = 'world'
    g.header.stamp = rospy.Time.now()
    g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, z
    g.pose.orientation.w = 1.0
    goal_pub.publish(g)

time.sleep(2)
WPS = [(-5.5, 5.5), (-2.0, 5.5), (1.5, 5.5), (5.0, 5.5), (8.5, 5.5), (12.0, 5.5), (15.0, 5.5)]
def send_path():
    pa = Path()
    pa.header.frame_id = 'world'
    pa.header.stamp = rospy.Time.now()
    for (x, y) in WPS:
        g = PoseStamped()
        g.header = pa.header
        g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, 0.25
        g.pose.orientation.w = 1.0
        pa.poses.append(g)
    path_pub.publish(pa)
if MODE == 'path':
    send_path()
else:
    send(-14.5, 3.5)
t0 = time.time()
next_goal = t0 + 15
goal_i = 1
while time.time() - t0 < dur:
    if time.time() > next_goal and goal_i < 4:
        if MODE == 'path':
            send_path()
        else:
            pts = [(-10.0, 4.0), (-8.0, -2.0), (-14.0, -4.0)]
            send(*pts[goal_i - 1])
        goal_i += 1
        next_goal = time.time() + 15
    time.sleep(0.05)

import bisect
def last_before(arr, t):
    i = bisect.bisect_right(arr, t) - 1
    return arr[i] if i >= 0 else None

cycles = []
for pt in plan_t:
    ct = last_before(cloud_t, pt)
    if ct is None:
        continue
    j = bisect.bisect_left(cmd_t, pt)
    cmdt = cmd_t[j] if j < len(cmd_t) else None
    cycles.append({'cloud_to_plan_ms': (pt - ct) * 1000,
                   'plan_to_cmd_ms': (cmdt - pt) * 1000 if cmdt else None,
                   'cloud_to_cmd_ms': (cmdt - ct) * 1000 if cmdt else None})
def stats(vals):
    vals = sorted(v for v in vals if v is not None)
    if not vals:
        return None
    def q(p):
        return vals[min(len(vals) - 1, int(round(p * (len(vals) - 1))))]
    return {'n': len(vals), 'p50_ms': round(q(0.5), 2), 'p95_ms': round(q(0.95), 2),
            'max_ms': round(vals[-1], 2)}
res = {
    'clock': 'ros_wall (go2_kinematic_sim integrates wall dt)',
    'duration_s': dur,
    'n_cloud': len(cloud_t), 'n_plan': len(plan_t), 'n_cmd': len(cmd_t),
    'cloud_rate_hz': round(len(cloud_t) / dur, 2) if dur else None,
    'cmd_rate_hz': round(len(cmd_t) / dur, 2) if dur else None,
    'cloud_to_plan': stats([c['cloud_to_plan_ms'] for c in cycles]),
    'plan_to_cmd': stats([c['plan_to_cmd_ms'] for c in cycles]),
    'cloud_to_cmd': stats([c['cloud_to_cmd_ms'] for c in cycles]),
    'cycles': cycles,
    'pose_end': pose, 'pose_start': start,
}
with open(out, 'w') as f:
    json.dump(res, f)
print(json.dumps({k: v for k, v in res.items() if k != 'cycles'}, ensure_ascii=False))
