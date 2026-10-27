"""目标传感器全链停车记录：/rslidar_points 近箱距离、/cmd_vel、/sim/odom_gt（全部仿真钟）。"""
import json, math, os, sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

BOX_FILE = '/work/scan_stop_box.json'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/work/scan_stop_run.json'
rospy.init_node('scan_stop_rec', disable_signals=True)
box = None
gt = []          # (t, x, y, yaw)
cmds = []        # (t, vx, wz)
planner_cmds = []
clouds = []      # dict(t, dmin, n_near, n)
stat = {'in': 0}

def yaw_of(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def cb_gt(m):
    p = m.pose.pose
    gt.append((m.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation)))

def cb_cmd(m):
    cmds.append((rospy.Time.now().to_sec(), m.linear.x, m.angular.z))

def latest_gt(t):
    best = None
    for g in reversed(gt[-50:]):
        if best is None or abs(g[0] - t) < abs(best[0] - t):
            best = g
        if g[0] < t - 0.2:
            break
    if best is None or abs(best[0] - t) > 0.15:
        return None
    return best

def cb_cloud(m):
    global box
    stat['in'] += 1
    if box is None and os.path.exists(BOX_FILE):
        try:
            box = json.load(open(BOX_FILE))
        except Exception:
            box = None
    t = m.header.stamp.to_sec()
    rec = {'t': round(t, 4), 't_recv': round(rospy.Time.now().to_sec(), 4), 'n': m.width, 'dmin': None, 'n_near': None}
    if box is not None and m.width > 0:
        g = latest_gt(t)
        if g is not None:
            n = m.width
            step = m.point_step
            buf = np.frombuffer(m.data, dtype=np.uint8, count=n * step).reshape(n, step)
            pts = np.stack([buf[:, 0:4].copy().view(np.float32).reshape(-1),
                            buf[:, 4:8].copy().view(np.float32).reshape(-1),
                            buf[:, 8:12].copy().view(np.float32).reshape(-1)], axis=1)
            good = np.isfinite(pts).all(axis=1)
            pts = pts[good]
            cy, sy = math.cos(g[3]), math.sin(g[3])
            wx = cy * pts[:, 0] - sy * pts[:, 1] + g[1]
            wy = sy * pts[:, 0] + cy * pts[:, 1] + g[2]
            wz = pts[:, 2] + 0.11
            face_x = box['x'] - box['hx']
            slab = (np.abs(wx - face_x) < 0.15) & (np.abs(wy - box['y']) < 0.5) & (wz > 0.2)
            if slab.any():
                rec['dmin'] = round(float(np.min(np.abs(wx[slab] - face_x))), 4)
                rec['n_near'] = int(slab.sum())
    clouds.append(rec)

rospy.Subscriber('/sim/odom_gt', Odometry, cb_gt, queue_size=200)
rospy.Subscriber('/cmd_vel', Twist, cb_cmd, queue_size=200)
rospy.Subscriber('/scan_planner/cmd_vel', Twist, lambda m: planner_cmds.append((rospy.Time.now().to_sec(), m.linear.x)), queue_size=200)
rospy.Subscriber('/rslidar_points', PointCloud2, cb_cloud, queue_size=5)

def dump(_=None):
    with open(OUT, 'w') as f:
        json.dump({'gt': gt, 'cmds': cmds, 'planner_cmds': planner_cmds, 'clouds': clouds, 'cloud_in': stat['in']}, f)
rospy.on_shutdown(dump)
rospy.Timer(rospy.Duration(1.0), lambda _: dump())
rospy.spin()
