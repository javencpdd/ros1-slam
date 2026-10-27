"""D1 图审计采集：把机器人摆到已知 GT 位姿，采 5 帧 /rslidar_points 与 GT。"""
import json
import math
import sys
import time

import numpy as np
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

OUT = sys.argv[1] if len(sys.argv) > 1 else '/work/map_audit_scans.json'
POSES = [(0.0, 0.0, 0.0), (4.186, 0.0, 0.0), (4.186, 0.0, math.pi), (2.0, 0.0, 0.0)]

rospy.init_node('map_audit_capture', disable_signals=True)
latest = {'cloud': None, 'gt': None}


def cb_cloud(m):
    latest['cloud'] = (m.header.stamp.to_sec(), m)


def cb_gt(m):
    latest['gt'] = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)


rospy.Subscriber('/rslidar_points', PointCloud2, cb_cloud, queue_size=3)
rospy.Subscriber('/sim/odom_gt', Odometry, cb_gt, queue_size=50)
rospy.wait_for_service('/gazebo/set_model_state', timeout=10)
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


def cloud_xyz(m):
    f = {fd.name: fd.offset for fd in m.fields}
    n = m.width * m.height
    buf = np.frombuffer(m.data, dtype=np.uint8, count=n * m.point_step).reshape(n, m.point_step)
    pts = np.empty((n, 3), dtype=np.float64)
    for i, k in enumerate('xyz'):
        pts[:, i] = buf[:, f[k]:f[k] + 4].copy().view(np.float32).reshape(-1)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] > 8000:
        idx = np.linspace(0, pts.shape[0] - 1, 8000).astype(int)
        pts = pts[idx]
    return np.round(pts, 4).tolist()


out = []
for (x, y, yaw) in POSES:
    state = ModelState()
    state.model_name = 'lite3_planar'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, 0.15
    pose.orientation.z = math.sin(yaw / 2)
    pose.orientation.w = math.cos(yaw / 2)
    state.pose = pose
    set_state(state)
    time.sleep(2.0)
    scans = []
    t_end = time.time() + 3.0
    seen = set()
    while time.time() < t_end and len(scans) < 5:
        if latest['cloud'] is not None and latest['cloud'][0] not in seen:
            seen.add(latest['cloud'][0])
            scans.append(cloud_xyz(latest['cloud'][1]))
        time.sleep(0.05)
    out.append({'pose': [x, y, yaw], 'gt': latest['gt'], 'scans': scans})
    print('pose', x, y, round(yaw, 3), 'scans', len(scans), 'gt', latest['gt'])

with open(OUT, 'w') as f:
    json.dump(out, f)
print('written', OUT)
