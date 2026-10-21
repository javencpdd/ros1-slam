#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W2 T2 执行体（小明 2026-10-07）：stageB 门控 G1–G4 + GetPath→ExePath 按名点单 + 存档。
输入: controller runlabel；输出: T2_<label>.txt 摘要 + /tmp/matrix_w2/T2_<label>/{gt.tum,odom.tum,cmdvel.csv}"""
import sys, os, math, time, csv
import rospy, actionlib
import numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import Twist
from mbf_msgs.msg import GetPathAction, GetPathGoal, ExePathAction, ExePathGoal
import tf as tf_py

CTRL, LABEL = sys.argv[1], sys.argv[2]
OUT = '/tmp/matrix_w2/T2_%s' % LABEL
os.makedirs(OUT, exist_ok=True)
rospy.init_node('w2_t2_%s' % LABEL)

S = dict(gt=None, gyaw=None, odom=None, iq=None, imut=None)
def qyaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rospy.Subscriber('/gazebo/model_states', ModelStates,
                 lambda m: [S.update(gt=(p.position.x, p.position.y),
                                     gq=(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                            for p, n in zip(m.pose, m.name) if n == 'lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/Odometry', Odometry, lambda m: S.update(odom=m), queue_size=2)
def on_imu(m):
    S['iq'] = m.orientation
    S['imut'] = m.header.stamp.to_sec()
rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=1)
w0 = time.time()
while (S['gt'] is None or S['odom'] is None) and time.time() - w0 < 90: time.sleep(0.1)
assert S['gt'] is not None, 'no /gazebo/model_states'
sx, sy = S['gt'][0], S['gt'][1]; gx, gy = sx + 3.0, sy
print('W2T2 start=(%.3f,%.3f) goal=(%.3f,%.3f) controller=%s' % (sx, sy, gx, gy, CTRL))

# ---- G1–G4 门控（stageB 同款，总预算 120s wall）----
GATE_BUDGET = 120.0
gate_t0 = time.time()
tf_ = tf_py.TransformListener()
def gate_left(): return GATE_BUDGET - (time.time() - gate_t0)
def gate_wait(name, fn):
    w = time.time()
    while time.time() - w < gate_left() and not rospy.is_shutdown():
        try:
            if fn():
                print('W2T2 gate_%s=PASS t=%.1fs' % (name, time.time() - gate_t0)); return True
        except Exception: pass
        time.sleep(0.3)
    print('W2T2 gate_%s=FAIL t=%.1fs' % (name, time.time() - gate_t0)); return False
def costmap_up(topic):
    try: return rospy.wait_for_message(topic, OccupancyGrid, timeout=2.0).info.width > 0
    except Exception: return False
ok_map = gate_wait('map_data', lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
ok_tf  = gate_wait('tf_robot', lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
# W2 增补（DWA 114 根因防御）：首控周期前保证 ①sim 时间已走 ≥5s（Gazebo 起步瞬态外）
# ②位姿源新鲜（/Odometry stamp 距 now ≤1.0s）。tf1 getLatestCommonTime 对 /tf_static 链恒返 0，不可用作新鲜度（探针实证）。
fresh_hits = [0]
def pose_fresh():
    ok = (S.get('odom') is not None
          and rospy.get_rostime().to_sec() >= 5.0
          and abs(rospy.get_rostime().to_sec() - S['odom'].header.stamp.to_sec()) <= 1.0)
    if ok:
        fresh_hits[0] += 1
    else:
        fresh_hits[0] = 0
    return fresh_hits[0] >= 3   # 稳定窗：连续 3 次采样（每次 gate 间隔 0.3s）都新鲜，越过起步瞬态
ok_fresh = gate_wait('pose_fresh', pose_fresh)
ok_gcm = gate_wait('gcostmap_data', lambda: costmap_up('/move_base_flex/global_costmap/costmap'))
ok_lcm = gate_wait('lcostmap_data', lambda: costmap_up('/move_base_flex/local_costmap/costmap'))
if not all([ok_map, ok_tf, ok_fresh, ok_gcm, ok_lcm]):
    print('W2T2 gate=FAIL t=%.1fs（环境未就绪）' % (time.time() - gate_t0)); raise SystemExit(2)
print('W2T2 gate=PASS t=%.1fs' % (time.time() - gate_t0))

# ---- GetPath → ExePath 按名点单 ----
gp = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
ep = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
assert gp.wait_for_server(rospy.Duration(30)), 'get_path server absent'
assert ep.wait_for_server(rospy.Duration(30)), 'exe_path server absent'
g = GetPathGoal()
g.target_pose.header.frame_id = 'map'; g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = gx; g.target_pose.pose.position.y = gy
g.target_pose.pose.orientation.w = 1.0
g.planner = 'GlobalPlanner'; g.use_start_pose = False
gr = None
for attempt in range(1, 4):   # 起步窗瞬态容错：get_path 最多 3 试（114/57 类瞬态实证）
    gp.send_goal(g)
    if not gp.wait_for_result(rospy.Duration(30)):
        gp.cancel_goal(); print('W2T2 get_path attempt%d=TIMEOUT' % attempt); continue
    gr = gp.get_result()
    print('W2T2 get_path attempt%d outcome=%d "%s" poses=%d' % (attempt, gr.outcome, gr.message, len(gr.path.poses)))
    if gr.outcome == 0: break
    rospy.sleep(2.0)
if gr is None or gr.outcome != 0:
    raise SystemExit(3)

# 执行窗存档器
rec = dict(gt=[], odom=[], cmd=[], t0=None, tw0=None)
def on_gt_tum(m):
    if rec['t0'] is None: return
    for p, n in zip(m.pose, m.name):
        if n == 'lite3_planar':
            t = rospy.get_rostime().to_sec()
            o = p.orientation
            rec['gt'].append((t, p.position.x, p.position.y, p.position.z,
                              o.x, o.y, o.z, o.w))
rospy.Subscriber('/gazebo/model_states', ModelStates, on_gt_tum, queue_size=1)
def on_odom_tum(m):
    if rec['t0'] is None: return
    t = rospy.get_rostime().to_sec(); p = m.pose.pose; o = p.orientation
    rec['odom'].append((t, p.position.x, p.position.y, p.position.z, o.x, o.y, o.z, o.w))
rospy.Subscriber('/Odometry', Odometry, on_odom_tum, queue_size=2)
def on_cmd(m):
    if rec['t0'] is None: return
    rec['cmd'].append((rospy.get_rostime().to_sec(), m.linear.x, m.linear.y, m.angular.z))
rospy.Subscriber('/cmd_vel_raw', Twist, on_cmd, queue_size=5)

eg = ExePathGoal(); eg.path = gr.path; eg.controller = CTRL
eg.tolerance_from_action = False
t0 = rospy.get_rostime()
rec['t0'] = t0.to_sec()
rec['tw0'] = time.time()
ep.send_goal(eg)
ok = ep.wait_for_result(rospy.Duration(720))   # 120s sim × RTF~0.29 ≈ 414s wall +裕度
dt_sim = (rospy.get_rostime() - t0).to_sec()
dt_wall = time.time() - rec['tw0']
if not ok:
    ep.cancel_goal(); print('W2T2 exe_path=TIMEOUT(720s wall)'); raise SystemExit(4)
r = ep.get_result()
m = S['odom']; d = math.hypot(m.pose.pose.position.x - gx, m.pose.pose.position.y - gy)
gd = math.hypot(S['gt'][0] - gx, S['gt'][1] - gy)
print('W2T2 exe_path outcome=%d "%s" state=%s' % (r.outcome, r.message, ep.get_state()))
print('W2T2 dt=%.1fs(sim) dt_wall=%.1fs dev_odom=%.3f dev_gt=%.3f' % (dt_sim, dt_wall, d, gd))

# RTF 探针：运动窗 = IMU stamp 跨度 / 墙钟跨度
imu_span = (S['imut'] - rec['t0']) if S.get('imut') else 0.0
rtf = imu_span / dt_wall if dt_wall > 0 else 0.0
print('W2T2 rtf_motion=%.3f imu_span=%.1fs' % (rtf, imu_span))

# 平滑度：/cmd_vel_raw 角速度一阶差分 RMS（D 表 T2④）
wz = [c[3] for c in rec['cmd']]
if len(wz) >= 2:
    diffs = np.diff(np.array(wz))
    print('W2T2 smooth_rms=%.4f cmd_n=%d' % (float(np.sqrt((diffs ** 2).mean())), len(wz)))
else:
    print('W2T2 smooth_rms=NA cmd_n=%d' % len(wz))

# 存档（TUM 供 evo；cmd CSV 供复核）
with open(OUT + '/gt.tum', 'w') as f:
    for row in rec['gt']: f.write('%.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % row)
with open(OUT + '/odom.tum', 'w') as f:
    for row in rec['odom']: f.write('%.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % row)
with open(OUT + '/cmdvel.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t_sim', 'vx', 'vy', 'wz']); [w.writerow(c) for c in rec['cmd']]
print('W2T2 archived gt=%d odom=%d cmd=%d rows -> %s' % (len(rec['gt']), len(rec['odom']), len(rec['cmd']), OUT))
verdict = (ep.get_state() == 3 and r.outcome == 0 and dt_sim <= 120.0 and d <= 0.35)
print('W2T2 VERDICT %s state=%s outcome=%d dt_ok=%s dev_ok=%s rtf=%.3f' %
      ('PASS' if verdict else 'FAIL', ep.get_state(), r.outcome, dt_sim <= 120.0, d <= 0.35, rtf))
