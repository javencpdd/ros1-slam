#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W3 T2 执行体（小明 2026-10-07）：A5 组合矩阵 harness。
W2 版（w2_t2_exec.py 归档件为基线）+ 三项 Wave3 修正（D-W2-3/N2 裁决）：
  ①planner 按名点单参数化（argv[3]，GlobalPlanner|NavfnROS——A5 两组合同 harness）；
  ②dt_wall bug 修复固化（tw0 在 send_goal 前落锚）+ sanity 护栏（dt_wall<5s 判 HARNESS-BUG）；
  ③RTF 双时间戳锚点法逐跑必探：运动窗 Δsim(gt 序列 sim 时间跨)/Δwall(gt 首末消息墙钟到达跨)，
    A0 门=0.25（M2R2 主理人裁决），<0.25 判 INVALID exit=6（C 文「低于即减载重跑」保留）。
门集=5 门（G1–G4+pose_fresh 稳定窗），两组合同版（补登④闭环）。
用法: python3 w3_t2_exec.py <controller> <runlabel> <planner>
输出: /tmp/matrix_w3/T2_<label>/{gt.tum,odom.tum,cmdvel.csv} + stdout 摘要
"""
import sys, os, math, time, csv
import rospy, actionlib
import numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from mbf_msgs.msg import GetPathAction, GetPathGoal, ExePathAction, ExePathGoal
import tf as tf_py

CTRL, LABEL = sys.argv[1], sys.argv[2]
PLANNER = sys.argv[3] if len(sys.argv) > 3 else 'GlobalPlanner'
OUT = '/tmp/matrix_w3/T2_%s' % LABEL
os.makedirs(OUT, exist_ok=True)
rospy.init_node('w3_t2_%s' % LABEL)

S = dict(gt=None, odom=None)
rospy.Subscriber('/gazebo/model_states', ModelStates,
                 lambda m: [S.update(gt=(p.position.x, p.position.y))
                            for p, n in zip(m.pose, m.name) if n == 'lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/Odometry', Odometry, lambda m: S.update(odom=m), queue_size=2)
w0 = time.time()
while (S['gt'] is None or S['odom'] is None) and time.time() - w0 < 90: time.sleep(0.1)
assert S['gt'] is not None, 'no /gazebo/model_states'
sx, sy = S['gt'][0], S['gt'][1]; gx, gy = sx + 3.0, sy
print('W3T2 start=(%.3f,%.3f) goal=(%.3f,%.3f) planner=%s controller=%s' % (sx, sy, gx, gy, PLANNER, CTRL))

# ---- 5 门（G1–G4+pose_fresh，总预算 120s wall）----
GATE_BUDGET = 120.0
gate_t0 = time.time()
tf_ = tf_py.TransformListener()
def gate_left(): return GATE_BUDGET - (time.time() - gate_t0)
def gate_wait(name, fn):
    w = time.time()
    while time.time() - w < gate_left() and not rospy.is_shutdown():
        try:
            if fn():
                print('W3T2 gate_%s=PASS t=%.1fs' % (name, time.time() - gate_t0)); return True
        except Exception: pass
        time.sleep(0.3)
    print('W3T2 gate_%s=FAIL t=%.1fs' % (name, time.time() - gate_t0)); return False
def costmap_up(topic):
    try: return rospy.wait_for_message(topic, OccupancyGrid, timeout=2.0).info.width > 0
    except Exception: return False
ok_map = gate_wait('map_data', lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
ok_tf  = gate_wait('tf_robot', lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
fresh_hits = [0]
def pose_fresh():
    ok = (S.get('odom') is not None
          and rospy.get_rostime().to_sec() >= 5.0
          and abs(rospy.get_rostime().to_sec() - S['odom'].header.stamp.to_sec()) <= 1.0)
    fresh_hits[0] = fresh_hits[0] + 1 if ok else 0
    return fresh_hits[0] >= 3
ok_fresh = gate_wait('pose_fresh', pose_fresh)
ok_gcm = gate_wait('gcostmap_data', lambda: costmap_up('/move_base_flex/global_costmap/costmap'))
ok_lcm = gate_wait('lcostmap_data', lambda: costmap_up('/move_base_flex/local_costmap/costmap'))
if not all([ok_map, ok_tf, ok_fresh, ok_gcm, ok_lcm]):
    print('W3T2 gate=FAIL t=%.1fs（环境未就绪）' % (time.time() - gate_t0)); raise SystemExit(2)
print('W3T2 gate=PASS(5门) t=%.1fs' % (time.time() - gate_t0))

# ---- GetPath（按名点单）→ ExePath ----
gp = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
ep = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
assert gp.wait_for_server(rospy.Duration(30)), 'get_path server absent'
assert ep.wait_for_server(rospy.Duration(30)), 'exe_path server absent'
g = GetPathGoal()
g.target_pose.header.frame_id = 'map'; g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = gx; g.target_pose.pose.position.y = gy
g.target_pose.pose.orientation.w = 1.0
g.planner = PLANNER; g.use_start_pose = False
gr = None
for attempt in range(1, 4):
    gp.send_goal(g)
    if not gp.wait_for_result(rospy.Duration(30)):
        gp.cancel_goal(); print('W3T2 get_path attempt%d=TIMEOUT' % attempt); continue
    gr = gp.get_result()
    print('W3T2 get_path attempt%d planner=%s outcome=%d "%s" poses=%d' % (attempt, PLANNER, gr.outcome, gr.message, len(gr.path.poses)))
    if gr.outcome == 0: break
    rospy.sleep(2.0)
if gr is None or gr.outcome != 0:
    raise SystemExit(3)

# 执行窗存档器（+RTF 双时间戳锚点：gt 序列 sim 跨 / gt 首末消息墙钟到达跨）
rec = dict(gt=[], odom=[], cmd=[], t0=None, tw0=None, tw_first=None, tw_last=None)
def on_gt_tum(m):
    if rec['t0'] is None: return
    now_wall = time.time()
    if rec['tw_first'] is None: rec['tw_first'] = now_wall
    rec['tw_last'] = now_wall
    for p, n in zip(m.pose, m.name):
        if n == 'lite3_planar':
            t = rospy.get_rostime().to_sec(); o = p.orientation
            rec['gt'].append((t, p.position.x, p.position.y, p.position.z, o.x, o.y, o.z, o.w))
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
rec['tw0'] = time.time()          # ① dt_wall 修复固化：send_goal 前落锚
ep.send_goal(eg)
ok = ep.wait_for_result(rospy.Duration(720))   # 120s sim × RTF~0.29 ≈ 414s wall +裕度
dt_sim = (rospy.get_rostime() - t0).to_sec()
dt_wall = time.time() - rec['tw0']
if not ok:
    ep.cancel_goal(); print('W3T2 exe_path=TIMEOUT(720s wall)'); raise SystemExit(4)
r = ep.get_result()
m = S['odom']; d = math.hypot(m.pose.pose.position.x - gx, m.pose.pose.position.y - gy)
gd = math.hypot(S['gt'][0] - gx, S['gt'][1] - gy)
print('W3T2 exe_path outcome=%d "%s" state=%s' % (r.outcome, r.message, ep.get_state()))
print('W3T2 dt=%.1fs(sim) dt_wall=%.1fs dev_odom=%.3f dev_gt=%.3f' % (dt_sim, dt_wall, d, gd))
if dt_wall < 5.0:
    print('W3T2 HARNESS-BUG-SUSPECT dt_wall=%.1fs<5s（锚点异常，逐跑必探拦标）' % dt_wall)

# ---- RTF 双时间戳锚点法（A0 逐跑必探）----
rtf = None
if len(rec['gt']) >= 2 and rec['tw_first'] and rec['tw_last']:
    sim_span = rec['gt'][-1][0] - rec['gt'][0][0]
    wall_span = rec['tw_last'] - rec['tw_first']
    if wall_span > 1.0:
        rtf = sim_span / wall_span
        print('W3T2 rtf_anchor=%.3f (Δsim=%.1fs/Δwall=%.1fs, gt_n=%d)' % (rtf, sim_span, wall_span, len(rec['gt'])))
    else:
        print('W3T2 rtf_anchor=NA wall_span=%.1fs 过短' % wall_span)
else:
    print('W3T2 rtf_anchor=NA gt_n=%d' % len(rec['gt']))
rtf_ok = (rtf is not None and rtf >= 0.25)

# 平滑度：/cmd_vel_raw 角速度一阶差分 RMS
wz = [c[3] for c in rec['cmd']]
if len(wz) >= 2:
    diffs = np.diff(np.array(wz))
    print('W3T2 smooth_rms=%.4f cmd_n=%d' % (float(np.sqrt((diffs ** 2).mean())), len(wz)))
else:
    print('W3T2 smooth_rms=NA cmd_n=%d' % len(wz))

# 存档
with open(OUT + '/gt.tum', 'w') as f:
    for row in rec['gt']: f.write('%.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % row)
with open(OUT + '/odom.tum', 'w') as f:
    for row in rec['odom']: f.write('%.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % row)
with open(OUT + '/cmdvel.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t_sim', 'vx', 'vy', 'wz']); [w.writerow(c) for c in rec['cmd']]
print('W3T2 archived gt=%d odom=%d cmd=%d rows -> %s' % (len(rec['gt']), len(rec['odom']), len(rec['cmd']), OUT))
verdict = (ep.get_state() == 3 and r.outcome == 0 and dt_sim <= 120.0 and d <= 0.35 and rtf_ok)
print('W3T2 VERDICT %s state=%s outcome=%d dt_ok=%s dev_ok=%s rtf=%.3f rtf_ok(>=0.25)=%s' %
      ('PASS' if verdict else 'FAIL', ep.get_state(), r.outcome, dt_sim <= 120.0, d <= 0.35,
       -1.0 if rtf is None else rtf, rtf_ok))
if verdict: raise SystemExit(0)
if not rtf_ok: raise SystemExit(6)   # A0 INVALID：减载重跑口径
raise SystemExit(5)
