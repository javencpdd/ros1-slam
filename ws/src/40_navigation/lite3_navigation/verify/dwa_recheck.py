#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dwa_recheck.py —— W2/A4 遗留项「DWA 经 mbf 内建 nav_core 包装不可用」的重核执行体（小明 2026-10-08）

为何现在重测：W2 三败的失败签名是位姿获取域（原件 T2_dwa_run1.txt L8：
  get_path outcome=57 "Could not get the current robot pose!"；00_总览在册的败#3 原件行为
  「pose 5.372s old tolerance 3s」）。而今天 M2R5/U1 刚把该家族的根因根除
  （base_link 双父 + 静态 map->odom 零锚点 => tf2 回 Time(0) => age = sim_now > tolerance）。
  小美 Wave2 验收当时写的是「非算法否定，换栈/mbf 升级待后续重核」——前置修复已到位，
  不重测就把 roadmap C #4（自研 CostmapController，3–5d）直接撤掉是判据失职。

本件零默认行为变化：控制器按名点单（ExePathGoal.controller=DWAPlannerROS），
teb 仍是 mbf_navigation.yaml 里的首位默认控制器。

用法：python3 dwa_recheck.py <controller> <runlabel>
判据（roadmap C #4 原句）：mbf 试加载插件成功 + ExePath outcome=0 ×3；本件另附位姿龄取证。
"""
import sys, os, math, time, csv
import rospy, actionlib
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from mbf_msgs.msg import GetPathAction, GetPathGoal, ExePathAction, ExePathGoal
from sensor_msgs.msg import Imu

CTRL, LABEL = sys.argv[1], sys.argv[2]
OUT = os.environ.get('DWA_OUT', '/tmp/dwa_recheck') + '/R_%s' % LABEL
os.makedirs(OUT, exist_ok=True)
rospy.init_node('dwa_recheck_%s' % LABEL)

S = dict(gt=None, odom=None, imut=None)
rospy.Subscriber('/gazebo/model_states', ModelStates,
                 lambda m: [S.update(gt=(p.position.x, p.position.y))
                            for p, n in zip(m.pose, m.name) if n == 'lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/sim/odom_gt', Odometry, lambda m: S.update(odom=m), queue_size=2)  # 批1/F-D：与 m2_smoke 同源同名，/Odometry 已让给 faster_lio
def on_imu(m):
    S['imut'] = m.header.stamp.to_sec()
rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=1)
w0 = time.time()
while (S['gt'] is None or S['odom'] is None) and time.time() - w0 < 90:
    time.sleep(0.1)
assert S['gt'] is not None, 'no /gazebo/model_states'
sx, sy = S['gt']; gx, gy = sx + 3.0, sy
print('RECHK run=%s controller=%s start=(%.3f,%.3f) goal=(%.3f,%.3f)' % (LABEL, CTRL, sx, sy, gx, gy))

# ---- 门控：与 stageB 同族（含 M2R5 常驻 gate 项 G5c1 的同款取龄）----
import tf2_ros
buf2 = tf2_ros.Buffer()
tf2_ros.TransformListener(buf2)
MAP_FIRST = [None]
def _mapcb(m):
    if MAP_FIRST[0] is None:
        MAP_FIRST[0] = m.header.stamp
rospy.Subscriber('/map', OccupancyGrid, _mapcb, queue_size=1)

def age_map_base():
    st = buf2.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
    return (rospy.get_rostime() - st.header.stamp).to_sec(), st.header.stamp.to_sec()

BUDGET = 120.0
t0g = time.time()
def gate(name, fn):
    w = time.time()
    while time.time() - w < BUDGET - (time.time() - t0g) and not rospy.is_shutdown():
        try:
            if fn():
                print('RECHK gate_%s=PASS t=%.1fs' % (name, time.time() - t0g)); return True
        except Exception as e:
            pass
        time.sleep(0.3)
    print('RECHK gate_%s=FAIL t=%.1fs' % (name, time.time() - t0g)); return False

def costmap_up(topic):
    try:
        return rospy.wait_for_message(topic, OccupancyGrid, timeout=2.0).info.width > 0
    except Exception:
        return False

ok = []
ok.append(gate('map_data', lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0))
ok.append(gate('gcostmap_data', lambda: costmap_up('/move_base_flex/global_costmap/costmap')))
ok.append(gate('lcostmap_data', lambda: costmap_up('/move_base_flex/local_costmap/costmap')))
hits = [0]
def tf_fresh():
    try:
        a, _ = age_map_base()
    except Exception:
        hits[0] = 0; return False
    hits[0] = hits[0] + 1 if a <= 1.0 else 0
    return hits[0] >= 3
ok.append(gate('tf_fresh_mapbase', tf_fresh))
if not all(ok):
    print('RECHK gate=FAIL（环境未就绪，本轮作废重跑，不计 DWA 判据）'); raise SystemExit(2)

# ---- 位姿龄取证（这是当年真正卡住的那一维）----
a_last, s_last = None, None
for _ in range(5):
    try:
        a_last, s_last = age_map_base()
    except Exception as e:
        a_last = None
    time.sleep(0.2)
print('RECHK pose_age_map_base_link=%.3fs stamp=%.3f sim_now=%.3f' % (a_last if a_last is not None else -1,
      s_last if s_last is not None else -1, rospy.get_rostime().to_sec()))

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
for attempt in range(1, 4):
    gp.send_goal(g)
    if not gp.wait_for_result(rospy.Duration(30)):
        gp.cancel_goal(); print('RECHK get_path attempt%d=TIMEOUT' % attempt); continue
    gr = gp.get_result()
    print('RECHK get_path attempt%d outcome=%d "%s" poses=%d' % (attempt, gr.outcome, gr.message, len(gr.path.poses)))
    if gr.outcome == 0:
        break
    rospy.sleep(2.0)
if gr is None or gr.outcome != 0:
    print('RECHK VERDICT FAIL reason=get_path outcome=%s msg=%s' %
          (gr.outcome if gr else 'NONE', gr.message if gr else 'NONE'))
    raise SystemExit(3)

rec = dict(cmd=[], t0=None, tw0=None)
def on_cmd(m):
    if rec['t0'] is None: return
    rec['cmd'].append((rospy.get_rostime().to_sec(), m.linear.x, m.linear.y, m.angular.z))
rospy.Subscriber('/cmd_vel_raw', Twist, on_cmd, queue_size=5)

eg = ExePathGoal(); eg.path = gr.path; eg.controller = CTRL
eg.tolerance_from_action = False
t0 = rospy.get_rostime(); rec['t0'] = t0.to_sec(); rec['tw0'] = time.time()
ep.send_goal(eg)
done = ep.wait_for_result(rospy.Duration(720))
dt_sim = (rospy.get_rostime() - t0).to_sec(); dt_wall = time.time() - rec['tw0']
if not done:
    ep.cancel_goal(); print('RECHK exe_path=TIMEOUT(720s wall) state=%s' % ep.get_state()); raise SystemExit(4)
r = ep.get_result()
m = S['odom']
d = math.hypot(m.pose.pose.position.x - gx, m.pose.pose.position.y - gy)
gd = math.hypot(S['gt'][0] - gx, S['gt'][1] - gy)
rtf = (S['imut'] - rec['t0']) / dt_wall if S.get('imut') and dt_wall > 0 else 0.0
wz = [c[3] for c in rec['cmd']]
sm = 'NA'
if len(wz) >= 2:
    dd = [wz[i + 1] - wz[i] for i in range(len(wz) - 1)]
    sm = '%.4f' % math.sqrt(sum(x * x for x in dd) / len(dd))
print('RECHK exe_path outcome=%d "%s" state=%s' % (r.outcome, r.message, ep.get_state()))
print('RECHK dt=%.1fs(sim) dt_wall=%.1fs dev_odom=%.3f dev_gt=%.3f rtf=%.3f smooth_rms=%s cmd_n=%d'
      % (dt_sim, dt_wall, d, gd, rtf, sm, len(rec['cmd'])))
with open(OUT + '/cmdvel.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t_sim', 'vx', 'vy', 'wz']); [w.writerow(c) for c in rec['cmd']]
# 判据与有效性门**分开判**（M2R2 主理人裁决在先：RTF<0.25 属"本轮作废重跑、不计 FAIL"；
#   首版把 rtf 折进 verdict，使一次环境抖动被误读成"DWA 不可用"——判据语义混叠，已纠）。
valid = (rtf >= 0.25 and dt_sim > 0)
verdict = (ep.get_state() == 3 and r.outcome == 0 and dt_sim <= 120.0 and d <= 0.35)
print('RECHK 有效性门 rtf=%.3f%s ｜ 判据 state=%s outcome=%d dt_ok=%s dev_ok=%s'
      % (rtf, 'PASS' if valid else 'INVALID(作废重跑，不计 FAIL)', ep.get_state(), r.outcome,
         dt_sim <= 120.0, d <= 0.35))
print('RECHK VERDICT %s' % ('PASS' if verdict else 'FAIL'))
# R40（小美 M2R8 裁定 2）：有效性不过而判据亦不过时给**并存独立档 7**，不再被 6 吸收——
#   旧写法只要 valid 为假就 exit 6，等于把「这一跑不算数」与「判据也没过」揉成一个数，
#   调查面会只重跑而不看判据（与 SM 的 #30 同型，件自有 rc 层）。
if not valid and not verdict:
    print('RECHK 并存：有效性门不过 且 判据不过 ⇒ rc=7 停手上报（不得由重试消化）')
    raise SystemExit(7)
if not valid:
    raise SystemExit(6)
raise SystemExit(0 if verdict else 5)
