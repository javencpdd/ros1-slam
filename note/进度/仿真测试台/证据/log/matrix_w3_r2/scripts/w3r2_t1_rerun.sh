#!/usr/bin/env bash
# w3r2_t1_rerun.sh —— W3-R2 T1 全 8 对复跑（小明 2026-10-07；manifest §四：A 落地后 T1 8 对 ≥8/8）
# 口径=W2 T1（w3_t1_smoke.sh）：start↔goals 正反向=8 对，GetPath outcome=0 成功率、弧长/穿墙=0；
# R2 增量：①GlobalPlanner/navfn 按名点单各一轮（主理人指令）；②mbf 存活探针+NO-RESULT 整栈重启同对公平重试 1 次
#   （W3 停手诊断设计输入"harness 增 mbf 存活探针"落地，判据不变）；③goals 环带 4–8m 结构断言（远角红线）。
# 判据：每 planner 轮 8 对 ≥8/8 outcome=0（manifest ≥90%=8/8），零穿墙。
LOGDIR=/tmp/matrix_w3/logs
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345

cleanup() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null; sleep 2; }
trap cleanup EXIT

FINAL=9
for CYCLE in 1 2 3 4 5 6; do
  echo "[w3r2-T1] === cycle $CYCLE 起栈 ==="
  setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
    map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
    perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
    map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGDIR/T1_perlin3d_r2_launch_c$CYCLE.log" 2>&1 &
  sleep 2
  CY=$CYCLE python3 - "$CYCLE" <<'PYEOF' 2>&1 | tee "$LOGDIR/T1_perlin3d_getpath_r2_c$CYCLE.log"
import sys, json, math, os, time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

name = 'perlin3d'
CY = os.environ.get('CY', '?')
STATE_F = '/tmp/matrix_w3/w3r2_t1_state.json'
G = json.load(open('/tmp/matrix_w3/%s_goals.json' % name))
start, goals = G['start'], G['goals']
RING_LO, RING_HI = 4.0, 8.0
for gi, g in enumerate(goals):
    d = math.hypot(g[0] - start[0], g[1] - start[1])
    assert RING_LO <= d <= RING_HI, 'T1 目标出环带（远角红线）g%d d=%.2f' % (gi, d)
rospy.init_node('w3r2_t1_%s' % name)
budget, t0 = 240.0, time.time()
tf_ = tf_py.TransformListener()
def gate(fn):
    while time.time() - t0 < budget and not rospy.is_shutdown():
        try:
            if fn(): return True
        except Exception: pass
        time.sleep(0.5)
    return False
g1 = gate(lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
g2 = gate(lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
g3 = gate(lambda: actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction).wait_for_server(rospy.Duration(2.0)))
print('T1R2 gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0), flush=True)
if not (g1 and g2 and g3):
    raise SystemExit(2)

def mbf_alive():
    try:
        return b'move_base_flex' in subprocess.run(['rosnode', 'list'], capture_output=True, timeout=15).stdout
    except Exception:
        return False
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
mg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
res_, ox, oy, w_, h_ = mg.info.resolution, mg.info.origin.position.x, mg.info.origin.position.y, mg.info.width, mg.info.height
def occ_at(x, y):
    i, j = int((x - ox) / res_), int((y - oy) / res_)
    if not (0 <= i < w_ and 0 <= j < h_): return 100
    return mg.data[j * w_ + i]

def call(a, b, planner):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    g.planner = planner
    rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(8))
    dt = (rospy.Time.now() - rt0).to_sec()
    if not ok:
        ac.cancel_goal()
        print('T1R2 (%.2f,%.2f)->(%.2f,%.2f) %s NO-RESULT %.2fs mbf_alive=%s' % (a[0], a[1], b[0], b[1], planner, dt, mbf_alive()), flush=True)
        return dict(outcome=-1, message='NO-RESULT(楔死)', L=0.0, dt=dt, n=0, wall=-1)
    r = ac.get_result(); p = r.path
    L = sum(math.dist((p.poses[k].pose.position.x, p.poses[k].pose.position.y),
                      (p.poses[k + 1].pose.position.x, p.poses[k + 1].pose.position.y))
            for k in range(len(p.poses) - 1)) if len(p.poses) > 1 else 0.0
    hits = sum(1 for q in p.poses if occ_at(q.pose.position.x, q.pose.position.y) >= 90)
    print('T1R2 (%.2f,%.2f)->(%.2f,%.2f) %s outcome=%2d "%s" 弧长=%7.2fm poses=%4d dt=%.3fs 穿墙点=%d mbf_alive=%s'
          % (a[0], a[1], b[0], b[1], planner, r.outcome, r.message[:24], L, len(p.poses), dt, hits, mbf_alive()), flush=True)
    return dict(outcome=r.outcome, message=r.message, L=round(L, 3), dt=round(dt, 4), n=len(p.poses), wall=hits)

st = json.load(open(STATE_F)) if os.path.exists(STATE_F) else {'done': []}
pairs = [(start, g) for g in goals] + [(g, start) for g in goals]
rows_all = []
for planner in ['GlobalPlanner', 'NavfnROS']:
    rows = []
    for i, (a, b) in enumerate(pairs):
        key = '%s#%02d' % (planner, i)
        if key in st['done']:
            print('T1R2 %s 已完成（前 cycle），跳过' % key, flush=True)
            continue
        time.sleep(1.0)
        r = call(a, b, planner)
        r['pair'] = key; r['planner'] = planner; r['idx'] = i; r['cycle'] = CY
        rows.append(r); rows_all.append(r)
        if r['outcome'] == -1:
            print('T1R2 %s NO-RESULT → 整栈重启同对公平重试' % key, flush=True)
            json.dump(st, open(STATE_F, 'w'))
            raise SystemExit(3)
        st['done'].append(key)
        json.dump(st, open(STATE_F, 'w'))
import csv
if rows_all:
    with open('/tmp/matrix_w3/logs/T1_perlin3d_r2_rows_c%s.csv' % CY, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['pair', 'planner', 'idx', 'cycle', 'outcome', 'message', 'L', 'dt', 'n', 'wall'])
        w.writeheader() if f.tell() == 0 else None
        w.writerows(rows_all)
if len(st['done']) < 16:
    print('T1R2 未采满 16 采数位（cycle 中断续跑）', flush=True)
    raise SystemExit(3)
# ---- 汇总判读（8 对 ×2 planner 轮）----
import csv as csv2
alldone = []
for c in range(1, 7):
    fp = '/tmp/matrix_w3/logs/T1_perlin3d_r2_rows_c%d.csv' % c
    try:
        alldone += list(csv2.DictReader(open(fp)))
    except IOError:
        pass
ok_g = sum(1 for r in alldone if r['planner'] == 'GlobalPlanner' and int(r['outcome']) == 0)
ok_n = sum(1 for r in alldone if r['planner'] == 'NavfnROS' and int(r['outcome']) == 0)
tw = sum(int(r['wall']) for r in alldone if int(r['outcome']) == 0)
Ls = [float(r['L']) for r in alldone if int(r['outcome']) == 0]
dts = [float(r['dt']) for r in alldone if int(r['outcome']) == 0]
print('---- perlin3d W3-R2 T1 汇总（8 对 ×2 planner）----')
print('T1R2 成功率 GlobalPlanner=%d/8 NavfnROS=%d/8 判据各≥8/8' % (ok_g, ok_n))
print('T1R2 弧长均值=%.2fm 范围=[%.2f,%.2f] 耗时均值=%.3fs max=%.3fs 穿墙合计=%d'
      % (sum(Ls) / len(Ls), min(Ls), max(Ls), sum(dts) / len(dts), max(dts), tw) if Ls else 'T1R2 无成功对')
raise SystemExit(0 if (ok_g == 8 and ok_n == 8 and tw == 0) else 1)
PYEOF
  RC=${PIPESTATUS[0]}
  echo "[w3r2-T1] cycle $CYCLE rc=$RC，退净..."
  cleanup
  if [ "$RC" -eq 0 ]; then FINAL=0; break; fi
  if [ "$RC" -eq 1 ] || [ "$RC" -eq 2 ]; then FINAL=$RC; break; fi
  FINAL=3
done
echo "[w3r2-T1] 终态 rc=$FINAL"
exit $FINAL
