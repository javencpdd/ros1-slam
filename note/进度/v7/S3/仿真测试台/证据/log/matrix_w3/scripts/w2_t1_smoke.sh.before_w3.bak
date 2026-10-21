#!/usr/bin/env bash
# w2_t1_smoke.sh —— 矩阵W2 每图 T1 冒烟（小明 2026-10-07）
# 门控（stageB G1–G4 同款思路：/map 有数据 + tf map→base_link + 全局 costmap 有数据 + getpath server 在位）
# → GetPath 组（勘察 start↔goals，use_start_pose=True，默认 planner=GlobalPlanner）
# → 判据（C 文 D 表 T1 行）：outcome=0 / 弧长 / 耗时 / 穿墙=0
# 用法: bash w2_t1_smoke.sh <mapname> <launch 附加参数...>
MAP=$1; shift
LOGDIR=/tmp/matrix_w2/logs
mkdir -p "$LOGDIR"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
echo "[w2] roslaunch mode_nav_sim.launch（附加参数: $*）"
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false "$@" > "$LOGDIR/T1_${MAP}_launch.log" 2>&1 &
echo "launch started"
sleep 2
set -o pipefail
python3 - "$MAP" <<'PYEOF' 2>&1 | tee "$LOGDIR/T1_${MAP}_getpath.log"
import sys, json, math, time, rospy, actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

name = sys.argv[1]
G = json.load(open('/tmp/matrix_w2/%s_goals.json' % name))
start, goals = G['start'], G['goals']
rospy.init_node('w2_t1_%s' % name)

# ---- 门控（总预算 240s wall；任一未过=FAIL 环境）----
budget = 240.0
t0 = time.time()
tf_ = tf_py.TransformListener()
def gate_wait(fn_name, fn):
    while time.time() - t0 < budget and not rospy.is_shutdown():
        try:
            if fn():
                print('GATE %s=PASS t=%.1fs' % (fn_name, time.time() - t0)); return True
        except Exception:
            pass
        time.sleep(0.5)
    print('GATE %s=FAIL t=%.1fs' % (fn_name, time.time() - t0)); return False
ok_map = gate_wait('map_data', lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
ok_tf  = gate_wait('tf_robot', lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
def costmap_up():
    try: return rospy.wait_for_message('/move_base_flex/global_costmap/costmap', OccupancyGrid, timeout=2.0).info.width > 0
    except Exception: return False
ok_gcm = gate_wait('gcostmap_data', costmap_up)
ok_srv = gate_wait('getpath_server', lambda: actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction).wait_for_server(rospy.Duration(2.0)))
if not all([ok_map, ok_tf, ok_gcm, ok_srv]):
    print('T1_%s gate=FAIL（环境未就绪，判据零污染）' % name); sys.exit(2)
mg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
res_, ox, oy, w_, h_ = mg.info.resolution, mg.info.origin.position.x, mg.info.origin.position.y, mg.info.width, mg.info.height
def occ_at(x, y):
    i, j = int((x - ox) / res_), int((y - oy) / res_)
    if not (0 <= i < w_ and 0 <= j < h_): return 100
    return mg.data[j * w_ + i]
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
def call(a, b):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b)
    g.tolerance = 0.0
    t0 = time.time(); rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(30))
    rosdt = (rospy.Time.now() - rt0).to_sec(); wall = time.time() - t0
    if not ok:
        ac.cancel_goal(); return dict(outcome=-1, message='30s 无 result', L=0.0, dt=rosdt, n=0, wall=-1)
    r = ac.get_result(); p = r.path
    L = sum(math.dist((p.poses[k].pose.position.x, p.poses[k].pose.position.y),
                      (p.poses[k + 1].pose.position.x, p.poses[k + 1].pose.position.y))
            for k in range(len(p.poses) - 1)) if len(p.poses) > 1 else 0.0
    hits = sum(1 for q in p.poses if occ_at(q.pose.position.x, q.pose.position.y) >= 90)
    return dict(outcome=r.outcome, message=r.message, L=L, dt=rosdt, n=len(p.poses), wall=hits)
rows = []
pairs = [(start, g) for g in goals] + [(g, start) for g in goals]
for i, (a, b) in enumerate(pairs):
    r = call(a, b)
    rows.append(r)
    print('PAIR%02d (%.2f,%.2f)->(%.2f,%.2f) outcome=%2d %-24s 弧长=%7.2fm poses=%4d 耗时=%.3fs 穿墙点=%d'
          % (i, a[0], a[1], b[0], b[1], r['outcome'], r['message'][:24], r['L'], r['n'], r['dt'], r['wall']), flush=True)
ok = sum(1 for r in rows if r['outcome'] == 0)
Ls = [r['L'] for r in rows if r['outcome'] == 0]; dts = [r['dt'] for r in rows if r['outcome'] == 0]
tw = sum(r['wall'] for r in rows if r['outcome'] == 0)
print('---- %s 汇总 ----' % name)
print('T1_%s 成功率=%d/%d (%.0f%%) 判据≥90%%' % (name, ok, len(rows), 100.0 * ok / len(rows)))
if Ls:
    print('T1_%s 弧长均值=%.2fm 范围=[%.2f,%.2f] 耗时均值=%.3fs max=%.3fs 穿墙合计=%d'
          % (name, sum(Ls) / len(Ls), min(Ls), max(Ls), sum(dts) / len(dts), max(dts), tw))
import csv
with open('/tmp/matrix_w2/T1_%s_results.csv' % name, 'w', newline='') as f:
    wcsv = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wcsv.writeheader(); wcsv.writerows(rows)
PYEOF
RC=$?
echo "[w2] T1 冒烟采数完成 rc=$RC，退净中..."
rosnode kill -a 2>/dev/null; sleep 3
pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null
sleep 2
sleep 2
bash -c 'unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313
rosnode list 2>/dev/null | grep -v "^/rosout$" | grep -v "^/survey_mockamap$" | wc -l' | xargs echo "[w2] 残留节点数(除 rosout):"
[ -f "$LOGDIR/T1_${MAP}_getpath.log" ] && grep -E "gate_.*=FAIL|成功率" "$LOGDIR/T1_${MAP}_getpath.log" | tail -2
exit ${RC:-1}
