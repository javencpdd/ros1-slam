#!/usr/bin/env bash
# w3r2_healthcheck.sh —— W3-R2 接手健康校验（小明 2026-10-07）
# 背景：18:48 probe_rev KG 对 0.05s 通过；19:11–19:33（旧 master 346893 残留时段）全 55/楔死。
# 本脚本在已清空的现场起一版干净栈，验证：(a) 栈健康（KG 对恢复 0.05–0.15s）；
# (b) 18:30 实证过的环带快候选 (-0.75,-7.85)（d=7.86m，GP fwd 0.061s）能否复现。
# 其反向在 18:30 曾 NO-RESULT 楔栈 → 放最后，楔了也不影响前序判读。
# 零 yaml diff、零 dynparam；launch 参数与 w3r2_preverify.sh 逐字一致。
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

echo "[hc] 起栈"
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
  perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
  map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGDIR/w3r2_healthcheck_launch.log" 2>&1 &
sleep 2
python3 - <<'PYEOF' 2>&1 | tee "$LOGDIR/w3r2_healthcheck_getpath.log"
import time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

rospy.init_node('w3r2_hc')
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
print('HC gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0), flush=True)
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
def call(tag, a, b, planner='GlobalPlanner'):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    g.planner = planner
    rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(8))
    dt = (rospy.Time.now() - rt0).to_sec()
    if not ok:
        ac.cancel_goal()
        print('HC %s %s (%.2f,%.2f)->(%.2f,%.2f) NO-RESULT %.2fs mbf_alive=%s' % (tag, planner, a[0], a[1], b[0], b[1], dt, mbf_alive()), flush=True)
        return False
    r = ac.get_result()
    L = sum(((p.pose.position.x - q.pose.position.x) ** 2 + (p.pose.position.y - q.pose.position.y) ** 2) ** 0.5
            for p, q in zip(r.path.poses, r.path.poses[1:])) if len(r.path.poses) > 1 else 0.0
    print('HC %s %s (%.2f,%.2f)->(%.2f,%.2f) outcome=%d "%s" poses=%d L=%.2f dt=%.3fs mbf_alive=%s'
          % (tag, planner, a[0], a[1], b[0], b[1], r.outcome, r.message[:24], len(r.path.poses), L, dt, mbf_alive()), flush=True)
    return r.outcome == 0 and dt <= 1.0

S = (0.05, 0.05)
KGa, KGb = (-2.15, -8.45), (-7.55, -7.05)   # 已知好对 7.8m 弧长（diag 18:30 实证 0.05–0.15s）
C0 = (-0.75, -7.85)                          # 18:30 首轮预验 GP fwd 0.061s 实证环带候选
r1 = call('H1_kg_fwd', KGa, KGb)
time.sleep(1.0); r2 = call('H2_kg_rev', KGb, KGa)
time.sleep(1.0); r3 = call('H3_c0_fwd', S, C0)
time.sleep(1.0); r4 = call('H4_c0_rev', C0, S)
print('HC SUMMARY kg_fwd=%s kg_rev=%s c0_fwd=%s c0_rev=%s' % (r1, r2, r3, r4), flush=True)
raise SystemExit(0 if (r1 and r2 and r3 and r4) else 1)
PYEOF
RC=${PIPESTATUS[0]}
echo "[hc] rc=$RC 退净中..."
cleanup
exit $RC
