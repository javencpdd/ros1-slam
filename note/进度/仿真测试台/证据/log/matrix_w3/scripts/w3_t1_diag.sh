#!/usr/bin/env bash
# w3_t1_diag.sh —— perlin3d 0/8 最小复现诊断（小明 2026-10-07）
# 序：W2 已知好对（in-box 7.9m）→ 反向 → 远角对 PAIR00 复现 → mbf 存活探针 → 第二远角对
# 目的：区分「planner 能力病态」与「环境/栈故障」，非判据采数。
MAP=perlin3d
LOGDIR=/tmp/matrix_w3/logs
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
LOGDIAG="$LOGDIR/diag_perlin3d_launch.log"
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
  perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
  map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGDIAG" 2>&1 &
sleep 2
python3 - <<'PYEOF' 2>&1 | tee "$LOGDIR/diag_perlin3d_getpath.log"
import math, time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

rospy.init_node('w3_t1_diag')
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
print('DIAG gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0))
if not (g1 and g2 and g3): raise SystemExit(2)
mbf_alive = lambda: b'move_base_flex' in subprocess.run(['rosnode', 'list'], capture_output=True, timeout=15).stdout
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
def call(tag, a, b, tmo=40.0):
    alive0 = mbf_alive()
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    t = time.time(); rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(tmo))
    dt = (rospy.Time.now() - rt0).to_sec()
    if ok:
        r = ac.get_result()
        n = len(r.path.poses)
        print('DIAG %s (%.2f,%.2f)->(%.2f,%.2f) outcome=%d "%s" poses=%d rosdt=%.2fs' % (tag, a[0], a[1], b[0], b[1], r.outcome, r.message[:30], n, dt))
    else:
        ac.cancel_goal()
        print('DIAG %s (%.2f,%.2f)->(%.2f,%.2f) NO-RESULT in %.0fs' % (tag, a[0], a[1], b[0], b[1], tmo))
    alive1 = mbf_alive()
    print('DIAG mbf_alive before=%s after=%s' % (alive0, alive1))
    return alive1
A = (-2.15, -8.45); B = (-7.55, -7.05)   # W2 已知好对（8/8 过，7.9m）
S = (0.05, 0.05); G1 = (-19.55, -9.55); G2 = (19.65, -9.75)  # W3 远角对
if not call('A_known_good', A, B): raise SystemExit(3)
if not call('B_known_good_rev', B, A): raise SystemExit(3)
c_alive = call('C_far_corner_PAIR00', S, G1)
if not c_alive:
    print('DIAG VERDICT: mbf 死于远角对 C —— 栈侧病态实证'); raise SystemExit(4)
call('D_far_corner_PAIR01', S, G2)
print('DIAG done, mbf_alive=%s' % mbf_alive())
PYEOF
RC=$?
echo "[w3-diag] rc=$RC 退净..."
rosnode kill -a 2>/dev/null; sleep 3
pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null
sleep 2
grep -aE "patience|exceeded|died|terminate|lock_error" "$LOGDIAG" | tail -6
exit $RC
