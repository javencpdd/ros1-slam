#!/usr/bin/env bash
# w3_t1_probe.sh —— perlin3d 停手前设计输入探针（小明 2026-10-07）
# 探针（诊断，非判据采数）：远角对在 ①GlobalPlanner+patience 60s（dynparam 运行时放宽，探「慢 vs 不可解」）
# ②NavfnROS 按名点单（探「GlobalPlanner 特有 vs 图难」）。零文件改动，dynparam 用后即复。
LOGDIR=/tmp/matrix_w3/logs
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
LOGPROBE="$LOGDIR/probe_far_corner_launch.log"
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
  perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
  map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGPROBE" 2>&1 &
sleep 2
python3 - <<'PYEOF' 2>&1 | tee "$LOGDIR/probe_far_corner_getpath.log"
import time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

rospy.init_node('w3_t1_probe')
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
print('PROBE gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0))
if not (g1 and g2 and g3): raise SystemExit(2)
def dynparam(name, val):
    subprocess.run(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base_flex', name, val],
                   capture_output=True, timeout=20)
mbf_alive = lambda: b'move_base_flex' in subprocess.run(['rosnode', 'list'], capture_output=True, timeout=15).stdout
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
def call(tag, a, b, planner, tmo):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    g.planner = planner
    t = time.time(); rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(tmo))
    dt = (rospy.Time.now() - rt0).to_sec()
    if ok:
        r = ac.get_result()
        L = sum(math.dist((p.pose.position.x, p.pose.position.y), (q.pose.position.x, q.pose.position.y))
                for p, q in zip(r.path.poses, r.path.poses[1:])) if len(r.path.poses) > 1 else 0.0
        print('PROBE %s planner=%s outcome=%d "%s" poses=%d 弧长=%.1fm rosdt=%.2fs' % (tag, planner, r.outcome, r.message[:30], len(r.path.poses), L, dt))
    else:
        ac.cancel_goal()
        print('PROBE %s planner=%s NO-RESULT in %.0fs' % (tag, planner, tmo))
    print('PROBE mbf_alive=%s' % mbf_alive())
import math
S = (0.05, 0.05); G1 = (-19.55, -9.55)
# ① GlobalPlanner + patience 60s
dynparam('planner_patience', '60')
call('P1_gp_patience60', S, G1, 'GlobalPlanner', 90.0)
dynparam('planner_patience', '3')   # 即复
# ② NavfnROS 按名点单（patience 3.0 原值）
call('P2_navfn_patience3', S, G1, 'NavfnROS', 30.0)
call('P3_navfn_rev', G1, S, 'NavfnROS', 30.0)
print('PROBE planner_patience restored=3')
PYEOF
RC=$?
echo "[w3-probe] rc=$RC 退净..."
rosnode kill -a 2>/dev/null; sleep 3
pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null
sleep 2
grep -n "planner_patience" /home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/move_base_params.yaml
exit $RC
