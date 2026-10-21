#!/usr/bin/env bash
# w3_t1_probe2.sh —— perlin3d 停手前设计输入探针②（小明 2026-10-07）
# P2/P3: NavfnROS 远角对正反向（探 GlobalPlanner 特有 vs 图难）；P4: GlobalPlanner 中程对（定位触发域）
# 零文件改动、零 dynparam（上一轮 dynparam 已实证会卡死在 mbf reconfigure 锁上）。
LOGDIR=/tmp/matrix_w3/logs
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
LOGPROBE="$LOGDIR/probe2_far_corner_launch.log"
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
  perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
  map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGPROBE" 2>&1 &
sleep 2
python3 - <<'PYEOF' 2>&1 | tee "$LOGDIR/probe2_far_corner_getpath.log"
import math, time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

rospy.init_node('w3_t1_probe2')
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
print('PROBE2 gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0))
if not (g1 and g2 and g3): raise SystemExit(2)
mbf_alive = lambda: b'move_base_flex' in subprocess.run(['rosnode', 'list'], capture_output=True, timeout=15).stdout
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
def call(tag, a, b, planner, tmo):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    g.planner = planner
    rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(tmo))
    dt = (rospy.Time.now() - rt0).to_sec()
    if ok:
        r = ac.get_result()
        L = sum(math.dist((p.pose.position.x, p.pose.position.y), (q.pose.position.x, q.pose.position.y))
                for p, q in zip(r.path.poses, r.path.poses[1:])) if len(r.path.poses) > 1 else 0.0
        print('PROBE2 %s planner=%s outcome=%d "%s" poses=%d 弧长=%.1fm rosdt=%.2fs' % (tag, planner, r.outcome, r.message[:30], len(r.path.poses), L, dt))
    else:
        ac.cancel_goal()
        print('PROBE2 %s planner=%s NO-RESULT in %.0fs' % (tag, planner, tmo))
    print('PROBE2 mbf_alive=%s' % mbf_alive())
S = (0.05, 0.05); G1 = (-19.55, -9.55); MID = (-9.75, -4.75)
call('P2_navfn_far', S, G1, 'NavfnROS', 30.0)
call('P3_navfn_far_rev', G1, S, 'NavfnROS', 30.0)
call('P4_gp_mid_11m', S, MID, 'GlobalPlanner', 30.0)
PYEOF
RC=$?
echo "[w3-probe2] rc=$RC 退净..."
rosnode kill -a 2>/dev/null; sleep 3
pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null
sleep 2
exit $RC
