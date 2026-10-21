#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════════════════
# sim_nav_smoke.sh —— mode_nav_sim 仿真测试台一键算法测试冒烟（小张 2026-10-05，payload C）
# 用法（无人值守，前台跑完自动退净）：
#   source /home/ros1-slam/ws/devel/setup.bash   # 或各自的环境 source 口径
#   roscore -p 11312 &                            # 若无共享 master
#   export ROS_MASTER_URI=http://localhost:11312
#   bash verify/sim_nav_smoke.sh
# 判据链（全部有界超时，任一 FAIL 即汇总标 FAIL，退出码 1）：
#   1) 环境：ws 已 source、master 可达、无残留 /move_base_flex、ros2qnx 全程不存在（真机红线 G5-①）
#   2) 起量：/map_generator/global_cloud 有发布者 → /rslidar_points ≥0.5Hz（CPU 渲染分支实测 ~1.5Hz 封顶）
#      → /map 有数据（真值图桥）→ TF map→base_link 可查 → 全局 costmap 有占据格
#   3) 规划正确性：GetPath ×3（目标=traj_maze 29 格蛇形路径上的走廊格心，迷宫树拓扑任意在路径位姿均可达）
#      → 逐个 outcome=0 且 末点≈目标（≤0.35m）且 路径长∈[0.5, 4×直线距+2]m
#   4) 控制活性：ExePath 冒烟 → /cmd_vel_raw 12s 内有输出（闭环不可仿真：cmd_vel 不驱动仿真狗，只验规划链活性）
#      → /cmd_vel publisher 唯一 = M6（§2.6-a）
#   5) 退净：roslaunch SIGINT → 无 /move_base_flex、/pcl_render_node、/sim_adapter 残留，ros2qnx 仍为空
# 退出码：0=全 PASS；1=有 FAIL。日志：/tmp/sim_nav_smoke_launch.log（launch 全量）、本脚本 stdout（CHECK 摘要）。
# ══════════════════════════════════════════════════════════════════════════════════════════
set -u
LOG=/tmp/sim_nav_smoke_launch.log

rospack find lite3_bringup >/dev/null 2>&1 || { echo "CHECK|env_sourced|FAIL|ws 未 source（rospack 找不到 lite3_bringup）"; exit 1; }
timeout 5 rosnode list >/dev/null 2>&1 || { echo "CHECK|master|FAIL|master 不可达（先 roscore / 核对 ROS_MASTER_URI）"; exit 1; }
timeout 5 rosnode list 2>/dev/null | grep -q "^/move_base_flex$" && { echo "CHECK|leftover|FAIL|残留 /move_base_flex，先退净再跑"; exit 1; }

echo "[smoke] 起 mode_nav_sim.launch（日志 $LOG）..."
setsid roslaunch /opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r467_单块矩形靶活体窗/mode_nav_sim_r467.launch rviz:=false use_gpu:=false >"$LOG" 2>&1 &
LAUNCH_PGID=$!
cleanup() {
  echo "[smoke] 退净中..."
  kill -INT -- -"$LAUNCH_PGID" 2>/dev/null
  for i in $(seq 1 20); do timeout 2 rosnode list 2>/dev/null | grep -qE "^/(move_base_flex|pcl_render_node|sim_adapter)$" || break; sleep 1; done
  for n in move_base_flex pcl_render_node sim_adapter sim_pose_tf sim_map_bridge; do timeout 3 rosnode kill "/$n" >/dev/null 2>&1; done
  kill -KILL -- -"$LAUNCH_PGID" 2>/dev/null
  timeout 5 rosnode list 2>/dev/null | grep -q "^/ros2qnx$" && echo "CHECK|ros2qnx_exit|FAIL|退出后出现 ros2qnx（真机红线！留证上报）" || echo "CHECK|ros2qnx_exit|PASS|ros2qnx 全程不存在"
  timeout 5 rosnode list 2>/dev/null | grep -qE "^/(move_base_flex|pcl_render_node|sim_adapter)$" && echo "CHECK|teardown|FAIL|有节点残留" || echo "CHECK|teardown|PASS|launch 内关键节点已退净"
}
trap cleanup EXIT

python3 - <<'PY'
import math, subprocess, sys
import rospy, tf, actionlib
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from mbf_msgs.msg import GetPathAction, GetPathGoal, ExePathAction, ExePathGoal

R = []
def chk(name, ok, detail):
    R.append(ok)
    print('CHECK|%s|%s|%s' % (name, 'PASS' if ok else 'FAIL', detail)); sys.stdout.flush()

def conn_wait(sub, n, tmo):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < tmo:
        if sub.get_num_connections() > 0: return True
        rospy.sleep(0.5)
    return False

rospy.init_node('sim_nav_smoke')

# —— 2) 起量 ——
sub_cloud = rospy.Subscriber('/map_generator/global_cloud', PointCloud2, lambda m: None, queue_size=1)
chk('venue_map_cloud', conn_wait(sub_cloud, 0, 30), '/map_generator/global_cloud 有发布者（mockamap）')
cnt = {'n': 0, 'first': None}
sub_pts = rospy.Subscriber('/rslidar_points', PointCloud2, lambda m: cnt.update(n=cnt['n'] + 1, first=rospy.Time.now()), queue_size=1)
t0 = rospy.Time.now()
while not rospy.is_shutdown() and cnt['n'] < 1 and (rospy.Time.now() - t0).to_sec() < 120: rospy.sleep(0.5)
n0 = cnt['n']
rospy.sleep(8.0)
hz = (cnt['n'] - n0) / 8.0
chk('rslidar_points', hz >= 0.5, '/rslidar_points 实测 %.2fHz（≥0.5，G8 适配链活性）' % hz)
try:
    m = rospy.wait_for_message('/map', OccupancyGrid, timeout=25)
    occ = sum(1 for v in m.data if v >= 90)
    chk('truth_map', occ > 0, '/map 有数据且占据格=%d（sim_map_bridge）' % occ)
except rospy.ROSException:
    chk('truth_map', False, '/map 25s 未到（sim_map_bridge 未出图）')
tf_ = tf.TransformListener()
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not tf_.canTransform('map', 'base_link', rospy.Time(0)) and (rospy.Time.now() - t0).to_sec() < 15: rospy.sleep(0.3)
chk('tf_robot', tf_.canTransform('map', 'base_link', rospy.Time(0)), 'TF map→base_link 可查（sim_pose_tf）')
try:
    cm = rospy.wait_for_message('/move_base_flex/global_costmap/costmap', OccupancyGrid, timeout=45)
    occ2 = sum(1 for v in cm.data if v >= 90)
    chk('costmap_data', occ2 > 0, '全局 costmap 占据格=%d（static+stvl 有数据）' % occ2)
except rospy.ROSException:
    chk('costmap_data', False, '全局 costmap 45s 未出（mbf 未起/costmap 未就绪）')

# —— 3) GetPath ×3（目标=traj_maze.yaml 29 格蛇形路径上的格心，走廊内可达位形）——
GOALS = [(-1.85, 0.55), (0.55, 2.95), (2.95, 4.15)]
gp = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
chk('getpath_server', gp.wait_for_server(rospy.Duration(30)), 'mbf get_path action server 在位')
last_path = None
for i, (gx, gy) in enumerate(GOALS, 1):
    try:
        rp = tf_.lookupTransform('map', 'base_link', rospy.Time(0))
        rx, ry = rp[0][0], rp[0][1]
    except Exception:
        rx = ry = None
    g = GetPathGoal()
    g.target_pose = PoseStamped()
    g.target_pose.header.frame_id, g.target_pose.header.stamp = 'map', rospy.Time(0)
    g.target_pose.pose.position.x, g.target_pose.pose.position.y = gx, gy
    g.target_pose.pose.orientation.w = 1.0
    gp.send_goal(g)
    ok = gp.wait_for_result(rospy.Duration(20))
    res = gp.get_result() if ok else None
    if not ok or res is None:
        chk('getpath_%d' % i, False, 'goal(%s,%s) 20s 无 result' % (gx, gy)); continue
    p = res.path
    L = sum(math.dist((p.poses[k].pose.position.x, p.poses[k].pose.position.y),
                      (p.poses[k + 1].pose.position.x, p.poses[k + 1].pose.position.y))
            for k in range(max(0, len(p.poses) - 1))) if p.poses else 0.0
    end = p.poses[-1].pose.position if p.poses else None
    dev = math.dist((end.x, end.y), (gx, gy)) if end else 9e9
    straight = math.dist((rx, ry), (gx, gy)) if rx is not None else 10.0
    # D-仿真可用化-3：4×直线距 对完美迷宫树拓扑不成立（run2 实测直线距3.2m/走廊路径22.3m 仍正确），
    # 下限钳到蛇形单向长 33.6m（95_sim.launch traj_maze 67.2m/2=迷宫树直径，任意两路径位姿走廊距≤直径）
    okp = (res.outcome == 0 and dev <= 0.35 and 0.5 <= L <= max(4.0 * straight, 33.6) + 2.0)
    chk('getpath_%d' % i, okp, 'outcome=%d %s | 末点偏差=%.2fm 长度=%.1fm（当前位姿直线距=%.1fm，判据≤%.1fm）'
        % (res.outcome, res.message, dev, L, straight, max(4.0 * straight, 33.6) + 2.0))
    last_path = p if res.outcome == 0 else last_path

# —— 4) ExePath 冒烟：/cmd_vel_raw 有输出 ——
if last_path is not None:
    ep = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
    if not ep.wait_for_server(rospy.Duration(15)):
        chk('exepath_cmdvel', False, 'exe_path action server 不在位')
    else:
        cv = {'n': 0}
        sub_cv = rospy.Subscriber('/cmd_vel_raw', Twist, lambda m: cv.update(n=cv['n'] + 1), queue_size=20)
        eg = ExePathGoal(); eg.path = last_path
        ep.send_goal(eg)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and cv['n'] < 1 and (rospy.Time.now() - t0).to_sec() < 12: rospy.sleep(0.2)
        ep.cancel_goal()
        chk('exepath_cmdvel', cv['n'] >= 1, '/cmd_vel_raw 12s 内消息数=%d（M4→M6 速度链活性；闭环不可仿真，不验运动效果）' % cv['n'])
        rospy.sleep(1.0)
        sub_cv.unregister()
else:
    chk('exepath_cmdvel', False, '无可用 GetPath 路径，ExePath 跳过')

# —— /cmd_vel 写者唯一 = M6 ——
try:
    info = subprocess.check_output(['rostopic', 'info', '/cmd_vel'], timeout=8, stderr=subprocess.DEVNULL).decode()
    sec = 'P'; pubs_cmdvel = []
    for l in info.splitlines():
        if l.startswith('Publishers'): sec = 'P'
        elif l.startswith('Subscribers'): sec = 'S'
        elif l.startswith(' * ') and sec == 'P': pubs_cmdvel.append(l.strip()[2:])
    chk('cmdvel_writer', len(pubs_cmdvel) == 1, '/cmd_vel publisher=%s（唯一写者=M6 safe_cmd_vel_mux，§2.6-a）' % pubs_cmdvel)
except Exception as e:
    chk('cmdvel_writer', False, 'rostopic info /cmd_vel 失败: %s' % e)

sys.exit(0 if all(R) else 1)
PY
PYRC=$?

echo "[smoke] launch 日志尾 20 行（失败时排查用）："; tail -20 "$LOG"
exit $PYRC
