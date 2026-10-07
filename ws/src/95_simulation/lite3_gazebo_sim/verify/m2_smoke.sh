#!/usr/bin/env bash
# m2_smoke.sh —— M2 无人值守冒烟（小张，2026-10-06；m1_smoke.sh 同构扩展）
# Stage A（M2-1/M2-4）：闭环回归四判据 —— IMU 姿态反馈转向 + odom 位置反馈回程；drift ≤0.5m、turn/back 双 done、
#                       /rslidar_points ≥5Hz、/livox/imu ≥100Hz、飞点 <1%
# Stage B（M2-2）     ：teb 物理闭环 —— goal=(start+3m,0)，sim 时 ≤120s 到达偏差 ≤0.35m（gt 参考）
# Stage C（M2-3）     ：hdl 重定位 —— 建图存 sim_venue.pcd（样张备份/恢复 sha256 红线）→ /initialpose 注入
#                       → ≤10s 收敛 ≤0.3m/10°，30s 观察窗无 >0.5m 单步跳变
# 隔离域：ROS master 11313 + GAZEBO_MASTER_URI=11345（本会话 P10 仲裁值，见 L21 实际 export）；生产 11312 全程零监听；foxy 栈零接触。
# 用法：bash /tmp/m2_payload/m2_smoke.sh [stageA|stageB|stageC|all]（缺省 all）
set -u
PAYLOAD=/tmp/m2_payload
SIM_PKG=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
MAP=/home/ros1-slam/data/maps/lite3.pcd
VENUE=/home/ros1-slam/data/maps/sim_venue.pcd
STAGE=${1:-all}
LOGDIR="$PAYLOAD/logs-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"; echo "[m2] 日志目录: $LOGDIR"

source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

PASS=1
INVALID=0  # A0 前置：运动窗 RTF<0.25（M2R2 主理人裁决 0.3→0.25：D-01 后 200Hz IMU 正确配置合理 RTF 平台 0.288，门意抓塌缩非惩罚正确配置）→ 本轮作废重跑，不计 FAIL
mark() { [ "$1" = "1" ] && echo "[PASS] $2" || { echo "[FAIL] $2"; PASS=0; }; }
wait_topic() {  # D-M2-P4（D-M1-3 同款修法移植）：启动窗内单次长 echo 挂起/吞 TERM（M1 实证 4/4）；list 门控+短 echo 反复重试
  local t=0
  while [ "$t" -lt "$2" ]; do
    if rostopic list 2>/dev/null | grep -qx -- "$1"; then
      timeout 15 rostopic echo -n 1 "$1" >/dev/null 2>&1 && return 0
    fi
    sleep 2; t=$((t+17))
  done
  return 1
}
cleanup() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f hdl_nodelet 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  # [C8 返工修正 2026-10-08] 入口改指 TOP 后命令为 `roslaunch lite3_bringup mode_sim_*.launch`，
  #   不含 "m2_" ⇒ 原模式漏网（stageB 活体回归实测：roslaunch+rosmaster 存活、11313 未释放）。
  #   保留旧模式兼容历史跑法，另补 mode_sim_ 与 rosmaster 兜底。
  pkill -f "roslaunch.*m2_" 2>/dev/null; pkill -f "roslaunch.*mode_sim_" 2>/dev/null
  pkill -f "rosmaster --core -p 11313" 2>/dev/null; sleep 2
  # 样张恢复兜底（M2-3 建图阶段万一中断）
  if [ -f "$LOGDIR/lite3.pcd.bak" ] && [ -f "$MAP" ] && ! cmp -s "$LOGDIR/lite3.pcd.bak" "$MAP"; then
    cp "$LOGDIR/lite3.pcd.bak" "$MAP"; echo "[m2] 样张 pcd 已从备份恢复"; fi
  true; }
trap cleanup EXIT

if [ -f "$MAP" ]; then cp "$MAP" "$LOGDIR/lite3.pcd.bak"
  sha256sum "$MAP" | awk '{print $1}' > "$LOGDIR/pcd.sha256.before"; echo "[m2] 样张 pcd 已备份（O2 勘误：比对前移至此）"; fi

# [C8 返工 2026-10-08 主理人] 六个 roslaunch 入口由 $PAYLOAD(/tmp 载荷副本) 改指 ws/src 包内路径：
#   (1) 防复验跑到过期副本（m2_teb.launch 已因 [6] 改名与 [5] include 上收而变，/tmp 侧仍是旧版）
#   (2) teb/hdl 改指 TOP 编排件 lite3_bringup/mode_sim_*.launch（[5] 返工；判据/阈值/stage 数/进程拓扑零改动）

# ============ Stage A：闭环回归（M2-1/M2-4） ============
stage_a() {
  roslaunch lite3_gazebo_sim m2_gazebo.launch > "$LOGDIR/gazeboA.log" 2>&1 &
  # 注：m2_gazebo.launch 亦 install 至包内 launch/（$(find lite3_gazebo_sim)/launch/m2_gazebo.launch），二选一等价
  wait_topic /clock 300 || { echo "FAIL A: /clock 未出"; return 1; }
  wait_topic /rslidar_points 480 || { echo "FAIL A: /rslidar_points 未出"; return 1; }
  wait_topic /livox/imu 300 || { echo "FAIL A: /livox/imu 未出"; return 1; }
  roslaunch lite3_gazebo_sim m2_mapping.launch > "$LOGDIR/mappingA.log" 2>&1 &
  wait_topic /cloud_registered 360 || { echo "FAIL A: /cloud_registered 未出"; return 1; }

  python3 - <<'PYEOF' > "$LOGDIR/stageA.txt" 2>&1
import math, time
import numpy as np, rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from gazebo_msgs.msg import ModelStates
rospy.init_node('m2_stage_a')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
S = dict(odom=None, gt=None, g0=None, iq=None,
         mot=0, pts_m=0, imu_m=0, pts_t0=None, pts_t1=None, imu_t0=None, imu_t1=None, m0_wall=None,
         fly_tot=0, fly_bad=0, fly_frames=0)
def on_pts(m):  # F4（M2-R1）：仅运动窗计数 + 消息头 stamp 差分（sim-time 分母，剔除启动/settle 段）
    if not S['mot']: return
    S['pts_m'] += 1; st = m.header.stamp.to_sec()
    if S['pts_t0'] is None: S['pts_t0'] = st
    S['pts_t1'] = st
def on_imu(m):  # F4：iq 全程更新（供初始等待/转向判据）；频率计数仅运动窗
    S['iq'] = m.orientation
    if not S['mot']: return
    S['imu_m'] += 1; st = m.header.stamp.to_sec()
    if S['imu_t0'] is None: S['imu_t0'] = st
    S['imu_t1'] = st
rospy.Subscriber('/Odometry', Odometry, lambda m: S.update(odom=m), queue_size=2)
rospy.Subscriber('/rslidar_points', PointCloud2, on_pts, queue_size=1)
rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=1)
rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m: [S.update(gt=(p.position.x, p.position.y))
    for p, n in zip(m.pose, m.name) if n=='lite3_planar'] and None, queue_size=1)
DZ = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('pad','V20')])
def on_cloud(m):
    if not m.width: return
    S['fly_frames'] += 1; S['fly_tot'] += m.width
    a = np.frombuffer(bytes(m.data), dtype=DZ, count=m.width)
    S['fly_bad'] += int((np.abs(a['z'])>10.0).sum())
rospy.Subscriber('/cloud_registered', PointCloud2, on_cloud, queue_size=1)
def qyaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def drive(vx, wz): t=Twist(); t.linear.x=vx; t.angular.z=wz; pub.publish(t)
w0=time.time()
while (S['odom'] is None or S['gt'] is None or S['iq'] is None) and time.time()-w0<60: time.sleep(0.1)
assert S['odom'] is not None, 'no /Odometry'
p0=S['odom'].pose.pose.position; sx,sy=p0.x,p0.y; i_yaw0=qyaw(S['iq']); S['g0']=S['gt']
print('SMOKE start=(%.3f,%.3f) imu_yaw0=%.3f' % (sx,sy,i_yaw0))
def phase(name, vx, wz, done, wall_limit=150.0):
    w0=time.time()
    while not rospy.is_shutdown():
        if done(): drive(0,0); print('SMOKE phase_%s_done'%name); return True
        if time.time()-w0>wall_limit: drive(0,0); print('SMOKE phase_%s_TIMEOUT'%name); return False
        drive(vx,wz); time.sleep(0.05)
    return False
# ★ M2-1 修法 A 语义：转向判据=IMU 物理姿态反馈（不再信 odom yaw 开环值）；回程=位置反馈
S['mot']=1; S['m0_wall']=time.time()  # F4：运动窗起点（墙钟，供 A0 RTF 前置判据）
ok_out = phase('fwd_out', 0.5, 0.0, lambda: S['odom'].pose.pose.position.x - sx >= 4.85)
ok_turn = phase('turn180', 0.0, 0.6, lambda: math.degrees(abs(qyaw(S['iq'])-i_yaw0)) >= 178.0)
ok_back = phase('fwd_back', 0.5, 0.0, lambda: math.hypot(S['odom'].pose.pose.position.x-sx,
                                                          S['odom'].pose.pose.position.y-sy) <= 0.5)  # M2R2 主理人裁决 done 窗 0.25→0.5m（转向接触非确定残差 ±2-4° 在 4.85m 直线天然 ±0.34m 侧偏；与 A③ 同尺度，A③ 本体不动）
S['mot']=0
mw_wall = max(time.time()-S['m0_wall'], 0.001)  # F4：运动窗墙钟宽
t0=rospy.get_rostime(); rospy.sleep(5.0); t1=rospy.get_rostime()
m=S['odom']; drift=math.hypot(m.pose.pose.position.x-sx, m.pose.pose.position.y-sy)
g=S['gt']; gdrift=math.hypot(g[0]-S['g0'][0], g[1]-S['g0'][1])
# F4（M2-R1）：A①② 分母改运动窗 sim-time（消息头 stamp 差分），剔除启动/settle 段；
#   阈值不变（口径 bug 修复，非放宽）；旧口径=全程计数÷5s settle 窗（rate_pts 曾虚标 200.2/20.6）。
pts_span = (S['pts_t1']-S['pts_t0']) if (S['pts_t0'] is not None and S['pts_t1'] is not None) else 0.0
imu_span = (S['imu_t1']-S['imu_t0']) if (S['imu_t0'] is not None and S['imu_t1'] is not None) else 0.0
rate_pts = S['pts_m']/pts_span if pts_span > 0 else 0.0
rate_imu = S['imu_m']/imu_span if imu_span > 0 else 0.0
rtf_m = imu_span/mw_wall if imu_span > 0 else 0.0  # 运动窗 RTF（A0 前置判据，<0.25 记 INVALID）
print('SMOKE drift=%.3f gt_drift=%.3f rate_pts=%.1f rate_imu=%.1f fly_ratio=%.4f rtf_motion=%.3f mw_sim=%.1f mw_wall=%.1f phases out=%s turn=%s back=%s'
      % (drift, gdrift, rate_pts, rate_imu, S['fly_bad']/max(S['fly_tot'],1), rtf_m, imu_span, mw_wall, ok_out, ok_turn, ok_back))
PYEOF
  cat "$LOGDIR/stageA.txt"
  get() { grep -o "$1=[-0-9.e]*" "$LOGDIR/stageA.txt" | head -1 | cut -d= -f2; }
  grep -q "phase_fwd_back_done" "$LOGDIR/stageA.txt" && mark 1 "A: 闭环走完（out/turn/back 全 done）" || mark 0 "A: 闭环未走完"
  le() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a<=b)}'; }; ge() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a>=b)}'; }
  D=$(get drift); RP=$(get rate_pts); RI=$(get rate_imu); FR=$(get fly_ratio)
  le "$D" 0.5 && mark 1 "A③(M2-4): 漂移 $D ≤0.5" || mark 0 "A③(M2-4): 漂移 $D >0.5"
  ge "$RP" 5 && mark 1 "A①: 点云 $RP ≥5Hz" || mark 0 "A①: 点云 $RP <5Hz"
  ge "$RI" 100 && mark 1 "A②: IMU $RI ≥100Hz" || mark 0 "A②: IMU $RI <100Hz"
  le "$FR" 0.01 && mark 1 "A④: 飞点 $FR <1%" || mark 0 "A④: 飞点 $FR ≥1%"
  RF=$(get rtf_motion)  # A0 前置判据（M2R2 主理人裁决：门 0.3→0.25，依据=D-01 后 200Hz IMU 正确配置合理平台 0.288）
  if [ -n "$RF" ]; then
    ge "$RF" 0.25 && mark 1 "A0(前置): 运动窗 RTF $RF ≥0.25" || { echo "[INVALID] A0: 运动窗 RTF $RF <0.25 —— 本轮作废重跑（不计 FAIL）"; INVALID=1; }
  else
    echo "[INVALID] A0: rtf_motion 缺失（运动窗无消息 stamp）—— 本轮作废重跑（不计 FAIL）"; INVALID=1
  fi
}

# ============ Stage B：teb 物理闭环（M2-2） ============
stage_b() {
  # [M2R3] TF_SOURCE 环境变量为阴性对照口子：置 static 可回退旧静态件，
  #   用于证明新门控 G5/G5b 确有分辨力（该跑必须 G5 FAIL）；缺省空=用 launch 内 fresh 缺省。
  roslaunch lite3_bringup mode_sim_teb.launch ${TF_SOURCE:+tf_source:=$TF_SOURCE} > "$LOGDIR/tebB.log" 2>&1 &
  wait_topic /clock 300 || { echo "FAIL B: /clock 未出"; return 1; }
  wait_topic /rslidar_points 480 || { echo "FAIL B: /rslidar_points 未出"; return 1; }
  timeout 60 rostopic echo -n 1 /cmd_vel >/dev/null 2>&1 || true
  # 等 mbf move_base action server（60s）；goal=(start+3,0)
  # M2-R2：goal 前加就绪门控（b858a86 FAIL：teb_state=4 ABORTED @dt=0，"Can not get robot pose" 启动窗竞态）
  python3 - <<'PYEOF' > "$LOGDIR/stageB.txt" 2>&1
import math, time, rospy, actionlib
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, OccupancyGrid
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal   # mbf 原生 action（G8sim get_path 同族）
import tf as tf_py
rospy.init_node('m2_stage_b')
S=dict(gt=None,odom=None)
rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m:[S.update(gt=(p.position.x,p.position.y))
    for p,n in zip(m.pose,m.name) if n=='lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/Odometry', Odometry, lambda m:S.update(odom=m), queue_size=2)
w0=time.time()
while (S['gt'] is None or S['odom'] is None) and time.time()-w0<60: time.sleep(0.1)
sx,sy=S['gt'][0],S['gt'][1]; gx,gy=sx+3.0,sy
print('SMOKE start=(%.3f,%.3f) goal=(%.3f,%.3f)'%(sx,sy,gx,gy))
cli=actionlib.SimpleActionClient('/move_base_flex/move_base', MoveBaseAction)
if not cli.wait_for_server(rospy.Duration(60)): print('SMOKE mbf_server=ABSENT'); raise SystemExit(1)
# —— M2-R2 门控（G1–G4，总预算 120s；任一未过 = FAIL 环境不进闭环，不计入 teb 判据）——
# 注：sim_nav_smoke 的 occupied>0（v>=90）判据在本域不适用——m2_free_map 发布全 FREE 底图
#     （m2_free_map.py L24 恒 [0]，无 v>=90 格），故 G1/G3/G4 为「有数据」门控，不查占据数。
GATE_BUDGET = 120.0
gate_t0 = time.time()
tf_ = tf_py.TransformListener()
def gate_left(): return GATE_BUDGET - (time.time() - gate_t0)
def gate_wait(name, fn):
    w = time.time()
    while time.time() - w < gate_left() and not rospy.is_shutdown():
        try:
            if fn(): print('SMOKE gate_%s=PASS t=%.1fs' % (name, time.time()-gate_t0)); return True
        except Exception: pass
        time.sleep(0.3)
    print('SMOKE gate_%s=FAIL t=%.1fs' % (name, time.time()-gate_t0)); return False
ok_map = gate_wait('map_data',      lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
ok_tf  = gate_wait('tf_robot',      lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))   # 竞态根因直查
# —— [M2R3 2026-10-08 小张 §0/§1] 新鲜度门 G5 / G5b / G5c ——
# 旧 G1–G4 是「可达性门」：map→odom 若挂 /tf_static，静态条目语义为「对所有时刻恒真」，
# canTransform(Time(0)) 永远返回 True ⇒ 结构上不具备新鲜度分辨力（不是门太松，是维度缺失）。
# G5  = map→base_link 连续 3 次采样 Δ_age ≤ 1.0s(sim)
# G5b = 在取龄内否决「零戳」与「/map 首帧戳」两种形态（后者与零戳同量级同后果但可绕纯零戳断言）
# G5c = odom→base_link 与 map→odom 分段各取龄，用于断链定位（静态段单独判）
MAP_FIRST = [None]
def _mapcb(m):
    if MAP_FIRST[0] is None:
        MAP_FIRST[0] = m.header.stamp
rospy.Subscriber('/map', OccupancyGrid, _mapcb, queue_size=1)

# 取龄必须走 tf2 Buffer：老 tf.TransformListener 既无 lookup_transform（蛇形名是 tf2 的），
# 其 lookupTransform 也只收三参且返回裸 [x,y,z]/[x,y,z,w] 列表、不含 header.stamp，无从取时间戳。
# tf2 Buffer.lookup_transform 返回带 header.stamp 的 TransformStamped，且与 mbf 保鲜校验同构。
import tf2_ros
buf2 = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(buf2)

def _age_of(a, b):
    st = buf2.lookup_transform(a, b, rospy.Time(0), rospy.Duration(2.0))
    s = st.header.stamp
    if s == rospy.Time(0):
        return None, 'G5b_hit=zero_stamp'
    if MAP_FIRST[0] is not None and abs((s - MAP_FIRST[0]).to_sec()) < 1e-6:
        return None, 'G5b_hit=first_map_stamp'
    return (rospy.get_rostime() - s).to_sec(), None

def g5_sample(a, b, need=3, gap=0.2, lim=1.0):
    ds = []
    for _ in range(need):
        d, why = _age_of(a, b)
        if why:
            return False, why
        if d > lim:
            return False, 'age=%.3f>%.1f' % (d, lim)
        ds.append('%.3f' % d)
        time.sleep(gap)
    return True, 'ages=[' + ','.join(ds) + ']'

DET = {}
def gate_wait_d(name, fn):
    w = time.time()
    while time.time() - w < gate_left() and not rospy.is_shutdown():
        try:
            ok, det = fn()
            DET[name] = det
            if ok:
                print('SMOKE gate_%s=PASS t=%.1fs %s' % (name, time.time() - gate_t0, det))
                return True
        except Exception as e:
            import traceback as _tb
            DET[name] = 'exc=%s: %s | %s' % (type(e).__name__, e,
                                             _tb.format_exc().strip().splitlines()[-2:-1])
        time.sleep(0.3)
    print('SMOKE gate_%s=FAIL t=%.1fs %s' % (name, time.time() - gate_t0, DET.get(name, '?')))
    return False
def costmap_up(topic):
    try: return rospy.wait_for_message(topic, OccupancyGrid, timeout=2.0).info.width > 0
    except Exception: return False
ok_gcm = gate_wait('gcostmap_data', lambda: costmap_up('/move_base_flex/global_costmap/costmap'))
ok_lcm = gate_wait('lcostmap_data', lambda: costmap_up('/move_base_flex/local_costmap/costmap'))
# —— [M2R4 2026-10-08] G5 取龄定点延时口子（小美 M2R3 复核条件「5 连正中 ≥1 次把取龄推到 sim 时钟 ≥21.5s」）——
#   fresh1/2/3 实测取龄均在 t=1.2–1.6s（病灶区之外）⇒ 正向证据未覆盖病理时刻，故补齐需可推后采样。
#   用法：G5_DELAY_SIM=21.5 bash m2_smoke.sh stageB ；缺省 0 = 不延时，行为与 M2R3 逐字一致（零回归）。
#   注：延时消耗的墙钟从 gate_t0 扣回——门控 120s 预算语义 = 环境就绪耗时上限，不含人为注入的观测延时，
#       否则 G5/G5c2 会因预算被吃掉而假 FAIL。
import os
G5_DELAY_SIM = float(os.environ.get('G5_DELAY_SIM', '0') or 0)
def sim_wait_until(sec, budget=300.0):
    w = time.time()
    while time.time() - w < budget and not rospy.is_shutdown():
        if rospy.get_rostime().to_sec() >= sec: return True
        time.sleep(0.5)
    return False
ok_g5   = gate_wait_d('tf_fresh',        lambda: g5_sample('map', 'base_link'))
ok_g5c2 = gate_wait_d('tf_fresh_mapodom', lambda: g5_sample('map', 'odom'))
sim_after_p1 = rospy.get_rostime().to_sec()
print('SMOKE SIMNOW point=age_sample_1 sim_now=%.2fs gate_t=%.1fs' % (sim_after_p1, time.time()-gate_t0))
ok_g5b = True; ok_g5c2b = True
# [M2R5/R3] 加压档 = 同跑「取龄点2」：延时到 sim 过线后再取一次龄，两点 sim 差即小美新口径的可打印量。
#   记账方向为**加回** gate_t0（原 -= 反了：t 与预算皆按 现在减 gate_t0 计，减 t0 反使 t 变大，
#   延时被计两次——fresh4 t=1.4 → fresh5 t=7.4、差 6.0 = 2×wall_spent 实证）。raw/adjusted 两个 t 都打。
if G5_DELAY_SIM > 0:
    _raw_t = time.time() - gate_t0
    _w = time.time()
    _r = sim_wait_until(G5_DELAY_SIM)
    _spent = time.time() - _w
    gate_t0 += _spent
    print('SMOKE G5_DELAY_SIM target=%.1fs reached=%s sim_now=%.2fs wall_spent=%.1fs t_raw=%.1fs t_adj=%.1fs'
          % (G5_DELAY_SIM, _r, rospy.get_rostime().to_sec(), _spent, _raw_t, time.time() - gate_t0))
    ok_g5b   = gate_wait_d('tf_fresh_late',            lambda: g5_sample('map', 'base_link'))
    ok_g5c2b = gate_wait_d('tf_fresh_mapodom_late',    lambda: g5_sample('map', 'odom'))
    _sim2 = rospy.get_rostime().to_sec()
    print('SMOKE SIMNOW point=age_sample_2 sim_now=%.2fs gate_t=%.1fs sim_span_vs_point1=%.2fs'
          % (_sim2, time.time()-gate_t0, _sim2 - sim_after_p1))
# G5c1 **自 M2R5 起为阻断 gate 项**：小美 M2R4 复核节判 L307 前置③成立（base_link 双父未修正而
#   G5c1 7/7 全命中）⇒ 本档 M2R3 的「降级追认」作废，按 L307 原文回补进 gate 字典。
#   调用形态与阈值一律未放宽：仍单次 g5_sample('odom','base_link')，need=3 / lim=1.0 / 零戳与首帧戳否决照旧。
ok_g5c1 = g5_sample('odom', 'base_link')[0]
print('SMOKE gate_G5c1(odom->base_link)=%s' % ('PASS' if ok_g5c1 else 'FAIL'))
# —— [M2R4] E3：门控时刻对原始 /tf + /tf_static 直读一次（小美指定「一步定阴阳，最省」）——
#   M2R4 首发已定阴阳：原始 /tf 上 odom→base_link 戳是新鲜的（发布侧无罪），病灶在图结构——
#   xacro 的 fixed 关节 torso_to_base 使 base_link 同时有静态父 TORSO 与动态父 odom（双父）。
#   M2R5/U1 反挂该关节后，本探针转任**回归哨兵**：期望签名 = base_link_parents 恰为 ['odom'] 单元素，
#   且 static 行集里不出现任何 ->base_link 的边（应只见 base_link->TORSO 与 base_link->rslidar）。
#   本探针仍只打印、不进 gate 字典（非阻断）；结构面判定归验收。
from tf2_msgs.msg import TFMessage
def e3_raw_tf(win=2.0):
    seen = []
    def cb(m):
        for tr in m.transforms:
            seen.append((tr.header.frame_id, tr.child_frame_id, tr.header.stamp))
    sub = rospy.Subscriber('/tf', TFMessage, cb, queue_size=200)
    w = time.time()
    while time.time() - w < win and not rospy.is_shutdown(): time.sleep(0.1)
    sub.unregister()
    now = rospy.get_rostime()
    agg = {}
    par = {}
    for f, c, s in seen:
        par.setdefault(c, set()).add(f)
        if c != 'base_link': continue
        t, n = agg.get((f, c), (s, 0))
        agg[(f, c)] = (max(t, s), n + 1)
    for (f, c), (s, n) in sorted(agg.items()):
        print('SMOKE E3 tf %s->%s hits=%d last_stamp=%.3f age=%.3f%s'
              % (f, c, n, s.to_sec(), (now - s).to_sec(), ' [ZERO]' if s == rospy.Time(0) else ''))
    print('SMOKE E3 base_link_parents=%s transforms_total=%d'
          % (sorted(par.get("base_link", [])), len(seen)))
    try:
        st = rospy.wait_for_message('/tf_static', TFMessage, timeout=3.0)
        for tr in st.transforms:
            print('SMOKE E3 static %s->%s stamp=%.3f'
                  % (tr.header.frame_id, tr.child_frame_id, tr.header.stamp.to_sec()))
    except Exception as e:
        print('SMOKE E3 static=ERR %s' % type(e).__name__)
e3_raw_tf()
gate = dict(map_data=ok_map, tf_robot=ok_tf, gcostmap_data=ok_gcm, lcostmap_data=ok_lcm,
            tf_fresh_G5=ok_g5, tf_fresh_mapodom_G5c2=ok_g5c2, tf_fresh_G5c1=ok_g5c1,
            tf_fresh_late_G5=ok_g5b, tf_fresh_mapodom_late_G5c2=ok_g5c2b)
if not all(gate.values()):
    print('SMOKE gate=FAIL t=%.1fs %s' % (time.time()-gate_t0, gate)); raise SystemExit(2)
print('SMOKE gate=PASS t=%.1fs' % (time.time()-gate_t0))
g=MoveBaseGoal(); g.target_pose.header.frame_id='map'
g.target_pose.header.stamp=rospy.Time.now()
g.target_pose.pose.position.x=gx; g.target_pose.pose.position.y=gy
g.target_pose.pose.orientation.w=1.0
t0=rospy.get_rostime(); cli.send_goal(g)
print('SMOKE SIMNOW point=goal_send sim_now=%.2fs' % t0.to_sec())
ok=cli.wait_for_result(rospy.Duration(120))
dt=(rospy.get_rostime()-t0).to_sec()
print('SMOKE SIMNOW point=goal_done sim_now=%.2fs dt=%.2f' % (rospy.get_rostime().to_sec(), dt))
m=S['odom']; d=math.hypot(m.pose.pose.position.x-gx, m.pose.pose.position.y-gy)
gd=math.hypot(S['gt'][0]-gx, S['gt'][1]-gy)
print('SMOKE teb_state=%s dt=%.1fs dev_odom=%.3f dev_gt=%.3f'%(cli.get_state(), dt, d, gd))
PYEOF
  cat "$LOGDIR/stageB.txt"
  grep -q "SMOKE gate=FAIL" "$LOGDIR/stageB.txt" && { mark 0 "B: 门控未过（环境未就绪，FAIL 环境；见 stageB.txt gate_*=FAIL 行）"; return 1; }
  grep -q "SMOKE gate=PASS" "$LOGDIR/stageB.txt" && mark 1 "B: 门控通过（≤120s，goal 前就绪）" || mark 0 "B: 门控结果缺失"
  # [M2R3 2026-10-08 小张 §3] G6：goal 之后 pose 类 ERROR 命中数必须为 0
  #   G5 系前置新鲜度门、G6 系后置哨兵——两者双保险，防「门控时刻新鲜、执行中途断流」漏检。
  #   注：grep -c 计数为 0 时既输出 0 又退出码 1，故**不可**接 `|| echo 0`（会拼成 "0\n0" 两行，
  #       使算术比较判假、把 0 命中误标 FAIL）——首版即踩此坑，已改为单值归一取数。
  G6=$(grep -acE "Can not get the robot pose|Could not get the current robot pose|Most recent robot pose is" "$LOGDIR/tebB.log" 2>/dev/null | head -1)
  G6=${G6:-0}
  [ "$G6" -eq 0 ] 2>/dev/null && mark 1 "B/G6: goal 后 pose 类 ERROR=0" || mark 0 "B/G6: pose 类 ERROR=${G6} 命中（TF 时基新鲜度回归）"
  get() { grep -o "$1=[-0-9.e]*" "$LOGDIR/stageB.txt" | head -1 | cut -d= -f2; }
  D=$(get dev_odom); T=$(get dt)
  awk -v a="$D" 'BEGIN{exit !(a<=0.35)}' && mark 1 "B: 到达偏差 $D ≤0.35m" || mark 0 "B: 到达偏差 $D >0.35m"
  awk -v a="$T" 'BEGIN{exit !(a<=120.0)}' && mark 1 "B: 用时 $T ≤120s" || mark 0 "B: 用时 $T >120s"
}

# ============ Stage C：hdl 重定位（M2-3） ============
stage_c() {
  # C-1 建图（pcd_save:=true）：复用 Stage A 驱动序列走一遍（out/turn/back），优雅 SIGINT 落盘
  roslaunch lite3_gazebo_sim m2_gazebo.launch > "$LOGDIR/gazeboC.log" 2>&1 &
  wait_topic /rslidar_points 480 || { echo "FAIL C: /rslidar_points 未出"; return 1; }
  wait_topic /livox/imu 300 || { echo "FAIL C: /livox/imu 未出"; return 1; }
  roslaunch lite3_gazebo_sim m2_mapping.launch pcd_save:=true > "$LOGDIR/mappingC.log" 2>&1 &
  wait_topic /cloud_registered 360 || { echo "FAIL C: /cloud_registered 未出"; return 1; }
  python3 - <<'PYEOF' >/dev/null 2>&1 &
import math, time, rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
rospy.init_node('m2_stage_c_drive')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
S=dict(o=None,iq=None)
rospy.Subscriber('/Odometry',Odometry,lambda m:S.update(o=m),queue_size=2)
rospy.Subscriber('/livox/imu',Imu,lambda m:S.update(iq=m.orientation),queue_size=1)
def qyaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
w0=time.time()
while (S['o'] is None or S['iq'] is None) and time.time()-w0<60: time.sleep(0.1)
sx=S['o'].pose.pose.position.x; y0=qyaw(S['iq'])
def ph(vx,wz,done,lim=60.0):
    w0=time.time()
    while not rospy.is_shutdown():
        if done(): pub.publish(Twist()); return True
        if time.time()-w0>lim: pub.publish(Twist()); return False
        t=Twist(); t.linear.x=vx; t.angular.z=wz; pub.publish(t); time.sleep(0.05)
ph(0.5,0.0,lambda:S['o'].pose.pose.position.x-sx>=4.85)
ph(0.0,0.6,lambda:math.degrees(abs(qyaw(S['iq'])-y0))>=178.0)
ph(0.5,0.0,lambda:math.hypot(S['o'].pose.pose.position.x-sx,S['o'].pose.pose.position.y)<=0.4)
rospy.sleep(2.0)
PYEOF
  sleep 150   # 驱动循环跑完（26 sim-s × 实测 RTF≈0.30 → ~87 wall-s，150s 裕度）D-M2-P5
  rosnode kill /faster_lio_online 2>/dev/null
  for i in $(seq 1 30); do   # 等优雅落盘：样张 sha256 变化 = 新图已写
    [ -f "$MAP" ] && sha256sum "$MAP" | awk '{print $1}' > "$LOGDIR/pcd.sha256.mapped"
    cmp -s "$LOGDIR/pcd.sha256.before" "$LOGDIR/pcd.sha256.mapped" 2>/dev/null || break
    sleep 2
  done
  sleep 3
  if [ -f "$MAP" ] && ! cmp -s "$LOGDIR/lite3.pcd.bak" "$MAP"; then
    cp "$MAP" "$VENUE"; sha256sum "$VENUE" | awk '{print $1}' > "$LOGDIR/sim_venue.sha256"
    cp "$LOGDIR/lite3.pcd.bak" "$MAP"; echo "[m2] sim_venue.pcd 已转存，样张已恢复"
  else echo "FAIL C: 建图落盘未发生（样张未变）"; return 1; fi
  [ -f "$VENUE" ] || { echo "FAIL C: sim_venue.pcd 缺失"; return 1; }
  rosnode kill -a 2>/dev/null; sleep 3; pkill -f gzserver 2>/dev/null; sleep 2
  # C-2 重定位：hdl 载入 sim_venue → /initialpose 注入真值邻域（±0.15m/5°）→ 三判据
  T_HDL0=$(date +%s)   # M2-R2C：稳定性判据计时起点（hdl 栈存活 ≥120s 观测窗）
  roslaunch lite3_bringup mode_sim_hdl.launch > "$LOGDIR/hdlC.log" 2>&1 &
  wait_topic /rslidar_points 480 || { echo "FAIL C: /rslidar_points 未出"; return 1; }
  timeout 60 rostopic echo -n 1 /hdl_odom >/dev/null 2>&1 || true
  python3 - <<'PYEOF' > "$LOGDIR/stageC.txt" 2>&1
import math, time, rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
rospy.init_node('m2_stage_c_reloc')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
S=dict(gt=None,gyaw=None,ip=None,iq=None)
rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m:[S.update(gt=(p.position.x,p.position.y),
    gyaw=math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
    1-2*(p.orientation.y**2+p.orientation.z**2)))
    for p,n in zip(m.pose,m.name) if n=='lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/hdl_odom', Odometry, lambda m:S.update(ip=m), queue_size=2)
rospy.Subscriber('/livox/imu', Imu, lambda m:S.update(iq=m.orientation), queue_size=1)
def qyaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
w0=time.time()
while (S['gt'] is None or S['iq'] is None) and time.time()-w0<30: time.sleep(0.1)
# —— M2-R2 门控（hdl 同款预防：图载入 + odom 管线/TF 上线后才注 initialpose，总预算 120s）——
# globalmap_server 载完 pcd 后 latched 发 /globalmap（globalmap_server_nodelet.cpp:36,76）；/hdl_odom 出帧=估计管线上线
from sensor_msgs.msg import PointCloud2 as PC2
import tf as tf_py2
g0t=time.time()
try:
    gm=rospy.wait_for_message('/globalmap', PC2, timeout=60.0); ok_map=gm.width>0
except rospy.ROSException:
    ok_map=False
print('SMOKE gate_mapload=%s t=%.1fs' % (ok_map, time.time()-g0t))
gw0=time.time()
while S.get('ip') is None and time.time()-gw0<60 and time.time()-g0t<120: time.sleep(0.2)
tf2_=tf_py2.TransformListener()
gtw=time.time()
while not tf2_.canTransform('map','base_link',rospy.Time(0)) and time.time()-gtw<30 and time.time()-g0t<120: time.sleep(0.2)
ok_tf=tf2_.canTransform('map','base_link',rospy.Time(0))
gateC=dict(globalmap=ok_map, hdl_odom=S.get('ip') is not None, tf_robot=ok_tf)
if not all(gateC.values()):
    print('SMOKE gateC=FAIL t=%.1fs %s' % (time.time()-g0t, gateC)); raise SystemExit(2)
print('SMOKE gateC=PASS t=%.1fs' % (time.time()-g0t))
ip=PoseStamped(); ip.header.frame_id='map'; ip.header.stamp=rospy.Time.now()
ip.pose.position.x=S['gt'][0]+0.15; ip.pose.position.y=S['gt'][1]-0.10
ip.pose.orientation.w=math.cos((-5.0/2)/2); ip.pose.orientation.z=math.sin((-5.0/2)/2)
pubi=rospy.Publisher('/initialpose', PoseStamped, queue_size=1, latch=True)
for _ in range(5): pubi.publish(ip); time.sleep(0.2)
print('SMOKE initialpose=(%.3f,%.3f,-5deg) gt=(%.3f,%.3f)'%(ip.pose.position.x,ip.pose.position.y,S['gt'][0],S['gt'][1]))
t0=time.time(); conv=False; last=None; jump=0.0
while time.time()-t0<40 and not rospy.is_shutdown():
    m=S['ip']
    if m is not None:
        d=math.hypot(m.pose.pose.position.x-S['gt'][0], m.pose.pose.position.y-S['gt'][1])
        dy=abs(math.degrees(qyaw(m.pose.pose.orientation)-S['gyaw']))
        if last is not None: jump=max(jump, abs(d-last)); 
        last=d
        if time.time()-t0<=10.0 and d<=0.30 and dy<=10.0: conv=True; break
    time.sleep(0.05)
t_conv=time.time()-t0
worst=0.0; t0b=time.time()
while time.time()-t0b<30 and not rospy.is_shutdown():
    m=S['ip']
    if m is not None:
        d=math.hypot(m.pose.pose.position.x-S['gt'][0], m.pose.pose.position.y-S['gt'][1])
        worst=max(worst, d)
    time.sleep(0.05)
print('SMOKE reloc conv=%s t=%.1fs dev=%.3f worst30s=%.3f'%(conv,t_conv,last if last else -1,worst))
PYEOF
  cat "$LOGDIR/stageC.txt"
  grep -q "SMOKE gateC=FAIL" "$LOGDIR/stageC.txt" && { mark 0 "C: 门控未过（hdl 图/TF 未就绪，FAIL 环境）"; return 1; }
  grep -q "SMOKE gateC=PASS" "$LOGDIR/stageC.txt" && mark 1 "C: 门控通过（图载入+TF 就绪后注 initialpose）" || mark 0 "C: 门控结果缺失"
  get() { grep -o "$1=[A-Za-z0-9.-]*" "$LOGDIR/stageC.txt" | head -1 | cut -d= -f2; }
  CV=$(get conv); DV=$(get dev); W=$(get worst30s)
  [ "$CV" = "True" ] && mark 1 "C: 10s 内收敛 ≤0.3m/10°" || mark 0 "C: 10s 未收敛（dev=$DV）"
  awk -v a="$W" 'BEGIN{exit !(a<=0.5)}' && mark 1 "C: 30s 最差偏差 $W ≤0.5m（无跳变）" || mark 0 "C: 30s 窗最差 $W >0.5m"
  # —— M2-R2C 新增稳定性判据（R2C manifest 施工件4/回归判据）：hdl 栈（manager+双 nodelet）存活 ≥120s，
  #    无 exit-11/无 Bond broken/无 nodelet 进程死亡。必须在 cleanup 前判定——cleanup 的 rosnode kill -a
  #    会产生正常的 process has died 行（上轮 124125/125320 hdlC.log L78/80/82 实证），不可混入。
  #    勘误注记：Bond broken 实际落点=本 launch 的 ROS log stdout 文件（上轮实证 hdlC.log 仅 process has died
  #    行），故 manifest「grep hdlC.log」扩为 hdlC.log + 本 launch 日志目录 *-stdout.log 双扫。
  ELAPSED=$(( $(date +%s) - T_HDL0 ))
  if [ "$ELAPSED" -lt 120 ]; then sleep $(( 120 - ELAPSED )); fi
  ROSL_LOGDIR="${ROS_LOG_DIR:-$HOME/.ros}/logs"
  HDL_UUID=$(ls -t "$ROSL_LOGDIR" 2>/dev/null | head -1)
  HDL_STDOUT_GLOB="$ROSL_LOGDIR/$HDL_UUID"/*-stdout.log
  if grep -Eq "process has died|exit code -11|Bond broken" "$LOGDIR/hdlC.log" 2>/dev/null \
     || grep -q "Bond broken" $HDL_STDOUT_GLOB 2>/dev/null; then
    mark 0 "C: 稳定性 FAIL（观测窗内命中进程死亡/exit-11/Bond broken）"
    grep -En "process has died|exit code -11|Bond broken" "$LOGDIR/hdlC.log" 2>/dev/null | head -5
    grep -l "Bond broken" $HDL_STDOUT_GLOB 2>/dev/null | head -3
  else
    mark 1 "C: 稳定性 PASS（hdl 栈存活 ≥120s：无 exit-11/Bond broken/进程死亡）"
  fi
  rosnode list 2>/dev/null | grep -qx "velodyne_nodelet_manager" \
    && mark 1 "C: 稳定性旁证（manager 观测窗末在册存活）" || mark 0 "C: 稳定性旁证（manager 观测窗末不在册）"
  # 静置微动（真实底盘静止，仅扰动指令零速）判据旁证：发零速
  true
}

case "$STAGE" in
  stageA) stage_a ;;
  stageB) stage_b ;;
  stageC) stage_c ;;
  all) stage_a; cleanup; sleep 3; stage_b; cleanup; sleep 3; stage_c ;;
  *) echo "用法: $0 [stageA|stageB|stageC|all]"; exit 2 ;;
esac

# ============ 退净核验（GAP-5，O2 勘误：比对前移 trap 已兜底） ============
if [ -f "$MAP" ] && [ -f "$LOGDIR/pcd.sha256.before" ]; then
  sha256sum "$MAP" | awk '{print $1}' > "$LOGDIR/pcd.sha256.after"
  cmp -s "$LOGDIR/pcd.sha256.before" "$LOGDIR/pcd.sha256.after" \
    && mark 1 "样张 lite3.pcd sha256 未变" || mark 0 "样张 lite3.pcd 被改动（备份 $LOGDIR/lite3.pcd.bak）"
fi
echo "=================================================="
if [ "$INVALID" = "1" ]; then
  echo "M2 SMOKE: INVALID（运动窗 RTF<0.25，重跑不计 FAIL）"
  echo "=================================================="
  exit 3
fi
[ "$PASS" = "1" ] && echo "M2 SMOKE: PASS" || echo "M2 SMOKE: FAIL"
echo "=================================================="
exit $([ "$PASS" = "1" ] && echo 0 || echo 1)
