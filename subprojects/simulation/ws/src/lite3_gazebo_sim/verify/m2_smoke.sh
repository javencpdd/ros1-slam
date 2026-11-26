#!/usr/bin/env bash
# m2_smoke.sh —— M2 无人值守冒烟（小张，2026-10-06；m1_smoke.sh 同构扩展）
# Stage A（M2-1/M2-4）：闭环回归四判据 —— IMU 姿态反馈转向 + odom 位置反馈回程；drift ≤0.5m、turn/back 双 done、
#                       /rslidar_points ≥5Hz、/livox/imu ≥100Hz、飞点 <1%
# Stage B（M2-2）     ：物理闭环（2026-11-21 迁至 BT+v8 链）—— goal=(start+3m,0)，sim 时 ≤120s 到达偏差 ≤0.35m（gt 参考）
# Stage C（M2-3）     ：hdl 重定位 —— 建图存 sim_venue.pcd（样张备份/恢复 sha256 红线）→ /initialpose 注入
#                       → ≤10s 收敛 ≤0.3m/10°，30s 观察窗无 >0.5m 单步跳变
# 隔离域：ROS master 11313 + GAZEBO_MASTER_URI=11345（本会话 P10 仲裁值，见 L21 实际 export）；生产 11312 全程零监听；foxy 栈零接触。
# 用法：bash /opt/ros1-slam/sim_ws/src/lite3_gazebo_sim/verify/m2_smoke.sh [stageA|stageB|stageC|all]（缺省 all；或 docker/sim.sh smoke）
#   批次驱动用同目录 m2_batch_runner.sh；整批复取用同目录 retake_driver.sh（现役入口全在包内）。/tmp/m2_payload 侧
#   副本已就地改名 .STALE_do_not_run_20261008，旧用法行指的是停用路径（她 #14／R26 四；上一轮我漏落，根因见 02 坑31）。
#   LOGDIR 落址口径（R34）：LOGDIR 仍在 /tmp（$PAYLOAD/logs-时间戳），**跑完即归档属驱动件的责任**——retake_driver.sh
#   每段结束就把本次新 LOGDIR 的文本原件拉进 证据/log/raw/retake_*/ 并打 sha 清单；不经驱动单跑本件时必须自己归档，
#   否则证据随 /tmp 一清或机器一重启即永久不可回溯（O-4 三次同型＋ 10-08 那次硬重启实测丢过一整段 SCAN 证据）。
set -u
PAYLOAD=/tmp/m2_payload
# 2026-11-21：仿真包物理拆仓后旧绝对路径 /home/ros1-slam/ws/src/95_simulation 悬空（跑不了而非没跑）；
# 改为按脚本自身位置自推导包根，布局再变不断链（容器内解析到 /opt/ros1-slam/sim_ws/src/lite3_gazebo_sim）。
SIM_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP=/home/ros1-slam/data/maps/lite3.pcd
VENUE=/home/ros1-slam/data/maps/sim_venue.pcd
STAGE=${1:-all}
LOGDIR="$PAYLOAD/logs-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"; echo "[m2] 日志目录: $LOGDIR"
# R25（小美 M2R6）：harness 自打本体指纹 —— 跑完即可断言「这跑跑的是哪一份件」，
#   不再靠 mtime 或打印形态反推（旧缺陷：/tmp 侧过期副本曾被当现役件跑）。
echo "[m2] HARNESS sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}') path=${BASH_SOURCE[0]}"

source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
# 2026-11-21：拆仓后 livox 插件在 sim_ws/devel/lib（与 docker/sim.sh 同序：sim_ws → 主 ws → gazebo_ros）
export GAZEBO_PLUGIN_PATH=/opt/ros1-slam/sim_ws/devel/lib:/home/ros1-slam/ws/devel/lib:/opt/ros/noetic/lib

PASS=1
INVALID=0  # A0 前置：运动窗 RTF<0.25（M2R2 主理人裁决 0.3→0.25：D-01 后 200Hz IMU 正确配置合理 RTF 平台 0.288，门意抓塌缩非惩罚正确配置）→ 本轮作废重跑，不计 FAIL
NPMARK=0; NFMARK=0   # R27 四：mark 自计数，供判定行与原件计数互核（她 #2：表上 6/2 与原件 3/1 不可指认）
mark() { if [ "$1" = "1" ]; then NPMARK=$((NPMARK+1)); echo "[PASS] $2"; else NFMARK=$((NFMARK+1)); PASS=0; echo "[FAIL] $2"; fi; }
# [R35 落地] 判据库与被测件的跑内指纹（R28 二 的实际兑现）。只打 harness 自身不够：批1 恰在改 xacro，
#   单行自证无法断言「这跑跑的是哪一份被测件」；取不到件必须打 ABSENT 并记一条 mark 0，不许静默少打。
# ★插件指纹必须打"实载的那一份"★（2026-11-27，/proc 为证）：m2_gazebo.launch 的
# GAZEBO_PLUGIN_PATH 把 sim_ws/devel/lib 排第一，实测 gzserver 的 /proc/<pid>/maps 命中的是
# /opt/ros1-slam/sim_ws/devel/lib/liblite3_planar_drive.so（sha fcc3852b…），
# 而原先这里打的是 /home/ros1-slam/ws/devel/lib 那份（sha 19211b76…）——两者同尺寸不同 sha。
# 这里按 GAZEBO_PLUGIN_PATH 的先后取第一个存在件，并在行里标出是哪一个。
_plug=""
for _d in /opt/ros1-slam/sim_ws/devel/lib /home/ros1-slam/ws/devel/lib; do
  [ -f "$_d/liblite3_planar_drive.so" ] && { _plug="$_d/liblite3_planar_drive.so"; break; }
done
_plug=${_plug:-/home/ros1-slam/ws/devel/lib/liblite3_planar_drive.so}
for _f in "$SIM_PKG/verify/m2_exitclean_lib.sh" "$SIM_PKG/urdf/lite3_planar.urdf.xacro" "$SIM_PKG/src/lite3_planar_drive.cpp" "$_plug"; do
  if [ -f "$_f" ]; then echo "[m2] FP $(basename "$_f") sha256=$(sha256sum "$_f" | awk '{print $1}')";
  else echo "[m2] FP $(basename "$_f") ABSENT path=$_f"; mark 0 "自证不可指认：指纹件缺位 $_f（R35：缺件即判不洁）"; fi
done
#   注：本块必须排在 mark() 之后——上一版我把它插在跑首行自证之后、mark 定义之前，
#   ABSENT 分支一触发就是 command not found（顺序错，本轮自查出并搬到位）。
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
PAT='roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|topic_tools|sim_adapter|m2_free_map|m2_cmd_relay|m2_tf_fresh|hdl_nodelet|run_mapping_online'
# 自有标识（R22 二）：差集命中者命令行含其中之一才判死；**禁按 PID 硬编码白名单**（R22 四），
#   既存命中一律靠「与基线快照取差集」自然排除，不写死任何 PID。
OWN='m2_tf_fresh|m2_free_map|m2_cmd_relay|gzserver|rosmaster --core -p 11313'
PORTS="11311 11312 11313 11345"
# [R26 落地 2026-10-08] 退净判据四函数（port_cnt／port_ids／probe_sens_check／exitbase_snapshot／exitclean_check）
#   搬到同目录 m2_exitclean_lib.sh：她 R26 的验收判据要求「改件后随批做一次已知态标定」，而标定必须跑在
#   活体同一份判据代码上（另抄一份去测＝测的不是在跑的那份，05 B2），故判据住库件、活体与零节点标定件同 source。
#   三条改式出处：差集非空即判死（R26 一，OWN 降为打印分栏）／端口面比身份三元（R26 二）／按段令牌各检一次（R26 三）。
SEG=pre          # 段名令牌：stage_a／stage_b／stage_c 各自置 A／B／C（R26 三）
CHECKED_SEG=""   # 已检段；旧 CHECK_DONE 整跑一锁，使 stage=all 只检第一段（她 #13）
source "$SIM_PKG/verify/m2_exitclean_lib.sh"

kill_all() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f hdl_nodelet 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  # [C8 返工修正 2026-10-08] 入口改指 TOP 后命令为 roslaunch lite3_bringup mode_sim_*.launch，
  #   不含 "m2_" ⇒ 原模式漏网（stageB 活体回归实测：roslaunch+rosmaster 存活、11313 未释放）。
  pkill -f "roslaunch.*m2_" 2>/dev/null; pkill -f "roslaunch.*mode_sim_" 2>/dev/null
  pkill -f "rosmaster --core -p 11313" 2>/dev/null; sleep 4
  # 样张恢复兜底（M2-3 建图阶段万一中断）
  if [ -f "$LOGDIR/lite3.pcd.bak" ] && [ -f "$MAP" ] && ! cmp -s "$LOGDIR/lite3.pcd.bak" "$MAP"; then
    cp "$LOGDIR/lite3.pcd.bak" "$MAP"; echo "[m2] 样张 pcd 已从备份恢复"; fi
  true; }

cleanup() { kill_all; exitclean_check; true; }
trap cleanup EXIT

probe_sens_check  # R27 三：端口查询式探针的阳性（非零）自检，随批常驻
exitbase_snapshot   # R21 二：起跑前基线（在任何 roslaunch/gzserver 起来之前）
if [ -f "$MAP" ]; then cp "$MAP" "$LOGDIR/lite3.pcd.bak"
  sha256sum "$MAP" | awk '{print $1}' > "$LOGDIR/pcd.sha256.before"; echo "[m2] 样张 pcd 已备份（O2 勘误：比对前移至此）"; fi

# [C8 返工 2026-10-08 主理人] 六个 roslaunch 入口由 $PAYLOAD(/tmp 载荷副本) 改指 ws/src 包内路径：
#   (1) 防复验跑到过期副本（m2_teb.launch 已因 [6] 改名与 [5] include 上收而变，/tmp 侧仍是旧版）
#   (2) teb/hdl 改指 TOP 编排件 lite3_bringup/mode_sim_*.launch（[5] 返工；判据/阈值/stage 数/进程拓扑零改动）

# ★激光传感器激活 helper★（2026-11-27 实测）：mid360 插件**有订阅者才发布**点云，
# 冷启动时 `wait_topic /rslidar_points` 会等到超时（插件日志却显示已加载）。
# 顺序：先等 master 就绪（master 由 launch 自己拉起）→ 再挂**常驻**订阅者（-n1 拿到一条就退出，
# 传感器随即失活）。只做激活，不改判据、不重打时间戳。
prime_lidar() {
  local tag=${1:-stage}
  # ★必须能扛住 master 重启★：`roslaunch` 会自己拉起/重启 master，一次性 `rostopic hz` 经常抢在
  # 重启窗口里，得到 "Unable to communicate with master!" 后白白退掉（实测连续两轮）。
  # 这里用一个**会重连**的订阅者进程，连不上就每秒重试，直到整段跑完被清场。
  M2_PRIME_TAG="$tag" nohup python3 - > "${LOGDIR:-/tmp}/livox_primed_${tag}.log" 2>&1 <<'PY' &
import os, sys, time
import rospy
from sensor_msgs.msg import PointCloud2
tag = os.environ.get('M2_PRIME_TAG', 'x')
while True:
    try:
        rospy.init_node('livox_prime_%s' % tag, anonymous=True, disable_signals=True)
        rospy.Subscriber('/livox/lidar', PointCloud2, lambda m: None, queue_size=1)
        print('[prime] subscribed as livox_prime_%s' % tag)
        sys.stdout.flush()
        rospy.spin()
    except Exception as e:
        print('[prime] retry: %s' % e); sys.stdout.flush()
    time.sleep(1.0)
PY
}

# ★不要重排 ROS_PACKAGE_PATH★（2026-11-27 实跑回退）：把 sim_ws/src 前置后，
# stageC 在第一个 `/rslidar_points` 等待处就失败——说明容器实际部署用的是
# `ws/.profiles/x86_sim/src` 那棵被构建过的树，python 侧 `roslaunch lite3_gazebo_sim` 依赖它的世界/插件布局。
# 这属于"部署树 vs 源码树"的架构问题，不能用具名脚本顺手改环境来解决；
# 重复包引起的歧义仍按既有做法用**显式 launch 路径**绕开。

# ============ Stage A：闭环回归（M2-1/M2-4） ============
stage_a() {
  SEG=A   # R26 三：本段退净按段各检一次
  roslaunch lite3_gazebo_sim m2_gazebo.launch > "$LOGDIR/gazeboA.log" 2>&1 &
  # 注：m2_gazebo.launch 亦 install 至包内 launch/（$(find lite3_gazebo_sim)/launch/m2_gazebo.launch），二选一等价
  wait_topic /clock 300 || { echo "FAIL A: /clock 未出"; return 1; }
  wait_topic /rslidar_points 480 || { echo "FAIL A: /rslidar_points 未出"; return 1; }
  wait_topic /livox/imu 300 || { echo "FAIL A: /livox/imu 未出"; return 1; }
  roslaunch lite3_gazebo_sim m2_mapping.launch > "$LOGDIR/mappingA.log" 2>&1 &
  wait_topic /cloud_registered 360 || { echo "FAIL A: /cloud_registered 未出"; return 1; }
  # [批1/F-D 行 C] 位姿源身份断言（小张 F-D §10.5；post_a 档：/Odometry 恰 1 且只剩 camera_init 族，
  #   /sim/odom_gt 恰 1 且为 odom 族）。**只打印不判据**——升不升为阻断项属验收判据增删（05 A3），权在小美。
  bash "$SIM_PKG/verify/v_pose_source.sh" post_a 8 2>&1 | sed 's/^/[posesrc] /' || echo "[posesrc] 断言件执行异常（不判 FAIL，见坑27 同源纪律）"

  # ★2026-11-25 新增：可选挂载钩子★
  # 目的：让"导航图 + 探针"在**冒烟自己的上下文**里跑，而不是另起一套手搓底座。
  # 实测教训：同一份 m2_gazebo.launch、同一份 URDF、同一套环境变量，另起进程跑时 /livox/imu
  # 不出现（`rostopic list` 里连话题都没有），而在本脚本里 IMU 稳定 200Hz；差异只在执行上下文。
  # 用法：M2_SMOKE_HOOK=/path/to/hook.sh bash docker/sim.sh smoke stageA
  if [ -n "${M2_SMOKE_HOOK:-}" ]; then
    echo "[hook] 执行挂载脚本：$M2_SMOKE_HOOK"
    bash "$M2_SMOKE_HOOK" || echo "[hook] 返回非零（不影响本段判定，结果见 hook 自己的证据）"
  fi

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
rospy.Subscriber('/sim/odom_gt', Odometry, lambda m: S.update(odom=m), queue_size=2)  # 批1/F-D：插件世界位里程计改名；/Odometry 归 faster_lio
rospy.Subscriber('/rslidar_points', PointCloud2, on_pts, queue_size=1)
rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=1)
def _on_gt_states(m):
    for p, n in zip(m.pose, m.name):
        if n == 'lite3_planar':
            S.update(gt=(p.position.x, p.position.y))
            # 记 GT 朝向历史：只用于和 LIO 的**相对** yaw 变化比较（不参与生产定位）
            S.setdefault('gt_hist', []).append(
                (0.0, p.position.x, p.position.y, qyaw(p.orientation)))
rospy.Subscriber('/gazebo/model_states', ModelStates, _on_gt_states, queue_size=1)
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
assert S['odom'] is not None, 'no /sim/odom_gt（插件里程计断供；/Odometry 已让给 faster_lio，勿混用）'
p0=S['odom'].pose.pose.position; sx,sy=p0.x,p0.y; i_yaw0=qyaw(S['iq']); S['g0']=S['gt']
print('SMOKE start=(%.3f,%.3f) imu_yaw0=%.3f' % (sx,sy,i_yaw0))
def phase(name, vx, wz, done, wall_limit=150.0, max_travel=25.0):
    """跑到 done 或墙钟超时；另外**累计行程超限即停**——判据万一再次失效时不许跑飞，
    宁可判 TIMEOUT 也不能把 27 m 的行程当成一次正常闭环。"""
    w0=time.time(); p_start=S['odom'].pose.pose.position
    travel=0.0; last=(p_start.x, p_start.y)
    while not rospy.is_shutdown():
        if done(): drive(0,0); print('SMOKE phase_%s_done'%name); return True
        p=S['odom'].pose.pose.position
        travel += math.hypot(p.x-last[0], p.y-last[1]); last=(p.x, p.y)
        if travel > max_travel:
            drive(0,0); print('SMOKE phase_%s_RUNAWAY(%.2fm)'%(name,travel)); return False
        if time.time()-w0>wall_limit: drive(0,0); print('SMOKE phase_%s_TIMEOUT'%name); return False
        drive(vx,wz); time.sleep(0.05)
    return False
# ★ M2-1 修法 A 语义：转向判据=IMU 物理姿态反馈（不再信 odom yaw 开环值）；回程=位置反馈
# ★真实 LIO 读数（2026-11-27 补齐）★：原来 A③ 的 `drift` 取自 `/sim/odom_gt`、`gt_drift` 取自
# `/gazebo/model_states`，**两个都是 Gazebo 真值**，等于拿真值回环比真值，根本没测 Faster-LIO。
# 这里按消息戳记一份 `/Odometry`（LIO）轨迹，用**相对位移**比较（LIO 的 camera_init 与 GT 的 odom
# 原点不同，只有相对量可比），并给 yaw 差的跨界折返值。
S['lio'] = None
S['lio_hist'] = []
def _on_lio(m):
    S['lio'] = m
    if S['mot']:
        S['lio_hist'].append((m.header.stamp.to_sec(),
                              m.pose.pose.position.x, m.pose.pose.position.y,
                              qyaw(m.pose.pose.orientation)))
rospy.Subscriber('/Odometry', Odometry, _on_lio, queue_size=5)
S['lio0'] = None
S['mot']=1; S['m0_wall']=time.time()  # F4：运动窗起点（墙钟，供 A0 RTF 前置判据）
ok_out = phase('fwd_out', 0.5, 0.0, lambda: S['odom'].pose.pose.position.x - sx >= 4.85)
ok_turn = phase('turn180', 0.0, 0.6, lambda: math.degrees(abs(qyaw(S['iq'])-i_yaw0)) >= 178.0)
# ★回程终止判据必须能"越过起点即停"★（2026-11-27 实测）：原来只判"离起点 ≤0.5 m"，
# 而回程是**定速开环**直行（vx=0.5），只要某一帧跨过 0.5 m 圆就再也不会满足条件——
# 实测一路开到 27.17 m 才被 150 s 墙钟掐断，判据与里程同时爆表（drift=gt_drift=27.170）。
# 这不是 RTF 的问题：27.17 m ÷ 0.5 m/s = 54 s 仿真时间 < 窗内可用 63 s（150 s×RTF 0.422）。
# 修法：终点窗保留 0.5 m，另加"沿出程轴已回到起点以内"与"累计行程超限"两条终止条件。
def _back_done():
    p = S['odom'].pose.pose.position
    return (math.hypot(p.x - sx, p.y - sy) <= 0.5) or ((p.x - sx) <= 0.35)
ok_back = phase('fwd_back', 0.5, 0.0, _back_done)
S['mot']=0
mw_wall = max(time.time()-S['m0_wall'], 0.001)  # F4：运动窗墙钟宽
t0=rospy.get_rostime(); rospy.sleep(5.0); t1=rospy.get_rostime()
m=S['odom']; drift=math.hypot(m.pose.pose.position.x-sx, m.pose.pose.position.y-sy)
g=S['gt']; gdrift=math.hypot(g[0]-S['g0'][0], g[1]-S['g0'][1])
# —— 真实 LIO 相对误差（同一条消息流，不使用 GT 修正）——
def _wrap(a): return (a + math.pi) % (2*math.pi) - math.pi
lio_drift = None; lio_gt_delta = None; lio_yaw_err = None
if S.get('lio_hist'):
    t0_, x0_, y0_, yaw0_ = S['lio_hist'][0]
    t1_, x1_, y1_, yaw1_ = S['lio_hist'][-1]
    lio_drift = math.hypot(x1_-x0_, y1_-y0_)
    lio_gt_delta = lio_drift - gdrift
    # GT 的起终 yaw：用首末 /sim/odom_gt 的朝向差（与 LIO 的起终朝向差比较，都是相对量）
    if S.get('gt_hist') and len(S['gt_hist']) >= 2:
        lio_yaw_err = _wrap((yaw1_-yaw0_) - _wrap(S['gt_hist'][-1][2] - S['gt_hist'][0][2]))
# ★键名不得与主行撞车★：外壳用 `grep -o "drift=[...]" | head -1` 取值，
# 原来这行写成 "SMOKE LIO drift=" 会被当成 A③ 的漂移（实测把 0.564 顶掉了 0.487）。
# 这里改用 lio_loop / lio_gt_delta 两个独立键，A③ 仍只看真值回环那一行。
# yaw 差暂不给数：GT 朝向采样与本窗口不同步，实测给出 172.93° 明显是采样错位，
# 在核实配对方式前不写进读数（宁可缺一项，也不报一个假指标）。
print('SMOKE_LIO lio_loop=%.3f gt_loop=%.3f lio_gt_delta=%.3f samples=%d yaw_check=NA'
      % (lio_drift if lio_drift is not None else -1, gdrift,
         lio_gt_delta if lio_gt_delta is not None else -1,
         len(S.get('lio_hist') or [])))
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

# ============ Stage B：物理闭环（M2-2；2026-11-21 迁至 BT+v8 自研链） ============
stage_b() {
  # ★2026-11-21 迁移★ 真人拍板「取消 MBF，全走 BT+自研」（设计＝note/架构/MBF吸收与导航链重构.md）：
  #   执行链 MBF+TEB → BT+v8（mode_sim_bt.launch）。判据主体逐字不变：
  #     · goal＝起点+3 m；≤120 s（sim 时钟）到达；到达偏差 ≤0.35 m（dev_odom 口径，dev_gt 陪印）
  #     · goal 后 pose 类错误＝0（G6'：主题由 TEB 日志换为 v8 的 E_LOCAL_POSE_*/E_LOCAL_LOCALIZATION）
  #   就绪门从 MBF 面（costmap/TF 时基）换为 v8 输入面（support_map/BT lease/pose/obstacles/localization）；
  #   costmap 数据门随 MBF 退场；G5c1（odom→base_link 双父结构哨兵）与 E3 探针不变。
  #   判据面变更按 05 A3 挂 04 台账待追认；07 代际表升 H15；revert 本提交即回退旧 Stage B。
  SEG=B   # R26 三：本段退净按段各检一次
  # [M2R3] TF_SOURCE 环境变量为阴性对照口子：置 static 可回退旧静态件；
  #   缺省空＝用 launch 内 fresh 缺省（mode_sim_bt 透传 m2_teb 的 tf_source）。
  # ★用显式路径起 launch★（2026-11-27 实测）：`ws/.profiles/x86_sim/src` 里有一份**内容完全相同**
  # 的 `lite3_bringup`，包名解析会撞出
  #   RLException: multiple files named [mode_sim_bt.launch] in package [lite3_bringup]
  # 于是 /clock 永远等不到、整段判 FAIL。这里优先用 ws/src 下的规范件；找不到再退回包名。
  local bt_launch
  bt_launch=$(ls -1 "${M2_WS:-/opt/ros1-slam/ws}/src"/*/lite3_bringup/launch/mode_sim_bt.launch 2>/dev/null | head -1)
  roslaunch "${bt_launch:-lite3_bringup}" ${bt_launch:+} \
            ${TF_SOURCE:+tf_source:=$TF_SOURCE} > "$LOGDIR/btB.log" 2>&1 &
  wait_topic /clock 300 || { echo "FAIL B: /clock 未出"; return 1; }
  wait_topic /rslidar_points 480 || { echo "FAIL B: /rslidar_points 未出"; return 1; }
  timeout 60 rostopic echo -n 1 /cmd_vel >/dev/null 2>&1 || true
  # 就绪门 → /v8/bt/goal（goal＝起点+3m）→ 等 BT outcome 或 120 s sim 预算
  python3 - <<'PYEOF' > "$LOGDIR/stageB.txt" 2>&1
import math, time, json, rospy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import tf as tf_py
rospy.init_node('m2_stage_b')
S=dict(gt=None,odom=None,pose=None,obs_at=None,loc=None,bts=None,goal_sent=False,poseerr=0)
rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m:[S.update(gt=(p.position.x,p.position.y))
    for p,n in zip(m.pose,m.name) if n=='lite3_planar'] and None, queue_size=1)
rospy.Subscriber('/sim/odom_gt', Odometry, lambda m:S.update(odom=m), queue_size=2)  # 批1/F-D：/Odometry 归 faster_lio
rospy.Subscriber('/v8/local/pose', PoseStamped, lambda m:S.update(pose=m), queue_size=1)
rospy.Subscriber('/v8/local/obstacles', String,
                 lambda m:S.update(obs_at=rospy.get_rostime().to_sec()), queue_size=1)
rospy.Subscriber('/v8/local/localization_ok', Bool, lambda m:S.update(loc=m.data), queue_size=1)
def on_bts(m):
    try: S.update(bts=json.loads(m.data))
    except Exception: pass
rospy.Subscriber('/v8/bt/status', String, on_bts, queue_size=1)
POSE_ERR={'E_LOCAL_POSE_STALE','E_LOCAL_POSE_INVALID','E_LOCAL_LOCALIZATION'}
def on_ls(m):
    try: r=(json.loads(m.data) or {}).get('reason') or ''
    except Exception: return
    if S['goal_sent'] and r in POSE_ERR: S['poseerr']+=1
rospy.Subscriber('/v8/local/status', String, on_ls, queue_size=5)

w0=time.time()
while (S['gt'] is None or S['odom'] is None) and time.time()-w0<60: time.sleep(0.1)
sx,sy=S['gt'][0],S['gt'][1]; gx,gy=sx+3.0,sy
print('SMOKE start=(%.3f,%.3f) goal=(%.3f,%.3f)'%(sx,sy,gx,gy))
# —— 就绪门（G1'—G4'，总预算 120s；任一未过＝FAIL 环境不进闭环，不计入到达判据）——
#   与旧版（M2-R2 G1—G4 / M2R3 G5 系）同一语义：goal 前环境就绪＋输入新鲜；主题换为 v8 链输入面。
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
def support_map_ok():
    d = json.loads(rospy.wait_for_message('/v8_support_map', String, timeout=2.0).data)
    return bool(d.get('surfaces'))
ok_smap = gate_wait('support_map', support_map_ok)
ok_bt   = gate_wait('bt_lease', lambda: rospy.wait_for_message('/v8/bt/lease', Bool, timeout=2.0).data is not None)
ok_loc  = gate_wait('loc_ok',    lambda: S['loc'] is True)
ok_tf   = gate_wait('tf_robot',  lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
# —— 输入新鲜度门（G5'/G5c2'，取代旧 TF 时基门）：pose 50 Hz / obstacles 10 Hz 各连续 3 次 ≤1.0 s ——
def fresh_pose(need=3, gap=0.2, lim=1.0):
    ds=[]
    for _ in range(need):
        if S['pose'] is None: raise RuntimeError('no_pose')
        d=(rospy.get_rostime()-S['pose'].header.stamp).to_sec()
        if d>lim: return False,'age=%.3f>%.1f'%(d,lim)
        ds.append('%.3f'%d); time.sleep(gap)
    return True,'ages=['+','.join(ds)+']'
def fresh_obs(need=3, gap=0.2, lim=1.0):
    ds=[]
    for _ in range(need):
        if S['obs_at'] is None: raise RuntimeError('no_obs')
        d=rospy.get_rostime().to_sec()-S['obs_at']
        if d>lim: return False,'age=%.3f>%.1f'%(d,lim)
        ds.append('%.3f'%d); time.sleep(gap)
    return True,'ages=['+','.join(ds)+']'
DET = {}
def gate_wait_d(name, fn):
    w = time.time()
    while time.time() - w < gate_left() and not rospy.is_shutdown():
        try:
            ok, det = fn()
            DET[name] = det
            if ok:
                print('SMOKE gate_%s=PASS t=%.1fs %s' % (name, time.time() - gate_t0, det)); return True
        except Exception as e:
            import traceback as _tb
            DET[name] = 'exc=%s: %s | %s' % (type(e).__name__, e, _tb.format_exc().strip().splitlines()[-2:-1])
        time.sleep(0.3)
    print('SMOKE gate_%s=FAIL t=%.1fs %s' % (name, time.time() - gate_t0, DET.get(name, '?'))); return False
ok_pose = gate_wait_d('pose_fresh_G5', fresh_pose)
ok_obs  = gate_wait_d('obs_fresh_G5c2', fresh_obs)
# 取龄必须走 tf2 Buffer（G5c1 结构哨兵沿用；老 tf.TransformListener 无 lookup_transform）
import tf2_ros
buf2 = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(buf2)
def _age_of(a, b):
    st = buf2.lookup_transform(a, b, rospy.Time(0), rospy.Duration(2.0))
    s = st.header.stamp
    if s == rospy.Time(0): return None, 'G5b_hit=zero_stamp'
    return (rospy.get_rostime() - s).to_sec(), None
def g5_sample(a, b, need=3, gap=0.2, lim=1.0):
    ds = []
    for _ in range(need):
        d, why = _age_of(a, b)
        if why: return False, why
        if d > lim: return False, 'age=%.3f>%.1f' % (d, lim)
        ds.append('%.3f' % d); time.sleep(gap)
    return True, 'ages=[' + ','.join(ds) + ']'
# —— 加压档（G5_DELAY_SIM）：同跑「取龄点2」，口径＝观测延时不计入门控预算（gate_t0 加回）。——
import os
G5_DELAY_SIM = float(os.environ.get('G5_DELAY_SIM', '0') or 0)
def sim_wait_until(sec, budget=300.0):
    w = time.time()
    while time.time() - w < budget and not rospy.is_shutdown():
        if rospy.get_rostime().to_sec() >= sec: return True
        time.sleep(0.5)
    return False
ok_g5b = True; ok_g5c2b = True; ok_press = True  # 未开档时三项恒真
sim_after_p1 = rospy.get_rostime().to_sec()
print('SMOKE SIMNOW point=age_sample_1 sim_now=%.2fs gate_t=%.1fs' % (sim_after_p1, time.time()-gate_t0))
if G5_DELAY_SIM > 0:
    _raw_t = time.time() - gate_t0
    _w = time.time()
    _r = sim_wait_until(G5_DELAY_SIM)
    _spent = time.time() - _w
    gate_t0 += _spent
    print('SMOKE G5_DELAY_SIM target=%.1fs reached=%s sim_now=%.2fs wall_spent=%.1fs t_raw=%.1fs t_adj=%.1fs'
          % (G5_DELAY_SIM, _r, rospy.get_rostime().to_sec(), _spent, _raw_t, time.time() - gate_t0))
    ok_g5b   = gate_wait_d('pose_fresh_late_G5', fresh_pose)
    ok_g5c2b = gate_wait_d('obs_fresh_late_G5c2', fresh_obs)
    _sim2 = rospy.get_rostime().to_sec()
    _span = _sim2 - sim_after_p1
    print('SMOKE SIMNOW point=age_sample_2 sim_now=%.2fs gate_t=%.1fs sim_span_vs_point1=%.2fs'
          % (_sim2, time.time()-gate_t0, _span))
    _b_abs = (_sim2 >= 60.0 and _span >= 35.0)
    _b_span = (_span >= 40.0)
    ok_press = bool(_r) and (_b_abs or _b_span)
    print('SMOKE gate_press_late=%s branch=%s (reached=%s sim2=%.2f span=%.2f 判据=(sim2>=60 且 span>=35) 或 span>=40)'
          % ('PASS' if ok_press else 'FAIL',
             ('unreached' if not _r else ('abs+span' if _b_abs else ('span' if _b_span else 'NONE'))), _r, _sim2, _span))
# G5c1（odom→base_link）自 M2R5 起为阻断门，本版原样保留：链无关的 xacro 结构哨兵
_c1ok, _c1det = g5_sample('odom', 'base_link')
ok_g5c1 = _c1ok
print('SMOKE gate_G5c1(odom->base_link)=%s %s' % ('PASS' if ok_g5c1 else 'FAIL', _c1det))
# —— [M2R4] E3：门控时刻对原始 /tf + /tf_static 直读一次（只打印、不进 gate 字典）——
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
          % (sorted(par.get('base_link', [])), len(seen)))
    try:
        st = rospy.wait_for_message('/tf_static', TFMessage, timeout=3.0)
        for tr in st.transforms:
            print('SMOKE E3 static %s->%s stamp=%.3f'
                  % (tr.header.frame_id, tr.child_frame_id, tr.header.stamp.to_sec()))
    except Exception as e:
        print('SMOKE E3 static=ERR %s' % type(e).__name__)
e3_raw_tf()
gate = dict(support_map=ok_smap, bt_lease=ok_bt, loc_ok=ok_loc, tf_robot=ok_tf,
            pose_fresh_G5=ok_pose, obs_fresh_G5c2=ok_obs, tf_fresh_G5c1=ok_g5c1,
            pose_fresh_late_G5=ok_g5b, obs_fresh_late_G5c2=ok_g5c2b, press_late_G5=ok_press)
if not all(gate.values()):
    print('SMOKE gate=FAIL t=%.1fs %s' % (time.time()-gate_t0, gate)); raise SystemExit(2)
print('SMOKE gate=PASS t=%.1fs' % (time.time()-gate_t0))
g=PoseStamped(); g.header.frame_id='map'; g.header.stamp=rospy.Time.now()
g.pose.position.x=gx; g.pose.position.y=gy; g.pose.orientation.w=1.0
pub=rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=5)
time.sleep(0.5)
t0=rospy.get_rostime(); t0w=time.time(); S['goal_sent']=True; pub.publish(g)
print('SMOKE SIMNOW point=goal_send sim_now=%.2fs' % t0.to_sec())
outcome=''
while not rospy.is_shutdown():
    b=S['bts']
    if b and b.get('outcome'):
        outcome=str(b['outcome']); break
    if rospy.get_rostime().to_sec()-t0.to_sec() > 120.0: break
    if time.time()-t0w > 200.0: break   # 墙钟兜底：sim 停钟时不悬挂
    time.sleep(0.2)
dt=(rospy.get_rostime()-t0).to_sec()
print('SMOKE SIMNOW point=goal_done sim_now=%.2fs dt=%.2f' % (rospy.get_rostime().to_sec(), dt))
m=S['odom']; d=math.hypot(m.pose.pose.position.x-gx, m.pose.pose.position.y-gy)
gd=math.hypot(S['gt'][0]-gx, S['gt'][1]-gy)
print('SMOKE bt_outcome=%s dt=%.1fs dev_odom=%.3f dev_gt=%.3f pose_class_errors=%d'
      % (outcome or 'NONE', dt, d, gd, S['poseerr']))
PYEOF
  cat "$LOGDIR/stageB.txt"
  if grep -q "SMOKE gate=FAIL" "$LOGDIR/stageB.txt"; then
    # R20 四：判定行按键分类。加压档属观测口径未达标、新鲜度/结构属回归，不得混打成「环境未就绪」。
    _k=$(grep -oE "'[a-zA-Z_0-9]+': False" "$LOGDIR/stageB.txt" | tr -d "':" | sed 's/^ *//' | paste -sd, -)
    case "$_k" in
      *press_late_G5*) mark 0 "B: 门控未过（键=$_k ⇒ 加压档观测口径未达标，非环境故障、非输入新鲜度回归）";;
      *fresh*|*G5c1*)  mark 0 "B: 门控未过（键=$_k ⇒ v8 输入新鲜度/结构回归）";;
      *)               mark 0 "B: 门控未过（键=${_k:-未打印} ⇒ 环境未就绪，FAIL 环境）";;
    esac
    return 1
  fi
  grep -q "SMOKE gate=PASS" "$LOGDIR/stageB.txt" && mark 1 "B: 门控通过（≤120s，goal 前就绪）" || mark 0 "B: 门控结果缺失"
  # G6'（原 [M2R3] G6 的 v8 等价）：goal 之后 pose 类 ERROR 命中数必须为 0。
  #   双面：status 采样（pose_class_errors，goal_sent 后才计数）＋ btB.log 文本哨兵。
  #   grep -c 计数为 0 时退出码 1，故照旧用单值归一取数（坑：不可接 || echo 0）。
  G6S=$(grep -o "pose_class_errors=[0-9]*" "$LOGDIR/stageB.txt" | head -1 | cut -d= -f2); G6S=${G6S:-0}
  G6L=$(grep -acE "E_LOCAL_POSE_STALE|E_LOCAL_POSE_INVALID|E_LOCAL_LOCALIZATION" "$LOGDIR/btB.log" 2>/dev/null | head -1); G6L=${G6L:-0}
  [ "$G6S" -eq 0 ] 2>/dev/null && [ "$G6L" -eq 0 ] 2>/dev/null \
    && mark 1 "B/G6: goal 后 pose 类错误=0（status 0／日志 0）" \
    || mark 0 "B/G6: pose 类错误 status=${G6S} 日志=${G6L} 命中（v8 输入新鲜度回归）"
  get() { grep -o "$1=[-0-9.e]*" "$LOGDIR/stageB.txt" | head -1 | cut -d= -f2; }
  D=$(get dev_odom); T=$(get dt)
  awk -v a="$D" 'BEGIN{exit !(a<=0.35)}' && mark 1 "B: 到达偏差 $D ≤0.35m" || mark 0 "B: 到达偏差 $D >0.35m"
  awk -v a="$T" 'BEGIN{exit !(a<=120.0)}' && mark 1 "B: 用时 $T ≤120s" || mark 0 "B: 用时 $T >120s"
}

# ============ Stage C：hdl 重定位（M2-3） ============
stage_c() {
  # ★段首硬清场★（2026-11-27 实测）：上一轮残留的 gazebo/rosmaster/launch 会让本段的
  # `wait_topic` 立刻通过（连着旧栈），驱动却跑在旧状态上、结果文件不产出，
  # 表现为 "驱动未产出结果文件"。这里先按进程面清干净再起。
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
  pkill -f "roslaunch.*m2_" 2>/dev/null; pkill -f "roslaunch.*mode_sim_" 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f "rosmaster --core -p 11313" 2>/dev/null
  sleep 4
  SEG=C
  # C-1 建图（pcd_save:=true）：复用 Stage A 驱动序列走一遍（out/turn/back），优雅 SIGINT 落盘
  # ★先挂一个 /livox/lidar 订阅者★（2026-11-27 实测）：该 mid360 插件**在有订阅者之后**才发布点云，
  # 冷启动时 `wait_topic /rslidar_points` 会等到超时（实跑三次都卡在这里，插件日志却显示已加载）。
  # 这里只是让传感器激活，不改变任何判据，也不重打时间戳。
  roslaunch lite3_gazebo_sim m2_gazebo.launch > "$LOGDIR/gazeboC.log" 2>&1 &
  # ★顺序很关键★（2026-11-27 实测两次）：master 是**这个 launch 拉起来的**，抢在它之前挂订阅者
  # 只会得到 "Unable to communicate with master!"。正确做法＝先等 master 就绪，再挂**常驻**订阅者
  # （`-n1` 拿到一条就退出，传感器随即失活，实测仍等不到 /rslidar_points）。
  prime_lidar C1
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
rospy.Subscriber('/sim/odom_gt',Odometry,lambda m:S.update(o=m),queue_size=2)  # 批1/F-D 改名配套
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
def wrap(a):
    """角度跨界：差值的绝对值要折到 [-pi, pi]，否则 ±180° 附近会判成没转够。"""
    return (a + math.pi) % (2*math.pi) - math.pi

def back_done():
    p = S['o'].pose.pose.position
    # ★与 Stage A 同修法★：终点窗 0.4 m，另加"沿出程轴已回到起点以内"——
    # 定速开环只要跨过终点圆，单一条 hypot 判据就永不成立（Stage A 曾因此跑到 27.17 m）。
    return (math.hypot(p.x-sx, p.y) <= 0.4) or ((p.x-sx) <= 0.3)

res = []
res.append(('fwd_out', ph(0.5, 0.0, lambda: S['o'].pose.pose.position.x - sx >= 4.85)))
res.append(('turn180', ph(0.0, 0.6, lambda: abs(wrap(qyaw(S['iq']) - y0)) >= math.radians(178.0))))
res.append(('fwd_back', ph(0.5, 0.0, back_done)))
rospy.sleep(2.0)
# ★返回值必须落盘被外壳检查★：原来三个 ph() 的返回值直接丢掉，只靠 sleep 150 就去收地图。
import json as _json, os as _os
_out = _os.environ.get('M2_DRIVE_OUT', '/tmp/m2_driveC.json')
with open(_out, 'w') as fh:
    fh.write(_json.dumps({'phases': [{'name': n, 'ok': bool(v)} for n, v in res],
                          'all_ok': all(v for _, v in res)}, ensure_ascii=False) + '\n')
print('DRIVE ' + _json.dumps(res))
PYEOF
  # ★等驱动**真的跑完**，不再固定 sleep 150★：固定等待既可能在没跑完时就收图，
  # 也可能在跑飞时白等；同时检查三个阶段返回值（任一 False 即判 C 失败，不进地图验收）。
  _drv="${TMPDIR:-/tmp}/m2_driveC.json"; rm -f "$_drv"
  for _ in $(seq 1 300); do [ -f "$_drv" ] && break; sleep 1; done
  if [ ! -f "$_drv" ]; then
    echo "FAIL C: 驱动未产出结果文件（未跑完/未启动）"; return 1
  fi
  if ! grep -q '"all_ok": true' "$_drv"; then
    echo "FAIL C: 驱动阶段未全部完成 -> $(cat "$_drv")"; return 1
  fi
  echo "[m2] 驱动三阶段全部完成：$(cat "$_drv")"
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
  # ★观测窗采样（2026-11-27 新增）★：只判"末了还在不在册"无法回答"谁先消失、第几秒消失"。
  # 每 5 s 记一行：节点总数 / manager / relay / hdl_odom 是否有数据。循环自己在 125 s 退出。
  ( while : ; do
      _el=$(( $(date +%s) - T_HDL0 )); [ "$_el" -ge 125 ] && break
      printf '%s t=%ss nodes=%s mgr=%s relay=%s odom=%s\n' \
        "$(date +%H:%M:%S)" "$_el" \
        "$(rosnode list 2>/dev/null | wc -l)" \
        "$(rosnode list 2>/dev/null | grep -c 'nodelet_manager')" \
        "$(rosnode list 2>/dev/null | grep -c 'imu_relay')" \
        "$(timeout 2 rostopic hz /hdl_odom 2>/dev/null | grep -c 'average rate')" \
        >> "$LOGDIR/stageC_alive.tsv"
      sleep 5
    done ) &
  ALIVE_SAMPLER=$!
  local hdl_launch
  # 同 Stage B：`.profiles/x86_sim` 里有同内容的 lite3_bringup，包名解析会撞
  # RLException: multiple files named [...]; 优先用 ws/src 下的规范件。
  hdl_launch=$(ls -1 "${M2_WS:-/opt/ros1-slam/ws}/src"/*/lite3_bringup/launch/mode_sim_hdl.launch 2>/dev/null | head -1)
  # 诊断口：HDL_USE_IMU=false 只用于"关 IMU"单变量对照，**不作为正式 PASS**。
  roslaunch "${hdl_launch:-lite3_bringup}" use_imu:="${HDL_USE_IMU:-true}" > "$LOGDIR/hdlC.log" 2>&1 &
  prime_lidar C2
  wait_topic /rslidar_points 480 || { echo "FAIL C: /rslidar_points 未出（relocalization 段，已挂常驻订阅）"; return 1; }
  timeout 60 rostopic echo -n 1 /hdl_odom >/dev/null 2>&1 || true
  # ★GMS 地图参数必须按**实际命名空间**取★（2026-11-27 真人指出并已核实）：
  # 正式件 `ws/src/20_localization/lite3_localization/launch/hdl_localization.launch:77-79`
  # 的节点名是 `globalmap_server_nodelet`，其 `<param name="globalmap_pcd">` 落在**节点私有命名空间**
  # ⇒ 键是 `/globalmap_server_nodelet/globalmap_pcd`。本 launch 下**不存在** `/gms/globalmap_pcd`
  # （`/gms/*` 只出现在 asan_hdl_repro.sh 里，且那份 nodelet 读 private_nh，根本绑不上）。
  # 所以旧写法取到空值 ⇒ 曾被我判成"GMS 未启动"，属**探针查错键**，不是 GMS 没起。
  # 这里先解析实际键名，再分三种判词：节点不在册 / 键缺失 / 路径不符。
  _gms_node=$(rosnode list 2>/dev/null | grep -o '/[^ ]*globalmap_server[^ ]*' | head -1)
  _pcd_key=$(rosparam list 2>/dev/null | grep -E 'globalmap_pcd$' | head -1)
  echo "[C] GMS 节点=${_gms_node:-<不在册>} 参数键=${_pcd_key:-<缺失>}"
  if [ -z "$_gms_node" ]; then
    echo "FAIL C: GMS 节点不在册（globalmap_server_nodelet 未起，与探针键名无关）"; return 1
  fi
  if [ -z "$_pcd_key" ]; then
    echo "FAIL C: GMS 参数键缺失（${_gms_node} 私有命名空间下没有 globalmap_pcd）"; return 1
  fi
  _pcd=$(rosparam get "$_pcd_key" 2>/dev/null)
  _utm_key=$(printf '%s' "$_pcd_key" | sed 's/globalmap_pcd$/convert_utm_to_local/')
  _utm_on=$(rosparam get "$_utm_key" 2>/dev/null)
  echo "[C] 定位源地图=$_pcd sha256=$(sha256sum "$_pcd" 2>/dev/null | cut -c1-16) size=$([ -f "$_pcd" ] && stat -c %s "$_pcd" || echo MISSING)"
  echo "[C] utm 侧车=$([ -f "${_pcd}.utm" ] && echo present || echo absent) convert_utm_to_local=${_utm_on:-<未设>}"
  case "$_pcd" in
    */sim_venue.pcd) ;;
    *) echo "FAIL C: 地图路径不是本段生成的 sim_venue.pcd（$_pcd）"; return 1 ;;
  esac
  _pub=$(rostopic info /hdl_odom 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c '^ \*')
  echo "[C] /hdl_odom 发布者数=$_pub"
  [ "$_pub" = "1" ] || { echo "FAIL C: /hdl_odom 发布者数=$_pub（应恰 1，多个说明有残留栈）"; return 1; }
  # ★rostopic echo 打印 string 字段会带引号★（2026-11-27 实测：取到的是 "map" 而不是 map，
  # 于是 `[ "$_fid" = "map" ]` 恒假，把正常的定位源判成不对）。这里剥掉引号与 CR。
  _fid=$(timeout 6 rostopic echo -n1 /hdl_odom/header/frame_id 2>/dev/null | head -1 | tr -d '"\r')
  echo "[C] /hdl_odom frame_id=$_fid"
  [ "$_fid" = "map" ] || { echo "FAIL C: /hdl_odom frame_id=$_fid（应为 map）"; return 1; }
  # ★实跑 relay 身份核验★（2026-11-27）：/initialpose 与 IMU 旋转都过 relay，
  # 若跑到的是 .profiles 旧副本，本轮的修复根本没被执行，判据再对也没意义。
  # 取**进程 cmdline 里的脚本路径**（不是我们以为的路径），比 sha，并要求旋转日志出现。
  _rp=""; for _i in $(seq 1 30); do
    _rp=$(pgrep -af "m2_cmd_relay" | grep -v pgrep | head -1 | awk '{for(i=1;i<=NF;i++) if($i ~ /m2_cmd_relay\.py$/){print $i; exit}}')
    [ -n "$_rp" ] && break; sleep 1
  done
  if [ -z "$_rp" ]; then echo "FAIL C: 找不到 imu_relay 进程（relay 未起，后续判据无意义）"; return 1; fi
  _rp_cmd=$(pgrep -af "m2_cmd_relay" | grep -v pgrep | head -1)
  echo "[C] relay 路径=$_rp"
  echo "[C] relay sha256=$(sha256sum "$_rp" 2>/dev/null | cut -c1-16) 工作树sha256=$(sha256sum "$SIM_PKG/scripts/m2_cmd_relay.py" 2>/dev/null | cut -c1-16)"
  echo "[C] relay argv=$_rp_cmd"
  if ! cmp -s "$_rp" "$SIM_PKG/scripts/m2_cmd_relay.py" 2>/dev/null; then
    echo "FAIL C: 实跑 relay 与工作树不是同一份（$_rp）→ 本轮修复未生效，禁止用该次结果判收敛"; return 1
  fi
  case "$_rp_cmd" in
    *" Imu tf"*) echo "[C] relay 旋转源=tf（单一来源）" ;;
    *) echo "FAIL C: relay 未按 tf 取旋转（argv 见上）"; return 1 ;;
  esac
  M2_HDLLOG="$LOGDIR/hdlC.log" python3 - <<'PYEOF' > "$LOGDIR/stageC.txt" 2>&1
import os
import math, time, rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu
rospy.init_node('m2_stage_c_reloc')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
S=dict(gt=None,gyaw=None,ip=None,iq=None)
# ★GT 取证三件套★（2026-11-27）：①回调计数（0 次＝根本没收到 /gazebo/model_states）；
# ②模型名一次性打印（名字不符时 gt 会恒为 None，而不是零——原源码初值就是 None）；
# ③同时取 z 与 yaw，供 initialpose 用真实高度与"相对 yaw 扰动"（而不是写死 -5°）。
S['gt_n'] = 0
S['gt_names'] = None
def _on_model_states(m):
    S['gt_n'] += 1
    if S['gt_names'] is None:
        S['gt_names'] = list(m.name)
    for p, n in zip(m.pose, m.name):
        if n == 'lite3_planar':
            S.update(gt=(p.position.x, p.position.y), gtz=p.position.z,
                     gyaw=math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                                     1-2*(p.orientation.y**2+p.orientation.z**2)))
            return
rospy.Subscriber('/gazebo/model_states', ModelStates, _on_model_states, queue_size=1)
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
# ★/initialpose 的真实类型是 PoseWithCovarianceStamped★（2026-11-27 核实）：
#   hdl_localization_nodelet.cpp:65 订阅 `/initialpose`，:394 回调签名是
#   `const geometry_msgs::PoseWithCovarianceStampedConstPtr&`。
#   原实现发 PoseStamped（类型不符），ROS 层根本连不上 ⇒ HDL 从未收到初值，
#   于是从缺省位姿起配准、极易发散（与实测 ~1 km 偏差吻合）。这里按真实接口发。
# 初值构造：位置取 GT + (0.15, -0.10) 平移扰动；z 取 GT 的真实高度；
# yaw 取 **GT 真实 yaw + (-5°)** 的相对扰动，差值用 wrap 折到 [-pi, pi]，避免跨界。
def _wrap(a): return (a + math.pi) % (2*math.pi) - math.pi
_yaw0 = S.get('gyaw')
if _yaw0 is None:
    print('SMOKE initialpose=SKIP gt_yaw_unknown'); raise SystemExit(3)
_yp = _wrap(_yaw0 + math.radians(-5.0))
ip = PoseWithCovarianceStamped(); ip.header.frame_id='map'; ip.header.stamp=rospy.Time.now()
ip.pose.pose.position.x=S['gt'][0]+0.15; ip.pose.pose.position.y=S['gt'][1]-0.10
ip.pose.pose.position.z=S.get('gtz') or 0.0
ip.pose.pose.orientation.z=math.sin(_yp/2.0); ip.pose.pose.orientation.w=math.cos(_yp/2.0)
cv=ip.pose.covariance
for _i in (0,7): cv[_i]=0.25          # x/y: 0.5 m 1sigma（与 ±0.15 m 扰动同量级）
cv[14]=0.01                            # z: 0.1 m
cv[35]=0.0685                          # yaw: 15° 1sigma
pubi=rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
_conn = pubi.get_num_connections()
for _ in range(5): pubi.publish(ip); time.sleep(0.2)
print('SMOKE gt_n=%d gt=(%.3f,%.3f,%.3f) gt_yaw=%.1fdeg inject=(%.3f,%.3f,%.3f) inject_yaw=%.1fdeg subscribers=%d'
      % (S['gt_n'], S['gt'][0], S['gt'][1], S.get('gtz') or -1, math.degrees(_yaw0),
         ip.pose.pose.position.x, ip.pose.pose.position.y, ip.pose.pose.position.z,
         math.degrees(_yp), _conn))
print('SMOKE gt_models=%s' % (','.join(S['gt_names'] or [])[:300]))
# ★初值是否真被消费★：HDL 在 initialpose_callback 里打 `initial pose received!!`
# （hdl_localization_nodelet.cpp:395）。只有它出现，才能说"初值进了估计器"；
# 发布端 `subscribers=N` 只证明连上了，不证明被处理。
_ip_hit = 0
for _ in range(20):
    if os.path.exists(os.environ.get('M2_HDLLOG', '')):
        try:
            _ip_hit = open(os.environ['M2_HDLLOG'], errors='ignore').read().count('initial pose received!!')
        except Exception:
            _ip_hit = 0
    if _ip_hit: break
    time.sleep(0.5)
print('SMOKE initial_pose_consumed=%d' % _ip_hit)
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
# ★30s 窗逐样本落盘★（2026-11-27）：只给 worst 一个数，无法区分"缓慢漂移"还是"某一刻跳变"，
# 也就没法判断是配准退化还是重定位解跳。这里每 0.5 s 记一行 d/yaw，供事后对时刻。
_trace = open((os.environ.get('M2_TRACE_C') or '/tmp/m2_stageC_trace.tsv'), 'w')
_trace.write('t\td\tyaw_err_deg\tgt_x\tgt_y\n'); _trace.flush()
_tick = 0.0
while time.time()-t0b<30 and not rospy.is_shutdown():
    m=S['ip']
    if m is not None:
        d=math.hypot(m.pose.pose.position.x-S['gt'][0], m.pose.pose.position.y-S['gt'][1])
        worst=max(worst, d)
        if True:   # 逐样本记录（20 Hz）：0.5 s 抽样会把单帧尖峰漏掉，而 worst 是全样本最大值
            _tick += 0.5
            _trace.write('%.1f\t%.3f\t%.1f\t%.3f\t%.3f\n' % (
                time.time()-t0b, d,
                abs(math.degrees(qyaw(m.pose.pose.orientation)-S['gyaw'])),
                S['gt'][0], S['gt'][1])); _trace.flush()
    time.sleep(0.05)
_trace.close()
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
  wait "$ALIVE_SAMPLER" 2>/dev/null || true
  echo "[C] 观测窗采样首行：$(head -1 "$LOGDIR/stageC_alive.tsv" 2>/dev/null)"
  echo "[C] 观测窗采样末行：$(tail -1 "$LOGDIR/stageC_alive.tsv" 2>/dev/null)"
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
  # ★`rosnode list` 输出带前导斜杠★：原来 grep -qx "velodyne_nodelet_manager" 永远匹配不上，
  # 这条"旁证"因此**结构性恒 FAIL**（2026-11-27 实测：窗口内采样 nodes=12 稳定，却仍判不在册）。
  rosnode list 2>/dev/null | grep -qx "/velodyne_nodelet_manager" \
    && mark 1 "C: 稳定性旁证（manager 观测窗末在册存活）" || mark 0 "C: 稳定性旁证（manager 观测窗末不在册）"
  # 静置微动（真实底盘静止，仅扰动指令零速）判据旁证：发零速
  true
}

# 2026-11-21：段内提前中止（环境未就绪的 return 1）此前不落 mark ⇒ 顶层仍打印 PASS（假绿）。
#   补：段返回非零且本段无其他失败 mark 时补一条 mark 0，使 SUMMARY/verdict 与段结果一致。
abort_mark() { local seg=$1 rc=$2; [ "$rc" = 0 ] && return 0; [ "$NFMARK" -gt 0 ] && return 0; mark 0 "$seg: 段提前中止（环境未就绪，见上方 FAIL 行）"; }
case "$STAGE" in
  stageA) stage_a; abort_mark A $? ;;
  stageB) stage_b; abort_mark B $? ;;
  stageC) stage_c; abort_mark C $? ;;
  all) stage_a; abort_mark A $?; cleanup; sleep 3; stage_b; abort_mark B $?; cleanup; sleep 3; stage_c; abort_mark C $? ;;
  *) echo "用法: $0 [stageA|stageB|stageC|all]"; exit 2 ;;
esac

# ============ 退净核验（GAP-5，O2 勘误：比对前移 trap 已兜底） ============
if [ -f "$MAP" ] && [ -f "$LOGDIR/pcd.sha256.before" ]; then
  sha256sum "$MAP" | awk '{print $1}' > "$LOGDIR/pcd.sha256.after"
  cmp -s "$LOGDIR/pcd.sha256.before" "$LOGDIR/pcd.sha256.after" \
    && mark 1 "样张 lite3.pcd sha256 未变" || mark 0 "样张 lite3.pcd 被改动（备份 $LOGDIR/lite3.pcd.bak）"
fi
# ============ 退净判据前移（R21/R22 落地关键） ============
# 为何必须显式调用：mark 只改 PASS 变量，而 rc 由本段之后的 verdict 打印决定；
#   若只依赖 trap cleanup EXIT，trap 是在 exit 之后才跑 ⇒ 退净判据永远影响不到 rc（假判据）。
# 故此处先 kill_all 落退净态、再 exitclean_check 加 mark；段令牌 CHECKED_SEG 让 trap 成为幂等兜底（R26 三）。
kill_all
exitclean_check
echo "SUMMARY marks pass=$NPMARK fail=$NFMARK"   # R27 四：供 driver 与原件 ^[PASS]／^[FAIL] 计数互核
echo "=================================================="
if [ "$INVALID" = "1" ]; then
  echo "M2 SMOKE: INVALID（运动窗 RTF<0.25，重跑不计 FAIL）并存判据 FAIL=$NFMARK"
  if [ "$NFMARK" -gt 0 ]; then      # #30 定档（小美 M2R8 裁定 1）：INVALID 且并存判据 FAIL ⇒ 另立一档并停手
    grep -rhoE '^\[FAIL\].*' "$LOGDIR" 2>/dev/null | sort -u | sed 's/^/  [并存FAIL明细] /'
    echo "M2 SMOKE: rc=4（INVALID 且并存 ≥1 条判据 FAIL ⇒ 停手上报，不得由重试消化；仍不入 FAIL 计数、不入连击）"
    echo "=================================================="
    exit 4
  fi
  echo "=================================================="
  exit 3
fi
[ "$PASS" = "1" ] && echo "M2 SMOKE: PASS" || echo "M2 SMOKE: FAIL"
echo "=================================================="
exit $([ "$PASS" = "1" ] && echo 0 || echo 1)
