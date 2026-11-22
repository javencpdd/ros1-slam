#!/usr/bin/env bash
# B5 §4.5 到达验收 · 在 ros1-slam-sim 容器内执行
#   bash b5_arrival.sh <输出目录>
#
# 链路：mode_sim_bt.launch global_backend:=far enable_bt:=false
#   ⇒ 仿真 + 位姿源 + far_stack + path_goal + local_planner 全在，BT 不来抢 /v8/local/goal。
# 判据面：/v8/bt/arrived（适配器把它当成到达信号出口）。
#
# ★ 先读这条，否则会误读下面的绿 ★
# 单层仿真里没有第二层楼板，所以 C2／C4 只能证明「不误判」，不能证明「判对层」。
# 「到对了楼层」要 support_map 按 (x,y) 取 z（押后项 D1）与真实双层图，见计划 §4.5「做不到的部分」。
#
# 所有等待一律按墙钟（time.monotonic）算：仿真 /use_sim_time 为真，实测墙钟只推进约 0.44 倍，
# 按 ROS 时间等 60 s 会变成墙钟 136 s。
set -uo pipefail

OUT="${1:?输出目录}"
mkdir -p "$OUT"
LOG="$OUT/arrival.log"
: > "$LOG"

export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_HOME="$OUT/ros_home"
export PYTHONNOUSERSITE=1
mkdir -p "$ROS_HOME"

source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /opt/ros1-slam/far_overlay/devel/setup.bash

say() { printf '%s\n' "$*" | tee -a "$LOG"; }

# 端口必须由本次自己占：别的进程占着 11313/11345 时，后面的判据全会串味
python3 - <<'PY' >> "$LOG" 2>&1 || { say "ARRIVAL_PRECHECK=PORT_BUSY"; exit 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('ARRIVAL_PRECHECK=PORTS_FREE')
PY
grep -q 'ARRIVAL_PRECHECK=PORTS_FREE' "$LOG" || exit 9

LAUNCH_PID=""
cleanup() {
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

roslaunch lite3_bringup mode_sim_bt.launch \
    global_backend:=far enable_bt:=false \
    arrival_topic:=/v8/bt/arrived \
    far_debug:=true > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY' >> "$LOG" 2>&1 || { say "ARRIVAL_BOOT=MASTER_TIMEOUT"; exit 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 90
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('ARRIVAL_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
grep -q 'ARRIVAL_BOOT=MASTER_READY' "$LOG" || exit 5

# 小工具：观察 / 发目标 / 读位姿。参数一律用 argv，不拼 shell 字符串。
cat > "$OUT/_watch.py" <<'PY'
# 在 N 秒墙钟窗口里数 /v8/bt/arrived 的 true 条数与 /global_path 的唯一时间戳数
import sys, time, rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Path
window = float(sys.argv[1])
state = {'true': 0, 'false': 0, 'stamps': set()}
def on_arrived(msg):
    state['true' if msg.data else 'false'] += 1
def on_path(msg):
    state['stamps'].add(msg.header.stamp.to_nsec())
rospy.init_node('b5_watch', anonymous=True, disable_signals=True)
rospy.Subscriber('/v8/bt/arrived', Bool, on_arrived, queue_size=10)
rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
end = time.monotonic() + window
while not rospy.is_shutdown() and time.monotonic() < end:
    time.sleep(0.05)
print('ARRIVED_TRUE=%d ARRIVED_FALSE=%d PATH_UNIQ_STAMPS=%d'
      % (state['true'], state['false'], len(state['stamps'])))
PY

cat > "$OUT/_goal.py" <<'PY'
import sys, time, rospy
from geometry_msgs.msg import PoseStamped
x, y, z = (float(v) for v in sys.argv[1:4])
rospy.init_node('b5_goal', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/v8/global/goal', PoseStamped, queue_size=1, latch=True)
msg = PoseStamped()
msg.header.frame_id = 'map'
msg.header.stamp = rospy.Time.now()
msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
msg.pose.orientation.w = 1.0
for _ in range(5):
    pub.publish(msg)
    time.sleep(0.1)
print('GOAL_SENT %.3f %.3f %.3f' % (x, y, z))
PY

cat > "$OUT/_pose.py" <<'PY'
import sys, time, rospy
from geometry_msgs.msg import PoseStamped
got = {}
def cb(msg):
    got['p'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
rospy.init_node('b5_pose', anonymous=True, disable_signals=True)
rospy.Subscriber('/v8/local/pose', PoseStamped, cb, queue_size=1)
end = time.monotonic() + 25.0
while not rospy.is_shutdown() and time.monotonic() < end and 'p' not in got:
    time.sleep(0.05)
p = got.get('p', (float('nan'),) * 3)
print('POSE %.3f %.3f %.3f' % p)
PY

cat > "$OUT/_shift_pose.py" <<'PY'
import sys, time, rospy
from geometry_msgs.msg import PoseStamped
x, y, z, dur = (float(v) for v in sys.argv[1:5])
rospy.init_node('b5_shift_pose', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
msg = PoseStamped()
end = time.monotonic() + dur
while not rospy.is_shutdown() and time.monotonic() < end:
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    time.sleep(0.05)
PY

read_pose() { python3 "$OUT/_pose.py" 2>/dev/null | tee -a "$LOG"; }
send_goal() { python3 "$OUT/_goal.py" "$1" "$2" "$3" 2>/dev/null | tee -a "$LOG"; }
watch() { python3 "$OUT/_watch.py" "$1" 2>/dev/null | tee -a "$LOG"; }

# 冷启动窗口：先等 /way_point 真出现过，再开始判，否则读数是「还没热身」不是「判据错了」
cat > "$OUT/_waypoint_probe.py" <<'PY'
import time, rospy
from geometry_msgs.msg import PointStamped
seen = []
def cb(msg):
    seen.append(msg)
rospy.init_node('b5_wp_probe', anonymous=True, disable_signals=True)
rospy.Subscriber('/way_point', PointStamped, cb, queue_size=5)
end = time.monotonic() + 120.0
while not rospy.is_shutdown() and time.monotonic() < end and not seen:
    time.sleep(0.1)
print('WAYPOINT_SEEN=%d' % len(seen))
PY
python3 "$OUT/_waypoint_probe.py" 2>/dev/null | tee -a "$LOG"

say '==== C3 · 验收 3 偏置标定（平地静止，goal z 取地面标高 0.000）===='
read_pose
say 'OFFSET_READING=pose.z 见上一行；goal_z 固定写 0.000，两者之差即 z 偏置'

say '==== C1 · 验收 2 正常到达（goal 与位姿同点）===='
POSE_LINE=$(python3 "$OUT/_pose.py" 2>/dev/null)
say "$POSE_LINE"
GX=$(printf '%s\n' "$POSE_LINE" | awk '/^POSE/{print $2}')
GY=$(printf '%s\n' "$POSE_LINE" | awk '/^POSE/{print $3}')
GZ=$(printf '%s\n' "$POSE_LINE" | awk '/^POSE/{print $4}')
say "POSE_XY=${GX:-0} ${GY:-0}"
send_goal "${GX:-0}" "${GY:-0}" "${GZ:-0}"
watch 60

say '==== C2 · 验收 1 跨层同 xy（goal 抬 3.0 m，xy 不变）===='
send_goal "${GX:-0}" "${GY:-0}" 3.0
watch 20

say '==== C4 · 验收 1a 位姿挪层（goal 回地面标高，位姿 z 抬 3.0 m）===='
# 停掉仿真位姿源，自己按原 xy 发一条抬高的位姿：这就是「把喂给适配器的 pose 人为挪层」
rosnode kill /sim_pose_source >> "$LOG" 2>&1 || say 'C4_KILL_POSE_FAILED=1'
send_goal "${GX:-0}" "${GY:-0}" 0.0
python3 "$OUT/_shift_pose.py" "${GX:-0}" "${GY:-0}" 3.0 30 > "$OUT/shift_pose.log" 2>&1 &
sleep 3
watch 20

say '==== 收尾 ===='
cleanup
LAUNCH_PID=""
python3 - <<'PY' >> "$LOG" 2>&1
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('ARRIVAL_CLEANUP_PORTS_FREE')
PY
say 'ARRIVAL_DONE'
