#!/usr/bin/env bash
# R660a 仿真运行域基线（两 case）：
#  stairs＝stairstudy.launch（楼梯世界：模型在位/时钟/tf 门）
#  m1   ＝m1_gazebo.launch（传感流：/rslidar_points publisher 门）
# 采集教训：RTF 低时 /clock 墙钟间隔大 ⇒ 双戳递增证流；模型表取自
# /world_load_assert/loaded（launch 内置断言已调过 gazebo 服务并缓存）。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r660/logs
mkdir -p /opt/ros1-slam/r660/logs

CASE="${1:-stairs}"
if [ "$CASE" = "m1" ]; then
  LAUNCH="lite3_gazebo_sim m1_gazebo.launch"
  WAIT_PARAM="/tf_frame_assert/ok"
else
  LAUNCH="lite3_gazebo_sim stairstudy.launch world_file:=stair_lab_stairs_easy.world"
  WAIT_PARAM="/world_load_assert/ok"
fi

roslaunch $LAUNCH > /opt/ros1-slam/r660/logs/roslaunch-$CASE.log 2>&1 &
RL_PID=$!
cleanup() {
  [ -n "${RL_PID:-}" ] && kill -INT "$RL_PID" 2>/dev/null; sleep 6
  [ -n "${RL_PID:-}" ] && kill "$RL_PID" 2>/dev/null
  pkill -INT -f gzserver 2>/dev/null; sleep 2
  pkill -f gzserver 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 1 90); do
  rosparam get "$WAIT_PARAM" >/dev/null 2>&1 && break
  sleep 1
done
# 物理推进门：/clock 单条值实际变化才算仿真在走（RTF 依负载浮动，不猜固定等待）
CLOCK_A=""; CLOCK_B=""
for _ in $(seq 1 14); do
  CLOCK_A=$(timeout 20 rostopic echo -n1 /clock 2>/dev/null | grep -m1 'nsecs:' | grep -oE '[0-9]+')
  sleep 12
  CLOCK_B=$(timeout 20 rostopic echo -n1 /clock 2>/dev/null | grep -m1 'nsecs:' | grep -oE '[0-9]+')
  [ -n "$CLOCK_A" ] && [ -n "$CLOCK_B" ] && [ "$CLOCK_A" != "$CLOCK_B" ] && break
done
CLOCK_PAIR="$CLOCK_A $CLOCK_B"

NODES=$(rosnode list 2>/dev/null | wc -l)
TF_OK=$(rosparam get /tf_frame_assert/ok 2>/dev/null || echo pending)
WORLD_OK=$(rosparam get /world_load_assert/ok 2>/dev/null || echo missing)
MODELS=$(rosparam get /world_load_assert/loaded 2>/dev/null || tr -d '\n' <<< "[]")
LIDAR_PUBS=$(timeout 15 rostopic info /rslidar_points 2>/dev/null | grep -cE 'Publishers: .*[1-9]|Publishers:$' ; true)
LIDAR_LINES=$(timeout 15 rostopic info /rslidar_points 2>/dev/null | wc -l)
LOG_SIM_STAMPS=$(grep -oE '\[[0-9.]+, [0-9.]+\]' /opt/ros1-slam/r660/logs/roslaunch-$CASE.log | grep -oE ', [0-9.]+\]' | tr -dc '0-9.\n' | sort -n | tail -3 | tr '\n' ' ')

python3 -B - "$CASE" "$NODES" "$CLOCK_PAIR" "$TF_OK" "$WORLD_OK" "$MODELS" "$LIDAR_LINES" "$LOG_SIM_STAMPS" <<'PY'
import json, sys
case, nodes, clock_pair, tf_ok, world_ok, models, lidar_lines, log_sim = sys.argv[1:9]
nums = clock_pair.split()
clocks = []
for i in range(0, len(nums) - 1, 2):
    clocks.append(int(nums[i]) + int(nums[i+1]) / 1e9)
sim_stamps = [float(x) for x in log_sim.split()] if log_sim.strip() else []
result = {
  'stage': 'sim_baseline', 'case': case,
  'ros_nodes': int(nodes),
  'clock_samples': clocks,
  'clock_advancing': len(clocks) >= 2 and clocks[1] > clocks[0],
  'log_sim_stamps_latest': sim_stamps[-3:],
  'tf_frame_assert_ok': tf_ok,
  'world_load_assert_ok': world_ok,
  'models_loaded': models,
  'spawn_present': 'lite3_planar' in models,
  'world_models_present': all(m in models for m in ('ground_plane', 'stairs_easy')),  # launch 断言口径＝declared+ground_plane（sun 缺如实记录）
  'lidar_info_lines': int(lidar_lines),
  'lidar_topic_present': int(lidar_lines) > 0,
}
if case == 'm1':
    result['accepted'] = bool(result['clock_advancing']
                              and result['tf_frame_assert_ok'] in ('True', 'true')
                              and result['lidar_topic_present'])
else:
    result['accepted'] = bool(result['clock_advancing']
                              and result['tf_frame_assert_ok'] in ('True', 'true')
                              and result['world_load_assert_ok'] in ('True', 'true')
                              and result['spawn_present'] and result['world_models_present'])
open('/opt/ros1-slam/r660/sim-baseline.json', 'w').write(json.dumps(result, indent=2) + '\n')
print('SIM_BASELINE', json.dumps(result, ensure_ascii=False))
PY
python3 -c "import json;d=json.load(open('/opt/ros1-slam/r660/sim-baseline.json'));exit(0 if d['accepted'] else 6)"
