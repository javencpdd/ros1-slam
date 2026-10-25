#!/usr/bin/env bash
# R660a 仿真运行域基线：headless Gazebo（stairstudy 楼梯世界）容器内烟测。
# 采集教训（首两跑实证）：容器 4 核限载下 gzserver RTF 可低至 ~0.006，
# /clock 墙钟间隔可达 ~15 s ⇒ echo 短窗必超时；证流改用两条戳递增＋日志 sim 戳。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r660/logs
mkdir -p /opt/ros1-slam/r660/logs

WORLD="${1:-stair_lab_stairs_easy.world}"

roslaunch lite3_gazebo_sim stairstudy.launch world_file:="$WORLD" \
  > /opt/ros1-slam/r660/logs/roslaunch-sim.log 2>&1 &
RL_PID=$!
cleanup() {
  [ -n "${RL_PID:-}" ] && kill -INT "$RL_PID" 2>/dev/null; sleep 6
  [ -n "${RL_PID:-}" ] && kill "$RL_PID" 2>/dev/null
  pkill -INT -f gzserver 2>/dev/null; sleep 2
  pkill -f gzserver 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 1 90); do
  rosparam get /world_load_assert_rc >/dev/null 2>&1 && break
  sleep 1
done
sleep 6

NODES=$(rosnode list 2>/dev/null | wc -l)
CLOCK_PAIR=$(timeout 45 rostopic echo -n2 /clock 2>/dev/null | grep 'secs:' | tr -dc '0-9\n' | tr '\n' ' ')
TF_OK=$(rosparam get /tf_frame_assert/ok 2>/dev/null || echo pending)
WORLD_RC=$(rosparam get /world_load_assert_rc 2>/dev/null || echo missing)
MODEL_NAMES=$(timeout 25 rosservice call /gazebo/get_world_properties 2>/dev/null | tr '\n' ' ' | grep -oE 'model_names: \[[^]]*\]' | head -1)
LIDAR_INFO=$(timeout 15 rostopic info /rslidar_points 2>/dev/null)
LIDAR_PUBS=$(echo "$LIDAR_INFO" | grep -c 'Publishers:')
LOG_SIM_STAMPS=$(grep -oE '\[[0-9.]+, [0-9.]+\]' /opt/ros1-slam/r660/logs/roslaunch-sim.log | grep -oE ', [0-9.]+\]' | tr -dc '0-9.\n' | sort -n | tail -3 | tr '\n' ' ')

python3 -B - "$WORLD" "$NODES" "$CLOCK_PAIR" "$TF_OK" "$WORLD_RC" "$MODEL_NAMES" "$LIDAR_PUBS" "$LOG_SIM_STAMPS" <<'PY'
import json, sys
(world, nodes, clock_pair, tf_ok, world_rc, model_names,
 lidar_pubs, log_sim_stamps) = sys.argv[1:9]
clocks = [float(x) for x in clock_pair.split()] if clock_pair.strip() else []
sim_stamps = [float(x) for x in log_sim_stamps.split()] if log_sim_stamps.strip() else []
models = model_names or ''
result = {
  'stage': 'sim_baseline', 'world': world,
  'ros_nodes': int(nodes),
  'clock_samples': clocks, 'clock_advancing': len(clocks) >= 2 and clocks[1] > clocks[0],
  'log_sim_stamps_latest': sim_stamps[-3:] if sim_stamps else [],
  'tf_frame_assert_ok': tf_ok,
  'world_load_assert_rc': world_rc,
  'gazebo_model_names': models or None,
  'spawn_present': 'lite3_planar' in models,
  'world_models_present': all(m in models for m in ('ground_plane', 'sun', 'stairs_easy')),
  'lidar_publisher_present': int(lidar_pubs) >= 1,
}
result['accepted'] = bool(
    result['clock_advancing'] and result['spawn_present'] and result['world_models_present']
    and result['tf_frame_assert_ok'] in ('True', 'true')
    and str(result['world_load_assert_rc']) in ('0', '0.0')
    and result['lidar_publisher_present'])
open('/opt/ros1-slam/r660/sim-baseline.json', 'w').write(json.dumps(result, indent=2) + '\n')
print('SIM_BASELINE', json.dumps(result, ensure_ascii=False))
PY
python3 -c "import json;d=json.load(open('/opt/ros1-slam/r660/sim-baseline.json'));exit(0 if d['accepted'] else 6)"
