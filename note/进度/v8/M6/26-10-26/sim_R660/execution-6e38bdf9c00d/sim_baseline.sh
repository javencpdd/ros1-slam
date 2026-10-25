#!/usr/bin/env bash
# R660a 仿真运行域基线：headless Gazebo（stairstudy 楼梯世界）容器内烟测。
# 不 set -u（ROS setup 链）；ROS_HOSTNAME 钉回环（network=none，R633 教训）。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r660/logs
mkdir -p /opt/ros1-slam/r660/logs

WORLD="${1:-stair_lab_stairs_easy.world}"
OUT=/opt/ros1-slam/r660/sim-baseline.json

roslaunch lite3_gazebo_sim stairstudy.launch world_file:="$WORLD" \
  > /opt/ros1-slam/r660/logs/roslaunch-sim.log 2>&1 &
RL_PID=$!
cleanup() {
  [ -n "${RL_PID:-}" ] && kill -INT "$RL_PID" 2>/dev/null; sleep 6
  [ -n "${RL_PID:-}" ] && kill "$RL_PID" 2>/dev/null
  # gzserver 兜底（roslaunch INT 后仍可能残留）
  pkill -INT -f gzserver 2>/dev/null; sleep 2
  pkill -f gzserver 2>/dev/null
}
trap cleanup EXIT

# 就绪＝tf_frame_assert 出结论（launch 内置在位断言，30 s 超时）＋模型在位
for _ in $(seq 1 90); do
  rosparam get /tf_frame_assert/ok >/dev/null 2>&1 && break
  sleep 1
done
sleep 4  # 观察窗尾＋点云起流

# 采集（就绪条件的可观测证据，不靠固定等待冒充）
TOPICS=$(rosnode list 2>/dev/null | wc -l)
CLOCK_OK=0
timeout 8 rostopic echo -n1 /clock >/dev/null 2>&1 && CLOCK_OK=1
CLOCK_T=$(timeout 8 rostopic echo -n1 /clock 2>/dev/null | grep -m1 'secs:' | tr -dc '0-9')
MODELS=$(timeout 10 rostopic service /gazebo/get_world_properties 2>/dev/null | head -40)
MODEL_NAMES=$(echo "$MODELS" | grep -oE 'model_names: \[[^]]*\]' | head -1)
LIDAR_HZ=$(timeout 12 rostopic hz /rslidar_points 2>/dev/null | grep -m1 'average rate' | grep -oE '[0-9.]+$')
TF_OK=$(rosparam get /tf_frame_assert/ok 2>/dev/null || echo pending)

python3 -B - "$WORLD" "$TOPICS" "$CLOCK_OK" "$CLOCK_T" "$LIDAR_HZ" "$TF_OK" "$MODEL_NAMES" <<'PY'
import json, sys
world, topics, clock_ok, clock_t, lidar_hz, tf_ok, model_names = sys.argv[1:8]
result = {
  'stage': 'sim_baseline', 'world': world,
  'ros_nodes': int(topics),
  'clock_publishing': bool(int(clock_ok)), 'clock_sample_secs': clock_t or None,
  'lidar_topic_hz': lidar_hz or None,
  'tf_frame_assert_ok': tf_ok,
  'gazebo_model_names': model_names or None,
  'spawn_present': 'lite3_planar' in (model_names or ''),
  'world_models_present': all(m in (model_names or '') for m in ('ground_plane', 'sun')),
  'accepted': None,
}
result['accepted'] = bool(
    result['clock_publishing'] and result['spawn_present'] and result['world_models_present']
    and result['tf_frame_assert_ok'] in ('True', 'true') and result['lidar_topic_hz'])
open('/opt/ros1-slam/r660/sim-baseline.json', 'w').write(json.dumps(result, indent=2) + '\n')
print('SIM_BASELINE', json.dumps(result, ensure_ascii=False))
PY
RC=$?
[ "$RC" -eq 0 ] || exit $RC
python3 -c "import json;d=json.load(open('/opt/ros1-slam/r660/sim-baseline.json'));exit(0 if d['accepted'] else 6)"
