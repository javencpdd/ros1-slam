#!/usr/bin/env bash
# w3_survey_run.sh —— 矩阵W3 perlin3d 勘察驱动（小明 2026-10-07）
# 独立 mockamap（perlin3d 模板参数=95_sim W2 T1 同值）→ 修法版勘察（封边①模板地形边+②云覆盖带）
# → 产出 /tmp/matrix_w3/perlin3d_goals.json + traj_perlin3d.yaml → 退净
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
LOGDIR=/tmp/matrix_w3/logs; mkdir -p "$LOGDIR"

cleanup() {
  rosnode kill -a 2>/dev/null; sleep 2
  pkill -f "roslaunch.*survey_perlin3d" 2>/dev/null; pkill -f mockamap_node 2>/dev/null; sleep 2
}
trap cleanup EXIT

echo "[w3-survey] 起独立 mockamap（perlin3d 40x20x5 seed=511 res=0.15）"
setsid roslaunch /tmp/matrix_w3/survey_perlin3d.launch > "$LOGDIR/survey_perlin3d_launch.log" 2>&1 &
sleep 5
echo "[w3-survey] 跑修法版勘察（4 goals → 正反向 8 对）"
python3 /tmp/matrix_w3/w2_survey.py perlin3d 40 0.1 4 40 20 /mock_map 2>&1 | tee "$LOGDIR/survey_perlin3d_w3.log"
RC=${PIPESTATUS[0]}
echo "[w3-survey] rc=$RC 退净复核"
sleep 2
bash -c 'unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313
rosnode list 2>/dev/null | grep -v "^/rosout$" | wc -l' | xargs echo "[w3-survey] 残留节点数(除 rosout):"
exit $RC
