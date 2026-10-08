#!/usr/bin/env bash
# R49 批的零节点回归（BR 打桩可测性口子改后必跑）：RAS/STS/TEC/MUX/BRS/GTC/SYN/ISO
set +u; unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash; source /home/ros1-slam/ws/devel/setup.bash; set -u
cd /home/ros1-slam
V=ws/src/95_simulation/lite3_gazebo_sim/verify
echo "### RAS"; bash $V/v_rd_archive_selftest.sh; echo "RAS rc=$?"
echo "### BRS"; bash $V/v_br_expect_negative_selftest.sh; echo "BRS rc=$?"
echo "### STS"; bash $V/v_m2_exitclean_selftest.sh | grep -E "用例 PASS|SELFTEST"; echo
echo "### TEC"; bash ws/src/40_navigation/lite3_scan/verify/v_tf_edge_conflict.sh | tail -2
echo "### MUX"; bash ws/src/60_safety/lite3_safety/verify/v_mux_sources.sh | tail -2
echo "### GTC"; bash scripts/verify/v_harness_generation_table.sh --check | grep -E "GENERATION TABLE|分母面"
echo "### SYN"; bash scripts/verify/v_sh_py_syntax.sh $V/m2_batch_runner.sh $V/v_br_expect_negative_selftest.sh $V/retake_driver.sh $V/v_rd_archive_selftest.sh $V/m2_smoke.sh $V/m2_exitclean_lib.sh; echo "SYN rc=$?"
echo "### ISO"; bash scripts/verify/v3_isolation.sh | tail -2; echo "ISO rc=$?"
