# R1050 scoped 冻结独立复核回执（非真人签署）

- 对象：R1048 18-run ROS45 blocked-goal 数据；scoped 两键
  `stop_chain_ros45_time_s=1.2`、`stop_chain_ros45_distance_m=0.21`，
  scope=`ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime`，
  sample_n=min=18。
- 复核判 **PASS（scoped）**：18/18 PASS、每速 9 互异、RTF 1.0；
  按 t_sense（首个障碍输入 wall 插值 sim）最坏 **0.8559 s / 0.15492 m**、
  前向 0.11904 m、净空 0.58210 m；1.2/0.21 均 ≥×1.30；run 末位移 0.18488<0.21。
- 要求已落：键名/scope 精确字面、mujoco（无 Gazebo）、obstacle_input（注入非感知）、
  含 0.5 s hold（0.2 m/s/0.05 m；后漂≤0.10 m，实测≤16.4 mm）、t_sense→确认点、
  res .05/seed0 未入 run_config、净空=障碍面至机身前缘(+0.25 m)、0.2 未达注入点。
- 边界：18 条件格互异、非 18 独立样本；stop 仅 4/3 档；“2/18<0.2”为末帧口径，
  插值 6/18（r06=0.168 最坏时间源）；单 seed/无同批负控/低触发速不判 FAIL。
- formal 两键仍不闭合；scoped 不替代 formal。
