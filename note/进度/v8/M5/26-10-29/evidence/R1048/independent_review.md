# R1048 独立复核回执（formal 冻结提议，非真人签署）

- 对象：18-run ROS 45 策略 blocked-goal 矩阵；提议 formal `stop_time_s=1.1`、
  `stop_distance_m=0.15`、scope `.3/.4`、n=18。
- 复核判 **FAIL（不得 formal 冻结）**，要点：
  1. 18/18 PASS、3 起点×3 相位×2 速、每速 9 互异、RTF 1.0、ROS/动力学实闭；
     最坏（原起算）0.800 s/0.1142 m/0.0825 m/0.5821 m。
  2. 触发应从**首个障碍输入**起算，而非注入后首个 applied；晚 52—96 ms，
     重算最坏 **0.856 s / 0.1549 m**；0.1549>0.15；0.856×1.30=1.113→**1.2 s**，
     30% 余量后 1.1/0.15 仍不足，重演 R852 漏位移。
  3. 0.2 重跑 map_velocity 全 0、status 全 `E_LOCAL_POSE_STALE`、仅动 1.1 cm，
     未归因；R882 的 0.2/0.4 未闭，0.3/0.4 只能 scoped。
  4. 2/18 注入时 <0.2 m/s，moving 检查弱；3 相位仅 1 seed；smoke 静止=
     0.2 m/s/0.05 m/0.5 s hold，后漂 25 mm；真静止 1.1—2.4 s，含 hold 须明示。
  5. raw 仍写 270/非 formal，45 scope 未入证，status 仍 kinematic；scope 未写
     虚拟/无 Gazebo/真机；无同批负控。
- 结论：若要冻结，只能 scoped（并先补 0.2 或裁定、按 t_sense+R881 重算、
  重签 scope、补负控）；本批未改 calibration，formal 两键仍 pending。
