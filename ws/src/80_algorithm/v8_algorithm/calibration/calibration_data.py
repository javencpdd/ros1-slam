"""V8-12 冻结判据值（机读配置）。自 calibration.py 拆出，2026-11-18 计划 M9。

★本文件是纯数据★：只放冻结值与其期望哈希，不含任何校验逻辑。
DIGEST 是 FROZEN **参数值**的哈希（不是文件哈希）——搬到这里不影响其有效性，
但**不得重排字典**：dict 插入顺序参与 json 序列化，重排会让哈希变化。

改值流程不变：改 FROZEN → 重算 digest → 同步 DIGEST → 走 unfreeze 登记。
重算：python3 -B -c "import calibration;print(calibration.digest())"
"""

FROZEN = {
    'calibration_version': 2,
    # 解封记录（R657 封版 v1 → R661 v2）：official_scenarios 6→14（SC04—06/09/11—14
    # 构造器已落，V8-13 矩阵按 kind 分派执行）；其余五节零变更。
    # 2026-10-27 判据面（小美，R713 #137）：sampling 两项按 24 §4.1 补齐、resources
    # 两项按开发机采样冻结；每条带 key/from/to/reason，供 check_loosening 识别。
    # 2026-10-27 判据面（小美，R715 #137）：latency.p95_global_s 与
    # budgets.cancel_time_s 按带域冻结（scope/sample_n/evidence 三件齐全）；
    # 值仅在声明域内成立，全栈时延/全链停车仍由 PENDING_REQUIRED 把关。
    'unfreeze_log': [
        {'to_version': 2, 'when': '2026-10-26',
         'change': 'denominators.official_scenarios 扩至全 14 场景',
         'reason': 'SC 构造器填充完成（R661），分母只认构造器在册场景的原则维持'},
        {'when': '2026-10-27',
         'key': 'sampling.scenario_repetitions.value', 'from': 1, 'to': 3,
         'change': '每场景重复次数 1→3（24 §4.1 采样补齐）',
         'reason': '24 §4.1 要求每 seed 3 次重复；原 1 次不足，按 R713 #137 显式解封，属收紧非放宽'},
        {'when': '2026-10-27',
         'key': 'sampling.perturbation_seeds.value', 'from': [], 'to': ['0', '1', '2'],
         'change': '预登记 seed 空表→0/1/2（24 §4.1 采样补齐）',
         'reason': '24 §4.1 要求每适用闭环场景至少 3 个预登记 seed；按 R713 #137 预登记 0/1/2，属收紧非放宽'},
        {'when': '2026-10-27',
         'key': 'resources.max_cpu_percent.value',
         'from': '未定标（pending）', 'to': 50.0,
         'change': '新增宿主相对负载上限 50%（开发机采样）',
         'reason': 'R625/R626 load1 8.29、R713 汇总 7.96 对 72 逻辑核≈11.1%—11.5%；按 R621 load1÷核数≤0.5 取 36 核当量，余量≈4.3 倍；开发机采样、非正式定标'},
        {'when': '2026-10-27',
         'key': 'resources.max_memory_mb.value',
         'from': '未定标（pending）', 'to': 6144,
         'change': '新增单 run 内存上限 6144 MB（开发机采样）',
         'reason': 'R625 MemAvailable=15315600 kB≈15.3 GB；运行容器内存上限 8 GiB（R625/R660），取 75%＝6144 MB 留 2 GiB 余量；开发机采样、非正式定标'},
        {'when': '2026-10-27',
         'key': 'latency.p95_global_s.value',
         'from': '未定标（pending）', 'to': 0.1394,
         'change': '新增全局规划 p95 预算 0.1394 s（带域 planner_service_only）',
         'reason': 'R715 定标首轮 3 组起终点×3 次重复共 9 次 /make_plan 实测 p95=139.4 ms'
                   '（中位 118.6 ms），与 R714 n=20 的 136.1 ms 同量级；'
                   'scope=planner_service_only（仅规划服务端到端，不含感知/调度/路径下发/执行），'
                   'sample_n=9；仅在该域内可作预算，全栈时延未测、不据此扩域；属新增判据非放宽'},
        {'when': '2026-10-27',
         'key': 'budgets.cancel_time_s.value',
         'from': '未定标（pending）', 'to': 0.0372,
         'change': '新增取消时间预算 0.0372 s（带域 mux_hold_only）',
         'reason': 'R715 定标首轮 3 次 hold→/cmd_vel 归零实测最大 37.2 ms，sample_n=3；'
                   'scope=mux_hold_only（仅 mux 保持归零，不含 follower 收尾/底盘减速/进程退出）；'
                   '全链停车仍由 budgets.stop_time_s（pending）把关；属新增判据非放宽'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_brake_peak_distance_m.value',
         'from': '未定标（pending）', 'to': 0.076,
         'change': '新增制动段前冲距离分量 0.076 m（带域 lite3_mj_dyn_hold_peak_0p3_flat）',
         'reason': 'R719/R720 0.3 m/s 平地 3 相位×3=9 次（seeds 6—8/31—33/34—36，均带运行日志）'
                   '最大前冲 0.0656 m，按判据面 +15% 取 0.076 m；只覆盖 hold→前冲峰值，'
                   '不含感知/规划/控制上游；全链 budgets.stop_distance_m 仍 pending；属新增判据非放宽'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_brake_peak_time_s.value',
         'from': '未定标（pending）', 'to': 0.387,
         'change': '新增制动段峰值时间分量 0.387 s（同带域）',
         'reason': '同 9 次 hold→前冲峰值时间最大 0.387 s；确认静止 0.8—2.0 s 属另一指标、不并入；'
                   '全链 budgets.stop_time_s 仍 pending；属新增判据非放宽'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_brake_peak_distance_m.value',
         'from': 0.076, 'to': 0.124,
         'change': '同域重定：制动段前冲距离 0.076→0.124 m（scope 加 wall_render，域语义不变）',
         'reason': 'R726 同域 3×3 九次 hold→峰值 max 0.1075 m，旧 3 相位漏掉长制动相；'
                   '×1.15 取 0.124；15 次补充 0.1081 仅绝对覆盖、不享受余量；scope 加 wall_render 编码测试条件，'
                   '域语义未变。抬高停车上限在验收语义上算放宽，'
                   'check_loosening 对 budgets 仅识别下调（规则盲区），须独立复核'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_brake_peak_time_s.value',
         'from': 0.387, 'to': 1.17,
         'change': '同域重定：制动段峰值时间 0.387→1.17 s（scope 加 wall_render，域语义不变）',
         'reason': 'R726 同域 hold→峰值 max 1.17 s（实测最大，未加 15%）；含墙/渲染线程；'
                   'scope 加 wall_render 编码测试条件；同属抬高停车上限，须独立复核'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_chain_osmesa_time_s.value',
         'from': '未定标（pending）', 'to': 2.471,
         'change': '新增 osmesa 全链代理时间 2.471 s（scope 限深度 osmesa 感知→静止）',
         'reason': 'R726 3×3 base_x 静止 max 2.471 s（0_a），15 次同；同载体同 run 同 sim 钟，'
                   '含墙/渲染；osmesa 约 3 Hz；无规划段、非目标传感器、仅平地 0.3 m/s；'
                   '不替代 budgets.stop_time_s（仍 pending）'},
        {'when': '2026-10-27',
         'key': 'budgets.stop_chain_osmesa_distance_m.value',
         'from': '未定标（pending）', 'to': 0.3306,
         'change': '新增 osmesa 全链代理距离 0.3306 m（scope 限深度 osmesa 感知→峰值）',
         'reason': 'R726 3×3 峰值前冲 max 0.3306 m（0_b），15 次同；取峰值非静止 0.3242；'
                   '同载体同 run 同 sim 钟，含墙/渲染；osmesa 约 3 Hz；无规划段、非目标传感器、'
                   '仅平地 0.3 m/s；不替代 budgets.stop_distance_m（仍 pending）'},
        {'when': '2026-10-27',
         'key': 'latency.p95_local_s.value',
         'from': '未定标（pending）', 'to': 0.8966,
         'change': '新增局部层响应 p95 0.8966 s（带域 scan_sim_consumed_cloud_to_bspline_27run）',
         'reason': 'R722 27 独立 run/470 周期实测；grid_map 记实际消费点云 stamp、FSM 计划起点打点；'
                   'scope 仅 SCAN 局部层，含输入年龄、非纯计算；0.5/f_out 不适用；属新增判据'},
        # B 方案（真人 2026-10-27 明令：NDT 降级为只做重定位/失效检测，精度门只压 LIO）。
        {'when': '2026-10-27',
         'key': 'tolerances.max_localization_error_m.value',
         'from': '未定标（pending）', 'to': 0.22,
         'change': '新增定位最大误差 0.22 m（带域 9run LIO 单后端，B 方案）',
         'reason': 'R785 9 个有效 run（3 起点×3 次，l9b 六＋l9c 三；s4—s6 出生跌落作废由 9c 同起点替换）'
                   'LIO 单后端 xy/base_link max 0.1167—0.1903（9/9≤0.194）；'
                   '按 R742 乙案取值式 T=min(ceil_cm(1.15×0.1903),0.22)=0.22，仍低于几何硬顶 0.2236；'
                   '真人 B 方案授权单后端主键；p95 0.105—0.157（4/9≤0.12）随 NDT 双后端目标退役，只作报告统计量；'
                   '0.194→0.22 属上限上调（+13.4%），按放宽纪律另做独立增量复核'},
        {'when': '2026-10-28',
         'key': 'success_rate.min_pass_rate.value',
         'from': '未定标（pending）', 'to': 0.9,
         'change': '新增成功率门槛 0.90（带域 36 组 v11：SC01/SC02/SC09 各 4 目标×3 次）',
         'reason': 'R797 预登记 36 组一遍跑完 36/36 PASS（到点 max 0.120、净空 min 0.208）；'
                   'Clopper-Pearson 单侧 95% 下限 0.9202≥0.90；判据未改，属新增判据非放宽'},
        {'when': '2026-10-28',
         'key': 'success_rate.min_pass_rate.value',
         'from': 0.9, 'to': '未定标（pending）',
         'change': '回退 R797 的冻结：独立复核（R798）不签，恢复 pending',
         'reason': '复核三问：scope 字面与实跑不符（official_positive vs run_class=calibration/替代场景）；'
                   '时限口径未裁（30 s 计 2/36、60 s 计 27/36）；批日志 CP 行二分方向反（0.0000）。'
                   '36 组读数保留作旁证，不据此判绿'},
        {'when': '2026-10-28',
         'key': 'success_rate.min_pass_rate.value',
         'from': '未定标（pending）', 'to': 0.9,
         'change': '重新冻结成功率 0.90（scope 改 flat2d 专用 token；时限不并入成功判定）',
         'reason': '真人 2026-10-28 勾选：①scope 改用 v8_flat2d_calibration_… 专用名；'
                   '②时限不并入（成功=到点+净空），先补仿真钟记录再裁是否并入。'
                   '36/36、CP 0.920153（名义值）；R798 复核三条已逐项处置（scope/时限/CP 日志）'},
        {'when': '2026-10-28',
         'key': 'success_rate.min_pass_rate.scope',
         'from': 'v8_flat2d_calibration_sc01_sc02_sc09_4group_3rep_n36_truth_mux_volume_relay_lite3_dwa_f64nli',
         'to': 'v8_flat2d_calibration_sc01_sc02_sc09_4group_3rep_n36_truth_mux_volume_relay_lite3_dwa_f64nli25',
         'change': '栈后缀 _f64nli→_f64nli25（值 0.90 不变；证据切 v12 R803 36/36）',
         'reason': '真人 2026-10-28 勾选采纳 f64nli25：R801 复现 f64nli 低频切角死锁（1/72），'
                   'R802 候选两卡点 6/6、R803 v12 全矩阵 36/36（净空 min 0.324）；属 scope 表述同步，非放宽'},
    ],
    'tolerances': {
        'tracking_lateral_m': {'value': 0.15, 'evidence': 'R655 tracking 默认'},
        'tracking_longitudinal_m': {'value': 0.20, 'evidence': 'R655'},
        'tracking_vertical_m': {'value': 0.10, 'evidence': 'R655'},
        'goal_position_m': {'value': 0.30, 'evidence': 'R656 场景任务夹具'},
        'z_support_tol_m': {'value': 0.10, 'evidence': 'R653 检查器默认'},
        'min_clearance_m': {'value': 0.20, 'evidence': 'R653'},
        # B 方案（真人 2026-10-27）：定位输出＝LIO（map→odom 启动固定），
        # NDT 只做重定位/失效检测，不进精度主键。值按 R742 乙案取值式。
        'max_localization_error_m': {
            'value': 0.22,
            'scope': 'm1_corridor_12p57m_xy_base_link_se2init_9runs_lio_only_b_ndt_monitor',
            'sample_n': 9,
            'evidence': 'R785 9 有效 run（3 起点×3 次；l9b 六＋l9c 三，s4—s6 出生跌落作废由 9c 同起点替换）'
                        'LIO 单后端 xy/base_link 对齐后 max 0.1167—0.1903（9/9≤0.194）、p95 0.105—0.157'
                        '（4/9≤0.12，仅报告不入阈值）；R742 乙案 T=min(ceil_cm(1.15×0.1903),0.22)=0.22，'
                        '低于 RSS 硬顶 0.2236；B 方案（真人 2026-10-27）令 NDT 仅重定位/失效检测，'
                        '故主键只压 LIO；z/yaw 不覆盖；11.5—12.57 m 单走廊、平地、base_link，'
                        '不含跨层/真机；NDT 监控器证据另案（非本键）'},
    },
    'budgets': {
        'planner_max_expansions': {'value': 20000, 'evidence': 'R652 plan 默认'},
        'rejection_limit_consecutive': {'value': 3, 'evidence': 'R654 MAX_REJECTIONS'},
        'task_budget_s': {'value': 30.0, 'evidence': 'R656 任务夹具'},
        'harness_deadline_s': {'value': 60.0, 'evidence': 'R656'},
        # 带域冻结（2026-10-27 判据面，小美，R715）：仅在 scope 声明域内可作预算。
        'cancel_time_s': {'value': 0.0372,
                          'scope': 'mux_hold_only',
                          'sample_n': 3,
                          'evidence': 'R715 定标首轮 3 次 hold→/cmd_vel 归零，最大 37.2 ms（n=3）；'
                                      'scope=mux_hold_only 仅 mux 保持归零，不含 follower 收尾/底盘减速/进程退出；'
                                      '全链停车由 budgets.stop_time_s（仍 pending）把关'},
        # 制动分量（2026-10-27 判据面，小美，R720；R727 同域重定）：只覆盖
        # hold→前冲峰值这一段，不含感知/规划/控制上游；全链两键仍 pending。
        'stop_brake_peak_distance_m': {'value': 0.124,
                                       'scope': 'lite3_mj_dyn_hold_peak_0p3_flat_wall_render',
                                       'sample_n': 9,
                                       'evidence': 'R726 同域 3×3 九次 hold→峰值最大 0.1075 m（旧 9 次隔离无渲染，物理域同 flat hold→peak，'
                                                   '但不在本 token 的 wall_render 测试条件内；旧 9 次 0.0656 漏了长制动相）；×1.15=0.123625，取 0.124 m；'
                                                   '15 次补充的 0.1081 m 仅绝对覆盖（0.1081×1.15=0.124315>0.124），不享受 15% 余量；'
                                                   'scope token 加 wall_render 编码 3×3 测试条件，域语义仍为 flat hold→peak；'
                                                   '不含感知/规划/控制上游与静止回摆；台阶 6 次无效、坡组触发速度 0.418—0.423 不计入本域；'
                                                   '全链 budgets.stop_distance_m 仍 pending'},
        'stop_brake_peak_time_s': {'value': 1.17,
                                   'scope': 'lite3_mj_dyn_hold_peak_0p3_flat_wall_render',
                                   'sample_n': 9,
                                   'evidence': 'R726 同域 hold→峰值时间 max 1.17 s（实测最大，未加 15%）；'
                                               'hold→静止 0.83—2.05 s 不并入；含墙/渲染线程；'
                                               '不含感知/规划/控制上游；全链 budgets.stop_time_s 仍 pending'},
        # 全链 osmesa 代理键（2026-10-27 判据面，小美，R726）：只覆盖
        # 深度 osmesa 感知→制动，不含规划段；不替代 24 §4.1 全链两键。
        'stop_chain_osmesa_time_s': {'value': 2.471,
                                     'scope': 'lite3_mj_depth_osmesa_perception_to_rest_flat_0p3_3x3',
                                     'sample_n': 9,
                                     'evidence': 'R726 3×3 base_x 静止 max 2.471 s（0_a），15 次同；'
                                                 '同载体同 run 同 sim 钟，含墙/渲染；osmesa 96×72 约 3 Hz；'
                                                 '无规划段；非目标传感器；仅平地 0.3 m/s；不替代 budgets.stop_time_s'},
        'stop_chain_osmesa_distance_m': {'value': 0.3306,
                                         'scope': 'lite3_mj_depth_osmesa_perception_to_peak_flat_0p3_3x3',
                                         'sample_n': 9,
                                         'evidence': 'R726 3×3 峰值前冲 max 0.3306 m（0_b），15 次同；'
                                                     '取峰值非静止 0.3242 m；同载体同 run 同 sim 钟，含墙/渲染；'
                                                     'osmesa 约 3 Hz；无规划段；非目标传感器；仅平地 0.3 m/s；'
                                                     '不替代 budgets.stop_distance_m'},
        # 带域 pilot（R1044，2026-10-31）：MuJoCo Lite3 力矩策略全链 pilot。
        # 感知→全局/局部规划→目标被静态障碍占据即拒（E_LOCAL_NO_ROUTE）→
        # 策略零速→物理停住；in-process，无 ROS 中间件/无真机。两键为 scoped
        # 代理，不替代 24 §4.1 的全链 stop_time_s/stop_distance_m（仍 pending）。
        'stop_chain_mj_torque_time_s': {
            'value': 3.6,
            'scope': 'mujoco_lite3_torque_inprocess_goal_blocked_flat_cmd0p2_0p4_18run_10distinct_simtime',
            'sample_n': 10,
            'evidence': 'R1044 18 run（2 命令速度 0.2/0.4 ×3 起点 ×3 触发相位）全部 PASS，'
                        '10 条互异结果；sensing→stop 最大 2.744 s（r00，含 0.5 s 静止确认窗），'
                        '×1.30=3.5672 向上取 0.1 得 3.6 s；触发前 .5 s 平均体速 0.12—0.17（cmd 0.2）'
                        '与 0.42—0.43（cmd 0.4）；标称“0.2”为命令值，实速低于命令值属策略步态；'
                        '含 Ray→support→global/local→latch→policy torque→物理静止；无 ROS 中间件/无真机；'
                        '不替代 budgets.stop_time_s（仍 pending）'},
        'stop_chain_mj_torque_distance_m': {
            'value': 0.20,
            'scope': 'mujoco_lite3_torque_inprocess_goal_blocked_flat_cmd0p2_0p4_18run_10distinct_simtime',
            'sample_n': 10,
            'evidence': 'R1044 同 18 run/10 distinct；sensing→stop 路径长最大 0.1533466 m（r00），'
                        '×1.30=0.19935 向上取 0.01 得 0.20 m；前向峰值最大 0.104878 m（r12）；'
                        '碰撞真值最小净空 0.454 m（r12），0.20 m 路径预算远低于净空；'
                        '静态障碍生成入口，机器人运动中首次感知；无 ROS 中间件/无真机；'
                        '不替代 budgets.stop_distance_m（仍 pending）'},
        # 带域 ROS 全链（R1048/R1049）：真实 ros_node + 45 策略桥/MuJoCo，平地
        # blocked-goal；0.3/0.4、3 起点×3 相位；formal 仍 pending。
        'stop_chain_ros45_time_s': {
            'value': 1.2,
            'scope': 'ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime',
            'sample_n': 18,
            'evidence': 'R1048 18 run（0.3/0.4 ×3 起点 ×3 相位）全 PASS，每速 9 互异；'
                        '按首个障碍输入 wall 插值 sim 起算最坏 0.8559 s；×1.30=1.1127 向上取 0.1 '
                        '得 1.2 s；含 0.5 s hold（0.05 m/0.2 m/s；窗后无>0.2 m/s、漂≤0.10 m，'
                        '实测≤16.4 mm）；t_sense→确认点；res .05/seed0 未入 run_config；'
                        '18 条件格互异、非 18 独立样本；stop 仅 4/3 档；不含 0.2（res .05 预算耗尽/'
                        'res .15 低命令死区）、真机、Gazebo；不替代 budgets.stop_time_s（仍 pending）'},
        'stop_chain_ros45_distance_m': {
            'value': 0.21,
            'scope': 'ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime',
            'sample_n': 18,
            'evidence': 'R1048 同 18 run/9 互异；t_sense→确认点路径长最坏 0.15492 m，×1.30=0.2014 '
                        '向上取 0.01 得 0.21 m；前向最坏 0.11904 m；净空=障碍面至机身前缘(+0.25 m) '
                        '最坏 0.58210 m；18 条件格互异、非 18 独立样本；res .05/seed0 未入 run_config；'
                        '不含 0.2、真机、Gazebo；不替代 budgets.stop_distance_m（仍 pending）'},
    },
    # 时延：带域冻结（2026-10-27 判据面，小美，R715）。p95_global 只覆盖
    # /make_plan 服务端；全栈/全场景时延未测，不得据此扩域或报正式验收。
    'latency': {
        # 局部层响应（2026-10-27 判据面，小美，R722）：仅 SCAN 运行面 consumed→bspline，
        # 含输入快照年龄、非纯计算；不含 v8 support_map/执行/全栈。
        'p95_local_s': {'value': 0.8966,
                        'scope': 'scan_sim_consumed_cloud_to_bspline_27run',
                        'sample_n': 470,
                        'evidence': 'R722 27 独立 run/470 周期（3 路径×3 seed×3 重复，每次全新 roslaunch，45 s）；'
                                    'ROS 墙钟无 /clock、RTF≈1、/cmd_vel 102—104 Hz；grid_map 记实处理点云 stamp，'
                                    'FSM 于 callReboundReplan 起点取值、bspline 发布打点；c2b p95=896.63 ms'
                                    '（p50 327.85、max 1499.95）；plan compute p95=7.97（max 125.37 离群）；'
                                    'b2c p95=9.42（max 10.06，控制器轮询）。重规划 0.333—0.511 Hz，含输入快照年龄、'
                                    '慢节拍主导，非纯计算；每 run p95 401.9—1104.9，5/27>896.63，本值 27-run 汇总口径、'
                                    '非单 run 保证；scope 仅 SCAN 随机箱体地图＋go2_kinematic_sim 局部层响应，'
                                    '不含 v8 support_map/执行/全栈；0.5/f_out 不适用，不作高频控制判定'},
        'p95_global_s': {'value': 0.1394,
                         'scope': 'planner_service_only',
                         'sample_n': 9,
                         'evidence': 'R715 定标首轮 3 组起终点×3 次重复，n=9，p95=139.4 ms、中位 118.6 ms；'
                                     'scope=planner_service_only 仅 /make_plan 服务端到端，'
                                     '不含感知/调度/路径下发/执行；与 R714 n=20 的 136.1 ms 同量级；'
                                     '单图三组起终点，非全栈定标'},
    },
    # 资源上限：开发机采样口径（R625/R626 load1 8.29/72 核、MemAvailable 15315600 kB；
    # R713 汇总 7.96/72），非正式峰值定标；正式定标须用定标集实测峰值替换。
    'resources': {
        'max_cpu_percent': {'value': 50.0,
                            'evidence': '宿主 load1 占 72 逻辑核百分比：R625/R626 8.29、R713 7.96 → 11.1%—11.5%；'
                                        '按 R621 load1÷核数≤0.5 取 50%（=36 核当量，与 R625/R633 预检 load1≤36 同口径），'
                                        '余量≈4.3 倍；开发机采样、非正式定标'},
        'max_memory_mb': {'value': 6144,
                          'evidence': 'R625 同批 /proc/meminfo MemAvailable=15315600 kB≈15.3 GB（宿主 32 GiB）；'
                                      '运行容器内存上限 8 GiB（R625/R660），取 75%＝6144 MB，留 2 GiB 页缓存/节点余量；'
                                      '开发机采样、非正式定标'},
    },
    'sampling': {
        'scenario_repetitions': {'value': 3,
                                 'evidence': '24 §4.1 每 seed 3 次重复；原 1 次为采样欠账，2026-10-27 解封补至 3（R713 #137）'},
        'perturbation_seeds': {'value': ['0', '1', '2'],
                               'evidence': '24 §4.1 每适用闭环场景至少 3 个预登记 seed；2026-10-27 预登记 0/1/2（R713 #137）'},
    },
    'denominators': {
        'official_scenarios': {'value': ['SC01', 'SC02', 'SC03', 'SC04', 'SC05', 'SC06',
                                        'SC07', 'SC08', 'SC09', 'SC10', 'SC11', 'SC12',
                                        'SC13', 'SC14'],
                               'evidence': 'v1＝已落构造器六场景；v2 解封扩至全 14（构造器齐，R661）'},
        'calibration_scenarios': {'value': ['SC01', 'SC08'],
                                  'evidence': '定标集（正向＋单向正例）'},
        'combos': {'value': ['hdl+dwa', 'hdl+teb', 'lite3+dwa', 'lite3+teb'],
                   'evidence': 'R648 组合矩阵'},
        'tracks': {'value': ['truth', 'estimate'],
                   'evidence': 'V8-13 双轨（真值轨/估计轨）'},
    },
    # 成功率：带域冻结（2026-10-28 R800 起；R804 按真人勾选把栈后缀切到 f64nli25，值不变）。
    # 证据为 v12 36/36（R803）；时限只作报告量、不并入成功判定（仿真钟数据见 R801/R803）。
    'success_rate': {
        'min_pass_rate': {
            'value': 0.9,
            'scope': 'v8_flat2d_calibration_sc01_sc02_sc09_4group_3rep_n36_'
                     'truth_mux_volume_relay_lite3_dwa_f64nli25',
            'sample_n': 36,
            'method': 'clopper_pearson_lower_95',
            'alpha': 0.05,
            'denominator': '36 = SC01 12 + SC02 12 + SC09 12（v12 预登记，FAIL 全计）',
            'aggregation': '36 组逐组 PASS/FAIL 后按 36 聚合；INVALID 不入串（本次 0 例）；'
                           '时限（task_budget_s/harness_deadline_s）不并入成功判定，只作报告量',
            'evidence': 'R803（证据切自 v12）：runs_v12/（12）+ runs_v12_sc0102/（24），'
                        'index.sha256 36/36 一致；flat2d_stack_sc09_f64nli25.launch sha256 cc03065e…7386；'
                        'RELAY_MODE=volume；36/36 PASS，到点 max 0.133、净空 min 0.324；'
                        'CP 下限 0.920153（k=36/n=36, α=0.05）为名义值（3 次重复非独立样本）；'
                        '前批 v11/v11b（f64nli 36/36、35/36）与 R798 复核记录作历史留档',
        },
    },
    'rerun_rules': {
        'max_reruns': {'value': 1, 'evidence': '失败可复跑一次，二次失败计失败'},
        'calibration_runs_count_official': {'value': False, 'evidence': '铁则：定标不入正式分母'},
    },
}

REQUIRED = {
    'calibration_version': {'types': (int,)},
    'unfreeze_log': {'types': (dict, list), 'non_empty': True},
    'tolerances': {'types': (dict,), 'non_empty': True},
    'tolerances.tracking_lateral_m.value': {'types': (int, float)},
    'tolerances.tracking_lateral_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.tracking_longitudinal_m.value': {'types': (int, float)},
    'tolerances.tracking_longitudinal_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.tracking_vertical_m.value': {'types': (int, float)},
    'tolerances.tracking_vertical_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.goal_position_m.value': {'types': (int, float)},
    'tolerances.goal_position_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.z_support_tol_m.value': {'types': (int, float)},
    'tolerances.z_support_tol_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.min_clearance_m.value': {'types': (int, float)},
    'tolerances.min_clearance_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.max_localization_error_m.value': {'types': (int, float)},
    'tolerances.max_localization_error_m.scope': {'types': (str,), 'non_empty': True},
    'tolerances.max_localization_error_m.sample_n': {'types': (int,)},
    'tolerances.max_localization_error_m.evidence': {'types': (str,), 'non_empty': True},
    'budgets': {'types': (dict,), 'non_empty': True},
    'budgets.planner_max_expansions.value': {'types': (int,)},
    'budgets.planner_max_expansions.evidence': {'types': (str,), 'non_empty': True},
    'budgets.rejection_limit_consecutive.value': {'types': (int,)},
    'budgets.rejection_limit_consecutive.evidence': {'types': (str,), 'non_empty': True},
    'budgets.task_budget_s.value': {'types': (int, float)},
    'budgets.task_budget_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.harness_deadline_s.value': {'types': (int, float)},
    'budgets.harness_deadline_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.cancel_time_s.value': {'types': (int, float)},
    'budgets.cancel_time_s.scope': {'types': (str,), 'non_empty': True},
    'budgets.cancel_time_s.sample_n': {'types': (int,)},
    'budgets.cancel_time_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_brake_peak_distance_m.value': {'types': (int, float)},
    'budgets.stop_brake_peak_distance_m.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_brake_peak_distance_m.sample_n': {'types': (int,)},
    'budgets.stop_brake_peak_distance_m.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_brake_peak_time_s.value': {'types': (int, float)},
    'budgets.stop_brake_peak_time_s.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_brake_peak_time_s.sample_n': {'types': (int,)},
    'budgets.stop_brake_peak_time_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_osmesa_time_s.value': {'types': (int, float)},
    'budgets.stop_chain_osmesa_time_s.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_osmesa_time_s.sample_n': {'types': (int,)},
    'budgets.stop_chain_osmesa_time_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_osmesa_distance_m.value': {'types': (int, float)},
    'budgets.stop_chain_osmesa_distance_m.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_osmesa_distance_m.sample_n': {'types': (int,)},
    'budgets.stop_chain_osmesa_distance_m.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_mj_torque_time_s.value': {'types': (int, float)},
    'budgets.stop_chain_mj_torque_time_s.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_mj_torque_time_s.sample_n': {'types': (int,)},
    'budgets.stop_chain_mj_torque_time_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_mj_torque_distance_m.value': {'types': (int, float)},
    'budgets.stop_chain_mj_torque_distance_m.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_mj_torque_distance_m.sample_n': {'types': (int,)},
    'budgets.stop_chain_mj_torque_distance_m.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_ros45_time_s.value': {'types': (int, float)},
    'budgets.stop_chain_ros45_time_s.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_ros45_time_s.sample_n': {'types': (int,)},
    'budgets.stop_chain_ros45_time_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_ros45_distance_m.value': {'types': (int, float)},
    'budgets.stop_chain_ros45_distance_m.scope': {'types': (str,), 'non_empty': True},
    'budgets.stop_chain_ros45_distance_m.sample_n': {'types': (int,)},
    'budgets.stop_chain_ros45_distance_m.evidence': {'types': (str,), 'non_empty': True},
    'latency': {'types': (dict,), 'non_empty': True},
    'latency.p95_global_s.value': {'types': (int, float)},
    'latency.p95_global_s.scope': {'types': (str,), 'non_empty': True},
    'latency.p95_global_s.sample_n': {'types': (int,)},
    'latency.p95_global_s.evidence': {'types': (str,), 'non_empty': True},
    'latency.p95_local_s.value': {'types': (int, float)},
    'latency.p95_local_s.scope': {'types': (str,), 'non_empty': True},
    'latency.p95_local_s.sample_n': {'types': (int,)},
    'latency.p95_local_s.evidence': {'types': (str,), 'non_empty': True},
    'resources': {'types': (dict,), 'non_empty': True},
    'resources.max_cpu_percent.value': {'types': (int, float)},
    'resources.max_cpu_percent.evidence': {'types': (str,), 'non_empty': True},
    'resources.max_memory_mb.value': {'types': (int, float)},
    'resources.max_memory_mb.evidence': {'types': (str,), 'non_empty': True},
    'sampling': {'types': (dict,), 'non_empty': True},
    'sampling.scenario_repetitions.value': {'types': (int,)},
    'sampling.scenario_repetitions.evidence': {'types': (str,), 'non_empty': True},
    'sampling.perturbation_seeds.value': {'types': (list,), 'items': (str,)},
    'sampling.perturbation_seeds.evidence': {'types': (str,), 'non_empty': True},
    'denominators': {'types': (dict,), 'non_empty': True},
    'denominators.official_scenarios.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.official_scenarios.evidence': {'types': (str,), 'non_empty': True},
    'denominators.calibration_scenarios.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.calibration_scenarios.evidence': {'types': (str,), 'non_empty': True},
    'denominators.combos.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.combos.evidence': {'types': (str,), 'non_empty': True},
    'denominators.tracks.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.tracks.evidence': {'types': (str,), 'non_empty': True},
    # success_rate 不进 REQUIRED：它有专用的 fail-closed 校验块（R748/R752），
    # 半填节点按 pending 报告而不是抛异常，R748 裸值测试锁的就是这条语义。
    'rerun_rules': {'types': (dict,), 'non_empty': True},
    'rerun_rules.max_reruns.value': {'types': (int,)},
    'rerun_rules.max_reruns.evidence': {'types': (str,), 'non_empty': True},
    'rerun_rules.calibration_runs_count_official.value': {'types': (bool,)},
    'rerun_rules.calibration_runs_count_official.evidence': {'types': (str,), 'non_empty': True},
}

PENDING_REQUIRED = [
    {'key': 'budgets.stop_time_s.value', 'types': (int, float),
     'reason': '24 §4.1 要求全链停车时间；当前有制动分量、osmesa 代理键与 R1044 in-process '
               'MuJoCo 力矩 pilot（scoped stop_chain_mj_torque_*），但全链仍缺 0.2 速度（res .05 预算耗尽/res .15 低命令死区）、真机、单 seed、同批负控；'
               'pilot 0.2 组实测平均体速 0.12—0.17、18 run 仅 10 条互异结果；保持 pending（禁 freeze）'},
    {'key': 'budgets.stop_distance_m.value', 'types': (int, float),
     'reason': '24 §4.1 要求全链停车距离并覆盖感知/控制延迟与制动；当前有制动分量、osmesa 代理键与 '
               'R1044 in-process MuJoCo 力矩 pilot（scoped stop_chain_mj_torque_*），但全链仍缺 0.2 速度、真机、单 seed、同批负控；pilot 18 run 仅 10 条互异结果；保持 pending（禁 freeze）'},
]

SCOPED_REQUIRED = {
    'latency.p95_global_s': {'scope': 'planner_service_only', 'min_sample_n': 3},
    'latency.p95_local_s': {'scope': 'scan_sim_consumed_cloud_to_bspline_27run',
                            'min_sample_n': 3},
    'budgets.cancel_time_s': {'scope': 'mux_hold_only', 'min_sample_n': 3},
    'budgets.stop_brake_peak_distance_m': {'scope': 'lite3_mj_dyn_hold_peak_0p3_flat_wall_render',
                                           'min_sample_n': 3},
    'budgets.stop_brake_peak_time_s': {'scope': 'lite3_mj_dyn_hold_peak_0p3_flat_wall_render',
                                       'min_sample_n': 3},
    'budgets.stop_chain_osmesa_time_s': {
        'scope': 'lite3_mj_depth_osmesa_perception_to_rest_flat_0p3_3x3',
        'min_sample_n': 3},
    'budgets.stop_chain_osmesa_distance_m': {
        'scope': 'lite3_mj_depth_osmesa_perception_to_peak_flat_0p3_3x3',
        'min_sample_n': 3},
    # R1044：in-process MuJoCo 力矩全链 pilot；10 条互异结果（18 run）为样本下限。
    'budgets.stop_chain_mj_torque_time_s': {
        'scope': 'mujoco_lite3_torque_inprocess_goal_blocked_flat_cmd0p2_0p4_18run_10distinct_simtime',
        'min_sample_n': 10},
    'budgets.stop_chain_mj_torque_distance_m': {
        'scope': 'mujoco_lite3_torque_inprocess_goal_blocked_flat_cmd0p2_0p4_18run_10distinct_simtime',
        'min_sample_n': 10},
    # R1048/R1049：ROS 全链 scoped；18 条件格（非 18 独立样本）为下限。
    'budgets.stop_chain_ros45_time_s': {
        'scope': 'ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime',
        'min_sample_n': 18},
    'budgets.stop_chain_ros45_distance_m': {
        'scope': 'ros_local_node_45policy_mujoco_flat_goal_blocked_obstacle_input_cmd0p3_0p4_3start_3phase_n18_simtime',
        'min_sample_n': 18},
    # B 方案（真人 2026-10-27）：LIO 单后端主键；scope 精确匹配，9run 下限防缩样。
    'tolerances.max_localization_error_m': {
        'scope': 'm1_corridor_12p57m_xy_base_link_se2init_9runs_lio_only_b_ndt_monitor',
        'min_sample_n': 9},
}

SAMPLING_MIN_SEEDS = 3

SAMPLING_MIN_REPETITIONS = 3

SUCCESS_RATE_SCOPE = ('v8_flat2d_calibration_sc01_sc02_sc09_4group_3rep_n36_'
                      'truth_mux_volume_relay_lite3_dwa_f64nli25')

DIGEST = '9c280489f14edc3954016d6756eb74a3d3239d34b2a23276edfb8de454898a28'
