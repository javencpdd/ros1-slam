"""V8-12：判据定标与正式实验封版（R657）——冻结配置（机读、无待定关键值）。

A0/A7 面「无待定关键值」：本件把 V8-08—V8-11 各设计层用到的判据值**冻结为机读配置**，
任何值缺省/待定＝E_CAL_PENDING（fail-closed，不许带 TODO 封版）。
定标集与正式集分开：定标 run 与正式 run 记录分离，**定标结果不得计入正式分母**
（23 号：V8-12 试跑用于定标，不得提前计入 V8-13 正式分母）。
封版后变更须走显式 unfreeze（记录变更与理由），不得静默改值。
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scenarios'))
import scenarios as scn  # noqa: E402

# 冻结值来源＝各批次已定判据（引用在 evidence 字段）
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
        # 全链两键（R850，2026-10-28；真人「你自行判断」授权，口径见 R849）：
        # 全链＝感知/规划触发→物理停住；仿真钟；0.4 m/s；不含 staleness 等待；
        # Gazebo lite3_planar 动力学平面载体；三次 sensed_no_route。
        'stop_time_s': {'value': 0.50,
                        'scope': 'gazebo_planar_full_chain_sensed_no_route_0p4_simtime_n3_no_staleness',
                        'sample_n': 3,
                        'evidence': 'R850 三次 sensed_no_route（0.4 m/s、Gazebo 动力学平面载体、仿真钟）：'
                                    '0.372/0.365/0.385 s，max 0.385×1.30≈0.50；不含 staleness 等待'
                                    '（另立输入看门狗）、不含真机/台阶/跨层；墙钟不并入（受 OS 调度影响）'},
        'stop_distance_m': {'value': 0.15,
                            'scope': 'gazebo_planar_full_chain_sensed_no_route_0p4_simtime_n3_no_staleness',
                            'sample_n': 3,
                            'evidence': 'R850 三次 sensed_no_route：0.1221/0.1153/0.1157 m，max 0.1221×1.23≈0.15；'
                                        '口径同 stop_time_s（全链、仿真钟、0.4 m/s、不含 staleness）；'
                                        '不代表腿部载体制动段，也不含真机'},
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

# ---------------------------------------------------------------------------
# 整改（2026-10-27，独立复核 P0）：形状 fail-closed / golden digest / 放宽守卫。
# FROZEN 的数值与 evidence 一字未改；以下常量只增加校验面。
# ---------------------------------------------------------------------------

# 必需键：键路径 → 允许类型 / 是否非空 / 列表元素类型。
# 缺键、None、空表、类型不符一律 E_CAL_PENDING（fail-closed）。
# 覆盖六节（tolerances/budgets/resources/sampling/denominators/rerun_rules）
# 全部叶子的 value 与 evidence；sampling 是否达到 4.1 门槛由 _pending_report
# 按值核验（3 seed×3 重复），空表在形状层只判合法、不据此判绿。
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

# 当前确实缺失、24 §4.1 要求但尚未定标的项：只报告 pending+blocked，
# 不塞进 FROZEN（不改已冻结数值），也绝不据此判绿。
# 2026-10-27 判据面（小美，R713 #137）：resources 两项已按开发机采样冻结，
# 从本表移出；sampling 两项由 _pending_report 按 4.1 门槛核验，达标后不再 pending。
# 2026-10-27 判据面（小美，R715 #137）：p95_global 与 cancel_time 已带域冻结，从本表
# 移出；两者只在 scope 声明域内成立，全栈时延/全链停车仍由本表把关。
# 2026-10-27 判据面（小美，R716 #137）：stop_time/stop_distance 虽有全链读数
# （follower→mux→Gazebo planar 底盘，0.3 m/s 行进中 hold：0.1265 m／108.4 ms），
# 仍裁定**不解冻、保持 pending**，两条独立理由：
#   1) 样本 n=1，低于 SCOPED_REQUIRED 的 min_sample_n=3；把下限降到 1 属放宽守卫，
#      明令禁止（同测首轮 0.3568 m／1648.5 ms 因脚本 bug 作废，R716 §3，不得计入样本）；
#   2) 载体为 kinematic planar 底盘、无制动模型：24 §4.1 的 stop_distance 预算须覆盖
#      感知/控制延迟**与制动**，无制动平台测得值物理上不含制动段，不能代表该预算；
#      stop_time 同理只含消息/控制器链路时延，不能代表含制动收尾的停车时间。
# 待 n≥3 且 stop_distance 在有制动/动力学底盘上复测后，再交判据面带域冻结。
# 2026-10-27（R720）：动态载体矩阵已补（9 次有效/0.3 平地三相、9 次/0.4、低摩擦 3、坡 3、
# 台阶 6 无效），但 24 §4.1 的全链口径仍缺感知/规划/控制上游与台阶有效样本；
# 判据面只批准另立制动分量键（stop_brake_peak_*），本表两条全链键继续 pending。
# 2026-10-27（R722）：p95_local 已按 27 独立 run、实际消费点云 stamp 的口径带域冻结，
# 从本表移出；scope 只含 SCAN 局部层响应、含输入年龄，不含 v8 support_map/全栈。
# 2026-10-27（R787）：定位键按 B 方案冻结（LIO 单后端 9run），从本表移出；
# 真人明令「NDT 降级为只做重定位/失效检测，精度门只压 LIO」。
# 2026-10-28（R797）：成功率键按 36 组 v11 定标冻结；R798 复核否决回退；R800 按真人勾选（flat2d token＋时限不并入）重新冻结，从本表移出。剩两停车键。
PENDING_REQUIRED = []  # R850：全链两键按 n=3 带域冻结，从本表移出；其余项均已落实


# 带域冻结：值只在声明 scope 与最少样本内成立；scope 精确匹配，防静默扩域。
# R716 的 stop_time/stop_distance 未列入本表（n=1＜3 且载体无制动），
# 不得为了收下这两项把对应条目的 min_sample_n 调低——那是放宽守卫。
# R720：制动分量（stop_brake_peak_*）入表；全链 stop_time/stop_distance 仍 pending，
# 分量 scope 不含感知/规划/控制上游，不得据此替代全链预算。
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
    'budgets.stop_time_s': {'scope': 'gazebo_planar_full_chain_sensed_no_route_0p4_simtime_n3_no_staleness',
                           'min_sample_n': 3},
    'budgets.stop_distance_m': {'scope': 'gazebo_planar_full_chain_sensed_no_route_0p4_simtime_n3_no_staleness',
                               'min_sample_n': 3},
    # B 方案（真人 2026-10-27）：LIO 单后端主键；scope 精确匹配，9run 下限防缩样。
    'tolerances.max_localization_error_m': {
        'scope': 'm1_corridor_12p57m_xy_base_link_se2init_9runs_lio_only_b_ndt_monitor',
        'min_sample_n': 9},
}

# 24 §4.1 采样要求：每个适用闭环场景至少 3 个预登记 seed、每 seed 3 次重复。
SAMPLING_MIN_SEEDS = 3
SAMPLING_MIN_REPETITIONS = 3

# R748/R752：成功率带域 token（flat2d 定标集）。值域、元数据类型与 scope 精确匹配
# 在 pending 阶段就 fail-closed；真正冻结仍需改 FROZEN＋digest＋unfreeze＋独立复核。
# 2026-10-28（R800，真人勾选）：token 由 `official_positive…` 改为 flat2d 专用名。
# 理由（R798 复核 #1）：实跑是 run_class=calibration 的 flat2d 替代场景（SC01/02/09 与官方同号
# 不同义，官方 SC09 为净空负例）、单组合 lite3_dwa、volume 中继；旧名字实不符。
# 2026-10-28（R804，真人勾选）：栈后缀 _f64nli→_f64nli25——R803 证明 f64nli 有低频切角死锁
# （1/72），f64nli25（局部 inflation 0.25/5.0）在 v12 36 组上 36/36；值 0.90 与判据不变。
SUCCESS_RATE_SCOPE = ('v8_flat2d_calibration_sc01_sc02_sc09_4group_3rep_n36_'
                      'truth_mux_volume_relay_lite3_dwa_f64nli25')

# golden digest：由当前 FROZEN 实算后填入（2026-10-28，R804 切 f64nli25 后重算）。
# 重算：python3 -B -c "import calibration;print(calibration.digest())"
# 任何值变更都会使 assert_digest() 抛 E_CAL_DIGEST；解封须同步重算并登记。
DIGEST = '4f570e94f9ffad14620a23fd3f6c34f701baeb5aa2da9f3e65205714fbfeee36'


class CalibrationError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


_MISSING = object()


def _resolve(config, path):
    """按键路径取值；任一层缺失或非对象返回 _MISSING。"""
    node = config
    for part in path.split('.'):
        if not isinstance(node, dict) or part not in node:
            return _MISSING
        node = node[part]
    return node


def _type_ok(value, types):
    """bool 不冒充 int/float；类型白名单严格匹配。"""
    if isinstance(value, bool):
        return bool in types
    return isinstance(value, types)


def _raise_pending(path, detail):
    raise CalibrationError('E_CAL_PENDING', '%s %s（fail-closed）' % (path, detail))


def _validate_shape(config):
    """REQUIRED 形状检查：缺键/None/空表/类型不符一律 E_CAL_PENDING。"""
    if not isinstance(config, dict):
        _raise_pending('config', '根节点不是对象')
    for path in sorted(REQUIRED):
        rule = REQUIRED[path]
        value = _resolve(config, path)
        if value is _MISSING:
            _raise_pending(path, '缺失')
        if value is None:
            _raise_pending(path, '为 None')
        if not _type_ok(value, rule['types']):
            _raise_pending(path, '类型 %s 不符（允许 %s）'
                           % (type(value).__name__, rule['types']))
        if isinstance(value, str) and value.strip().upper() in ('TODO', 'TBD'):
            _raise_pending(path, '为待定占位 %r' % value)
        if rule.get('non_empty'):
            if isinstance(value, str) and not value.strip():
                _raise_pending(path, '为空字符串')
            if isinstance(value, (dict, list, tuple)) and len(value) == 0:
                _raise_pending(path, '为空表')
        item_types = rule.get('items')
        if item_types is not None and isinstance(value, list):
            for index, item in enumerate(value):
                if not _type_ok(item, item_types):
                    _raise_pending('%s[%d]' % (path, index),
                                   '元素类型 %s 不符' % type(item).__name__)


def _validate_unfreeze_log(config):
    """解封记录每条都须是对象且 reason 非空。"""
    log = config['unfreeze_log']
    entries = [log] if isinstance(log, dict) else log
    for index, entry in enumerate(entries):
        label = 'unfreeze_log[%d]' % index
        if not isinstance(entry, dict):
            _raise_pending(label, '不是对象')
        reason = entry.get('reason')
        if not isinstance(reason, str) or not reason.strip():
            _raise_pending(label + '.reason', '缺失或为空（解封须记录理由）')


def _validate_scoped_fields(config):
    """带域冻结项：scope 精确匹配、sample_n 为不低于采样下限的整数。

    scope 是口径本身：改 scope 等于改判据语义，必须重走解封与独立复核，
    不允许在值不变时静默把「组件域」扩成「全栈域」。
    """
    for path in sorted(SCOPED_REQUIRED):
        rule = SCOPED_REQUIRED[path]
        node = _resolve(config, path)
        if not isinstance(node, dict):
            _raise_pending(path, '不是带域对象')
        scope = node.get('scope')
        if scope != rule['scope']:
            _raise_pending(path + '.scope',
                           '须为 %r（当前 %r）；带域冻结不许静默扩域'
                           % (rule['scope'], scope))
        sample_n = node.get('sample_n')
        if not _type_ok(sample_n, (int,)) or sample_n < rule['min_sample_n']:
            _raise_pending(path + '.sample_n',
                           '须为 ≥%d 的整数（当前 %r）；样本不足不得带域冻结'
                           % (rule['min_sample_n'], sample_n))


def _pending_report(config):
    """24 §4.1 尚未定标/采样的项：列出 pending 键，不判绿。"""
    pending = []
    for item in PENDING_REQUIRED:
        path = item['key']
        value = _resolve(config, path)
        bad = False
        if value is _MISSING or value is None:
            bad = True
        elif not _type_ok(value, item['types']):
            bad = True
        elif isinstance(value, str) and (not value.strip()
                                         or value.strip().upper() in ('TODO', 'TBD')):
            bad = True
        elif isinstance(value, (dict, list)) and len(value) == 0:
            bad = True
        if bad:
            pending.append({'key': path, 'reason': item['reason']})
    # R748 判据面：成功率是比例门槛，裸 value 不得离开 pending；必须有 scope/
    # sample_n/method/alpha/denominator/aggregation/evidence，且 36-run 下限与
    # Clopper-Pearson 方法不得缺省（fail-closed，不因填了数就判绿）。
    sr_node = _resolve(config, 'success_rate.min_pass_rate')
    if sr_node is not _MISSING and sr_node is not None:
        if not isinstance(sr_node, dict):
            pending.append({
                'key': 'success_rate.min_pass_rate',
                'reason': '须为对象（含 value/scope/sample_n/method/alpha/denominator/'
                          'aggregation/evidence）；裸值不得冻结（R748）'})
        elif sr_node.get('value') is not None:
            value = sr_node.get('value')
            # R752：比例域 (0,1]；负值/0/NaN/Inf 不得靠“填了数”离开 pending。
            if not (_is_number(value) and 0 < value <= 1):
                pending.append({
                    'key': 'success_rate.min_pass_rate.value',
                    'reason': '成功率须在 (0,1]（当前 %r）；域外值不得冻结（R752）'
                              % (value,)})
            text_fields = ('scope', 'denominator', 'aggregation', 'evidence')
            missing = []
            for field in ('scope', 'sample_n', 'method', 'alpha',
                          'denominator', 'aggregation', 'evidence'):
                item = sr_node.get(field)
                if item is None:
                    missing.append(field)
                elif field in text_fields and (not isinstance(item, str)
                                               or not item.strip()):
                    missing.append(field)
                elif field not in text_fields and isinstance(item, str) \
                        and not item.strip():
                    missing.append(field)
            if missing:
                pending.append({
                    'key': 'success_rate.min_pass_rate',
                    'reason': '值已填但缺元数据 %s；无 scope/sample_n/evidence 不得冻结（R748）'
                              % ','.join(missing)})
            else:
                if sr_node.get('scope') != SUCCESS_RATE_SCOPE:
                    pending.append({
                        'key': 'success_rate.min_pass_rate.scope',
                        'reason': 'scope 须精确为 %r（R752，带域冻结不许静默扩域）'
                                  % SUCCESS_RATE_SCOPE})
                if sr_node.get('method') != 'clopper_pearson_lower_95':
                    pending.append({
                        'key': 'success_rate.min_pass_rate.method',
                        'reason': '方法须为 clopper_pearson_lower_95（R748）'})
                if (not isinstance(sr_node.get('sample_n'), int)
                        or isinstance(sr_node.get('sample_n'), bool)
                        or sr_node['sample_n'] < 36):
                    pending.append({
                        'key': 'success_rate.min_pass_rate.sample_n',
                        'reason': '36-run 定标下限：sample_n 须为 ≥36 的整数（R748）'})
                alpha = sr_node.get('alpha')
                if not (_is_number(alpha) and 0 < alpha <= 0.5):
                    pending.append({
                        'key': 'success_rate.min_pass_rate.alpha',
                        'reason': 'alpha 须在 (0,0.5]（R748）'})
    # R759（R751 P3-1）：另两个 pending 键与 success_rate 同族——填了值不算数，
    # 必须有 scope/evidence 且为正数；否则仍 pending（fail-closed）。
    # （定位键已于 R787 按 B 方案带域冻结，移出本循环。）
    for key in ('budgets.stop_time_s', 'budgets.stop_distance_m'):
        node = _resolve(config, key)
        if not isinstance(node, dict) or node.get('value') is None:
            continue
        value = node.get('value')
        if not (_is_number(value) and value > 0):
            pending.append({
                'key': key + '.value',
                'reason': '须为正数（当前 %r）；域外值不得冻结（R759）' % (value,)})
        missing = [field for field in ('scope', 'evidence')
                   if not isinstance(node.get(field), str) or not node.get(field).strip()]
        if missing:
            pending.append({
                'key': key,
                'reason': '值已填但缺元数据 %s；无 scope/evidence 不得冻结（R759）'
                          % ','.join(missing)})
    repetitions = config['sampling']['scenario_repetitions']['value']
    seeds = config['sampling']['perturbation_seeds']['value']
    if repetitions < SAMPLING_MIN_REPETITIONS:
        pending.append({
            'key': 'sampling.scenario_repetitions.value',
            'reason': '24 §4.1 要求每 seed 至少 %d 次重复；当前=%r，未达 4.1 采样要求，禁 freeze'
                      % (SAMPLING_MIN_REPETITIONS, repetitions)})
    if len(seeds) < SAMPLING_MIN_SEEDS:
        pending.append({
            'key': 'sampling.perturbation_seeds.value',
            'reason': '24 §4.1 要求每个适用闭环场景至少 %d 个预登记 seed；当前=%d，未达 4.1 采样要求，禁 freeze'
                      % (SAMPLING_MIN_SEEDS, len(seeds))})
    return pending


def validate_frozen(config=None):
    """无待定关键值＋引用完整性；形状 fail-closed，pending 项报 blocked 不报绿。

    返回 dict：ok/status/freeze_allowed/pending/pending_detail/digest。
    pending 非空时 ok=False、status='blocked'、freeze_allowed=False。
    """
    config = FROZEN if config is None else config
    _validate_shape(config)
    _validate_unfreeze_log(config)
    _validate_scoped_fields(config)

    official = set(config['denominators']['official_scenarios']['value'])
    calibration = set(config['denominators']['calibration_scenarios']['value'])
    known = {k for k, v in scn.SCENARIOS.items() if not v.get('slot')}
    unknown = (official | calibration) - known
    if unknown:
        raise CalibrationError('E_CAL_UNKNOWN_SCENARIO', ','.join(sorted(unknown)))
    if not official or not calibration:
        raise CalibrationError('E_CAL_EMPTY_SET', '分母/定标集不得为空')

    pending = _pending_report(config)
    return {
        'ok': not pending,
        'status': 'blocked' if pending else 'pass',
        'blocked': bool(pending),
        'freeze_allowed': not pending,
        'pending': [item['key'] for item in pending],
        'pending_detail': pending,
        'calibration_version': config['calibration_version'],
        'digest': digest(config),
    }


def separation_guard(run_records):
    """定标与正式分开：run 记录须带 track 字段且定标 run 不入正式分母。"""
    official = [r for r in run_records if r.get('run_class') == 'official']
    leaked = [r for r in official if r.get('calibration_only')]
    if leaked:
        raise CalibrationError('E_CAL_LEAK', '定标 run 混入正式分母：%s' % leaked[:1])
    return {'official_count': len(official),
            'calibration_count': len(run_records) - len(official)}


def digest(config=None):
    import hashlib
    config = FROZEN if config is None else config
    canonical = json.dumps(config, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(canonical.encode()).hexdigest()


def assert_digest(config=None):
    """golden digest：值一变（含空 dict 兜底）即 E_CAL_DIGEST。"""
    actual = digest(config)
    if actual != DIGEST:
        raise CalibrationError('E_CAL_DIGEST',
                               '冻结配置已变更：期望 %s 实得 %s（须显式解封并登记 unfreeze_log）'
                               % (DIGEST, actual))
    return actual


def _iter_value_leaves(node, prefix=''):
    """展开 dict 叶子；unfreeze_log 是元数据，不参与放宽比较。"""
    if isinstance(node, dict):
        for key, value in node.items():
            path = key if not prefix else prefix + '.' + key
            if path == 'unfreeze_log' or path.startswith('unfreeze_log.'):
                continue
            if isinstance(value, dict):
                yield from _iter_value_leaves(value, path)
            else:
                yield path, value


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _loosening_reason(path, old, new):
    """返回放宽理由；None 表示未放宽或变化方向不属于放宽。"""
    if path.startswith('tolerances.') and path.endswith('.value'):
        if not (_is_number(old) and _is_number(new)):
            return None
        if path == 'tolerances.min_clearance_m.value':
            return '最小净空下调' if new < old else None
        return '容差上调' if new > old else None
    if path.startswith('budgets.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new < old:
            return '预算/上限下调'
        return None
    if path.startswith('latency.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new > old:
            return '时延上限上调'
        return None
    if path.startswith('resources.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new > old:
            return '资源上限上调'
        return None
    if path == 'sampling.scenario_repetitions.value':
        if _is_number(old) and _is_number(new) and new < old:
            return '重复次数下调'
        return None
    if path == 'sampling.perturbation_seeds.value':
        if isinstance(old, list) and isinstance(new, list):
            return '预登记 seed 减少' if not set(new) >= set(old) else None
        return None
    if path.startswith('denominators.') and path.endswith('.value'):
        if isinstance(old, list) and isinstance(new, list):
            return '分母/矩阵条目减少' if not set(new) >= set(old) else None
        return None
    if path == 'rerun_rules.max_reruns.value':
        if _is_number(old) and _is_number(new) and new > old:
            return '重跑上限上调'
        return None
    if path == 'rerun_rules.calibration_runs_count_official.value':
        return '定标 run 计入正式分母' if old is False and new is True else None
    # R748 判据面：成功率/覆盖率类门槛只能上调（更严）；下调一律按放宽处理。
    if path.startswith('success_rate.') and path.endswith('.value'):
        if not (_is_number(old) and _is_number(new)):
            return None
        return '成功率/覆盖率门槛下调' if new < old else None
    return None


def _unfreeze_entries(config):
    log = config.get('unfreeze_log') if isinstance(config, dict) else None
    if isinstance(log, dict):
        return [log]
    if isinstance(log, list):
        return [entry for entry in log if isinstance(entry, dict)]
    return []


def _authorizes(entry, path, old, new):
    """该条 unfreeze 是否覆盖此放宽：from/to 均在、reason 非空；带 key 时须精确匹配。"""
    reason = entry.get('reason')
    if not isinstance(reason, str) or not reason.strip():
        return False
    if 'from' not in entry or 'to' not in entry:
        return False
    if entry.get('from') is None or entry.get('to') is None:
        return False
    if isinstance(entry.get('from'), str) and not entry['from'].strip():
        return False
    if isinstance(entry.get('to'), str) and not entry['to'].strip():
        return False
    scoped = entry.get('keys', entry.get('key', entry.get('path')))
    if scoped is None:
        return True
    if isinstance(scoped, str):
        scoped = [scoped]
    if not isinstance(scoped, (list, tuple, set)):
        return False
    if path not in scoped:
        return False
    return entry.get('from') == old and entry.get('to') == new


def _success_rate_metadata_changes(previous, current):
    """R755（R751 P2-1）：成功率节点级口径变更也算放宽。

    带域冻结的 scope 是判据语义本体；scope/分母/聚合口径变更或样本数下调
    都必须重走 unfreeze，不能只比较 .value 叶子。
    """
    prev = _resolve(previous, 'success_rate.min_pass_rate') if isinstance(previous, dict) else _MISSING
    cur = _resolve(current, 'success_rate.min_pass_rate') if isinstance(current, dict) else _MISSING
    if not (isinstance(prev, dict) and isinstance(cur, dict)):
        return []
    path = 'success_rate.min_pass_rate'
    changes = []
    for field, reason in (('scope', '成功率 scope 变更/扩域'),
                          ('sample_n', '成功率样本数下调'),
                          ('denominator', '成功率分母口径变更'),
                          ('aggregation', '成功率聚合口径变更')):
        old, new = prev.get(field), cur.get(field)
        if old is None and new is None:
            continue
        if field == 'sample_n':
            if _is_number(old) and _is_number(new) and new < old:
                changes.append((path + '.' + field, old, new, reason))
        elif old != new:
            changes.append((path + '.' + field, old, new, reason))
    return changes


BASELINE_FILE = HERE / 'frozen_baseline.json'


def freeze_guard(config, baseline_path=None):
    """R758（R751 P2-2）：freeze 前对上一版冻结基线跑 loosening。

    无基线＝首次冻结（允许）；基线不可读或结构不对＝E_CAL_BASELINE；
    基线存在时任何未授权放宽＝E_CAL_LOOSENED。纯读，不写文件。
    """
    path = Path(baseline_path) if baseline_path is not None else BASELINE_FILE
    if not path.exists():
        return []
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise CalibrationError('E_CAL_BASELINE', '基线不可读：%s' % exc)
    previous = data.get('frozen') if isinstance(data, dict) else None
    if not isinstance(previous, dict):
        raise CalibrationError('E_CAL_BASELINE', '基线缺少 frozen 对象')
    return check_loosening(previous, config)


def check_loosening(previous, current):
    """纯函数：相对 previous，current 的放宽项须有 from/to 且 reason 非空的 unfreeze。

    放宽口径：容差上调（min_clearance 为下调）、预算/上限下调、时延上限上调、
    资源上限上调、分母/矩阵条目减少、采样重复与 seed 减少、重跑上限上调、
    定标 run 改为计入正式分母；删除受控键亦算。
    返回被放宽的键路径列表；无放宽返回 []；未授权抛 E_CAL_LOOSENED。
    """
    previous_values = dict(_iter_value_leaves(previous))
    current_values = dict(_iter_value_leaves(current))
    changes = []
    for path, old in previous_values.items():
        if not path.endswith('.value'):
            continue
        if path not in current_values:
            changes.append((path, old, '<missing>', '键被删除'))
            continue
        new = current_values[path]
        reason = _loosening_reason(path, old, new)
        if reason:
            changes.append((path, old, new, reason))
    changes.extend(_success_rate_metadata_changes(previous, current))
    if not changes:
        return []
    entries = _unfreeze_entries(current)
    unauthorized = []
    for path, old, new, reason in changes:
        if not any(_authorizes(entry, path, old, new) for entry in entries):
            unauthorized.append('%s: %r→%r（%s）' % (path, old, new, reason))
    if unauthorized:
        raise CalibrationError('E_CAL_LOOSENED',
                               '放宽缺 unfreeze_log 的 from/to/reason：%s'
                               % '; '.join(unauthorized))
    return [item[0] for item in changes]


def main(argv):
    if argv and argv[0] == 'freeze':
        try:
            report = validate_frozen()
            current_digest = assert_digest()
            freeze_guard(FROZEN)
        except CalibrationError as exc:
            print(json.dumps({'ok': False, 'code': exc.code, 'detail': exc.detail},
                             ensure_ascii=False))
            return 2
        # ok 仅表示形状/契约校验无硬错（兼容既有 CLI 断言）；pending 未清时
        # freeze_allowed=False、status=blocked，不得当作可封版。
        if report['freeze_allowed']:
            BASELINE_FILE.write_text(json.dumps(
                {'frozen': FROZEN, 'digest': current_digest,
                 'calibration_version': FROZEN['calibration_version']},
                ensure_ascii=False, sort_keys=True))
        print(json.dumps({'ok': True,
                          'ok_scope': 'shape_and_contract_only',
                          'status': report['status'],
                          'blocked': report['blocked'],
                          'freeze_allowed': report['freeze_allowed'],
                          'pending': report['pending'],
                          'calibration_version': FROZEN['calibration_version'],
                          'digest': current_digest,
                          'baseline': str(BASELINE_FILE)}, ensure_ascii=False))
        return 0
    print('usage: calibration.py freeze', file=sys.stderr)
    return 9


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
