#!/usr/bin/env python3
# lite3_task 一期最小顺序状态机（C12，N4）：验证 task_primitives.yaml 契约
# ── W1 实链版（2026-10-07 小明，测试矩阵 Wave1-M5；改前留底 证据/raw/task_smach_min.py.before_w1.bak=49393bfd…，逐处登记 04）──
# P3a 骨架=纯 log 桩，零改动首跑已收口（task outcome=succeeded+原语三连，run0）。本版按契约接真实 mbf action：
#   · task_primitives.yaml：原语 target/goal_fields/by_name_field（get_path/exe_path/recover/navigate_to_pose/wait）
#   · recovery_policy.yaml：outcome_tables（数值→常量名）+disposition（→处置分级键）+recovery_actions（max_retries 预算）
#   · 处置分级→SMACH transitions：retry_local→retry / replan→replan / clear_and_replan→clear / abort_report→aborted
#     （骨架头注原口径）；clear_and_replan 按 C11 复合定义走 RECOVER 原语（behavior 按 recover.by_name_field=behavior
#     点单，缺省 clear_costmaps=C0 注册表名）→ then:[get_path, exe_path] = RECOVER→GET_PATH→EXE_PATH。
#   · 超时看门狗：action 到点未归 → cancel → 按 CANCELED 常量名走 disposition（不悬挂）。
import rospy, yaml
import smach
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32   # M5.2 scan_navigate 用 J1 到达实体（lite3_task 的 rosdep 声明面本件不改，见 04 注记）
from mbf_msgs.msg import (GetPathAction, GetPathGoal, ExePathAction, ExePathGoal,
                          RecoveryAction, RecoveryGoal, MoveBaseAction, MoveBaseGoal)

DISP2OUT = {'retry_local': 'retry', 'replan': 'replan', 'clear_and_replan': 'clear', 'abort_report': 'aborted'}


class PrimitiveState(smach.State):
    """task_primitives.yaml 原语→真实 mbf action 客户端；outcome 数值经 recovery_policy.yaml 两级查表得 SMACH 转移。"""
    def __init__(self, name, reg, policy, ud_keys, outcomes, table_key=None, out_keys=(), in_keys=()):
        self.name = name
        self.spec = reg['primitives'][name]
        tk = table_key or name
        # wait 原语（local:timer）无 outcome_tables/disposition 条目——容忍缺键置空（WaitState 不消费）
        self.tables = policy['outcome_tables'].get(tk, {'SUCCESS': 0, 'CANCELED': None})
        self.disp = policy['disposition'].get(tk, {})
        self.budget_key = {'retry': 'retry_local', 'replan': 'replan', 'clear': 'clear_and_replan'}
        self.actions = policy['recovery_actions']
        self.ud_keys = {'retry_local': 'retry_n', 'replan': 'replan_n', 'clear_and_replan': 'clear_n'}  # 预算→userdata 计数名
        # 预算计数与 path 须声明为 output_keys——smach 的 ud 写入经 output_keys 过滤器，未声明即静默丢弃（修法#3 实证）
        smach.State.__init__(self, outcomes=outcomes, output_keys=list(ud_keys) + list(out_keys),
                             input_keys=list(ud_keys) + list(in_keys))   # 读侧同样过 input_keys 过滤器
        # target 串契约自检（接口错位即拒起——P3a0 log 实测类型权威；local: 原语放行）
        assert self.spec['target'].startswith(('action:mbf_msgs/', 'local:')), '%s target 契约变改：%s' % (name, self.spec['target'])

    def _route(self, ud, outcome_num):
        """outcome 数值→常量名→处置键→（预算内？转移 : aborted）。"""
        name = next((k for k, v in self.tables.items() if v == outcome_num), 'UNKNOWN_%d' % outcome_num)
        if name == 'SUCCESS':   # C11：SUCCESS=成功路径不列 disposition，不列即不走恢复——短路直返
            return 'succeeded'
        disp = self.disp.get(name, self.disp.get('default', 'abort_report'))
        out = DISP2OUT[disp]
        if out in self.budget_key:
            key = self.budget_key[out]
            used = ud[self.ud_keys[key]] + 1
            if used > int(self.actions[key].get('max_retries', 0)):
                rospy.loginfo('[M5] 原语 %s outcome=%d(%s)→%s 预算耗尽（%d>%d）→ aborted',
                              self.name, outcome_num, name, disp, used - 1, int(self.actions[key].get('max_retries', 0)))
                return 'aborted'
            ud[self.ud_keys[key]] = used
        return out

    def _goal_pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = 'map', rospy.Time(0)
        p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
        return p


class WaitState(PrimitiveState):
    """wait 原语（target=local:timer）：本地定时器，缺省 duration_s=1.0（契约口径）。"""
    def __init__(self, reg, policy):
        PrimitiveState.__init__(self, 'wait', reg, policy, [], ['succeeded', 'aborted'])

    def execute(self, ud):
        t0 = rospy.Time.now()
        d = float(self.spec.get('params', {}).get('duration_s', 1.0)) if isinstance(self.spec.get('params', {}), dict) else 1.0
        rospy.sleep(d)
        rospy.loginfo('[M5] 原语 wait（target=local:timer）outcome=succeeded 耗时 %.2fs', (rospy.Time.now() - t0).to_sec())
        return 'succeeded'


class GetPathState(PrimitiveState):
    def __init__(self, reg, policy):
        PrimitiveState.__init__(self, 'get_path', reg, policy, ['replan_n', 'clear_n', 'retry_n'],
                                ['succeeded', 'retry', 'replan', 'clear', 'aborted'], out_keys=['path'])

    def execute(self, ud):
        ac = actionlib.SimpleActionClient('/%s/get_path' % rospy.get_param('~mbf_ns', 'move_base_flex'), GetPathAction)
        if not ac.wait_for_server(rospy.Duration(30)):
            rospy.loginfo('[M5] 原语 get_path：action server 不在位'); return 'aborted'
        g = GetPathGoal()
        g.use_start_pose = False
        g.target_pose = self._goal_pose(rospy.get_param('~goal_x'), rospy.get_param('~goal_y'))
        g.tolerance = float(rospy.get_param('~path_tolerance', 0.35))
        g.planner = rospy.get_param('~planner', '')   # 按名点单（task_primitives get_path.by_name_field=planner）；空=mbf 缺省
        t0 = rospy.Time.now(); ac.send_goal(g)
        ok = ac.wait_for_result(rospy.Duration(float(rospy.get_param('~getpath_timeout', 20))))
        if ok:
            res_out, note, res = ac.get_result().outcome, ac.get_result().message, ac.get_result()
        else:
            ac.cancel_goal(); res_out, note, res = self.tables['CANCELED'], '看门狗超时 cancel', None
        dt = (rospy.Time.now() - t0).to_sec()
        out = self._route(ud, res_out)
        rospy.loginfo('[M5] 原语 get_path outcome=%d %s → %s 耗时 %.2fs', res_out, note, out, dt)
        if out == 'succeeded':
            ud.path = res.path
        return out


class ExePathState(PrimitiveState):
    def __init__(self, reg, policy):
        PrimitiveState.__init__(self, 'exe_path', reg, policy, ['replan_n', 'clear_n', 'retry_n'],
                                ['succeeded', 'retry', 'replan', 'clear', 'aborted'], in_keys=['path'])

    def execute(self, ud):
        if 'path' not in ud or not ud.path.poses:
            rospy.loginfo('[M5] 原语 exe_path：无可用路径'); return 'aborted'
        ac = actionlib.SimpleActionClient('/%s/exe_path' % rospy.get_param('~mbf_ns', 'move_base_flex'), ExePathAction)
        if not ac.wait_for_server(rospy.Duration(15)):
            rospy.loginfo('[M5] 原语 exe_path：action server 不在位'); return 'aborted'
        g = ExePathGoal()
        g.path = ud.path
        g.controller = rospy.get_param('~controller', '')   # 按名点单（by_name_field=controller）；空=mbf 缺省
        # 修法#6：mbf 自带到达判定（ExePathGoal.tolerance_from_action=true→mbf 逐周期自查 dist/angle 容差，
        # 与 teb isGoalReached 解耦——T1 on-rails 位姿过格心时的到达判定路径）；false=teb 自带 0.2/0.2
        g.tolerance_from_action = bool(rospy.get_param('~exepath_tolerance_from_action', False))
        g.dist_tolerance = float(rospy.get_param('~exepath_dist_tolerance', 0.4))
        g.angle_tolerance = float(rospy.get_param('~exepath_angle_tolerance', 1.3))
        t0 = rospy.Time.now(); ac.send_goal(g)
        ok = ac.wait_for_result(rospy.Duration(float(rospy.get_param('~exepath_timeout', 90))))
        if not ok:
            ac.cancel_goal(); res_out, note = self.tables['CANCELED'], '看门狗超时 cancel'
        else:
            res_out, note = ac.get_result().outcome, ac.get_result().message
        dt = (rospy.Time.now() - t0).to_sec()
        out = self._route(ud, res_out)
        rospy.loginfo('[M5] 原语 exe_path outcome=%d %s → %s 耗时 %.2fs', res_out, note, out, dt)
        return out


class RecoverState(PrimitiveState):
    def __init__(self, reg, policy):
        PrimitiveState.__init__(self, 'recover', reg, policy, [], ['succeeded', 'aborted'])

    def execute(self, ud):
        ac = actionlib.SimpleActionClient('/%s/recovery' % rospy.get_param('~mbf_ns', 'move_base_flex'), RecoveryAction)
        if not ac.wait_for_server(rospy.Duration(15)):
            rospy.loginfo('[M5] 原语 recover：action server 不在位'); return 'aborted'
        g = RecoveryGoal()
        g.behavior = rospy.get_param('~recover_behavior', 'clear_costmaps')   # 点单值=C0 recovery_behaviors 注册表 name
        t0 = rospy.Time.now(); ac.send_goal(g)
        ok = ac.wait_for_result(rospy.Duration(float(rospy.get_param('~recovery_timeout', 30))))
        if not ok:
            ac.cancel_goal(); res_out, note = self.tables['CANCELED'], '看门狗超时 cancel'
        else:
            res_out, note = ac.get_result().outcome, ac.get_result().message
        dt = (rospy.Time.now() - t0).to_sec()
        out = 'succeeded' if res_out == self.tables['SUCCESS'] else 'aborted'
        rospy.loginfo('[M5] 原语 recover behavior=%s outcome=%d %s → %s 耗时 %.2fs', g.behavior, res_out, note, out, dt)
        return out


class NavigateState(PrimitiveState):
    """navigate_to_pose 原语（组合形态=mbf move_base action）：C11 move_base 表处置；goal.recovery_behaviors=[]=
       mbf 内不跑恢复、失败 outcome 直返任务层（recovery_enabled=false 设计口径：恢复编排收口 M5）。
       ★ 修法#4（mbf 实测对齐）：mbf 组合 action 透传【阶段 outcome】（105=exe_path OSCILLATION、55=get_path
       PAT_EXCEEDED，drill 实测），非 move_base 表码（13 等）——move_base 表缺键时按 exe_path/get_path 表回退分流。"""
    def __init__(self, reg, policy):
        PrimitiveState.__init__(self, 'navigate_to_pose', reg, policy, ['replan_n', 'clear_n', 'retry_n'],
                                ['succeeded', 'retry', 'replan', 'clear', 'aborted'], table_key='move_base')
        self.stage_tables = [(policy['outcome_tables'][k], policy['disposition'][k]) for k in ('exe_path', 'get_path')]
        self._movebase_tables = (self.tables, self.disp)

    def _route(self, ud, outcome_num):
        if not any(v == outcome_num for v in self.tables.values()):
            for t, d in self.stage_tables:   # 阶段 outcome 回退（C11 整链语义=阶段表处置）
                if any(v == outcome_num for v in t.values()):
                    self.tables, self.disp = t, d
                    out = PrimitiveState._route(self, ud, outcome_num)
                    self.tables, self.disp = self._movebase_tables
                    return out
        return PrimitiveState._route(self, ud, outcome_num)

    def execute(self, ud):
        ac = actionlib.SimpleActionClient('/%s/move_base' % rospy.get_param('~mbf_ns', 'move_base_flex'), MoveBaseAction)
        if not ac.wait_for_server(rospy.Duration(30)):
            rospy.loginfo('[M5] 原语 navigate_to_pose：action server 不在位'); return 'aborted'
        g = MoveBaseGoal()
        g.target_pose = self._goal_pose(rospy.get_param('~goal_x'), rospy.get_param('~goal_y'))
        g.planner = rospy.get_param('~planner', '')
        g.controller = rospy.get_param('~controller', '')
        g.recovery_behaviors = []   # 恢复编排收口 M5：mbf 内零恢复，OSCILLATION 等 outcome 原样上抛 C11 分级
        t0 = rospy.Time.now(); ac.send_goal(g)
        ok = ac.wait_for_result(rospy.Duration(float(rospy.get_param('~navigate_timeout', 60))))
        if not ok:
            ac.cancel_goal(); res_out, note = self.tables['CANCELED'], '看门狗超时 cancel'
        else:
            res_out, note = ac.get_result().outcome, ac.get_result().message
        dt = (rospy.Time.now() - t0).to_sec()
        out = self._route(ud, res_out)
        rospy.loginfo('[M5] 原语 navigate_to_pose outcome=%d %s → %s 耗时 %.2fs', res_out, note, out, dt)
        return out



class ScanNavigateState(smach.State):
    """scan_navigate（M5.2）：发布 /scan_planner/goal，等 J1 到达实体置位——任务层的最小闭环原语。

    三条契约口径都不自造：
      · target=local:scan_goal，属注册表既有前缀空间，与 wait 的 local:timer 同族；
      · 二态 succeeded/aborted：失败侧**只有超时档**——盘上 SCAN 的 FSM 没有失败事件
        （scan_replan_fsm.cpp:670→:691 段完成只看轨迹时间耗尽，静默回 WAIT_TARGET），
        J1 观测件设计又明写「不造失败信号」⇒ 任务层不替上游造事件，只把「到点未达」报 aborted；
      · 到达与偏差都取 J1 实体的读数，参考点是我方请求的 goal（不取 bspline 末点＝设计件事实 A）。
    活体口径沿用 A7 P3 的两条实测教训：publisher 建好即发的第一波会被丢 ⇒ 先 sleep 再连发三次。
    """
    def __init__(self, reg, policy):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.spec = reg['primitives']['scan_navigate']
        assert self.spec['target'] == 'local:scan_goal', \
            'scan_navigate target 契约变改：%s' % self.spec['target']

    def execute(self, ud):
        gx, gy = float(rospy.get_param('~goal_x')), float(rospy.get_param('~goal_y'))
        timeout_s = float(rospy.get_param('~scan_timeout_s', 120.0))     # A7 判据④ 同值，任务层不放宽
        tol = float(rospy.get_param('~scan_tol_m', 0.35))                # A7 判据③ 同值
        frame = rospy.get_param('~scan_goal_frame', 'world')             # P2/P3 实测：goal 帧取占据图 frame_id=world
        box = {'arrived': False, 'dist': None}
        s1 = rospy.Subscriber('/lite3/scan/arrived', Bool, lambda m: box.__setitem__('arrived', bool(m.data)))
        s2 = rospy.Subscriber('/lite3/scan/arrival_dist', Float32, lambda m: box.__setitem__('dist', float(m.data)))
        pub = rospy.Publisher('/scan_planner/goal', PoseStamped, queue_size=1)
        g = PoseStamped()
        g.header.frame_id = frame
        g.pose.position.x, g.pose.position.y = gx, gy
        g.pose.orientation.w = 1.0
        rospy.sleep(1.5)                          # 连接建立前的第一波必丢（A7 P3 活体教训）
        for _ in range(3):
            g.header.stamp = rospy.Time.now()
            pub.publish(g)
            rospy.sleep(0.4)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < timeout_s:
            if box['arrived'] and box['dist'] is not None and box['dist'] <= tol:
                dt = (rospy.Time.now() - t0).to_sec()
                rospy.loginfo('[M5] 原语 scan_navigate outcome=succeeded 到达偏差 %.3fm≤%.2fm 耗时 %.2fs',
                              box['dist'], tol, dt)
                s1.unregister(); s2.unregister(); pub.unregister()
                return 'succeeded'
            rospy.sleep(0.2)
        dt = (rospy.Time.now() - t0).to_sec()
        rospy.loginfo('[M5] 原语 scan_navigate outcome=aborted 超时 %.1fs 未到（arrived=%s dist=%s）',
                      timeout_s, box['arrived'], box['dist'])
        s1.unregister(); s2.unregister(); pub.unregister()
        return 'aborted'


def main():
    rospy.init_node('lite3_task_smach_min')
    with open(rospy.get_param('~task_primitives')) as f:
        reg = yaml.safe_load(f)
    with open(rospy.get_param('~recovery_policy',
                             '/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/recovery_policy.yaml')) as f:
        policy = yaml.safe_load(f)
    mode = rospy.get_param('~mode', 'chain')   # chain=GET_PATH→EXE_PATH（骨架拓扑实链）| navigate=navigate_to_pose 组合 | scan=scan_navigate（M5.2，走 J1 到达实体）

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    sm.userdata.replan_n, sm.userdata.clear_n, sm.userdata.retry_n = 0, 0, 0
    with sm:
        smach.StateMachine.add('WAIT', WaitState(reg, policy),
                               transitions={'succeeded': ('SCAN_NAVIGATE' if mode == 'scan'
                                                                else 'GET_PATH' if mode == 'chain' else 'NAVIGATE'),
                                                'aborted': 'aborted'})
        smach.StateMachine.add('GET_PATH', GetPathState(reg, policy),
                               transitions={'succeeded': 'EXE_PATH', 'retry': 'GET_PATH', 'replan': 'GET_PATH',
                                            'clear': 'RECOVER', 'aborted': 'aborted'})
        smach.StateMachine.add('EXE_PATH', ExePathState(reg, policy),
                               transitions={'succeeded': 'succeeded', 'retry': 'EXE_PATH', 'replan': 'GET_PATH',
                                            'clear': 'RECOVER', 'aborted': 'aborted'})   # RECOVER 在 scan 模式下不可达（scan 无 mbf 阶段 outcome，不自造恢复编排）
        smach.StateMachine.add('NAVIGATE', NavigateState(reg, policy),
                               transitions={'succeeded': 'succeeded', 'retry': 'NAVIGATE', 'replan': 'NAVIGATE',
                                            'clear': 'RECOVER', 'aborted': 'aborted'})
        smach.StateMachine.add('SCAN_NAVIGATE', ScanNavigateState(reg, policy),
                               transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
        # C11 clear_and_replan 复合定义：clear_costmaps + then:[get_path, exe_path]——RECOVER 成功即回 GET_PATH
        smach.StateMachine.add('RECOVER', RecoverState(reg, policy),
                               transitions={'succeeded': 'GET_PATH' if mode == 'chain' else 'NAVIGATE',
                                            'aborted': 'aborted'})
    t0 = rospy.Time.now()
    outcome = sm.execute()
    rospy.loginfo('task outcome: %s（总时长 %.1fs）', outcome, (rospy.Time.now() - t0).to_sec())


if __name__ == '__main__':
    main()
