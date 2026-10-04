#!/usr/bin/env python3
# TaskTransfer.py —— pipeline_tracking 任务点间转移（C5 改写版，v4 §8 C5/N3；非改动逻辑保持原样，改动处用【C5】标注）
# 【C5】客户端 pipeline.msg 自研 MoveBase msg → mbf_msgs/MoveBase（mbf 联邦式主路线，v4 N1）
# 【C5】重试语义：原"盲目 while 无限重发" → 读 recovery_policy.yaml（C11）按错误码分级重试：
#        retry_local 级按 max_retries 限次；replan / clear_and_replan 级升级处置（逐级递进、各级限次）；
#        abort_report 级立即停止并上报。任务级失败语义与 mbf action outcome 的对接在 P6 mode_nav_stub 联调中验证（v4 C5）。
import rospy
import actionlib
import yaml
from tf.transformations import *
import mbf_msgs.msg  # 【C5】原：from pipeline.msg import MoveBaseAction, MoveBaseGoal


class MoveBaseOutcome(object):
    """【C5】mbf_msgs/MoveBaseResult outcome 常量。
    权威：P3a.0 实测 log（outcome_tables.move_base，同 recovery_policy.yaml），禁止凭记忆增删。"""
    SUCCESS = 0
    FAILURE = 10
    CANCELED = 11
    COLLISION = 12
    OSCILLATION = 13
    START_BLOCKED = 14
    GOAL_BLOCKED = 15
    TF_ERROR = 16
    INTERNAL_ERROR = 17
    PLAN_FAILURE = 50
    CTRL_FAILURE = 100

    @classmethod
    def name_of(cls, outcome):
        for k, v in vars(cls).items():
            if k.isupper() and v == outcome:
                return k
        return 'UNKNOWN'


# 【C5】C11 处置分级递进顺序（低 → 高）
TIER_ORDER = ['retry_local', 'replan', 'clear_and_replan', 'abort_report']


class TaskTransfer:
    def __init__(self):
        # 【C5】action 话题名随 mbf 服务器节点名（mbf_navigation.launch: move_base_flex）→ 精确名 Q11 联调确认后固化（v4 C12 同注记）
        action_name = rospy.get_param('~move_base_action', '/move_base_flex/move_base')
        self.moveBaseClient = actionlib.SimpleActionClient(action_name, mbf_msgs.MoveBaseAction)
        self.moveBaseClient.wait_for_server()
        rospy.loginfo("Action '%s' is up!", action_name)
        # 【C5】读 C11 recovery_policy.yaml（路径经 rosparam 注入，默认 = P3a 落位）
        policy_path = rospy.get_param(
            '~recovery_policy',
            '/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/recovery_policy.yaml')
        with open(policy_path) as f:
            policy = yaml.safe_load(f)
        self.disposition = policy['disposition']['move_base']   # move_base 整链语义（C11）
        self.recovery_actions = policy['recovery_actions']
        self.tries_used = {}

    def _disposition_of(self, outcome):
        # 【C5】错误码 → 处置分级（C11 disposition.move_base；default 兜底）
        return self.disposition.get(MoveBaseOutcome.name_of(outcome),
                                    self.disposition.get('default', 'abort_report'))

    def _next_tier(self, tier):
        # 【C5】本级限次用尽 → 递进下一处置级（C11 分级顺序）
        return TIER_ORDER[min(TIER_ORDER.index(tier) + 1, len(TIER_ORDER) - 1)]

    def IsPlanFailed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.ABORTED

    def IsActionSucceeded(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED

    def TaskTransfer(self, src_point, des_point):
        des_point.SetPreTaskPoint(src_point)
        goal_msg = mbf_msgs.MoveBaseGoal()  # 【C5】原：MoveBaseGoal()（pipeline.msg）
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = des_point.GetPoseX()
        goal_msg.target_pose.pose.position.y = des_point.GetPoseY()
        goal_msg.target_pose.pose.position.z = 0
        my_q = quaternion_from_euler(0, 0, des_point.GetYaw())
        goal_msg.target_pose.pose.orientation.x = my_q[0]
        goal_msg.target_pose.pose.orientation.y = my_q[1]
        goal_msg.target_pose.pose.orientation.z = my_q[2]
        goal_msg.target_pose.pose.orientation.w = my_q[3]
        # 【C5】按名点单（§2.6 决策 5）：点单值 = C0 注册表 name（mbf_navigation.yaml planners/controllers 第一项）
        goal_msg.planner = rospy.get_param('~planner_name', 'GlobalPlanner')
        goal_msg.controller = rospy.get_param('~controller_name', 'TebLocalPlannerROS')

        # 【C5】分级重试主循环（替换原盲目 while；5s 轮询等待语义与原实现一致，仅失败后的处置改为分级）
        self.tries_used = {}
        while not rospy.is_shutdown():
            self.moveBaseClient.send_goal(goal_msg)
            rospy.logwarn(
                "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
            )
            done = self.moveBaseClient.wait_for_result(timeout=rospy.Duration(5.0))
            if rospy.is_shutdown():
                break
            if not done:
                continue  # 结果未决：维持原 5s 轮询等待，不重发目标、不计重试
            if self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return True
            result = self.moveBaseClient.get_result()
            outcome = result.outcome if result is not None else MoveBaseOutcome.FAILURE
            tier = self._disposition_of(outcome)
            rospy.logwarn("Transfer [%s]->[%s] outcome=%s(%d) -> 处置 %s",
                          src_point.name, des_point.name,
                          MoveBaseOutcome.name_of(outcome), outcome, tier)
            if tier == 'abort_report':
                rospy.logfatal("Transfer [%s]->[%s] 处置 abort_report —— 立即停止并上报（v4 C5）",
                               src_point.name, des_point.name)
                return False
            act = self.recovery_actions.get(tier, {})
            if self.tries_used.get(tier, 0) >= act.get('max_retries', 0):
                tier = self._next_tier(tier)
                if tier == 'abort_report':
                    rospy.logfatal("Transfer [%s]->[%s] 各级限次用尽 -> abort_report —— 立即停止并上报（v4 C5）",
                                   src_point.name, des_point.name)
                    return False
            self.tries_used[tier] = self.tries_used.get(tier, 0) + 1
            rospy.sleep(self.recovery_actions.get(tier, {}).get('backoff_s', 1.0))  # C11 退避
        return False
