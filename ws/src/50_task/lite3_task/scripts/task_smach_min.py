#!/usr/bin/env python3
# lite3_task 一期最小顺序状态机（C12，N4）：验证 task_primitives.yaml 契约
import rospy, yaml
import smach

class PrimitiveState(smach.State):
    """读 task_primitives.yaml 原语契约；一期返回骨架 outcome，P6 换真实 action client（经 C11 disposition 映射）。"""
    def __init__(self, name, registry, outcomes):
        self.name, self.spec = name, registry['primitives'][name]
        smach.State.__init__(self, outcomes=outcomes)
    def execute(self, ud):
        rospy.loginfo('[M5] 原语 %s（target=%s）', self.name, self.spec['target'])
        return 'succeeded'

def main():
    rospy.init_node('lite3_task_smach_min')
    with open(rospy.get_param('~task_primitives')) as f:
        registry = yaml.safe_load(f)
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
        # transitions 键 = C11 处置分级：retry_local→retry / replan→replan / clear_and_replan→clear / abort_report→aborted
        smach.StateMachine.add('WAIT', PrimitiveState('wait', registry, ['succeeded', 'aborted']),
                               transitions={'succeeded': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('GET_PATH', PrimitiveState('get_path', registry, ['succeeded', 'retry', 'aborted']),
                               transitions={'succeeded': 'EXE_PATH', 'retry': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('EXE_PATH', PrimitiveState('exe_path', registry,
                                     ['succeeded', 'retry', 'replan', 'clear', 'aborted']),
                               transitions={'succeeded': 'succeeded', 'retry': 'EXE_PATH',
                                            'replan': 'GET_PATH', 'clear': 'GET_PATH', 'aborted': 'aborted'})
    rospy.loginfo('task outcome: %s', sm.execute())

if __name__ == '__main__':
    main()
