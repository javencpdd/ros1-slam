"""V8-09 运行面包装：把零 ROS 的 support_planner 暴露为 ROS1 节点。

设计层（support_planner.plan）不 import ROS；本文件只做参数/消息适配，
核心转换放在纯函数里（to_request / to_path_points / refusal），便于离线测试。
"""
import json
import math
import time

import support_planner


def to_request(map_path, start, goal, budget, hint):
    """把节点参数折算成 plan() 的入参；缺项一律显式报错，不取缺省。"""
    with open(map_path, encoding='utf-8') as stream:
        value = json.load(stream)
    if not isinstance(start, (list, tuple)) or len(start) != 3:
        raise ValueError('start must be [x, y, z]')
    if not isinstance(goal, (list, tuple)) or len(goal) != 3:
        raise ValueError('goal must be [x, y, z]')
    if goal[2] is None:
        raise ValueError('support map goal requires z')
    if budget <= 0:
        raise ValueError('budget must be positive')
    request = {'map': value, 'start': tuple(start), 'goal': tuple(goal), 'budget': int(budget)}
    if hint is not None:
        request['goal'] = (goal[0], goal[1], goal[2])
        request['hint'] = hint
    return request


def plan_route(request, cancel=None):
    """调用设计层；hint 只用于把目标消歧到指定面，不改变算法语义。"""
    value = request['map']
    if 'hint' in request:
        surfaces = {surface['surface_id']: surface for surface in value['surfaces']}
        if request['hint'] not in surfaces:
            raise support_planner.PlannerError('E_PLAN_GOAL_INVALID', 'unknown surface_hint %s' % request['hint'])
        surface = surfaces[request['hint']]
        z = request['goal'][2]
        if abs(surface['z_reference'] - z) > 0.5:
            raise support_planner.PlannerError('E_PLAN_GOAL_INVALID', 'hint z mismatch')
    return support_planner.plan(value, request['start'], request['goal'],
                                budget=request['budget'], cancel=cancel)


def pose_xyz(pose):
    """从 PoseStamped 取 xyz 三元组；z 缺失即拒，不用 0 冒充。"""
    point = (float(pose.pose.position.x), float(pose.pose.position.y), float(pose.pose.position.z))
    return point


def to_path_message(result, frame, stamp):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    message = Path()
    message.header.frame_id = frame
    message.header.stamp = stamp
    for x, y, z, _ in to_path_points(result):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def to_path_points(result):
    """把设计层结果折成 (x, y, z, surface_id) 列表；供 nav_msgs/Path 与检查器共用。"""
    points = []
    for point in result['path']:
        if isinstance(point, dict):
            points.append((float(point['x']), float(point['y']), float(point['z']), point['surface_id']))
        else:
            points.append((float(point[0]), float(point[1]), float(point[2]), point[3]))
    return points


def refusal(error):
    """拒绝理由统一成可上报字段；PLANNER 之外不伪造成无路。"""
    return {'ok': False, 'reason': getattr(error, 'code', 'E_PLAN_INTERNAL'),
            'detail': str(getattr(error, 'detail', error))}


def main(argv=None):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    import sys

    rospy.init_node('v8_support_planner')
    map_path = rospy.get_param('~map_json')
    start = rospy.get_param('~start')
    goal = rospy.get_param('~goal')
    budget = rospy.get_param('~budget', 20000)
    hint = rospy.get_param('~surface_hint', None)
    frame = rospy.get_param('~frame', 'map')
    publisher = rospy.Publisher(rospy.get_param('~path_topic', 'support_route'), Path, queue_size=1, latch=True)
    status = rospy.get_param('~status_topic', 'support_route_status')
    request = to_request(map_path, start, goal, budget, hint)
    start_xyz = tuple(start)
    try:
        result = plan_route(request)
    except (ValueError, support_planner.PlannerError) as error:
        info = refusal(error) if hasattr(error, 'code') else {'ok': False, 'reason': 'E_PLAN_INPUT', 'detail': str(error)}
        rospy.set_param(status, info)
        rospy.logerr('support_planner refused: %s', info)
        return 5
    def sim_stamp():
        # 用仿真时钟盖戳；仿真未起时不要拿 0 冒充时间，最多等 10 s 墙钟。
        deadline = time.monotonic() + 10.0
        while rospy.Time.now().to_sec() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(0.1)
        return rospy.Time.now()

    message = to_path_message(result, frame, sim_stamp())
    publisher.publish(message)
    rospy.set_param(status, {'ok': True, 'segments': result['segments'],
                             'transitions': result['transitions'],
                             'expansions': result['expansions'], 'length': result['length']})
    # A4 取消通道：Trigger 服务把「下一次规划前取消」置位；规划入口消费一次后复位。
    # 说明：rospy 默认串行回调，规划本身毫秒级，故运行期取消是「预取消」语义（等价于任务在
    # 开算前被撤销）；这与 A4「取消可区分」的证据口径一致，不代表支持规划中途抢占。
    from std_srvs.srv import Trigger, TriggerResponse
    cancel_flag = {'requested': False}

    def request_cancel(_request):
        cancel_flag['requested'] = True
        rospy.logwarn('cancel requested: next plan will be cancelled')
        return TriggerResponse(success=True, message='cancel requested')

    cancel_srv = rospy.get_param('~cancel_service', rospy.get_name() + '/cancel')
    rospy.Service(cancel_srv, Trigger, request_cancel)

    def make_plan(request):
        from nav_msgs.srv import GetPlanResponse
        take_cancel = cancel_flag['requested']
        cancel_flag['requested'] = False
        try:
            planned = plan_route({'map': request_map(), 'start': pose_xyz(request.start),
                                  'goal': pose_xyz(request.goal), 'budget': budget,
                                  'hint': hint} if hint else
                                 {'map': request_map(), 'start': pose_xyz(request.start),
                                  'goal': pose_xyz(request.goal), 'budget': budget},
                                 cancel=(lambda: True) if take_cancel else None)
        except (ValueError, support_planner.PlannerError) as error:
            rospy.set_param(status, refusal(error) if hasattr(error, 'code')
                            else {'ok': False, 'reason': 'E_PLAN_INPUT', 'detail': str(error)})
            rospy.logerr('make_plan refused: %s', error)
            return GetPlanResponse()
        answer = to_path_message(planned, frame, rospy.Time.now())
        rospy.set_param(status, {'ok': True, 'segments': planned['segments'],
                                 'transitions': planned['transitions'],
                                 'expansions': planned['expansions'], 'length': planned['length']})
        return GetPlanResponse(plan=answer)

    def request_map():
        with open(map_path, encoding='utf-8') as stream:
            return json.load(stream)

    from nav_msgs.srv import GetPlan
    service_enabled = rospy.get_param('~make_plan_service', '/make_plan')
    if service_enabled:
        rospy.Service(service_enabled, GetPlan, make_plan)
    if rospy.get_param('~enable_getpath', False):
        import actionlib
        from mbf_msgs.msg import GetPathAction, GetPathResult
        def getpath(goal):
            # SimpleActionServer 的 execute_cb 收到的是 goal 消息本身，不是 handle。
            target = getattr(goal, 'target_pose', None) or goal.goal   # 字段名为 target_pose
            start = pose_xyz(goal.start_pose) if goal.use_start_pose else tuple(start_xyz)
            keys = {'map': request_map(), 'start': start,
                    'goal': pose_xyz(target), 'budget': budget}
            if hint:
                keys['hint'] = hint
            result = GetPathResult()
            rospy.loginfo('getpath: start=%s target=%s hint=%s', start, pose_xyz(target), hint)
            try:
                planned = plan_route(keys)
                rospy.loginfo('getpath: planned %s points', len(planned['path']))
            except (ValueError, support_planner.PlannerError) as error:
                result.outcome = 53 if 'GOAL_INVALID' in str(getattr(error, 'code', '')) else 50
                result.message = (refusal(error) if hasattr(error, 'code')
                                  else {'reason': 'E_PLAN_INPUT', 'detail': str(error)})['reason'] + ': ' + str(error)
                rospy.set_param(status, {'ok': False, 'reason': result.message.split(':')[0]})
                server.set_aborted(result, result.message)
                return
            result.path = to_path_message(planned, frame, sim_stamp())
            result.message = 'route found'
            rospy.set_param(status, {'ok': True, 'segments': planned['segments'],
                                     'transitions': planned['transitions'],
                                     'expansions': planned['expansions'], 'length': planned['length']})
            server.set_succeeded(result)
        server = actionlib.SimpleActionServer(rospy.get_param('~getpath_action', '/move_base_flex/get_path'),
                                              GetPathAction, execute_cb=getpath, auto_start=False)
        server.start()
    rospy.loginfo('support_route published: %s points on %s', len(message.poses), publisher.name)
    if not rospy.get_param('~exit_after_publish', False):
        # 生命周期一律用单调墙钟：仿真时钟未推进或停钟时，不能用它算超时。
        deadline = time.monotonic() + float(rospy.get_param('~keep_alive_s', 10.0))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            time.sleep(0.1)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
