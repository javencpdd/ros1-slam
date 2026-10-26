"""V8-09 运行面包装：把零 ROS 的 support_planner 暴露为 ROS1 节点。

设计层（support_planner.plan）不 import ROS；本文件只做参数/消息适配，
核心转换放在纯函数里（to_request / to_path_points / refusal），便于离线测试。
"""
import json
import math

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
    try:
        result = plan_route(request)
    except (ValueError, support_planner.PlannerError) as error:
        info = refusal(error) if hasattr(error, 'code') else {'ok': False, 'reason': 'E_PLAN_INPUT', 'detail': str(error)}
        rospy.set_param(status, info)
        rospy.logerr('support_planner refused: %s', info)
        return 5
    message = Path()
    message.header.frame_id = frame
    message.header.stamp = rospy.Time.now()
    for x, y, z, _ in to_path_points(result):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    publisher.publish(message)
    rospy.set_param(status, {'ok': True, 'segments': result['segments'],
                             'transitions': result['transitions'],
                             'expansions': result['expansions'], 'length': result['length']})
    rospy.loginfo('support_route published: %s points on %s', len(message.poses), publisher.name)
    if not rospy.get_param('~exit_after_publish', False):
        deadline = rospy.Time.now() + rospy.Duration(rospy.get_param('~keep_alive_s', 10.0))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rospy.sleep(0.2)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
