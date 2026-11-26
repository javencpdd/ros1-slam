#!/usr/bin/env python3
"""ROS1 topic adapter for the real support planner: map/start/goal in, Path out."""
import json
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import support_planner  # noqa: E402
import ros_node as legacy  # noqa: E402


def plan_request(map_value, start, goal, budget=20000, cancel=None,
                 min_clearance_m=None, robot_radius_m=None, robot_height_m=None,
                 goal_surface_id=None):
    return support_planner.plan(map_value, tuple(start), tuple(goal), budget=budget,
                                cancel=cancel,
                                min_clearance_m=min_clearance_m,
                                robot_radius_m=robot_radius_m,
                                robot_height_m=robot_height_m,
                                goal_surface_id=goal_surface_id)


def map_from_message(text):
    value = json.loads(text)
    if isinstance(value, dict) and 'map' in value:
        value = value['map']
    if not isinstance(value, dict):
        raise ValueError('map message must be an object')
    return value


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_support_planner_topic')
    frame = rospy.get_param('~frame', 'map')
    budget = int(rospy.get_param('~budget', 20000))
    policy = {key: rospy.get_param('~' + key, None)
              for key in ('min_clearance_m', 'robot_radius_m', 'robot_height_m')}
    goal_surface_id = rospy.get_param('~surface_hint', None)
    from std_msgs.msg import String as _String
    path_meta_pub = rospy.Publisher(rospy.get_param('~path_meta_topic', '/v8/3d/global/path_meta'),
                                    _String, queue_size=10)
    path_task_pub = rospy.Publisher(rospy.get_param('~path_task_topic', '/v8/3d/global/path_task'),
                                    _String, queue_size=10)
    path_pub = rospy.Publisher(rospy.get_param('~path_topic', '/global_path'),
                               Path, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/global_path/status'),
                                 String, queue_size=10, latch=True)
    state = {'map': None, 'map_version': None, 'start': None, 'goal': None,
             # ★任务代次★：goal 的 header.seq 由 BT 盖章（每次出队新任务自增）。
             # 必须原样带到 path 上——下游 path_goal 用它区分"同一任务的滚动重发"与"新任务"，
             # 只看时间戳无法区分（旧任务刷新时间戳照样会来）。
             'goal_seq': 0,
             # ★任务身份快照★：由 /v8/bt/goal_meta 提供（mission_id/path_revision/waypoint_index/
             # execution_generation）。**在请求时快照**，异步结果回写时只用这份快照，
             # 绝不读"最新的"共享值给旧结果盖章。
             'goal_meta': None}
    lock = threading.Lock()
    job = {'generation': 0, 'cancel': None}

    def publish(value):
        status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def run(map_value, start, goal, version, generation, cancel, meta=None):
        try:
            result = plan_request(map_value, start, goal, budget=budget,
                                  cancel=cancel.is_set,
                                  goal_surface_id=goal_surface_id, **policy)
        except Exception as error:
            with lock:
                if generation != job['generation']:
                    return
            path_pub.publish(Path())
            publish(dict(legacy.refusal(error), map_version=version))
            return
        points = legacy.to_path_points(result)
        # ★原子路径：身份与点列同一条消息★（跨话题不保序，配对不可靠）
        if meta:
            payload = dict(meta)
            payload['poses'] = [[round(x, 6), round(y, 6), round(z, 6)] for x, y, z, _s in points]
            path_task_pub.publish(_String(data=json.dumps(payload, ensure_ascii=False)))
        # 兼容：仍发 meta + Path 给旧消费者
        if meta:
            path_meta_pub.publish(_String(data=json.dumps(meta, ensure_ascii=False)))
        message = Path()
        message.header.frame_id, message.header.stamp = frame, rospy.Time.now()
        message.header.seq = int(state.get('goal_seq') or 0)
        for x, y, z, _surface in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        with lock:
            if generation != job['generation']:
                return
        path_pub.publish(message)
        publish({'ok': True, 'reason': result.get('reason'),
                 'points': len(points), 'map_version': version,
                 'expansions': result.get('expansions'), 'goal': list(goal)})

    def replan():
        if state['map'] is None or state['start'] is None or state['goal'] is None:
            publish({'ok': False, 'reason': 'E_PLAN_NOT_READY',
                     'map_version': state['map_version']})
            return
        # 身份快照与几何快照一起取，保证"这一份结果"配的就是"这一份身份"
        snapshot = (state['map'], state['start'], state['goal'], state['map_version'],
                    state['goal_meta'])
        with lock:
            job['generation'] += 1
            if job['cancel'] is not None:
                job['cancel'].set()
            cancel = threading.Event()
            job['cancel'] = cancel
            generation = job['generation']
        threading.Thread(target=run, args=snapshot[:4] + (generation, cancel) + (snapshot[4],),
                         daemon=True).start()
        publish({'ok': False, 'reason': 'E_PLAN_RUNNING',
                 'map_version': snapshot[3]})

    def on_cancel(_request):
        with lock:
            if job['cancel'] is not None:
                job['cancel'].set()
        publish({'ok': False, 'reason': 'E_PLAN_CANCELLED_REQUESTED'})
        return TriggerResponse(success=True, message='cancel requested')

    def on_map(message):
        try:
            value = map_from_message(message.data)
        except ValueError:
            publish({'ok': False, 'reason': 'E_PLAN_MAP_JSON'})
            return
        state['map'], state['map_version'] = value, value.get('map_version')
        replan()

    def on_pose(key):
        def callback(message):
            if message.header.frame_id != frame:
                publish({'ok': False, 'reason': 'E_PLAN_FRAME'})
                return
            p = message.pose.position
            state[key] = (p.x, p.y, p.z)
            replan()
        return callback

    rospy.Subscriber(rospy.get_param('~map_topic', '/v8_support_map'), String,
                     on_map, queue_size=1)
    rospy.Subscriber(rospy.get_param('~start_topic', '/v8/global/start'), PoseStamped,
                     on_pose('start'), queue_size=1)
    def on_goal_task(message):
        """★原子任务★：身份与位姿同一条消息，直接设置 state（不依赖跨话题顺序）。"""
        try:
            value = json.loads(message.data)
        except Exception:
            return
        try:
            state['goal'] = (float(value['x']), float(value['y']), float(value.get('z', 0.0)))
            state['goal_meta'] = {k: int(value.get(k, 0)) for k in
                                  ('mission_id', 'path_revision', 'waypoint_index',
                                   'execution_generation')}
        except (KeyError, TypeError, ValueError):
            return
        replan()

    rospy.Subscriber(rospy.get_param('~goal_task_topic', '/v8/bt/goal_task'),
                     _String, on_goal_task, queue_size=5)

    def on_goal_meta(message):
        try:
            value = json.loads(message.data)
        except Exception:
            return
        state['goal_meta'] = {k: int(value.get(k, 0)) for k in
                              ('mission_id', 'path_revision', 'waypoint_index', 'execution_generation')}

    rospy.Subscriber(rospy.get_param('~goal_meta_from_bt', '/v8/bt/goal_meta'),
                     _String, on_goal_meta, queue_size=10)

    _goal_cb = on_pose('goal')

    def on_goal(message):
        # 记录任务代次后再走通用路径（顺序无关，但要保证 run() 读到的是本任务的值）
        try:
            state['goal_seq'] = int(getattr(message.header, 'seq', 0) or 0)
        except Exception:
            state['goal_seq'] = 0
        _goal_cb(message)

    rospy.Subscriber(rospy.get_param('~goal_topic', '/v8/global/goal'), PoseStamped,
                     on_goal, queue_size=1)
    rospy.Service(rospy.get_param('~cancel_service',
                                 rospy.get_name() + '/cancel'), Trigger, on_cancel)
    keep_alive_s = float(rospy.get_param('~keep_alive_s', 0.0))
    publish({'ok': True, 'waiting': True})
    deadline = time.monotonic() + keep_alive_s
    while not rospy.is_shutdown():
        if keep_alive_s > 0 and time.monotonic() >= deadline:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
