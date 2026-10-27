"""flat2d 定标 driver（仓内版，v2）：读预登记 manifest，下发 move_base 目标，
记录 GT/cmd，按 manifest 里的冻结阈值评价，写结果 JSON。

与 /work 版（v1 smoke）差异：
- 判据阈值不再硬编码：从 manifest.criteria 读取（预登记时锁定冻结值 0.30/0.20）；
- 障碍几何从 world_model.json 读（v1 的 OBSTACLES 有三处尺寸与 m1_world.world 不符）；
- 支持 SC09 动态注入：manifest.inject 给出模型/位姿/触发里程，driver 用
  /gazebo/spawn_sdf_model 生成方块（地图里没有它，靠感知与重规划绕行）。

用法：python3 driver.py <manifest.json> <out.json>
退出码：0=PASS，6=FAIL，2=manifest 缺字段
"""
import json
import math
import pathlib
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from geometry import min_clearance, sdf_box

HERE = pathlib.Path(__file__).resolve().parent

DEADLINE_S = 240.0
ARRIVE_HOLD_S = 3.0
ARRIVE_RADIUS_M = 0.28


def spawn_box(name, size, pose_xy_yaw):
    from gazebo_msgs.srv import DeleteModel, GetModelState, SpawnModel
    from geometry_msgs.msg import Pose
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10.0)
    # 同名模型跨 run 复用；上一轮 delete 若滞后会让 spawn 直接失败（R792 实测间歇
    # injection_failed）。生成前先删一次同名实体，删不到也无害。
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(name)
    except Exception:
        pass
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    pose = Pose()
    pose.position.x, pose.position.y = pose_xy_yaw[0], pose_xy_yaw[1]
    pose.position.z = size[2] / 2
    pose.orientation.w = 1.0
    # gazebo_ros 的 spawn 用 initial_pose，SDF 里的 <pose> 被忽略（R792 实测：
    # 旧版传全零 Pose，方块一直生成在原点，ROI 里永远看不到它）。SDF 位姿写零。
    resp = spawn(name, sdf_box(name, size, (0.0, 0.0, 0.0), 0.0), '/', pose, 'world')
    if not resp.success:
        return False
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state = get_state(name, 'world')
        if not state.success:
            return False
        dx = state.pose.position.x - pose_xy_yaw[0]
        dy = state.pose.position.y - pose_xy_yaw[1]
        return dx * dx + dy * dy <= 0.15 ** 2
    except Exception:
        return False


def main(argv):
    manifest_path, out_path = argv[0], argv[1]
    man = json.loads(pathlib.Path(manifest_path).read_text())
    criteria = man.get('criteria') or {}
    goal_tol = float(criteria.get('goal_position_m', 0.30))
    deadline_s = float(criteria.get('deadline_s', DEADLINE_S))
    clearance_min = float(criteria.get('min_clearance_m', 0.20))
    goal = tuple(man['goal'])
    start = tuple(man.get('start', [0.0, 0.0]))
    inject = man.get('inject')

    world = json.loads((HERE / 'world_model.json').read_text())
    boxes = [tuple(b) for b in world['boxes']]
    inject_box = None
    if inject:
        # 注入块也进净空判据；否则「穿过方块也到点」会被误判 PASS（R789）。
        ix, iy, iyaw = inject['pose_xy_yaw']
        inject_box = (ix, iy, inject['size'][0], inject['size'][1], iyaw)
        boxes.append(inject_box)

    rospy.init_node('flat2d_driver', anonymous=True, disable_signals=True)
    odom = []
    cmd_count = [0]
    motion_count = [0]
    map_seen = [False]

    def on_cmd(msg):
        cmd_count[0] += 1
        lin = getattr(getattr(msg, 'linear', None), 'x', 0.0)
        ang = getattr(getattr(msg, 'angular', None), 'z', 0.0)
        if abs(lin) + abs(ang) > 0.01:
            motion_count[0] += 1

    rospy.Subscriber('/sim/odom_gt', Odometry,
                     lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y)))
    rospy.Subscriber('/cmd_vel', Twist, on_cmd)
    rospy.Subscriber('/map', rospy.AnyMsg, lambda _m: map_seen.__setitem__(0, True))
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    box_pub = rospy.Publisher('/sc09/box', Float32MultiArray, queue_size=1, latch=True)

    t0 = time.time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        if map_seen[0] and len(odom) >= 30:
            topics = [t for t, _ in rospy.get_published_topics()]
            if '/move_base/result' in topics:
                break
        rate.sleep()

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = goal
    pose.pose.orientation.w = 1.0
    goal_pub.publish(pose)

    injected_at = None
    injection_ok = None
    inject_idx = None
    start_ref = None
    stable = 0.0
    last = time.time()
    deadline = time.time() + deadline_s
    while not rospy.is_shutdown() and time.time() < deadline:
        now = time.time()
        dt = now - last
        last = now
        if odom:
            px, py = odom[-1]
            if inject and injection_ok is None:
                if start_ref is None:
                    start_ref = (px, py)  # 复位后首帧为里程基准，避免遥测跳变提前触发
                travelled = math.hypot(px - start_ref[0], py - start_ref[1])
                if travelled >= float(inject['at_progress_m']):
                    injection_ok = spawn_box(inject['model_name'], inject['size'], inject['pose_xy_yaw'])
                    injected_at = round(now - t0, 1)
                    inject_idx = len(odom)
                    if injection_ok:
                        box_pub.publish(Float32MultiArray(
                            data=list(inject['pose_xy_yaw'][:2]) + [inject['pose_xy_yaw'][2]]
                            + list(inject['size'][:3])))
            if math.hypot(px - goal[0], py - goal[1]) <= ARRIVE_RADIUS_M:
                stable += dt
                if stable >= ARRIVE_HOLD_S:
                    break
            else:
                stable = 0.0
        rate.sleep()

    end = odom[-1] if odom else None
    dist_goal = math.hypot(end[0] - goal[0], end[1] - goal[1]) if end else None
    clear = min_clearance(odom, boxes)
    result = {
        'run_id': man['run_id'],
        'run_class': man['run_class'],
        'calibration_only': man['calibration_only'],
        'scenario_id': man['scenario_id'],
        'seed': man['seed'],
        'repetition': man['repetition'],
        'goal': list(goal),
        'criteria': {'goal_position_m': goal_tol, 'min_clearance_m': clearance_min},
        'map_received': map_seen[0],
        'cmd_vel_messages': cmd_count[0],
        'motion_command_messages': motion_count[0],
        'odom_samples': len(odom),
        'path_length_m': round(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                   for a, b in zip(odom, odom[1:])), 2) if odom else 0,
        'final_pos': list(end) if end else None,
        'dist_to_goal_m': round(dist_goal, 3) if dist_goal is not None else None,
        'obstacle_min_clearance_m': round(clear, 3) if clear is not None else None,
        'injected_box_clearance_m': (
            round(min_clearance(odom[inject_idx:], [inject_box]), 3)
            if (inject_box is not None and inject_idx is not None
                and odom[inject_idx:]) else None),
        'inject': inject,
        'injected_at_s': injected_at,
        'injection_ok': injection_ok,
        'wall_duration_s': round(time.time() - t0, 1),
        'deadline_hit': time.time() >= deadline,
    }
    passed = bool(
        result['map_received'] and motion_count[0] >= 10
        and result['odom_samples'] >= 50 and dist_goal is not None and dist_goal <= goal_tol
        and clear is not None and clear >= clearance_min
        and (injection_ok is not False))
    result['verdict'] = 'PASS' if passed else 'FAIL'
    result['fail_reason'] = None if passed else (
        'no_map' if not result['map_received'] else
        'no_command' if motion_count[0] < 10 else
        'insufficient_odom' if result['odom_samples'] < 50 else
        'injection_failed' if injection_ok is False else
        'not_at_goal' if (dist_goal is None or dist_goal > goal_tol) else
        'clearance_violation')
    box_pub.publish(Float32MultiArray())  # 撤方块：中继停止 ROI 转发，STVL 衰减
    pathlib.Path(out_path).write_text(json.dumps(result, ensure_ascii=False, indent=1))
    print('FLAT2D', json.dumps(result, ensure_ascii=False))
    return 0 if passed else 6


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
