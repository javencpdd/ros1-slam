"""mode3 执行中继的控制律（纯函数，零 ROS）：把「当前位姿→下一个路点」折成 (v, w)。

只做最小闭环：朝路点转向、按距离减速、到达即零速；取消或超时一律零速，
不保留最后一次速度（这正是 22 号 §4「取消后继续用最后速度」的反例）。
"""
import math


def wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def twist_for(pose, waypoint, *, max_v=0.25, max_w=0.8, k_ang=1.5, xy_tol=0.08, allow_reverse=False):
    """pose=(x,y,yaw)，waypoint=(x,y)；返回 (v, w, arrived)。"""
    dx, dy = waypoint[0] - pose[0], waypoint[1] - pose[1]
    dist = math.hypot(dx, dy)
    if dist <= xy_tol:
        return 0.0, 0.0, True
    heading = math.atan2(dy, dx)
    error = wrap(heading - pose[2])
    w = max(-max_w, min(max_w, k_ang * error))
    if abs(error) >= math.pi / 2 - 1e-9 and not allow_reverse:
        return 0.0, w, False          # 先转向，不硬拐
    v = max_v * min(1.0, dist / 0.5)
    if abs(error) > 0.6:
        v *= 0.3
    return v, w, False


def stop_twist():
    """取消/超时/故障统一出口：零速零角。"""
    return 0.0, 0.0
