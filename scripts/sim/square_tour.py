#!/usr/bin/env python3
"""方形轨迹巡游：闭环点位控制，用真值 TF(map→base_link) 做反馈。

为什么闭环而不是开环定时：
- 开环（按时间发速度）会因惯性/摩擦累积误差，方形走成梯形；
- 闭环读真值位置，到点才切下一个目标，误差 ≤ 阈值。

为什么用真值而不是 /Odometry：
- /Odometry 是 faster_lio 的估计，会漂；用它做反馈等于"用被测量的东西校准自己"。
- 评分（闭环误差）也必须用真值，否则测的是 SLAM 自洽性而非准确度。

持续以 10 Hz 发 cmd_vel：驱动器有 stale 超时（约 2.8s），不持续发就会停车。
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist

SPEED = 0.45          # m/s
TURN = 0.9            # rad/s
TOL = 0.22            # 到点阈值 m
MAX_T = 45.0          # 单段超时 s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('square_tour', anonymous=True)
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(3)

    def pose():
        t = buf.lookup_transform('map', 'base_link', rospy.Time(0))
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    x0, y0, _ = pose()
    print(f'[tour] 起点 ({x0:.2f}, {y0:.2f})', flush=True)
    # 方形：先向东 2.4m，再向南 2.4m，再向西 2.4m，最后回起点
    side = 2.4
    waypoints = [(x0 + side, y0), (x0 + side, y0 - side), (x0, y0 - side), (x0, y0)]

    rate = rospy.Rate(10)
    for idx, (tx, ty) in enumerate(waypoints):
        print(f'[tour] 目标 {idx+1}/4: ({tx:.2f}, {ty:.2f})', flush=True)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            cx, cy, cyaw = pose()
            dx, dy = tx - cx, ty - cy
            dist = math.hypot(dx, dy)
            if dist < TOL:
                print(f'[tour]   到点，剩余 {dist:.2f} m', flush=True)
                break
            if rospy.Time.now().to_sec() - t0 > MAX_T:
                print(f'[tour]   超时，剩余 {dist:.2f} m —— 可能被挡', flush=True)
                break
            err = math.atan2(dy, dx) - cyaw
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            cmd = Twist()
            if abs(err) > 0.5:                 # 朝向差太多：先转
                cmd.angular.z = TURN if err > 0 else -TURN
            else:
                cmd.linear.x = min(SPEED, 0.9 * dist)
                cmd.angular.z = max(-TURN, min(TURN, 1.6 * err))
            pub.publish(cmd)
            rate.sleep()

    pub.publish(Twist())                      # 停
    rospy.sleep(0.5)
    xe, ye, _ = pose()
    print(f'[tour] 终点 ({xe:.2f}, {ye:.2f})', flush=True)
    print(f'[tour] === 闭环误差 {math.hypot(xe-x0, ye-y0):.3f} m ===', flush=True)


if __name__ == '__main__':
    main()
