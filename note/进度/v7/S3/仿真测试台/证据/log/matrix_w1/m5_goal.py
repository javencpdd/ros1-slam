# 取 M5 目标点：当前轨线位姿前方第 AHEAD 个路点（traj_maze 走廊格心，rails 精确穿过）
import sys, rospy, yaml, math
from nav_msgs.msg import Odometry
AHEAD = int(sys.argv[1]) if len(sys.argv) > 1 else 5
rospy.init_node('m5_goal_picker')
traj = yaml.safe_load(open('/home/ros1-slam/ws/src/00_drivers/lite3_drivers/config/traj_maze.yaml'))
wp = traj['waypoints']
msg = rospy.wait_for_message('/quad_0/body_pose', Odometry, timeout=10)
x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
d = [math.dist((x, y), (p[0], p[1])) for p in wp]
i = d.index(min(d))
gi = min(i + AHEAD, len(wp) - 1)
print('CUR=(%.3f,%.3f) wp[%d] GOAL=wp[%d]=(%.3f,%.3f) 轨线距=%.1fm' % (x, y, i, gi, wp[gi][0], wp[gi][1], min(d)))
print('%.3f %.3f' % (wp[gi][0], wp[gi][1]))
