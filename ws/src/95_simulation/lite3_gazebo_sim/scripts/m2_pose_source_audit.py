#!/usr/bin/env python3
# m2_pose_source_audit.py —— /Odometry 发布者身份与混流审计（小明，2026-10-08，源=小张 F-D 件更正①）
# 为何存在：faster_lio 在 laser_mapping.cc:265 以绝对名 advertise /Odometry 并于 :343-344 无条件发布，
#   而自研驱动插件按 lite3_planar.urdf.xacro:140 <odometryTopic>/Odometry</odometryTopic> 亦发布同名话题。
#   m2_smoke.sh 的 stageA（M2-1/M2-4 判据）在 :109 订阅 /Odometry 算回程与漂移 —— 若两写者同时在位，
#   ROS1 订阅端会收到**交织双流**，则已验收判据的位姿源身份不可归因。本件把「谁在发、发的是哪一系、
#   两系差多少」实测出来，判定权在验收。
# 用法：python3 m2_pose_source_audit.py <标签>（在 stageA 同场链起好后运行）
import sys, time, collections
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

TAG = sys.argv[1] if len(sys.argv) > 1 else 'a'
rospy.init_node('m2_pose_source_audit')

gt = [None]
rospy.Subscriber('/gazebo/model_states', ModelStates,
                 lambda m: [gt.__setitem__(0, (p.position.x, p.position.y))
                            for p, n in zip(m.pose, m.name) if n == 'lite3_planar'] and None, queue_size=1)

by_pair = collections.defaultdict(list)
def cb(m):
    by_pair[(m.header.frame_id, m.child_frame_id)].append(
        (m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y))
rospy.Subscriber('/Odometry', Odometry, cb, queue_size=50)

time.sleep(8.0)
print('AUDIT tag=%s 不同(frame,child) 组合数=%d' % (TAG, len(by_pair)))
for (f, c), rows in sorted(by_pair.items()):
    xs = [r[1] for r in rows]; ys = [r[2] for r in rows]
    st = [r[0] for r in rows]
    print('AUDIT tag=%s src=%s->%s n=%d stamp_first=%.3f stamp_last=%.3f x=[%.3f,%.3f] y=[%.3f,%.3f]'
          % (TAG, f, c, len(rows), min(st), max(st), min(xs), max(xs), min(ys), max(ys)))
if gt[0] is not None:
    print('AUDIT tag=%s gt_model_states=(%.3f,%.3f)' % (TAG, gt[0][0], gt[0][1]))
    for (f, c), rows in sorted(by_pair.items()):
        dx = rows[-1][1] - gt[0][0]; dy = rows[-1][2] - gt[0][1]
        print('AUDIT tag=%s dev(%s->%s vs gt)=(%.3f,%.3f) |d|=%.3f' % (TAG, f, c, dx, dy, (dx * dx + dy * dy) ** 0.5))
if len(by_pair) > 1:
    print('AUDIT tag=%s VERDICT=DOUBLE_WRITER（/Odometry 双流交织，判据位姿源不可归因）' % TAG)
else:
    print('AUDIT tag=%s VERDICT=SINGLE_WRITER' % TAG)
