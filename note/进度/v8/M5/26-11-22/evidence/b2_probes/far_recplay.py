#!/usr/bin/env python3
"""B2 复放：把真实 /terrain_cloud 的一帧录下来，再喂给一个干净的 far_planner。

record 子命令：订 /terrain_cloud 与 /odom_world，稳定后存最后一帧点集与当时的机器人位姿。
replay 子命令：按录下的位姿发 /odom_world，按录下的点集发 /terrain_cloud，数 /way_point。

为什么要换新进程：同一个 far_planner 实例在前面已经吃过真实帧，格网里留了状态，
直接换数据源分不清「数据不行」还是「前面已经污染了」。
"""
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

FRAME = 'map'
FIELDS = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(('x', 'y', 'z', 'intensity'))]


def record(path, seconds):
    got = {'cloud': None, 'pose': None, 'n': 0}

    def on_cloud(m):
        pts = np.array(list(pc2.read_points(
            m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)), dtype=np.float32)
        got['cloud'] = pts
        got['n'] += 1

    def on_odom(m):
        p = m.pose.pose.position
        got['pose'] = (p.x, p.y, p.z)

    rospy.init_node('far_recplay_rec', anonymous=True)
    rospy.Subscriber('/terrain_cloud', PointCloud2, on_cloud, queue_size=1)
    rospy.Subscriber('/odom_world', Odometry, on_odom, queue_size=1)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        time.sleep(0.2)
    if got['cloud'] is None or got['pose'] is None:
        print('REC_FAIL cloud=%s pose=%s' % (got['cloud'] is None, got['pose'] is None))
        return 1
    np.savez(path, cloud=got['cloud'], pose=np.array(got['pose'], dtype=np.float32))
    c = got['cloud']
    print('REC_MSGS=%d' % got['n'])
    print('REC_N=%d' % len(c))
    print('REC_X=(%.3f, %.3f) REC_Y=(%.3f, %.3f) REC_Z=(%.3f, %.3f)'
          % (c[:, 0].min(), c[:, 0].max(), c[:, 1].min(), c[:, 1].max(),
             c[:, 2].min(), c[:, 2].max()))
    print('REC_I=(%.3f, %.3f) REC_OBS_FRAC=%.4f'
          % (c[:, 3].min(), c[:, 3].max(), float((c[:, 3] >= 0.2).mean())))
    print('REC_POSE=(%.3f, %.3f, %.3f)' % got['pose'])
    return 0


def synth_cloud(wall='up'):
    """合成对照：±10 m 地面 + x=3 m 处 2 m 宽的墙。

    wall='up'  ：墙在 z=0..1.2（地面以上，FAR 的常规输入形态）
    wall='down' ：墙在 z=-1.2..0（镜像，用来测接受窗口是否偏下）
    wall='mid'  ：墙在 z=-0.4..0.8（骑跨机器人高度层）
    """
    pts = []
    step = 0.2
    n = int(10.0 / step)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            pts.append((i * step, j * step, 0.0, 0.0))
    rows = {'up': range(0, 7), 'down': range(-6, 1), 'mid': range(-2, 5)}[wall]
    for iy in range(-5, 6):
        for iz in rows:
            pts.append((3.0, iy * 0.2, iz * 0.2, 1.0))
    return pts


def replay(path, seconds, synth=False, z_override=None, zshift=0.0, wall='up'):
    # 位姿一律取录制值；合成对照与真实数据只差「点云」这一个变量。
    # z_override 用来单独把 z 拿出来做第四个变量（见 far_ab2.sh 的四相设计）。
    pose = None
    try:
        pose = [float(v) for v in np.load(path)['pose']]
    except Exception:
        pose = [0.0, 0.0, 0.436]
    if synth:
        cloud = synth_cloud(wall)
    else:
        cloud = [tuple(float(v) for v in row) for row in np.load(path)['cloud']]
    px, py, pz = pose
    if z_override is not None:
        pz = float(z_override)
    if zshift:
        cloud = [(x, y, z + zshift, it) for x, y, z, it in cloud]
    cnt = {'way': 0, 'obs': 0, 'free': 0, 'new': 0}
    obs_pts = {'max': 0}
    free_pts = {'max': 0}
    new_pts = {'max': 0}

    def on_way(_m):
        cnt['way'] += 1

    def on_obs(m):
        cnt['obs'] += 1
        n = m.width * m.height
        if n > obs_pts['max']:
            obs_pts['max'] = n

    def on_free(m):
        cnt['free'] += 1
        n = m.width * m.height
        if n > free_pts['max']:
            free_pts['max'] = n

    def on_new(m):
        cnt['new'] += 1
        n = m.width * m.height
        if n > new_pts['max']:
            new_pts['max'] = n

    rospy.init_node('far_recplay_play', anonymous=True)
    odom_pub = rospy.Publisher('/odom_world', Odometry, queue_size=1)
    cloud_pub = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=1)
    goal_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
    rospy.Subscriber('/way_point', PointStamped, on_way, queue_size=10)
    rospy.Subscriber('/FAR_obs_debug', PointCloud2, on_obs, queue_size=1)
    rospy.Subscriber('/FAR_free_debug', PointCloud2, on_free, queue_size=1)
    rospy.Subscriber('/FAR_new_debug', PointCloud2, on_new, queue_size=1)
    print('REPLAY_N=%d REPLAY_POSE=(%.3f, %.3f, %.3f) ZSHIFT=%.3f'
          % (len(cloud), px, py, pz, zshift))
    start = time.time()
    end = start + seconds
    sent_goal = False
    while not rospy.is_shutdown() and time.time() < end:
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = FRAME
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        cloud_pub.publish(pc2.create_cloud(rospy.Header(stamp=now, frame_id=FRAME), FIELDS, cloud))
        # 目标必须发：/way_point 只在有目标时才由 PlanningCallBack 产出，
        # 不发目标时 0 航点什么也证明不了。
        if not sent_goal and time.time() - start > 10.0:
            g = PointStamped()
            g.header.stamp = now
            g.header.frame_id = FRAME
            g.point.x, g.point.y, g.point.z = 8.0, 0.0, pz
            goal_pub.publish(g)
            sent_goal = True
            print('REPLAY_GOAL_SENT')
        time.sleep(0.05)
    print('REPLAY_MSGS way=%d obs=%d free=%d new=%d'
          % (cnt['way'], cnt['obs'], cnt['free'], cnt['new']))
    print('REPLAY_PTS obs=%d free=%d new=%d' % (obs_pts['max'], free_pts['max'], new_pts['max']))
    print('REPLAY_WAYPOINTS=%d' % cnt['way'])
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        raise SystemExit(2)
    if sys.argv[1] == 'record':
        raise SystemExit(record(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 12.0))
    synth = len(sys.argv) > 4 and sys.argv[4] == 'synth'
    zov = None
    if len(sys.argv) > 5 and sys.argv[5] != 'rec':
        zov = float(sys.argv[5])
    zsh = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    wl = sys.argv[7] if len(sys.argv) > 7 else 'up'
    raise SystemExit(replay(sys.argv[2], float(sys.argv[3]), synth, zov, zsh, wl))
