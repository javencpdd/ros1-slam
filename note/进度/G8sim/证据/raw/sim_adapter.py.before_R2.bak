#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# sim_adapter.py —— G8-sim 仿真预演适配器（Q14 自研件；设计依据：小张 payload 03 + faster_lio 契约实测）
# ① sensor_cloud(PointXYZI)→/rslidar_points（velodyne_ros::Point 布局 intensity@16/time@20/ring@24,step=32；
#    time=0（坑13-R1④：render 单位姿整帧成像，合成扫掠时戳致 deskew 剪切，改全 0=帧内瞬时）；ring=i%32；
#    intensity 透传；world 系 z≥-0.02 地下幻影点剔除（坑13-R1①，配对 lidar_pose 同戳）；frame_id=rslidar；
#    限频 10Hz 最新者胜）
# ② /quad_0/lidar_pose 差分合成→/livox/imu @200Hz（陀螺四元数差+中值+低通→R^T·ω_w；加速度 5 点 SG 二阶
#    差分→R^T(a_w−g)；偏置+噪声+4g 限幅；Δq 奇异沿用上帧；stamp 单调化；断流>1s 停发）
# ③ --mode traj：--traj yaml→/quad_0/body_pose @50Hz（静置→waypoints→回起点，偏航限速，开环完美位姿）。
# 已知简化（Q14）：渲染点无扫描几何/IMU 位姿差分/开环。红线：不启 M6、不碰 /cmd_vel、不改拷贝包。
import argparse
import collections
import math

import numpy as np
import yaml
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField

G_VEC = np.array([0.0, 0.0, -9.81])
SCAN_PERIOD = 0.1          # s，与真机雷达节拍一致
GYRO_BIAS, GYRO_SIGMA, ACC_SIGMA = 0.002, 0.01, 0.05   # 偏置/噪声（防纯零激励）
ACC_CLAMP = 4.0 * 9.81     # |a_b| 限幅
LPF_ALPHA, YAW_RATE_MAX = 0.3, 1.0                     # 低通系数 / 轨迹偏航角速率限幅 rad/s
Z_MIN_WORLD = -0.02     # 坑13-R1①：world 系地下幻影点剔除阈值（mockamap 墙底格心 z≥0.0，幻影层实测 ≤-0.19m，-0.02 给 0.17m 安全边距）

# 输入视图 pcl::PointXYZI（x@0,y@4,z@8,intensity@16）/ 输出视图 velodyne_ros::Point（preprocess.h:12-20）
DTYPE_IN = np.dtype({'names': ['x', 'y', 'z', 'p0', 'intensity', 'p1'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', 'V12'],
                     'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 32})
DTYPE_OUT = np.dtype({'names': ['x', 'y', 'z', 'w', 'intensity', 'time', 'ring', 'p2'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<f4', '<u2', 'V6'],
                      'offsets': [0, 4, 8, 12, 16, 20, 24, 26], 'itemsize': 32})
FIELDS_OUT = [PointField(n, o, 7, 1) for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 16), ('time', 20))] \
             + [PointField('ring', 24, 4, 1)]


def quat_to_R(q):
    """q=[x,y,z,w] → 3x3 旋转阵（归一化，退化返回单位阵）"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def quat_slerp(q0, q1, u):
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        o = q0 + u * (q1 - q0)
        return o / np.linalg.norm(o)
    th, s = math.acos(min(1.0, d)), math.sin(math.acos(min(1.0, d)))
    return (math.sin((1 - u) * th) / s) * q0 + (math.sin(u * th) / s) * q1


def quat_from_yaw(yaw):
    return np.array([0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)])


def world_gyro_w(q1, q0, dt):
    """Δq=q1⊗q0*（世界系增量）→ 轴角/Δt → 世界系角速度；奇异返回 None"""
    if dt <= 1e-9:
        return None
    dq = quat_mul(q1, np.array([-q0[0], -q0[1], -q0[2], q0[3]]))   # q1⊗q0*
    if dq[3] < 0: dq = -dq
    s = float(np.linalg.norm(dq[:3]))
    if s < 1e-6:
        return None
    return dq[:3] / s * (2.0 * math.atan2(s, dq[3]) / dt)


def sg2_accel(pos5, dt):
    """5 点 Savitzky-Golay 二阶差分（等间隔近似）：[2,-1,-2,-1,2]/(7dt²)"""
    return (2 * pos5[0] - pos5[1] - 2 * pos5[2] - pos5[3] + 2 * pos5[4]) / (7.0 * dt * dt)


def repack_cloud(msg, time_unit, zcut=None):
    """sensor_cloud → velodyne_ros::Point 布局 PointCloud2（纯函数，供 selfcheck 复用）
    zcut=(R 第 3 行 (3,), 配对位姿 z, z_min)：world 系 zw=x*r2[0]+y*r2[1]+z*r2[2]+pz < z_min 的点剔除（坑13-R1①）"""
    if msg.point_step != 32:
        raise ValueError('point_step=%d 期望 32（pcl::PointXYZI 契约）' % msg.point_step)
    pts = np.frombuffer(bytes(msg.data), dtype=DTYPE_IN)
    n = len(pts)
    out = np.zeros(n, dtype=DTYPE_OUT)
    out['x'], out['y'], out['z'], out['intensity'] = pts['x'], pts['y'], pts['z'], pts['intensity']
    out['time'] = 0.0   # 坑13-R1④：render 单位姿整帧成像，合成 0..0.1s 扫掠时戳致 deskew 剪切（0.5m/s 端点~5cm/帧），改全 0=帧内瞬时
    out['ring'] = np.arange(n) % 32
    if zcut is not None:                      # 坑13-R1①：world 系地下幻影点剔除
        r2, pz, z_min = zcut                  # R 第 3 行 (3,)、配对位姿 z、阈值
        zw = out['x']*r2[0] + out['y']*r2[1] + out['z']*r2[2] + pz
        out = out[zw >= z_min]
        if len(out) == 0: raise ValueError('all points underground')
        n = len(out)
    pc = PointCloud2()
    pc.header.stamp, pc.header.frame_id = msg.header.stamp, 'rslidar'
    pc.height, pc.width, pc.fields = 1, n, FIELDS_OUT
    pc.is_bigendian, pc.is_dense = False, True
    pc.point_step, pc.row_step, pc.data = 32, 32 * n, out.tobytes()
    return pc


def load_traj(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    wp = np.array(cfg['waypoints'], dtype=np.float64)
    assert wp.ndim == 2 and wp.shape[1] == 2 and len(wp) >= 2, 'waypoints 需 ≥2 个 [x,y]'
    return {'wp': wp, 'z': float(cfg.get('z', 0.0)), 'speed': float(cfg.get('speed', 0.5)),
            'hold_start': float(cfg.get('hold_start_s', 5.0)), 'hold_end': float(cfg.get('hold_end_s', 5.0))}


class SimAdapter(object):
    def __init__(self, opt):
        self.opt = opt
        self.dt_imu = 1.0 / opt.rate_imu
        self.latest_cloud = None
        self.pose_buf = collections.deque(maxlen=64)   # (t, p[3], q[4] xyzw)
        self.t_cursor = None                            # IMU 发布游标（传感器时域，单调）
        self.last_pose_stamp = self.last_pose_wall = None
        self.gyro_med = collections.deque(maxlen=3)
        self.ww_filt = np.zeros(3)
        self.last_ww_meas = np.zeros(3)     # 世界系 ω 测量值（奇异帧沿用；不回喂输出，防噪声反馈）
        self.rng = np.random.default_rng(20261005)
        self.pub_pts = rospy.Publisher('/rslidar_points', PointCloud2, queue_size=2)
        self.pub_imu = rospy.Publisher('/livox/imu', Imu, queue_size=200, tcp_nodelay=True)

    def on_cloud(self, msg):
        self.latest_cloud = msg                         # 最新者胜，低频定时器统一发

    def on_pose(self, msg):
        t = msg.header.stamp.to_sec()
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        if self.last_pose_stamp is not None and t < self.last_pose_stamp - 1e-6:
            self.pose_buf.clear(); self.t_cursor = None  # stamp 回跳：清缓冲重置
            rospy.logwarn('[sim_adapter] lidar_pose stamp 回跳，缓冲已清')
        self.pose_buf.append((t, p, q))
        self.last_pose_stamp, self.last_pose_wall = t, rospy.get_rostime()

    def paired_zcut(self, t_cloud):
        """坑13-R1①：cloud 戳（=body_pose 戳域）与 pose_buf 精确同戳配对 → world 系 z 剔除参数；无配对返回 None 不误杀"""
        for (t, p, q) in reversed(self.pose_buf):
            if abs(t - t_cloud) <= 5e-3:
                R = quat_to_R(q)
                return R[2], float(p[2]), Z_MIN_WORLD   # R 第 3 行、配对位姿 z、阈值 -0.02
        return None                                # 无配对位姿：不剔（保底不误杀）

    def cloud_timer(self, _):
        """① 限频发布（--rate-lidar Timer）"""
        msg, self.latest_cloud = self.latest_cloud, None
        if msg is None:
            return
        try:
            pc = repack_cloud(msg, self.opt.time_unit, zcut=self.paired_zcut(msg.header.stamp.to_sec()))
        except ValueError as e:
            rospy.logwarn_throttle(5.0, '[sim_adapter] 点云契约不符: %s' % e)
            return
        # 保留 render 原戳（内容时刻），fix⑥：now() 覆盖曾致点云-IMU 系统性时域错位
        self.pub_pts.publish(pc)

    def imu_timer(self, _):
        """② 合成 IMU（--rate-imu Timer，默认 200Hz）"""
        buf = self.pose_buf
        self._dbg = getattr(self, '_dbg', 0) + 1
        if self._dbg % 200 == 0 and len(self.pose_buf) >= 2:
            b = self.pose_buf
            rospy.loginfo_throttle(2, '[imu_dbg] calls=%d dt=%.4f ww_filt=%s q1=%s' % (self._dbg, b[-1][0] - b[-2][0], np.round(self.ww_filt, 3).tolist(), np.round(b[-1][2], 3).tolist()))
        if len(buf) < 5 or self.last_pose_wall is None:
            return
        if (rospy.get_rostime() - self.last_pose_wall).to_sec() > 1.0:
            return  # 断流保护：停发，不喂冻结 IMU
        t_now = buf[-1][0]                  # 插值目标=最新位姿时刻（墙钟域，防爬行/跳变 dt 畸变）
        i = len(buf) - 2
        for k in range(len(buf) - 1):
            if buf[k][0] <= t_now <= buf[k + 1][0]:
                i = k
                break
        (t0, _, q0), (t1, _, q1) = buf[i], buf[i + 1]
        q_now = quat_slerp(q0, q1, (t_now - t0) / max(t1 - t0, 1e-9))
        ww = world_gyro_w(buf[-1][2], buf[-2][2], buf[-1][0] - buf[-2][0])
        if ww is None:
            ww = self.last_ww_meas          # 静置/奇异：沿用上帧测量（payload 03 坑②语义）
        else:
            self.last_ww_meas = ww
        self.gyro_med.append(ww)
        self.ww_filt = (1 - LPF_ALPHA) * self.ww_filt + LPF_ALPHA * np.median(np.array(self.gyro_med), axis=0)
        R = quat_to_R(q_now)
        w_b = R.T @ self.ww_filt + GYRO_BIAS + self.rng.normal(0, GYRO_SIGMA, 3)
        pos5 = np.array([buf[j][1] for j in range(-5, 0)])
        dt5 = (buf[-1][0] - buf[-5][0]) / 4.0
        a_b = R.T @ (sg2_accel(pos5, dt5 if dt5 > 1e-9 else 1e-9) - G_VEC)
        nrm = np.linalg.norm(a_b)
        if nrm > ACC_CLAMP:
            a_b *= ACC_CLAMP / nrm
        a_b = a_b + self.rng.normal(0, ACC_SIGMA, 3)
        im = Imu()
        im.header.stamp, im.header.frame_id = rospy.get_rostime(), 'livox_frame'  # fix③墙钟域
        im.orientation.x, im.orientation.y, im.orientation.z, im.orientation.w = q_now
        im.angular_velocity.x, im.angular_velocity.y, im.angular_velocity.z = w_b
        im.linear_acceleration.x, im.linear_acceleration.y, im.linear_acceleration.z = a_b
        for arr, v in ((im.orientation_covariance, 0.01), (im.angular_velocity_covariance, 0.01),
                       (im.linear_acceleration_covariance, 0.1)):
            arr[0] = arr[4] = arr[8] = v
        self.pub_imu.publish(im)

    def spin_traj(self, traj):
        """③ 轨迹源（--mode traj，50Hz）：静置→waypoints 匀速→回起点静置"""
        wp, z, speed, yaw = traj['wp'], traj['z'], traj['speed'], 0.0
        pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=50)
        rate = rospy.Rate(50)
        seg_len = np.linalg.norm(np.diff(wp, axis=0), axis=1)
        rospy.loginfo('[sim_adapter] traj: %d waypoints, 总长 %.1fm @%.2fm/s', len(wp), seg_len.sum(), speed)
        stage, t0 = 'hold_start', rospy.get_rostime()
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            p = wp[0].copy()
            if stage == 'hold_start' and (now - t0).to_sec() >= traj['hold_start']:
                stage, t0 = 'move', now
            elif stage == 'move':
                remain, seg = speed * (now - t0).to_sec(), 0
                for L in seg_len:
                    if remain > L:
                        remain -= L
                        seg += 1
                    else:
                        break
                else:
                    stage, t0 = 'hold_end', now
                if stage == 'move':
                    d = remain / max(seg_len[seg], 1e-9)
                    p = wp[seg] + d * (wp[seg + 1] - wp[seg])
                    dv = wp[seg + 1] - wp[seg]
                    ty = math.atan2(dv[1], dv[0])
                    dy = (ty - yaw + math.pi) % (2 * math.pi) - math.pi
                    yaw += max(-YAW_RATE_MAX / 50.0, min(YAW_RATE_MAX / 50.0, dy))
                if stage == 'hold_end':
                    p = wp[-1].copy()                   # 回起点静置
            om = Odometry()
            om.header.stamp, om.header.frame_id, om.child_frame_id = now, 'world', 'base_link'
            oq = quat_from_yaw(yaw)
            om.pose.pose.position.x, om.pose.pose.position.y, om.pose.pose.position.z = p[0], p[1], z
            om.pose.pose.orientation.x, om.pose.pose.orientation.y = oq[0], oq[1]
            om.pose.pose.orientation.z, om.pose.pose.orientation.w = oq[2], oq[3]
            pub.publish(om)
            rate.sleep()


def selfcheck():
    """静置 5s 管线自检（纯函数级，无需 master）：|acc|∈9.81±0.05 且 |ω|<0.01 → exit 0/1"""
    rate, dur = 50.0, 5.0
    buf = [(k / rate, np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])) for k in range(int(rate * dur))]
    ww_prev, w_max, a_err_max = np.zeros(3), 0.0, 0.0
    for k in range(5, len(buf)):
        ww = world_gyro_w(buf[k][2], buf[k - 1][2], 1.0 / rate)
        if ww is not None: ww_prev = ww
        a_w = sg2_accel(np.array([buf[j][1] for j in range(k - 4, k + 1)]), 1.0 / rate)
        a_err_max = max(a_err_max, abs(float(np.linalg.norm(quat_to_R(buf[k][2]).T @ (a_w - G_VEC))) - 9.81))
        w_max = max(w_max, float(np.linalg.norm(ww_prev)))
    n = 128                                        # 点云重排布局自检（字段偏移/步长/值透传）
    din = np.zeros(n, dtype=DTYPE_IN)
    din['x'], din['y'], din['z'], din['intensity'] = 1, 2, 3, 7
    msg = PointCloud2()
    msg.point_step, msg.data, msg.header.stamp = 32, din.tobytes(), rospy.Time(0)
    out = np.frombuffer(repack_cloud(msg, 's').data, dtype=DTYPE_OUT)
    layout_ok = (out['x'][0] == 1 and out['intensity'][0] == 7 and
                 out['time'][-1] == 0 and out['ring'][33] == 1 and DTYPE_OUT.itemsize == 32)
    din2 = np.zeros(2, dtype=DTYPE_IN)             # 坑13-R1① zcut 自检：配对位姿=单位阵（R 第3行=(0,0,1), pz=0）
    din2['x'], din2['y'], din2['z'] = 0, 0, [-0.5, 0.5]
    msg2 = PointCloud2()
    msg2.point_step, msg2.data = 32, din2.tobytes()
    out2 = np.frombuffer(repack_cloud(msg2, 's', zcut=(np.array([0.0, 0.0, 1.0]), 0.0, Z_MIN_WORLD)).data, dtype=DTYPE_OUT)
    zcut_ok = len(out2) == 1 and out2['z'][0] == 0.5     # 地下 z=-0.5 剔除、地上 z=0.5 保留 → N=1
    ok = a_err_max <= 0.05 and w_max < 0.01 and layout_ok and n == len(out) and zcut_ok
    print('[selfcheck] |acc|误差峰值=%.4f(≤0.05) |w|峰值=%.6f(<0.01) 点云布局=%s zcut剔除=%s → %s'
          % (a_err_max, w_max, layout_ok, zcut_ok, 'PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='adapt', choices=['traj', 'adapt'])
    ap.add_argument('--traj', default='')
    ap.add_argument('--time-unit', dest='time_unit', default='s', choices=['s', 'ms'])
    ap.add_argument('--rate-lidar', dest='rate_lidar', type=float, default=10.0)
    ap.add_argument('--rate-imu', dest='rate_imu', type=float, default=200.0)
    ap.add_argument('--selfcheck', action='store_true')
    opt, _ = ap.parse_known_args()
    if opt.selfcheck:
        raise SystemExit(selfcheck())
    rospy.init_node('sim_adapter')
    if opt.mode == 'traj' and not opt.traj:
        rospy.logerr('[sim_adapter] --mode traj 需 --traj <yaml>')
        raise SystemExit(2)
    ad = SimAdapter(opt)
    rospy.Subscriber('/pcl_render_node/sensor_cloud', PointCloud2, ad.on_cloud, queue_size=2)
    rospy.Subscriber('/quad_0/lidar_pose', Odometry, ad.on_pose, queue_size=50)
    rospy.Timer(rospy.Duration(1.0 / opt.rate_lidar), ad.cloud_timer)
    rospy.Timer(rospy.Duration(ad.dt_imu), ad.imu_timer)
    rospy.loginfo('[sim_adapter] up: mode=%s time-unit=%s rate_lidar=%.0f rate_imu=%.0f',
                  opt.mode, opt.time_unit, opt.rate_lidar, opt.rate_imu)
    if opt.mode == 'traj':
        ad.spin_traj(load_traj(opt.traj))           # 阻塞发轨迹
        rospy.loginfo('[sim_adapter] 轨迹结束，保持常驻（adapt-only）')
    rospy.spin()


if __name__ == '__main__':
    main()
