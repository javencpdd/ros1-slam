#!/usr/bin/env python3
"""地形/坐标 A/B 探针（2026-11-26 P0 重写）：**同一帧、同一时刻的 TF、逐点对照**。

为什么重写（旧版三个致命缺陷，直接导致 2026-11-25 的错误结论）：
  ① `raw` 与 `terrain` 是**两份不同的消息**（各自 wait_for_message）——比的是两帧不同点云；
  ② TF 用 `rospy.Time(0)`（最新）而不是**这帧点云的戳**，运动时等于拿未来的变换去搬过去的点；
  ③ 只比分位数（median/分位），不逐点对照 ⇒ "两条分布相同"被当成"逐点一致"。

现在的做法：
  · 取**一帧** `/rslidar_points`，记下 `header.stamp` 与 `frame_id`；
  · 用**该戳**查 `target←source` 变换（查不到就退到最新并把 `tf_at` 标注为 `latest`，读数里写明）；
  · 变换后与**同戳**的 `/terrain_cloud`（容差 `--stamp-tol`）逐点对照，给出体素命中率；
  · 同时打印每一跳（map→odom→base_link→rslidar）的平移/欧拉角与该跳的时间戳来源；
  · 地面平面用 RANSAC 拟合，报**传感器到地面垂距**与**相对传感器 z 轴的倾角**——
    这两个量与安装外参直接对应（当前场景应为 0.45 m / 15°），比"z 中位"稳健得多
    （零值假点会直接把中位数按到 0，2026-11-25 就是这么误判成"点云已被压平"的）。

用法：
  terrain_tf_ab_probe.py                       # 默认 map←rslidar
  terrain_tf_ab_probe.py --source rslidar --target map --terrain-topic /terrain_cloud
"""
import argparse
import json

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def read_xyz(msg):
    return np.array([[p[0], p[1], p[2]] for p in
                     pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                    dtype=np.float64)


def quat_to_R(q):
    x, y, z, w = (float(v) for v in q)
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_deg(R):
    return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
            np.degrees(np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))),
            np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def ransac_ground(P, thresh=0.03, iters=3000, seed=0):
    """返回 (内点数, 单位法向, 原点到平面距离)。只接受接近水平的候选。"""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        p0, p1, p2 = P[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        if abs(n[2]) < 0.85:
            continue
        c = -n @ p0
        inl = int((np.abs(P @ n + c) < thresh).sum())
        if best is None or inl > best[0]:
            best = (inl, n, c)
    if best is None:
        return 0, None, None
    inl, n, c = best
    if n[2] < 0:
        n, c = -n, -c
    return inl, n, abs(c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='rslidar')
    ap.add_argument('--target', default='map')
    ap.add_argument('--cloud-topic', default='/rslidar_points')
    ap.add_argument('--terrain-topic', default='/terrain_cloud')
    ap.add_argument('--allow-latest', action='store_true',
                    help='允许 TF 退到最新（默认禁止：退到最新就等于拿未来的变换搬过去的点）')
    ap.add_argument('--expect-z', type=float, default=0.45)
    ap.add_argument('--expect-tol', type=float, default=0.05)
    ap.add_argument('--no-terrain', action='store_true')
    args = ap.parse_args()

    rospy.init_node('terrain_ab_probe', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    raw = rospy.wait_for_message(args.cloud_topic, PointCloud2, timeout=20)
    stamp = raw.header.stamp
    P = read_xyz(raw)
    print('[raw] %s frame=%s stamp=%.3f n=%d' % (args.cloud_topic, raw.header.frame_id,
                                                 stamp.to_sec(), len(P)))
    r = np.linalg.norm(P, axis=1)
    print('[raw] 精确零点 %d (%.1f%%)；r 分位 1%%=%.2f 50%%=%.2f 99%%=%.2f'
          % (int((r < 1e-6).sum()), 100 * (r < 1e-6).mean(),
             *np.percentile(r, [1, 50, 99])))

    inl, n, dist = ransac_ground(P)
    if n is not None:
        print('[raw] RANSAC 地面: 内点 %d (%.1f%%) 相对传感器 z 轴倾角 %.2f° 传感器到地面垂距 %.3f m'
              % (inl, 100 * inl / len(P), np.degrees(np.arccos(min(1.0, abs(n[2])))), dist))

    # ---- 每一跳 ----
    for a, b in ((args.target, 'odom'), ('odom', 'base_link'), ('base_link', args.source)):
        try:
            tr = buf.lookup_transform(a, b, stamp, rospy.Duration(0.5))
            at = 'stamp'
        except Exception:
            tr = buf.lookup_transform(a, b, rospy.Time(0), rospy.Duration(1.0))
            at = 'latest'
        R = quat_to_R((tr.transform.rotation.x, tr.transform.rotation.y,
                       tr.transform.rotation.z, tr.transform.rotation.w))
        t = tr.transform.translation
        print('[hop] %-18s t=(%.3f,%.3f,%.3f) rpy=(%.2f,%.2f,%.2f) tf_at=%s'
              % ('%s<-%s' % (a, b), t.x, t.y, t.z, *rpy_deg(R), at))

    # ---- 严格用本帧戳的变换 ----
    try:
        tr = buf.lookup_transform(args.target, args.source, stamp, rospy.Duration(1.0))
        tf_at = 'stamp'
    except Exception as error:
        if not args.allow_latest:
            print('[tf] 该戳查不到（%.3f）：%s' % (stamp.to_sec(), str(error)[:120]))
            print('RESULT ' + json.dumps({'verdict': 'FAIL', 'reason': 'tf_not_at_stamp'},
                                         ensure_ascii=False))
            raise SystemExit(2)
        tr = buf.lookup_transform(args.target, args.source, rospy.Time(0), rospy.Duration(2.0))
        tf_at = 'latest'
        print('[tf] 该戳查不到，退到最新（--allow-latest）：%s' % str(error)[:120])
    R = quat_to_R((tr.transform.rotation.x, tr.transform.rotation.y,
                   tr.transform.rotation.z, tr.transform.rotation.w))
    t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                  tr.transform.translation.z])
    # ★复合一致性★：直查 map←rslidar 与 map←odom∘odom←base_link∘base_link←rslidar 必须逐元素相等
    chain = np.eye(4)
    for a, b in ((args.target, 'odom'), ('odom', 'base_link'), ('base_link', args.source)):
        h = buf.lookup_transform(a, b, stamp, rospy.Duration(0.5))
        H = np.eye(4)
        H[:3, :3] = quat_to_R((h.transform.rotation.x, h.transform.rotation.y,
                               h.transform.rotation.z, h.transform.rotation.w))
        H[:3, 3] = [h.transform.translation.x, h.transform.translation.y,
                    h.transform.translation.z]
        chain = chain @ H
    T_direct = np.eye(4)
    T_direct[:3, :3], T_direct[:3, 3] = R, t
    dt = float(np.abs(chain[:3, 3] - t).max())
    dR = float(np.degrees(np.arccos(min(1.0, (np.trace(chain[:3, :3].T @ R) - 1) / 2))))
    print('[compose] 直查 vs 链式：平移最大差 %.3g m，旋转最大差 %.3g°' % (dt, dR))
    M = P @ R.T + t
    print('[map] %s<-%s t=(%.3f,%.3f,%.3f) rpy=(%.2f,%.2f,%.2f) tf_at=%s'
          % (args.target, args.source, *t, *rpy_deg(R), tf_at))
    zg = M[:, 2]
    print('[map] z 分位 1%%=%.2f 50%%=%.2f 99%%=%.2f  z<0.1 占 %.1f%%'
          % (*np.percentile(zg, [1, 50, 99]), 100 * (zg < 0.1).mean()))

    ginl, gn, gz0 = ransac_ground(M)
    gtilt = None
    if gn is not None:
        gtilt = float(np.degrees(np.arccos(min(1.0, abs(gn[2])))))
        print('[map] 地面 RANSAC: 内点 %d (%.1f%%) 倾角 %.2f° 平面 z0=%.3f'
              % (ginl, 100 * ginl / len(M), gtilt, gz0))

    checks = {'tf_at_stamp': tf_at == 'stamp',
              'compose_consistent': dt <= 1e-3 and dR <= 5e-2,
              'map_rslidar_z': abs(float(t[2]) - args.expect_z) <= args.expect_tol,
              'map_ground_level': (gtilt is not None and gtilt <= 1.0
                                   and abs(gz0) <= 0.05)}
    verdict = 'PASS' if all(checks.values()) else 'FAIL'
    print('[verdict] %s  %s' % (verdict, ' '.join('%s=%s' % (k, v) for k, v in checks.items())))

    result = {'verdict': verdict, 'checks': checks,
              'compose_dt_m': dt, 'compose_dr_deg': dR,
              'map_ground_tilt_deg': gtilt, 'map_ground_z0': None if gz0 is None else float(gz0),
              'raw_frame': raw.header.frame_id, 'stamp': stamp.to_sec(),
              'n_raw': int(len(P)), 'tf_at': tf_at,
              'map_to_source_t': t.tolist(), 'map_to_source_rpy_deg': list(rpy_deg(R)),
              'map_z_percentiles': [float(v) for v in np.percentile(zg, [1, 50, 99])],
              'map_z_below_0p1_frac': float((zg < 0.1).mean()),
              'ground_inliers': int(inl),
              'ground_tilt_deg': (float(np.degrees(np.arccos(min(1.0, abs(n[2])))))
                                  if n is not None else None),
              'sensor_to_ground_m': float(dist) if dist is not None else None}

    if not args.no_terrain:
        terrain = None
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            cand = rospy.wait_for_message(args.terrain_topic, PointCloud2, timeout=8.0)
            if cand.header.stamp == stamp:      # ★帧身份必须精确相等，不再用秒级容差★
                terrain = cand
                break
            terrain = terrain or cand
        if terrain is not None:
            T = read_xyz(terrain)
            print('[terrain] %s frame=%s n=%d 与 raw 戳差 %.3f s'
                  % (args.terrain_topic, terrain.header.frame_id, len(T),
                     abs(terrain.header.stamp.to_sec() - stamp.to_sec())))
            if len(T) and len(M):
                sub = T[::max(1, len(T) // 4000)]
                key = lambda A: np.round(A / 0.25).astype(np.int64)
                K = set(map(tuple, key(M)))
                hit = sum(1 for k in key(sub) if tuple(k) in K)
                same = terrain.header.stamp.to_sec() == stamp.to_sec()
                print('[terrain] 帧身份：与 raw 同戳=%s' % same)
                result['checks']['terrain_same_frame'] = bool(same)
                if not same:
                    result['verdict'] = 'FAIL'
                else:
                    # ★同帧才做逐点残差★：地形云就是这一帧点的变换结果，
                    # 残差只应反映"far_node 用的那份 TF"与"本探针按戳查到的 TF"之间的差。
                    q = sub[::max(1, len(sub) // 800)]
                    d = np.array([np.min(np.linalg.norm(M - p0, axis=1)) for p0 in q])
                    med, p95 = float(np.median(d)), float(np.percentile(d, 95))
                    print('[terrain] 与变换后原始点的最近邻残差：中位 %.3f m、95%% %.3f m'
                          % (med, p95))
                    result['terrain_residual_median_m'] = med
                    result['terrain_residual_p95_m'] = p95
                    result['checks']['terrain_residual_small'] = med <= 0.05
                    if med > 0.05:
                        result['verdict'] = 'FAIL'

    print('RESULT ' + json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
