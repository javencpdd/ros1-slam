#!/usr/bin/env python3
"""点云累积器：订阅 /cloud_registered，体素降采样累积，可保存 PCD。

为什么需要它：faster_lio 只发当前帧（无累积地图接口），rviz 的 Decay 也只是
视觉叠加、进程一停就没。要拿到可交付的地图文件，必须在外部累积。

落盘路径刻意避开 data/maps/lite3.pcd（项目红线：sim 跑批不得覆盖样张）。
"""
import os
import sys
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

VOXEL = 0.05            # 体素边长（m）
OUT = os.environ.get('ACC_OUT', '/opt/ros1-slam/data/maps/sim_accumulated.pcd')
SAVE_EVERY = 300        # 每 N 帧存一次（防中途丢失）
MAX_VOXELS = 1500000    # ★上限★：约 1.5 个 0.05m 体素（≈150MB 内存）。
                        # 无上限时长跑会一直涨（实测 22 分钟已 106MB 且线性增长）。
                        # 触顶后停止接纳新体素并告警——宁可地图不再变密，也不让进程被 OOM 杀掉。

voxels = {}             # (i,j,k) -> [x,y,z] 累加值
frames = [0]


def cb(msg):
    pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
    full = len(voxels) >= MAX_VOXELS
    if full and frames[0] % 600 == 0:
        print(f'[acc] 已达体素上限 {MAX_VOXELS}，停止接纳新点（地图不再变密）', flush=True)
    for x, y, z in pts:
        if full:
            break
        if not np.isfinite([x, y, z]).all():
            continue
        k = (int(np.floor(x / VOXEL)), int(np.floor(y / VOXEL)), int(np.floor(z / VOXEL)))
        if k not in voxels:
            voxels[k] = [x, y, z]
    frames[0] += 1
    if frames[0] % SAVE_EVERY == 0:
        save()


def save():
    if not voxels:
        return
    arr = np.array(list(voxels.values()), dtype=np.float32)
    # ★写 binary 不写 ascii★：项目自带的 regression/pcd_util.read_pcd 只解析
    # `DATA binary`（用 np.frombuffer）。原先写 ascii 时，用项目工具读出来
    # x 全是 0.00（把文本当二进制解），等于产出了一个"别人读不了的文件"。
    header = ('# .PCD v0.7 - Point Cloud Data file format\n'
              'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              f'COUNT 1 1 1\nWIDTH {len(arr)}\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(arr)}\nDATA binary\n')
    with open(OUT, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())
    print(f'[acc] saved {len(arr)} pts -> {OUT}', flush=True)


def main():
    rospy.init_node('cloud_accumulator', anonymous=False)
    rospy.Subscriber('/cloud_registered', PointCloud2, cb, queue_size=5)
    rospy.on_shutdown(save)
    print(f'[acc] 订阅 /cloud_registered，体素 {VOXEL}m，输出 {OUT}', flush=True)
    rospy.spin()


if __name__ == '__main__':
    main()
