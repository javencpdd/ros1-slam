#!/usr/bin/env python3
# x2_pub_two_frame.py —— X2 四格窗包的**两帧测试图发布器**（R332 建；＃166 工单格② 的"第二图首发是否进层"判据面）
# 来历＝r59 探针的 pub.py 每 0.2s 重发同一份图 ⇒ 测不出 first_map_only 的语义（在册缺口之一）。
# 本件改成：开机发**第一帧**、只发一次（latch）；隔 X2_GAP 秒再发**第二帧**（新增一格区 80），此后不再发。
# ★三块测试图面（40x40、res 0.1、origin -2.0 ⇒ 1600 格）：
#   A 图：(10,10..14)=100 必杀带｜(20,20)=50 A 独占点｜其余 0
#   B 帧1：(10,10..14)=50 ★与 A 的 100 同格＝判别区｜(12,12..14)=100 B 独占 lethal 区｜(30,0..4)=-1 未知面｜其余 0
#   B 帧2：帧1 全部保留 ＋ (25,25..29)=100 ★第二帧新置区（读回 100⇒后续图进层／读回 0⇒只吃首帧）
# ★★R332 第二扇改形的缘由（第一扇六段跑出来才发现的在册参数面事实）＝`trinary_costmap: true` ＋`lethal_cost_threshold: 100`
#   把**一切低于 100 的代价抹成 0**（第一扇现跑：B 独占区投 70 读回 0、A 独占点投 50 读回 0、第二帧投 80 读回 0）
#   ⇒ 第一版的 50／70／80 三处**全部不可测**；本版把需要"看得见"的格子一律改成 **100**，只保留 A 的 (20,20)=50
#   作为**三量化抹零本身的见证格**（读回 0＝抹零成立）。
# 用法：X2_GAP=6 X2_RUN=/tmp/x2r python3 x2_pub_two_frame.py
import os
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

W = H = 40
RES = 0.1
ORG = -2.0
GAP = float(os.environ.get("X2_GAP", "6"))
TAG = os.environ.get("X2_TAG", "g1")


def mk(frame, vals):
    m = OccupancyGrid()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.info.resolution = RES
    m.info.width = W
    m.info.height = H
    m.info.origin.position.x = ORG
    m.info.origin.position.y = ORG
    m.data = [int(v) for v in vals]
    return m


def idx(r, c):
    return r * W + c


def build_a():
    a = np.zeros(W * H, dtype=np.int8)
    for c in range(10, 15):
        a[idx(10, c)] = 100          # 必杀带（与 B 的 50 同格＝判别区）
    a[idx(20, 20)] = 50              # A 独占点
    return a


def build_b(second):
    b = np.zeros(W * H, dtype=np.int8)
    for c in range(10, 15):
        b[idx(10, c)] = 50           # 判别区（B 侧值）
    for c in range(12, 15):
        b[idx(12, c)] = 100          # B 独占 lethal 区（★原 70 会被 trinary 抹成 0，见头注）
    for c in range(0, 5):
        b[idx(30, c)] = -1           # 未知面
    if second:
        for c in range(25, 30):
            b[idx(25, c)] = 100      # 第二帧新置区（★原 80 会被 trinary 抹成 0）
    return b


def main():
    rospy.init_node("x2_pub_%s" % TAG)
    a = build_a()
    pa = rospy.Publisher("/pmap_a", OccupancyGrid, queue_size=1, latch=True)
    pb = rospy.Publisher("/pmap_b", OccupancyGrid, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pa.publish(mk("map", a))
    pb.publish(mk("map", build_b(False)))
    rospy.loginfo("X2_PUB first frame sent tag=%s", TAG)
    rospy.sleep(GAP)
    pb.publish(mk("map", build_b(True)))
    rospy.loginfo("X2_PUB second frame sent tag=%s gap=%.1f", TAG, GAP)
    rospy.spin()


if __name__ == "__main__":
    main()
