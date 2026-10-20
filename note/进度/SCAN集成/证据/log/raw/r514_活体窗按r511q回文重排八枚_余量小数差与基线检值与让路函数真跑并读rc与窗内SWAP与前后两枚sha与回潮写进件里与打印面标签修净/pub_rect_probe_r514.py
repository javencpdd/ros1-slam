#!/usr/bin/env python3
# pub_rect_probe_r514.py —— ★C7-B(a-R) 的靶＝世界坐标★单块矩形★（不再用机器人相对带）
#   被试体具名（★判据面 R465q 要求的三项＝绝对列／行区间＋n_T＋与对照态的索引平移量★）＝
#     A 规范格点 列 476..534 × 行 523..543 ⇒ 世界格心 x∈[-2.35,3.45]、y∈[2.35,4.35]（外沿 x∈[-2.40,3.50]、y∈[2.30,4.40]）
#     n_T = 59 × 21 = ★1239 格★（★留底 P100 下 X_A=118、0.1n_T=123.9、余量 5.9 格＝11 枚合规块里余量最大的一枚★）
#   选这一枚的理由（施工侧自陈）＝★判据面穷举给的"面积≥1200 且合规"里余量最大者★，比 1475 那枚（余量 2.5）扛得住更大的 A 态漂移
#   地图几何沿用 /map（同尺寸同分辨率同原点同坐标系）；矩形内＝100（输入侧 ≥ lethal_cost_threshold:100 ⇒ 层内记 254），其余＝0
import os
import sys

import rospy, time
import tf
from nav_msgs.msg import OccupancyGrid

X0, X1 = -2.40, 3.50
Y0, Y1 = 2.30, 4.40
N_EXPECT = 1239
LATCH_S = float(sys.argv[1]) if len(sys.argv) > 1 else 170.0


def bye(sig, frm):
    rospy.signal_shutdown("SIGINT from window")


def main():
    import signal
    signal.signal(signal.SIGINT, bye)
    rospy.init_node("rect_probe_r514")
    pub = rospy.Publisher("/elev_map", OccupancyGrid, queue_size=1, latch=True)
    try:
        base = rospy.wait_for_message("/map", OccupancyGrid, timeout=60)
    except rospy.ROSException:
        print("SELFREPORT FAIL: /map 60s 未到 ⇒ C 态 INVALID")
        sys.exit(3)
    w, h = int(base.info.width), int(base.info.height)
    res = float(base.info.resolution)
    ox, oy = float(base.info.origin.position.x), float(base.info.origin.position.y)
    data = [0] * (w * h)
    painted = 0
    cols, rows = [], []
    for r in range(h):
        cy = oy + (r + 0.5) * res
        if Y0 <= cy < Y1:
            rows.append(r)
    for c in range(w):
        cx = ox + (c + 0.5) * res
        if X0 <= cx < X1:
            cols.append(c)
    for r in rows:
        for c in cols:
            data[r * w + c] = 100
            painted += 1
    listener = tf.TransformListener()
    rx = ry = None
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and rx is None and (rospy.Time.now() - t0).to_sec() < 20:
        try:
            tr, _rot = listener.lookupTransform("map", "base_link", rospy.Time(0))
            rx, ry = tr[0], tr[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.2)
    if rx is None:
        print("SELFREPORT FAIL: TF map→base_link 20s 不可查 ⇒ C 态 INVALID（★robot= 取不到就不自报★）")
        sys.exit(3)
    lethal_recount = sum(1 for v in data if v == 100)   # ★独立来源二＝对已发布数组逐格重数★
    m = OccupancyGrid()
    m.header.frame_id = base.header.frame_id
    m.header.stamp = rospy.Time.now()
    m.info = base.info
    m.data = data
    pub.publish(m)
    print("SELFREPORT topic=/elev_map size=%dx%d res=%.3f frame=%s origin=(%.2f,%.2f) rect_world x∈[%.2f,%.2f) y∈[%.2f,%.2f) 图层列 %d..%d × 行 %d..%d robot=(%.2f,%.2f) band=(%.2f,%.2f) free=%d lethal=%d unknown=0 total=%d painted=%d lethal_重数=%d" % (
        w, h, res, m.header.frame_id, ox, oy, X0, X1, Y0, Y1,
        cols[0], cols[-1], rows[0], rows[-1], rx, ry, X0, X1,
        w * h - painted, lethal_recount, w * h, painted, lethal_recount))
    print("SELFREPORT 具名声明＝本窗以 lethal= 承担 r464q 第 56 行在册模板那一位；★两枚算式＝图层计数 painted 与 发布数组逐格重数 lethal_重数、同出一枚 data ⇒ 二者★不构成★两来源（★这句的断言力按 r499q Q4② 就地消灭、只留下面那条自陈★）；★robot= 由 TF 现查、band= ＝所用世界 x 区间 ⇒ 模板缺的两枚本窗补齐★")
    print("SELFREPORT ★两枚算式核对（同源两派生＝同一枚 data 的增量计数与逐格重数 ⇒ 在册意义上的独立来源仍未合、本窗自记缺项★）：painted=%d ‖ lethal_重数=%d ‖ ★两枚算式同值＝%s★（★R492 那版两个字段吃同一变量＝恒真、本窗改法★）" % (
        painted, lethal_recount, "成立" if painted == lethal_recount else "★不成立★"))
    print("SELFREPORT N 比对：n_T 期望=%d ‖ painted 实际=%d ‖ n_T==N ⇒ %s ‖ 面积下限 IV（≥1200）⇒ %s ‖ 索引平移量：图层列 j ↔ A 规范列 c 满足 c = j + 440" % (
        N_EXPECT, painted, "★成立★" if painted == N_EXPECT else "★不成立★",
        "★达标★" if painted >= 1200 else "★不达标★"))
    sys.stdout.flush()
    t1 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < LATCH_S:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.5)
    print("SELFREPORT 退出（latch 保活到点或被 SIGINT）")
    sys.stdout.flush()


main()

# ★丁3＝读数件自身带〈第几遍／共几遍〉（r496q L-O：窗志那一枚不算、件自身必须带）★
try:
    _rp = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".runs")
    _rn = open(_rp, encoding="utf-8").read().strip()
except Exception as e:
    _rp, _rn = "★读 .runs 失败＝%s：%s★" % (type(e).__name__, e), "★缺★"
print("SELFREPORT ★遍次（件自身带）＝本遍＝第 %s 遍 ‖ 共 %s 遍（★共几遍＝截至本遍已起过的窗数、不是计划总数★） ‖ 出处＝%s ‖ 本件时刻＝%s（★容器面 UTC；宿主 CST＝+8★）" % (
    _rn, _rn, _rp, time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())))

