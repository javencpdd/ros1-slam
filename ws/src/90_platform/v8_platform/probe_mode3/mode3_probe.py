"""V8-03 首切片：SCAN mode3 几何与抽稀数学探针（离线，不启容器、不接运行时）。

忠实复现两段第三方源码的数学面，逐案例对照手算期望值：
  1) scan_replan_fsm.cpp pathCallback（mode3=REFERENCE_PATH）
     —— 每点无条件 z+body_height_；constexpr min_dist=0.5 按 3D 距离抽稀；
        终点与抽稀末点差 >1e-6 时强制回补；body_height 参数缺省 0.0。
  2) dyn_a_star.cpp interpolateZIndexOnSearchPlane
     —— XY 邻居 z 不从地图取，钉在起终点直线投影比例上（ratio 钳 [0,1]）；
        XY 退化（len2<1e-8）取起点 z；邻居只在 x/y 展开，z 只计算不搜索。

本件只复现数学、不做 ROS 行为；结论与案例表见同目录 README.md。
"""
import math
import sys

MIN_DIST = 0.5      # scan_replan_fsm.cpp:378 constexpr，写死不可配
END_EPS = 1e-6      # scan_replan_fsm.cpp:397
LEN2_EPS = 1e-8     # dyn_a_star.cpp:168


def mode3_lift_thin(poses, body_height):
    """逐点 z+body_height_ → 首点必留 → 3D 距离>=0.5 抽稀 → 终点回补。

    poses: [(x, y, z_ground), ...]；返回 waypoint 列表（与 FSM waypoints 同构）。
    """
    end_pt = (poses[-1][0], poses[-1][1], poses[-1][2] + body_height)
    waypoints = []
    last = None
    for px, py, pz in poses:
        wp = (px, py, pz + body_height)
        if last is None or math.dist(wp, last) >= MIN_DIST:
            waypoints.append(wp)
            last = wp
    if math.dist(waypoints[-1], end_pt) > END_EPS:
        waypoints.append(end_pt)
    return waypoints


def search_plane_z(x, y, start_xyz, end_xyz):
    """A* 搜索平面在 (x,y) 处的 z（dyn_a_star.cpp:167-178 的连续化复现）。"""
    sx, sy = start_xyz[0], start_xyz[1]
    ex, ey = end_xyz[0], end_xyz[1]
    len2 = (ex - sx) ** 2 + (ey - sy) ** 2
    if len2 < LEN2_EPS:
        return start_xyz[2]
    dx, dy = ex - sx, ey - sy
    ratio = ((x - sx) * dx + (y - sy) * dy) / len2
    ratio = max(0.0, min(1.0, ratio))
    return start_xyz[2] + ratio * (end_xyz[2] - start_xyz[2])


CASES = []


def case(name, got, want, note=''):
    ok = all(abs(g - w) < 1e-9 for g, w in zip(got, want)) and len(got) == len(want)
    CASES.append((name, ok))
    print('%-46s %s  got=%s want=%s' % (name, 'PASS' if ok else 'FAIL', got, want))
    return ok


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


print('== A. 高度相加（body_height 抬升，FSM :372-398） ==')

# A1 正例：地面系输入 + bh=0.28 → 单次抬升
w = mode3_lift_thin([(0.0, 0, 0.0), (0.6, 0, 0.0)], 0.28)
case('A1 单次抬升(0+0.28)', [v for p in w for v in p],
     [0.0, 0, 0.28, 0.6, 0, 0.28])

# A2 反例：上游若发体心系 z(0.28) 再加 bh → 双重补偿 0.56
w = mode3_lift_thin([(1.0, 0, 0.28)], 0.28)
case('A2 双重补偿(0.28+0.28=0.56)', [w[0][2]], [0.56])

# A3 缺省风险：grid_map/body_height 缺省 0.0 → 无抬升（静默）
w = mode3_lift_thin([(1.0, 0, 0.0)], 0.0)
case('A3 参数缺省 bh=0 无抬升', [w[0][2]], [0.0])

print()
print('== B. 关键点抽稀（min_dist=0.5 写死，3D 距离） ==')

# B1 正例：平地 0..2.0 步 0.1 → 保留 {0, 0.5, 1.0, 1.5, 2.0}
poses = [(0.1 * i, 0, 0.0) for i in range(21)]
w = mode3_lift_thin(poses, 0.0)
case('B1 平地等距抽稀', [p[0] for p in w], [0.0, 0.5, 1.0, 1.5, 2.0])

# B2 反例（3-4-5 短坡道）：地面 0..0.3，坡道 run0.3/rise0.4 至 (0.6,0.4)，
# 夹层 0.7..1.0 全在 0.4。坡道中点 (0.45,0.2) 距首点 sqrt(.45²+.2²)=0.4924<0.5 被吞；
# 保留 (0,0,0)→(0.6,0,0.4) 直弦，坡脚 x=0.3 处弦高 0.2 vs 实际地表 0.0，偏差 0.2 m。
poses = ([(0.1 * i, 0, 0.0) for i in range(4)]
         + [(0.45, 0, 0.2), (0.60, 0, 0.4)]
         + [(0.7 + 0.1 * i, 0, 0.4) for i in range(4)])
w = mode3_lift_thin(poses, 0.0)
case('B2 3-4-5 坡道吞点(保留点集)', [v for p in w for v in p],
     [0.0, 0, 0.0, 0.6, 0, 0.4, 1.0, 0, 0.4])
chord_z_at_foot = 0.4 * (0.3 / 0.6)          # 弦在坡脚 x=0.3 的高度
case('B2 弦@坡脚 vs 地表(偏差0.2)', [chord_z_at_foot - 0.0], [0.2])
chord_z_at_mid = 0.4 * (0.45 / 0.6)          # 弦在坡道中点 x=0.45
case('B2 弦@坡道中点 vs 0.2(偏差0.1)', [chord_z_at_mid - 0.2], [0.1])

# B3 正例：末点被抽稀吞掉但终点强制回补，终点高度精确
w = mode3_lift_thin([(0.0, 0, 0.0), (0.6, 0, 0.0), (0.9, 0, 0.0)], 0.28)
case('B3 终点回补(0.9,0,0.28)精确', [v for p in w for v in p],
     [0.0, 0, 0.28, 0.6, 0, 0.28, 0.9, 0, 0.28])

print()
print('== C. 转层判定（A* 搜索平面插值，dyn_a_star :167-178） ==')

# C1 正例：单调坡 start(0,0,0.28)→end(4,0,0.88)，z(x)=0.28+0.15x 与坡面体心一致
start, end = (0.0, 0.0, 0.28), (4.0, 0.0, 0.88)
case('C1 单调坡 z(2)=0.58', [search_plane_z(2.0, 0.0, start, end)], [0.58])

# C2 反例（同高天桥）：起终同高 → 平面恒 0.28，x=4 处需 0.88 不可表达
start, end = (0.0, 0.0, 0.28), (8.0, 0.0, 0.28)
case('C2 天桥 z(4)=0.28≠0.88(Δ0.6)', [search_plane_z(4.0, 0.0, start, end) - 0.88], [-0.6])

# C3 反例（同 XY 双层列）：start(0,0,0.28)→end(4,0,0.88)，x=2 列有地面层与
# 板体 [0.5,0.6]（板面体心 0.88）；插值 z(2)=0.58 落进板体 → 幻影中层层被占，
# 两层各自可自由通行也无济于事。
start, end = (0.0, 0.0, 0.28), (4.0, 0.0, 0.88)
z2 = search_plane_z(2.0, 0.0, start, end)
case('C3 双层列 z(2)=0.58∈[0.5,0.6]', [1.0 if 0.5 <= z2 <= 0.6 else 0.0], [1.0])

# C4 边界（垂直退化）：起终同 XY → len2<1e-8 → 平面 z 恒为起点 0.28，纯爬升不可表达
start, end = (2.0, 0.0, 0.28), (2.0, 0.0, 0.88)
case('C4 垂直退化 z≡0.28', [search_plane_z(2.5, 0.5, start, end)], [0.28])

# C5 边界（垂直投影钳位）：侧向偏移点 ratio 钳 0 → 取起点侧 z
start, end = (0.0, 0.0, 0.28), (4.0, 0.0, 0.88)
case('C5 侧向点 z 取线段投影(带钳位)',
     [search_plane_z(-1.0, 3.0, start, end), search_plane_z(6.0, -2.0, start, end)],
     [0.28, 0.88])

print()
print('== D. 非支撑面（两段数学均无支撑/未知语义） ==')

# D1：未知格上方的输入点照常抬升保留，无任何支撑标记或拒绝
w = mode3_lift_thin([(0.0, 0, 0.0), (0.7, 0, 0.0)], 0.28)   # (0.35,0) 设为 unknown
case('D1 未知格上方点无拒绝', [v for p in w for v in p],
     [0.0, 0, 0.28, 0.7, 0, 0.28])

print()
failed = [n for n, ok in CASES if not ok]
print('—' * 60)
print('%d 例，通过 %d，失败 %d' % (len(CASES), len(CASES) - len(failed), len(failed)))
sys.exit(1 if failed else 0)
