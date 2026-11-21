#!/bin/bash
# elevation_probe.sh —— 高程生产者／合并层的正反控验活脚本（B 线，真 2026-10-22 裁「丙」配套）
#
# 三件事，各一条命令：
#   rosrun lite3_costmap_layers elevation_probe.sh stats     # 订阅 GridMap，按合并层判据统计（人读）
#   rosrun lite3_costmap_layers elevation_probe.sh positive  # 正控：前方有台阶 ⇒ 应有格过阈（rc=0 才算过）
#   rosrun lite3_costmap_layers elevation_probe.sh negative  # 反控：空白景 ⇒ 应无格过阈（rc=0 才算过）
#
# ★为什么要一个"消费侧"脚本★：话题上有消息 ≠ 合并层能读进去。本脚本复刻合并层的三条读法
# （层名在 layers 里、按 column_index/row_index 维度还原矩阵、h > ground_z + height_threshold），
# 任一条不成立立刻 rc=1。这样"链路通"是被证明的，不是被相信的。
#
# 依赖：只需 rospy + numpy（不需 grid_map_ros 的 Python 绑定，本镜像也没有）。
set -u
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}
MODE=${1:-stats}

exec python3 -u - "$MODE" <<'PYEOF'
import sys
import numpy as np
import rospy
from grid_map_msgs.msg import GridMap

MODE = sys.argv[1]

# 与合并层 yaml 逐项对表（config/elevation_merged_layer.yaml）
IN_TOPIC = "/elevation_mapping/elevation_map"
LAYER = "elevation"
H_TH = 0.15
G_Z = 0.0
RES = 0.1

rospy.init_node("elevation_probe", anonymous=True)
box = {}


def cb(msg):
    box["m"] = msg


rospy.Subscriber(IN_TOPIC, GridMap, cb, queue_size=1)
t0 = rospy.Time.now()
while "m" not in box and (rospy.Time.now() - t0).to_sec() < 20.0:
    rospy.sleep(0.2)
if "m" not in box:
    print("FAIL: 20s 内没收到 GridMap（生产者没起来，或话题名不对）")
    sys.exit(1)

m = box["m"]
if LAYER not in list(m.layers):
    print("FAIL: 层名 %s 不在 layers=%s 里（合并层会走 ERROR 分支，一格都不写）" % (LAYER, list(m.layers)))
    sys.exit(1)

idx = list(m.layers).index(LAYER)
arr = m.data[idx]
labels = [d.label for d in arr.layout.dim]
sizes = [d.size for d in arr.layout.dim]
# 列主序：dim[0]='column_index'（列数）、dim[1]='row_index'（行数）
if labels[0] != "column_index":
    print("FAIL: dim[0].label=%s ≠ column_index（存储序与 grid_map::Matrix 不一致）" % labels[0])
    sys.exit(1)
ncols, nrows = sizes[0], sizes[1]
data = np.array(arr.data, dtype=np.float32)
if data.size != ncols * nrows:
    print("FAIL: 扁平数据 %d ≠ 列×行 %d" % (data.size, ncols * nrows))
    sys.exit(1)
mat = data.reshape(ncols, nrows).T
finite = np.isfinite(mat)
hv = mat[finite]

# ---- 统计口径（同一条读法，三种模式共用）----
n_valid = int(finite.sum())
h_min = float(hv.min()) if hv.size else float("nan")
h_max = float(hv.max()) if hv.size else float("nan")
h_mean = float(hv.mean()) if hv.size else float("nan")
n_obst = int((hv > G_Z + H_TH).sum()) if hv.size else 0
res_ok = abs(m.info.resolution - RES) < 1e-6

print("frame      : %s" % m.info.header.frame_id)
print("resolution : %.3f（与合并层 yaml 的 %.1f %s）" % (m.info.resolution, RES, "一致" if res_ok else "★不一致★"))
print("window     : %.2f x %.2f m = %d x %d 格" % (m.info.length_x, m.info.length_y, ncols, nrows))
print("center     : (%.3f, %.3f)" % (m.info.pose.position.x, m.info.pose.position.y))
print("valid cells: %d / %d" % (n_valid, mat.size))
print("h          : min=%.4f max=%.4f mean=%.4f" % (h_min, h_max, h_mean))
print("h > %.2f     : %d 格（合并层会写 %d 格致命代价）" % (G_Z + H_TH, n_obst, n_obst))

if MODE == "stats":
    sys.exit(0)

if MODE == "positive":
    # 正控：站在台阶前 ⇒ 地面必须归零、且必须有格过阈
    ok = (abs(h_min) < 0.10) and (n_obst > 0) and res_ok
    if abs(h_min) >= 0.10:
        print("FAIL(正控): h_min=%.3f 偏离 0 超过 0.10 ⇒ 地面没归零（未命中填充点没剔？）" % h_min)
    if n_obst == 0:
        print("FAIL(正控): 没有任何格过 %.2f 阈值 ⇒ 台阶读不出来（或车还没开到台阶前）" % (G_Z + H_TH))
    print("RESULT(正控):", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)

if MODE == "negative":
    # 反控：空白景（只有 ground_plane）⇒ 必须地面归零且**一格都不过阈**
    ok = (abs(h_min) < 0.10) and (abs(h_max) < 0.10) and (n_obst == 0)
    if n_obst != 0:
        print("FAIL(反控): 空白景里有 %d 格过阈 ⇒ 生产者把平地读成了障碍" % n_obst)
    if abs(h_max) >= 0.10:
        print("FAIL(反控): 空白景 h_max=%.3f，平地不该有高度" % h_max)
    print("RESULT(反控):", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)

print("未知模式 %s（只支持 stats / positive / negative）" % MODE)
sys.exit(2)
PYEOF
