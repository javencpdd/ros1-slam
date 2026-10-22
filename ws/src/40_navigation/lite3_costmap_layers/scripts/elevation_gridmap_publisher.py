#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# elevation_gridmap_publisher.py —— 高程 2.5D 网格最小生产者（B 线，真人 2026-10-22 裁「优化零＝勾丙」落地件）
#
# ══════════════════════════════════════════════════════════════════════════════════════════
# ★本节点在整条链路里干什么★
#   仿真点云（/rslidar_points）→ 滚动窗 xy 栅格化 → 每格 2.5D 高程 → grid_map_msgs/GridMap
#   → 发到 /elevation_mapping/elevation_map，供 lite3_costmap_layers 的 ElevationMergedLayer 消费。
#
# ★★ h 的语义（本节点自己定义，此后由我方持有，不再"取决于上游某个包"）★★
#   h ＝ 相对局部地面的离地高，单位：米·正上。
#   亦即：h = z_点 − z_局部地面，同一格里逐点取 h 的最大值。地面本身 h ≡ 0。
#   已与合并层 yaml 逐项对表（本包 config/elevation_merged_layer.yaml）：
#     层名        layer_name    = "elevation"   ← 与本节点 layers[0] 逐字相同
#     分辨率      resolution    = 0.1           ← 与本节点 grid_resolution 缺省逐字相同
#     话题        input_topic   = /elevation_mapping/elevation_map
#     判据        h > ground_z + height_threshold 判为障碍（那两枚是消费侧参数，不在本节点）
#   ★为什么必须这么钉★：R569 §三 优化一指出「h 语义取决于生产者」是挂载前置的最大不确定——
#   若上游 elevation_mapping 发的是绝对高度，h 的零点就随雷达架高漂；若发的是离地高，零点固定。
#   我们自己产，就自己定：零点＝局部地面，与雷达架高、下倾角、机器人俯仰全部解耦。
#
# ★局部地面怎么求（正反控能证伪，见 --selftest）★
#   在滚动窗内取**近地分位**：把落在 [ground_z_min, ground_z_max] 竖窗内的点按 z 排序，
#   取第 ground_percentile 分位作"本帧地面高"（默认 20%，对台阶占比不敏感；平地即窗口内的地）。
#   一帧只有一个地面高（不做逐格局部地面）——这是本版的刻意简化，边界见下。
#
# ★本版刻意不做的事（真人钉死的边界，别顺手扩）★
#   - 不做滤波链（不接任何滤波/融合/漂移修正）；
#   - 不做位姿估计（直接吃 /sim/odom_gt 给的世界位姿；真机上由定位层给）；
#   - 不追 elevation_mapping 的功能全集（没有 inpainting、没有可见性射线、没有机器人运动补偿）；
#   - 不做多帧融合/时序持久（每帧独立成图，栅格外即输出 NaN）；
#   - 不发可视化话题、不读任何门值文件、不改 nav 侧在册配置。
#
# ★坐标与栅格约定（写死，改任何一条都要在 04 登记）★
#   - 输入点云 frame：/rslidar_points 的 header.frame_id（仿真里是 rslidar）→ 先按 tf 变到 map；
#   - 输出 GridMap frame：map（与在册 costmap_common.yaml 的 global_frame 一致）；
#   - 栅格中心＝map 系下的固定对齐网格（左上角对齐到 resolution 整数倍），机器人走时窗口滑、
#     格子不重采样 —— 这样同一世界位置在多帧里落在同一格，读数可比。
#
# 跑法（叠加到任意已起 Gazebo 的台上）：
#   rosrun lite3_costmap_layers elevation_gridmap_publisher.py
#   rosrun lite3_costmap_layers elevation_gridmap_publisher.py _mode:=blank     # 空白对照景
#   rosrun lite3_costmap_layers elevation_gridmap_publisher.py --selftest       # 十枚自测，不起 ROS
# ══════════════════════════════════════════════════════════════════════════════════════════

import argparse
import array
import math
import sys

import numpy as np

# ★grid_map_ros 没有 Python 绑定（本镜像实核：import grid_map_ros → ModuleNotFoundError），
#   所以 MultiArray 的存储序与维度标签只能按 C++ 侧 header 逐字照抄：
#   /opt/ros/noetic/include/grid_map_ros/GridMapMsgHelpers.hpp:
#     storageIndexNames[StorageIndices::Column] = "column_index"、[Row] = "row_index"
#     isRowMajor(): dim[0].label == "row_index" ⇒ row-major，否则 column-major
#     getRows():    row-major ⇒ dim[0].size；getCols(): row-major ⇒ dim[1].size
#   /opt/ros/noetic/include/grid_map_core/GridMap.hpp: typedef Eigen::MatrixXf Matrix（★列主序★）
#   ⇒ 本节点一律按**列主序**填，标签写 "column_index"，两端语义严格对齐，不做任何猜测。
LABEL_COLUMN = "column_index"
LABEL_ROW = "row_index"


# ══════════════════════════════════════════════════════════════════════════════════════════
# 纯函数层（不依赖 ROS，可被 --selftest 直接跑）
# ══════════════════════════════════════════════════════════════════════════════════════════

def drop_invalid_points(points, min_range=0.20, max_range=None):
    """剔除无效点。返回 (保留下来的点, 剔除数)。

    ★★这一条是本节点最要紧的卫生工序，缺了就必错，不是"可选优化"★★
    仿真里 livox_laser_simulation 对**射线未命中**的点填 (0,0,0)（不是 NaN）。这些点经 tf 变到
    odom 后恰好全部落在"雷达安装高"那一层（本批实测：20000 点里 16675 点 range=0，变换后
    z 全等于 0.45＝MID360 安装高），于是：
      ① 地面分位法把它们当成地面 ⇒ ground_z 读出 0.45（真地面是 0）；
      ② h = z − 0.45 ⇒ 整片地面被判成 −0.45，台阶高的那几级也压到 0 以下 ⇒ 图上 h_max=0、
         有效格只剩百来个 —— 看起来"链路通了但读不出东西"，极容易被误判成算法问题。
    正解＝在**求地面之前**先按 range 剔：range < min_range 的点既有未命中填充、也有近距离自击回波，
    两类都不属于场景，一律不进栅格。
    """
    if points is None or len(points) == 0:
        return points, 0
    r = np.linalg.norm(points[:, :3], axis=1)
    if max_range is None:
        keep = r >= min_range
    else:
        keep = (r >= min_range) & (r <= max_range)
    return points[keep], int((~keep).sum())


def estimate_ground_z(points_map, ground_z_min=-2.0, ground_z_max=0.5, percentile=20.0):
    """从一帧 map 系点云里估"局部地面高"。

    只看竖窗 [ground_z_min, ground_z_max] 内的点（排除天花板与远处高物），取给定分位。
    返回 (ground_z, n_used)；窗口内无点（或点太少）时返回 (None, 0)，调用方须据此判 INVALID。

    ★为什么是分位而不是最小值★：最小值对单点噪声、对地面上的小台阶极敏感；
    分位对"窗口里台阶占了 x%"这种构图变化不敏感，平地时收敛到真地面。
    ★为什么是 20% 而不是 50%★：上楼梯时地平面上方开始有点（第一级台面），
    分位越低越贴真地面；但太低（如 1%）又会被地面噪点往下拽。20% 在本仓仿真点云下
    平地与梯脚都能读出正确地面（见 --selftest 的 T5／T6）。
    """
    if points_map is None or len(points_map) == 0:
        return None, 0
    z = points_map[:, 2]
    sel = z[(z >= ground_z_min) & (z <= ground_z_max)]
    if sel.size < 30:          # 少于 30 点不足以定地面，宁判 INVALID 不硬给一个数
        return None, int(sel.size)
    return float(np.percentile(sel, percentile)), int(sel.size)


def rasterize_elevation(points_map, center_xy, size_x, size_y, resolution,
                        ground_z, height_clip=(-1.0, 2.0)):
    """把 map 系点云栅格化成 2.5D 高程矩阵。

    返回 (grid, valid)：grid 形如 (n_rows, n_cols)，float32，NaN＝无点；
                        valid 为布尔掩码（有限值处为 True）。
    栅格几何：以 center_xy 为窗口中心，左上角对齐到 resolution 的整数倍；
              第 (i, j) 格（i＝行号＝沿 y，j＝列号＝沿 x）的中心见下面 gx/gy 公式。
    高程语义：格内逐点 h = z − ground_z，取该格最大值（最"高"的那点代表这一格）。
              h 先与 height_clip 取交集——clip 只防离群点（窗外的树、天上的东西），
              不参与障碍判据（判据在消费侧 yaml 里）。
    """
    n_cols = int(round(size_x / resolution))
    n_rows = int(round(size_y / resolution))
    grid = np.full((n_rows, n_cols), np.nan, dtype=np.float32)
    if points_map is None or len(points_map) == 0:
        return grid, np.zeros((n_rows, n_cols), dtype=bool)

    x0 = center_xy[0] - size_x * 0.5
    y0 = center_xy[1] - size_y * 0.5
    # 左上角对齐到 resolution 整数倍（同一世界位置多帧落在同一格，读数可比）
    x0 = math.floor(x0 / resolution) * resolution
    y0 = math.floor(y0 / resolution) * resolution

    xs = points_map[:, 0]
    ys = points_map[:, 1]
    zs = points_map[:, 2]

    jj = np.floor((xs - x0) / resolution).astype(np.int64)
    ii = np.floor((ys - y0) / resolution).astype(np.int64)
    inside = (jj >= 0) & (jj < n_cols) & (ii >= 0) & (ii < n_rows)
    if not np.any(inside):
        return grid, np.zeros((n_rows, n_cols), dtype=bool)

    ii = ii[inside]
    jj = jj[inside]
    hh = (zs[inside] - ground_z).astype(np.float32)
    keep = np.isfinite(hh) & (hh >= height_clip[0]) & (hh <= height_clip[1])
    ii, jj, hh = ii[keep], jj[keep], hh[keep]
    if hh.size == 0:
        return grid, np.zeros((n_rows, n_cols), dtype=bool)

    # 逐格取大：★不能用 np.maximum.at 直接铺在 NaN 初值上★——NaN 参与 maximum 后结果仍是 NaN
    #（本机 Python 3.13/numpy 实核：acc 初值全 NaN ⇒ maximum.at 之后整片仍 NaN，且抛
    #  "invalid value encountered in maximum"），所以先用 -inf 铺底做逐格取大，再把**从未被写到**
    # 的格显式改回 NaN —— "无点 ≠ 高 0"，这一条若不显式处理，消费侧就会把真空格当成平地放行。
    flat = ii * n_cols + jj
    acc = np.full(n_rows * n_cols, -np.inf, dtype=np.float32)
    np.maximum.at(acc, flat, hh)
    written = np.zeros(n_rows * n_cols, dtype=bool)
    written[flat] = True
    acc[~written] = np.nan
    grid = acc.reshape(n_rows, n_cols)
    valid = np.isfinite(grid)
    return grid, valid


def grid_to_gridmap_message_fields(grid, resolution, center_xy, frame_id, stamp_secs, layer="elevation"):
    """把高程矩阵包成 GridMap 消息的各个字段（不 import ROS 也能跑，便于自测）。

    返回 dict：info(resolution/length_x/length_y/pose/header) + layers + data(扁平 list)
    存储序＝★列主序★（.T.ravel(order='C') 等价于原矩阵按列展开），标签＝"column_index"，
    与 grid_map::Matrix（Eigen 列主序）逐字一致。NaN 原样发出（消费侧按 !isfinite 跳过）。
    """
    n_rows, n_cols = grid.shape
    length_x = n_cols * resolution
    length_y = n_rows * resolution
    # 列主序展平：先第 0 列全部行、再第 1 列……
    flat = np.ascontiguousarray(grid.T).ravel(order="C").astype(np.float32)
    return {
        "frame_id": frame_id,
        "stamp_secs": stamp_secs,
        "layer": layer,
        "resolution": float(resolution),
        "length_x": float(length_x),
        "length_y": float(length_y),
        "center_x": float(center_xy[0]),
        "center_y": float(center_xy[1]),
        "n_rows": int(n_rows),
        "n_cols": int(n_cols),
        "data": flat,
        "outer_size": int(n_cols),   # dim[0]＝列（列主序）
        "inner_size": int(n_rows),   # dim[1]＝行
    }


def fields_to_matrix(flat, n_rows, n_cols):
    """把消息的扁平数据还原成 (n_rows, n_cols) 矩阵——自测里用作往返校验（等价于 consumption 侧读法）。"""
    arr = np.asarray(flat, dtype=np.float32).reshape(n_cols, n_rows).T
    return arr


# ══════════════════════════════════════════════════════════════════════════════════════════
# ROS 层
# ══════════════════════════════════════════════════════════════════════════════════════════

class ElevationGridMapPublisher(object):
    def __init__(self):
        import rospy
        from grid_map_msgs.msg import GridMap
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        import tf2_ros

        self._rospy = rospy
        self._GridMap = GridMap

        # ---- 参数（全部带缺省，缺省值即在册值；改缺省＝改在册语义，需 04 登记）----
        self.point_topic = rospy.get_param("~point_topic", "/rslidar_points")
        self.odom_topic = rospy.get_param("~odom_topic", "/sim/odom_gt")
        self.out_topic = rospy.get_param("~output_topic", "/elevation_mapping/elevation_map")
        self.layer_name = rospy.get_param("~layer_name", "elevation")
        self.grid_frame = rospy.get_param("~grid_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        # 帧候选序：首选 grid_frame（＝消费侧 global_frame），不可达时依次退。
        # 为什么 odom 是最稳的退路：仿真里 odom→base_link 由 lite3_planar_drive 闭环发、恒在；
        # 而 map→odom 只在 m2_teb 链路里有。★退到 odom 不是降级、是同一几何的等价表述★。
        # ★这一段必须在 robot_frame 之后★（本批实测踩过：先引用后定义 ⇒ AttributeError 起不来）。
        cands = rospy.get_param("~frame_candidates", "")
        if isinstance(cands, str):
            cands = [c.strip() for c in cands.split(",") if c.strip()]
        self.frame_candidates = cands if cands else [self.grid_frame, "odom", self.robot_frame]
        self.resolution = float(rospy.get_param("~grid_resolution", 0.1))
        self.size_x = float(rospy.get_param("~grid_size_x", 4.0))
        self.size_y = float(rospy.get_param("~grid_size_y", 4.0))
        self.ground_percentile = float(rospy.get_param("~ground_percentile", 20.0))
        self.ground_z_min = float(rospy.get_param("~ground_z_min", -2.0))
        self.ground_z_max = float(rospy.get_param("~ground_z_max", 0.5))
        self.height_clip = (float(rospy.get_param("~height_clip_min", -1.0)),
                            float(rospy.get_param("~height_clip_max", 2.0)))
        self.min_points = int(rospy.get_param("~min_points", 200))
        # ★无效点剔除阈值（见 drop_invalid_points 的长注释）★
        # min_range 缺省 0.20 m：既覆盖 livox 未命中填充 (0,0,0)，也覆盖机体自击的近距回波；
        # max_range 缺省 0（＝不限）——远处地面点对"局部地面"是有用的，不该裁。
        self.min_range = float(rospy.get_param("~min_range", 0.20))
        _maxr = float(rospy.get_param("~max_range", 0.0))
        self.max_range = _maxr if _maxr > 0 else None
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.use_tf = bool(rospy.get_param("~use_tf", True))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))

        self.latest_cloud = None
        self.latest_odom = None
        self.used_frame = self.grid_frame
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0)) if self.use_tf else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) if self.use_tf else None

        self.n_frames_in = 0
        self.n_frames_out = 0
        self.n_invalid = 0

        self.sub_cloud = rospy.Subscriber(self.point_topic, PointCloud2, self._on_cloud, queue_size=2)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=2)
        self.pub = rospy.Publisher(self.out_topic, GridMap, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 0.1)), self._tick)

        rospy.loginfo(
            "[elevation_gridmap_publisher] 在位：in=%s odom=%s out=%s layer=%s frame=%s "
            "res=%.3f size=%.1fx%.1f ground_pct=%.1f  ★h＝相对局部地面的离地高[m]，地面 h≡0★",
            self.point_topic, self.odom_topic, self.out_topic, self.layer_name, self.grid_frame,
            self.resolution, self.size_x, self.size_y, self.ground_percentile)

    # ---- 回调 ----
    def _on_cloud(self, msg):
        self.latest_cloud = msg

    def _on_odom(self, msg):
        self.latest_odom = msg

    # ---- 点云解码 ----
    def _cloud_to_numpy(self, msg):
        """按 header 的 fields/offsets 自适应解码 xyz（不写死 velodyne 布局，C16 布局也照吃）。

        ★别写成 buf[idx:idx+4] 那种逐点切片★：切片窗口长度恒定，numpy 会把结果降维成
        (n,4) 的花式索引尝试，直接抛 "only integer scalar arrays can be converted to a scalar index"
        （本批实测踩到）。正解＝先把字节流 reshape 成 (n, point_step)，再按列取偏移后的 4 字节。
        """
        names = {f.name: f.offset for f in msg.fields}
        if not all(k in names for k in ("x", "y", "z")):
            return None
        n = msg.width * msg.height
        need = msg.point_step * n
        if n <= 0 or len(msg.data) < need:
            return None
        step = msg.point_step
        # is_dense=False 时数据可能带填充，但 point_step 恒定 ⇒ reshape 后按列取偏移即安全
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=need).reshape(n, step)
        out = np.empty((n, 3), dtype=np.float32)
        for k, name in enumerate(("x", "y", "z")):
            off = names[name]
            if off + 4 > step:
                return None
            out[:, k] = raw[:, off:off + 4].copy().view(np.float32).reshape(n)
        return out[np.isfinite(out).all(axis=1)]

    def _to_map(self, pts, cloud_frame, stamp):
        """把点云从 cloud_frame 变到目标网格帧。返回 (points_target 或 None, 说明, 实际使用帧)。

        ★目标帧不是死取的★：消费侧成本图的 global_frame 是 map，但**仿真台上 map→odom 段
        只在 m2_teb 链路里由 m2_tf_fresh 提供**，stairstudy / m1_gazebo 这些台子上根本没有 map 帧
        （本批实测：tf 树只有 odom→base_link→rslidar）。硬取 map ⇒ 本节点在那些台子上永远 INVALID。
        正解＝按 frame_candidates 顺序**探第一个能查到的祖先帧**，并把**实际用的那个帧**写进
        GridMap 头；消费侧 ElevationMergedLayer 本来就会对"帧 ≠ 全局帧"做 tf 平面变换
        （updateBounds 里 map.getFrameId() != global_frame 分支），所以换成 odom 不改变其语义，
        只是少绕一层 map→odom 的恒等变换。
        """
        if not self.use_tf:
            return pts, "no-tf", self.grid_frame
        ts = stamp if not stamp.is_zero() else self._rospy.Time(0)
        for cand in self.frame_candidates:
            if cloud_frame == cand:
                return pts, "identity", cand
            try:
                tr = self.tf_buffer.lookup_transform(cand, cloud_frame, ts, self._rospy.Duration(0.2))
            except Exception:                       # tf2 的异常族较杂，统一捕获后换下一个候选
                continue
            q = tr.transform.rotation
            tx = tr.transform.translation.x
            ty = tr.transform.translation.y
            tz = tr.transform.translation.z
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ], dtype=np.float64)
            if cand != self.grid_frame:
                self._rospy.loginfo_throttle(
                    10.0, "[elevation_gridmap_publisher] 首选帧 %s 不可达 ⇒ 本轮用 %s（消费侧按 tf 平面变换）",
                    self.grid_frame, cand)
            return pts.astype(np.float64).dot(R.T) + np.array([tx, ty, tz]), "tf:" + cand, cand
        return None, "tf-fail:no-candidate", self.grid_frame

    def _robot_xy(self):
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        return (p.x, p.y)

    # ---- 主循环 ----
    def _tick(self, _evt):
        rospy = self._rospy
        if self.latest_cloud is None:
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 还没收到点云（%s）", self.point_topic)
            return
        now = rospy.Time.now()
        age = (now - self.latest_cloud.header.stamp).to_sec()
        if self.stale_timeout > 0 and age > self.stale_timeout:
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 点云已 stale %.2fs ⇒ 本拍不发", age)
            return

        pts = self._cloud_to_numpy(self.latest_cloud)
        if pts is None or len(pts) < self.min_points:
            self.n_invalid += 1
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 点数不足（%s < %d）⇒ INVALID，不发",
                                   "None" if pts is None else len(pts), self.min_points)
            return

        # ★★先剔无效点，再算任何东西★★（顺序不可换）：livox 插件对未命中射线填 (0,0,0)，
        # 那些点变换后全落在雷达安装高那一层，会被地面分位法当成地面、把 ground_z 抬到 0.45。
        n_raw = len(pts)
        pts, n_dropped = drop_invalid_points(pts, self.min_range, self.max_range)
        if len(pts) < self.min_points:
            self.n_invalid += 1
            rospy.logwarn_throttle(
                5.0, "[elevation_gridmap_publisher] 剔无效点后剩 %d < %d（原 %d、剔 %d）⇒ INVALID，不发",
                len(pts), self.min_points, n_raw, n_dropped)
            return

        cloud_frame = self.latest_cloud.header.frame_id.lstrip("/")
        pts_map, how, used_frame = self._to_map(pts, cloud_frame, self.latest_cloud.header.stamp)
        if pts_map is None:
            self.n_invalid += 1
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 点云变到 %s 失败（%s）⇒ INVALID，不发",
                                   self.frame_candidates, how)
            return
        self.used_frame = used_frame

        ground_z, n_ground = estimate_ground_z(pts_map, self.ground_z_min, self.ground_z_max,
                                               self.ground_percentile)
        if ground_z is None:
            self.n_invalid += 1
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 地面估不出（窗内仅 %d 点）⇒ INVALID，不发",
                                   n_ground)
            return

        center = self._robot_xy()
        if center is None:
            center = (float(np.median(pts_map[:, 0])), float(np.median(pts_map[:, 1])))
            rospy.logwarn_throttle(5.0, "[elevation_gridmap_publisher] 未收到里程计（%s）⇒ 窗口中心暂按点云中位",
                                   self.odom_topic)

        grid, _valid = rasterize_elevation(pts_map, center, self.size_x, self.size_y, self.resolution,
                                           ground_z, self.height_clip)

        msg = self._build_message(grid, center, self.latest_cloud.header.stamp, used_frame)
        self.pub.publish(msg)
        self.n_frames_in += 1
        self.n_frames_out += 1

        n_valid = int(np.isfinite(grid).sum())
        h_fin = grid[np.isfinite(grid)]
        h_max = float(h_fin.max()) if h_fin.size else float("nan")
        rospy.loginfo_throttle(
            2.0, "[elevation_gridmap_publisher] 发出 GridMap（%dx%d、res=%.3f、frame=%s、ground_z=%.3f、"
                 "有效格 %d、h_max=%.3f、点数 %d）",
            grid.shape[1], grid.shape[0], self.resolution, used_frame, ground_z, n_valid, h_max, len(pts_map))

    def _build_message(self, grid, center, stamp, frame_id=None):
        rospy = self._rospy
        # ★帧取实际使用的那个★（首选 map 不可达时是 odom），不是死写 grid_frame
        fid = frame_id or self.grid_frame
        f = grid_to_gridmap_message_fields(grid, self.resolution, center, fid,
                                           stamp.to_sec(), self.layer_name)
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
        from geometry_msgs.msg import Pose, Point, Quaternion

        gmap = self._GridMap()
        # ---- info ----
        gmap.info.header.stamp = stamp
        gmap.info.header.frame_id = f["frame_id"]
        gmap.info.resolution = f["resolution"]
        gmap.info.length_x = f["length_x"]
        gmap.info.length_y = f["length_y"]
        gmap.info.pose = Pose(Point(f["center_x"], f["center_y"], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        # ---- layers ----
        gmap.layers = [f["layer"]]
        gmap.basic_layers = [f["layer"]]
        # ---- data：一枚 Float32MultiArray，列主序（与 Eigen 列主序一致）----
        arr = Float32MultiArray()
        layout = MultiArrayLayout()
        layout.data_offset = 0
        d0 = MultiArrayDimension()
        d0.label = LABEL_COLUMN
        d0.size = f["outer_size"]          # 列
        d0.stride = f["outer_size"] * f["inner_size"]
        d1 = MultiArrayDimension()
        d1.label = LABEL_ROW
        d1.size = f["inner_size"]          # 行
        d1.stride = f["inner_size"]
        layout.dim = [d0, d1]
        arr.layout = layout
        # ★array('f') 是 C++ std::vector<float> 的 Python 对应物★：直接赋 numpy 数组 rospy 会拒
        #（报 "Expected a list or array of floats"），必须走 array.array。
        # ★仍然只用 tolist() 这一条路径★（曾试先 tobytes 再 array，字节序在部分平台会错，已删）。
        arr.data = array.array("f", f["data"].astype(np.float32).tolist())
        gmap.data = [arr]
        gmap.outer_start_index = 0
        gmap.inner_start_index = 0
        return gmap


# ══════════════════════════════════════════════════════════════════════════════════════════
# 自测（十枚；--selftest 直接跑，不上 ROS、不碰任何在册件）
# ══════════════════════════════════════════════════════════════════════════════════════════

def _flat_ground(n=4000, z=0.0, spread=2.0):
    rng = np.random.RandomState(0)
    x = rng.uniform(-spread, spread, n)
    y = rng.uniform(-spread, spread, n)
    zz = np.full(n, z, dtype=np.float64) + rng.normal(0, 0.005, n)
    return np.stack([x, y, zz], axis=1)


def _stairs(base_x=0.30, riser=0.06, tread=0.10, n_steps=8, y_half=0.4, per_step=200):
    """造一段台阶点云：第 k 级台面 y∈[-0.4,0.4]、x∈[base_x+k*tread, base_x+(k+1)*tread]、z=(k+1)*riser。"""
    pts = []
    rng = np.random.RandomState(1)
    for k in range(n_steps):
        x0 = base_x + k * tread
        x = rng.uniform(x0, x0 + tread, per_step)
        y = rng.uniform(-y_half, y_half, per_step)
        z = np.full(per_step, (k + 1) * riser)
        pts.append(np.stack([x, y, z], axis=1))
    return np.concatenate(pts, axis=0)


def _selftest():
    fails = []
    lines = []

    def chk(name, cond, detail):
        lines.append(("PASS" if cond else "FAIL") + "  " + name + "  " + detail)
        if not cond:
            fails.append(name)

    # T1 平地地面估计：ground ≈ 0
    g, n = estimate_ground_z(_flat_ground())
    chk("T1 平地 ground_z≈0", g is not None and abs(g) < 0.02, "ground=%.4f n=%d" % (g if g else -9, n))

    # T2 平地栅格：所有有效格 h≈0（这就是"平地 h≈0"那条正控的可执行形）
    pts = _flat_ground()
    grid, valid = rasterize_elevation(pts, (0.0, 0.0), 4.0, 4.0, 0.1, g or 0.0)
    h = grid[np.isfinite(grid)]
    chk("T2 平地 h≈0", h.size > 100 and abs(float(h.max())) < 0.05,
        "有效格=%d h_max=%.4f h_min=%.4f" % (h.size, float(h.max()), float(h.min())))

    # T3 栅格几何：4x4 m / 0.1 ⇒ 40x40
    chk("T3 栅格尺寸 40x40", grid.shape == (40, 40), "shape=%s" % (grid.shape,))

    # T4 窗口外点不进图（窗内只留 4x4 m）
    pts_far = _flat_ground()
    pts_far[:] = pts_far + np.array([50.0, 50.0, 0.0])
    grid_far, _ = rasterize_elevation(pts_far, (0.0, 0.0), 4.0, 4.0, 0.1, 0.0)
    chk("T4 窗外点不入图", int(np.isfinite(grid_far).sum()) == 0,
        "有效格=%d" % int(np.isfinite(grid_far).sum()))

    # T5 梯脚台阶读出：第一级 0.06 m 应被读出为 ≈0.06（"梯脚台阶高度读出"那条正控）
    # ★这一枚同时是"取大 vs 取中位"的分辨器★：那一列里平地回波与第一级台面**同格共存**，
    #   按格取大 ⇒ 读到 0.06（台面在，不能漏）；若实现成取中位 ⇒ 读到 ≈0（平地把台阶淹没）。
    #   所以判据必须用 **max**，且断言点是"那一列能读出 0.06"这唯一一件事，不掺别的量。
    pts_st = np.concatenate([_flat_ground(n=6000, spread=2.0), _stairs()], axis=0)
    g_st, n_st = estimate_ground_z(pts_st)
    grid_st, _ = rasterize_elevation(pts_st, (1.0, 0.0), 4.0, 4.0, 0.1, g_st or 0.0)
    # 第一级台面：x∈[0.30,0.40]，栅格列号 = floor((x-x0)/res)，x0 与函数内的对齐式逐字同式
    x0 = math.floor((1.0 - 2.0) / 0.1) * 0.1
    j_first = int(math.floor((0.30 + 0.05 - x0) / 0.1))
    col = grid_st[:, j_first]
    col = col[np.isfinite(col)]
    h_first = float(col.max()) if col.size else float("nan")
    chk("T5 第一级台阶高度≈0.06", abs(h_first - 0.06) < 0.02,
        "h_first=%.4f（该列 max；取中位会是 %.4f）ground=%.4f"
        % (h_first, float(np.median(col)) if col.size else float("nan"), g_st or -9))

    # T6 台阶在平地上估地面不被抬高（分位法抗台阶污染）
    chk("T6 台阶不动地面估计", g_st is not None and abs(g_st) < 0.02, "ground=%.4f" % (g_st if g_st else -9))

    # T7 列主序往返：消息扁平数据 → 还原矩阵，与原矩阵逐格相同
    n_rows, n_cols = 5, 7
    m = np.arange(n_rows * n_cols, dtype=np.float32).reshape(n_rows, n_cols)
    f = grid_to_gridmap_message_fields(m, 0.1, (0.0, 0.0), "map", 0.0)
    back = fields_to_matrix(f["data"], n_rows, n_cols)
    chk("T7 列主序往返一致", np.array_equal(back, m), "n_rows=%d n_cols=%d dim0=%s" % (n_rows, n_cols, f["outer_size"]))

    # T8 NaN 保持（无点格发 NaN，不被零填充骗过消费侧）
    m2 = np.full((3, 3), np.nan, dtype=np.float32)
    m2[1, 1] = 0.5
    f2 = grid_to_gridmap_message_fields(m2, 0.1, (0.0, 0.0), "map", 0.0)
    back2 = fields_to_matrix(f2["data"], 3, 3)
    chk("T8 NaN 不丢", np.isnan(back2).sum() == 8 and abs(float(back2[1, 1]) - 0.5) < 1e-6,
        "nan数=%d 中心=%.3f" % (int(np.isnan(back2).sum()), float(back2[1, 1])))

    # T9 点太少 ⇒ ground 判不出（INVALID，而不是给个假数）
    g_few, n_few = estimate_ground_z(np.zeros((5, 3)))
    chk("T9 少点判 INVALID", g_few is None and n_few < 30, "ground=%s n=%d" % (g_few, n_few))

    # T10 height_clip 生效（窗外高物被裁掉，不冒充障碍）
    pts_hi = np.concatenate([_flat_ground(), np.array([[0.0, 0.0, 9.0]])], axis=0)
    grid_hi, _ = rasterize_elevation(pts_hi, (0.0, 0.0), 4.0, 4.0, 0.1, 0.0, height_clip=(-1.0, 2.0))
    h_hi = grid_hi[np.isfinite(grid_hi)]
    chk("T10 clip 裁离群", h_hi.size > 0 and float(h_hi.max()) < 2.1, "h_max=%.3f" % float(h_hi.max()))

    # T11 ★未命中填充点 (0,0,0) 必须被剔除，且剔完地面仍是 0（不是被抬到安装高 0.45）★
    #     这一枚直指本批实测吃到的那只坑：仿真 83% 的点是 (0,0,0)，变换后恰好落在雷达安装高
    #     那一层，若不剔，ground_z 会读出 0.45、整片地面被算成 −0.45。
    #     ★地面点要生成在 min_range 之外（spread=1.0 ⇒ 绝大多数点 r≥1 m）★，
    #     否则会被一起剔掉，测试就跑成了"剔掉地面"的反例（本批自测第二版踩过）。
    pts_ground = _flat_ground(n=3000, spread=1.0)
    pad = np.zeros((12000, 3))                                    # 未命中填充：(0,0,0)
    pts_sim = np.concatenate([pts_ground, pad], axis=0)
    pts_kept, n_drop = drop_invalid_points(pts_sim, min_range=0.20)
    g_pad, _ = estimate_ground_z(pts_kept)
    near_ground_dropped = sum(1 for p in pts_ground if float(np.linalg.norm(p)) < 0.20)
    chk("T11 未命中填充点被剔、地面仍≈0",
        n_drop == 12000 + near_ground_dropped and g_pad is not None and abs(g_pad) < 0.02,
        "剔 %d 点（含填充 12000 + 近距地面 %d）、剩 %d、ground=%.4f"
        % (n_drop, near_ground_dropped, len(pts_kept), g_pad if g_pad is not None else -9))

    # T12 反控：模拟"自击回波把地面分位抬走"的真实情形。
    #     ★构造要点★：分位法只有在该高度上的点**足够多**时才会被抬走。真实现场 83% 的点是
    #     未命中填充（z 落在安装高），所以此处也必须让"抬高点"占多数，否则 20 分位老老实实
    #     落在地面那一侧、反控就白写（本批自测第三版正是这么白写的：6000 自击 vs 3000 地面 ⇒ 仍读 0）。
    #     这里 6000 地面 vs 12000 朝下自击 ⇒ 不剔时 20 分位被抬到 0.45 附近；加严 min_range 后回正。
    selfhit = np.zeros((12000, 3))
    selfhit[:, 2] = 0.45                                          # 变换后在安装高那一层
    pts_self = np.concatenate([pts_ground, selfhit], axis=0)
    g_bad, _ = estimate_ground_z(pts_self)
    pts_fix, n_fix = drop_invalid_points(pts_self, min_range=0.60)
    g_fix, _ = estimate_ground_z(pts_fix)
    chk("T12 反控：抬高点不剔则地面被拉走、加严 min_range 后回正",
        g_bad is not None and g_bad > 0.10 and g_fix is not None and abs(g_fix) < 0.05,
        "不剔 ground=%.3f（被抬走）／min_range=0.60 后 ground=%.4f（剔 %d 点）"
        % (g_bad if g_bad is not None else -9, g_fix if g_fix is not None else -9, n_fix))

    print("\n".join(lines))
    print("-" * 78)
    print("自测 %d 枚，FAIL %d 枚" % (len(lines), len(fails)))
    return 0 if not fails else 1


def main():
    # 参数解析分两段：先看是不是 --selftest（那时不 init_node）
    argv = [a for a in sys.argv if not a.startswith("__")]
    if "--selftest" in argv:
        return _selftest()

    import rospy
    rospy.init_node("elevation_gridmap_publisher")
    ElevationGridMapPublisher()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
