"""世界坐标 → 格索引的浮点核（全仓唯一实现，口径逐字冻结）。

约定（格覆盖语义，供读者对照；各函数实际行为以其冻结表达式与差分测试为准）：
  cell i 覆盖 [origin + i*res, origin + (i+1)*res)（左闭右开）。
  在浮点里 x=k*res 的商是 (k-ε)（如 0.3/0.1 = 2.9999999999999996），
  因此带 epsilon 的 cell_snap 把它判给右格 k，不带的 cell_floor 判给左格 k-1。
  ★ 两种口径当前都合法存在，交换数据的两侧必须用同一种；这是 Phase B 要
    裁决的事，本包不做默默合一。★

每个函数注明历史出处（文件:行，以 2026-11-18 HEAD 为准）——差分测试的
oracle 就是这些出处表达式的字面拷贝。
"""
import math

import numpy as np

EPS = 1e-9  # 冻结自 support_map.py:157 的 epsilon


def cell_floor(x, res):
    """绝对索引，floor 除法取整，无 epsilon。

    冻结自 representation/elevation.py:38 `int(x // resolution)`。
    x=0.3, res=0.1 → 2（浮点商 2.999...，floor 后取 2）。
    """
    return int(x // res)


def cell_snap(x, res, eps=EPS):
    """绝对索引，商加 epsilon 再 floor。

    冻结自 representation/support_map.py:157 `math.floor(x / resolution + 1e-9)`。
    x=0.3, res=0.1 → 3。epsilon 吸收「x=k*res 的浮点商是 (k-ε)」这一类误差，
    使世界坐标恰为格边界时判给右格（左闭右开约定）。
    """
    return math.floor(x / res + eps)


def cell_clamp_edge(v, res):
    """包络收边：上取整。

    冻结自 planner/support_planner.py:269-272 的钳位分支 `int(math.ceil(v / resolution))`。
    用于「落点不在 cells 里则钳到足迹边界」的越界处理。
    """
    return int(math.ceil(v / res))


def cell_center(v, res):
    """半格中心索引：ceil((v/res) - 0.5)。

    冻结自 planner/support_planner.py:40-43 的 _cells 半格中心约定
    `int(math.ceil(v / resolution - .5))`。
    """
    return int(math.ceil(v / res - 0.5))


def image_axis(v, center, res, half):
    """图像轴索引：窗口中心原点 + 可选翻转（由调用侧组合）。

    冻结自 framework/terrain_observer.py:48-49 的标量核
    `int(math.floor((v - center) / res + half))`。
    ★ 行序翻转不在这里★ GridMap 图像行序（y 随 row 增大而减小）由调用侧
    传 v=center_y - y 表达；本函数只做单轴映射，保持语义独立。
    half = n_rows / 2.0 或 n_cols / 2.0。
    """
    return int(math.floor((v - center) / res + half))


def cells_floor_min(arr, min_v, res):
    """向量化：min 对齐批量落格。

    冻结自 representation/open_data_import.py:102-103
    `np.floor((band[:, 0] - min_x) / resolution).astype(int)`。
    返回 float 数组（调用侧按需 astype）。
    """
    return np.floor((arr - min_v) / res)


def cells_floor_origin(arr, x0, res):
    """向量化：窗口原点对齐批量落格。

    冻结自 40_navigation/lite3_costmap_layers/scripts/
    elevation_gridmap_publisher.py:141-142 `np.floor((xs - x0) / resolution)`。
    x0 应已对齐到 resolution 整数倍（调用侧职责，见该文件 134-135 行）。
    """
    return np.floor((arr - x0) / res)
