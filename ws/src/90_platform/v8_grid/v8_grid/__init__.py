"""v8_grid —— 世界坐标 → 格索引的几何基元（全仓唯一实现）。

设计裁定（2026-11-18，计划 M1 / Phase A）：
  统一的是「实现」，保留的是「口径」。本包每个函数的舍入表达式**逐字冻结**
  自一个历史调用点；各口径的差异是有意的语义（带不带 epsilon、绝对索引还是
  窗口原点），由差分测试 tests/test_differential.py 用 oracle 逐值锁死。
  口径之间的合一（例如修复「x=0.3, res=0.1 时 cell_floor=2 而 cell_snap=3」
  的交换偏移缺陷）属 Phase B，须先挂账裁决，不在本包默默进行。

依赖纪律：只依赖标准库（math）与 numpy（向量化两函数）。不 import ROS、
不读写运行目录、不依赖任何 v8_* 包。
"""
from v8_grid.indexing import (
    cell_floor,
    cell_snap,
    cell_clamp_edge,
    cell_center,
    image_axis,
    cells_floor_min,
    cells_floor_origin,
)

__all__ = [
    'cell_floor', 'cell_snap', 'cell_clamp_edge', 'cell_center',
    'image_axis', 'cells_floor_min', 'cells_floor_origin',
]
