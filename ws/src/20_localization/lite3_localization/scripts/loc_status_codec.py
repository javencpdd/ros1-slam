#!/usr/bin/env python3
"""HDL `/status` 的纯解码：把 ScanMatchingStatus 折成（收敛, 质量）两个标量。

为什么单独放进一个不依赖 rospy 的模块：这段折算是"定位健康"判据的输入，必须能被
单测逐个反控（比如 inlier_fraction 超出 [0,1] 时不能当成好质量）。挂在节点里就只能在
有 ROS 的环境里跑，且很容易写成"字段读不到就当作好"。
"""
from __future__ import annotations


class StatusDecodeError(ValueError):
    """字段缺失或非法：调用方必须当作"未知"处理，不得当作健康。"""


def decode(status):
    """入参是 ScanMatchingStatus 形状的对象；返回 {'converged', 'quality', 'quality_name'}。

    质量取 `inlier_fraction`（内点占比，越大越好，天然落在 [0,1]）。
    `matching_error` 是内点均方距离，量纲依雷达密度而变，先不用作阈值输入，
    只随结果回带，便于现场标定后再决定要不要用它。
    """
    for field in ('has_converged', 'inlier_fraction'):
        if not hasattr(status, field):
            raise StatusDecodeError('missing field: %s' % field)
    frac = float(status.inlier_fraction)
    if not (frac == frac) or frac < 0.0 or frac > 1.0:      # NaN 或越界
        raise StatusDecodeError('inlier_fraction out of range: %r' % frac)
    out = {'converged': bool(status.has_converged), 'quality': frac,
           'quality_name': 'inlier_fraction'}
    if hasattr(status, 'matching_error'):
        try:
            out['matching_error'] = float(status.matching_error)
        except (TypeError, ValueError):
            pass
    return out
