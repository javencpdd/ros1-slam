"""V8-10 第三切片：跟踪面设计（R655）。

语义：参考轨迹（V8-09/局部规划输出，已过检查器）vs 实际位姿流的**跟踪偏差核验**——
横向/纵向/垂向三向偏差对阈值；越限＝E_TRACK_DEVIATION（附最近参考点索引与偏差值）。
参考耗尽仍在跑＝E_TRACK_OVERRUN。越限即触发 enforcement.on_rejection 撤销链
（「持续被拒轨迹不算规划成功」的跟踪闭环：偏差大→撤销→Tracker 计数→达限停）。
估计与真值分开（A6）：本面只比对「估计位姿 vs 参考」，真值通道由评价器独立消费，
不在此混入。
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402


class TrackingError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def check_tracking(reference, actual, tol_lateral=0.15, tol_longitudinal=0.20,
                   tol_vertical=0.10, stamp=None, ref_index=None):
    """reference＝[(x,y,z[,t])...]；actual＝(x,y,z)（或逐次喂入位姿点）。

    配准优先级：ref_index（运行面由里程参数化给出）＞时戳＞XY 最近。
    XY 最近在转层桥接处会把跨面点错配到旧段（实测：mezz 点被配到 ground 段末，
    垂向差 0.6 全算偏差）——故 XY 最近带**转层扩展**：若最近点与次近候选跨面
    （z 差 >2×tol_vertical）且次近更近于实际 z，取次近。
    违例列表供 on_rejection 消费；空列表＝跟踪在限内。
    """
    if len(reference) < 1:
        raise TrackingError('E_TRACK_EMPTY_REFERENCE', '空参考轨迹')
    violations = []
    index = _nearest_reference(reference, actual, stamp, ref_index)
    rx, ry, rz = reference[index][0], reference[index][1], reference[index][2]
    ax, ay, az = actual[0], actual[1], actual[2]
    heading = _heading_at(reference, index)
    dx, dy = ax - rx, ay - ry
    longitudinal = dx * heading[0] + dy * heading[1]
    lateral = -dx * heading[1] + dy * heading[0]
    vertical = az - rz
    if abs(lateral) > tol_lateral:
        violations.append(('E_TRACK_DEVIATION', index,
                           '横向偏差 %.3f > %.3f' % (abs(lateral), tol_lateral)))
    if abs(longitudinal) > tol_longitudinal:
        violations.append(('E_TRACK_DEVIATION', index,
                           '纵向偏差 %.3f > %.3f' % (abs(longitudinal), tol_longitudinal)))
    if abs(vertical) > tol_vertical:
        violations.append(('E_TRACK_DEVIATION', index,
                           '垂向偏差 %.3f > %.3f' % (abs(vertical), tol_vertical)))
    return violations


def _nearest_reference(reference, actual, stamp, ref_index=None):
    if ref_index is not None:
        return max(0, min(ref_index, len(reference) - 1))
    if stamp is not None and all(len(p) > 3 for p in reference):
        return min(range(len(reference)), key=lambda i: abs(reference[i][3] - stamp))
    dists = [(math.hypot(p[0] - actual[0], p[1] - actual[1]), i)
             for i, p in enumerate(reference)]
    dists.sort()
    best = dists[0][1]
    # 转层扩展：XY 次近若 z 更贴近实际（跨面点），换配
    if len(dists) > 1:
        second = dists[1][1]
        if (abs(reference[second][2] - actual[2]) < abs(reference[best][2] - actual[2])
                and abs(reference[best][2] - reference[second][2]) > 0.2):
            return second
    return best


def _heading_at(reference, index):
    if len(reference) < 2:
        return (1.0, 0.0)  # 单点参考：航向退化，仅纵向分量有意义
    if index == 0:
        a, b = reference[0], reference[1]
    elif index == len(reference) - 1:
        a, b = reference[-2], reference[-1]
    else:
        a, b = reference[index - 1], reference[index + 1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def overrun(reference, consumed_index):
    """参考耗尽仍在跑（consumed_index 超末点）＝E_TRACK_OVERRUN。"""
    if consumed_index >= len(reference):
        return [('E_TRACK_OVERRUN', len(reference) - 1, '参考已耗尽仍收到位姿')]
    return []
