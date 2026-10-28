"""V8-10 首切片：独立轨迹检查器设计层（R653）。

22 号「独立轨迹检查器核验支撑、净空、时间有效性、地图版本和运动学上限；
被拒轨迹不算规划成功」——本件是该检查器的设计层实现，**独立于任何被检规划链**
（V8-03 实证：SCAN 优化器的连续松弛兜底会掩盖 A* 失败、产出未验证几何的轨迹，
本检查器正是对策：mode3 那种「坡脚悬空 0.298 m」的轨迹在此被逐点支撑检查拒绝）。

四项核验（均带理由码、逐违例报点）：
  支撑   E_TRAJ_UNSUPPORTED——每点须落某 supported 面足迹内且 |z−面 z|≤z_tol（悬空即拒）；
  净空   E_TRAJ_CLEARANCE——每点上方向最近面净距≥min_clearance（HeightColumn 复算）；
  连续安全 E_TRAJ_TRANSITION／E_TRAJ_DISCONTINUITY——跨面步须有显式声明的边
          （方向敏感：单向边只许 to 向）；同面相邻步距≤max_step（禁瞬移）；
  时间   E_TRAJ_TIME——带时戳时须单调且在预算窗口内。
分层：消费 map.v1（representation 视图）与 V8-09 规划器输出；不 import ROS。
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
import support_map as rep  # noqa: E402
import validate as contract  # noqa: E402


class CheckerError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def check(map_value, trajectory, z_tol=0.1, min_clearance=0.2, max_step=None,
          time_budget=None, resolution=None, expected_map_version=None):
    """trajectory＝[(x, y, z[, t]), ...]；返回 {'ok', 'violations', 'checked_points'}。

    ok=False 时 violations 逐条 (code, index, detail)——被拒轨迹不算规划成功（A5 面），
    调用方（编排/评价）据此分流，不得吞。expected_map_version 不一致＝E_TRAJ_MAP_STALE
    （22 号「地图版本」核验位：消费者持旧版地图不得判轨迹）。运动学上限核验需 robot
    profile 接线，归次切片。
    """
    try:
        contract.validate('map', map_value)
    except contract.ContractError as error:
        raise CheckerError('E_TRAJ_MAP_INVALID', '%s: %s' % (error.code, error.detail))
    if expected_map_version is not None and map_value['map_version'] != expected_map_version:
        raise CheckerError('E_TRAJ_MAP_STALE',
                           '地图 v%s，消费者期望 v%s' % (map_value['map_version'],
                                                          expected_map_version))
    if not trajectory:
        raise CheckerError('E_TRAJ_EMPTY', '空轨迹')
    resolution = resolution or map_value['resolution']
    max_step = max_step if max_step is not None else 2.5 * resolution

    surfaces = [s for s in map_value['surfaces'] if s['traversability'] == 'supported']
    # 有向边集合（单向边只登记声明方向）
    edges = set()
    for s in map_value['surfaces']:
        for e in s['edges']:
            edges.add((s['surface_id'], e['to']))
            if e['bidirectional']:
                edges.add((e['to'], s['surface_id']))

    def surface_at(x, y, z):
        for s in surfaces:
            fp = s['footprint']
            if (fp['min_x'] - 1e-9 <= x <= fp['max_x'] + 1e-9
                    and fp['min_y'] - 1e-9 <= y <= fp['max_y'] + 1e-9
                    and abs(z - s['z_reference']) <= z_tol):
                return s
        return None

    violations = []
    prev = None
    prev_surface = None
    for index, point in enumerate(trajectory):
        x, y, z = point[0], point[1], point[2]
        t = point[3] if len(point) > 3 else None
        surface = surface_at(x, y, z)
        if surface is None:
            violations.append(('E_TRAJ_UNSUPPORTED', index,
                               '(%.2f,%.2f,%.2f) 无支撑（悬空或面外）' % (x, y, z)))
        else:
            # 净空：该 XY 列上方向最近面距离
            # 世界坐标精确查询：整数列会把小屋顶的边缘外扩到整米，误拒合法绕行。
            column = rep.column_at(map_value, x, y)
            # A point already resolved to this support may be a sub-micrometre
            # below its reference. That same support is not a ceiling. Keep
            # every OTHER layer, and do not use the broad default z_tol=.1 as
            # permission to erase a genuinely low roof.
            above = [h for h in column.heights() if h > z + 1e-9 and not (
                column.surface_at(h) == surface['surface_id'] and abs(h - z) <= min(z_tol, 1e-6))]
            clearance = min(above) - z if above else None
            if clearance is not None and clearance < min_clearance:
                violations.append(('E_TRAJ_CLEARANCE', index,
                                   '净空 %.2f < %.2f' % (clearance, min_clearance)))
        if prev is not None:
            step = math.hypot(x - prev[0], y - prev[1])
            if surface is not None and prev_surface is not None:
                if surface['surface_id'] != prev_surface['surface_id']:
                    if (prev_surface['surface_id'], surface['surface_id']) not in edges:
                        violations.append(('E_TRAJ_TRANSITION', index,
                                           '%s→%s 无显式边' % (prev_surface['surface_id'],
                                                                 surface['surface_id'])))
                elif step > max_step:
                    violations.append(('E_TRAJ_DISCONTINUITY', index,
                                       '同面步距 %.2f > %.2f' % (step, max_step)))
            if t is not None and prev[3] is not None:
                if t <= prev[3]:
                    violations.append(('E_TRAJ_TIME', index, '时戳非单调'))
                elif time_budget is not None and t - trajectory[0][3] > time_budget:
                    violations.append(('E_TRAJ_TIME', index,
                                       '超预算窗口 %.2f' % (t - trajectory[0][3])))
        prev, prev_surface = (x, y, z, t), surface

    return {'ok': not violations, 'violations': violations,
            'checked_points': len(trajectory)}
