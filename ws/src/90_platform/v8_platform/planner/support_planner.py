"""V8-09 首切片：有界 A* 支撑图全局规划设计层（R652）。

图模型（消费 map.v1 support_map，由 representation 生产者构造）：
  节点＝(surface_id, ix, iy)——支撑面足迹内、traversability=supported 的栅格；
  邻接＝面内 8 邻（不出足迹）；**跨面只走显式声明的 edges**（断桥不误连的规划层落点），
  bidirectional=false 的边只可 to 方向通行；桥接＝两足迹最近格对。
预算＝最大展开节点数（超限＝E_PLAN_BUDGET_EXCEEDED，拒绝而非「无路」）；
取消＝每次弹出检查谓词（E_PLAN_CANCELLED）；诊断＝展开数/路径/分段/转层记录
（分段即 V8-03「切单调段」主路线的规划层供给）。
A4 落点：路径每点落在其分段面的支撑足迹内（正例有支撑且连续安全）；
负例＝目标无面/目标在不可通行面/单向边不可达/预算耗尽/取消——各有理由码不冒充无路。
"""
import heapq
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402


class PlannerError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _cells(footprint, resolution):
    x0 = int(math.ceil(footprint['min_x'] / resolution))
    x1 = int(math.ceil(footprint['max_x'] / resolution)) - 1
    y0 = int(math.ceil(footprint['min_y'] / resolution))
    y1 = int(math.ceil(footprint['max_y'] / resolution)) - 1
    return {(ix, iy) for ix in range(x0, x1 + 1) for iy in range(y0, y1 + 1)}


def _cell_center(ix, iy, resolution):
    return (ix + 0.5) * resolution, (iy + 0.5) * resolution


def _nearest_cell_pair(cells_a, cells_b):
    best, best_d = None, float('inf')
    for ca in cells_a:
        for cb in cells_b:
            d = math.hypot(ca[0] - cb[0], ca[1] - cb[1])
            if d < best_d:
                best, best_d = (ca, cb), d
    return best


def plan(value, start, goal, budget=20000, cancel=None, resolution=None,
         min_clearance_m=None):
    """start/goal＝(x, y, z)；多层面同 XY 时按 z 最近面消歧。返回路径与诊断。"""
    try:
        contract.validate('map', value)
    except contract.ContractError as error:
        raise PlannerError('E_PLAN_MAP_INVALID', '%s: %s' % (error.code, error.detail))
    resolution = resolution or value['resolution']
    surfaces = {s['surface_id']: s for s in value['surfaces']}
    if min_clearance_m is not None and (
            isinstance(min_clearance_m, bool) or not isinstance(min_clearance_m, (int, float))
            or not math.isfinite(min_clearance_m) or min_clearance_m < 0):
        raise PlannerError('E_PLAN_INPUT', 'min_clearance_m must be finite and nonnegative')

    def has_clearance(x, y, z):
        if min_clearance_m is None:
            return True
        for surface in value['surfaces']:
            box = surface['footprint']
            if (surface['z_reference'] > z + 1e-9
                    and box['min_x'] <= x <= box['max_x']
                    and box['min_y'] <= y <= box['max_y']
                    and surface['z_reference'] - z < min_clearance_m - 1e-9):
                return False
        return True

    def segment_has_clearance(a, b, z):
        """面内边也核验连续线段，防止细低顶落在两个栅格中心之间。"""
        if min_clearance_m is None:
            return True
        for surface in value['surfaces']:
            height = surface['z_reference'] - z
            if not (1e-9 < height < min_clearance_m - 1e-9):
                continue
            box = surface['footprint']
            lo, hi = 0.0, 1.0
            for axis, lower, upper in ((0, 'min_x', 'max_x'), (1, 'min_y', 'max_y')):
                delta = b[axis] - a[axis]
                if abs(delta) <= 1e-12:
                    if not box[lower] <= a[axis] <= box[upper]:
                        lo, hi = 1.0, 0.0
                        break
                else:
                    near, far = sorted(((box[lower] - a[axis]) / delta,
                                        (box[upper] - a[axis]) / delta))
                    lo, hi = max(lo, near), min(hi, far)
            if lo <= hi:
                return False
        return True

    def resolve(point, label):
        px, py, pz = point
        candidates = [(abs(s['z_reference'] - pz), s) for s in value['surfaces']
                      if s['traversability'] == 'supported'
                      and s['footprint']['min_x'] <= px <= s['footprint']['max_x']
                      and s['footprint']['min_y'] <= py <= s['footprint']['max_y']]
        if not candidates:
            raise PlannerError('E_PLAN_GOAL_INVALID', '%s 不在任何支撑面足迹内' % label)
        return min(candidates, key=lambda c: c[0])[1]

    start_surface, goal_surface = resolve(start, 'start'), resolve(goal, 'goal')

    for label, point, surface in (('start', start, start_surface), ('goal', goal, goal_surface)):
        if not has_clearance(point[0], point[1], surface['z_reference']):
            raise PlannerError('E_PLAN_CLEARANCE', '%s clearance < %.6g m' % (label, min_clearance_m))

    cells = {sid: _cells(s['footprint'], resolution) for sid, s in surfaces.items()
             if s['traversability'] == 'supported'}
    if min_clearance_m is not None:
        cells = {sid: {(ix, iy) for ix, iy in layer
                       if has_clearance(*_cell_center(ix, iy, resolution),
                                        surfaces[sid]['z_reference'])}
                 for sid, layer in cells.items()}

    def node_of(point, surface):
        ix = int(math.floor(point[0] / resolution))
        iy = int(math.floor(point[1] / resolution))
        if (ix, iy) not in cells[surface['surface_id']]:
            ix = min(max(ix, int(math.ceil(surface['footprint']['min_x'] / resolution))),
                     int(math.ceil(surface['footprint']['max_x'] / resolution)) - 1)
            iy = min(max(iy, int(math.ceil(surface['footprint']['min_y'] / resolution))),
                     int(math.ceil(surface['footprint']['max_y'] / resolution)) - 1)
        if (ix, iy) not in cells[surface['surface_id']]:
            raise PlannerError('E_PLAN_CLEARANCE', 'endpoint cell has insufficient clearance')
        return (surface['surface_id'], ix, iy)

    start_node = node_of(start, start_surface)
    goal_node = node_of(goal, goal_surface)

    # 跨面桥：显式 edges → 最近格对（双向按声明）
    bridges = {}
    for sid, s in surfaces.items():
        if s['traversability'] != 'supported':
            continue
        for edge in s['edges']:
            other = edge['to']
            if other not in cells or sid not in cells or not cells[sid] or not cells[other]:
                continue
            pair = _nearest_cell_pair(cells[sid], cells[other])
            bridges.setdefault((sid, other), set()).add(pair)
            if edge['bidirectional']:
                bridges.setdefault((other, sid), set()).add((pair[1], pair[0]))

    def neighbors(node):
        sid, ix, iy = node
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                nxt = (ix + dx, iy + dy)
                if (nxt in cells[sid]
                        and segment_has_clearance(_cell_center(ix, iy, resolution),
                                                  _cell_center(*nxt, resolution),
                                                  surfaces[sid]['z_reference'])):
                    out.append((sid, nxt[0], nxt[1]))
        for (a, b), pairs in bridges.items():
            if a == sid:
                for ca, cb in pairs:
                    if (ix, iy) == ca:
                        out.append((b, cb[0], cb[1]))
        return out

    gx, gy = goal_node[1], goal_node[2]

    def h(node):
        return math.hypot(node[1] - gx, node[2] - gy) * resolution

    open_heap = [(h(start_node), 0.0, start_node)]
    came, g_score = {}, {start_node: 0.0}
    expansions = 0
    while open_heap:
        if cancel is not None and cancel():
            raise PlannerError('E_PLAN_CANCELLED', '展开 %d 节点后取消' % expansions)
        _, g, node = heapq.heappop(open_heap)
        if node == goal_node:
            result = _result(node, came, surfaces, resolution, expansions)
            if min_clearance_m is not None:
                result['policy'] = {'min_clearance_m': min_clearance_m}
            return result
        expansions += 1
        if expansions > budget:
            raise PlannerError('E_PLAN_BUDGET_EXCEEDED',
                               '展开 %d 超预算 %d（拒绝，非无路）' % (expansions, budget))
        for nxt in neighbors(node):
            step = math.hypot(nxt[1] - node[1], nxt[2] - node[2]) * resolution
            if nxt[0] != node[0]:
                step += 0.1  # 转层固定附加成本（显式边）
            ng = g + step
            if ng < g_score.get(nxt, float('inf')) - 1e-12:
                g_score[nxt] = ng
                came[nxt] = node
                heapq.heappush(open_heap, (ng + h(nxt), ng, nxt))
    raise PlannerError('E_PLAN_NO_ROUTE',
                       '起点 %s 到目标 %s 无路（展开 %d）' % (start_node, goal_node, expansions))


def _result(node, came, surfaces, resolution, expansions):
    chain = []
    while node is not None:
        chain.append(node)
        node = came.get(node)
    chain.reverse()
    path, segments, transitions = [], [], []
    for sid, ix, iy in chain:
        x, y = _cell_center(ix, iy, resolution)
        path.append((round(x, 6), round(y, 6), surfaces[sid]['z_reference'], sid))
    run = [chain[0]]
    for prev, nxt in zip(chain, chain[1:]):
        if nxt[0] != prev[0]:
            segments.append({'surface': prev[0], 'cells': len(run)})
            transitions.append({'from': prev[0], 'to': nxt[0]})
            run = [nxt]
        else:
            run.append(nxt)
    segments.append({'surface': chain[-1][0], 'cells': len(run)})
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) * resolution
                 for a, b in zip(chain, chain[1:])) + 0.1 * len(transitions)
    return {'ok': True, 'path': path, 'segments': segments, 'transitions': transitions,
            'expansions': expansions, 'length': round(length, 6)}
