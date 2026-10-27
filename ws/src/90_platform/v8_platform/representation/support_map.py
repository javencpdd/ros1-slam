"""V8-08 首切片：support_map 生产者设计层（R649）。

A3 三判据的设计层落点（23 号 V8-08 行）：
  同 XY 双层不丢层 → HeightColumn 保留全部高度（与旧「同 XY 取最高点」相对）；
  未知不变自由     → EvidenceGrid 只在正向证据时标 free，缺证＝unknown（禁 treat_unknown_as_free）；
  断桥不误连       → edges 只显式声明、绝不从几何邻近推断（suggest_edges 仅咨询性输出）。

extent 语义定界（R631 挂账闭合）：support_map 的 extent＝支撑面足迹并集外包框按
resolution 换算的格数（width/height 整数，≥1），即「证据与面已声明的 XY 区域」；
无支撑面时拒绝出图（extent 无从定义）。

语义解释（本件登记的设计裁定）：map.v1 要求全部面从首面可达（单连通）——
不连通的支撑域（如两栋无桥塔楼）须**按连通分量拆分为多个 map 实例**，
一个 map_id＝一个可通行区域；builder 对不连通输入照契约拒（E_MAP_DISCONNECTED）。

分层：消费契约层（../contracts/validate.py 的 map.v1），不 import ROS、不读写运行目录。
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402  契约层


class RepresentationError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


class HeightColumn:
    """同一 XY 的多高度列：保留全部层，永不折叠（A3-1）。

    旧做法（同 XY 取最高点当三维地图，22 号差异表）在构造上不可表达——本类没有
    「取最高」的入口；查询永远返回全部条目。
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self._layers = {}  # z -> surface_id

    def add(self, z, surface_id):
        if z in self._layers and self._layers[z] != surface_id:
            raise RepresentationError('E_REP_Z_CONFLICT',
                                      '列 (%s,%s) z=%s 已属 %s，另指 %s' % (self.x, self.y, z,
                                                                             self._layers[z], surface_id))
        self._layers[z] = surface_id
        return self

    def heights(self):
        """全部高度（升序）——不丢层的查询面。"""
        return sorted(self._layers)

    def surface_at(self, z):
        return self._layers.get(z)

    def clearance(self, z):
        """z 到其上方最近支撑面的净距；上方无面＝None（开顶，不冒充无限净空）。"""
        above = [h for h in self._layers if h > z + 1e-9]
        return (min(above) - z) if above else None


class EvidenceGrid:
    """空闲证据：free 只来自正向证据，缺证＝unknown（A3-2，禁未知当自由）。"""

    def __init__(self, free_threshold=1):
        self.free_threshold = free_threshold
        self._hits = {}  # (x, y, z) -> free 正向证据计数

    def add_free_evidence(self, x, y, z, count=1):
        self._hits[(x, y, z)] = self._hits.get((x, y, z), 0) + count
        return self

    def state(self, x, y, z):
        """'free'（证据达阈值）或 'unknown'——没有『默认 free』这条路径。"""
        if self._hits.get((x, y, z), 0) >= self.free_threshold:
            return 'free'
        return 'unknown'


def suggest_edges(surfaces, max_xy_gap, max_z_step):
    """咨询性邻接建议：几何邻近＋高差在界内**只产生候选**，绝不自动成边（A3-3）。

    候选必须经上游（建图/人工）确认后才可作为显式 edges 传入 build_support_map。
    """
    candidates = []
    for i, a in enumerate(surfaces):
        for b in surfaces[i + 1:]:
            gap = _footprint_gap(a['footprint'], b['footprint'])
            drop = abs(b['z_reference'] - a['z_reference'])
            if gap <= max_xy_gap and drop <= max_z_step:
                candidates.append({'from': a['surface_id'], 'to': b['surface_id'],
                                   'xy_gap': round(gap, 4), 'z_drop': round(drop, 4),
                                   'advisory_only': True})
    return candidates


def _footprint_gap(fa, fb):
    """两足迹的最小 XY 间距（相交＝0）。"""
    dx = max(fa['min_x'] - fb['max_x'], fb['min_x'] - fa['max_x'], 0.0)
    dy = max(fa['min_y'] - fb['max_y'], fb['min_y'] - fa['max_y'], 0.0)
    return math.hypot(dx, dy)


def build_support_map(map_id, resolution, origin, surfaces, edges, source,
                      map_version=1, unknown_policy='unknown_not_traversable'):
    """由显式面＋显式边构造 map.v1 合法的 support_map（经契约层校验 fail-closed）。

    extent 定界：支撑面足迹并集外包框按 resolution 换算格数（见模块 docstring）。
    surfaces: [{surface_id, z_reference, footprint{min_x,min_y,max_x,max_y},
                traversability, edges[{to,kind,...}] ...}]（edges 挂在源面上）。
    """
    if not surfaces:
        raise RepresentationError('E_REP_NO_SURFACES', '无支撑面，extent 无从定义')
    min_x = min(s['footprint']['min_x'] for s in surfaces)
    max_x = max(s['footprint']['max_x'] for s in surfaces)
    min_y = min(s['footprint']['min_y'] for s in surfaces)
    max_y = max(s['footprint']['max_y'] for s in surfaces)
    extent = {'width': max(1, math.ceil((max_x - min_x) / resolution)),
              'height': max(1, math.ceil((max_y - min_y) / resolution))}
    value = {
        'schema_version': 1,
        'map_id': map_id,
        'map_version': map_version,
        'frame': 'map',
        'kind': 'support_map',
        'resolution': resolution,
        'origin': list(origin),
        'extent': extent,
        'cell_states': ['free', 'occupied', 'unknown'],
        'unknown_policy': unknown_policy,
        'multiple_surfaces_per_xy': True,
        'source': source,
        'surfaces': [
            {'surface_id': s['surface_id'], 'z_reference': s['z_reference'],
             'thickness_m': s.get('thickness_m', 0.05),
             'representation': s.get('representation', 'elevation_grid'),
             'data_uri': s['data_uri'],
             'traversability': s.get('traversability', 'supported'),
             'clearance_min_m': s.get('clearance_min_m', 0.4),
             'footprint': dict(s['footprint']),
             'edges': list(s.get('edges', []))}
            for s in surfaces],
    }
    try:
        contract.validate('map', value)
    except contract.ContractError as error:
        raise RepresentationError('E_REP_CONTRACT', '%s: %s' % (error.code, error.detail))
    return value


def columns_of(value):
    """从 support_map 反查多高度列（A3-1 的可验证查询面）。"""
    columns = {}
    for surface in value['surfaces']:
        fp = surface['footprint']
        for x in range(int(math.floor(fp['min_x'])), int(math.ceil(fp['max_x'])) + 1):
            for y in range(int(math.floor(fp['min_y'])), int(math.ceil(fp['max_y'])) + 1):
                columns.setdefault((x, y), HeightColumn(x, y)).add(
                    surface['z_reference'], surface['surface_id'])
    return columns
