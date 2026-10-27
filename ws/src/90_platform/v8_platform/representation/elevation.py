"""V8-08 第三切片：高程建图算法设计层（R651）。

全链：合成点云 → XY 格桶 → z 聚类分层（**全部保留，不丢层**）→ 同层连通聚类成面
（小簇拒绝，噪声不冒充支撑）→ edges 咨询建议（advisory_only，绝不自动成边）。
传感面（真实雷达/深度数据、滤波与配准）归仿真阶段；本层以合成点云验证算法语义。

A3 落点：extract 层同 XY 多 z 簇全保留（数据级不丢层）；面提取要求最小支撑域
（min_cells），孤立小簇如实列入 dropped_clusters 不冒充；edges 只经 suggest_edges
出咨询候选。
"""
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import support_map as rep  # noqa: E402


def _cluster_z(sorted_zs, tol):
    """相邻 gap>tol 即分层；返回 [(z 均值, 点数), ...]。"""
    clusters = []
    current = [sorted_zs[0]]
    for z in sorted_zs[1:]:
        if z - current[-1] > tol:
            clusters.append(current)
            current = [z]
        else:
            current.append(z)
    clusters.append(current)
    return [(round(sum(c) / len(c), 6), len(c)) for c in clusters]


def extract_columns(points, resolution, z_tol=0.05):
    """点云→XY 格桶→z 聚类分层。同格多簇**全保留**（A3-1 数据级保证）。"""
    cells = defaultdict(list)
    for x, y, z in points:
        cells[(int(x // resolution), int(y // resolution))].append(z)
    return {cell: _cluster_z(sorted(zs), z_tol) for cell, zs in cells.items()}


def _connected_cells(cells):
    """格集合的 8 邻接连通分量。"""
    remaining = set(cells)
    components = []
    while remaining:
        start = remaining.pop()
        stack, component = [start], [start]
        while stack:
            cx, cy = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nxt = (cx + dx, cy + dy)
                    if nxt in remaining:
                        remaining.remove(nxt)
                        stack.append(nxt)
                        component.append(nxt)
        components.append(sorted(component))
    return components


def extract_surfaces(columns, resolution, z_tol=0.05, min_cells=4):
    """分层→同层连通聚类→支撑面（足迹＝格并集外接框）；小簇拒绝并如实报数。

    返回 (surfaces, dropped)：surfaces 元素含 surface_id/z_reference/footprint/data_uri
    （适配 build_support_map 输入形）；dropped＝被拒小簇 [(z, 格数), ...]。
    """
    bands = defaultdict(list)  # z 层值 -> [(cell), ...]
    for cell, layers in columns.items():
        for z_mean, _count in layers:
            bands[z_mean].append(cell)
    surfaces, dropped = [], []
    for z_mean in sorted(bands):
        for component in _connected_cells(bands[z_mean]):
            if len(component) < min_cells:
                dropped.append((z_mean, len(component)))
                continue
            xs = [c[0] for c in component]
            ys = [c[1] for c in component]
            surfaces.append({
                'surface_id': 's%d' % len(surfaces),
                'z_reference': round(z_mean, 6),
                'footprint': {'min_x': min(xs) * resolution,
                              'min_y': min(ys) * resolution,
                              'max_x': (max(xs) + 1) * resolution,
                              'max_y': (max(ys) + 1) * resolution},
                'data_uri': 'elevation://s%d' % len(surfaces),
            })
    return surfaces, dropped


def mapping_pipeline(points, resolution, z_tol=0.05, min_cells=4,
                     advisory_gap=0.0, advisory_step=0.5):
    """全链：点云→列→面→edges 咨询建议（不自动成边）。"""
    columns = extract_columns(points, resolution, z_tol)
    surfaces, dropped = extract_surfaces(columns, resolution, z_tol, min_cells)
    advisory = rep.suggest_edges(surfaces, advisory_gap, advisory_step)
    return {'columns': {str(k): v for k, v in sorted(columns.items())},
            'surfaces': surfaces,
            'advisory_edges': advisory,
            'dropped_clusters': dropped}


def to_height_columns(result):
    """管线结果→HeightColumn 视图（面已赋 id；被拒小簇层仍保留在列上，数据不丢）。"""
    columns = {}
    for cell_text, layers in result['columns'].items():
        cx, cy = (int(p) for p in cell_text.strip('()').split(','))
        column = rep.HeightColumn(cx, cy)
        surface_z = {s['z_reference']: s['surface_id'] for s in result['surfaces']}
        for z_mean, _count in layers:
            column.add(z_mean, surface_z.get(z_mean, 'unsurfaced'))
        columns[(cx, cy)] = column
    return columns
