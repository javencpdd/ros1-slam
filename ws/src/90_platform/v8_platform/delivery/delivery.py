"""V8-14：性能定界与复现交付（R659）。

规模曲线：合成双面图（地面 N×N 格＋夹层）在 N∈{10,20,40,80} 的规划器展开数与
路径长——设计层以「展开节点数」为规模代理（确定性、可复跑；墙钟时间受机器噪声，
如实不冒称），并验证展开数有界（≤节点总数×常数）。
复现清单：交付树（本包内九模块）逐件 sha256 manifest——清单再生成逐字节
一致（复现性自证）；用户指南见本包 README.md；计划书见 note/计划书v8/README.md。
未接入说明：apt 后端（map_server/global/dwa/teb）与 SCAN 运行面未在本设计层接入
（manifest 声明在册、组合/接线经设计层验证；实装运行归仿真阶段）。
"""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402

RES = 0.5
SOURCE = {'uri': 'design://scale', 'sha256': '7' * 64}
DELIVERY_DIRS = ('contracts', 'probe_mode3', 'registry', 'composer', 'orchestrator',
                 'adapters', 'representation', 'planner', 'validator', 'scenarios',
                 'calibration', 'regression', 'delivery')


def scale_map(n_cells):
    half = n_cells * RES / 2
    return rep.build_support_map(
        'scale_%d' % n_cells, RES, [-half, -half, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -half, 'min_y': -half, 'max_x': half, 'max_y': half},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]},
         {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm',
          'footprint': {'min_x': half - 2, 'min_y': half - 2, 'max_x': half, 'max_y': half},
          'edges': [{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]}],
        None, SOURCE)


def scale_curve(sizes=(10, 20, 40, 80)):
    """逐规模跑对角规划；记录展开数/路径点数/长度，并断言展开有界（≤2×格数²）。"""
    curve = []
    for n in sizes:
        value = scale_map(n)
        half = n * RES / 2
        route = pln.plan(value, (-half + RES, -half + RES, 0.0),
                         (half - RES, half - RES, 0.6))
        bound = 2 * n * n
        if route['expansions'] > bound:
            raise ValueError('规模 %d 展开 %d 超界 %d（性能定界失败）'
                             % (n, route['expansions'], bound))
        curve.append({'cells_per_side': n, 'nodes': n * n,
                      'expansions': route['expansions'],
                      'path_points': len(route['path']),
                      'path_length': route['length']})
    return curve


def manifest(root=None):
    """交付树逐件 sha256 清单（九模块＋本模块）；确定性排序。"""
    root = Path(root) if root else HERE.parent
    entries = {}
    for module in DELIVERY_DIRS:
        base = root / module
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                entries[str(path.relative_to(root))] = hashlib.sha256(
                    path.read_bytes()).hexdigest()
    return entries


def main(argv):
    if argv and argv[0] == 'curve':
        print(json.dumps(scale_curve(), ensure_ascii=False, indent=1))
        return 0
    if argv and argv[0] == 'manifest':
        entries = manifest()
        print(json.dumps({'files': len(entries),
                          'digest': hashlib.sha256(json.dumps(
                              entries, sort_keys=True).encode()).hexdigest()},
                         ensure_ascii=False))
        return 0
    print('usage: delivery.py curve|manifest', file=sys.stderr)
    return 9


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
