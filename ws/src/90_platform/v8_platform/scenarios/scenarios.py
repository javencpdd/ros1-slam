"""V8-11 场景机读定义框架＋独立评价器（R656 首版；R661 全 14 场景构造器＋kind 分派）。

场景＝机读定义：地图（V8-08 管线产 support_map）＋任务（map.v1 配对）＋真值预期。
**场景定义独立于被测链**；评价器只消费「场景真值 × 被测输出」（A7：不回流、不修复）。
kind 分派（R661）：planning（规划全链）/ tracking（跟踪撤销）/ orchestration（多任务
隔离）/ versioning（版本漂移）/ matrix（组合子矩阵汇总）——由 V8-13 矩阵按类执行。
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'validator'))
import support_map as rep  # noqa: E402
import trajectory_checker as chk  # noqa: E402

RES = 0.5
SOURCE = {'uri': 'design://scenario', 'sha256': '5' * 64}


def _two_layer(z_mezz=0.6, mezz_fp=None, oneway=False):
    mezz_fp = mezz_fp or {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3}
    return rep.build_support_map(
        'scenario_map', RES, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': not oneway}]},
         {'surface_id': 'mezz', 'z_reference': z_mezz, 'data_uri': 'm',
          'footprint': mezz_fp,
          'edges': [] if oneway else [{'to': 'ground', 'kind': 'slope',
                                       'max_slope_deg': 12.0, 'bidirectional': True}]}],
        None, SOURCE)


def _bridge_map():
    """同高天桥：ground_a(左) — bridge(高 0.6) — ground_b(右)；a/b 不重叠（各与 bridge 重叠）。"""
    return rep.build_support_map(
        'bridge_map', RES, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground_a', 'z_reference': 0.0, 'data_uri': 'a',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': -1, 'max_y': 5},
          'edges': [{'to': 'bridge', 'kind': 'step', 'max_step_height_m': 0.7,
                     'bidirectional': True}]},
         {'surface_id': 'bridge', 'z_reference': 0.6, 'data_uri': 'b',
          'footprint': {'min_x': -2, 'min_y': -1, 'max_x': 2, 'max_y': 1},
          'edges': [{'to': 'ground_a', 'kind': 'step', 'max_step_height_m': 0.7,
                     'bidirectional': True},
                    {'to': 'ground_b', 'kind': 'step', 'max_step_height_m': 0.7,
                     'bidirectional': True}]},
         {'surface_id': 'ground_b', 'z_reference': 0.0, 'data_uri': 'c',
          'footprint': {'min_x': 1, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'bridge', 'kind': 'step', 'max_step_height_m': 0.7,
                     'bidirectional': True}]}],
        None, SOURCE)


def _low_roof_map():
    """净空不足：ground 全域＋roof(z=0.15, 小足迹)；roof 可达（step 0.15）。"""
    return rep.build_support_map(
        'low_roof_map', RES, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'roof', 'kind': 'step', 'max_step_height_m': 0.2,
                     'bidirectional': True}]},
         {'surface_id': 'roof', 'z_reference': 0.15, 'data_uri': 'r',
          'footprint': {'min_x': -1, 'min_y': -1, 'max_x': 1, 'max_y': 1},
          'edges': [{'to': 'ground', 'kind': 'step', 'max_step_height_m': 0.2,
                     'bidirectional': True}]}],
        None, SOURCE)


def _goal_task(map_value, start, goal):
    return {
        'schema_version': 1, 'task_id': 'scenario_task',
        'map': {'map_id': map_value['map_id'], 'map_version': map_value['map_version']},
        'frame': map_value['frame'],
        'goal_reference': 'support_surface',
        'start': {'xyz': list(start), 'yaw': 0.0},
        'goal': {'xyz': list(goal), 'yaw': 0.0,
                 'tolerance': {'position_m': 0.3, 'yaw_rad': 0.5, 'source': 'scenario'},
                 'surface_hint': 'mezz', 'surface_disambiguation': 'explicit_hint'},
        'timeout': {'budget_s': 30.0, 'harness_deadline_s': 60.0, 'source': 'scenario'},
    }


SCENARIOS = {
    'SC01': {
        'name': '平地直达', 'kind': 'planning',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (4.0, 4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'], 'forbidden_zones': []},
    },
    'SC02': {
        'name': '坡道爬升（地面→夹层）', 'kind': 'planning',
        'map': lambda: _two_layer(z_mezz=0.6),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
    },
    'SC03': {
        'name': '双层转层（含回程）', 'kind': 'planning',
        'map': lambda: _two_layer(z_mezz=0.6),
        'start': (3.0, 2.0, 0.6), 'goal': (-4.0, -4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['mezz', 'ground'],
                  'forbidden_zones': []},
    },
    'SC04': {
        'name': '短转层段（0.3 m 台阶边）', 'kind': 'planning',
        'map': lambda: _two_layer(z_mezz=0.3, mezz_fp={'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3}),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.3),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
        'note': 'mode3 抽稀吞段对照归运行面（V8-03 B2）；设计层验证台阶边可达',
    },
    'SC05': {
        'name': '同高天桥（非单调剖面：上—跨—下）', 'kind': 'planning',
        'map': lambda: _bridge_map(),
        'start': (-4.0, 0.0, 0.0), 'goal': (4.0, 0.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground_a', 'bridge', 'ground_b'],
                  'forbidden_zones': []},
        'note': '支撑图原生可解（对照 SCAN mode3 平面不可表达，R632 C2/R655 裁决）',
    },
    'SC06': {
        'name': '同 XY 双层列（路径穿夹层下方）', 'kind': 'planning',
        'map': lambda: _two_layer(z_mezz=0.6, mezz_fp={'min_x': 0, 'min_y': -2, 'max_x': 3, 'max_y': 2}),
        'start': (-4.0, 0.0, 0.0), 'goal': (4.0, 0.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'], 'forbidden_zones': []},
        'note': '双层列在支撑图原生保留（对照 SCAN 搜索平面插值落板体，R632 C3）',
    },
    'SC07': {
        'name': '断桥不可达（负例）', 'kind': 'planning',
        'map': lambda: _two_layer(oneway=True),
        'start': (3.0, 2.0, 0.6), 'goal': (-4.0, -4.0, 0.0),
        'truth': {'route_exists': False, 'must_surfaces': [], 'forbidden_zones': [],
                  'expected_reason': 'E_PLAN_NO_ROUTE'},
    },
    'SC08': {
        'name': '单向边限制（负例图的正向可达）', 'kind': 'planning',
        'map': lambda: _two_layer(oneway=True),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
    },
    'SC09': {
        'name': '净空不足（低顶下穿行）', 'kind': 'planning',
        'map': lambda: _low_roof_map(),
        'start': (-4.0, 0.0, 0.0), 'goal': (4.0, 0.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'],
                  'forbidden_zones': [], 'expected_reason': 'E_TRAJ_CLEARANCE'},
        'note': 'eval 期预期失败：净空 0.15 < 0.20（R657 冻结阈值）',
    },
    'SC10': {
        'name': '预算耗尽（负例）', 'kind': 'planning',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': [], 'planner_budget': 3,
                  'expected_reason': 'E_PLAN_BUDGET_EXCEEDED'},
    },
    'SC11': {
        'name': '跟踪偏差撤销（执行期闭环）', 'kind': 'tracking',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (4.0, 4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'], 'forbidden_zones': []},
        'note': '偏差轨迹→tracking 违例→on_rejection→Tracker 达限停',
    },
    'SC12': {
        'name': '多任务隔离（A 失败 B 完成）', 'kind': 'orchestration',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (4.0, 4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'], 'forbidden_zones': []},
        'note': 'A2 判据3：编排双任务隔离（R646 test_task_isolation 同型）',
    },
    'SC13': {
        'name': '障碍动态更新（版本漂移拒判）', 'kind': 'versioning',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
        'note': 'MapService 升版后消费者持旧版→E_TRAJ_MAP_STALE（R650/R653）',
    },
    'SC14': {
        'name': '组合矩阵回归（planning 子集汇总）', 'kind': 'matrix',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
        'note': 'V8-13 planning 类场景×组合×双轨子矩阵（不含 matrix 类自身）',
    },
}


def scenario(name):
    spec = SCENARIOS[name]
    map_value = spec['map']()
    return {'id': name, 'name': spec['name'], 'map': map_value,
            'task': _goal_task(map_value, spec['start'], spec['goal']),
            'start': spec['start'], 'goal': spec['goal'],
            'truth': dict(spec['truth']), 'kind': spec.get('kind', 'planning'),
            'planner_budget': spec['truth'].get('planner_budget')}


def evaluate(scenario_def, trajectory, use_truth_map=None):
    """独立评价器：只消费场景真值与被测输出（A7：不回流、不自动修复）。"""
    map_value = use_truth_map if use_truth_map is not None else scenario_def['map']
    verdict = chk.check(map_value, trajectory)
    result = {'scenario': scenario_def['id'], 'ok': verdict['ok'],
              'check_violations': verdict['violations'], 'failures': []}
    if not verdict['ok']:
        result['failures'].append('checker:%s' % verdict['violations'][0][0])
        return result
    visited = set()
    for x, y, z in trajectory:
        for s in map_value['surfaces']:
            fp = s['footprint']
            if (fp['min_x'] - 1e-9 <= x <= fp['max_x'] + 1e-9
                    and fp['min_y'] - 1e-9 <= y <= fp['max_y'] + 1e-9
                    and abs(z - s['z_reference']) <= 0.1):
                visited.add(s['surface_id'])
                break
    missing = set(scenario_def['truth']['must_surfaces']) - visited
    if missing:
        result['failures'].append('must_surfaces:%s' % ','.join(sorted(missing)))
    gx, gy, gz = scenario_def['goal']
    end = trajectory[-1]
    tol = scenario_def['task']['goal']['tolerance']['position_m']
    if (abs(end[0] - gx) > tol or abs(end[1] - gy) > tol
            or abs(end[2] - gz) > max(tol, 0.15)):
        result['failures'].append('goal_not_reached')
    result['ok'] = not result['failures']
    return result
