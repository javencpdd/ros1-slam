"""V8-11 首切片：场景机读定义框架＋独立评价器骨架（R656）。

场景＝机读定义：地图（V8-08 elevation/support_map 管线产）＋任务（map.v1 配对）＋
真值预期（预期路线存在性/到达/禁区）。**场景定义独立于被测链**（被测组合不参与
构造），评价器只消费「场景真值 × 被测输出」——评价数据不回流被测链（A7 面：
错误路径不能被载体自动修正掩盖）。

SC01—SC14 清单骨架：本批落 SC01（平地直达）、SC02（坡道单层爬升）、SC03（双层转层）
三场景全链；SC04—SC14 槽位登记（名称与判定面）待后续批次填充。
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'validator'))
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402
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


def _goal_task(map_value, start, goal, ref='plane_xy' if False else 'support_surface'):
    """task.v1 配对夹具（与 map 配对：kind/frame/map_id/version 一致）。"""
    return {
        'schema_version': 1, 'task_id': 'scenario_task',
        'map': {'map_id': map_value['map_id'], 'map_version': map_value['map_version']},
        'frame': map_value['frame'],
        'goal_reference': ref,
        'start': {'xyz': list(start), 'yaw': 0.0},
        'goal': {'xyz': list(goal), 'yaw': 0.0,
                 'tolerance': {'position_m': 0.3, 'yaw_rad': 0.5, 'source': 'scenario'},
                 'surface_hint': 'mezz' if ref == 'support_surface' else None,
                 'surface_disambiguation': 'explicit_hint' if ref == 'support_surface' else 'not_applicable'},
        'timeout': {'budget_s': 30.0, 'harness_deadline_s': 60.0, 'source': 'scenario'},
    }


SCENARIOS = {
    'SC01': {
        'name': '平地直达',
        'map': lambda: _two_layer(),
        'start': (-4.0, -4.0, 0.0), 'goal': (4.0, 4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['ground'],
                  'forbidden_zones': []},
    },
    'SC02': {
        'name': '坡道爬升（地面→夹层）',
        'map': lambda: _two_layer(z_mezz=0.6),
        'start': (-4.0, -4.0, 0.0), 'goal': (3.0, 2.0, 0.6),
        'truth': {'route_exists': True, 'must_surfaces': ['ground', 'mezz'],
                  'forbidden_zones': []},
    },
    'SC03': {
        'name': '双层转层（含回程）',
        'map': lambda: _two_layer(z_mezz=0.6),
        'start': (3.0, 2.0, 0.6), 'goal': (-4.0, -4.0, 0.0),
        'truth': {'route_exists': True, 'must_surfaces': ['mezz', 'ground'],
                  'forbidden_zones': []},
    },
    # SC04—SC14 槽位（登记名与判定面，待后续批次填充构造器）
    'SC04': {'name': '短转层段（<0.5 m）', 'slot': True,
             'verdict_face': 'mode3 抽稀吞段对照（V8-03 B2）'},
    'SC05': {'name': '同高天桥（非单调剖面）', 'slot': True,
             'verdict_face': '备选裁决（R655）'},
    'SC06': {'name': '同 XY 双层列', 'slot': True, 'verdict_face': '备选裁决＋检查器'},
    'SC07': {'name': '断桥不可达', 'slot': True, 'verdict_face': '负例无路（E_PLAN_NO_ROUTE）'},
    'SC08': {'name': '单向边限制', 'slot': True, 'verdict_face': '方向敏感负例'},
    'SC09': {'name': '净空不足', 'slot': True, 'verdict_face': 'E_TRAJ_CLEARANCE'},
    'SC10': {'name': '预算耗尽', 'slot': True, 'verdict_face': 'E_PLAN_BUDGET_EXCEEDED'},
    'SC11': {'name': '跟踪偏差撤销', 'slot': True, 'verdict_face': '跟踪闭环＋达限停'},
    'SC12': {'name': '多任务隔离', 'slot': True, 'verdict_face': 'A2 判据3'},
    'SC13': {'name': '障碍动态更新', 'slot': True, 'verdict_face': '版本升版＋漂移拒判'},
    'SC14': {'name': '组合矩阵回归', 'slot': True, 'verdict_face': 'V8-13 矩阵'},
}


def scenario(name):
    spec = SCENARIOS[name]
    if spec.get('slot'):
        raise KeyError('SC04—SC14 为登记槽位，构造器待后续批次（%s）' % spec['name'])
    map_value = spec['map']()
    return {'id': name, 'name': spec['name'], 'map': map_value,
            'task': _goal_task(map_value, spec['start'], spec['goal']),
            'start': spec['start'], 'goal': spec['goal'], 'truth': dict(spec['truth'])}


def evaluate(scenario_def, trajectory, use_truth_map=None):
    """独立评价器：只消费场景真值与被测输出（A7：不回流、不自动修正）。

    判定面：①路线/轨迹过检查器（支撑/净空/连续安全）；②途经面集⊇must_surfaces；
    ③终点到达（goal 容差）。任何一项不过＝FAIL（附理由），不修复轨迹。
    """
    map_value = use_truth_map if use_truth_map is not None else scenario_def['map']
    verdict = chk.check(map_value, trajectory)
    result = {'scenario': scenario_def['id'], 'ok': verdict['ok'],
              'check_violations': verdict['violations'], 'failures': []}
    if not verdict['ok']:
        result['failures'].append('checker')
        return result
    # 途经面集：由评价器独立重算（不信任被测输出自报的面标签）
    visited = set()
    for x, y, z in trajectory[:1] + trajectory:
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
