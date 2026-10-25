"""V8-13：双轨闭环与组合回归矩阵（R658）。

双轨：真值轨（规划器吃真值图）vs 估计轨（规划器吃**确定性扰动图**——夹层 z 偏移
0.05 m，在检查器 z 容差内：估计误差不至假失败、也不被静默吞）；两轨的轨迹都按
**真值图**评价（A6：估计与真值明确分开；A7：评价器独立）。
组合：V8-12 冻结分母的组合四款（hdl/lite3 × dwa/teb）——设计层规划-检查链对后端
无关（后端差异在 ROS 层），组合以 composer 锁 digest 记入每个 cell（矩阵可追溯到
锁面），运行面差异归仿真阶段，如实不冒称。
失败定位：逐 cell 记录首个失败阶段（map/plan/check/eval）与理由码——负例场景
（SC07/SC10）的预期失败＝expected_failure（通过＝按预期失败）。
"""
import copy
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scenarios'))
sys.path.insert(0, str(HERE.parent / 'calibration'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'composer'))
sys.path.insert(0, str(HERE.parent / 'registry'))
import calibration as cal  # noqa: E402
import composer as comp  # noqa: E402
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402

MDIR = HERE.parent / 'registry' / 'data' / 'manifests'
EIGHT = ['v8.odometry.faster_lio', 'v8.global_localization.lite3', 'v8.map_server.apt',
         'v8.representation.costmap_layers', 'v8.local_planner.scan_planner',
         'v8.global_planner.apt', 'v8.controller.dwa_adapter', 'v8.safety_arbiter.lite3']
PERTURB_Z = 0.05  # 估计轨确定性扰动（< z 支撑容差 0.10，R657 冻结）


def _perturbed(map_value):
    bumped = copy.deepcopy(map_value)
    for s in bumped['surfaces']:
        if s['surface_id'] == 'mezz':
            s['z_reference'] = round(s['z_reference'] + PERTURB_Z, 6)
    return bumped


def _combo_locks():
    store = comp.load(MDIR)
    locks = {}
    for localization in ('v8.global_localization.lite3', 'v8.global_localization.hdl'):
        for controller in ('v8.controller.dwa_adapter', 'v8.controller.teb_adapter'):
            ids = [localization if c == 'v8.global_localization.lite3'
                   and c in EIGHT else c for c in EIGHT]
            ids = [controller if c == 'v8.controller.dwa_adapter' else c for c in ids]
            name = '%s+%s' % (localization.split('.')[-1],
                              controller.split('.')[-1].replace('_adapter', ''))
            locks[name] = comp.lock(store, ids)['digest']
    return locks


def run_matrix(run_class='official'):
    """跑冻结分母：正式集场景 × 组合四款 × 双轨。返回逐 cell 记录与汇总。"""
    cal.validate_frozen()
    cells = []
    for scenario_id in cal.FROZEN['denominators']['official_scenarios']['value']:
        case = scn.scenario(scenario_id)
        for combo, lock_digest in _combo_locks().items():
            for track in cal.FROZEN['denominators']['tracks']['value']:
                cells.append(_run_cell(case, combo, lock_digest, track, run_class))
    summary = {
        'run_class': run_class,
        'cells': len(cells),
        'passed': sum(1 for c in cells if c['outcome'] == 'pass'),
        'expected_failures': sum(1 for c in cells if c['outcome'] == 'expected_failure'),
        'unexpected_failures': sum(1 for c in cells if c['outcome'] == 'unexpected_failure'),
        'calibration_runs_count_official':
            cal.FROZEN['rerun_rules']['calibration_runs_count_official']['value'],
    }
    return {'summary': summary, 'cells': cells}


def _run_cell(case, combo, lock_digest, track, run_class):
    cell = {'scenario': case['id'], 'combo': combo, 'track': track,
            'lock_digest': lock_digest[:12], 'run_class': run_class,
            'stage': None, 'reason': None, 'outcome': None}
    map_for_plan = _perturbed(case['map']) if track == 'estimate' else case['map']
    try:
        route = pln.plan(map_for_plan, case['start'], case['goal'],
                         budget=case.get('planner_budget') or
                         cal.FROZEN['budgets']['planner_max_expansions']['value'])
        trajectory = [(x, y, z) for x, y, z, _sid in route['path']]
    except pln.PlannerError as error:
        cell.update(stage='plan', reason=error.code)
        cell['outcome'] = ('expected_failure'
                           if error.code == case['truth'].get('expected_reason')
                           else 'unexpected_failure')
        return cell
    verdict = scn.evaluate(case, trajectory)  # 恒按真值图评价（A6/A7）
    if verdict['ok']:
        cell.update(stage='eval', reason='ok', outcome='pass')
    else:
        cell.update(stage='eval', reason=';'.join(verdict['failures']) or 'checker',
                    outcome='unexpected_failure')
    return cell


def main(argv):
    if argv and argv[0] == 'run':
        result = run_matrix(argv[1] if len(argv) > 1 else 'official')
        print(json.dumps(result['summary'], ensure_ascii=False))
        for cell in result['cells']:
            if cell['outcome'] != 'pass':
                print(json.dumps(cell, ensure_ascii=False))
        return 0 if result['summary']['unexpected_failures'] == 0 else 5
    print('usage: regression.py run [official|calibration]', file=sys.stderr)
    return 9


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
