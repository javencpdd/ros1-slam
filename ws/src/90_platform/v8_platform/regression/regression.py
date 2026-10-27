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
sys.path.insert(0, str(HERE.parent / 'orchestrator'))
sys.path.insert(0, str(HERE.parent / 'validator'))
sys.path.insert(0, str(HERE.parent / 'representation'))
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
    """跑冻结分母（v2 全 14 场景）：按 kind 分派 × 组合四款 × 双轨。"""
    cal.validate_frozen()
    cells = []
    locks = _combo_locks()
    for scenario_id in cal.FROZEN['denominators']['official_scenarios']['value']:
        case = scn.scenario(scenario_id)
        for combo, lock_digest in locks.items():
            for track in cal.FROZEN['denominators']['tracks']['value']:
                cells.append(_dispatch(case, combo, lock_digest, track, run_class))
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


def _dispatch(case, combo, lock_digest, track, run_class):
    """kind 分派：planning→全链 cell；tracking/orchestration/versioning→各自闭环 cell；
    matrix→planning 子集汇总 cell（不含 matrix 自身，防递归）。"""
    runner = {'planning': _run_cell, 'tracking': _run_tracking_cell,
              'orchestration': _run_orchestration_cell, 'versioning': _run_versioning_cell,
              'matrix': _run_matrix_cell}.get(case['kind'])
    if runner is None:
        raise ValueError('unknown kind %s' % case['kind'])
    return runner(case, combo, lock_digest, track, run_class)


def _cell(case, combo, lock_digest, track, run_class, stage, reason, outcome):
    return {'scenario': case['id'], 'kind': case['kind'], 'combo': combo, 'track': track,
            'lock_digest': lock_digest[:12], 'run_class': run_class,
            'stage': stage, 'reason': reason, 'outcome': outcome}


def _run_tracking_cell(case, combo, lock_digest, track, run_class):
    """SC11：planner 参考→注入偏差→tracking 违例→on_rejection→Tracker 达限停。"""
    import enforcement as enf
    import support_planner as pln
    import tracking as trk
    route = pln.plan(case['map'], case['start'], case['goal'])
    reference = [(x, y, z) for x, y, z, _sid in route['path']]
    mid = reference[len(reference) // 2]
    deviations = trk.check_tracking(reference, (mid[0] + 0.5, mid[1], mid[2]))
    if not deviations:
        return _cell(case, combo, lock_digest, track, run_class, 'tracking',
                     'no_deviation_detected', 'unexpected_failure')
    action = enf.on_rejection({'ok': False, 'violations': deviations})
    tracker = enf.RejectionTracker(limit=2)
    tracker.record({'ok': False, 'violations': deviations})
    stopped = tracker.record({'ok': False, 'violations': deviations}) == 'stop'
    ok = action['action'] == 'stop_chain' and stopped
    return _cell(case, combo, lock_digest, track, run_class, 'tracking',
                 'deviation→stop_chain+tracker_stop' if ok else 'chain_broken',
                 'pass' if ok else 'unexpected_failure')


def _run_orchestration_cell(case, combo, lock_digest, track, run_class):
    """SC12：双任务隔离——A 崩溃退净、B 正常完成（A2 判据3）。"""
    import orchestrator as orch
    specs = [orch.ComponentSpec('a.scan', ['local_planner']),
             orch.ComponentSpec('a.odo', ['odometry']),
             orch.ComponentSpec('b.scan', ['local_planner']),
             orch.ComponentSpec('b.odo', ['odometry'])]
    engine = orch.Orchestrator()
    task_a = engine.submit('A', [s for s in specs if s.id.startswith('a.')])
    task_b = engine.submit('B', [s for s in specs if s.id.startswith('b.')])
    runners = {s.id: orch.SimulatedRunner(s) for s in specs}
    engine.start(task_a, {k: v for k, v in runners.items() if k.startswith('a.')})
    engine.start(task_b, {k: v for k, v in runners.items() if k.startswith('b.')})
    runners['a.scan'].die()
    engine.health_check(task_a, {k: v for k, v in runners.items() if k.startswith('a.')})
    b_alive = all(r.is_alive() for k, r in runners.items() if k.startswith('b.'))
    engine.complete(task_b, {k: v for k, v in runners.items() if k.startswith('b.')})
    ok = (task_a.state == orch.TaskState.FAILED and b_alive
          and task_b.state == orch.TaskState.COMPLETED
          and all(not r.is_alive() for r in runners.values()))
    return _cell(case, combo, lock_digest, track, run_class, 'orchestration',
                 'A_failed_B_completed_clean' if ok else 'isolation_broken',
                 'pass' if ok else 'unexpected_failure')


def _run_versioning_cell(case, combo, lock_digest, track, run_class):
    """SC13：MapService 升版→消费者持旧版→E_TRAJ_MAP_STALE 拒判。"""
    import copy as _copy
    import map_service as svc
    import trajectory_checker as chk
    service = svc.MapService(case['map']['map_id'])
    service.publish(case['map'])
    stale_map = _copy.deepcopy(case['map'])
    bumped = _copy.deepcopy(case['map'])
    for s in bumped['surfaces']:
        if s['surface_id'] == 'mezz':
            s['z_reference'] = 0.8
    service.update(bumped)
    route = __import__('support_planner').plan(stale_map, case['start'], case['goal'])
    try:
        chk.check(stale_map, [(x, y, z) for x, y, z, _sid in route['path']],
                  expected_map_version=service.current['map_version'])
        return _cell(case, combo, lock_digest, track, run_class, 'versioning',
                     'stale_not_detected', 'unexpected_failure')
    except chk.CheckerError as error:
        ok = error.code == 'E_TRAJ_MAP_STALE'
        return _cell(case, combo, lock_digest, track, run_class, 'versioning',
                     error.code if ok else 'wrong:%s' % error.code,
                     'pass' if ok else 'unexpected_failure')


def _run_matrix_cell(case, combo, lock_digest, track, run_class):
    """SC14：planning 子集（SC01—SC10、SC13）× 单组合 lite3+dwa × 真值轨汇总。"""
    planning_ids = [sid for sid in
                    cal.FROZEN['denominators']['official_scenarios']['value']
                    if scn.SCENARIOS[sid].get('kind', 'planning') == 'planning']
    sub = []
    for sid in planning_ids:
        sub_case = scn.scenario(sid)
        sub.append(_run_cell(sub_case, combo, lock_digest, 'truth', run_class))
    passed = sum(1 for c in sub if c['outcome'] == 'pass')
    expected = sum(1 for c in sub if c['outcome'] == 'expected_failure')
    unexpected = sum(1 for c in sub if c['outcome'] == 'unexpected_failure')
    ok = unexpected == 0
    return _cell(case, combo, lock_digest, track, run_class, 'matrix',
                 'planning子集 %d/%d 过 %d 预期失败' % (passed, len(sub), expected)
                 if ok else 'unexpected=%d' % unexpected,
                 'pass' if ok else 'unexpected_failure')


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
        return cell
    reason = ';'.join(verdict['failures']) or 'checker'
    expected = case['truth'].get('expected_reason')
    if expected and expected in reason:
        cell.update(stage='eval', reason=reason, outcome='expected_failure')
    else:
        cell.update(stage='eval', reason=reason, outcome='unexpected_failure')
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
