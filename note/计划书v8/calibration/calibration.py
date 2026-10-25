"""V8-12：判据定标与正式实验封版（R657）——冻结配置（机读、无待定关键值）。

A0/A7 面「无待定关键值」：本件把 V8-08—V8-11 各设计层用到的判据值**冻结为机读配置**，
任何值缺省/待定＝E_CAL_PENDING（fail-closed，不许带 TODO 封版）。
定标集与正式集分开：定标 run 与正式 run 记录分离，**定标结果不得计入正式分母**
（23 号：V8-12 试跑用于定标，不得提前计入 V8-13 正式分母）。
封版后变更须走显式 unfreeze（记录变更与理由），不得静默改值。
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scenarios'))
import scenarios as scn  # noqa: E402

# 冻结值来源＝各批次已定判据（引用在 evidence 字段）
FROZEN = {
    'calibration_version': 1,
    'tolerances': {
        'tracking_lateral_m': {'value': 0.15, 'evidence': 'R655 tracking 默认'},
        'tracking_longitudinal_m': {'value': 0.20, 'evidence': 'R655'},
        'tracking_vertical_m': {'value': 0.10, 'evidence': 'R655'},
        'goal_position_m': {'value': 0.30, 'evidence': 'R656 场景任务夹具'},
        'z_support_tol_m': {'value': 0.10, 'evidence': 'R653 检查器默认'},
        'min_clearance_m': {'value': 0.20, 'evidence': 'R653'},
    },
    'budgets': {
        'planner_max_expansions': {'value': 20000, 'evidence': 'R652 plan 默认'},
        'rejection_limit_consecutive': {'value': 3, 'evidence': 'R654 MAX_REJECTIONS'},
        'task_budget_s': {'value': 30.0, 'evidence': 'R656 任务夹具'},
        'harness_deadline_s': {'value': 60.0, 'evidence': 'R656'},
    },
    'sampling': {
        'scenario_repetitions': {'value': 1, 'evidence': '设计层确定性（无随机源）'},
        'perturbation_seeds': {'value': [], 'evidence': '设计层无随机；运行面注入另批'},
    },
    'denominators': {
        'official_scenarios': {'value': ['SC01', 'SC02', 'SC03', 'SC07', 'SC08', 'SC10'],
                               'evidence': '已落构造器的场景；SC04—06/09/11—14 槽位不入分母'},
        'calibration_scenarios': {'value': ['SC01', 'SC08'],
                                  'evidence': '定标集（正向＋单向正例）'},
        'combos': {'value': ['hdl+dwa', 'hdl+teb', 'lite3+dwa', 'lite3+teb'],
                   'evidence': 'R648 组合矩阵'},
        'tracks': {'value': ['truth', 'estimate'],
                   'evidence': 'V8-13 双轨（真值轨/估计轨）'},
    },
    'rerun_rules': {
        'max_reruns': {'value': 1, 'evidence': '失败可复跑一次，二次失败计失败'},
        'calibration_runs_count_official': {'value': False, 'evidence': '铁则：定标不入正式分母'},
    },
}


class CalibrationError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def validate_frozen(config=None):
    """无待定关键值＋引用完整性：每个值具体、场景 id 在册、集合可分离。"""
    config = config or FROZEN
    for section, entries in config.items():
        if isinstance(entries, dict):
            for key, record in entries.items():
                if not isinstance(record, dict) or 'value' not in record or 'evidence' not in record:
                    raise CalibrationError('E_CAL_PENDING',
                                           '%s.%s 缺 value/evidence（无待定关键值铁则）' % (section, key))
                if record['value'] is None or (isinstance(record['value'], str)
                                               and record['value'].strip().upper() in ('TODO', 'TBD')):
                    raise CalibrationError('E_CAL_PENDING', '%s.%s 待定' % (section, key))
    official = set(config['denominators']['official_scenarios']['value'])
    calibration = set(config['denominators']['calibration_scenarios']['value'])
    known = {k for k, v in scn.SCENARIOS.items() if not v.get('slot')}
    unknown = (official | calibration) - known
    if unknown:
        raise CalibrationError('E_CAL_UNKNOWN_SCENARIO', ','.join(sorted(unknown)))
    if not official or not calibration:
        raise CalibrationError('E_CAL_EMPTY_SET', '分母/定标集不得为空')
    return True


def separation_guard(run_records):
    """定标与正式分开：run 记录须带 track 字段且定标 run 不入正式分母。"""
    official = [r for r in run_records if r.get('run_class') == 'official']
    leaked = [r for r in official if r.get('calibration_only')]
    if leaked:
        raise CalibrationError('E_CAL_LEAK', '定标 run 混入正式分母：%s' % leaked[:1])
    return {'official_count': len(official),
            'calibration_count': len(run_records) - len(official)}


def digest(config=None):
    import hashlib
    config = config or FROZEN
    canonical = json.dumps(config, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(canonical.encode()).hexdigest()


def main(argv):
    if argv and argv[0] == 'freeze':
        validate_frozen()
        print(json.dumps({'ok': True, 'calibration_version': FROZEN['calibration_version'],
                          'digest': digest()}, ensure_ascii=False))
        return 0
    print('usage: calibration.py freeze', file=sys.stderr)
    return 9


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
