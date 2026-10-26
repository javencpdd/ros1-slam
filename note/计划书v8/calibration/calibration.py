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
    'calibration_version': 2,
    # 解封记录（R657 封版 v1 → R661 v2）：official_scenarios 6→14（SC04—06/09/11—14
    # 构造器已落，V8-13 矩阵按 kind 分派执行）；其余五节零变更。
    'unfreeze_log': {'to_version': 2, 'when': '2026-10-26',
                     'change': 'denominators.official_scenarios 扩至全 14 场景',
                     'reason': 'SC 构造器填充完成（R661），分母只认构造器在册场景的原则维持'},
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
        'official_scenarios': {'value': ['SC01', 'SC02', 'SC03', 'SC04', 'SC05', 'SC06',
                                        'SC07', 'SC08', 'SC09', 'SC10', 'SC11', 'SC12',
                                        'SC13', 'SC14'],
                               'evidence': 'v1＝已落构造器六场景；v2 解封扩至全 14（构造器齐，R661）'},
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

# ---------------------------------------------------------------------------
# 整改（2026-10-27，独立复核 P0）：形状 fail-closed / golden digest / 放宽守卫。
# FROZEN 的数值与 evidence 一字未改；以下常量只增加校验面。
# ---------------------------------------------------------------------------

# 必需键：键路径 → 允许类型 / 是否非空 / 列表元素类型。
# 缺键、None、空表、类型不符一律 E_CAL_PENDING（fail-closed）。
# 覆盖五节（tolerances/budgets/sampling/denominators/rerun_rules）全部叶子的
# value 与 evidence；sampling.perturbation_seeds 的空表属 4.1 采样欠账，
# 由 PENDING_REQUIRED 报告，不在此处误判为形状缺项。
REQUIRED = {
    'calibration_version': {'types': (int,)},
    'unfreeze_log': {'types': (dict, list), 'non_empty': True},
    'tolerances': {'types': (dict,), 'non_empty': True},
    'tolerances.tracking_lateral_m.value': {'types': (int, float)},
    'tolerances.tracking_lateral_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.tracking_longitudinal_m.value': {'types': (int, float)},
    'tolerances.tracking_longitudinal_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.tracking_vertical_m.value': {'types': (int, float)},
    'tolerances.tracking_vertical_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.goal_position_m.value': {'types': (int, float)},
    'tolerances.goal_position_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.z_support_tol_m.value': {'types': (int, float)},
    'tolerances.z_support_tol_m.evidence': {'types': (str,), 'non_empty': True},
    'tolerances.min_clearance_m.value': {'types': (int, float)},
    'tolerances.min_clearance_m.evidence': {'types': (str,), 'non_empty': True},
    'budgets': {'types': (dict,), 'non_empty': True},
    'budgets.planner_max_expansions.value': {'types': (int,)},
    'budgets.planner_max_expansions.evidence': {'types': (str,), 'non_empty': True},
    'budgets.rejection_limit_consecutive.value': {'types': (int,)},
    'budgets.rejection_limit_consecutive.evidence': {'types': (str,), 'non_empty': True},
    'budgets.task_budget_s.value': {'types': (int, float)},
    'budgets.task_budget_s.evidence': {'types': (str,), 'non_empty': True},
    'budgets.harness_deadline_s.value': {'types': (int, float)},
    'budgets.harness_deadline_s.evidence': {'types': (str,), 'non_empty': True},
    'sampling': {'types': (dict,), 'non_empty': True},
    'sampling.scenario_repetitions.value': {'types': (int,)},
    'sampling.scenario_repetitions.evidence': {'types': (str,), 'non_empty': True},
    'sampling.perturbation_seeds.value': {'types': (list,), 'items': (str,)},
    'sampling.perturbation_seeds.evidence': {'types': (str,), 'non_empty': True},
    'denominators': {'types': (dict,), 'non_empty': True},
    'denominators.official_scenarios.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.official_scenarios.evidence': {'types': (str,), 'non_empty': True},
    'denominators.calibration_scenarios.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.calibration_scenarios.evidence': {'types': (str,), 'non_empty': True},
    'denominators.combos.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.combos.evidence': {'types': (str,), 'non_empty': True},
    'denominators.tracks.value': {'types': (list,), 'items': (str,), 'non_empty': True},
    'denominators.tracks.evidence': {'types': (str,), 'non_empty': True},
    'rerun_rules': {'types': (dict,), 'non_empty': True},
    'rerun_rules.max_reruns.value': {'types': (int,)},
    'rerun_rules.max_reruns.evidence': {'types': (str,), 'non_empty': True},
    'rerun_rules.calibration_runs_count_official.value': {'types': (bool,)},
    'rerun_rules.calibration_runs_count_official.evidence': {'types': (str,), 'non_empty': True},
}

# 当前确实缺失、24 §4.1 要求但尚未定标的项：只报告 pending+blocked，
# 不塞进 FROZEN（不改已冻结数值），也绝不据此判绿。
PENDING_REQUIRED = [
    {'key': 'latency.p95_global_s.value', 'types': (int, float),
     'reason': '24 §4.1 要求 p95 全局时延；当前未定标（缺项，禁 freeze）'},
    {'key': 'latency.p95_local_s.value', 'types': (int, float),
     'reason': '24 §4.1 要求 p95 局部时延；当前未定标（缺项，禁 freeze）'},
    {'key': 'budgets.cancel_time_s.value', 'types': (int, float),
     'reason': '24 §4.1 要求取消时间；当前未定标（缺项，禁 freeze）'},
    {'key': 'budgets.stop_time_s.value', 'types': (int, float),
     'reason': '24 §4.1 要求停车时间；当前未定标（缺项，禁 freeze）'},
    {'key': 'budgets.stop_distance_m.value', 'types': (int, float),
     'reason': '24 §4.1 要求停车距离预算并覆盖感知/控制延迟与制动；当前缺失（禁 freeze）'},
    {'key': 'tolerances.max_localization_error_m.value', 'types': (int, float),
     'reason': '24 §4.1 要求最大定位误差上限；当前未定标（缺项，禁 freeze）'},
    {'key': 'resources.max_cpu_percent.value', 'types': (int, float),
     'reason': '24 §4.1 要求资源上限；当前未定标（缺项，禁 freeze）'},
    {'key': 'resources.max_memory_mb.value', 'types': (int, float),
     'reason': '24 §4.1 要求资源上限；当前未定标（缺项，禁 freeze）'},
    {'key': 'success_rate.min_pass_rate.value', 'types': (int, float),
     'reason': '24 §4.1 要求成功率门槛；当前未定标（缺项，禁 freeze）'},
]

# 24 §4.1 采样要求：每个适用闭环场景至少 3 个预登记 seed、每 seed 3 次重复。
SAMPLING_MIN_SEEDS = 3
SAMPLING_MIN_REPETITIONS = 3

# golden digest：由当前 FROZEN 实算后填入（2026-10-27）。
# 重算：python3 -B -c "import calibration;print(calibration.digest())"
# 任何值变更都会使 assert_digest() 抛 E_CAL_DIGEST；解封须同步重算并登记。
DIGEST = '9a00d49bdd8bed7b7f922fcfe84c2867de17d5b707db6c349767642d48a52f1b'


class CalibrationError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


_MISSING = object()


def _resolve(config, path):
    """按键路径取值；任一层缺失或非对象返回 _MISSING。"""
    node = config
    for part in path.split('.'):
        if not isinstance(node, dict) or part not in node:
            return _MISSING
        node = node[part]
    return node


def _type_ok(value, types):
    """bool 不冒充 int/float；类型白名单严格匹配。"""
    if isinstance(value, bool):
        return bool in types
    return isinstance(value, types)


def _raise_pending(path, detail):
    raise CalibrationError('E_CAL_PENDING', '%s %s（fail-closed）' % (path, detail))


def _validate_shape(config):
    """REQUIRED 形状检查：缺键/None/空表/类型不符一律 E_CAL_PENDING。"""
    if not isinstance(config, dict):
        _raise_pending('config', '根节点不是对象')
    for path in sorted(REQUIRED):
        rule = REQUIRED[path]
        value = _resolve(config, path)
        if value is _MISSING:
            _raise_pending(path, '缺失')
        if value is None:
            _raise_pending(path, '为 None')
        if not _type_ok(value, rule['types']):
            _raise_pending(path, '类型 %s 不符（允许 %s）'
                           % (type(value).__name__, rule['types']))
        if isinstance(value, str) and value.strip().upper() in ('TODO', 'TBD'):
            _raise_pending(path, '为待定占位 %r' % value)
        if rule.get('non_empty'):
            if isinstance(value, str) and not value.strip():
                _raise_pending(path, '为空字符串')
            if isinstance(value, (dict, list, tuple)) and len(value) == 0:
                _raise_pending(path, '为空表')
        item_types = rule.get('items')
        if item_types is not None and isinstance(value, list):
            for index, item in enumerate(value):
                if not _type_ok(item, item_types):
                    _raise_pending('%s[%d]' % (path, index),
                                   '元素类型 %s 不符' % type(item).__name__)


def _validate_unfreeze_log(config):
    """解封记录每条都须是对象且 reason 非空。"""
    log = config['unfreeze_log']
    entries = [log] if isinstance(log, dict) else log
    for index, entry in enumerate(entries):
        label = 'unfreeze_log[%d]' % index
        if not isinstance(entry, dict):
            _raise_pending(label, '不是对象')
        reason = entry.get('reason')
        if not isinstance(reason, str) or not reason.strip():
            _raise_pending(label + '.reason', '缺失或为空（解封须记录理由）')


def _pending_report(config):
    """24 §4.1 尚未定标/采样的项：列出 pending 键，不判绿。"""
    pending = []
    for item in PENDING_REQUIRED:
        path = item['key']
        value = _resolve(config, path)
        bad = False
        if value is _MISSING or value is None:
            bad = True
        elif not _type_ok(value, item['types']):
            bad = True
        elif isinstance(value, str) and (not value.strip()
                                         or value.strip().upper() in ('TODO', 'TBD')):
            bad = True
        elif isinstance(value, (dict, list)) and len(value) == 0:
            bad = True
        if bad:
            pending.append({'key': path, 'reason': item['reason']})
    repetitions = config['sampling']['scenario_repetitions']['value']
    seeds = config['sampling']['perturbation_seeds']['value']
    if repetitions < SAMPLING_MIN_REPETITIONS:
        pending.append({
            'key': 'sampling.scenario_repetitions.value',
            'reason': '24 §4.1 要求每 seed 至少 %d 次重复；当前=%r，未达 4.1 采样要求，禁 freeze'
                      % (SAMPLING_MIN_REPETITIONS, repetitions)})
    if len(seeds) < SAMPLING_MIN_SEEDS:
        pending.append({
            'key': 'sampling.perturbation_seeds.value',
            'reason': '24 §4.1 要求每个适用闭环场景至少 %d 个预登记 seed；当前=%d，未达 4.1 采样要求，禁 freeze'
                      % (SAMPLING_MIN_SEEDS, len(seeds))})
    return pending


def validate_frozen(config=None):
    """无待定关键值＋引用完整性；形状 fail-closed，pending 项报 blocked 不报绿。

    返回 dict：ok/status/freeze_allowed/pending/pending_detail/digest。
    pending 非空时 ok=False、status='blocked'、freeze_allowed=False。
    """
    config = FROZEN if config is None else config
    _validate_shape(config)
    _validate_unfreeze_log(config)

    official = set(config['denominators']['official_scenarios']['value'])
    calibration = set(config['denominators']['calibration_scenarios']['value'])
    known = {k for k, v in scn.SCENARIOS.items() if not v.get('slot')}
    unknown = (official | calibration) - known
    if unknown:
        raise CalibrationError('E_CAL_UNKNOWN_SCENARIO', ','.join(sorted(unknown)))
    if not official or not calibration:
        raise CalibrationError('E_CAL_EMPTY_SET', '分母/定标集不得为空')

    pending = _pending_report(config)
    return {
        'ok': not pending,
        'status': 'blocked' if pending else 'pass',
        'blocked': bool(pending),
        'freeze_allowed': not pending,
        'pending': [item['key'] for item in pending],
        'pending_detail': pending,
        'calibration_version': config['calibration_version'],
        'digest': digest(config),
    }


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
    config = FROZEN if config is None else config
    canonical = json.dumps(config, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(canonical.encode()).hexdigest()


def assert_digest(config=None):
    """golden digest：值一变（含空 dict 兜底）即 E_CAL_DIGEST。"""
    actual = digest(config)
    if actual != DIGEST:
        raise CalibrationError('E_CAL_DIGEST',
                               '冻结配置已变更：期望 %s 实得 %s（须显式解封并登记 unfreeze_log）'
                               % (DIGEST, actual))
    return actual


def _iter_value_leaves(node, prefix=''):
    """展开 dict 叶子；unfreeze_log 是元数据，不参与放宽比较。"""
    if isinstance(node, dict):
        for key, value in node.items():
            path = key if not prefix else prefix + '.' + key
            if path == 'unfreeze_log' or path.startswith('unfreeze_log.'):
                continue
            if isinstance(value, dict):
                yield from _iter_value_leaves(value, path)
            else:
                yield path, value


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _loosening_reason(path, old, new):
    """返回放宽理由；None 表示未放宽或变化方向不属于放宽。"""
    if path.startswith('tolerances.') and path.endswith('.value'):
        if not (_is_number(old) and _is_number(new)):
            return None
        if path == 'tolerances.min_clearance_m.value':
            return '最小净空下调' if new < old else None
        return '容差上调' if new > old else None
    if path.startswith('budgets.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new < old:
            return '预算/上限下调'
        return None
    if path == 'sampling.scenario_repetitions.value':
        if _is_number(old) and _is_number(new) and new < old:
            return '重复次数下调'
        return None
    if path == 'sampling.perturbation_seeds.value':
        if isinstance(old, list) and isinstance(new, list):
            return '预登记 seed 减少' if not set(new) >= set(old) else None
        return None
    if path.startswith('denominators.') and path.endswith('.value'):
        if isinstance(old, list) and isinstance(new, list):
            return '分母/矩阵条目减少' if not set(new) >= set(old) else None
        return None
    if path == 'rerun_rules.max_reruns.value':
        if _is_number(old) and _is_number(new) and new > old:
            return '重跑上限上调'
        return None
    if path == 'rerun_rules.calibration_runs_count_official.value':
        return '定标 run 计入正式分母' if old is False and new is True else None
    return None


def _unfreeze_entries(config):
    log = config.get('unfreeze_log') if isinstance(config, dict) else None
    if isinstance(log, dict):
        return [log]
    if isinstance(log, list):
        return [entry for entry in log if isinstance(entry, dict)]
    return []


def _authorizes(entry, path, old, new):
    """该条 unfreeze 是否覆盖此放宽：from/to 均在、reason 非空；带 key 时须精确匹配。"""
    reason = entry.get('reason')
    if not isinstance(reason, str) or not reason.strip():
        return False
    if 'from' not in entry or 'to' not in entry:
        return False
    if entry.get('from') is None or entry.get('to') is None:
        return False
    if isinstance(entry.get('from'), str) and not entry['from'].strip():
        return False
    if isinstance(entry.get('to'), str) and not entry['to'].strip():
        return False
    scoped = entry.get('keys', entry.get('key', entry.get('path')))
    if scoped is None:
        return True
    if isinstance(scoped, str):
        scoped = [scoped]
    if not isinstance(scoped, (list, tuple, set)):
        return False
    if path not in scoped:
        return False
    return entry.get('from') == old and entry.get('to') == new


def check_loosening(previous, current):
    """纯函数：相对 previous，current 的放宽项须有 from/to 且 reason 非空的 unfreeze。

    放宽口径：容差上调（min_clearance 为下调）、预算/上限下调、分母/矩阵条目减少、
    采样重复与 seed 减少、重跑上限上调、定标 run 改为计入正式分母；删除受控键亦算。
    返回被放宽的键路径列表；无放宽返回 []；未授权抛 E_CAL_LOOSENED。
    """
    previous_values = dict(_iter_value_leaves(previous))
    current_values = dict(_iter_value_leaves(current))
    changes = []
    for path, old in previous_values.items():
        if not path.endswith('.value'):
            continue
        if path not in current_values:
            changes.append((path, old, '<missing>', '键被删除'))
            continue
        new = current_values[path]
        reason = _loosening_reason(path, old, new)
        if reason:
            changes.append((path, old, new, reason))
    if not changes:
        return []
    entries = _unfreeze_entries(current)
    unauthorized = []
    for path, old, new, reason in changes:
        if not any(_authorizes(entry, path, old, new) for entry in entries):
            unauthorized.append('%s: %r→%r（%s）' % (path, old, new, reason))
    if unauthorized:
        raise CalibrationError('E_CAL_LOOSENED',
                               '放宽缺 unfreeze_log 的 from/to/reason：%s'
                               % '; '.join(unauthorized))
    return [item[0] for item in changes]


def main(argv):
    if argv and argv[0] == 'freeze':
        try:
            report = validate_frozen()
            current_digest = assert_digest()
        except CalibrationError as exc:
            print(json.dumps({'ok': False, 'code': exc.code, 'detail': exc.detail},
                             ensure_ascii=False))
            return 2
        # ok 仅表示形状/契约校验无硬错（兼容既有 CLI 断言）；pending 未清时
        # freeze_allowed=False、status=blocked，不得当作可封版。
        print(json.dumps({'ok': True,
                          'ok_scope': 'shape_and_contract_only',
                          'status': report['status'],
                          'blocked': report['blocked'],
                          'freeze_allowed': report['freeze_allowed'],
                          'pending': report['pending'],
                          'calibration_version': FROZEN['calibration_version'],
                          'digest': current_digest}, ensure_ascii=False))
        return 0
    print('usage: calibration.py freeze', file=sys.stderr)
    return 9


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
