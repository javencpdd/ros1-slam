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

# 2026-11-18（M9）：冻结判据值已移到 calibration_data.py（纯数据 328 行）。
# 下面 re-export 保证 5 处 `import calibration as cal` 的调用方零改动。
from calibration_data import (  # noqa: F401
    FROZEN, REQUIRED, PENDING_REQUIRED, SCOPED_REQUIRED, SAMPLING_MIN_SEEDS,
    SAMPLING_MIN_REPETITIONS, SUCCESS_RATE_SCOPE, DIGEST)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scenarios'))
import scenarios as scn  # noqa: E402

# 冻结值来源＝各批次已定判据（引用在 evidence 字段）


HERE = Path(__file__).resolve().parent

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

def _validate_scoped_fields(config):
    """带域冻结项：scope 精确匹配、sample_n 为不低于采样下限的整数。

    scope 是口径本身：改 scope 等于改判据语义，必须重走解封与独立复核，
    不允许在值不变时静默把「组件域」扩成「全栈域」。
    """
    for path in sorted(SCOPED_REQUIRED):
        rule = SCOPED_REQUIRED[path]
        node = _resolve(config, path)
        if not isinstance(node, dict):
            _raise_pending(path, '不是带域对象')
        scope = node.get('scope')
        if scope != rule['scope']:
            _raise_pending(path + '.scope',
                           '须为 %r（当前 %r）；带域冻结不许静默扩域'
                           % (rule['scope'], scope))
        sample_n = node.get('sample_n')
        if not _type_ok(sample_n, (int,)) or sample_n < rule['min_sample_n']:
            _raise_pending(path + '.sample_n',
                           '须为 ≥%d 的整数（当前 %r）；样本不足不得带域冻结'
                           % (rule['min_sample_n'], sample_n))

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
    # R748 判据面：成功率是比例门槛，裸 value 不得离开 pending；必须有 scope/
    # sample_n/method/alpha/denominator/aggregation/evidence，且 36-run 下限与
    # Clopper-Pearson 方法不得缺省（fail-closed，不因填了数就判绿）。
    sr_node = _resolve(config, 'success_rate.min_pass_rate')
    if sr_node is not _MISSING and sr_node is not None:
        if not isinstance(sr_node, dict):
            pending.append({
                'key': 'success_rate.min_pass_rate',
                'reason': '须为对象（含 value/scope/sample_n/method/alpha/denominator/'
                          'aggregation/evidence）；裸值不得冻结（R748）'})
        elif sr_node.get('value') is not None:
            value = sr_node.get('value')
            # R752：比例域 (0,1]；负值/0/NaN/Inf 不得靠“填了数”离开 pending。
            if not (_is_number(value) and 0 < value <= 1):
                pending.append({
                    'key': 'success_rate.min_pass_rate.value',
                    'reason': '成功率须在 (0,1]（当前 %r）；域外值不得冻结（R752）'
                              % (value,)})
            text_fields = ('scope', 'denominator', 'aggregation', 'evidence')
            missing = []
            for field in ('scope', 'sample_n', 'method', 'alpha',
                          'denominator', 'aggregation', 'evidence'):
                item = sr_node.get(field)
                if item is None:
                    missing.append(field)
                elif field in text_fields and (not isinstance(item, str)
                                               or not item.strip()):
                    missing.append(field)
                elif field not in text_fields and isinstance(item, str) \
                        and not item.strip():
                    missing.append(field)
            if missing:
                pending.append({
                    'key': 'success_rate.min_pass_rate',
                    'reason': '值已填但缺元数据 %s；无 scope/sample_n/evidence 不得冻结（R748）'
                              % ','.join(missing)})
            else:
                if sr_node.get('scope') != SUCCESS_RATE_SCOPE:
                    pending.append({
                        'key': 'success_rate.min_pass_rate.scope',
                        'reason': 'scope 须精确为 %r（R752，带域冻结不许静默扩域）'
                                  % SUCCESS_RATE_SCOPE})
                if sr_node.get('method') != 'clopper_pearson_lower_95':
                    pending.append({
                        'key': 'success_rate.min_pass_rate.method',
                        'reason': '方法须为 clopper_pearson_lower_95（R748）'})
                if (not isinstance(sr_node.get('sample_n'), int)
                        or isinstance(sr_node.get('sample_n'), bool)
                        or sr_node['sample_n'] < 36):
                    pending.append({
                        'key': 'success_rate.min_pass_rate.sample_n',
                        'reason': '36-run 定标下限：sample_n 须为 ≥36 的整数（R748）'})
                alpha = sr_node.get('alpha')
                if not (_is_number(alpha) and 0 < alpha <= 0.5):
                    pending.append({
                        'key': 'success_rate.min_pass_rate.alpha',
                        'reason': 'alpha 须在 (0,0.5]（R748）'})
    # R759（R751 P3-1）：另两个 pending 键与 success_rate 同族——填了值不算数，
    # 必须有 scope/evidence 且为正数；否则仍 pending（fail-closed）。
    # （定位键已于 R787 按 B 方案带域冻结，移出本循环。）
    for key in ('budgets.stop_time_s', 'budgets.stop_distance_m'):
        node = _resolve(config, key)
        if not isinstance(node, dict) or node.get('value') is None:
            continue
        value = node.get('value')
        if not (_is_number(value) and value > 0):
            pending.append({
                'key': key + '.value',
                'reason': '须为正数（当前 %r）；域外值不得冻结（R759）' % (value,)})
        missing = [field for field in ('scope', 'evidence')
                   if not isinstance(node.get(field), str) or not node.get(field).strip()]
        if missing:
            pending.append({
                'key': key,
                'reason': '值已填但缺元数据 %s；无 scope/evidence 不得冻结（R759）'
                          % ','.join(missing)})
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
    _validate_scoped_fields(config)

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
    if path.startswith('latency.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new > old:
            return '时延上限上调'
        return None
    if path.startswith('resources.') and path.endswith('.value'):
        if _is_number(old) and _is_number(new) and new > old:
            return '资源上限上调'
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
    # R748 判据面：成功率/覆盖率类门槛只能上调（更严）；下调一律按放宽处理。
    if path.startswith('success_rate.') and path.endswith('.value'):
        if not (_is_number(old) and _is_number(new)):
            return None
        return '成功率/覆盖率门槛下调' if new < old else None
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

def _success_rate_metadata_changes(previous, current):
    """R755（R751 P2-1）：成功率节点级口径变更也算放宽。

    带域冻结的 scope 是判据语义本体；scope/分母/聚合口径变更或样本数下调
    都必须重走 unfreeze，不能只比较 .value 叶子。
    """
    prev = _resolve(previous, 'success_rate.min_pass_rate') if isinstance(previous, dict) else _MISSING
    cur = _resolve(current, 'success_rate.min_pass_rate') if isinstance(current, dict) else _MISSING
    if not (isinstance(prev, dict) and isinstance(cur, dict)):
        return []
    path = 'success_rate.min_pass_rate'
    changes = []
    for field, reason in (('scope', '成功率 scope 变更/扩域'),
                          ('sample_n', '成功率样本数下调'),
                          ('denominator', '成功率分母口径变更'),
                          ('aggregation', '成功率聚合口径变更')):
        old, new = prev.get(field), cur.get(field)
        if old is None and new is None:
            continue
        if field == 'sample_n':
            if _is_number(old) and _is_number(new) and new < old:
                changes.append((path + '.' + field, old, new, reason))
        elif old != new:
            changes.append((path + '.' + field, old, new, reason))
    return changes

BASELINE_FILE = HERE / 'frozen_baseline.json'

def freeze_guard(config, baseline_path=None):
    """R758（R751 P2-2）：freeze 前对上一版冻结基线跑 loosening。

    无基线＝首次冻结（允许）；基线不可读或结构不对＝E_CAL_BASELINE；
    基线存在时任何未授权放宽＝E_CAL_LOOSENED。纯读，不写文件。
    """
    path = Path(baseline_path) if baseline_path is not None else BASELINE_FILE
    if not path.exists():
        return []
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise CalibrationError('E_CAL_BASELINE', '基线不可读：%s' % exc)
    previous = data.get('frozen') if isinstance(data, dict) else None
    if not isinstance(previous, dict):
        raise CalibrationError('E_CAL_BASELINE', '基线缺少 frozen 对象')
    return check_loosening(previous, config)

def check_loosening(previous, current):
    """纯函数：相对 previous，current 的放宽项须有 from/to 且 reason 非空的 unfreeze。

    放宽口径：容差上调（min_clearance 为下调）、预算/上限下调、时延上限上调、
    资源上限上调、分母/矩阵条目减少、采样重复与 seed 减少、重跑上限上调、
    定标 run 改为计入正式分母；删除受控键亦算。
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
    changes.extend(_success_rate_metadata_changes(previous, current))
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
            freeze_guard(FROZEN)
        except CalibrationError as exc:
            print(json.dumps({'ok': False, 'code': exc.code, 'detail': exc.detail},
                             ensure_ascii=False))
            return 2
        # ok 仅表示形状/契约校验无硬错（兼容既有 CLI 断言）；pending 未清时
        # freeze_allowed=False、status=blocked，不得当作可封版。
        if report['freeze_allowed']:
            BASELINE_FILE.write_text(json.dumps(
                {'frozen': FROZEN, 'digest': current_digest,
                 'calibration_version': FROZEN['calibration_version']},
                ensure_ascii=False, sort_keys=True))
        print(json.dumps({'ok': True,
                          'ok_scope': 'shape_and_contract_only',
                          'status': report['status'],
                          'blocked': report['blocked'],
                          'freeze_allowed': report['freeze_allowed'],
                          'pending': report['pending'],
                          'calibration_version': FROZEN['calibration_version'],
                          'digest': current_digest,
                          'baseline': str(BASELINE_FILE)}, ensure_ascii=False))
        return 0
    print('usage: calibration.py freeze', file=sys.stderr)
    return 9

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

