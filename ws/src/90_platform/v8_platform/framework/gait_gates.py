#!/usr/bin/env python3
"""步态门值解析：声明基线（默认）／实测契约反算（可选），纯 Python 无 ROS。

背景（在册事实，勿凭记忆改）：
  - 声明基线来自 configs/lite3/graph_3d_lite3.yaml 的 gait_guard.params，
    由 param_exec.py 灌成 rosparam，节点用 rospy.get_param 读。
  - GAIT_PARAM_CONTRACT.md §5 给出前视距离反算式：
        lookahead_min_m = v*(cmd_lag_p95_s + switch_settle_s) + stop_distance_m + margin_m
    其中 cmd_lag_p95_s / switch_settle_s 在契约 latency_jitter 组，
    stop_distance_m 在 failure 组，v 取 envelope.v_max_mps。
  - 契约模板当前所有数值字段均为 null，且 status=TEMPLATE_ONLY / effective=false。
    因此本模块的 contract 源在真机回填前**必须拒用并回退声明基线**，不编数值。

设计纪律：
  1. 默认 declared，返回与调用方传入的声明基线逐字相同的值（零回归可证）。
  2. contract 源任一半字段缺失／非数值／非有限／模板未生效 -> 整体回退，不做部分应用。
  3. 反算结果只增不减：lookahead 小于声明基线时保持声明值（缩短前视会降低安全裕度）。
  4. 没有实测依据的门值（enter_m / exit_m / min_step_m）一律保持声明值，不反算、不编造。
  5. 空集拒判：declared 缺键或非法时抛 ValueError，不印绿。
"""
import math

GATE_KEYS = ('lookahead_m', 'enter_m', 'exit_m', 'min_step_m', 'max_step_m')

# 契约字段 -> 反算用途。逐字取自 gait_param_contract.yaml.template，勿凭印象改。
CONTRACT_FIELDS = {
    'cmd_lag_p95_s': ('gaits', '{gait}', 'latency_jitter', 'cmd_lag_p95_s'),
    'switch_settle_s': ('gaits', '{gait}', 'latency_jitter', 'switch_settle_s'),
    'stop_distance_m': ('gaits', '{gait}', 'failure', 'stop_distance_m'),
    'v_max_mps': ('gaits', '{gait}', 'envelope', 'v_max_mps'),
    'step_max_m': ('gaits', '{gait}', 'envelope', 'step_max_m'),
}


def _finite_positive(value):
    """数值有限且为正才通过；None / bool / 字符串 / NaN / inf 一律拒绝。"""
    if value is None or isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value)) and float(value) > 0.0


def validate_declared(declared):
    """空集拒判：声明基线必须齐 5 键且均为有限正数，否则抛错而非静默放行。"""
    if not isinstance(declared, dict):
        raise ValueError('declared gates must be a dict')
    missing = [k for k in GATE_KEYS if k not in declared]
    if missing:
        raise ValueError('declared gates missing keys: %s' % ','.join(missing))
    bad = [k for k in GATE_KEYS if not _finite_positive(declared[k])]
    if bad:
        raise ValueError('declared gates non-finite or non-positive: %s' % ','.join(bad))
    return True


def _dig(contract, path, gait):
    cur = contract
    for part in path:
        key = part.format(gait=gait) if '{gait}' in part else part
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def read_contract_fields(contract, gait='flat_mid'):
    """抽出反算所需字段；返回 (values, missing)。模板未生效单独标记。"""
    values, missing = {}, []
    if not isinstance(contract, dict):
        return {}, list(CONTRACT_FIELDS)
    status = contract.get('status')
    effective = contract.get('effective')
    if status == 'TEMPLATE_ONLY' or effective is False:
        return {}, ['template_not_effective']
    for name, path in CONTRACT_FIELDS.items():
        value = _dig(contract, path, gait)
        if not _finite_positive(value):
            missing.append(name)
        else:
            values[name] = float(value)
    return values, missing


def resolve_gates(declared, source='declared', contract=None, margin_m=0.0,
                  gait='flat_mid'):
    """返回门值解析结果 dict，不改变调用方传入的 declared。

    result = {
      'gates':      生效门值（5 键 float），
      'source':     'declared' | 'contract'（实际生效的源，回退后为 declared），
      'requested':  'declared' | 'contract'（调用方请求的源），
      'applied':    被反算改写的键列表，
      'refusals':   拒用原因列表（模板未生效／字段缺失／低于声明基线等），
      'derived':    反算中间量（仅 contract 成功时非空），
    }
    """
    validate_declared(declared)
    gates = {k: float(declared[k]) for k in GATE_KEYS}
    out = {
        'gates': gates, 'source': 'declared', 'requested': source,
        'applied': [], 'refusals': [], 'derived': {},
    }
    if source != 'contract':
        return out

    values, missing = read_contract_fields(contract, gait)
    if missing:
        out['refusals'].append('contract_unusable:%s' % ','.join(sorted(missing)))
        return out
    if not _finite_positive(margin_m) and float(margin_m or 0.0) != 0.0:
        out['refusals'].append('margin_m_non_finite')
        return out
    margin = float(margin_m or 0.0)

    # lookahead_min_m = v*(lag + settle) + stop + margin
    v = values['v_max_mps']
    lookahead_min = (v * (values['cmd_lag_p95_s'] + values['switch_settle_s'])
                     + values['stop_distance_m'] + margin)
    derived = {
        'formula': 'v*(cmd_lag_p95_s+switch_settle_s)+stop_distance_m+margin_m',
        'v_mps': round(v, 6),
        'cmd_lag_p95_s': round(values['cmd_lag_p95_s'], 6),
        'switch_settle_s': round(values['switch_settle_s'], 6),
        'stop_distance_m': round(values['stop_distance_m'], 6),
        'margin_m': round(margin, 6),
        'lookahead_min_m': round(lookahead_min, 6),
        'step_max_m': round(values['step_max_m'], 6),
    }
    out['derived'] = derived

    # 只增不减：反算前视小于声明基线时保持声明值。
    if lookahead_min < gates['lookahead_m']:
        out['refusals'].append('lookahead_below_declared:%.6f<%.6f'
                               % (lookahead_min, gates['lookahead_m']))
    else:
        gates['lookahead_m'] = round(lookahead_min, 6)
        out['applied'].append('lookahead_m')

    # step_max_m 有实测依据，但同样只向更保守方向（更小上限）改写。
    if values['step_max_m'] < gates['max_step_m']:
        gates['max_step_m'] = round(values['step_max_m'], 6)
        out['applied'].append('max_step_m')
    else:
        out['refusals'].append('step_max_m_not_tighter:%.6f>=%.6f'
                               % (values['step_max_m'], gates['max_step_m']))

    out['source'] = 'contract' if out['applied'] else 'declared'
    return out


# ── 三连自测：主测（零回归）＋注入有效性（假契约必须真反算）＋反控（撤掉即回退）──
def _selftest():
    declared = {'lookahead_m': 1.5, 'enter_m': 1.0, 'exit_m': 1.5,
                'min_step_m': 0.05, 'max_step_m': 0.15}

    # 主测：默认 declared 逐字零回归
    r = resolve_gates(declared)
    assert r['source'] == 'declared' and r['applied'] == []
    assert r['gates'] == {k: float(declared[k]) for k in GATE_KEYS}
    assert r['gates'] is not declared, 'must not mutate caller dict'
    print('主测 OK：declared 逐字零回归，未改动调用方 dict')

    # 空集拒判
    try:
        resolve_gates({'lookahead_m': 1.5})
        raise AssertionError('空集拒判 FAIL：缺键却放行')
    except ValueError:
        print('空集拒判 OK：declared 缺键抛 ValueError')

    # 模板未生效必须拒用（真机回填前的真实状态）
    r = resolve_gates(declared, source='contract',
                      contract={'status': 'TEMPLATE_ONLY', 'effective': False})
    assert r['source'] == 'declared' and r['gates']['lookahead_m'] == 1.5
    assert any('template_not_effective' in x for x in r['refusals'])
    print('模板态拒用 OK：TEMPLATE_ONLY 回退声明基线')

    # 注入有效性：构造一份字段齐全的假契约，反算必须真的改变门值
    fake = {
        'status': 'FILLED', 'effective': True,
        'gaits': {'flat_mid': {
            'latency_jitter': {'cmd_lag_p95_s': 0.10, 'switch_settle_s': 0.50},
            'failure': {'stop_distance_m': 0.30},
            'envelope': {'v_max_mps': 1.0, 'step_max_m': 0.12},
        }},
    }
    r = resolve_gates(declared, source='contract', contract=fake, margin_m=0.20)
    assert r['source'] == 'contract', '注入有效性 FAIL：假契约没生效'
    # 1.0*(0.10+0.50)+0.30+0.20 = 1.10 -> 小于声明 1.5，
    # 按「只增不减」lookahead_m 不得进入 applied；只有 max_step_m 收紧生效。
    assert 'max_step_m' in r['applied'] and 'lookahead_m' not in r['applied']
    assert r['gates']['lookahead_m'] == 1.5, '只增不减失效'
    assert r['gates']['max_step_m'] == 0.12, 'step_max_m 未收紧'
    assert any('lookahead_below_declared' in x for x in r['refusals'])
    print('注入有效性 OK：假契约触发反算（1.10 m 因低于 1.5 被保持），max_step 收紧到 0.12')

    # 换一组更快的速度，让反算值超过声明基线 -> lookahead 必须真的被改写
    fake2 = {'status': 'FILLED', 'effective': True,
             'gaits': {'flat_mid': {
                 'latency_jitter': {'cmd_lag_p95_s': 0.20, 'switch_settle_s': 0.80},
                 'failure': {'stop_distance_m': 0.40},
                 'envelope': {'v_max_mps': 2.0, 'step_max_m': 0.12}}}}
    r2 = resolve_gates(declared, source='contract', contract=fake2, margin_m=0.20)
    # 2.0*(0.20+0.80)+0.40+0.20 = 2.60
    assert r2['gates']['lookahead_m'] == 2.6, ('反算值错误', r2['gates'])
    assert 'lookahead_m' in r2['applied']
    print('反算生效 OK：v=2.0 时 lookahead 1.5 -> 2.6 m')

    # 反控：把契约改回模板态，门值必须逐字复原
    fake2['status'] = 'TEMPLATE_ONLY'
    fake2['effective'] = False
    r3 = resolve_gates(declared, source='contract', contract=fake2, margin_m=0.20)
    assert r3['source'] == 'declared'
    assert r3['gates'] == {k: float(declared[k]) for k in GATE_KEYS}
    print('反控 OK：撤掉契约生效态后门值逐字复原')
    print('SELFTEST PASS (gait_gates)')
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_selftest())
