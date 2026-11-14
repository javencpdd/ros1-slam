#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V8-X1 台阶/坡道通过判据（纯函数，零 ROS/零 MuJoCo 依赖）。

读数形式：trace＝[(t, x, z)]（机身中心，0.5 s 采样）；世界几何由调用方给出。
规则（防止 R809/R810 里那次「坡道下坡被 z<0.18 记摔」的误判）：
- 台阶：`reached`＝曾到前沿；`crossed`＝曾越过后沿；`fell`＝到前沿之后 z 跌破 z_floor；
  `pass`＝crossed 且未 fell。落阶后的正常下降不单独记摔（由 z_floor 兜底，低于地面才记）。
- 坡道：`traversed`＝曾越过后沿；`fell` 只在 z 跌破 z_floor 时成立；下坡回到地面不算摔。
- 打滑：本轮不判（需足端接触位移，基线脚本尚未采集），接口留 `slip_m` 由后续版本填。
"""


def judge_step(trace, front_x, rear_x, z_floor=0.12):
    xs = [p[1] for p in trace]
    if not xs:
        return {'reached': False, 'crossed': False, 'fell': False, 'pass': False,
                'reason': 'empty_trace', 'max_x': None}
    max_x = max(xs)
    reached = max_x >= front_x
    crossed = max_x >= rear_x
    fell = False
    if reached:
        started = False
        for _t, x, z in trace:
            if x >= front_x:
                started = True
            if started and z < z_floor:
                fell = True
                break
    reason = 'ok' if (crossed and not fell) else (
        'fell_after_step' if fell else ('not_crossed' if not crossed else 'unknown'))
    return {'reached': reached, 'crossed': crossed, 'fell': fell,
            'pass': bool(crossed and not fell), 'reason': reason,
            'max_x': round(max_x, 3)}


def judge_ramp(trace, rear_x, z_floor=0.12):
    xs = [p[1] for p in trace]
    if not xs:
        return {'traversed': False, 'fell': False, 'pass': False,
                'reason': 'empty_trace', 'max_x': None}
    max_x = max(xs)
    traversed = max_x >= rear_x
    fell = any(z < z_floor for _t, _x, z in trace)
    reason = 'ok' if (traversed and not fell) else (
        'fell' if fell else 'not_traversed')
    return {'traversed': traversed, 'fell': fell,
            'pass': bool(traversed and not fell), 'reason': reason,
            'max_x': round(max_x, 3)}


def judge_with_slip(base, slip_m, slip_limit_m=0.05):
    """把足端打滑并进判据：打滑超限即不通过（即使越过了后沿）。"""
    result = dict(base)
    result['slip_m'] = slip_m
    if slip_m is not None and slip_m > slip_limit_m:
        result['pass'] = False
        result['reason'] = 'slip_exceeded'
    return result
