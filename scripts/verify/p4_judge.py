#!/usr/bin/env python3
"""P4 四项验收的机器判定：只读各探针落盘的 JSON，按写死的阈值出 PASS/FAIL/INVALID。

为什么单独抽出来：以前四项的结论散在 shell 的整数比较里，口径各写一遍，
出现过「窗口没覆盖也报 PASS」「rc 是 tail 的 rc」这类假绿。这里集中一处，
且**任一项 FAIL/INVALID 就返回非零**，让 CI/回归能靠退出码拦。

阈值都有出处，改动必须同时改注释与 note 里的判据表：
  1 正常导航  真值到点 ≤ NAV_GT_TOL_M(0.15 m)，且 **BT 必须报 reached**、**采样必须连续**
  2 障碍安全  离线回归 judgement.verdict == PASS（5 场景 + 反控）
  3 定位失效  末非零命令后不再复动；停止延迟 ≤ LOC_STOP_DELAY_MAX_S(1.55 s)；真值滑行 ≤ LOC_COAST_MAX_M(0.93 m)；
              且**定位恢复后不得自行复动**（loc_recovery.json 必须 PASS）
  4 取消      取消后 ≥ CANCEL_ZERO_WINDOW_SIM_S(5 s) 仿真时间零速、期间真值位移 ≤ CANCEL_MOVE_MAX_M(0.05 m)，
              且**旧 stamp 路径被拒（cancel_replay 必须有且 PASS，缺文件按 FAIL 计）**

用法：
  p4_judge.py --root <evidence 目录>
"""
import argparse
import json
import os
import sys

NAV_GT_TOL_M = 0.15
NAV_EST_TOL_M = 0.35
# 停止延迟的界不是拍的，是三级串行超时之和：
#   local.input_max_age_s 0.70 + local.tick_s 0.60 + mux.timeout_s 0.25 = 1.55 s
# 传感器是 10 Hz（仿真里被 relay 泄漏拖到约 4.7 Hz），单帧 0.21 s，已在 0.70 内覆盖。
# 实测 1.24～1.29 s 落在这个界内 ⇒ 是既有设计的必然结果，不是本轮引入的缺陷。
# 要缩短它必须改这三个参数，那是产品决策，不能为了测试好看而调小。
LOC_STOP_DELAY_MAX_S = 1.55
# 滑行上界按同一口径反推：0.6 m/s（local 的 max_speed_mps）× 1.55 s = 0.93 m
LOC_COAST_MAX_M = 0.93
CANCEL_ZERO_WINDOW_SIM_S = 5.0
CANCEL_MOVE_MAX_M = 0.05


def load(root, name):
    path = os.path.join(root, name)
    if not os.path.isfile(path):
        return None
    try:
        return json.load(open(path))
    except Exception:
        return None


def judge_nav(root):
    arr = load(root, 'nav_arrival.json')
    probe = load(root, 'nav_probe.json')
    if not arr:
        return 'INVALID', '缺 nav_arrival.json', '-'
    gt = arr.get('gt_to_goal_m')
    est = arr.get('est_to_goal_m')
    ok = (gt is not None and gt <= NAV_GT_TOL_M
          and (est is None or est <= NAV_EST_TOL_M))
    detail = 'gt=%.3f m est=%s' % (gt if gt is not None else -1,
                                   '%.3f' % est if est is not None else 'NA')
    extra = '-'
    if probe:
        extra = 'frames=%s travel=%.3f' % (probe.get('speed_frames'),
                                           probe.get('est_travel_m') or 0)
        if not probe.get('speed_frames'):
            ok = False
            detail += ' 无非零运动'
        # ★BT 到达事件★：真值近但树没收敛（没报 reached）不算闭环走完
        if not probe.get('bt_arrived'):
            ok = False
            detail += ' BT未报reached'
        else:
            extra += ' bt=reached'
        # ★采样连续★：探针自身漏采时读数不可信
        gap = probe.get('max_cmd_gap_s')
        if gap is not None and gap > 0.5:
            ok = False
            detail += ' 采样断档%.2fs' % gap
    else:
        ok = False
        detail += ' 缺 nav_probe.json'
    return ('PASS' if ok else 'FAIL'), detail, extra


def judge_obstacle(root):
    data = load(root, 'obstacle.json')
    online = load(root, 'obstacle_inmotion.json')
    if not data:
        return 'INVALID', '缺 obstacle.json', '-'
    judgement = data.get('judgement') or data
    verdict = judgement.get('verdict')
    per = judgement.get('per_scene') or {}
    detail = ' '.join('%s=%s' % (k.split('_')[0], v) for k, v in sorted(per.items())) or '-'
    ok = (verdict == 'PASS')
    # ★在线段必须也有证据且 PASS★：离线只验算法本身，证不了"运动中整条链会停/绕"。
    if not online:
        ok = False
        extra = '在线=缺'
    elif online.get('verdict') != 'PASS':
        ok = False
        extra = '在线=%s' % online.get('verdict')
    else:
        extra = '在线PASS(zero=%ss,pen=%s)' % (online.get('max_zero_run_s'),
                                               online.get('penetrations'))
    if verdict not in ('PASS', 'FAIL'):
        return 'INVALID', detail, extra
    return ('PASS' if ok else 'FAIL'), detail, extra


def judge_loc(root):
    d = load(root, 'loc_stop.json')
    if not d:
        return 'INVALID', '缺 loc_stop.json', '-'
    if not d.get('motion_seen'):
        return 'INVALID', '注入前未取得非零运动', '-'
    delay = d.get('stop_delay_s')
    # 滑行取「注入之后」的位移（gt_coast_after_inject_m）。旧字段 gt_travel_after_m
    # 锚在末条非零命令上，会把注入到末条非零之间的行程漏掉，读数偏小。
    coast = d.get('gt_coast_after_inject_m', d.get('gt_travel_after_m'))
    after = d.get('max_speed_after')
    ok = (after == 0.0
          and bool(d.get('moving_at_inject'))
          and delay is not None and delay <= LOC_STOP_DELAY_MAX_S
          and coast is not None and coast <= LOC_COAST_MAX_M)
    rearm = load(root, 'loc_rearm.json')
    if not rearm or not rearm.get('rearmed'):
        ok = False
        detail_re = '新任务未解锁'
    else:
        detail_re = '新任务可解锁'
    rec = load(root, 'loc_recovery.json')
    if not rec:
        ok = False
        detail_rec = '恢复未验'
    elif not rec.get('pass'):
        ok = False
        checks = rec.get('checks') or {}
        if checks.get('sampling_coverage') is False:
            detail_rec = '恢复未验(采样缺)'
        else:
            detail_rec = '恢复后复动%.2fm' % (rec.get('moved_m') or 0)
    else:
        detail_rec = '恢复不动'
    detail = 'delay=%s s coast=%s m after_v=%s' % (
        '%.3f' % delay if delay is not None else 'NA',
        '%.3f' % coast if coast is not None else 'NA', after)
    if not d.get('moving_at_inject'):
        detail += ' 注入时已停'
    return ('PASS' if ok else 'FAIL'), detail + ' ' + detail_rec + ' ' + detail_re, 'nodes=%s' % d.get('v8_nodes_after', 'NA')


def judge_cancel(root):
    d = load(root, 'cancel.json')
    if not d:
        return 'INVALID', '缺 cancel.json', '-'
    if not d.get('motion_seen'):
        return 'INVALID', '注入前未取得非零运动', '-'
    zero_sim = d.get('zero_window_sim_s')
    covers = d.get('gt_covers_5s_sim')
    move = d.get('gt_move_within_5s_sim_m')
    after = d.get('max_speed_after')
    ack_ok = bool(d.get('ack_count'))
    ok = (ack_ok and after == 0.0
          and zero_sim is not None and zero_sim >= CANCEL_ZERO_WINDOW_SIM_S
          and covers is True
          and move is not None and move <= CANCEL_MOVE_MAX_M)
    replay = load(root, 'cancel_replay.json')
    if replay:
        ok = ok and bool(replay.get('pass'))
        replay_txt = 'replay=%s' % ('PASS' if replay.get('pass') else 'FAIL')
    else:
        # ★缺 replay 不能当"未验"放过★：取消重放（旧 stamp 必被拒）是取消语义的活体反控，
        # 缺这份证据就说明反控没跑，按 FAIL 计。
        ok = False
        replay_txt = 'replay=缺'
    detail = 'ack=%s zero=%s s sim move5s=%s m after_v=%s' % (
        d.get('ack_last'), '%.2f' % zero_sim if zero_sim is not None else 'NA',
        '%.3f' % move if move is not None else 'NA', after)
    if not ack_ok:
        detail += ' 无 ACK'
    if covers is not True:
        detail += ' 5s 窗口未张满'
    return ('PASS' if ok else 'FAIL'), detail, replay_txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    args = ap.parse_args()
    rows = [
        ('1 正常导航', judge_nav(args.root)),
        ('2 障碍安全', judge_obstacle(args.root)),
        ('3 定位失效', judge_loc(args.root)),
        ('4 取消安全', judge_cancel(args.root)),
    ]
    print('%-12s %-8s %-58s %s' % ('项', '判定', '读数', '备注'))
    bad = 0
    for name, (verdict, detail, extra) in rows:
        print('%-12s %-8s %-58s %s' % (name, verdict, detail, extra))
        if verdict != 'PASS':
            bad += 1
    print('—' * 100)
    print('四项：%d PASS / %d 未通过' % (len(rows) - bad, bad))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
