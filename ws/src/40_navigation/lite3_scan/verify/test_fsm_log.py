#!/usr/bin/env python3
"""fsm_log 的离线单测（无 ROS；合成日志，含 roslaunch 前缀形态与反向例）。"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fsm_log import attach_fsm, parse_transitions, read_summary, summarize  # noqa: E402

FAIL = 0


def check(cond, what):
    global FAIL
    print('%s  %s' % ('PASS' if cond else 'FAIL', what))
    if not cond:
        FAIL += 1


# 合成一段与 scan_replan_fsm.cpp cout 逐字同形的日志（含 roslaunch 前缀）
SAMPLE = """[scan_planner_node-1] process started
[FSM]: state: WAIT_TARGET
[TRIG]: from WAIT_TARGET to GEN_NEW_TRAJ
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
[FSM]: state: EXEC_TRAJ
[SAFETY]: from EXEC_TRAJ to REPLAN_TRAJ
[FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[FSM]: from EXEC_TRAJ to REPLAN_TRAJ
[FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[TRIG]: from EXEC_TRAJ to REPLAN_TRAJ
[FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[SAFETY]: from EXEC_TRAJ to EMERGENCY_STOP
[FSM]: state: EMERGENCY_STOP
"""


def main():
    trans = parse_transitions(SAMPLE)
    check(len(trans) == 9, '例一 迁移行数=9（[FSM]: state: 打印行不得混入）')
    s = summarize(trans)
    check(s['replan_by_source'] == {'SAFETY': 1, 'FSM': 1, 'TRIG': 1},
          '例二 触发源计数 SAFETY=1/FSM=1/TRIG=1（重规进入源即触发源）')
    check(s['emergency_stops'] == 1, '例三 EMERGENCY_STOP 计数=1')
    check(s['emergency_by_source'] == {'SAFETY': 1, 'FSM': 0, 'TRIG': 0},
          '例三b 急停归因 SAFETY=1（SAFETY 可不经 REPLAN 直接急停，只数 replan 会漏）')

    # 反向例一：无迁移的日志必须数出 0，不得把无关行当迁移
    s0 = summarize(parse_transitions('[INFO] nothing here\n[FSM]: state: EXEC_TRAJ\n'))
    check(s0['transitions'] == 0 and sum(s0['replan_by_source'].values()) == 0,
          '反向例一 无迁移日志⇒全 0（解析器没在瞎数）')
    # 反向例二：状态名拼错不得匹配（上游改表时本件会漏数⇒由例一/二的正例兜住）
    check(parse_transitions('[FSM]: from EXEC_TRAJ to REPLANN_TRAJ') == [],
          '反向例二 非法状态名不匹配')
    # 反向例三：调用源不在册（如手改日志）不计入触发源，但迁移本身仍数
    s3 = summarize(parse_transitions('[HACK]: from EXEC_TRAJ to REPLAN_TRAJ\n[HACK]: from EXEC_TRAJ to EMERGENCY_STOP'))
    check(s3['transitions'] == 2 and sum(s3['replan_by_source'].values()) == 0
          and s3['emergency_stops'] == 1 and sum(s3['emergency_by_source'].values()) == 0,
          '反向例三 未知调用源不入触发源/急停归因（总数仍数，不得瞎归因）')

    # 文件面：取不到读数必须报 error，不得写成零重规
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'scan.log')
        with open(p, 'w') as f:
            f.write(SAMPLE)
        check(read_summary(p)['replan_by_source']['SAFETY'] == 1, '例四 文件读取与内存解析同果')
        check('error' in read_summary(os.path.join(d, 'missing.log')),
              '例五 缺文件⇒error 字段（「没读到」≠「零重规」）')
        # attach_fsm：env 未设不附字段；设了才附
        os.environ.pop('SCAN_FSM_LOG', None)
        out = {}
        attach_fsm(out)
        check('fsm' not in out, '例六 env 未设⇒不附 fsm 字段（缺省≠零）')
        os.environ['SCAN_FSM_LOG'] = p
        attach_fsm(out)
        check(out.get('fsm', {}).get('transitions') == 9, '例七 env 指向真日志⇒附摘要')
        os.environ.pop('SCAN_FSM_LOG', None)

    print('FSM_LOG SELFTEST VERDICT: %s' % ('PASS' if FAIL == 0 else 'FAIL=%d' % FAIL))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())