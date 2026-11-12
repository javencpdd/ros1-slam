#!/usr/bin/env python3
"""fsm_log.py —— SCAN FSM 状态迁移的离线解析（2026-11-12，审计缺口 c）。

背景：70_scan 的 roslaunch stdout（harness 收在 $RUN/scan.log）里本就有 FSM 迁移行
（scan_replan_fsm.cpp changeFSMExecState 的 cout：`[调用源]: from X to Y`），
但旧证据包只封存探针 stdout ⇒ 「重规由谁触发」（周期 FSM／安全 SAFETY／目标 TRIG）
只能靠轨迹身份反推。本件把迁移行抽出来并数触发源，**只作诊断、不参与 P4 判定**
（判定单一真值仍在 p4_replan.py 的判据行）。

纯标准库、不 import ROS ⇒ 宿主离线可测（test_fsm_log.py）。
"""
import os
import re

# 与 scan_replan_fsm.cpp L536/L549 的 state_str 同表；上游改表须同步本行
STATES = ("INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP")
_TRANS = re.compile(r"\[([A-Za-z_]+)\]: from (" + "|".join(STATES) + r") to (" + "|".join(STATES) + r")")


def parse_transitions(text):
    """从整段日志抽迁移行，返回 [(调用源, 前态, 后态)]；roslaunch 前缀不影响匹配。"""
    return [(m.group(1), m.group(2), m.group(3)) for m in _TRANS.finditer(text)]


def summarize(trans):
    """触发源计数。REPLAN_TRAJ 的进入源即触发源：SAFETY=碰撞检查、FSM=周期、TRIG=目标/参考路径。
    急停另列：SAFETY 可不经 REPLAN_TRAJ 直接进 EMERGENCY_STOP（撞点在 emergency_time_ 内且
    planFromCurrentTraj 失败），只数 replan 会把这条安全路径漏成「SAFETY=0」。"""
    s = {"transitions": len(trans),
         "replan_by_source": {"SAFETY": 0, "FSM": 0, "TRIG": 0},
         "emergency_stops": 0,
         "emergency_by_source": {"SAFETY": 0, "FSM": 0, "TRIG": 0}}
    for src, _pre, post in trans:
        if post == "REPLAN_TRAJ" and src in s["replan_by_source"]:
            s["replan_by_source"][src] += 1
        if post == "EMERGENCY_STOP":
            s["emergency_stops"] += 1
            if src in s["emergency_by_source"]:
                s["emergency_by_source"][src] += 1
    return s


def read_summary(path):
    """读日志文件给摘要；取不到读数返回 error 字段——「没读到」不得写成「零重规」。"""
    try:
        with open(path, "r", errors="replace") as f:
            return summarize(parse_transitions(f.read()))
    except OSError as e:
        return {"error": repr(e)[:120]}


def attach_fsm(out, env="SCAN_FSM_LOG"):
    """探针 JSON 的诊断挂点：env 指到 scan.log 时附摘要；未设则不附（缺省≠零）。"""
    path = os.environ.get(env)
    if path:
        out["fsm"] = read_summary(path)
    return out