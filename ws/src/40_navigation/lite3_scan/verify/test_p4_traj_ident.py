#!/usr/bin/env python3
"""P4 轨迹身份诊断单测：bs_degenerate / bs_ident（p4_replan.py）。

bs_degenerate 是 2026-11-12 新增的零长度轨迹标记——急停也会换轨迹身份，不标记会把
replanned=True 误读成避让重规。本件给它和上游 bs_ident 补自动化（有代码≠有自动化）。
只测纯函数与身份链路，不起 ROS 节点、不碰判定门值。

运行：需 ROS 环境（p4_replan 模块级 import rospy/msgs），在 sim-viz 容器内跑：
  source /opt/ros/noetic/setup.bash && source /home/ros1-slam/ws/devel/setup.bash
  python3 test_p4_traj_ident.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p4_replan as p4  # noqa: E402  (模块级 import rospy，须容器内)

CHECKS = []


def check(name, cond):
    CHECKS.append((name, bool(cond)))
    print(("PASS  " if cond else "FAIL  ") + name)


def fake_bs(ctrl, secs=10, nsecs=0):
    return SimpleNamespace(
        start_time=SimpleNamespace(secs=secs, nsecs=nsecs),
        pos_pts=[SimpleNamespace(x=x, y=y) for x, y in ctrl])


# --- bs_degenerate 纯函数面 ---
check("None 身份 ⇒ 非退化（没身份≠零长度）", p4.bs_degenerate(None) is False)
check("空 dict ⇒ 非退化", p4.bs_degenerate({}) is False)
check("缺 last ⇒ 非退化", p4.bs_degenerate({"first": [0, 0]}) is False)
check("缺 first ⇒ 非退化", p4.bs_degenerate({"last": [0, 0]}) is False)
check("首末重合（原点）⇒ 退化", p4.bs_degenerate({"first": [0, 0], "last": [0, 0]}) is True)
check("首末重合（非原点）⇒ 退化", p4.bs_degenerate({"first": [0.5, -1.25], "last": [0.5, -1.25]}) is True)
check("反控：首末不同 ⇒ 非退化（正常轨迹不得误标）", p4.bs_degenerate({"first": [0, 0], "last": [1, 0]}) is False)
check("空控制点列表（falsy）⇒ 非退化", p4.bs_degenerate({"first": [], "last": []}) is False)
check("first/last 为 None ⇒ 非退化", p4.bs_degenerate({"first": None, "last": None}) is False)

# --- bs_ident → bs_degenerate 身份链路（fake Bspline）---
check("bs_ident(None) ⇒ None", p4.bs_ident(None) is None)

zero = p4.bs_ident(fake_bs([(1.0, 2.0)]))
check("单控制点轨迹 bs_ident 首末重合", zero["first"] == zero["last"] == [1.0, 2.0])
check("急停零长度轨迹 ⇒ 判退化（replanned 不得读成避让）", p4.bs_degenerate(zero) is True)

normal = p4.bs_ident(fake_bs([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
check("正常轨迹 bs_ident 首末可区分", normal["first"] == [0.0, 0.0] and normal["last"] == [2.0, 0.0])
check("反控：正常轨迹 ⇒ 非退化", p4.bs_degenerate(normal) is False)
check("bs_ident 段数=控制点数", normal["n_ctrl"] == 3)
check("bs_ident start_time 秒+纳秒合成", p4.bs_ident(fake_bs([(0, 0)], secs=7, nsecs=500000000))["start_time"] == 7.5)

empty = p4.bs_ident(fake_bs([]))
check("空控制点 bs_ident 首末为 None", empty["first"] is None and empty["last"] is None)
check("空控制点 ⇒ 非退化", p4.bs_degenerate(empty) is False)

failed = [n for n, ok in CHECKS if not ok]
print("P4_TRAJ_IDENT SELFTEST: %d/%d" % (len(CHECKS) - len(failed), len(CHECKS)))
if failed:
    print("P4_TRAJ_IDENT SELFTEST VERDICT: FAIL")
    sys.exit(1)
print("P4_TRAJ_IDENT SELFTEST VERDICT: PASS")