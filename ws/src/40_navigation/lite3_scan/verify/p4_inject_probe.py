#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_inject_probe.py —— P4 扰动注入的**回读探针**（#55 O-9 根因核查，0 改口径）

为什么要它：R63 有 3/6 跑判「扰动未被占据图看见」（半径内 0/0/8 点），而 p4_replan.py 里
`grep -c GetModelState` = **0** —— 注入只发 `/gazebo/set_model_state`，**从不回读模型实际位姿**。
于是「雷达没看见」与「箱子根本没挪动」在现有证据里**不可分**。这与 P3 当年「goal 投递失能却被判通过」
同型（在册处置＝加投递阳性对照），所以本件只做一件事：**把注入这件事变成有回读的观测**。

判据口径（三态，与 P4 同）：
  PASS(0)  四个目标点全部落地、20s 内不回位，且阴性对照证明回读通道有分辨力
  FAIL(1)  有目标点未落地或中途回位 ⇒ O-9 的根因指向**注入面**，不是感知面
  INVALID(9) 服务不可用/世界未起 ⇒ 前置不成立，不得计入任何一侧

零改点：不动 p4_replan.py、不动 SEE_RADIUS/SEE_MIN_PTS/AVOID、不动世界件（只运行时搬 obs_5 并还原）。
"""
import json
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState          # 注意：ModelState 在 .msg 不在 .srv（首次跑 ImportError 就是这个）
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Twist

# R63 三个"未被看见"的注入点 ＋ 一个"被看见"的对照点（原样取自各跑 p4.json，未再算）
TARGETS = [("s13_unseen", (-0.805, -0.575)), ("s15_unseen", (-1.087, 0.169)),
           ("s16_unseen", (0.019, -0.991)), ("s11_seen", (-1.032, 0.490))]
MODEL = os.environ.get("P4_MODEL", "obs_5")
INJECT_Z = 0.40
HOLD_S = float(os.environ.get("PROBE_HOLD", "20"))          # 落地后观察多久（P4 的看见窗是 30s 墙钟）
POLL = 0.5
GHOST = "obs_5_ghost_not_in_world"                          # 阴性对照用的不存在模型


def get_svc():
    rospy.wait_for_service("/gazebo/get_model_state", timeout=60)
    return rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)


def set_svc():
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
    return rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


def read_pose(get, name):
    try:
        r = get(name, "world")
        return (round(r.pose.position.x, 3), round(r.pose.position.y, 3), round(r.pose.position.z, 3)), r.success, r.status_message
    except Exception as e:
        return None, False, "EXC %r" % (e,)


def probe_one(get, setst, tag, xy, log):
    """把 MODEL 搬到 xy，随后每 POLL 秒回读一次，共 HOLD_S 秒；返回该目标的判定。"""
    tgt = (xy[0], xy[1], INJECT_Z)
    ms = ModelState()
    ms.model_name = MODEL
    ms.reference_frame = "world"
    ms.pose = Pose(position=Point(*tgt))
    ms.twist = Twist()
    t0 = time.time()
    try:
        rs = setst(ms)
    except Exception as e:
        log.append("%-12s 调用 set_model_state 异常 %r" % (tag, e))
        return {"tag": tag, "target": tgt, "landed": False, "reason": "set_exc"}
    first_ok = None
    samples, revert = [], None
    while time.time() - t0 < HOLD_S:
        p, ok, _ = read_pose(get, MODEL)
        d = None if p is None else round(((p[0] - tgt[0]) ** 2 + (p[1] - tgt[1]) ** 2) ** 0.5, 3)
        samples.append((round(time.time() - t0, 1), p, d))
        if d is not None and d <= 0.05:
            if first_ok is None:
                first_ok = round(time.time() - t0, 1)
        elif first_ok is not None and d is not None and d > 0.20:
            revert = round(time.time() - t0, 1)
            break
        rospy.sleep(POLL)
    last = samples[-1][1] if samples else None
    lastd = samples[-1][2] if samples else None
    landed = first_ok is not None
    log.append("%-12s 目标(%6.3f,%6.3f) set.success=%s 首次命中=%s 末次读数=%s 末次偏差=%s 回位=%s"
               % (tag, tgt[0], tgt[1], getattr(rs, "success", "?"),
                  ("%.1fs" % first_ok) if first_ok is not None else "从未",
                  last, lastd, ("%.1fs" % revert) if revert else "无"))
    return {"tag": tag, "target": tgt, "set_success": bool(getattr(rs, "success", False)),
            "landed": landed, "t_first_ok": first_ok, "revert_at": revert,
            "last_pose": last, "last_dev": lastd, "n_samples": len(samples)}


def main():
    out = os.environ.get("PROBE_OUT")
    rospy.init_node("p4_inject_probe", anonymous=True)
    log = []

    def say(s):
        print(s)
        log.append(s)

    say("=== P4 注入回读探针（模型=%s，观察 %.0fs/点，轮询 %.1fs）===" % (MODEL, HOLD_S, POLL))
    try:
        get, setst = get_svc(), set_svc()
    except Exception as e:
        say("P4 注入探针 = INVALID|/gazebo 的 get/set_model_state 服务不可用 %r" % (e,))
        return 9
    orig, ok, msg = read_pose(get, MODEL)
    if not ok or orig is None:
        say("P4 注入探针 = INVALID|读不到 %s 的初始位姿（%s）⇒ 世界未起或模型名不对" % (MODEL, msg))
        return 9
    say("初始位姿（应＝世界件里的原位）：%s" % (orig,))

    # 阴性对照：回读通道必须能识破"不存在的模型"，否则它的"成功"一文不值（坑29/坑41 族）
    gp, gok, gmsg = read_pose(get, GHOST)
    say("阴性对照（查不存在的 %s）：success=%s 位姿=%s ｜ %s" % (GHOST, gok, gp, gmsg[:60]))
    ghost_ok = (not gok) or gp is None

    res = []
    for tag, xy in TARGETS:
        res.append(probe_one(get, setst, tag, xy, log))
        bk = ModelState(model_name=MODEL, reference_frame="world",
                        pose=Pose(position=Point(*orig)),
                        twist=Twist())
        try:
            setst(bk)
        except Exception as e:
            say("还原失败（后续点仍在同一世界内，继续）：%r" % (e,))
        rospy.sleep(1.0)
    fin, fok, _ = read_pose(get, MODEL)
    say("收尾回读：%s（应回到原位 %s）" % (fin, orig))

    bad = [r["tag"] for r in res if not r["landed"]]
    rev = [r["tag"] for r in res if r.get("revert_at")]
    if not ghost_ok:
        say("P4 注入探针 = FAIL|回读通道对不存在的模型也报成功 ⇒ 探针无分辨力，任何 PASS 都不成立")
        rc = 1
    elif bad:
        say("P4 注入探针 = FAIL|%d/%d 个注入点从未落地（%s）⇒ O-9 根因指向**注入面**（箱子没挪），不是感知面"
            % (len(bad), len(res), "、".join(bad)))
        rc = 1
    elif rev:
        say("P4 注入探针 = FAIL|落地后回位（%s）⇒ 注入不持久，看见窗内可能已复位" % "、".join(rev))
        rc = 1
    else:
        say("P4 注入探针 = PASS|四点全部落地且 %.0fs 内不回位（含 R63 三个「未见」点）"
            "⇒ 注入面被排除，O-9 的根因只能在感知/建图面" % HOLD_S)
        rc = 0
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"verdict_rc": rc, "orig": orig, "ghost_ok": ghost_ok,
                                 "final": fin, "targets": res}, ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())
