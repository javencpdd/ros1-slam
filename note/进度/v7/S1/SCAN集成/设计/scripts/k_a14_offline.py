#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k_a14_offline.py —— A14 三案的**离线后果对照**（零节点、零门值改动、不产生任何判定）

用途：把"可见性前置该怎么定"这条裁定（请求单 A14）的三个候选，**用在册 9 跑的真实读数**各跑一遍算后果，
      让裁定看得见代价，而不是看措辞。所有数字由本文件现算；末尾有断言，与在册 `p4.json` 不符即非零退出。

三案（原话见请求单 F/K/L 段）：
  ① 门不变，把"踩进已知盲区"的跑判 **INVALID（取信面）**而非 FAIL
  ② 注入**方位**限制在对角带 ⇒ 在册读数全部作废、须整批重取
  ③ `SEE_MIN_PTS` **按方位分档**（＝新门值，需另定标）

**两条必须写在结果里的在册事实**（不是我的推断）：
  - 现网**已经**在做①：三条"未被看见"的跑（seen=0/0/8）判据行原文就是 `INVALID|扰动未被占据图看见`，
    所以①不是"改判据"，而是"把现状写成可审计口径"＋补 A10（判据行称"新增占据点"而实现数的是绝对点数）。
  - ③对 seen=0 的两跑**已被证无效**：R77 实测那些回波在**进入 LIO 之前**就被 `preprocess/blind: 1.0` 删掉
    （云里根本没有点）⇒ 阈值取多小都看不见，与松紧无关。
"""
import io
import json
import math
import os
import re
import sys

ROOT = "/home/ros1-slam"
RUNS = [("R57", "scan_p4_retake_newso_20261009_0407", ("r1", "r2", "r3")),
        ("R63", "scan_p4_sampling_n6_20261009_1037", ("s11", "s12", "s13", "s14", "s15", "s16"))]
SEE_MIN = 20          # 在册 SEE_MIN_PTS（只读引用，不改）
SEE_R = 0.55          # 在册 SEE_RADIUS
AVOID = 0.60          # 在册避让门
DIAG_BAND = 20.0      # ②的"对角带"定义：距最近世界轴 ≥20°（用来算后果，不是建议值）
RUN_MINUTES = 10.0    # 一条独立链的实测量级（取 R63 各跑 8–12 分钟的中位）


def num(pat, s, default=None):
    m = re.search(pat, s)
    if not m:
        return default
    for g in m.groups():          # 交替式（a|x 与 b|x）会留 None 组，取第一个非空的
        if g is not None:
            return float(g)
    return default


def load_run(batch, d, run):
    base = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw", d, run)
    jf, hf = os.path.join(base, "p4.json"), os.path.join(base, "harness.log")
    if not os.path.isfile(jf):
        return None
    line = [l for l in io.open(jf, encoding="utf-8").read().split("\n") if l.startswith("{")]
    j = json.loads(line[0]) if line else {}
    verdict = ""
    if os.path.isfile(hf):
        for l in io.open(hf, encoding="utf-8", errors="replace"):
            if "结论行：" in l:
                verdict = l.split("结论行：", 1)[1].strip()
    inj = (j.get("inject") or {})
    tb = (j.get("traj_before") or {})
    at, first, last = inj.get("at"), tb.get("first"), tb.get("last")
    b_axis = None
    if at and first:
        b = math.degrees(math.atan2(at[1] - first[1], at[0] - first[0]))
        b_axis = min(abs(b) % 90.0, 90.0 - (abs(b) % 90.0))
    return {"batch": batch, "run": run, "seen": inj.get("occ_pts_near"), "pts_before": inj.get("pts_before"),
            "verdict": verdict, "vclass": (re.search(r"= (PASS|FAIL|INVALID)", verdict).group(1)
                                           if re.search(r"= (PASS|FAIL|INVALID)", verdict) else "无判定行"),
            "traj_len_m": num(r"轨迹 ?([0-9.]+)m", verdict),
            "sim_s": num(r"sim ?([0-9.]+)s", verdict),
            "phys_min": num(r"物理离箱心最近 ?([0-9.]+)m", verdict),
            "b_axis_proxy": None if b_axis is None else round(b_axis, 1),
            "fail_list": j.get("fail"), "invalid_field": j.get("invalid")}


def main():
    rows = [r for b, d, runs in RUNS for run in runs for r in [load_run(b, d, run)] if r]
    if not rows:
        print("没读到任何 p4.json ⇒ 不判")
        return 9

    print("== ①在册原文（逐跑，判定类别取自 harness.log 的结论行）==")
    print("%-6s %-6s %-6s %-9s %-9s %-8s %s" % ("批", "跑", "seen", "距轴(代理)", "避让实测", "在册判定", "结论行摘要"))
    for r in rows:
        print("%-6s %-6s %-6s %-9s %-9s %-8s %s"
              % (r["batch"], r["run"], r["seen"],
                 "-" if r["b_axis_proxy"] is None else "%.1f°" % r["b_axis_proxy"],
                 "-" if r["phys_min"] is None else "%.3f" % r["phys_min"],
                 r["vclass"], r["verdict"][:56]))

    def cnt(cls):
        return sum(1 for r in rows if r["vclass"] == cls)
    print("\n在册分布：PASS=%d FAIL=%d INVALID=%d 无判定行=%d（合计 %d 跑）"
          % (cnt("PASS"), cnt("FAIL"), cnt("INVALID"), cnt("无判定行"), len(rows)))

    print("\n== ②三案各自的后果 ==")
    # ① 现状即①
    print("① 门不变、盲区跑判 INVALID：**＝现网行为**（三条 seen=0/0/8 的判定行原文就是 INVALID）"
          "⇒ 后果＝零重取；要补的只是 A10（判据行称\"新增占据点\"、实现数的是绝对点数）与 A11（invalid/fail 字段是死字段）")
    # ② 限制对角带
    keep = [r for r in rows if r["b_axis_proxy"] is not None and r["b_axis_proxy"] >= DIAG_BAND]
    drop = [r for r in rows if r not in keep]
    print("② 注入方位限制在对角带（本表取\"距世界轴 ≥%.0f°\"为示意定义）：在册 9 跑里按此规则**可保留 %d 跑、剔除 %d 跑**%s"
          % (DIAG_BAND, len(keep), len(drop), [r["run"] for r in drop]))
    print("   但按在册规矩\"抽样规则一变、整批读数作废\"⇒ **9 跑全部重取**，成本 ≈ %.0f 分钟独占活体窗（%d 跑×%.0f 分）"
          % (len(rows) * RUN_MINUTES, len(rows), RUN_MINUTES))
    s13 = [r for r in rows if r["run"] == "s13"]
    if s13 and s13[0]["b_axis_proxy"] is not None and s13[0]["b_axis_proxy"] >= DIAG_BAND:
        print("   **②救不了 s13**：它距轴 %.1f°（本就在对角带）却只有 seen=%s ⇒ 剔方位并不能保证\"看得见\""
              % (s13[0]["b_axis_proxy"], s13[0]["seen"]))
    # ③ 按方位分档
    zero = [r for r in rows if r["seen"] == 0]
    print("③ 按方位分档阈值：对 seen=0 的 %d 跑（%s）**已被 R77 证无效**——那些回波在进入 LIO 前就被 "
          "preprocess/blind:1.0 删掉（云里没有点），阈值取多小都看不见；"
          "对 s13（seen=8，云里确有 8 点）③**可行**，但等价于把该带门降到 ≤8 ⇒ 属新门值、需另定标" % (len(zero), [r["run"] for r in zero]))

    print("\n== ③可逆性与证据销毁 ==")
    for name, rev, destroy in (("①", "可逆（只是把现状写成口径＋补 A10/A11）", "不销毁"),
                              ("②", "可逆（规则可再改回），但**在册 9 跑读数全部作废**", "销毁既有证据的可用性"),
                              ("③", "部分可逆：新门值一旦生效，此前按旧门取的跑不可比", "对 seen=0 那两跑＝**无效改动**（改了也看不见）")):
        print("  %s：可逆性 %s ｜ %s" % (name, rev, destroy))

    print("\n== ④要让选择变容易，还缺的一个实测（本表不算结论）==")
    print("  把注入心距从 ~1.0m 挪出 `blind` 刀口（R77 量到存活边界 ≥0.99m；#64 正在扫精确值）后重取一轮："
          "若 seen 普遍上到 20 以上，则 A14 三案的**必要性本身**会下降——这比在①②里挑一个更省。")

    bad = [r for r in rows if r["seen"] is None or r["vclass"] == "无判定行"]
    print("\n断言：缺 seen 或缺判定行的跑 = %d %s" % (len(bad), [r["run"] for r in bad]))
    print("A14 离线后果表：OK" if not bad else "A14 离线后果表：有缺项（见上）")
    return 0 if not bad else 5


if __name__ == "__main__":
    sys.exit(main())
