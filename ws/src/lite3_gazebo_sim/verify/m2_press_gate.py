#!/usr/bin/env python3
# m2_press_gate.py —— 加压档辅门的纯函数与零节点对表标定（小明 2026-10-08，小美 M2R8 路由 R37 的零节点那半）
#
# 为何要抽出来：辅门「sim2>=60 且 span>=35」里的 35 是她 M2R6 凑的数、无机理推导（她自记残余 #16），
#   而「这档到底挡不挡得住环境慢起造成的假达标」**从未被一次真反例证过**（她的原话）。
#   活体那半（真反例两条）要等窗（R41 三），但**判定式的可分性不需要仿真**——抽成纯函数即可对表标定。
#   注意本件与活体的关系：m2_smoke.sh 的 press 段自 H15 起改为本件的行为由 `press_gate()` 决定（活体读同一份代码，05 B2）；
#   在 SM 接线落地之前，本件的对表只证明「判定式本身两态可分」，不证明「活体跑走的正是它」——这句区分必须写清，不许混。
#
# 用法：python3 m2_press_gate.py            # 打印当前判定式对表（10 组，含边界）并自校验
#       python3 m2_press_gate.py --quiet     # 只在不过时打印
# rc：0 全部符合预期；5 有不符（判定式被改动而表未更新，或反之）。
import sys

TOL_ABS_SIM2 = 60.0     # 绝对门槛：取龄点2 的 sim 时刻（秒）
TOL_SUB_SPAN = 35.0     # 辅门：与取龄点1 的 sim 跨度下限（她 M2R6 #12 给的数，无机理推导）
TOL_SPAN_ONLY = 40.0    # 支路二：仅靠跨度达标所需的跨度


def press_gate(reached, sim2, span):
    """返回 (ok, branch)。式子出处：小美 M2R6 #12 原句，M2R7 结案 R20（一）追认 OR 口径。
       unreached 支路由 R27（二）要求补入（旧形态在 reached=False 时仍打 abs+span，易被快扫读成达标）。"""
    b_abs = (sim2 >= TOL_ABS_SIM2 and span >= TOL_SUB_SPAN)
    b_span = (span >= TOL_SPAN_ONLY)
    ok = bool(reached) and (b_abs or b_span)
    if not reached:
        branch = "unreached"
    elif b_abs:
        branch = "abs+span"
    elif b_span:
        branch = "span"
    else:
        branch = "NONE"
    return ok, branch


# 对表：前两行是在册真读数（sB3 与 M2R7 的 b3），其余是为把 35 这条辅门「挡住什么」钉死而设的反例与边界。
CASES = [
    # reached, sim2,  span,  期望 ok,   期望 branch,  说明
    (True,  62.36, 39.70, True,  "abs+span", "在册真读数 H12 sB3（原件 raw/retake_20261008_121830/：branch=abs+span sim2=62.36 span=39.70）"),
    (True,  62.45, 42.60, True,  "abs+span", "在册真读数 M2R7 b2（原件 m2r7_batch/b2.log：sim2=62.45 span=42.60，两支同过）"),
    (False, 101.47, 80.95, False, "unreached", "在册真读数 M2R7 b3（target=99999 未达 sim 目标走 300s 兜底；旧打印仍写 abs+span ⇒ 正是 R27（二）补 unreached 态的理由）"),
    (True,  62.00, 34.90, False, "NONE",      "反例一（R27 一 第一档）：sim2 过 60 但跨度 <35 ⇒ 辅门必须挡住"),
    (True,  62.00, 35.00, True,  "abs+span",  "辅门下沿（含）：跨度恰 35"),
    (True,  59.90, 36.00, False, "NONE",      "反例二（R27 一 第二档）：跨度落在 [35,40) 而 sim2 未到 60 ⇒ 不得算达标"),
    (True,  55.00, 41.00, True,  "span",      "支路二：仅靠跨度 >=40 过关（sim2 未到 60 也允许，OR 口径）"),
    (True,  60.00, 34.99, False, "NONE",      "边界对：sim2 恰 60 但跨度差 0.01 ⇒ 两支皆不过"),
    (True,  30.00, 39.90, False, "NONE",      "时钟区下沿：两点都早 ⇒ 不得算达标"),
    (True,  30.00, 40.10, True,  "span",      "同上但跨度过 40 ⇒ 支路二放行（记录口径，不是漏网）"),
    (False, 20.00,  10.00, False, "unreached", "reached=False 一律 branch=unreached（R27 二）"),
]


def main():
    quiet = "--quiet" in sys.argv
    bad = 0
    if not quiet:
        print("加压档辅门纯函数对表（常数 sim2>=%.2f 且 span>=%.2f，或 span>=%.2f）"
              % (TOL_ABS_SIM2, TOL_SUB_SPAN, TOL_SPAN_ONLY))
    for reached, sim2, span, exp_ok, exp_branch, note in CASES:
        ok, branch = press_gate(reached, sim2, span)
        hit = (ok == exp_ok and branch == exp_branch)
        if not hit:
            bad += 1
        if not quiet or not hit:
            print("%s reached=%-5s sim2=%6.2f span=%6.2f ⇒ ok=%-5s branch=%-9s（期望 %s/%s）｜%s"
                  % ("[OK]  " if hit else "[FAIL]", reached, sim2, span, ok, branch,
                     exp_ok, exp_branch, note))
    n_unreached = sum(1 for c in CASES if c[4] == "unreached")
    n_none = sum(1 for c in CASES if c[4] == "NONE")
    if not quiet:
        print("PRESS GATE: 用例 %d 组，不符 %d；其中期望 FAIL(NONE) %d 组、期望 unreached %d 组"
              % (len(CASES), bad, n_none, n_unreached))
        print("  辅门可分性结论：%s（反例一/二同时红 ⇒ 「环境慢起使 sim2 自然过 60」这类假达标确被挡住）"
              % ("成立" if bad == 0 else "不成立"))
        print("  未闭项：活体真反例两条仍属 R41（三）；SM 接线到本件后，此表才算「活体走的同一份式子」。")
    return 0 if bad == 0 else 5


if __name__ == "__main__":
    sys.exit(main())
