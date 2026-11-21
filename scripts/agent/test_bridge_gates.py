#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge.py 四道质量闸的自测：A new / B check / C claim / D done。

为什么要这四道闸
----------------
历史读数：115 张任务卡里 100 张「目标」停在占位符、99 张「验收标准」为空，
却一路 claim/handoff/done 走完了。原因不是谁偷懒，而是**工具全程没问过**
「要交付什么」「怎么算做完」「凭什么说做完了」。本自测就是钉住这四问。

三连结构（缺一条就是空测）
--------------------------
1. 主测：正反两向都观测——该拦的拦住、该放的放行。
2. 注入有效性：把占位符原文当目标喂进去，必须仍被判空；否则「判空」只是判了它自己。
3. 反控：补齐目标/给出真证据后必须放行，证明拦截来自目标与证据本身，
   不是无脑拒绝（无脑拒绝也能让主测全绿）。

跑法：python3 scripts/agent/test_bridge_gates.py
退出码 0 = 全绿；非 0 = 有用例失败（会逐条打印期望与实际）。
"""

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# 允许指向被注入缺陷的副本，用来证明这套自测真的会红（自测自己也可能是瞎的）
BRIDGE = os.environ.get("BRIDGE_UNDER_TEST") or os.path.join(HERE, "bridge.py")
PLACEHOLDER_GOAL = "一句话说清要交付什么"

FAILS = []
CHECKS = [0]


def ok(cond, what, detail=""):
    CHECKS[0] += 1
    if cond:
        return True
    FAILS.append("%s%s" % (what, ("　→ " + detail) if detail else ""))
    return False


def run(root, *argv):
    """跑 bridge.py，返回 (rc, stdout, stderr)。--root 必须放在子命令之前。"""
    cmd = [sys.executable, BRIDGE, "--root", root] + [str(x) for x in argv]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       universal_newlines=True)
    return p.returncode, p.stdout, p.stderr


def card(root, tid):
    with open(os.path.join(root, ".ai", "tasks", "%s.md" % tid), encoding="utf-8") as f:
        return f.read()


def section(text, heading):
    """取某个二级小节的正文（到下一个 ## 之前）。"""
    i = text.find(heading + "\n")
    if i < 0:
        return ""
    start = i + len(heading) + 1
    j = text.find("\n## ", start)
    return text[start:j if j >= 0 else len(text)].strip()


def main():
    root = tempfile.mkdtemp(prefix="bridge-gates-")
    try:
        rc, out, err = run(root, "init")
        if rc != 0:
            print("准备失败：init rc=%d\n%s\n%s" % (rc, out, err))
            return 2
        os.makedirs(os.path.join(root, "scripts"), exist_ok=True)
        os.makedirs(os.path.join(root, "note"), exist_ok=True)

        # ---------------- A：new 收目标与验收
        rc, out, err = run(root, "new", "空卡", "--as", "human", "--scope", "scripts")
        tid_empty = out.strip().splitlines()[-1] if out.strip() else ""
        ok(rc == 0 and tid_empty, "A1 空卡应创建成功", "rc=%d out=%r" % (rc, out))
        body = card(root, tid_empty)
        ok(PLACEHOLDER_GOAL in body, "A2 不带 --goal 时正文应留占位符")
        ok("- [ ]" in section(body, "## 验收标准"), "A3 不带 --accept 时验收标准应为空占位")
        ok("警告" in err and "--goal" in err, "A4 不带 --goal 应打警告并给出补法", "err=%r" % err)

        rc, out, err = run(root, "new", "满卡", "--as", "human", "--scope", "scripts",
                           "--goal", "把 X 做成 Y", "--accept", "验一条", "验二条")
        tid_full = out.strip().splitlines()[-1] if out.strip() else ""
        ok(rc == 0 and tid_full, "A5 带目标/验收的卡应创建成功", "rc=%d out=%r" % (rc, out))
        body = card(root, tid_full)
        ok("把 X 做成 Y" in section(body, "## 目标"), "A6 目标应原样落进正文")
        ok(PLACEHOLDER_GOAL not in body, "A7 带目标时不应残留占位符")
        acc = section(body, "## 验收标准")
        ok("- [ ] 验一条" in acc and "- [ ] 验二条" in acc, "A8 验收项应逐条成 checkbox",
           "实际=%r" % acc)
        ok("警告" not in err, "A9 带全参数时不应打警告", "err=%r" % err)

        # ---------------- C：claim 拒绝空目标，补齐后放行（反控）
        rc, out, err = run(root, "claim", tid_empty, "--as", "workbuddy")
        ok(rc != 0, "C1 空目标卡 claim 应失败", "rc=%d" % rc)
        ok("拒绝认领" in err, "C2 失败原因应写明拒绝认领", "err=%r" % err)
        ok("bridge.py goal" in err, "C3 失败信息里应给出补救命令", "err=%r" % err)

        # 注入有效性：占位符原文当目标，必须仍被判空
        rc, out, err = run(root, "goal", tid_empty, PLACEHOLDER_GOAL, "--as", "human")
        ok(rc != 0 and "占位符" in err, "C4 把占位符原文当目标写入应被拒",
           "rc=%d err=%r" % (rc, err))
        ok(PLACEHOLDER_GOAL in card(root, tid_empty), "C5 被拒后正文不应被改动")

        rc, out, err = run(root, "goal", tid_empty, "补上的目标", "--as", "human",
                           "--accept", "可核一条")
        ok(rc == 0, "C6 补目标应成功", "rc=%d err=%r" % (rc, err))
        body = card(root, tid_empty)
        ok("补上的目标" in section(body, "## 目标"), "C7 补的目标应落进正文")
        ok(PLACEHOLDER_GOAL not in body, "C8 补目标后占位符应消失")
        ok("- [ ] 可核一条" in section(body, "## 验收标准"), "C9 补的验收项应落进正文")

        rc, out, err = run(root, "claim", tid_empty, "--as", "workbuddy")
        ok(rc == 0, "C10 反控：补齐目标后 claim 必须放行", "rc=%d err=%r" % (rc, err))

        # ---------------- B：check 报出空目标/空验收，不误报已填的
        rc, out, err = run(root, "new", "又一张空卡", "--as", "human", "--scope", "scripts")
        tid_bad = out.strip().splitlines()[-1] if out.strip() else ""
        rc, out, err = run(root, "check")
        ok("`%s` 没有目标" % tid_bad in out, "B1 check 应报出空目标的卡", "out=%r" % out)
        ok("`%s` 没有验收标准" % tid_bad in out, "B2 check 应报出空验收的卡", "out=%r" % out)
        ok("`%s` 没有目标" % tid_full not in out, "B3 check 不应误报已填目标的卡")
        ok("`%s` 没有目标" % tid_empty not in out, "B4 check 不应误报补过目标的卡")
        ok("历史积压" in out, "B5 应对历史空目标卡给一行汇总而不刷屏", "out=%r" % out)

        # ---------------- D：done 必须给真证据
        rc, out, err = run(root, "done", tid_full)
        ok(rc != 0, "D1 无证据 done 应失败", "rc=%d" % rc)
        ok("必须给出证据" in err, "D2 失败原因应写明要证据", "err=%r" % err)

        rc, out, err = run(root, "done", tid_full, "--evidence", "note/并不存在.md")
        ok(rc != 0 and "不存在" in err, "D3 证据路径不存在应失败", "rc=%d err=%r" % (rc, err))
        ok("已完成。" not in card(root, tid_full), "D4 失败时不应把卡改成完成")

        real = os.path.join(root, "note", "证据.md")
        with open(real, "w", encoding="utf-8") as f:
            f.write("evidence\n")
        rc, out, err = run(root, "done", tid_full, "--summary", "做完了",
                           "--evidence", "note/证据.md")
        ok(rc == 0, "D5 反控：给了真实证据必须放行", "rc=%d err=%r" % (rc, err))
        body = card(root, tid_full)
        ok("证据：" in body and "note/证据.md" in body, "D6 完成记录应写入证据清单")
        ok("状态：`done`" in body, "D7 卡头状态应刷新成 done")

        rc, out, err = run(root, "check")
        ok("`%s` 没有目标" % tid_full not in out, "D8 done 后不应再报该卡缺目标")

        # ---------------- E：老卡的目标只写在正文里（state 无该字段），判据不得假红
        # 历史 115 张卡就是这么来的：正文有目标，state.json 里根本没有 goal 这个键。
        # 若判据只读 state，会把它们全误判成空 —— 假红比闸门闭眼更坏，人会学会无视它。
        rc, out, err = run(root, "new", "老卡", "--as", "human", "--scope", "scripts")
        tid_legacy = out.strip().splitlines()[-1] if out.strip() else ""
        tf = os.path.join(root, ".ai", "tasks", "%s.md" % tid_legacy)

        def _rewrite(text, heading, body):
            head = heading + "\n"
            i = text.find(head)
            if i < 0:
                return text
            start = i + len(head)
            j = text.find("\n## ", start)
            return text[:start] + "\n" + body + "\n" + text[j:]

        with open(tf, encoding="utf-8") as f:
            txt = f.read()
        txt = _rewrite(txt, "## 目标", "正文里写全的老目标")
        txt = _rewrite(txt, "## 验收标准", "- [ ] 正文里的老验收项")
        with open(tf, "w", encoding="utf-8") as f:
            f.write(txt)

        rc, out, err = run(root, "check")
        ok("`%s` 没有目标" % tid_legacy not in out,
           "E1 目标只写在正文的老卡不应被判空（假红）", "out=%r" % out)
        ok("`%s` 没有验收标准" % tid_legacy not in out,
           "E2 验收只写在正文的老卡不应被判空（假红）")
        rc, out, err = run(root, "claim", tid_legacy, "--as", "workbuddy")
        ok(rc == 0, "E3 正文有目标的老卡应能正常 claim", "rc=%d err=%r" % (rc, err))

    finally:
        shutil.rmtree(root, ignore_errors=True)

    total = CHECKS[0]
    if FAILS:
        print("✘ %d/%d 条失败：" % (len(FAILS), total))
        for f in FAILS:
            print("   -", f)
        return 1
    print("✔ %d 条全绿（A new 9／B check 5／C claim 10／D done 8 的对应断言）" % total)
    return 0


if __name__ == "__main__":
    sys.exit(main())
