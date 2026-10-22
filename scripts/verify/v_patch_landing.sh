#!/usr/bin/env bash
# v_patch_landing.sh —— 补丁稿「有载体」判据（★不写死版本号，从各稿首个一级标题现取）
#
# 为什么存在：R547／批203 的 P8 可复跑判据归纳后失效（v7.3 Y9 已登记一枚能力退化）。
#   原式＝「逐个 v4 补丁稿去 README 找版本串」，依赖 00_README 里那几行原文，而那几行已被归纳掉。
#   替代式不依赖任何原文行：★两侧都现取★——稿侧取各稿自身的版本章，载体侧取 v7 正文出现的版本串，
#   然后做双向差集。稿侧有而正文无 ⇒ 该稿的改读没落在载体上；正文有而稿侧无 ⇒ 正文引了不成立的稿。
#   ★本件不判「谁该落册」，只判「有稿必有形迹、有形迹必有稿」★——落册与否仍归真人裁。
#
# 用法：bash scripts/verify/v_patch_landing.sh [仓根]        # 缺省＝当前目录
#       bash scripts/verify/v_patch_landing.sh --selftest    # 自测面：四枚夹具
# rc：0 双向差集皆空；5 有差集／反控无牙／自测不合期望；9 前置缺失（目录/正文/稿件空集）
# 注：★空集不当「全绿」读★（与坑29 同族）：没有补丁稿、或正文取不到，一律 9 不出 0。
set -u

# ★自测面与被检面走同一段 python，只靠入参区分★。
# ★为什么把这行刻意压成「一行写完」★（R573 自曝，不是风格偏好）：
#   v_sh_py_syntax.sh 认「这段 heredoc 是 python」的条件是「同一行里既有 python 调用、又有 heredoc 重定向」。
#   上一版写成 `if …; then python3 - --selftest; else …; python3 - "$ROOT"; fi <<'PYEOF'`
#   ⇒ python 调用在 then/else 行、重定向在 fi 行，★两条件跨行 ⇒ SYN 判「该件无内嵌 python 段」并印绿★，
#   一旦这段 python 被写坏，SYN 抓不到。★不改 SYN 件本体（门禁串成员，改动须真人明令）★，
#   改本件的书写形，让两条件落在同一行 ⇒ SYN 段数从 0 变 1，这段从此受检。
ARG="${1:---root}"; if [ "$ARG" = "--selftest" ]; then SEED=--selftest; else ROOT="${1:-$PWD}"; if [ ! -d "$ROOT" ]; then echo "[前置缺失] 仓根不可达：$ROOT"; exit 9; fi; SEED="$ROOT"; fi; python3 - "$SEED" <<'PYEOF'
# -*- coding: utf-8 -*-
import os, re, sys

argv = sys.argv[1:]
SELFTEST = bool(argv) and argv[0] == "--selftest"
ROOT = os.getcwd() if SELFTEST else (argv[0] if argv else os.getcwd())
PATCH_DIR = os.path.join(ROOT, "note", "计划书v7")
V7 = os.path.join(PATCH_DIR, "21_ros1-slam中心计划书-v7.md")
VER = re.compile(r"v7\.[0-9]+")


def versions_in_first_heading(path):
    """只取文件首个一级标题里的版本串；标题之外的引用一概不算该稿的版本章。"""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("# "):
                return set(VER.findall(line))
    return set()


def landing_state(path):
    """落册态＝★先去本稿自己的「落册记录」节看节内容，找不到才退到文首另起句★。

    两处都踩过：
      ① 第一版只读文首十行 ⇒ v7.1 被读成「尚未落册」（它文首第 3 行写着「未落册」，
         真转正在 20 行后的「## 落册记录（转正，R491／批141）」节里）；
      ② 第二版改成「有落册记录节＝已落册」⇒ v7.2／v7.3／v7.4 全被读成已落册，
         而它们的节标题就叫「落册记录（待真人到签）」、节内首句逐字写着「本稿尚未落册」。
    ⇒ ★节标题不是判据、节内容才是★。
    """
    with open(path, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().split("\n")
    for i, line in enumerate(lines):
        if not line.startswith("## 落册记录"):
            continue
        seg = "\n".join(lines[i + 1:i + 4])
        if "尚未落册" in seg or "待真人" in line:
            return "尚未落册（节在册：%s）" % line[2:].strip()
        return "已落册（节在册：%s）" % line[2:].strip()
    for line in lines[:10]:
        if "本稿" in line and ("转正" in line or "已落册" in line):
            return "已落册（文首另起句）"
    if "未落册" in data_head(path):
        return "尚未落册"
    return "状态未明"


def data_head(path, n=10):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return "".join(fh.readlines()[:n])


def run(root):
    patch_dir = os.path.join(root, "note", "计划书v7")
    v7 = os.path.join(patch_dir, "21_ros1-slam中心计划书-v7.md")

    if not os.path.isdir(patch_dir):
        print("[前置缺失] 找不到补丁稿目录：%s" % patch_dir)
        return 9
    patches = sorted(f for f in os.listdir(patch_dir) if f.endswith(".md") and "补丁" in f)
    if not patches:
        print("[空集] 目录里没有补丁稿 —— 空集不当「全部落册」或「无需检查」读")
        return 9

    table, missing = [], []
    for name in patches:
        p = os.path.join(patch_dir, name)
        vers = versions_in_first_heading(p)
        if not vers:
            missing.append(name)
        table.append((name, vers, landing_state(p)))

    print("== 补丁稿清单（版本章一律取自各稿首个一级标题，不写死）==")
    for name, vers, state in table:
        print("  %-10s %-8s %s" % (",".join(sorted(vers)) or "（取不出）", state, name))
    if missing:
        print("[红] 下列稿件标题里取不出版本章，判据无法成立：")
        for name in missing:
            print("      %s" % name)
        return 5

    if not os.path.isfile(v7):
        print("[前置缺失] 找不到中心计划书正文：%s" % v7)
        return 9
    with open(v7, encoding="utf-8", errors="replace") as fh:
        in_body = set(VER.findall(fh.read()))

    declared = set()
    for _, vers, _ in table:
        declared |= vers
    only_file = sorted(declared - in_body)
    only_body = sorted(in_body - declared)

    print("")
    print("== 双向差集 ==")
    print("  稿侧有、正文无＝%s" % (", ".join(only_file) or "空"))
    print("  正文有、稿侧无＝%s" % (", ".join(only_body) or "空"))

    bad = 0
    if only_file:
        print("[红] 下列版本章只在稿件里、未进 v7 正文 ⇒ 该稿的改读没有落在载体上：%s" % ", ".join(only_file))
        bad += 1
    if only_body:
        print("[红] 下列版本章在 v7 正文里、却没有对应稿件 ⇒ 正文引了不成立的稿：%s" % ", ".join(only_body))
        bad += 1

    # ★反控：反的是「触发路径」，不是外层的 or 短路★
    #   第一版写 `declared - (in_body | {fake})`，而 declared ⊂ in_body 时差集本来就空
    #   ⇒ 塞不塞假章都是空 ⇒ 反控恒「失败」、本件永远不会绿。那是把自己的短路读成了缺陷。
    fake = "v7.9"
    fake_body = in_body | {fake}
    probe_body = fake_body - declared
    if fake not in probe_body:
        print("[反控失败] 塞入假版本章 %s 没落进「正文有、稿侧无」支 ⇒ 判据无牙" % fake)
        return 5
    print("[反控] 塞入假版本章 %s ⇒ 「正文有、稿侧无」当场翻红（该支 %d 枚）；"
          "「稿侧有、正文无」不受影响（%d 枚）" % (fake, len(probe_body), len(declared - fake_body)))

    if bad:
        return 5
    print("[判定] 双向差集皆空 ⇒ 有稿必有形迹、有形迹必有稿")
    return 0


def selftest():
    """四枚夹具，落在临时目录、不碰真仓：
    ① 双向一致 ⇒ 0；② 稿侧有、正文无 ⇒ 5；③ 正文有、稿侧无 ⇒ 5；
    ④ 补丁稿目录空集 ⇒ 9（★空集不当全绿★）。"""
    import shutil, tempfile

    base = tempfile.mkdtemp(prefix="vpl_selftest_")
    n = [0]

    def build(with_patch, body_ver):
        root = os.path.join(base, "r%03d" % n[0]); n[0] += 1
        d = os.path.join(root, "note", "计划书v7")
        os.makedirs(d)
        if with_patch:
            with open(os.path.join(d, "21_ros1-slam中心计划书-v7.1-x补丁.md"), "w", encoding="utf-8") as fh:
                fh.write("# 21_ros1-slam中心计划书-v7.1-x补丁（稿）\n\n## 落册记录（转正）\n\n- 已落册。\n")
        with open(os.path.join(d, "21_ros1-slam中心计划书-v7.md"), "w", encoding="utf-8") as fh:
            fh.write("# 21 · 中心计划书（v7）\n\n%s\n" % body_ver)
        return root

    cases = [("双向一致", build(True, "v7.1"), 0),
             ("稿侧有正文无", build(True, "v7.0"), 5),
             ("正文有稿侧无", build(True, "v7.1 v7.2"), 5),
             ("空集", build(False, "v7.1"), 9)]
    bad = []
    for title, root, want in cases:
        rc = run(root)
        ok = "过" if rc == want else "★不合★"
        print("  [自测] %-12s 期望 rc=%d 实得 rc=%d %s" % (title, want, rc, ok))
        if rc != want:
            bad.append(title)
    shutil.rmtree(base, ignore_errors=True)
    if bad:
        print("★自测不合期望★＝%s" % "／".join(bad))
        return 5
    print("  自测四枚全合期望")
    return 0


sys.exit(selftest() if SELFTEST else run(ROOT))
PYEOF
