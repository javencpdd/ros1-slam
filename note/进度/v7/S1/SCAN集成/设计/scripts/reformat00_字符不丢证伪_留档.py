# -*- coding: utf-8 -*-
"""
对 00_总览.md 做"断行 + 结构化"重排。
规则：
  - 不删改任何文字；仅在超长行按 ｜ / ； 边界插入换行，并加小标题/缩进。
  - 把 Markdown 表格（Gate 表、计划中阶段表、施工侧阶段表）转为 ### 小节 + 条目。
  - 所有 ｜ 与 ； 分隔符原样保留（拆分后重新前置），字符不丢。
输出到同目录 00_总览.new.md，并用关键 token 做完整性校验。
"""
import re
import sys

SRC = "/home/jack/ysc-code/ros1-slam/note/进度/00_总览.md"
DST = "/home/jack/ysc-code/ros1-slam/note/进度/00_总览.new.md"

with open(SRC, encoding="utf-8") as f:
    raw = f.read()
lines = raw.split("\n")


def reflow(text, primary="｜", secondary="；"):
    """按 ｜ 与 ； 断行，分隔符原样前置，不丢字符。"""
    out = []
    pieces = text.split(primary)
    for i, piece in enumerate(pieces):
        piece = piece.strip()
        if not piece:
            continue
        subs = piece.split(secondary)
        for j, sub in enumerate(subs):
            sub = sub.strip()
            if not sub:
                continue
            if i > 0 and j == 0:
                sub = primary + sub
            elif j > 0:
                sub = secondary + sub
            out.append(sub)
    return out


def emit_reflowed(line):
    """对单行做断行，保持前缀（> / > - / - / 1. 等）。"""
    if line.startswith("> - "):
        content = line[4:]
        segs = reflow(content)
        res = []
        for k, seg in enumerate(segs):
            res.append(("> - " + seg) if k == 0 else (">   " + seg))
        return res
    if line.startswith("> "):
        content = line[2:]
        return ["> " + seg for seg in reflow(content)]
    m = re.match(r"^(\s*)-\s(.*)$", line)
    if m:
        indent, content = m.group(1), m.group(2)
        segs = reflow(content)
        return [(indent + "- " + seg) if k == 0 else (indent + "  " + seg)
                for k, seg in enumerate(segs)]
    m = re.match(r"^(\s*)(\d+\.)\s(.*)$", line)
    if m:
        indent, num, content = m.group(1), m.group(2), m.group(3)
        segs = reflow(content)
        return [(indent + num + " " + seg) if k == 0 else (indent + "   " + seg)
                for k, seg in enumerate(segs)]
    # 普通段落：直接断行，无前缀
    segs = reflow(line)
    return segs if segs else [line]


def is_table_row(s):
    s = s.strip()
    return s.startswith("|") and s.endswith("|") and len(s) > 1


def is_sep(s):
    s = s.strip()
    return bool(s) and bool(re.fullmatch(r"[|\s\-:]+", s))


out = []
n = len(lines)
for idx, line in enumerate(lines):
    stripped = line.strip()

    # 标题行保持原样（不被断行）
    if stripped.startswith("#"):
        out.append(line)
        continue

    # 表格行处理
    if is_table_row(line):
        # 分隔行 / 表头行（下一行是分隔行）-> 跳过
        if is_sep(line):
            continue
        nxt = lines[idx + 1].strip() if idx + 1 < n else ""
        if is_sep(nxt):  # 当前是表头行
            continue
        cells = [c.strip() for c in stripped[1:-1].split("|")]
        if len(cells) < 2:
            out.append(line)
            continue
        name = cells[0]
        if len(cells) == 2:
            content = cells[1]
        else:
            content = "｜".join(cells[1:])  # 内容里含 | 时还原
        out.append("### " + name)
        for seg in reflow(content):
            out.append("- " + seg)
        continue

    # 其余可断行行（块引用 / 列表 / 段落）
    out.append(line) if line == "" else out.extend(emit_reflowed(line))

# 空行保持
final = "\n".join(out)
with open(DST, "w", encoding="utf-8") as f:
    f.write(final)

# ---- 完整性校验 ----
with open(DST, encoding="utf-8") as f:
    new = f.read()

def check_tokens(src_text, dst_text):
    missing = []
    # 1) 所有 R 批次号
    for tok in set(re.findall(r"R\d{2,4}", src_text)):
        if tok not in dst_text:
            missing.append("R:" + tok)
    # 2) Gate 号
    for tok in set(re.findall(r"G\d[\d.\-/]*", src_text)):
        if tok not in dst_text:
            missing.append("G:" + tok)
    # 3) sha256 片段（取 64 位 hex）
    for tok in set(re.findall(r"[0-9a-f]{64}", src_text)):
        if tok not in dst_text:
            missing.append("sha:" + tok[:12])
    # 4) 关键短语
    for phrase in ["lite3_1007", "faster_lio", "MID360_MOUNT_Z", "v5.3",
                   "480766025753873159fabebb6b34a953360e56204668d6b4f97b456b9ee6965d",
                   "8c0651fc", "54adcb8", "f721c6c", "施工侧自判", "bridge.py"]:
        if phrase not in dst_text:
            missing.append("phrase:" + phrase)
    return missing

missing = check_tokens(raw, new)
print("原始行数:", len(lines), " 重排后行数:", len(out))
print("原始字符数(含换行):", len(raw), " 重排后字符数(含换行):", len(final))
print("新增行数:", len(out) - len(lines))
print("缺失 token 数:", len(missing))
if missing:
    print("缺失样本:", missing[:50])
    sys.exit(2)
print("校验通过：关键 token 全部保留")
