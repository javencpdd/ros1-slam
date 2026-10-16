#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros1-slam 跨 Agent 协作黑板 CLI
================================

给 zcode / qoder（以及任何能敲命令的 Agent）用的共享状态机。

设计要点
--------
1. 单一真相源：.ai/state.json（机器读写）+ .ai/HANDOFF.md / TASKS.md（人/LLM 读）
   两者总是由本脚本同步生成，禁止手写，避免出现「两份账本」。
2. epoch 版本号：任何状态迁移都 +1。HANDOFF.md 里写明 epoch，
   Agent 读到的 epoch 落后于 state.json 时，check 会报「快照过期」。
3. 任务隔离：同一时刻只有一个 current 任务；每个任务带 scope（文件白名单）。
   resume 只吐 current 任务，从物理上减少串台。

用法
----
  python3 .ai/bridge.py new     "给 slam 节点加 IMU 预积分" --as zcode
  python3 .ai/bridge.py claim   T-20261016-01 --as zcode
  python3 .ai/bridge.py note    T-20261016-01 "发现 /imu topic 时间戳没对齐"
  python3 .ai/bridge.py handoff T-20261016-01 --to qoder --summary "已改完，待你跑 bag 回归"
  python3 .ai/bridge.py done    T-20261016-01
  python3 .ai/bridge.py resume  --as qoder
  python3 .ai/bridge.py status
  python3 .ai/bridge.py check
  python3 .ai/bridge.py knowledge "imu话题时间戳不对齐，odom 里要手动插值"

只依赖标准库，Python 3.7+ 可跑。
"""

import argparse
import fcntl
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone, timedelta

# 本项目 note/02_问题与坑.md 坑17 明确记录：
#   ① temp+rename 式写入会冲掉已补的 ACL；
#   ② ysc 落盘会让属主漂移成 ysc:ysc，导致 ros1-slam 落到 other::r-- 写不动。
# 因此本脚本一律「原地写 + 每次写完补 setfacl」，不可改成 os.replace。
# 需要补 ACL 的账号（本项目为 ros1-slam），可用环境变量覆盖。
ACL_USER = os.environ.get("AI_BRIDGE_ACL_USER", "ros1-slam")

# ---------------------------------------------------------------- 常量路径

try:
    ROOT = os.environ.get("AI_BRIDGE_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:  # pragma: no cover
    ROOT = os.getcwd()

AI_DIR     = os.path.join(ROOT, ".ai")
STATE      = os.path.join(AI_DIR, "state.json")
LOCK       = os.path.join(AI_DIR, ".state.lock")
TASKS_MD   = os.path.join(AI_DIR, "TASKS.md")
HANDOFF_MD = os.path.join(AI_DIR, "HANDOFF.md")
JOURNAL    = os.path.join(AI_DIR, "JOURNAL.md")
KNOWLEDGE  = os.path.join(AI_DIR, "knowledge.md")
TASKS_DIR  = os.path.join(AI_DIR, "tasks")
TEMPLATE   = os.path.join(TASKS_DIR, "_template.md")

CST = timezone(timedelta(hours=8))

VALID_AGENTS  = ("zcode", "qoder", "human", "unknown")
VALID_STATUS  = ("todo", "in_progress", "blocked", "review", "done", "abandoned")
OPEN_STATUS   = ("todo", "in_progress", "blocked", "review")


# ---------------------------------------------------------------- 基础工具

def now():
    return datetime.now(CST)


def ts():
    return now().strftime("%Y-%m-%d %H:%M:%S")


def die(msg, code=1):
    sys.stderr.write("ERROR: %s\n" % msg)
    sys.exit(code)


def _setfacl(path, default=False):
    """给单个路径补 ACL，让 ros1-slam 账号始终可写（坑17 要求：每次编辑后都要补）。"""
    if not ACL_USER or not shutil.which("setfacl"):
        return
    args = ["setfacl"]
    if default:
        args.append("-d")
    args += ["-m", "u:%s:rwx" % ACL_USER, path]
    try:
        subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    except Exception:
        pass


def write_text(path, content):
    """统一写入口：原地写 + 写完补 ACL。所有落盘都必须走这里。"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    _setfacl(path)


def append_text(path, text):
    with open(path, "a", encoding="utf-8") as f:
        f.write(text)
    _setfacl(path)


def _ensure_dirs():
    created = []
    for d in (AI_DIR, TASKS_DIR):
        if not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
            created.append(d)
    for d in created:
        _setfacl(d)
        _setfacl(d, default=True)


class Lock(object):
    """跨进程文件锁，防止两个 Agent 同时改状态。"""

    def __enter__(self):
        _ensure_dirs()
        self.f = open(LOCK, "a+")
        try:
            fcntl.flock(self.f.fileno(), fcntl.LOCK_EX)
        except OSError:
            pass
        return self

    def __exit__(self, *a):
        try:
            fcntl.flock(self.f.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        self.f.close()
        return False


def load_state():
    if not os.path.exists(STATE):
        die("找不到 %s —— 请先运行 python3 .ai/bridge.py init" % STATE)
    with open(STATE, "r", encoding="utf-8") as f:
        return json.load(f)


def save_state(st):
    """原地写，避免 temp+rename 冲掉 ACL（坑17）。写完立刻补 ACL。"""
    _ensure_dirs()
    with open(STATE, "w", encoding="utf-8") as f:
        json.dump(st, f, ensure_ascii=False, indent=2)
    _setfacl(STATE)


def bump(st):
    st["epoch"] = int(st.get("epoch", 0)) + 1
    return st["epoch"]


def journal(line):
    if not os.path.exists(JOURNAL):
        return
    append_text(JOURNAL, "- %s | %s\n" % (ts(), line))


# ---------------------------------------------------------------- 渲染

STATUS_ICON = {
    "todo":        " ",
    "in_progress": "▶",
    "blocked":     "⛔",
    "review":      "👀",
    "done":        "✔",
    "abandoned":   "✘",
}


def render(st):
    """由 state.json 生成 TASKS.md 和 HANDOFF.md，保证两者永不分叉。"""
    tasks = st["tasks"]

    # ---- TASKS.md
    lines = [
        "# 任务账本 · ros1-slam",
        "",
        "> **本文件由 bridge.py 自动生成，请勿手改。**",
        "> 改状态请跑：`python3 .ai/bridge.py claim|note|handoff|done <ID>`",
        "> 当前 epoch：`%d`（每次状态迁移 +1，读到比这小的说明快照过期）" % st["epoch"],
        "",
        "| | ID | 状态 | 负责人 | 标题 | 更新于 |",
        "|---|---|---|---|---|---|",
    ]
    order = sorted(tasks.values(), key=lambda t: (t.get("created_at", ""), t["id"]), reverse=True)
    for t in order:
        mark = "*" if t["id"] == st.get("current_task") else " "
        lines.append("| %s | `%s` | %s %s | %s | %s | %s |" % (
            mark, t["id"], STATUS_ICON.get(t["status"], "?"), t["status"],
            t.get("owner") or "-", t["title"], t.get("updated_at", ""),
        ))
    lines += [
        "",
        "图例：`*` = 当前任务（current），同一时刻只会有一个。",
        "",
        "## 未闭合任务明细",
        "",
    ]
    opened = [t for t in order if t["status"] in OPEN_STATUS]
    if not opened:
        lines.append("_（无）_")
    for t in opened:
        lines.append("### `%s` %s" % (t["id"], t["title"]))
        lines.append("- 状态：`%s`　负责人：`%s`　创建：`%s`" % (
            t["status"], t.get("owner") or "-", t.get("created_at", "")))
        if t.get("scope"):
            lines.append("- 作用域：`%s`" % ", ".join("`%s`" % s for s in t["scope"]))
        if t.get("blocked_by"):
            lines.append("- 阻塞于：%s" % ", ".join("`%s`" % s for s in t["blocked_by"]))
        lines.append("- 详情：[`.ai/tasks/%s.md`](tasks/%s.md)" % (t["id"], t["id"]))
        lines.append("")

    write_text(TASKS_MD, "\n".join(lines).rstrip() + "\n")

    # ---- HANDOFF.md
    cur = tasks.get(st.get("current_task") or "")
    h = [
        "# 交接卡 · HANDOFF",
        "",
        "> **本文件由 bridge.py 自动生成，请勿手改。它是「当前进行到哪」的唯一游标。**",
        "",
        "**epoch: `%d`**　最后更新：%s" % (st["epoch"], ts()),
        "",
    ]
    if not cur:
        h += [
            "## 当前没有进行中的任务",
            "",
            "新开一个：",
            "",
            "```bash",
            "python3 .ai/bridge.py new \"任务标题\" --as zcode",
            "```",
            "",
            "或接手一个：`python3 .ai/bridge.py claim <ID> --as zcode`",
        ]
    else:
        h += [
            "## ▶ 当前任务：`%s` %s" % (cur["id"], cur["title"]),
            "",
            "| 字段 | 值 |",
            "|---|---|",
            "| 责任 Agent | `%s` |" % (cur.get("owner") or "-"),
            "| 状态 | `%s` |" % cur["status"],
            "| 创建 | %s |" % cur.get("created_at", ""),
            "| 最后更新 | %s |" % cur.get("updated_at", ""),
            "| 任务卡 | [`.ai/tasks/%s.md`](tasks/%s.md) |" % (cur["id"], cur["id"]),
            "",
        ]
        if cur.get("scope"):
            h += ["**作用域（本次只允许碰这些路径）**：", ""]
            h += ["- `%s`" % s for s in cur["scope"]]
            h += [""]
        if cur.get("latest_handoff"):
            h += ["## 上一棒留下的话", "", "> %s" % cur["latest_handoff"].replace("\n", "\n> "), ""]
        h += [
            "## 接手后先做这三件事",
            "",
            "1. 读全卡：`cat %s/.ai/tasks/%s.md`" % (ROOT, cur["id"]),
            "2. 核 epoch：上面写的 `%d` 必须等于 `python3 .ai/bridge.py check` 报的当前 epoch" % st["epoch"],
            "3. 认领：`python3 .ai/bridge.py claim %s --as <你>`" % cur["id"],
            "",
            "## 收工格式（必须执行，不能直接停）",
            "",
            "```bash",
            "python3 .ai/bridge.py note    %s \"本次做了什么 / 关键发现\"" % cur["id"],
            "python3 .ai/bridge.py handoff %s --to qoder --summary \"下一步是什么、坑在哪、别碰什么\"" % cur["id"],
            "```",
        ]
    write_text(HANDOFF_MD, "\n".join(h).rstrip() + "\n")


def commit(st, note_line):
    st["updated_at"] = ts()
    save_state(st)
    render(st)
    if note_line:
        journal(note_line)


# ---------------------------------------------------------------- 命令

def cmd_init(args):
    if os.path.exists(STATE) and not args.force:
        die("state.json 已存在，加 --force 覆盖")
    _ensure_dirs()
    st = {
        "project": os.path.basename(ROOT) or "ros1-slam",
        "root": ROOT,
        "version": 1,
        "epoch": 1,
        "created_at": ts(),
        "updated_at": ts(),
        "current_task": None,
        "counters": {},
        "tasks": {},
    }
    save_state(st)
    if not os.path.exists(TEMPLATE):
        write_template(st)
    render(st)
    print("初始化完成：%s" % STATE)
    print("  epoch=%d  root=%s" % (st["epoch"], ROOT))


def write_template(st):
    if os.path.exists(TEMPLATE):
        return
    content = """# 任务卡模板

> 复制本文件创建任务卡，或直接交给 bridge.py 生成。

## 元信息

- ID：`T-YYYYMMDD-NN`
- 负责人：`<zcode|qoder|human>`
- 状态：`todo`
- 创建：
- 作用域（本次只允许改动的文件/目录）：
  - `src/xxx/`

## 目标（做什么）

<一句话说清要交付什么>

## 非目标（不做什么）

- 不顺手重构无关模块
- 不改不在上面作用域里的文件

## 已尝试 / 已知结论

- （填，防止下一棒重复踩坑）

## 阻塞 & 风险

- （无则写「无」）

## 验收标准

- [ ] <可执行的验证命令>

## 工作笔记（按时间追加，不要覆盖）

### <时间戳> · <agent>

<做了什么 / 发现什么>

## 交接记录

### <时间戳> · 交给 <agent>

摘要：<下一步是什么、坑在哪、别碰什么>
"""
    write_text(TEMPLATE, content)


def next_id(st):
    day = now().strftime("%Y%m%d")
    key = "T-%s" % day
    counters = st.setdefault("counters", {})
    n = counters.get(key, 0) + 1
    counters[key] = n
    return "%s-%02d" % (key, n)


def cmd_new(args):
    with Lock():
        st = load_state()
        tid = next_id(st)
        form_date = now().strftime("%Y-%m-%d")
        st["tasks"][tid] = {
            "id": tid,
            "title": args.title,
            "status": "todo",
            "owner": args.as_agent or None,
            "created_at": ts(),
            "updated_at": ts(),
            "created_date": form_date,
            "scope": args.scope or [],
            "blocked_by": [],
            "file": "tasks/%s.md" % tid,
            "latest_handoff": "",
        }
        # 第一个任务自动成为 current
        if not st.get("current_task"):
            st["current_task"] = tid
        bump(st)

        write_template(st)
        tf = os.path.join(TASKS_DIR, "%s.md" % tid)
        write_text(tf, render_task_card(st["tasks"][tid]))

        commit(st, "新建任务 `%s`「%s」 by %s" % (tid, args.title, args.as_agent or "-"))
    print(tid)


def render_task_card(t):
    scoped = "\n".join("  - `%s`" % s for s in t.get("scope", [])) or "  - （未限定，开工前请补上）"
    return """# `%s` %s

## 元信息

- ID：`%s`
- 负责人：`%s`
- 状态：`%s`
- 创建：%s
- 最后更新：%s
- 作用域（本次只允许改动的文件/目录）：
%s

## 目标

<一句话说清要交付什么 —— 请第一时间补上>

## 非目标

- 不顺手重构无关模块

## 已尝试 / 已知结论

-

## 阻塞 & 风险

- 无

## 验收标准

- [ ]

## 工作笔记（按时间追加）

## 交接记录
""" % (t["id"], t["title"], t["id"], t.get("owner") or "-", t["status"],
       t.get("created_at", ""), t.get("updated_at", ""), scoped)


def get_task(st, tid):
    t = st["tasks"].get(tid)
    if not t:
        die("没有任务 `%s`。现有任务：%s" % (tid, ", ".join(sorted(st["tasks"])) or "(空)"))
    return t


SECTION_NOTES   = "## 工作笔记"
SECTION_HANDOFF = "## 交接记录"


def append_to_card(tid, section_heading, heading, body):
    """把一段内容插到任务卡指定的二级小节里（该小节末尾、下个 ## 之前）。

    这样「工作笔记」和「交接记录」永远各归各位，不会像简单 append 那样全堆到文末。
    """
    tf = os.path.join(TASKS_DIR, "%s.md" % tid)
    if not os.path.exists(tf):
        return
    with open(tf, "r", encoding="utf-8") as f:
        content = f.read()
    block = "\n### %s\n\n%s\n" % (heading, body)
    i = content.find(section_heading)
    if i < 0:
        content = content.rstrip() + "\n" + block
    else:
        j = content.find("\n## ", i + len(section_heading))
        if j < 0:
            content = content.rstrip() + "\n" + block
        else:
            content = content[:j].rstrip() + "\n" + block + content[j:]
    write_text(tf, content)


def cmd_claim(args):
    with Lock():
        st = load_state()
        t = get_task(st, args.id)
        other_open = [x["id"] for x in st["tasks"].values()
                      if x["status"] == "in_progress" and x["id"] != args.id]
        if other_open:
            print("警告：还有任务处于 in_progress：%s —— 请先 handoff 或 done，别同时开两条战线。"
                  % ", ".join(other_open))
        t["owner"] = args.as_agent
        t["status"] = "in_progress"
        t["updated_at"] = ts()
        st["current_task"] = args.id
        bump(st)
        append_to_card(args.id, SECTION_NOTES, "%s · %s 认领" % (ts(), args.as_agent),
                       "开始接手本任务。\n\n%s" % (args.summary or "（无备注）"))
        commit(st, "`%s` 被 %s 认领" % (args.id, args.as_agent))
    print("claimed %s -> %s" % (args.id, args.as_agent))


def cmd_note(args):
    with Lock():
        st = load_state()
        t = get_task(st, args.id)
        t["updated_at"] = ts()
        bump(st)
        append_to_card(args.id, SECTION_NOTES, "%s · %s" % (ts(), args.as_agent or "unknown"),
                       args.text)
        commit(st, "`%s` 笔记 by %s" % (args.id, args.as_agent or "unknown"))
    print("noted")


def cmd_handoff(args):
    with Lock():
        st = load_state()
        t = get_task(st, args.id)
        summary = args.summary or "（未填写摘要 —— 请回去补）"
        t["status"] = args.status if args.status else "blocked"
        t["owner"] = args.to
        t["updated_at"] = ts()
        t["latest_handoff"] = summary
        st["current_task"] = args.id
        bump(st)
        append_to_card(args.id, SECTION_HANDOFF,
                       "%s · %s 交给 %s" % (ts(), args.as_agent or "unknown", args.to),
                       "摘要：%s\n\n接续状态：`%s`　现任：`%s`"
                       % (summary, t["status"], args.to))
        commit(st, "`%s` 由 %s 交接给 %s" % (args.id, args.as_agent or "unknown", args.to))
    print("handed off %s -> %s (epoch=%d)" % (args.id, args.to, load_state()["epoch"]))


def cmd_done(args):
    with Lock():
        st = load_state()
        t = get_task(st, args.id)
        t["status"] = "done"
        t["owner"] = "human"
        t["updated_at"] = ts()
        t["latest_handoff"] = "已完成。%s" % (args.summary or "")
        bump(st)
        if st.get("current_task") == args.id:
            # 自动顶替一个还没做完的
            rest = sorted([x["id"] for x in st["tasks"].values() if x["status"] in OPEN_STATUS])
            st["current_task"] = rest[0] if rest else None
        append_to_card(args.id, SECTION_HANDOFF, "%s · 完成" % ts(),
                       args.summary or "标记完成。")
        commit(st, "`%s` 完成" % args.id)
    print("done %s" % args.id)


def cmd_status(args):
    st = load_state()
    print("epoch=%s  current=%s" % (st["epoch"], st.get("current_task") or "-"))
    print("%-16s %-12s %-8s %s" % ("ID", "STATUS", "OWNER", "TITLE"))
    print("-" * 78)
    for t in sorted(st["tasks"].values(), key=lambda x: x["id"]):
        mark = "*" if t["id"] == st.get("current_task") else " "
        print("%s%-15s %-12s %-8s %s" % (mark, t["id"], t["status"], t.get("owner") or "-", t["title"]))


def cmd_check(args):
    st = load_state()
    problems, warnings = [], []
    cur = st.get("current_task")
    if cur and cur not in st["tasks"]:
        problems.append("current_task `%s` 不在任务表里" % cur)
    inprog = [t for t in st["tasks"].values() if t["status"] == "in_progress"]
    if len(inprog) > 1:
        warnings.append("同时有多个 in_progress：%s —— 大概率串台了" % ", ".join(t["id"] for t in inprog))
    if inprog and cur not in [t["id"] for t in inprog]:
        warnings.append("current 任务 `%s` 并不是 in_progress 状态" % cur)
    for t in st["tasks"].values():
        fp = os.path.join(AI_DIR, t.get("file", ""))
        if not os.path.exists(fp):
            problems.append("`%s` 的任务卡缺失：%s" % (t["id"], fp))
        if t["status"] in OPEN_STATUS and not t.get("scope"):
            warnings.append("`%s` 没有限定 scope，容易误伤其他模块" % t["id"])
        if t["status"] == "in_progress" and args.stale_days >= 0:
            try:
                d = datetime.strptime(t["updated_at"], "%Y-%m-%d %H:%M:%S")
                if (now().replace(tzinfo=None) - d).days > args.stale_days:
                    warnings.append("`%s` 已 %d 天没更新，可能已经烂尾或被遗忘" % (
                        t["id"], (now().replace(tzinfo=None) - d).days))
            except ValueError:
                pass
    if problems:
        print("✘ 硬错误：")
        for p in problems:
            print("   -", p)
    if warnings:
        print("⚠ 提示：")
        for w in warnings:
            print("   -", w)
    if not problems and not warnings:
        print("✔ 一致，epoch=%d" % st["epoch"])
    return 1 if problems else 0


def cmd_resume(args):
    """换 Agent 后第一件事：把 resume 的完整输出贴给新 Agent。"""
    st = load_state()
    cur = st["tasks"].get(st.get("current_task") or "")
    who = args.as_agent or "unknown"
    out = []
    out.append("=" * 72)
    out.append("RESUME PACKET · epoch=%d · 生成于 %s · 请求方=%s" % (st["epoch"], ts(), who))
    out.append("=" * 72)
    if not cur:
        out.append("当前无进行中任务。用 `python3 .ai/bridge.py new \"标题\" --as %s` 开一个。" % who)
        print("\n".join(out))
        return
    out.append("▶ 当前任务：%s  %s" % (cur["id"], cur["title"]))
    out.append("   状态=%s  现任=%s  最后更新=%s" % (cur["status"], cur.get("owner") or "-", cur.get("updated_at", "")))
    if cur.get("owner") and cur["owner"] not in (who, "human", "unknown"):
        out.append("   ⚠ 注意：本任务当前归属 `%s`，你是 `%s`。接手请先 claim 抢占。" % (cur["owner"], who))
    out.append("")
    out.append("强制阅读顺序：")
    out.append("  1) cat %s/AGENTS.md" % ROOT)
    out.append("  2) cat %s/.ai/HANDOFF.md" % ROOT)
    out.append("  3) cat %s/.ai/tasks/%s.md   ← 最重要，含全部历史结论" % (ROOT, cur["id"]))
    out.append("  4) tail -30 %s/.ai/JOURNAL.md" % ROOT)
    out.append("")
    if cur.get("scope"):
        out.append("本次作用域（之外的文件不要动）：")
        out += ["  - %s" % s for s in cur["scope"]]
    else:
        out.append("本次作用域：未限定 —— 开工前先跟用户确认再改代码。")
    out.append("")
    if cur.get("latest_handoff"):
        out.append("上一棒交接摘要：")
        out.append("  > %s" % cur["latest_handoff"].replace("\n", "\n  > "))
        out.append("")
    # 任务卡里的交接记录与笔记尾巴
    tf = os.path.join(TASKS_DIR, "%s.md" % cur["id"])
    if os.path.exists(tf):
        with open(tf, "r", encoding="utf-8") as f:
            body = f.read()
        idx = body.find("## 工作笔记")
        if idx > 0:
            tail = body[idx:].strip().splitlines()
            if len(tail) > 40:
                tail = tail[-40:]
            out.append("任务卡最近记录：")
            out += ["  " + l for l in tail]
    if os.path.exists(KNOWLEDGE):
        with open(KNOWLEDGE, "r", encoding="utf-8") as f:
            kn = [l for l in f.read().splitlines()
                  if l.strip() and not l.lstrip().startswith(("#", ">", "- [__DEPRECATED"))]
        if kn:
            out.append("")
            out.append("长期知识库最后 8 条（详见 .ai/knowledge.md）：")
            out += ["  " + l for l in kn[-8:]]
    print("\n".join(out))


def cmd_knowledge(args):
    """手动沉淀跨工具共享的长期事实 —— 因为两边的 Auto-Memory 互不通用。"""
    if not os.path.exists(KNOWLEDGE):
        write_text(KNOWLEDGE,
                   "# 长期知识库（跨工具共享，请勿删历史条目）\n\n"
                   "> zcode 的 Project Memory 与 qoder 的 Auto-Memory 都只存在各自本机、\n"
                   "> 且互不可导出。**想让另一个工具也知道，就必须手写到这里。**\n\n"
                   "> 注意区分：本项目既有的 `note/02_问题与坑.md` 走「坑N 登记制」，\n"
                   "> 面向四人分角色 + 计划书同步；本文件只收**工具协作本身**的经验，\n"
                   "> 两者不要混。\n\n")
    append_text(KNOWLEDGE, "- [%s] %s\n" % (ts(), args.text))
    print("已写入 .ai/knowledge.md")


def cmd_mcp(args):
    """打印给 Agent 用的极简状态 JSON（方便脚本化解析）。"""
    st = load_state()
    cur = st["tasks"].get(st.get("current_task") or "")
    print(json.dumps({
        "epoch": st["epoch"],
        "current": st.get("current_task"),
        "task": cur,
        "open": [t["id"] for t in st["tasks"].values() if t["status"] in OPEN_STATUS],
    }, ensure_ascii=False, indent=2))


# ---------------------------------------------------------------- CLI

def main():
    p = argparse.ArgumentParser(prog="bridge.py", description="ros1-slam 跨 Agent 黑板")
    p.add_argument("--root", default=None, help="项目根目录，默认脚本上级目录")
    sub = p.add_subparsers(dest="cmd")

    def add(name, *aliases):
        return sub.add_parser(name, aliases=aliases)

    sp = add("init")
    sp.add_argument("--force", action="store_true")
    sp.set_defaults(func=cmd_init)

    sp = add("new")
    sp.add_argument("title")
    sp.add_argument("--as", dest="as_agent", default=os.environ.get("AI_BRIDGE_AGENT", "human"),
                    choices=list(VALID_AGENTS))
    sp.add_argument("--scope", nargs="*", default=[])
    sp.set_defaults(func=cmd_new)

    sp = add("claim")
    sp.add_argument("id")
    sp.add_argument("--as", dest="as_agent", default=os.environ.get("AI_BRIDGE_AGENT", "unknown"),
                    choices=list(VALID_AGENTS))
    sp.add_argument("--summary", default="")
    sp.set_defaults(func=cmd_claim)

    sp = add("note")
    sp.add_argument("id")
    sp.add_argument("text")
    sp.add_argument("--as", dest="as_agent", default=os.environ.get("AI_BRIDGE_AGENT", "unknown"),
                    choices=list(VALID_AGENTS))
    sp.set_defaults(func=cmd_note)

    sp = add("handoff")
    sp.add_argument("id")
    sp.add_argument("--to", required=True, choices=list(VALID_AGENTS))
    sp.add_argument("--summary", required=True)
    sp.add_argument("--status", default="blocked", choices=list(VALID_STATUS))
    sp.add_argument("--as", dest="as_agent", default=os.environ.get("AI_BRIDGE_AGENT", "unknown"),
                    choices=list(VALID_AGENTS))
    sp.set_defaults(func=cmd_handoff)

    sp = add("done")
    sp.add_argument("id")
    sp.add_argument("--summary", default="")
    sp.set_defaults(func=cmd_done)

    sp = add("status", "ls")
    sp.set_defaults(func=cmd_status)

    sp = add("check")
    sp.add_argument("--stale-days", type=int, default=3)
    sp.set_defaults(func=cmd_check)

    sp = add("resume")
    sp.add_argument("--as", dest="as_agent", default=os.environ.get("AI_BRIDGE_AGENT", "unknown"),
                    choices=list(VALID_AGENTS))
    sp.set_defaults(func=cmd_resume)

    sp = add("knowledge", "mem")
    sp.add_argument("text")
    sp.set_defaults(func=cmd_knowledge)

    sp = add("json")
    sp.set_defaults(func=cmd_mcp)

    args = p.parse_args()
    if args.root:
        globals()["ROOT"] = os.path.abspath(args.root)
        globals()["AI_DIR"] = os.path.join(ROOT, ".ai")
        globals()["STATE"] = os.path.join(AI_DIR, "state.json")
        globals()["LOCK"] = os.path.join(AI_DIR, ".state.lock")
        globals()["TASKS_MD"] = os.path.join(AI_DIR, "TASKS.md")
        globals()["HANDOFF_MD"] = os.path.join(AI_DIR, "HANDOFF.md")
        globals()["JOURNAL"] = os.path.join(AI_DIR, "JOURNAL.md")
        globals()["KNOWLEDGE"] = os.path.join(AI_DIR, "knowledge.md")
        globals()["TASKS_DIR"] = os.path.join(AI_DIR, "tasks")
        globals()["TEMPLATE"] = os.path.join(TASKS_DIR, "_template.md")

    if not getattr(args, "func", None):
        p.print_help()
        return 1
    return args.func(args) or 0


if __name__ == "__main__":
    sys.exit(main())
