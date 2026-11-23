#!/usr/bin/env python3
"""note/ 文档健康巡检（只读）。

回答四个「越维护越乱」的问题：
  1. 断链      —— 现行面的相对链接有没有指向不存在的文件；
  2. 孤儿      —— 有没有文档没有任何入口指向它（别人永远找不到）；
  3. 同名      —— 同一文件名出现在多个目录（看链接时极易点错）；
  4. 戳失真    —— 文内「最新更新时间」是否早于该件最后一次提交（自称比实际旧）；
  5. 空件      —— 0 字节的 Markdown。

不修改任何文件。退出码：0 无问题｜5 有问题（明细打印，最多 40 条）｜9 前置缺失。

用法：python3 scripts/ops/lint_notes.py [--all]
     默认只查「现行面」（note/ 根、索引/、编译/、架构/、进度/README 与 00_专题/、计划书v8.1/）；
     --all 连历史区（v4—v8 批次与证据、计划书 v3—v8）一起查。
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NOTE = ROOT / 'note'
CUR = ['note/00_README.md', 'note/进度/README.md']
CUR_DIRS = ['note/索引', 'note/编译', 'note/架构', 'note/进度/00_专题', 'note/计划书v8.1']
HIST_PREFIX = ('note/进度/v4/', 'note/进度/v5/', 'note/进度/v6/', 'note/进度/v7/', 'note/进度/v8/',
               'note/计划书v3/', 'note/计划书v4/', 'note/计划书v5/', 'note/计划书v6/', 'note/计划书v7/',
               'note/计划书v8/')
# 有入口即算「可被找到」的白名单（入口索引里会点到它们）
ENTRY = {'note/00_README.md', 'note/进度/README.md', 'note/README.md'}
# 三份 append-only 台账里有大量「括号形态」的历史文本（如 `[x](829|889)`），不是链接；
# 对它们不判链接，单独报数，避免把历史叙述当断链。
NO_LINK_SCAN = {'note/04_配置改动diff.md', 'note/02_问题与坑.md', 'note/05_复验作业单.md'}
LINK_RE = re.compile(r'\]\(([^)]+)\)')
STAMP_RE = re.compile(r'最新更新时间[：:]\s*(20\d{2}-\d{2}-\d{2})')


def tracked(pattern: str) -> list[str]:
    out = subprocess.run(['git', 'ls-files', '-z', pattern], cwd=ROOT,
                         capture_output=True, text=True).stdout
    return [f for f in out.split('\0') if f]


def in_current(path: str) -> bool:
    return path in CUR or any(path.startswith(d + '/') for d in CUR_DIRS)


def main() -> int:
    ap = argparse.ArgumentParser(description='note 文档健康巡检（只读）')
    ap.add_argument('--all', action='store_true', help='连历史区一起查')
    a = ap.parse_args()

    mds = [f for f in tracked('note') if f.endswith('.md')]
    if not mds:
        print('[fail] 未找到任何 md（前置缺失）', file=sys.stderr)
        return 9
    body = {}
    for f in mds:
        p = ROOT / f
        if p.exists():
            body[f] = p.read_text(errors='ignore')
    if not body:
        print('[fail] 所有 md 都读不到（前置缺失）', file=sys.stderr)
        return 9

    scope = [f for f in body if a.all or in_current(f)]

    # 1) 断链
    broken = []
    skipped = 0
    for f in scope:
        if f in NO_LINK_SCAN:
            skipped += 1
            continue
        base = Path(f).parent
        for m in LINK_RE.finditer(body[f]):
            t = m.group(1).split('#')[0].split('?')[0]
            if not t or t.startswith(('http', 'mailto:')):
                continue
            if not (ROOT / os.path.normpath(os.path.join(str(base), t))).exists():
                broken.append(f'{f} → {t}')

    # 2) 孤儿：没有任何 md 链接指向它（白名单与索引入口除外）
    referenced = set()
    for f, b in body.items():
        base = Path(f).parent
        for m in LINK_RE.finditer(b):
            t = m.group(1).split('#')[0].split('?')[0]
            if not t or t.startswith(('http', 'mailto:')):
                continue
            tgt = os.path.relpath(os.path.normpath(os.path.join(str(base), t)), str(ROOT))
            referenced.add(tgt.replace(os.sep, '/'))
    orphans = [f for f in scope
               if f not in ENTRY and f not in referenced
               and not f.startswith(('note/安全核查', 'note/索引/README'))]

    # 3) 同名：同一 basename 出现在 ≥2 个目录
    byname = defaultdict(list)
    for f in scope:
        byname[Path(f).name].append(f)
    dup = {k: v for k, v in byname.items() if len(v) > 1}

    # 4) 戳失真：文内日期 < 最后一次提交日期
    stale = []
    for f in scope:
        m = STAMP_RE.search(body[f])
        if not m:
            continue
        last = subprocess.run(['git', 'log', '-1', '--format=%ad', '--date=short', '--', f],
                              cwd=ROOT, capture_output=True, text=True).stdout.strip()
        if last and last > m.group(1):
            stale.append(f'{f} 文内 {m.group(1)} < 提交 {last}')

    # 5) 空件
    empty = [f for f in scope if (ROOT / f).exists() and (ROOT / f).stat().st_size == 0]

    print(f'[lint] 范围 {len(scope)} 份 md（{"全量" if a.all else "现行面"}）'
          + (f'；其中 {skipped} 份 append-only 台账不判链接' if skipped else ''))
    for name, items in (('断链', broken), ('孤儿', orphans), ('戳失真', stale), ('空件', empty)):
        print(f'  {name}: {len(items)}')
        for x in items[:8]:
            print(f'    {x}')
    print(f'  同名文件（跨目录）: {len(dup)} 组')
    for k, v in list(sorted(dup.items(), key=lambda kv: -len(kv[1])))[:5]:
        print(f'    {k} × {len(v)}  {v[0]} …')

    if a.all:
        cur_broken = [b for b in broken if in_current(b.split(' → ')[0])]
        print(f'  ★历史区读数仅作普查、不参与退出码：历史区断链 {len(broken) - len(cur_broken)} 条'
              f'（含计划书占位符与已删件，属历史事实）')
        return 5 if (cur_broken or empty) else 0
    if broken or empty:
        return 5
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
