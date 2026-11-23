#!/usr/bin/env python3
"""把 note/进度/v8 的批次正文（R*.md）编译成可导航的日期页与模块页。

设计原则：
- **零幻觉**：只搬运原文里已存在的字段（标题、性质＝、依据：、读数首条），不生成新结论。
- **可重建**：输出全量覆盖，重跑即刷新；生成物页头标注「勿手改」。
- **不取代权威**：每行都链接回 R 正文，冲突时以正文为准。

用法（仓库根）：python3 scripts/ops/compile_notes.py [--check]
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / 'note/进度/v8'
OUT = ROOT / 'note/编译'

MODULES = {
    'bt-编排': r'BT|行为树|编排|goal_gate|取消|回执|mission|取消链',
    'planning-规划': r'规划|路径|planner|A\*|轨迹|局部|全局',
    'localization-定位': r'定位|NDT|hdl|LIO|里程计|失锁|重定位',
    'perception-感知地图': r'感知|地图|点云|支撑图|高程|语义|栅格|occupancy',
    'safety-安全限速': r'安全|限速|仲裁|mux|hold|急停|超声波',
    'sim-仿真载体': r'仿真|Gazebo|MuJoCo|载体|场景|world',
    'platform-平台契约': r'契约|schema|manifest|graph|装配|组件|registry|端口',
    'build-构建容器': r'构建|容器|docker|镜像|编译|依赖',
}

def parse(p: Path) -> dict:
    body = p.read_text(errors='ignore')
    lines = body.splitlines()
    title = next((l[2:].strip() for l in lines if l.startswith('# ')), p.stem)
    nature = ''
    basis = ''
    reading = ''
    for l in lines[:20]:
        m = re.search(r'性质＝([^；。\n]+)', l)
        if m and not nature:
            nature = m.group(1).strip()
        if l.startswith('- 依据：') and not basis:
            basis = l[len('- 依据：'):].strip()[:80]
    m = re.search(r'^## [一二三]、读数\s*\n+([^\n]+)', body, re.M)
    if m:
        reading = m.group(1).lstrip('- ').strip()[:90]
    rel = p.relative_to(ROOT).as_posix()
    return {'id': p.stem, 'path': rel, 'title': title, 'nature': nature,
            'basis': basis, 'reading': reading, 'date': p.parent.name,
            'ms': p.parent.parent.name}

def one_liner(r: dict) -> str:
    tail = re.split(r'[：:]', r['title'], maxsplit=1)
    subtitle = tail[1].strip() if len(tail) > 1 else ''
    for v in (r['nature'], subtitle, r['reading'], r['basis']):
        if v:
            return v.replace('|', '／')[:80]
    return '（原文只说标题，见正文）'

def modules_of(r: dict) -> list[str]:
    t = r['title'] + ' ' + r['nature']
    hit = [k for k, pat in MODULES.items() if re.search(pat, t, re.I)]
    return hit or ['milestone-里程碑与交付']

def head(desc: str) -> list[str]:
    return [f'# {desc}', '',
            '> 本页由 `scripts/ops/compile_notes.py` 自动生成，**勿手改**；重跑即刷新。',
            '> 它是**导航层**，不是权威：任何结论以链接到的 R 正文与 `note/` 台账为准。',
            '']

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='只统计不写盘')
    a = ap.parse_args()

    docs = [parse(p) for p in sorted(SRC.glob('M*/*/R*.md'))]
    if not docs:
        print('未找到 R 正文', file=sys.stderr)
        return 2
    by_date: dict[str, list[dict]] = {}
    by_mod: dict[str, list[dict]] = {}
    for d in docs:
        by_date.setdefault(d['date'], []).append(d)
        for m in modules_of(d):
            by_mod.setdefault(m, []).append(d)

    print(f'R 正文 {len(docs)} 份；日期页 {len(by_date)} 个；模块页 {len(by_mod)} 个')
    for k, v in sorted(by_mod.items(), key=lambda kv: -len(kv[1])):
        print(f'  {k:22} {len(v):4d}')
    if a.check:
        return 0

    (OUT / '日期').mkdir(parents=True, exist_ok=True)
    (OUT / '模块').mkdir(parents=True, exist_ok=True)
    # 全量重建：先清旧页，避免改名/删批后留下指向已不存在路径的陈旧页
    for old in list((OUT / '日期').glob('*.md')) + list((OUT / '模块').glob('*.md')) + [OUT / '索引.md']:
        if old.exists():
            old.unlink()

    for date, rows in sorted(by_date.items()):
        lines = head(f'{date} 当日批次（{len(rows)} 份）')
        lines += ['## 这一天做了什么', '',
                  '| 批次 | 一句话 | 里程碑 |', '| --- | --- | --- |']
        for r in sorted(rows, key=lambda x: int(re.sub(r'\D', '', x['id']) or 0)):
            rel = os.path.relpath(ROOT / r['path'], OUT / '日期').replace(os.sep, '/')
            lines.append(f"| [{r['id']}]({rel}) | {one_liner(r)} | {r['ms']} |")
        lines += ['', '## 原始文件', '']
        lines += [f"- [[{r['id']}]] — `{r['path']}`" for r in sorted(rows, key=lambda x: x['id'])]
        lines.append('')
        (OUT / '日期' / f'{date}.md').write_text('\n'.join(lines))

    for mod, rows in sorted(by_mod.items()):
        lines = head(f'{mod} · 进度时间线（{len(rows)} 份）')
        lines += ['## 按时间', '', '| 日期 | 批次 | 一句话 |', '| --- | --- | --- |']
        for r in sorted(rows, key=lambda x: (x['date'], int(re.sub(r'\D', '', x['id']) or 0))):
            rel = os.path.relpath(ROOT / r['path'], OUT / '模块').replace(os.sep, '/')
            lines.append(f"| {r['date']} | [{r['id']}]({rel}) | {one_liner(r)} |")
        lines.append('')
        (OUT / '模块' / f'{mod}.md').write_text('\n'.join(lines))

    idx = head('note 编译层索引')
    idx += [f'> 源：`note/进度/v8/M*/*/R*.md` 共 **{len(docs)}** 份；本层 {len(by_date) + len(by_mod)} 页。', '',
            '## 按日期（想知道某天做了什么）', '']
    idx += [f"- [{d}](日期/{d}.md) — {len(v)} 份批次" for d, v in sorted(by_date.items())]
    idx += ['', '## 按模块（想知道某块进度）', '']
    idx += [f"- [{m}](模块/{m}.md) — {len(v)} 份批次" for m, v in sorted(by_mod.items(), key=lambda kv: -len(kv[1]))]
    idx += ['', '## 怎么用', '',
            '1. 先按问题选维度：**查某天**看日期页；**查某项能力**看模块页。',
            '2. 页里每行都链回 R 正文；要看读数、边界、证据，点进去。',
            '3. 冲突时以 R 正文与 `note/04`／`05` 台账为准，本层不具权威。',
            '4. 批次有增删后重跑 `python3 scripts/ops/compile_notes.py`。']
    (OUT / '索引.md').write_text('\n'.join(idx) + '\n')
    print(f'已写 {len(by_date)} 个日期页、{len(by_mod)} 个模块页、1 个索引 -> {OUT.relative_to(ROOT)}')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
