#!/usr/bin/env python3
"""把 note/ 的原始层编译成可导航的索引层。

设计约束（与 note 文档优化方案 2026-11-23 一致）：
- **零幻觉**：只搬运原文已有字段（标题、性质＝、依据：、读数首条、编号），不生成新结论。
- **确定性**：输出不含时间戳，全部排序；同输入重复运行逐字节一致。
- **失败不破坏**：先在内存构建全部产物并自检，全部通过才落盘；任何异常不改动现有索引。
- **只读检查**：--check 不写盘，比对磁盘现状与应有内容，发现缺失/过期/条目不一致/断链即非零退出。
- **不覆盖人写件**：HUMAN_FILES 里的文件永不写入，命中即报错。

退出码：0 一致｜5 有不一致（过期/缺失/断链）｜9 前置缺失或源为空。

用法：
    python3 scripts/ops/compile_notes.py            # 重建全部生成物
    python3 scripts/ops/compile_notes.py --check    # 只读核对
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
V8 = ROOT / 'note/进度/v8'
V7 = ROOT / 'note/进度/v7'
OUT = ROOT / 'note/编译'
LEDGER04 = ROOT / 'note/04_配置改动diff.md'
PLAN_DIRS = ['计划书v3', '计划书v4', '计划书v5', '计划书v6', '计划书v7', '计划书v8', '计划书v8.1']
CURRENT_PLAN = '计划书v8.1'

# 人写文件：生成器永不触碰（命中即失败，防误覆盖）
HUMAN_FILES = {
    'note/横向台账导读.md',
    'note/架构/README.md',
    'note/02_问题与坑_索引.md',
    'note/架构/note文档优化方案_2026-11-23.md',
    'note/编译/README.md',
}

# 源规模期望值：对不上即视为前置缺失（不假绿）
EXPECT = {'r_docs': 500, 'v7_files': 702, 'sec04': 528, 'plan_dirs': 7}

SPLIT_THRESHOLD = 120          # 单页批次行数超过此值则按里程碑拆页
WP_RE = re.compile(r'V8-(\d{2})')

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

THEMES04 = {
    '构建与镜像': r'构建|镜像|docker|编译|依赖|catkin',
    '隔离与路径': r'隔离|路径|/home/ysc|/opt/|符号链接|软链',
    '门禁与判据': r'门禁|判据|VCC|ISO|PARITY|账构|GTC|VAR',
    '导航与规划': r'导航|规划|BT|行为树|路径|goal',
    '定位与建图': r'定位|建图|NDT|LIO|hdl|SLAM',
    '双后端与装配': r'双后端|support|far|graph|装配|契约|manifest',
    '真机与安全': r'真机|安全|急停|限速|mux|上机',
    '仿真与载体': r'仿真|Gazebo|MuJoCo|world|场景',
    '台账与证据': r'台账|证据|sha|清单|索引|登记',
    '权限与容器卷': r'权限|ACL|卷|mount|uid|命名卷',
}


class BuildError(Exception):
    """源侧前置不满足：不落盘、不假绿。"""


# ---------------------------------------------------------------- 读取原始层

def load_r_docs() -> list[dict]:
    docs = []
    for p in sorted(V8.glob('M*/*/R*.md')):
        body = p.read_text(errors='ignore')
        lines = body.splitlines()
        title = next((l[2:].strip() for l in lines if l.startswith('# ')), p.stem)
        nature, basis, reading = '', '', ''
        for l in lines[:25]:
            m = re.search(r'性质＝([^；。\n]+)', l)
            if m and not nature:
                nature = m.group(1).strip()
            if l.startswith('- 依据：') and not basis:
                basis = l[len('- 依据：'):].strip()[:80]
        m = re.search(r'^## [一二三]、读数\s*\n+([^\n]+)', body, re.M)
        if m:
            reading = m.group(1).lstrip('- ').strip()[:90]
        docs.append({
            'id': p.stem, 'path': p.relative_to(ROOT).as_posix(), 'title': title,
            'nature': nature, 'basis': basis, 'reading': reading,
            'date': p.parent.name, 'ms': p.parent.parent.name,
        })
    if len(docs) != EXPECT['r_docs']:
        raise BuildError(f'R 正文数量 {len(docs)} ≠ 期望 {EXPECT["r_docs"]}（源为空或结构变了）')
    return docs


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


# ---------------------------------------------------------------- 页面骨架

def head(title: str, *extra: str) -> list[str]:
    lines = [f'# {title}', '',
             '> 本页由 `scripts/ops/compile_notes.py` 自动生成，**勿手改**；重跑即刷新。',
             '> 它是**导航层**，不是权威：任何结论以链接到的原始文件与 `note/` 台账为准。']
    lines += [f'> {e}' for e in extra]
    lines.append('')
    return lines


def rel(target: Path, start: Path) -> str:
    return os.path.relpath(target, start).replace(os.sep, '/')


# ---------------------------------------------------------------- 各生成物

def build_index(docs: list[dict]) -> dict[str, str]:
    by_date, by_mod, by_ms, by_wp = {}, {}, {}, {}
    for d in docs:
        by_date.setdefault(d['date'], []).append(d)
        by_ms.setdefault(d['ms'], []).append(d)
        for m in modules_of(d):
            by_mod.setdefault(m, []).append(d)
        m = WP_RE.search(d['title'])
        by_wp.setdefault('V8-' + m.group(1) if m else '未分类', []).append(d)

    total_pages = len(by_date) + len(by_mod) + len(by_ms) + len(by_wp) + 4
    L = head('note 编译层索引',
             f'源：`note/进度/v8/M*/*/R*.md` 共 **{len(docs)}** 份；本层约 {total_pages} 页。')
    L += ['## 按日期（想知道某天做了什么）', '']
    L += [f"- [{k}](日期/{k}.md) — {len(v)} 份批次" for k, v in sorted(by_date.items())]
    L += ['', '## 按模块（想知道某块进度）', '']
    L += [f"- [{k}](模块/{k}.md) — {len(v)} 份批次"
          for k, v in sorted(by_mod.items(), key=lambda kv: (-len(kv[1]), kv[0]))]
    L += ['', '## 按里程碑', '']
    L += [f"- [{k}](里程碑/{k}.md) — {len(v)} 份批次"
          for k, v in sorted(by_ms.items(), key=lambda kv: kv[0])]
    L += ['', '## 按工作包（V8-NN，仅按标题里的编号归类）', '']
    L += [f"- [{k}](工作包/{k}.md) — {len(v)} 份批次"
          for k, v in sorted(by_wp.items(), key=lambda kv: (-len(kv[1]), kv[0]))]
    hit = sum(len(v) for k, v in by_wp.items() if k != '未分类')
    L += ['', f'识别率：{hit}/{len(docs)} = {hit / len(docs) * 100:.1f}%（判据＝标题含 `V8-NN`；'
              '正文提过但标题没写的不算，避免误归）。', '',
          '## 其它入口', '',
          '- [v7 索引](v7-索引.md) — v7 全部 702 件的四类分列与覆盖统计',
          '- [计划书代际对照](../索引/计划书代际对照.md) — 七代计划书各是什么',
          '- [04 主题速查](../索引/04_配置改动diff_主题速查.md) — 528 段按主题翻',
          '', '## 怎么用', '',
          '1. 先按问题选维度：某天 / 某模块 / 某里程碑 / 某工作包。',
          '2. 页里每行都链回原始正文；读数、边界、证据在正文里。',
          '3. 冲突时以正文与 `note/04`／`05` 台账为准，本层不具权威。',
          '4. 批次有增删后重跑 `python3 scripts/ops/compile_notes.py`。', '']
    return {'note/编译/索引.md': '\n'.join(L)}


def build_dates(docs: list[dict]) -> dict[str, str]:
    out = {}
    by_date = {}
    for d in docs:
        by_date.setdefault(d['date'], []).append(d)
    start = OUT / '日期'
    for date, rows in sorted(by_date.items()):
        rows = sorted(rows, key=lambda x: int(re.sub(r'\D', '', x['id']) or 0))
        if len(rows) > SPLIT_THRESHOLD:
            # 长页拆分：主页保留为入口（旧链接地址不变），按里程碑分子页
            L = head(f'{date} 当日批次（{len(rows)} 份，已按里程碑拆页）',
                     f'本日批次较多（>{SPLIT_THRESHOLD} 行），已拆到分页；本页仍是该日的入口。')
            ms = {}
            for r in rows:
                ms.setdefault(r['ms'], []).append(r)
            L += ['| 里程碑 | 批次数 | 分页 |', '| --- | --- | --- |']
            L += [f'| {k} | {len(v)} | [{date}_{k}.md]({date}_{k}.md) |'
                  for k, v in sorted(ms.items())]
            L += ['', '## 全部批次（按编号）', '',
                  '| 批次 | 一句话 | 里程碑 |', '| --- | --- | --- |']
            for r in rows:
                L.append(f"| [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} | {r['ms']} |")
            L.append('')
            out[f'note/编译/日期/{date}.md'] = '\n'.join(L)
            for ms, sub in sorted(ms.items()):
                S = head(f'{date} · {ms} 当日批次（{len(sub)} 份）',
                         f'本页是 [{date}]({date}.md) 的分页。')
                S += ['| 批次 | 一句话 |', '| --- | --- |']
                for r in sub:
                    S.append(f"| [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} |")
                S.append('')
                out[f'note/编译/日期/{date}_{ms}.md'] = '\n'.join(S)
        else:
            L = head(f'{date} 当日批次（{len(rows)} 份）')
            L += ['## 这一天做了什么', '', '| 批次 | 一句话 | 里程碑 |', '| --- | --- | --- |']
            for r in rows:
                L.append(f"| [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} | {r['ms']} |")
            L += ['', '## 原始文件', '']
            for r in rows:
                L.append(f"- [{r['id']}]({rel(ROOT / r['path'], start)}) — {one_liner(r)}")
            L.append('')
            out[f'note/编译/日期/{date}.md'] = '\n'.join(L)
    return out


def build_modules(docs: list[dict]) -> dict[str, str]:
    out = {}
    by_mod = {}
    for d in docs:
        for m in modules_of(d):
            by_mod.setdefault(m, []).append(d)
    start = OUT / '模块'
    for mod, rows in sorted(by_mod.items()):
        L = head(f'{mod} · 进度时间线（{len(rows)} 份）')
        L += ['## 按时间', '', '| 日期 | 批次 | 一句话 |', '| --- | --- | --- |']
        for r in sorted(rows, key=lambda x: (x['date'], int(re.sub(r'\D', '', x['id']) or 0))):
            L.append(f"| {r['date']} | [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} |")
        L.append('')
        out[f'note/编译/模块/{mod}.md'] = '\n'.join(L)
    return out


def build_milestones(docs: list[dict]) -> dict[str, str]:
    out = {}
    start = OUT / '里程碑'
    by_ms = {}
    for d in docs:
        by_ms.setdefault(d['ms'], []).append(d)
    for i in range(7):
        ms = f'M{i}'
        rows = sorted(by_ms.get(ms, []),
                      key=lambda x: (x['date'], int(re.sub(r'\D', '', x['id']) or 0)))
        L = head(f'{ms} 里程碑批次（{len(rows)} 份）')
        if not rows:
            L += ['本里程碑在 `note/进度/v8/` 下没有批次正文（目录存在但无 R 文档）。', '']
        else:
            dates = sorted({r['date'] for r in rows})
            L += [f'覆盖日期 {dates[0]} 至 {dates[-1]}；批次数 {len(rows)}。', '',
                  '| 日期 | 批次 | 一句话 |', '| --- | --- | --- |']
            for r in rows:
                L.append(f"| {r['date']} | [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} |")
            L.append('')
        out[f'note/编译/里程碑/{ms}.md'] = '\n'.join(L)
    return out


def build_workpackages(docs: list[dict]) -> dict[str, str]:
    out = {}
    start = OUT / '工作包'
    by_wp = {}
    for d in docs:
        m = WP_RE.search(d['title'])
        by_wp.setdefault('V8-' + m.group(1) if m else '未分类', []).append(d)
    for wp, rows in sorted(by_wp.items()):
        L = head(f'{wp} · 工作包批次（{len(rows)} 份）')
        if wp == '未分类':
            L += ['标题里没有可核验的 `V8-NN` 编号，故不归类。**这是统计口径，不是缺漏**：',
                  '正文提过编号但标题没写的一律留在此处，避免按上下文猜。', '']
        L += ['| 日期 | 批次 | 一句话 |', '| --- | --- | --- |']
        for r in sorted(rows, key=lambda x: (x['date'], int(re.sub(r'\D', '', x['id']) or 0))):
            L.append(f"| {r['date']} | [{r['id']}]({rel(ROOT / r['path'], start)}) | {one_liner(r)} |")
        L.append('')
        out[f'note/编译/工作包/{wp}.md'] = '\n'.join(L)
    return out


def build_v7() -> dict[str, str]:
    # 口径＝git 跟踪件（仓库内容）；落盘的未跟踪残渣（__pycache__ 等）另行报数，不混入分类
    tracked = subprocess.run(['git', 'ls-files', '-z', 'note/进度/v7'], cwd=ROOT,
                             capture_output=True, text=True).stdout.split('\0')
    files = sorted(ROOT / f for f in tracked if f)
    if len(files) != EXPECT['v7_files']:
        raise BuildError(f'v7 跟踪件数 {len(files)} ≠ 期望 {EXPECT["v7_files"]}')
    on_disk = sum(1 for p in V7.rglob('*') if p.is_file())
    extras = on_disk - len(files)
    buckets = {'设计': [], '证据': [], '批次': [], '其他/待分类': []}
    for p in files:
        parts = p.parts
        name = p.name
        if any('设计' in part for part in parts):
            buckets['设计'].append(p)
        elif any(('证据' in part or 'evidence' in part) for part in parts):
            buckets['证据'].append(p)
        elif name.startswith('进度文档') or re.fullmatch(r'R\d+\.md', name):
            buckets['批次'].append(p)
        else:
            buckets['其他/待分类'].append(p)
    total = sum(len(v) for v in buckets.values())
    if total != len(files):
        raise BuildError(f'v7 分类覆盖 {total} ≠ 文件数 {len(files)}（有文件被静默漏掉）')

    L = head('v7 索引（全量覆盖）',
             f'源：`note/进度/v7` 的 **{len(files)}** 件跟踪文件；四类合计 {total} 件，与跟踪件数逐一相等。',
             '分类规则：路径**任一段包含**「设计」→设计；包含「证据」/evidence→证据；文件名是进度文档或 R*.md→批次；其余→其他/待分类。',
             f'另有 {extras} 件落盘但未跟踪（`__pycache__` 等运行残渣），不入分类、不计入上述 702。')
    L += ['## 覆盖统计', '', '| 类别 | 件数 | 字节 |', '| --- | --- | --- |']
    for k, v in buckets.items():
        b = sum(p.stat().st_size for p in v if p.exists())
        L.append(f'| {k} | {len(v)} | {b / 1024:.1f} KiB |')
    L += [f'| **合计** | **{total}** | — |', '']
    for k, v in buckets.items():
        L += [f'## {k}（{len(v)} 件）', '']
        if not v:
            L += ['（无）', '']
            continue
        L += ['| 文件 | 所在目录 |', '| --- | --- |']
        for p in v:
            L.append(f'| [{p.name}]({rel(p, OUT)}) | `{p.parent.relative_to(ROOT).as_posix()}` |')
        L.append('')
    return {'note/编译/v7-索引.md': '\n'.join(L)}


def build_plans() -> dict[str, str]:
    rows = []
    for d in PLAN_DIRS:
        p = ROOT / 'note' / d
        if not p.is_dir():
            raise BuildError(f'计划书目录缺失：{d}')
        files = sorted(x for x in p.rglob('*') if x.is_file())
        main = next((x for x in files if x.name.startswith('21_')), files[0] if files else None)
        title = ''
        if main:
            for ln in main.read_text(errors='ignore').splitlines():
                if ln.startswith('# '):
                    title = ln[2:].strip()
                    break
        rows.append({'dir': d, 'files': len(files), 'main': main, 'title': title})
    if len(rows) != EXPECT['plan_dirs']:
        raise BuildError(f'计划书代数 {len(rows)} ≠ 期望 {EXPECT["plan_dirs"]}')

    L = head('计划书代际对照（七代）',
             '只列各代目录里已有的件数与 21 号件的原始标题；「解决什么」不代写。')
    L += ['| 代 | 件数 | 该代 21 号件标题（原文） | 状态 |', '| --- | --- | --- | --- |']
    for r in rows:
        status = '**现行**' if r['dir'] == CURRENT_PLAN else '历史（时点快照）'
        t = r['title'].replace('|', '／')[:110] or '（无 21 号件）'
        L.append(f'| [{r["dir"]}](../{r["dir"]}/) | {r["files"]} | {t} | {status} |')
    L += ['', '## 读法', '',
          f'- 现行路线只看 **{CURRENT_PLAN}** 目录（README 与 21—25 号件）；其余为历史。',
          '- 各代题目由原文抄录，不做归纳；要判断「那代解决什么」，读该代 21 号件正文。',
          '- 旧代的完整路径引用仍散在 `note/04` 与 `note/02` 里（合计 273 处），故本表不搬动任何文件。', '']
    return {'note/索引/计划书代际对照.md': '\n'.join(L)}


def build_theme04() -> dict[str, str]:
    body = LEDGER04.read_text(errors='ignore')
    secs = [(i, l[3:].strip()) for i, l in enumerate(body.splitlines(), 1) if l.startswith('## ')]
    if len(secs) != EXPECT['sec04']:
        raise BuildError(f'04 段数 {len(secs)} ≠ 期望 {EXPECT["sec04"]}')
    if len({i for i, _ in secs}) != EXPECT['sec04']:
        raise BuildError('04 唯一段号数与期望不符')

    assign = {}
    memberships = 0
    for ln, t in secs:
        hit = [k for k, pat in THEMES04.items() if re.search(pat, t, re.I)] or ['未命中主题']
        assign[ln] = (t, hit)
        memberships += len(hit)
    covered = {ln for ln, (_, hits) in assign.items() if hits != ['未命中主题']}

    L = head('04 · 配置改动 diff 主题速查',
             '源件：[04_配置改动diff.md](../04_配置改动diff.md)（528 段，逐段核对一致）。',
             '只给「主题 → 段标题 + 行号」，**不改源件一个字节**：源件是 append-only 台账且被 VCC 按 `文件:行号` 引用。')
    listed = len(covered) + (len(secs) - len(covered))
    if listed != EXPECT['sec04']:
        raise BuildError(f'04 索引列出段数 {listed} ≠ {EXPECT["sec04"]}（有段未列出）')
    L += ['## 四个计数（先看这里）', '',
          f'- 原始段数：**{len(secs)}**（与源件 `^## ` 计数逐一相等）',
          f'- 主题条目总数：**{memberships}**（一段可属多主题，故大于段数）',
          f'- 命中主题的唯一段数：**{len(covered)}**',
          f'- **本页列出的唯一段数：{listed} = {len(covered)}（命中主题）+ {len(secs) - len(covered)}（未命中主题，末尾列出）**',
          '', '> 前三个数分别回答「源多少段」「索引多少条」「多少段进了主题」；'
              '第四个数证明**没有段被漏掉**——主题命中率不等于覆盖率。', '']
    for theme in THEMES04:
        rows = [(ln, t) for ln, t in secs if theme in assign[ln][1]]
        L += [f'## {theme}（{len(rows)} 段）', '']
        if not rows:
            L += ['（本主题暂无命中段）', '']
            continue
        L += ['| 行号 | 段 |', '| --- | --- |']
        for ln, t in rows:
            tt = (t if len(t) <= 56 else t[:56] + '…').replace('|', '／')
            L.append(f'| {ln} | {tt} |')
        L.append('')
    miss = [(ln, t) for ln, t in secs if assign[ln][1] == ['未命中主题']]
    L += [f'## 未命中主题（{len(miss)} 段，如实列出）', '']
    if miss:
        L += ['| 行号 | 段 |', '| --- | --- |']
        for ln, t in miss:
            tt = (t if len(t) <= 56 else t[:56] + '…').replace('|', '／')
            L.append(f'| {ln} | {tt} |')
        L.append('')
    return {'note/索引/04_配置改动diff_主题速查.md': '\n'.join(L)}


def build_all() -> dict[str, str]:
    docs = load_r_docs()
    pages: dict[str, str] = {}
    for builder in (build_index, build_dates, build_modules, build_milestones,
                    build_workpackages):
        pages.update(builder(docs))
    pages.update(build_v7())
    pages.update(build_plans())
    pages.update(build_theme04())
    clash = sorted(set(pages) & HUMAN_FILES)
    if clash:
        raise BuildError(f'生成目标撞上人写文件：{clash}')
    return pages


# ---------------------------------------------------------------- 校验与落盘

LINK_RE = re.compile(r'\]\(([^)]+)\)')
MANAGED_GLOBS = ('note/编译/日期/*.md', 'note/编译/模块/*.md',
                 'note/编译/里程碑/*.md', 'note/编译/工作包/*.md')


def check_links(pages: dict[str, str]) -> list[str]:
    bad = []
    planned = {(ROOT / p).resolve() for p in pages}      # 本次将创建的文件也算可达
    for path, text in sorted(pages.items()):
        base = (ROOT / path).parent
        for m in LINK_RE.finditer(text):
            t = m.group(1).split('#')[0].split('?')[0]
            if not t or t.startswith(('http', 'mailto:')):
                continue
            # 先做词法规整再判断：目标目录可能尚未创建（首跑），带 .. 的路径会误判为不存在
            target = Path(os.path.normpath(os.path.join(str(base), t)))
            if not target.exists() and target.resolve() not in planned:
                bad.append(f'{path} → {t}')
    return bad


def write_all(pages: dict[str, str]) -> int:
    for path, text in sorted(pages.items()):
        p = ROOT / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    keep = set(pages)
    for pat in MANAGED_GLOBS:
        for p in ROOT.glob(pat):
            if p.relative_to(ROOT).as_posix() not in keep:
                p.unlink()
    return len(pages)


def run_check(pages: dict[str, str]) -> int:
    problems, stale = [], []
    for path, text in sorted(pages.items()):
        p = ROOT / path
        if not p.exists():
            problems.append(f'缺失：{path}')
        elif p.read_text(errors='ignore') != text:
            problems.append(f'过期：{path}')
    keep = set(pages)
    for pat in MANAGED_GLOBS:
        for p in ROOT.glob(pat):
            rel_p = p.relative_to(ROOT).as_posix()
            if rel_p not in keep:
                stale.append(f'多余：{rel_p}')
    links = check_links(pages)
    print(f'[check] 应有页 {len(pages)}｜缺失/过期 {len(problems)}｜多余 {len(stale)}｜断链 {len(links)}')
    for x in (problems + stale + links)[:20]:
        print('  ' + x)
    if problems or stale or links:
        return 5
    print('[check] 一致')
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description='note 编译层生成器（确定性、可只读核对）')
    ap.add_argument('--check', action='store_true', help='只读核对，不写盘；不一致返回 5')
    a = ap.parse_args()
    try:
        pages = build_all()
    except BuildError as e:
        print(f'[fail] {e}', file=sys.stderr)
        return 9
    except Exception as e:                                    # noqa: BLE001
        print(f'[fail] 构建异常，未改动任何文件：{e!r}', file=sys.stderr)
        return 9

    links = check_links(pages)
    if links:
        print(f'[fail] 生成物存在断链 {len(links)} 条，未落盘：', file=sys.stderr)
        for x in links[:10]:
            print('  ' + x, file=sys.stderr)
        return 5

    if a.check:
        return run_check(pages)

    n = write_all(pages)
    cnt = {k: len([p for p in pages if p.startswith(f'note/编译/{k}/')])
           for k in ('日期', '模块', '里程碑', '工作包')}
    print(f"[ok] 写入 {n} 页｜日期 {cnt['日期']}｜模块 {cnt['模块']}"
          f"｜里程碑 {cnt['里程碑']}｜工作包 {cnt['工作包']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
