#!/usr/bin/env bash
# v_ledger_batch_consistency.sh —— 台账「批号账实一致性」核验件（**R233 已裁入门、门禁串七→八**；小明 2026-10-11 原型，2026-10-13 R234 施工）
#
# ★成闸三参数＝主理人 2026-10-12 裁「0-b 照推荐」（v4.7 稿 Z3）：判定域下限 **R140**／FAIL 集＝**C2+C4**／滞后阈值 **2 批**；
#   **C1 与 C3 只报不判**（C1 提交前必差一批属结构性现象；C3 缺号靠人补条目、机器不代判）。
# ★「新增才拦、存量逐条带出处」＝照第七件 PARITY 的语义（Z3 原文要求）；存量表缺 ⇒ **rc=9 不判**（缺表不判绿，同第七件）。
#
# 为何存在：本仓的批号（RNNN）同时写在三处——①`note/进度/00_总览.md` 档头"更新时间"里声明的「RNNN 批同步」
#   ②同档文末的逐批条目 ③git 提交标题 `docs(RNNN)`。R160 滚档头时自查抓到两处**账实不符**：
#   旧档头自写「R145 批同步」而 git 里真正滚档头的那批是 **R146（fa2796d）**；**R153 在 00_总览 没有独立条目**
#   （读数被后批吸收、原件在 aa 稿 §9）。这类"合法、点得开、却指到别处"的错**没有任何东西变红**（坑60／坑61 那一族），
#   而档头滞后本身已**第五次复发**（R103／R104／R127／R145／R160）——没有件盯着就必然复发。
#
# ★本件**只报数，不自判红绿**：哪些项算 FAIL、阈值与域下限多少＝请求单 **A26** 待裁（主理人／小美，归 A15／#54／#16 那一族）。
#   我不自立门禁串、不改判据／门值。四条候选校验：
#     C1 档头声明的批号 vs git 里最后一次真动「档头那一行」的批号（不同值＝账实不符）
#     C2 git 有该批号、而总览条目区没有这一条（＝该批没落总览）
#     C3 总览条目批号在判定域内的缺号
#     C4 档头滞后批数（条目区域内最大批号 − 档头声明批号）
#
# ★两个必须写下来的已知局限（不写就会变成假红制造机，坑53）：
#   ①**判定域下限**（`VLB_MIN_R`，缺省 100）：R 号写进总览条目是 R100 前后才成形的，早于它的条目文体不带号
#      ⇒ 不设域会把整段早期史报成"缺条目"。域下限本身是要裁的参数之一。
#   ②**锚必须唯一**（坑42）：条目"自有批号"只认严格形状 `- 日期：**RNNN`；git 号只认标题前缀 `docs(RNNN)`。
#      本件**第一版**取的是"行内首个 R 号"，真仓一跑就露馅：169 条目行抽出 108 个号、最小竟成 R0，
#      因为很多条目开头是中文标题、行内引的是**别的批**——正是坑42「锚不唯一 ⇒ 静默抽到别家表达式」的复刻。
#
# 自标定（P1，阳性对照在前；坑29／坑42 姿势）：--selftest 用注桩件跑五项，任一对不上退 rc=9（普查结果一律不采信）。
#
# 用法：bash scripts/verify/v_ledger_batch_consistency.sh [--selftest]
#   环境变量（只为自测注桩，缺省一律读真仓）：
#     VLB_OVERVIEW＝替代 00_总览 的桩文件｜VLB_GIT_LOG＝每行一个提交标题的桩（第 1 行视为最新）
#     VLB_HDR_SUBJECT＝"最后一次改动档头那一行"的提交标题桩（真跑时用 git log -L 2,2 现取）
#     VLB_MIN_R＝判定域下限（缺省 100）｜VLB_ROOT＝仓根
#   退出码：0 数据齐（**只表示报得出数，不代表账实相符**）｜9 前置缺失或自标定不过
# 性质：零 ROS 节点、零链窗、零写盘、无 sudo、只读。
set -u
ROOT="${VLB_ROOT:-/home/ros1-slam}"
cd "$ROOT" || { echo "前置缺失：仓根 $ROOT 不在位"; echo "BATCHCHECK rc=9"; exit 9; }
MODE="report"
case "${1:-}" in
  --selftest) MODE="selftest" ;;
esac
python3 - "$MODE" "$ROOT" <<'PYEOF'
# -*- coding: utf-8 -*-
import os, re, subprocess, sys

MODE, ROOT = sys.argv[1], sys.argv[2]
OVERVIEW = os.environ.get('VLB_OVERVIEW') or os.path.join(ROOT, 'note/进度/00_总览.md')
ENTRY = re.compile(r'^- \d{4}-\d{2}-\d{2}')
SELF = re.compile(r'^- \d{4}-\d{2}-\d{2}：\*\*R(\d{2,4})')
# git 标题：只认**行首那个括号段**，但段内可以有多个号或带后缀（实测在册形状有 `docs(R153)`、
# `test+docs(R158)`、`docs(R106 更正)`、`docs(R105＋R106)` 四种——只认第一种会漏，漏了就误报成"缺号"）
GITSUB = re.compile(r'^[a-z+]+\(([^)]{1,80})\)')
# ★R344（真人 2026-10-17 明文取甲）＝号形放宽为可带连字符后缀（`R342-2` 一类子批号），声明号取基段（342）
#   ⇒ C4 滞后式与 FAIL 集／阈值／判定域下限一字未动；失配曾回落到历史串（R332）造出假滞后 10 批＝新增红根因。
HDR = re.compile(r'更新时间：[^（]*（\s*R(\d{2,4})(?:-\d+)? 批同步')
DOMAIN_MIN = int(os.environ.get('VLB_MIN_R', '140'))   # Z3 已裁：判定域下限 R140（旧缺省 100 的读数不回改，读时须带域）
LAG_MAX = int(os.environ.get('VLB_LAG_MAX', '2'))       # Z3 已裁：档头滞后阈值 2 批
FAIL_SET = os.environ.get('VLB_FAIL_SET', 'C2+C4+C5')   # Z3 原裁＝C2＋C4；R237 主理人加裁「C5 入集，仅拦新增」⇒ 现缺省 C2＋C4＋C5
C5_KNOWN_FILE = os.environ.get('VLB_C5_KNOWN') or os.path.join(ROOT, 'scripts/verify/batch_c5_known_lines.tsv')
KNOWN_FILE = os.environ.get('VLB_KNOWN') or os.path.join(ROOT, 'scripts/verify/batch_known_findings.tsv')


def read(path):
    return open(path, encoding='utf-8', errors='ignore').read()


def git_subjects():
    if os.environ.get('VLB_GIT_LOG'):
        return [l.rstrip('\n') for l in read(os.environ['VLB_GIT_LOG']).splitlines() if l.strip()]
    try:
        out = subprocess.run(['git', 'log', '--format=%s'], capture_output=True, text=True, cwd=ROOT)
        if out.returncode != 0:
            return None
        return [l for l in out.stdout.splitlines() if l.strip()]
    except Exception:
        return None


def hdr_commit_subject():
    """最后一次改动「档头那一行」的提交标题——按行定位，不靠人记（坑60：行号是版本限定的）。"""
    if os.environ.get('VLB_HDR_SUBJECT'):
        return os.environ['VLB_HDR_SUBJECT']
    try:
        out = subprocess.run(['git', 'log', '-L', '2,2:note/进度/00_总览.md', '--format=%s', '-n', '1'],
                             capture_output=True, text=True, cwd=ROOT)
        for line in out.stdout.splitlines():
            if git_rs(line):
                return line
        return None
    except Exception:
        return None


def self_r(line):
    m = SELF.match(line)
    return int(m.group(1)) if m else None


def git_rs(subject):
    """取一行提交标题**行首括号段**里的全部批号（返回 list，可能多个）。"""
    m = GITSUB.match(subject)
    return [int(x) for x in re.findall(r'R(\d{2,4})', m.group(1))] if m else []


def check(ov_text, subs, hdr_sub):
    hdr = HDR.search(ov_text)
    declared = int(hdr.group(1)) if hdr else None
    all_ids = [self_r(l) for l in ov_text.splitlines() if ENTRY.match(l)]
    ids_raw = [x for x in all_ids if x]
    n_entry = sum(1 for l in ov_text.splitlines() if ENTRY.match(l))
    ids = [x for x in ids_raw if x >= DOMAIN_MIN]
    git_ids = set(x for s in subs for x in git_rs(s) if x >= DOMAIN_MIN)
    _ha = git_rs(hdr_sub) if hdr_sub else []
    act = _ha[0] if _ha else None
    max_id = max(ids) if ids else None
    res = {}
    res['C1'] = {'declared': declared, 'actual': act, 'match': (declared is not None and declared == act)}
    res['C2'] = {'missing_from_ledger': sorted(git_ids - set(ids))}
    res['C3'] = {'gaps': sorted(set(range(min(ids), max(ids) + 1)) - set(ids))} if ids else {'gaps': []}
    res['C4'] = {'lag': (max_id - declared) if (declared is not None and max_id) else None, 'max': max_id}
    # C5 行首自测锁（R236，#110 第四段；**只报不判**——Z3 已裁 FAIL 集＝C2+C4，我不擅自把它加进去）
    FMT = re.compile(r'^- \d{4}-\d{2}-\d{2}：\*\*R\d{2,4}')
    FMT_MIN = os.environ.get('VLB_FMT_MIN_DATE', '2026-10-05')
    bad_fmt = []
    for ln in ov_text.splitlines():
        if not ENTRY.match(ln):
            continue
        d = ln[2:12]
        if d >= FMT_MIN and not FMT.match(ln):
            bad_fmt.append(d + '|' + ln[12:52].replace('\n', ''))
    res['C5_行首不合规'] = {'数': len(bad_fmt), '样': bad_fmt[:4], '样全': bad_fmt}
    res['_面'] = {'条目行': n_entry, '域内号数': len(set(ids)), '无自有号的条目行': n_entry - len(ids_raw),
                  '域外号数': len(set(x for x in ids_raw if x < DOMAIN_MIN)),
                  'git 提交': len(subs), 'git 域内号数': len(git_ids), '域下限': DOMAIN_MIN}
    return res


def load_known():
    """存量表：类别<TAB>键<TAB>定性<TAB>出处 ⇒ 命中即「存量」，不判；表缺＝rc=9（不判绿）。"""
    if not os.path.isfile(KNOWN_FILE):
        return None
    k = {'C2': {}, 'C4': {}}
    for row in open(KNOWN_FILE, encoding='utf-8', errors='ignore').read().splitlines():
        if not row.strip() or row.startswith('类别'):
            continue
        c = row.split('\t')
        if len(c) < 4:
            continue
        k.setdefault(c[0], {})[c[1]] = (c[2], c[3])
    return k


def load_c5_known():
    """C5 行首锁的存量基线（R237）：每行一个键，#开头为注释。缺文件 ⇒ 不判（rc=9）。"""
    if not os.path.isfile(C5_KNOWN_FILE):
        return None
    ks = set()
    for row in open(C5_KNOWN_FILE, encoding='utf-8', errors='ignore').read().splitlines():
        if not row.strip() or row.startswith('#'):
            continue
        ks.add(row)
    return ks


def verdict(res, known, c5_known=None):
    """FAIL 集＝C2 新增＋C4 滞后＞阈值＋（R237 加裁）C5 行首不合规之**新增**；C1／C3 仍只报不判。"""
    judge = set(FAIL_SET.split('+'))
    c2_new = [x for x in res['C2']['missing_from_ledger'] if str(x) not in (known or {}).get('C2', {})]
    c2_old = [x for x in res['C2']['missing_from_ledger'] if str(x) in (known or {}).get('C2', {})]
    lag = res['C4']['lag']
    c4_bad = ('C4' in judge and lag is not None and lag > LAG_MAX)
    c5_bad_list = res['C5_行首不合规']['样全'] if 'C5' in judge else []
    c5_new = [k for k in c5_bad_list if k not in (c5_known or set())]
    c5_old = [k for k in c5_bad_list if k in (c5_known or set())]
    rc = 5 if (c2_new or c4_bad or c5_new) else 0
    return {'rc': rc, 'C2_新增': c2_new, 'C2_存量命中': c2_old,
            'C4_判': ('滞后 %s 批＞阈值 %s ⇒ 拦' % (lag, LAG_MAX)) if c4_bad
                     else ('滞后 %s 批≤阈值 %s ⇒ 放' % (lag, LAG_MAX)),
            'C5_判': (('不合规 %d 行＝新增 %d／存量 %d ⇒ %s' % (len(c5_bad_list), len(c5_new), len(c5_old),
                     '拦' if c5_new else '放')) if 'C5' in judge else '只报不判（未入 FAIL 集）'),
            'C5_新增': c5_new, 'C5_存量命中': len(c5_old),
            'FAIL集': FAIL_SET, '域下限': DOMAIN_MIN, '阈值': LAG_MAX}


def show(res, tag=''):
    d, a, lag = res['C1']['declared'], res['C1']['actual'], res['C4']['lag']
    print('%s[C1 档头 vs 实际] 声明=R%s／实动档头=R%s ⇒ %s' % (
        tag, d if d is not None else '未抽到', a if a is not None else '未取到',
        '同值' if res['C1']['match'] else '**不同值（账实不符）**'))
    print('%s[C2 git 有而总览条目区无（域内）] %s' % (tag, res['C2']['missing_from_ledger'] or '无'))
    print('%s[C3 总览条目批号缺号（域内）] %s' % (tag, res['C3']['gaps'] or '无'))
    print('%s[C4 档头滞后] %s 批（条目区域内最大 R%s − 档头 R%s）' % (
        tag, '未可算' if lag is None else lag, res['C4']['max'], d))
    c5 = res['C5_行首不合规']
    print('%s[C5 行首自测锁（只报不判；日期≥%s 的条目行须以「- 日期：**R<号>」开头）] 不合规=%d 行%s' % (
        tag, os.environ.get('VLB_FMT_MIN_DATE', '2026-10-05'), c5['数'],
        ('｜样=' + str(c5['样'])) if c5['样'] else ''))
    print('%s[C5 入集态] FAIL 集=%s ⇒ %s' % (tag, FAIL_SET, 'C5 已入集（仅拦新增）' if 'C5' in set(FAIL_SET.split('+')) else 'C5 只报不判'))
    m = res['_面']
    print('%s[面] 条目行=%d 域内号=%d 无自有号=%d 域外号=%d ｜ 判定域≥R%d ｜ git 提交=%d 域内号=%d' % (
        tag, m['条目行'], m['域内号数'], m['无自有号的条目行'], m['域外号数'], m['域下限'],
        m['git 提交'], m['git 域内号数']))
    print('%s[域下限的来历] R 号写进条目是 R100 前后才成形的事，早于它的条目文体不带号 ⇒ 不设域就会把整段早期史'
          '报成"缺条目"（坑53 假红制造机形状）。域下限本身＝A26 要勾的参数之一。' % tag)


if MODE == 'selftest':
    ov = ('# 进度总览\n'
          '> 日期 ｜ 维护 ｜ **更新时间：2026-01-01 10:0x（R150 批同步：略）**\n'
          '## 别的段\n'
          '- 2026-01-01：**R51 早期一批**（域外，不参与缺号与缺条目判定）\n'
          '- 2026-01-02：老文体条目，不带自有号\n'
          '- 2026-01-04：**R151 一批**\n'
          '- 2026-01-05：**R152 一批**（此行文字里引用 R51 与 R999，考验锚是否唯一）\n'
          '- 2026-01-05：**R154 一批**（R153 有提交无条目）\n'
          '- 2026-01-06：**R155 最新一条**\n')
    subs = ['docs(R155): x', 'docs(R154 更正): 段内带后缀也要认得出 R154', 'docs(R153): 这一批没落总览条目',
            'docs(R157＋R158): 段内两个号 ⇒ 都得认（真仓有这形状，漏认就会误报缺号）', 'docs(R152): x',
            'test+docs(R151): x', 'docs(R51): 域外老批（有提交，但不该进缺条目名单）',
            'chore: 无批号', 'docs(x): 标题正文里写 R888 也不该算',
            'Re: docs(R999): 括号不在行首 ⇒ 是引用别批，不该认成本提交的号']
    # 条目桩补两条：让 R157／R158 有自有条目，否则它们会假报成缺条目（C2）
    ov += ('- 2026-01-06：**R156 一批**\n'
           '- 2026-01-06：**R157 一批**\n'
           '- 2026-01-06：**R158 一批**\n')
    hdr = 'docs(R151): 滚档头的那一次'
    r = check(ov, subs, hdr)
    ok1 = (r['C1']['declared'] == 150 and r['C1']['actual'] == 151 and r['C1']['match'] is False)
    ok2 = r['C2']['missing_from_ledger'] == [153] and r['_面']['git 域内号数'] == 7
    ok3 = r['C3']['gaps'] == [153]
    ok4 = r['C4']['lag'] == 8
    ok5 = r['_面']['域内号数'] == 7 and r['_面']['域外号数'] == 1 and r['_面']['无自有号的条目行'] == 1
    print('[%s] T1 C1 须报"声明 R150≠实动 R151"：实得 declared=%s actual=%s match=%s' % (
        'PASS' if ok1 else 'FAIL', r['C1']['declared'], r['C1']['actual'], r['C1']['match']))
    print('[%s] T2 C2 名单须恰为 [153]、git 域内号须=7（含带后缀与段内两号两种形状；R51 域外、R888 正文、R999 不在行首皆不计）：实得 %s／号数=%s' % (
        'PASS' if ok2 else 'FAIL', r['C2']['missing_from_ledger'], r['_面']['git 域内号数']))
    print('[%s] T3 C3 缺号须恰为 [153]（严格自有锚：行内引用 R51/R999 不得顶掉本批号）：实得 %s' % (
        'PASS' if ok3 else 'FAIL', r['C3']['gaps']))
    print('[%s] T4 C4 滞后须为 8 批（158−150）：实得 %s' % ('PASS' if ok4 else 'FAIL', r['C4']['lag']))
    print('[%s] T5 域与形状须对上（域内号=7 域外号=1 无号条目行=1）：实得 域内=%s 域外=%s 无号=%s' % (
        'PASS' if ok5 else 'FAIL', r['_面']['域内号数'], r['_面']['域外号数'], r['_面']['无自有号的条目行']))
    # ★ 判定层三例（R234 入门时加）：每例都必须能**单独失败**，否则等于没有守卫。
    #   第一版我把这三例接在 r（档头 R150、滞后 8 批）上 ⇒ T6 被 C4 污染成 rc=5 假 FAIL、T8 因把 git 标题当档头而 lag 仍=8：
    #   **C4 的滞后＝「条目区最大号 − 档头声明号」，与 git 标题无关**（这一点是被失败照出来的，不是我先想清的）。
    #   ⇒ 判定层必须用**滞后才合规的第二份档头桩** ov2，才能让 C2 单独决定结果。
    ov2 = ov.replace('（R150 批同步', '（R156 批同步')
    r2 = check(ov2, subs, 'docs(R156): x')
    kn = {'C2': {'153': ('存量', '桩出处')}, 'C4': {}}
    v_in = verdict(r2, kn)                       # 存量命中 ⇒ 应放
    v_out = verdict(r2, {'C2': {}, 'C4': {}})    # 同一个号不在表 ⇒ 必须拦
    # T8 阈值边界：滞后恰＝2（等于阈值）须放；把档头改成 R155（滞后 3）须拦
    v_edge = verdict(r2, kn)
    r3 = check(ov.replace('（R150 批同步', '（R155 批同步'), subs, 'docs(R155): x')
    v_over = verdict(r3, {'C2': {'153': ('存量', '桩')}, 'C4': {}})
    ok6 = (r2['C4']['lag'] == 2 and v_in['rc'] == 0 and v_in['C2_存量命中'] == [153] and not v_in['C2_新增'])
    ok7 = (v_out['rc'] == 5 and v_out['C2_新增'] == [153])
    ok8 = (v_edge['rc'] == 0 and r3['C4']['lag'] == 3 and v_over['rc'] == 5)
    print('[%s] T6 滞符合规＋C2 仅存量 ⇒ 须放（rc=0、新增为空）：实得 lag=%s rc=%s 新增=%s 存量=%s' % (
        'PASS' if ok6 else 'FAIL', r2['C4']['lag'], v_in['rc'], v_in['C2_新增'], v_in['C2_存量命中']))
    print('[%s] T7 同一个号从存量表去掉必须拦住（rc=5 且 C2_新增=[153]）——**这条不绿就说明表是橡皮图章**：实得 rc=%s 新增=%s' % (
        'PASS' if ok7 else 'FAIL', v_out['rc'], v_out['C2_新增']))
    print('[%s] T8 阈值边界：滞后＝2（等于阈值）须放、滞后＝3（＞阈值）须拦：实得 lag2=%s rc=%s／lag3=%s rc=%s' % (
        'PASS' if ok8 else 'FAIL', r2['C4']['lag'], v_edge['rc'], r3['C4']['lag'], v_over['rc']))
    bad = ov + '- 2026-10-11 09:00：R236 这批我把行首写成了「日期 时:分：」老误形\n'
    r_bad = check(bad, subs, hdr)
    ok9 = (r['C5_行首不合规']['数'] == 0 and r_bad['C5_行首不合规']['数'] == 1)
    print('[%s] T9 行首自测锁：合规桩须报 0 行、注入一行坏文体必须报出来（只报不判，但**报不出来＝锁失效**）：实得 干净=%s 注入=%s' % (
        'PASS' if ok9 else 'FAIL', r['C5_行首不合规']['数'], r_bad['C5_行首不合规']['数']))
    # 注意：必须用**滞符合规的桩 r2**，否则 C4 先拦、C5 的对照就失去意义（R235 刚记过这条，T10 第一版又踩了）
    bad2 = ov2 + '- 2026-10-11 09:00：R237 这批我把行首写成「日期 时:分：」老误形\n'
    r_bad2 = check(bad2, subs, 'docs(R156): x')
    k10 = set(r2['C5_行首不合规']['样全'])
    v_new = verdict(r_bad2, kn, k10)                                        # 注入行不在基线 ⇒ 必须拦
    v_old = verdict(r_bad2, kn, k10 | {r_bad2['C5_行首不合规']['样全'][0]})  # 同一行写进基线 ⇒ 必须放
    ok10 = (v_new['rc'] == 5 and len(v_new['C5_新增']) == 1 and v_old['rc'] == 0 and v_old['C5_新增'] == [])
    print('[%s] T10 C5 入集后：注入不在存量基线的坏行须拦（rc=5）、把同一行写进基线须放（rc=0）——**这条不绿＝"仅拦新增"是假的**：实得 拦=%s(新增%d) 放=%s' % (
        'PASS' if ok10 else 'FAIL', v_new['rc'], len(v_new['C5_新增']), v_old['rc']))

    print('[例四] 缺存量表须 rc=9（不判绿）：由 shell 侧以 VLB_KNOWN=不存在路径 复跑断言（本批实测 rc=9）')
    npass = sum([ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10])
    print('BATCHCHECK SELFTEST VERDICT: PASS=%d FAIL=%d（十例：T1-T5 解析层／T6-T8 判定层／T9-T10 行首锁）' % (npass, 10 - npass))
    sys.exit(0 if npass == 10 else 9)

if not os.path.isfile(OVERVIEW):
    print('前置缺失：总览不在位 %s' % OVERVIEW)
    print('BATCHCHECK rc=9')
    sys.exit(9)
subs = git_subjects()
if subs is None:
    print('前置缺失：git log 读不到（本件不做"读不到就当过"的事）')
    print('BATCHCHECK rc=9')
    sys.exit(9)
res = check(read(OVERVIEW), subs, hdr_commit_subject())
if res['C1']['declared'] is None and res['C4']['lag'] is None:
    print('前置缺失：档头与条目区都抽不到批号 ⇒ 形状变了，普查结果不采信（坑42：锚不唯一／已漂）')
    print('BATCHCHECK rc=9')
    sys.exit(9)
known = load_known()
if known is None:
    print('前置缺失：存量表不在位 %s ⇒ 不判（缺表不判绿，照第七件语义）' % KNOWN_FILE)
    print('BATCHCHECK rc=9')
    sys.exit(9)
show(res)
c5k = load_c5_known()
if 'C5' in set(FAIL_SET.split('+')) and c5k is None:
    print('前置缺失：C5 存量基线不在位 %s ⇒ 不判（缺基线不判绿，同存量表语义）' % C5_KNOWN_FILE)
    print('BATCHCHECK rc=9')
    sys.exit(9)
v = verdict(res, known, c5k)
print('[判定] FAIL集=%s／域下限=R%d／滞后阈值=%d 批' % (v['FAIL集'], v['域下限'], v['阈值']))
print('[判定] C2 新增=%s（拦）／C2 存量命中=%s（放，逐条带出处）' % (v['C2_新增'] or '无', v['C2_存量命中'] or '无'))
print('[判定] C4 %s' % v['C4_判'])
print('[判定] C5 %s%s（C1／C3 只报不判）' % (v['C5_判'], ('｜新增样=' + str(v['C5_新增'][:2])) if v['C5_新增'] else ''))
print('BATCHCHECK VERDICT: rc=%d（0 干净或仅存量／5 FAIL 集命中新增／9 前置缺失）' % v['rc'])
sys.exit(v['rc'])
PYEOF
