#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v_claim_field_audit.py —— 规矩⑨「结论强度词同句须有读数痕迹」**报告器（候选件，未入门禁串、无否决权）**

来历＝主理人 2026-10-11 立规矩⑨（条文在 `05` 追加八）；R215 出 `ae` 稿（存量普查）、R216 出 `af` 稿（三形代价）。
`af` 稿 §4 写了"若裁乙（报告器），施工侧需要哪些明文才动手"——本件按**推荐值**实现，四条明文逐条写在下面并标**待追认**：
  ①判据形状＝S4（三条降噪一起用）——`af` 稿 §1 实测同一面 S0＝56 行、S4＝5 行（降 91%）；
  ②词表边界＝`ae` 稿在册那 8 词，**不扩表**（扩表＝误拦更多且更好绕，`af` 稿 §4 第 2 条明说未量）；
  ③面＝写死清单文件 `claim_face.txt`（照 `parity_face.txt` 的做法，保证逐批可比）；
  ④输出＝逐条清单（不只计数），否则看不出趋势。
边界（写死，防被当成闸）：**只报不拦**——本件 rc 里**没有"违规⇒红"这一档**；要不要入门、阈值取几，归 #12／#19／A32 那一族，
施工侧不自落。**本件不使用 FORCE**（在册 O-5：FORCE 禁用于判定）。
用法：
  bash -c 'python3 scripts/verify/v_claim_field_audit.py'            # 用缺省清单跑全量
  python3 scripts/verify/v_claim_field_audit.py <文件 ...>            # 指定面
  python3 scripts/verify/v_claim_field_audit.py --selftest           # 五例，每例都必须能单独失败
  CFA_SKIP_LINES／CFA_META／CFA_NEIGH／CFA_FACE 可覆盖（仅供对照与自测注桩，不改缺省语义）
rc：0＝跑通（**报了行也是 0**，本件只报不拦）；9＝前置缺失（清单读不到／面为空）。
"""
import os
import re
import sys

WORDS = [u'被实证推翻', u'实证推翻', u'不可终止', u'推翻', u'已证', u'未证', u'坐实', u'确证']
# 长词排前面，免得"被实证推翻"被"推翻"先占位
TRACE = re.compile(r'(?:[A-Za-z_][A-Za-z_0-9]{1,}|=)\s*[=＝]\s*\S|rc\s*=\s*\d|\bsha256\b|sha\s*前\s*12|'
                   r'\d+\s*[.%％]|\d+\.\d+|\d+\s*(?:m|s|Hz|mm|件|条|行|批|字节)|R\d{2,}|[A-Za-z0-9_./-]+\.(?:sh|py|md|launch|yaml|world|pcd|pgm)\b')
META = [u'不得写', u'一律写', u'措辞', u'原写', u'词表', u'结论强度词', u'降档', u'规矩⑨', u'追加八',
        u'必须同句', u'须同句', u'级结论', u'误拦', u'本规矩']
QUOTED = re.compile(u'[「『“"\'（(【](?:' + u'|'.join(re.escape(w) for w in WORDS) + u')[」』”"\'）)】]')
FACE_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'claim_face.txt')


def head_skip():
    return int(os.environ.get('CFA_SKIP_LINES', '8') or 8)


def meta_on():
    return os.environ.get('CFA_META', '1') != '0'


def neigh_on():
    return os.environ.get('CFA_NEIGH', '1') != '0'


def has_word(line):
    return [w for w in WORDS if w in line]


def has_trace(line):
    return bool(TRACE.search(line))


def is_meta(line, words):
    if not meta_on():
        return False
    for t in META:
        if t in line:
            return True
    # 词只出现在引号包裹的短片段里＝提到它，不是下这个结论
    stripped = QUOTED.sub('', line)
    return not has_word(stripped) and len(words) > 0


def audit(lines, start_idx=0):
    """返回逐行判定列表（idx, 命中词, 判定）。降噪三条＝跳头部／元话语过滤／邻行也算痕迹。"""
    out = []
    n = len(lines)
    for i, line in enumerate(lines):
        words = has_word(line)
        if not words:
            continue
        if head_skip() and (start_idx + i) < head_skip():
            out.append((i, words, u'跳过(头部)'))
            continue
        if is_meta(line, words):
            out.append((i, words, u'元话语'))
            continue
        if has_trace(line):
            out.append((i, words, u'同句有痕迹'))
            continue
        if neigh_on():
            near = lines[i - 1] if i > 0 else u''
            nxt = lines[i + 1] if i + 1 < n else u''
            if has_trace(near) or has_trace(nxt):
                out.append((i, words, u'邻行有痕迹'))
                continue
        out.append((i, words, u'报'))
    return out


def run(paths):
    total_lines = 0
    reported = []
    cats = {}
    for p in paths:
        try:
            with open(p, encoding='utf-8', errors='replace') as f:
                lines = f.read().split('\n')
        except Exception as e:
            sys.stderr.write('读不到 %s：%r\n' % (p, e))
            continue
        total_lines += len(lines)
        for i, words, verdict in audit(lines):
            cats[verdict] = cats.get(verdict, 0) + 1
            if verdict == u'报':
                reported.append((p, i + 1, words, lines[i].strip()[:120]))
    return total_lines, reported, cats


def face_paths():
    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    env = os.environ.get('CFA_FACE', '')
    fp = env if env else FACE_DEFAULT
    if not os.path.isfile(fp):
        return None, fp
    ps = []
    with open(fp, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line if os.path.isabs(line) else os.path.join(root, line)
            if os.path.isfile(p):
                ps.append(p)
    return ps, fp


def selftest():
    import tempfile
    os.environ.pop('CFA_SKIP_LINES', None)
    T = tempfile.mkdtemp(prefix='cfa_selftest_')
    pos = os.path.join(T, 'pos.md')
    neg = os.path.join(T, 'neg.md')
    meta = os.path.join(T, 'meta.md')
    with open(pos, 'w', encoding='utf-8') as f:
        f.write(u'# 桩\n\n\n\n\n\n\n\n\n正文一行：该结论已证。\n另一行：这事被实证推翻。\n')
    with open(neg, 'w', encoding='utf-8') as f:
        f.write(u'# 桩\n\n\n\n\n\n\n\n\n正文一行：该结论已证（`restored=True`／rc=0）。\n')
    with open(meta, 'w', encoding='utf-8') as f:
        f.write(u'# 桩\n\n\n\n\n\n\n\n\n规矩⑨：凡「已证／未证／坐实」级结论，同句须给字段名＝字段值；给不出不得写。\n')
    cases = []
    a = run([pos])[1]
    cases.append((u'例一 阳性对照：同句无痕迹必须报出（桩里两行）', len(a) == 2, len(a)))
    b = run([neg])[1]
    cases.append((u'例二 阴性对照：同句有 restored=True／rc=0 必须不报', len(b) == 0, len(b)))
    c = run([meta])[1]
    cases.append((u'例三 元话语过滤：提到词不等于下结论，必须不报', len(c) == 0, len(c)))
    os.environ['CFA_META'] = '0'
    os.environ['CFA_NEIGH'] = '0'
    d = run([meta, neg])[1]
    del os.environ['CFA_META']
    del os.environ['CFA_NEIGH']
    cases.append((u'例四 撤掉降噪必须变多（证明那两条在起作用，不是装饰）', len(d) > len(c) + len(b), len(d)))
    e_paths, e_fp = face_paths()
    cases.append((u'例五 清单文件必须在位且面非空（缺＝前置不成立）',
                  bool(e_paths) and len(e_paths) > 0, 0 if not e_paths else len(e_paths)))
    npass = 0
    for name, ok, got in cases:
        npass += 1 if ok else 0
        print('[%s] %s（实得 %s）' % (u'PASS' if ok else u'FAIL', name, got))
    import shutil
    shutil.rmtree(T, ignore_errors=True)
    print(u'CLAIMAUDIT SELFTEST VERDICT: PASS=%d/%d（五例，每例都必须能单独失败）' % (npass, len(cases)))
    return 0 if npass == len(cases) else 9


def main(argv):
    if '--selftest' in argv:
        return selftest()
    if len(argv) > 1:
        paths = argv[1:]
        src = u'命令行给定'
    else:
        paths, fp = face_paths()
        src = u'清单=%s' % fp
        if not paths:
            print(u'CLAIMAUDIT VERDICT: rc=9 —— 面清单读不到或为空（缺省＝%s）；**空集不得判绿**' % fp)
            return 9
    total_lines, reported, cats = run(paths)
    print(u'面＝%s 件／文本行数=%s（缺省降噪三条＝跳头部 %s 行＋元话语过滤＋邻行±1）'
          % (len(paths), total_lines, head_skip()))
    print(u'[分类] %s' % u'／'.join(u'%s=%s' % (k, v) for k, v in sorted(cats.items())))
    print(u'[计数] 含结论强度词行=%s 其中**须人读**=%s' % (sum(cats.values()), len(reported)))
    for p, ln, words, snippet in reported:
        print(u'  [报] %s:%s｜词=%s｜%s' % (p, ln, u','.join(words), snippet))
    print(u'CLAIMAUDIT VERDICT: rc=0（**本件只报不拦**，报了行也返 0；9＝前置缺失。入门与阈值归 #12／#19／A32）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
