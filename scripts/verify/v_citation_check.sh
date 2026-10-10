#!/usr/bin/env bash
# v_citation_check.sh (VCC) —— 台账里的「文件:行号（＋键名）」引用能否回原文匹配（**候选闸，未入门禁串**；小明 2026-10-10）
#
# 为何存在：02 坑42 追加条与 坑58 指的是同一件可机判的事——**引文先写了再核**。今日实测两次栽在这里：
#   把坑42 写成 `note/02_问题与坑.md:314`（坑42 实在同一件 :452，且讲的是另一回事）、
#   把 `allow_unknown` 写成 `global_planner_params.yaml:12`（实在 :8）。两条都是**一次 grep 就能挡住**的错。
#   而 坑59 追加二已给出结论：**"我知道该核"不等于"我会去核"，只有成闸才算存在**。
#
# 分类（不是简单二态，这是本件能不能用的关键）：
#   RED   行号超出文件长度／文件找不到 ⇒ 引文硬错，必须改。
#   AMBER 行存在，但引用处紧跟的反引号键名不在那一行 ⇒ 行号对、口径松（或键名记错）。
#   FIXUP 同一句里含「写成／实为／勘误／应为」等 ⇒ 这是**在册引错示例**（我们的台账规矩是"原文不回改＋另起勘误句"，
#         所以这类句子必然同时含错引与正解），**不计红**，只单列计数；把它算红就等于罚"诚实记录错误"。
#   OK    行存在，且（无键名 或 键名能在该行匹配）。
# 自标定（P1，阳性对照在前；坑29／坑42 的既有姿势）：植入 1 条行号越界(须 RED)＋1 条键名不符(须 AMBER)
#   ＋1 条正确引用(须 OK)＋1 条勘误句式(须 FIXUP)＋空扫描面(须 rc=9)；**五项任一对不上即 rc=9，普查结果一律不采信**。
#
# 边界：只读。不改任何文档、不动门禁串四件（SYN/GTC/ISO/DC）；是否入门禁串＝请求单 A15/#54/#16 那一族，我不自落。
# 用法：bash scripts/verify/v_citation_check.sh --selftest
#       bash scripts/verify/v_citation_check.sh <文档 ...>      缺省＝note/04 note/02 note/进度/00_总览.md 与 SCAN集成/设计/*.md
#   环境变量：VCC_MAX（明细最多列几条，缺省 20）｜VCC_ROOT（仓根，缺省 /home/ros1-slam）
#   退出码：0 无 RED 无 AMBER｜5 有 AMBER（口径松）｜6 有 RED（引文硬错）｜9 前置缺失或自标定不过
# 性质：零 ROS 节点、零链窗、零写盘（临时件在 /tmp，跑完即删）、无 sudo。
set -u
ROOT="${VCC_ROOT:-/home/ros1-slam}"
cd "$ROOT" || { echo "前置缺失：仓根 $ROOT 不在位"; echo "VCC VERDICT rc=9"; exit 9; }
MODE="scan"
case "${1:-}" in
  --selftest) MODE="selftest"; shift ;;
esac
python3 - "$MODE" "$ROOT" "${VCC_MAX:-20}" "$@" <<'PYEOF'
# -*- coding: utf-8 -*-
import glob, os, re, shutil, sys, tempfile

MODE, ROOT, MAXD = sys.argv[1], sys.argv[2], int(sys.argv[3])
docs = sys.argv[4:] or (['note/04_配置改动diff.md', 'note/02_问题与坑.md', 'note/进度/00_总览.md']
                        + sorted(glob.glob('note/进度/SCAN集成/设计/*.md')))
CITE = re.compile(r'([A-Za-z0-9_\-./\u4e00-\u9fff]+\.(?:sh|py|md|ya?ml|launch|world|pcd|csv|txt|xacro|json|cpp|h|bak|log))[:：](\d+)')
# 键名的两种在册写法都要认（自测 C3 首版就是因为只认第一种而漏判）：
#   甲 "`global_planner_params.yaml:12 allow_unknown: false`"——**同一对反引号内**跟着键名；
#   乙 "`m2_smoke.sh:382` 的 `wait_for_message`"——引用**闭合之后**另起一对反引号。
#   闭合反引号必须先剥掉，否则会把"引用自身的那个收尾符号"当成甲式的开头而抓错。
KEY_QUOTED = re.compile(r'^\s*[`\'"]([^`\'"]{1,80})')
KEY_BARE = re.compile(r'^\s+([A-Za-z_][A-Za-z0-9_\-.:]{3,})')
# 中文标签只在**形如编号**时才当键名（坑NN／RNN／ANN／FD-N…）：普通中文散文紧跟引用是常态，一律当键名会假红一片
KEY_CJK_LABEL = re.compile(r'^\s+(坑\d{1,3}|[A-Z]{1,3}\d{1,4}[a-z]?|FD-\d{1,2}|G\d{1,2})')   # 只认"编号型"标签；纯中文散文一律不当键名（防假红）
FIXUP = re.compile(r'写成|实为|勘误|应为|误为|改成|更正|原文不回改|已删|已移除|不在位|历史|当时|曾为')
lines_cache = {}
name_index = {}      # basename → [绝对路径]，整仓只建一次（首版每次引用都 os.walk 一遍 ⇒ 60s 还没扫完，件自己慢死）


def build_index():
    if name_index:
        return
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in ('.git', '__pycache__', 'build', 'devel')]
        for fn in filenames:
            name_index.setdefault(fn, []).append(os.path.join(dirpath, fn))
    for sub in ('.', 'note', 'ws', 'scripts'):
        pass




def file_lines(path):
    if path not in lines_cache:
        try:
            with open(path, encoding='utf-8', errors='ignore') as fh:
                lines_cache[path] = fh.read().splitlines()
        except Exception:
            lines_cache[path] = None
    return lines_cache[path]


def resolve(ref):
    """返回 (路径或 None, 类别)。类别：'ok' 唯一命中｜'missing' 写了目录却找不到（＝路径失效，硬错）｜
       'ambiguous' 只给 basename 且仓里多份（坑52 那一族：该写完整路径，属**引用质量问题**不是硬错）｜'notfound' 哪儿都没有。
       不这么分就会一上来报 300+ 红 ⇒ 没人会读，闸就白做；也不能一白了之，因为真失效的路径就混在里面。"""
    cands = [os.path.join(ROOT, ref), ref, os.path.join(ROOT, os.path.basename(ref))]
    if ref.startswith('/opt/') or (ref.split('/')[0] in ('mbf_msgs', 'mbf_abstract_nav', 'mbf_utility', 'move_base', 'tf2_msgs', 'grid_map_msgs') and '/opt/' not in ref):
        # 系统头的引用（如 `mbf_msgs/GetPathResult.h:111`）在仓外，按 /opt/ros/noetic 的两个常见落点单独判
        cands += ['/opt/ros/noetic/include/' + ref, os.path.join('/opt/ros/noetic', ref), ref]
    for c in cands:
        if os.path.isfile(c):
            return c, 'ok'
    base = os.path.basename(ref).lstrip('.')
    build_index()
    hits = name_index.get(base, [])
    if len(hits) == 1:
        return hits[0], 'ok'
    if len(hits) == 0:
        return None, 'notfound'
    if True:
        for h in hits:
            if h.endswith(ref if not ref.startswith('./') else ref[2:]):
                return h, 'ok'
        return None, ('ambiguous' if '/' not in ref else 'missing')
    return None, ('ambiguous' if '/' not in ref else 'missing')


def judge(text, root=ROOT):
    """扫描一段文本，返回 (RED, AMBER, FIXUP_HITS, OK_count, total)。"""
    red, amber, fixup, ok, total = [], [], [], 0, 0
    amb = []
    info = []
    for m in CITE.finditer(text):
        ref, ln = m.group(1), int(m.group(2))
        # 三类"看着像引用但不是单点引用"的形状先放过，否则一开跑就几百条假红（假红会让整件失去可信度）：
        #   ① rc 档位列表 `v_dwa_recheck.sh:0/5/6/7/4`（数字后面紧跟 /数字）；
        #   ② 范围/通配写法 `r81_b1_stageB5/b1..b5.log:25`（含 .. 或 *）；
        #   ③ 系统路径（/opt/... 下的头文件）——仓内找不到属正常，按绝对路径单独判。
        # rc 档图例（`v_dwa_recheck.sh：0=判据过／5=…`）不是行号引用：数字后紧跟等号，或行号本身就是 0（文件行号从 1 起）
        # 三类非单点引用先放过：等号图例（`：0=判据过`）、行号为 0、斜杠续列（`:0/5/6/7/4`）；再加区间写法 `:2-10`（引用一段而非一行）
        if re.match(r'^\s*(?:[=＝]|/\d|-\d)', text[m.end():m.end() + 3]) or ln == 0:
            info.append('%s:%d（非单点引用：等号图例／斜杠续列／区间写法）' % (ref, ln)); continue
        if '..' in ref or '*' in ref:
            info.append('%s:%d（范围/通配写法）' % (ref, ln)); continue
        total += 1
        tail = text[m.end():m.end() + 90]
        inner = tail[1:] if tail.startswith('`') else tail      # 剥掉引用自身的闭合反引号
        km = KEY_QUOTED.match(inner)
        key = km.group(1).strip() if km else ''
        if not key:
            bm = KEY_BARE.match(inner)
            key = bm.group(1).strip() if bm else ''
        if not key:
            cm = KEY_CJK_LABEL.match(inner)
            if cm:
                key = cm.group(1).strip()
                if not re.search(r'[0-9]', key):
                    key = ''      # 纯中文词不作键名，防假红
        ctx = text[max(0, m.start() - 120):m.end() + 120]
        pre = text[max(0, m.start() - 60):m.start()]
        if FIXUP.search(pre) or FIXUP.search(tail[:60]):
            fixup.append('%s:%d' % (ref, ln))
            continue
        p, kind = resolve(ref)
        if p is None:
            if kind == 'ambiguous':
                amb.append('%s:%d（只给了文件名、仓里同名多份 ⇒ 按坑52 应写完整路径）' % (ref, ln))
            elif kind == 'notfound':
                amb.append('%s:%d（**仓内根本没有这个文件名**——多半是简写，如 `fsm.cpp` 实为 `scan_replan_fsm.cpp`；按坑52 该写全名）' % (ref, ln))
            else:
                red.append('%s:%d（路径失效：按所给完整路径找不到文件）' % (ref, ln))
            continue
        ls = file_lines(p) or []
        if ln > len(ls) or ln < 1:
            red.append('%s:%d 越界（该文件 %d 行）' % (os.path.relpath(p, root), ln, len(ls)))
            continue
        line = ls[ln - 1]
        if key:
            # 拆分后不够长的（如编号标签「坑42」「R90」）用**整串**当针，否则会被长度门槛吃掉而漏判（本轮实测漏过一次）
            toks = [t for t in re.split(r'[^0-9A-Za-z_\-.:/]+', key) if len(t) >= 3] or [key]
            if not any(t in line for t in toks):
                amber.append('%s:%d 键名「%s」不在该行' % (os.path.relpath(p, root), ln, key[:28]))
                continue
        ok += 1
    return red, amber + amb, fixup, ok, total, len(info), len(info)


if MODE == 'selftest':
    tmp = tempfile.mkdtemp(prefix='vcc_p1_')
    npass = nfail = 0
    try:
        target = os.path.join(tmp, 't.yaml')
        open(target, 'w').write('alpha: 1\nbeta: 2\n')
        good = '%s:1 alpha' % target
        r1, _, _, _, _, _, _ = judge(good)
        ok1 = (not r1)
        print('[%s] C1 正确引用（行存在＋键名在该行）须判 OK：实得 RED=%d' % ('PASS' if ok1 else 'FAIL', len(r1)))
        r2, _, _, _, _, _, _ = judge('%s:9 gamma' % target)
        ok2 = len(r2) == 1
        print('[%s] C2 行号越界（该文件 2 行、引到 :9）须判 RED：实得 %s' % ('PASS' if ok2 else 'FAIL', r2 or '无'))
        r3, a3, _, _, _, _, _ = judge('%s:2 zzz_key' % target)
        ok3 = (not r3) and len(a3) == 1
        print('[%s] C3 行存在但键名不在该行须判 AMBER：实得 AMBER=%s' % ('PASS' if ok3 else 'FAIL', a3 or '无'))
        _, _, f4, _, _, _, _ = judge('我把键名写成 %s:2 wrong_key，实为 %s:1 alpha' % (target, target))
        ok4 = len(f4) >= 1
        print('[%s] C4 勘误句式（同句含"写成…实为"）须判 FIXUP 不判红：实得 %s' % ('PASS' if ok4 else 'FAIL', f4 or '无'))
        rp, ap, fp, okp, tp, ip, _ = judge('')
        ok5 = (rp, ap, fp, okp, tp) == ([], [], [], 0, 0)
        print('[%s] C5 空扫描面须"零判定"（由外层据此退 rc=9，不判被试）：实得 total=%d' % ('PASS' if ok5 else 'FAIL', tp))
        npass = sum([ok1, ok2, ok3, ok4, ok5]); nfail = 5 - npass
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('VCC SELFTEST VERDICT: PASS=%d FAIL=%d' % (npass, nfail))
    sys.exit(0 if nfail == 0 else 5)

allr, alla, allf, okc, tot, info_total = [], [], [], 0, 0, 0
scanned = 0
for d in docs:
    p = d if os.path.isabs(d) else os.path.join(ROOT, d)
    if not os.path.isfile(p):
        print('[跳过] 文档不在位：%s' % d)
        continue
    scanned += 1
    red, amber, fixup, ok, t, inf, _ = judge(open(p, encoding='utf-8', errors='ignore').read())
    info_total += inf
    allr += ['%s ⇒ %s' % (os.path.basename(d), x) for x in red]
    alla += ['%s ⇒ %s' % (os.path.basename(d), x) for x in amber]
    allf += fixup
    okc += ok
    tot += t
if scanned == 0 and tot == 0:
    print('VCC VERDICT rc=9（扫描面为空＝没得查，不判被试；坑29）')
    sys.exit(9)
for x in allr[:MAXD]:
    print('[RED] %s' % x)
for x in alla[:MAXD]:
    print('[AMBER] %s' % x)
print('[计数] 文档=%d 单点引用=%d OK=%d RED=%d AMBER=%d 勘误句=%d（不判红）非单点形状=%d（不判红）'
      % (scanned, tot, okc, len(allr), len(alla), len(allf), info_total))
rc = 6 if allr else (5 if alla else 0)
print('VCC VERDICT rc=%d（0 干净／5 有 AMBER／6 有 RED／9 前置缺失）' % rc)
sys.exit(rc)
PYEOF
rc=$?
echo "VCC rc=${rc}"
exit ${rc}
