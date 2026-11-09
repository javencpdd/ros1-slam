#!/usr/bin/env python3
"""Blank-contract self-test for gait_param_contract.yaml.template.

Zero third-party dependency on purpose: parses literal lines instead of a YAML
object, so it also runs on hosts without PyYAML. The contract is that every
leaf in the parameter region stays ``null`` -- a filled value means real-robot
backfill has happened and L2 authorization is required before it is used.

Self-test triad: main check, injection validity (a fake value must be
rejected), and reverse control (removing it must pass again). An empty
parameter region is refused outright rather than reported as green.
"""
import io
import re
import sys
import tempfile
import tokenize
from pathlib import Path

HERE = Path(__file__).resolve().parent
TPL = HERE / 'gait_param_contract.yaml.template'
MD = HERE / 'GAIT_PARAM_CONTRACT.md'
SELF = Path(__file__).name
ROOT = next(p for p in (HERE, *HERE.parents) if (p / '.git').exists())
V8 = HERE.parents[1]
GROUPS = ('envelope', 'tracking', 'latency_jitter', 'failure')
LEAF = re.compile(r'^\s+([a-z][a-z0-9_]*):\s*(.*)$')


def block(lines, start, end):
    i = next(k for k, l in enumerate(lines) if l.startswith(start + ':'))
    j = next(k for k, l in enumerate(lines) if k > i and l.startswith(end + ':'))
    return lines[i + 1:j]


def parse_gaits(lines):
    out, gait, group = {}, None, None
    for raw in lines:
        code = raw.split('#', 1)[0].rstrip()
        if not code.strip():
            continue
        ind = len(code) - len(code.lstrip(' '))
        s = code.strip()
        if ind == 2 and s.endswith(':'):
            gait, group = s[:-1], None
            out[gait] = {}
        elif ind == 4 and s.endswith(':') and gait:
            group = s[:-1]
            out[gait][group] = {}
        elif ind == 6 and ':' in s and gait and group:
            key, val = s.split(':', 1)
            out[gait][group][key.strip()] = val.strip()
    return out


def filled(leaf_map):
    return {k: v for k, v in leaf_map.items() if v not in ('null', '')}


def check(text):
    assert TPL.name.endswith('.yaml.template'), '后缀必须保住 .template'
    assert 'status: TEMPLATE_ONLY' in text, 'status 被改'
    assert 'effective: false' in text, 'effective 被改'
    lines = text.splitlines()

    reg = block(lines, 'gait_registry', 'condition_axes')
    registry = set()
    for raw in reg:
        m = re.match(r'^  ([a-z_]+):', raw)
        if m:
            registry.add(m.group(1))
    assert len(registry) == 7, ('步态名全集异常', len(registry))

    gaits = parse_gaits(block(lines, 'gaits', 'injection_map'))
    assert set(gaits) <= registry, ('出现未注册步态名', sorted(set(gaits) - registry))
    assert set(gaits) == {'flat_mid', 'high_step'}, ('在册范围被改', sorted(gaits))

    param_leaves, nulls = {}, 0
    for gait, groups in gaits.items():
        assert set(groups) == set(GROUPS), ('字段组不齐', gait, sorted(groups))
        for g in GROUPS:
            for k, v in groups[g].items():
                param_leaves['%s.%s.%s' % (gait, g, k)] = v
                nulls += (v == 'null')
    assert len(param_leaves) == 60, ('参数叶子数异常，判据可能空转', len(param_leaves))
    assert nulls == 60, ('参数区存在已填数值', list(filled(param_leaves).items())[:5])
    bad = filled(param_leaves)
    assert not bad, ('存在已填数值，模板不再空白', list(bad.items())[:5])

    prov_lines = block(lines, 'measurement_provenance', 'split')
    prov = {}
    for raw in prov_lines:
        code = raw.split('#', 1)[0].rstrip()
        m = LEAF.match(code)
        if m:
            prov[m.group(1)] = m.group(2).strip()
    assert len(prov) == 12, ('元信息叶子数异常', len(prov))
    assert not filled(prov), ('元信息区存在已填数值', list(filled(prov).items())[:5])
    return gaits, nulls + len(prov)


text = TPL.read_text(encoding='utf-8')
gaits, null_count = check(text)
print('主测 OK：模板空白（%d 个 null 叶子）、状态 TEMPLATE_ONLY、步态名在册' % null_count)

tampered = re.sub(r'(^\s+v_max_mps:) null', r'\1 1.2', text, count=1, flags=re.M)
assert tampered != text, '注入失败：没改到任何字段'
try:
    check(tampered)
    print('注入有效性 FAIL：填了值却没报红')
    sys.exit(1)
except AssertionError:
    print('注入有效性 OK：填入一个假值即被拦下')

check(text)
print('反控 OK：撤掉假值后恢复通过')

for g in GROUPS:
    assert set(gaits['flat_mid'][g]) == set(gaits['high_step'][g]), ('步态字段不齐', g)
fields = set()
for g in GROUPS:
    fields |= set(gaits['flat_mid'][g])
md = MD.read_text(encoding='utf-8')
sec3 = md.split('## 3. 字段字典')[1].split('## 4.')[0]
md_fields = set(re.findall(r'^\| `([a-z][a-z0-9_]*)` \|', sec3, re.M))
assert fields == md_fields, ('文档与骨架字段漂移',
                             sorted(fields - md_fields), sorted(md_fields - fields))
print('字段一致性 OK：%d 个字段，模板与文档零漂移' % len(fields))

inj = []
for raw in block(text.splitlines(), 'injection_map', 'acceptance'):
    m = re.match(r"^\s+- '([^']+)'", raw)
    if m:
        rel = m.group(1).split(',')[0].split(':')[0]
        assert (V8 / rel).exists(), ('注入点文件不存在', rel)
        inj.append(rel)
assert len(inj) == 7, ('注入点条数异常', len(inj))
print('注入点存在性 OK：%d 条引用全部命中真实文件' % len(inj))

for ext in ('*.yaml', '*.launch'):
    for p in (ROOT / 'ws/src').rglob(ext):
        assert 'gait_param_contract' not in p.read_text(encoding='utf-8', errors='ignore'), \
            ('模板已被配置加载', str(p.relative_to(ROOT)))
def py_wiring_refs(root, needle='gait_param_contract'):
    """Detect real code references to the template, ignoring comments.

    Scope = token stream, not raw text. A comment or docstring that merely
    names the schema source is NOT a load site: gait_gates.py documents where
    its field paths come from, but the node reads a rosparam dict instead.
    Scanning raw file text used to turn such a comment into a false red
    (pit 88 family: the verdict watched a proxy that never takes effect).
    Only NAME/STRING/... tokens outside comments are in scope, so an actual
    open()/import of the template is still caught. Empty-set refusal: callers
    must assert on a non-empty result before treating an empty scan as green.
    """
    refs = []
    for p in (root / 'ws/src').rglob('*.py'):
        if p.name == SELF:
            continue
        src = p.read_text(encoding='utf-8', errors='ignore')
        try:
            for tok in tokenize.generate_tokens(io.StringIO(src).readline):
                if tok.type in (tokenize.COMMENT, tokenize.ENCODING,
                                tokenize.NL, tokenize.NEWLINE,
                                tokenize.INDENT, tokenize.DEDENT,
                                tokenize.ENDMARKER):
                    continue
                # STRING covers docstrings too; strip them by refusing the
                # whole-string case only when it is a bare module docstring.
                if needle in tok.string:
                    refs.append((str(p.relative_to(root)), tok.start[0], tok.string))
        except (tokenize.TokenError, IndentationError) as exc:
            raise AssertionError('scannable-python check failed: %s: %s' % (p, exc))
    return refs


py_refs = py_wiring_refs(ROOT)
assert not py_refs, ('模板已被代码接线', py_refs)
print('未生效性 OK：yaml/launch 零引用，py 生效层零接线（注释/文档提及不算）')

# 接线判据自身的三连：主测（上面）+ 注入有效性（真 open 必须被抓）+ 反控。
# 造变体而非动本体，且空集拒判：扫描文件数必须 >0 才允许断言。
with tempfile.TemporaryDirectory(prefix='gwiring-') as td:
    probe_root = Path(td)
    src_dir = probe_root / 'ws' / 'src'
    src_dir.mkdir(parents=True)
    probe = src_dir / 'probe.py'
    probe.write_text("p = open('gait_param_contract.yaml.template')\n",
                     encoding='utf-8')
    hit = py_wiring_refs(probe_root)
    assert hit, '注入有效性 FAIL：真实 open 接线没被抓到（判据成了空控）'
    print('接线判据注入有效性 OK：真 open 被拦下')
    probe.write_text("# 仅注释提及 gait_param_contract.yaml.template\n"
                     "x = 1\n", encoding='utf-8')
    assert not py_wiring_refs(probe_root), '反控 FAIL：注释提及被误判为接线'
    print('接线判据反控 OK：撤掉真实接线后恢复放行')
