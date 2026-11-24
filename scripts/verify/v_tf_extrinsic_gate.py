#!/usr/bin/env python3
"""真机 TF 边（默认 base_link→rslidar）的**唯一发布者**与**有效外参**门禁。

为什么需要它：`base_link→rslidar` 在真机栈里有两个可能的发布源——驱动侧
`01_sensors.launch`（经 `msg_MID360s.launch` 的 static_transform_publisher，参数是恒等）
与 `lite3_localization/launch/hdl_localization.launch`（外参已改必填）。两个都发就是重复
TF 边；只发恒等则等于"雷达装在 base_link 原点"，整条定位与感知偏 0.38 m 量级却毫无报错。

判据（三条，缺一即不算通过）：
  1. 该 TF 边在给定入口（递归展开 include 后）**恰好有一个**发布者；
  2. 该发布者不是恒等占位（x=y=z=0 视为未标定）；
  3. 若给出 `--attestation`，其记录值须与 launch 现值一致（±1e-6），否则视为
     "记录与实配不符"——这条防的是"标定过但装错/改错"。

★不提供数值★：本件只判"有没有标定、是不是唯一"，数值由 G8 现场实测后写入
launch 或本机覆盖件。未知一律 fail-closed（rc=5）。

用法：
  v_tf_extrinsic_gate.py --entry ws/src/70_bringup/lite3_bringup/launch/mode_navigation.launch
  v_tf_extrinsic_gate.py --entry ... --attestation data/calib/rslidar_extrinsic.json --json
rc：0 通过／5 不通过／9 前置缺失
"""
import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

FIND = re.compile(r'\$\(find ([^)]+)\)')
ARG = re.compile(r'\$\(arg ([^)]+)\)')
IDENTITY_EPS = 1e-9
MATCH_EPS = 1e-6


def package_roots(repo):
    """包名 → 包目录（仓内 ws/src 下，按 package.xml 定位）。"""
    roots = {}
    for dirpath, _dirs, files in os.walk(os.path.join(repo, 'ws/src')):
        if 'package.xml' in files:
            roots[os.path.basename(dirpath)] = dirpath
    return roots


def resolve_include(path_text, roots):
    """`$(find pkg)/launch/x.launch` → 绝对路径；解析不出来返回 None（由调用方记不可判定）。"""
    m = FIND.search(path_text or '')
    if not m:
        return None
    pkg = m.group(1)
    root = roots.get(pkg)
    if root is None:
        return None
    rest = path_text[m.end():].lstrip('/')
    return os.path.join(root, rest)


def parse_tf_args(args):
    """把 static_transform_publisher 的 args 折成 dict；解析不了返回 None（不可判定）。

    支持两种形态（Noetic tf2 都收）：
      8 段：x y z yaw pitch roll parent child
      9 段：x y z qx qy qz qw parent child
    """
    if not args:
        return None
    tok = args.split()
    if len(tok) not in (8, 9):
        return None
    nums, frames = tok[:-2], tok[-2:]
    try:
        vals = [float(x) for x in nums]
    except ValueError:
        return None          # 含 $(arg ...) 之类未展开的东西 ⇒ 不可判定
    return {'xyz': vals[:3], 'raw': vals, 'parent': frames[0].lstrip('/'),
            'child': frames[1].lstrip('/'), 'form': 'euler' if len(tok) == 8 else 'quat'}


def subst(text, argvals):
    """展开 $(arg X)；未知的保持原样（由调用方判不可判定）。

    ★None 必须原样返回 None★：属性缺失（没写 if/unless）若被折成空串，
    `truth('')` 会判成"假"，把整个节点当条件关闭悄悄跳过 —— 实测这条让驱动侧的
    base2livox 从发布者名单里消失，门禁误报"发布者数为 0"。
    """
    if text is None:
        return None

    def rep(m):
        v = argvals.get(m.group(1))
        return v if v is not None else m.group(0)
    return ARG.sub(rep, text or '')


def truth(text):
    """roslaunch 的 if/unless 取值：'false'/'0'/'' 为假；其余为真；无法判定返回 None。"""
    if text is None:
        return True
    if '$(' in text:
        return None
    return text.strip().lower() not in ('false', '0', '')


def walk(launch_path, roots, edge, args=None, seen=None, depth=0, out=None):
    """递归展开 include（带 arg 传播），收集该 TF 边的候选发布者。

    arg 传播不是锦上添花：真机两个入口给 hdl launch 传 `publish_extrinsic:=false`，
    不传播就会把那条被 `if` 关掉的声明当成"不可判定"，从而把已正确的入口判红。
    """
    out = out if out is not None else []
    seen = seen if seen is not None else set()
    args = dict(args or {})
    if depth > 12:
        return out
    real = os.path.realpath(launch_path)
    if real in seen:
        return out
    seen.add(real)
    try:
        tree = ET.parse(launch_path)
    except (ET.ParseError, OSError):
        out.append({'file': launch_path, 'error': 'parse_failed'})
        return out
    root = tree.getroot()
    # 先收本文件自己的 arg 声明（value 优先，其次 default）——include 传进来的优先
    for el in root:
        if el.tag == 'arg':
            name = el.get('name')
            if not name or name in args:
                continue
            for key in ('value', 'default'):
                if el.get(key) is not None:
                    args[name] = subst(el.get(key), args)
                    break
    for el in root:
        gate = truth(subst(el.get('if'), args))
        if el.get('unless') is not None:
            g2 = truth(subst(el.get('unless'), args))
            gate = None if (gate is None or g2 is None) else (gate and not g2)
        if gate is False:
            continue
        if el.tag == 'include':
            nxt = resolve_include(subst(el.get('file'), args), roots)
            if nxt is None:
                out.append({'file': launch_path, 'include': el.get('file'),
                            'error': 'include_unresolved'})
                continue
            child = {}
            for a in el.findall('arg'):
                if a.get('name'):
                    child[a.get('name')] = subst(a.get('value'), args)
            walk(nxt, roots, edge, child, seen, depth + 1, out)
        elif el.tag == 'node':
            typ = (el.get('type') or '').split('/')[-1]
            if typ != 'static_transform_publisher':
                continue
            if gate is None:
                out.append({'file': launch_path, 'node': el.get('name'),
                            'error': 'node_if_undecidable'})
                continue
            spec = parse_tf_args(subst(el.get('args'), args))
            if spec is None:
                out.append({'file': launch_path, 'node': el.get('name'),
                            'error': 'args_unparsed',
                            'args': (el.get('args') or '')[:120]})
                continue
            if (spec['parent'], spec['child']) != edge:
                continue
            spec.update({'file': launch_path, 'node': el.get('name'),
                         'identity': all(abs(v) < IDENTITY_EPS for v in spec['xyz'])})
            out.append(spec)
    return out


def load_attestation(path):
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


def audit(entry, roots, edge, attestation=None):
    """返回 (ok, 明细)。未知状态一律判不通过（fail-closed）。"""
    pubs = walk(entry, roots, edge)
    bad_unknown = [p for p in pubs if 'error' in p]
    real = [p for p in pubs if 'error' not in p]
    d = {'entry': entry, 'edge': '%s→%s' % edge, 'ok': False,
         'publishers': [{'node': p.get('node'), 'file': p.get('file'),
                         'xyz': [round(v, 6) for v in p['xyz']],
                         'identity': p['identity']} for p in real],
         'unknown': bad_unknown, 'attested': None}
    if bad_unknown:
        d['reason'] = '有不可判定的 TF 声明（args 含未展开变量或 include 解析失败）'
        return False, d
    if len(real) != 1:
        d['reason'] = '该 TF 边的发布者数为 %d（要求恰好 1）' % len(real)
        return False, d
    p = real[0]
    if p['identity']:
        d['reason'] = '唯一发布者是恒等占位（x=y=z=0）⇒ 未标定，禁止进入真机导航'
        return False, d
    if attestation is not None:
        got = attestation.get('xyz') or attestation.get('translation')
        if not got or len(got) < 3:
            d['reason'] = '标定记录缺少 xyz'
            return False, d
        if any(abs(float(got[i]) - p['xyz'][i]) > MATCH_EPS for i in range(3)):
            d['reason'] = '标定记录 %s 与 launch 现值 %s 不一致' % (
                [round(float(x), 6) for x in got[:3]], [round(v, 6) for v in p['xyz']])
            return False, d
        d['attested'] = {k: attestation.get(k) for k in
                         ('measured_at', 'measured_by', 'method') if attestation.get(k)}
    d['reason'] = '唯一发布者且非恒等' + ('（与标定记录一致）' if attestation else '（未对标定记录）')
    d['ok'] = True
    return True, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    ap.add_argument('--entry', action='append', default=[],
                    help='入口 launch（可重复）；缺省查真机两个入口')
    ap.add_argument('--edge', default='base_link:rslidar')
    ap.add_argument('--attestation', default='')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    repo = os.path.abspath(a.repo)
    roots = package_roots(repo)
    entries = a.entry or [
        os.path.join(repo, 'ws/src/70_bringup/lite3_bringup/launch/mode_navigation.launch'),
        os.path.join(repo, 'ws/src/70_bringup/lite3_bringup/launch/mode_localize.launch'),
    ]
    missing = [e for e in entries if not os.path.isfile(e)]
    if missing or not roots:
        print('前置缺失：%s' % (missing or 'package_roots 为空'))
        return 9
    att = None
    if a.attestation:
        if not os.path.isfile(a.attestation):
            print('前置缺失：标定记录不存在 %s' % a.attestation)
            return 9
        att = load_attestation(a.attestation)

    parent, child = a.edge.split(':')
    rows, ok_all = [], True
    for e in entries:
        ok, d = audit(e, roots, (parent, child), att)
        ok_all = ok_all and ok
        rows.append(d)
    if a.json:
        print(json.dumps({'ok': ok_all, 'edge': a.edge, 'rows': rows}, ensure_ascii=False))
    else:
        for d in rows:
            print('[%s] %s' % ('OK' if d['ok'] else 'NG', os.path.basename(d['entry'])))
            print('  该边发布者 %d：%s' % (len(d['publishers']), d['publishers'] or '无'))
            for u in d['unknown']:
                print('  不可判定：%s' % u)
            print('  结论：%s' % d.get('reason'))
        print('TFGATE VERDICT: %s（%s）' % ('PASS' if ok_all else 'FAIL', a.edge))
    return 0 if ok_all else 5


if __name__ == '__main__':
    sys.exit(main())
