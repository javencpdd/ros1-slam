#!/usr/bin/env python3
"""第三方完整性基线的**覆盖面**审计：哪些顶层包没进基线、登记件是否与其一一对应。

为什么要单独一件：原判据把「30_3rd 全部文件数 = 49」当成完整性条件，那是拿树大小冒充
覆盖面。基线只覆盖 2026-10-08 的 SCAN 五包 49 件；之后整包接入 elevation_mapping 与
message_logger（0fb03fa9），树长到 139 件，该条件恒红且不构成任何完整性结论。

本件的判据只有一条：**未纳入基线的顶层包集合 == 登记件里的集合**（双向，多登记也算错）。
完整性（基线 49 件逐字节全等）仍由调用方用 sha256sum -c 单独判，不经本件。

用法：
  third_party_coverage.py --tree ws/src/30_3rd \\
      --baseline note/进度/v7/S1/SCAN集成/证据/scan_src_baseline_49.sha256 \\
      --registry scripts/verify/third_party_unbaselined.tsv
  rc=0 对表一致；rc=5 不一致；rc=9 前置缺失
"""
import argparse
import os
import sys

HEADER_FIRST = '目录'


def covered_dirs(baseline_path):
    """基线里出现的顶层目录集合（sha256 文件的第 2 列是相对 30_3rd 的路径）。"""
    out = set()
    with open(baseline_path, encoding='utf-8-sig', errors='ignore') as fh:
        for line in fh:
            parts = line.split()
            if len(parts) >= 2 and '/' in parts[1]:
                out.add(parts[1].split('/')[0])
    return out


def present_dirs(tree):
    return {d for d in os.listdir(tree) if os.path.isdir(os.path.join(tree, d))}


def registered_dirs(registry_path):
    out = set()
    if not os.path.isfile(registry_path):
        return out
    with open(registry_path, encoding='utf-8-sig', errors='ignore') as fh:
        for line in fh:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            first = line.split('\t')[0].strip()
            if first == HEADER_FIRST:
                continue
            if first:
                out.add(first)
    return out


def audit(tree, baseline, registry):
    """返回 (ok, 明细 dict)。ok 只在"双向相等"时为真。"""
    if not os.path.isdir(tree):
        raise FileNotFoundError(tree)
    if not os.path.isfile(baseline):
        raise FileNotFoundError(baseline)
    covered = covered_dirs(baseline)
    present = present_dirs(tree)
    unreg = sorted(present - covered)
    registered = registered_dirs(registry)
    extra = sorted(registered - set(unreg))
    ok = set(unreg) == registered
    return ok, {'未纳入': unreg, '登记': sorted(registered),
                '多登记': extra, '已覆盖': sorted(covered & present)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tree', required=True)
    ap.add_argument('--baseline', required=True)
    ap.add_argument('--registry', required=True)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    try:
        ok, d = audit(a.tree, a.baseline, a.registry)
    except FileNotFoundError as exc:
        print('前置缺失：%s' % exc)
        return 9
    if not a.quiet:
        print('未纳入=%d[%s] 登记=%d[%s] 多登记=%s' % (
            len(d['未纳入']), ','.join(d['未纳入']), len(d['登记']),
            ','.join(d['登记']), ','.join(d['多登记']) or '无'))
    print('OK' if ok else 'BAD')
    return 0 if ok else 5


if __name__ == '__main__':
    sys.exit(main())
