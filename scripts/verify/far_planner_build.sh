#!/usr/bin/env bash
# 用途：把 FAR 的 `far_planner` 编成**带调试符号**的独立二进制，落到 devel/.dbg/，
#       **完全不触碰** devel/lib/far_planner（正式产物只读）。
#
# 为什么复用这套而不是装 gdb：2026-11-24 查 HDL SIGSEGV 时已经建立过同一条路线——
# `-O1 -g` 库 + catchsegv(libSegFault) 拿回溯 + addr2line 解行号，全程不需要 gdb。
# 本脚本是它在 FAR（可执行文件，多源）上的对应实现。
#
# 判据（调用方核验，脚本不自我宣告）：
#   1) 产物存在且 `readelf -S <bin> | grep -c .debug_info` > 0
#   2) 正式产物 devel/lib/far_planner/far_planner 的 sha256 前后不变
set -euo pipefail

OVERLAY=${OVERLAY:-/opt/ros1-slam/tooling/far_overlay}
B="$OVERLAY/build/far_planner/CMakeFiles/far_planner.dir"
OUT=${OUT:-$OVERLAY/devel/.dbg}
OFFICIAL="$OVERLAY/devel/lib/far_planner/far_planner"

mkdir -p "$OUT"
[ -f "$OFFICIAL" ] || { echo "FATAL: 正式产物不存在 $OFFICIAL" >&2; exit 2; }
BEFORE=$(sha256sum "$OFFICIAL" | cut -d' ' -f1)

/usr/bin/python3 - "$B" "$OUT" "$OVERLAY/src/far_planner" <<'PYEOF'
import os
import shlex
import subprocess
import sys

build, out, pkg_src = sys.argv[1], sys.argv[2], sys.argv[3]
flags = {}
for line in open(os.path.join(build, 'flags.make'), encoding='utf-8'):
    if '=' in line and line.split('=')[0].strip().startswith('CXX_'):
        key, _, value = line.partition('=')
        flags[key.strip()] = value.strip()

defines = flags.get('CXX_DEFINES', '')
includes = flags.get('CXX_INCLUDES', '')
common = shlex.split(defines) + shlex.split(includes) + ['-O1', '-g', '-fno-omit-frame-pointer']

link_line = open(os.path.join(build, 'link.txt'), encoding='utf-8').read().strip()
link = shlex.split(link_line)
objs, libs = [], []
for tok in link[1:]:
    if tok.endswith('.o'):
        objs.append(tok)
    else:
        libs.append(tok)

linked = []
for obj in objs:
    src = obj.replace('.dir/', '.dir/')
    # 目标文件按 CMake 的相对路径（CMakeFiles/.../src/x.cpp.o）→ 推导源码 src/x.cpp
    rel = obj.split('.dir/', 1)[1] if '.dir/' in obj else obj
    # CMake 的相对路径是相对**包根**（src/far_planner.cpp），不是相对 build 目录
    source = os.path.join(pkg_src, rel[:-2])
    cand = [source]
    source = next((c for c in cand if os.path.isfile(c)), None)
    if source is None:
        raise SystemExit('找不到源码：%s（试过 %s）' % (rel, cand))
    out_obj = os.path.join(out, os.path.basename(rel).replace('.cpp', '.o'))
    cmd = ['/usr/bin/c++'] + common + ['-c', source, '-o', out_obj]
    print('[cc] %s' % ' '.join(cmd[:3]), flush=True)
    subprocess.run(cmd, check=True)
    linked.append(out_obj)

binary = os.path.join(out, 'far_planner_dbg')
link_cmd = ['/usr/bin/c++', '-rdynamic', '-g'] + linked + libs + ['-o', binary]
print('[ld] %d objects -> %s' % (len(linked), binary), flush=True)
subprocess.run(link_cmd, check=True)
print('BUILT %s' % binary)
PYEOF

AFTER=$(sha256sum "$OFFICIAL" | cut -d' ' -f1)
DBG="$OUT/far_planner_dbg"
SECT=$(readelf -S "$DBG" 2>/dev/null | grep -c '\.debug_info' || true)
echo "MODE=dbg OUT=$OUT size=$(stat -c %s "$DBG" 2>/dev/null || echo 0) debug_info_sections=$SECT"
echo "official_sha_before=$BEFORE"
echo "official_sha_after =$AFTER"
[ "$BEFORE" = "$AFTER" ] || { echo "FATAL: 正式产物被改动！" >&2; exit 3; }
[ "$SECT" -gt 0 ] || { echo "FATAL: 产物没有 .debug_info" >&2; exit 4; }
echo "OK: 调试符号版本 $DBG（正式产物未动）"
