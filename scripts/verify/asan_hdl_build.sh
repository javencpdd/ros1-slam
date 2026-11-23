#!/usr/bin/env bash
# 用途：把 hdl_localization_nodelet 编成 ASAN 版，产物落在 devel/.asan/，**完全不触碰 devel/lib**。
#
# 为什么需要这个脚本：hdl_localization 的 SIGSEGV 只能在崩溃现场取证，而本机容器内无 gdb、
# ulimit -c=0，ASAN 是唯一可行路线。历史上为了加载 ASAN 版直接 cp 覆盖 devel/lib 的 .so，
# 事后忘记还原导致「正常版」长期被污染（2026-11-24 查实：devel/lib 里 52 MB、__asan 符号 35 个）。
# 本脚本改用 devel/.asan/ 独立目录，运行时靠 LD_LIBRARY_PATH 前置注入，正式库全程只读。
#
# 编译与链接都交给 python 执行（不从 shell 传参）：flags.make 里的 CXX_DEFINES 含
# -DROS_PACKAGE_NAME=\"hdl_localization\"，经 shell 展开会丢引号；且 CXX_INCLUDES 里没有
# /opt/ros/noetic/include，必须显式补上，否则 ros/ros.h 找不到。
#
# 判据（调用方必须核验，脚本不自我宣告）：
#   1) 产物存在且 `nm <so> | grep -c __asan` > 0
#   2) devel/lib/libhdl_localization_nodelet.so 的 sha256 前后不变
set -euo pipefail

WS=${WS:-/opt/ros1-slam/ws}
B="$WS/build/20_localization/hdl_localization/CMakeFiles/hdl_localization_nodelet.dir"
SRC="$WS/src/20_localization/hdl_localization"
# MODE=asan：-O1 -g -fsanitize=address，产物 devel/.asan/
# MODE=dbg ：-O1 -g 不带 sanitizer，产物 devel/.dbg/
#   dbg 是主线：把 libasan 预加载进「未用 ASAN 编译」的 nodelet manager 会让它单线程自旋、
#   XMLRPC 完全不响应（/proc/<pid>/status 显示 State: R、Threads: 1），load 请求静默挂死。
#   而普通 -g 库没有这个冲突，配合 catchsegv(libSegFault) 就能拿到带地址的回溯，
#   再用 addr2line 解行号，全程不需要 gdb。
MODE=${MODE:-dbg}
case "$MODE" in
  asan) OUT=${OUT:-$WS/devel/.asan}; SAN_FLAG=-fsanitize=address ;;
  dbg)  OUT=${OUT:-$WS/devel/.dbg};  SAN_FLAG= ;;
  *) echo "FATAL: MODE 只能是 asan 或 dbg" >&2; exit 2 ;;
esac
PY=${PY:-/usr/bin/python3}

mkdir -p "$OUT"

"$PY" - "$B/flags.make" "$B/link.txt" "$SRC" "$OUT" "$SAN_FLAG" <<'PYEOF'
import os, shlex, subprocess, sys

flags_path, link_path, src, out, san_flag = sys.argv[1:6]
san = [san_flag] if san_flag else []

raw = open(flags_path).read()
def grab(key):
    for line in raw.splitlines():
        if line.startswith(key + ' ='):
            return line.split('=', 1)[1].strip()
    return ''

defs = shlex.split(grab('CXX_DEFINES'))
incs = shlex.split(grab('CXX_INCLUDES'))
if not defs or not incs:
    sys.exit("FATAL: flags.make 解析为空 (defs=%d incs=%d)" % (len(defs), len(incs)))

# flags.make 缺 ROS 主头路径；不补就 ros/ros.h 找不到
if '-I/opt/ros/noetic/include' not in incs:
    incs.insert(0, '-I/opt/ros/noetic/include')

CFLAGS = ['-std=c++17', '-fopenmp', '-O1', '-g', '-fno-omit-frame-pointer',
          '-fPIC'] + san

targets = [
    (os.path.join(src, 'src/hdl_localization/pose_estimator.cpp'),
     os.path.join(out, 'pose_estimator.o')),
    (os.path.join(src, 'apps/hdl_localization_nodelet.cpp'),
     os.path.join(out, 'hdl_localization_nodelet.o')),
]
for s, o in targets:
    if not os.path.isfile(s):
        sys.exit("FATAL: 缺源文件 %s" % s)
    print("[compile] %s" % os.path.basename(s), flush=True)
    subprocess.check_call(['/usr/bin/c++'] + defs + incs + CFLAGS + ['-c', s, '-o', o])

parts = shlex.split(open(link_path).read().strip())
if not parts:
    sys.exit("FATAL: link.txt 为空")

obj_map = {
    'CMakeFiles/hdl_localization_nodelet.dir/src/hdl_localization/pose_estimator.cpp.o':
        targets[0][1],
    'CMakeFiles/hdl_localization_nodelet.dir/apps/hdl_localization_nodelet.cpp.o':
        targets[1][1],
}
so_path = os.path.join(out, 'libhdl_localization_nodelet.so')
new, i, saw_o = [], 0, False
while i < len(parts):
    p = parts[i]
    if p == '-o':
        new += ['-o', so_path]
        saw_o = True
        i += 2
        continue
    new.append(obj_map.get(p, p))
    i += 1
if not saw_o:
    sys.exit("FATAL: link.txt 里没有 -o，无法替换输出路径")

for s in san:
    new.insert(1, s)
print("[link] argv=%d -> %s" % (len(new), so_path), flush=True)
subprocess.check_call(new)
PYEOF

SO="$OUT/libhdl_localization_nodelet.so"
[ -f "$SO" ] || { echo "FATAL: 链接未产出 $SO" >&2; exit 4; }
echo "MODE=$MODE OUT=$SO size=$(stat -c %s "$SO") asan_syms=$(nm "$SO" 2>/dev/null | grep -c __asan || true) hdl_syms=$(nm "$SO" 2>/dev/null | grep -c 'HdlLocalizationNodelet' || true)"
