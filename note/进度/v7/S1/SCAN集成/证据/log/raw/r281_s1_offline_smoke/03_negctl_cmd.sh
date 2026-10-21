#!/bin/bash
# R281 负面对照（变异测试）：三条断言各自必须能失败 ⇒ 否则"rc=0"没有意义
# 用法：bash 03_negctl_cmd.sh   （零窗、零开链；变异件写 /tmp，不污染工作根与 PYST 面）
set -u
ROOT=/home/ros1-slam
SRC="$ROOT/note/进度/SCAN集成/设计/scripts/s1_offline_frames_smoke.py"
OUT=${1:-/tmp/negctl_r281.txt}
: > "$OUT"
for k in M1 M2 M3; do
  cp "$SRC" "/tmp/mut_$k.py"
  # 让变异件在 /tmp 也能找到工作根（原件靠向上搜根）
  python3 - "$k" <<'PY'
import io, sys
k = sys.argv[1]
p = "/tmp/mut_%s.py" % k
s = io.open(p, encoding="utf-8").read()
s = s.replace('    root = None\n', '    root = "/home/ros1-slam"    # ← 变异件专用：跳过向上搜根\n    _ = None\n', 1)
s = s.replace('        if os.path.isdir(os.path.join(d, "ws", "src"))', '        if False and os.path.isdir(os.path.join(d, "ws", "src"))', 1)
if k == "M1":
    s = s.replace('run(mod, frames, "B 显式剔零后喂", window, True, tot_all, zeros)',
                  'run(mod, frames, "B 显式剔零后喂", window, False, tot_all, zeros)')   # 剔零失效
if k == "M2":
    s = s.replace('    window = envelope(frames)\n', '    window = (100, 180, 100, 180)  # 变异：窗口平移，原点（占位零点）落窗外\n', 1)
if k == "M3":
    s = s.replace('    frames = load_frames(root)', '    mod.FREE = 7  # 变异：三值契约被破坏\n    frames = load_frames(root)', 1)
io.open(p, "w", encoding="utf-8").write(s)
PY
  echo "==== $k 变异注入处 ====" >> "$OUT"
  diff "$SRC" "/tmp/mut_$k.py" | grep '^[<>]' | grep -v '_ = None\|if False and\|root = "/home' >> "$OUT"
  echo "==== $k 运行 ====" >> "$OUT"
  ( cd "$ROOT" && PYTHONNOUSERSITE=1 python3 "/tmp/mut_$k.py" ) >> "$OUT" 2>&1
  echo "---- $k rc=$? （须≠0 才算该断言真能失败）" >> "$OUT"
  echo >> "$OUT"
done
