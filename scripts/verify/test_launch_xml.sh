#!/usr/bin/env bash
# 全部 .launch 的 XML 良构性检查（防"注释里写 -- 导致整份 launch 解析失败"这类事故）。
#
# 由来（2026-11-25，我自己踩的）：在 m1_gazebo.launch 的注释里写了 `--no-imu`，
# XML 规定注释内不得出现连续两个短横线 ⇒ roslaunch 直接报
# `RLException: Invalid roslaunch XML syntax: not well-formed`，整份 launch 连解析都过不去，
# 表现为"gzserver 没起、/clock 不出"，很容易被误判成显示或插件问题（我确实又误判了一轮）。
#
# 判据：每个 .launch 必须能被 XML 解析器解析；解析失败即非零退出。
set -uo pipefail
ROOT="${1:-/opt/ros1-slam}"
bad=0; n=0
while IFS= read -r f; do
  n=$((n+1))
  if ! python3 -c "import sys,xml.dom.minidom as m; m.parse(sys.argv[1])" "$f" 2>/tmp/xmlerr.txt; then
    echo "[FAIL] XML 不良构：$f"
    head -2 /tmp/xmlerr.txt | sed 's/^/        /'
    bad=$((bad+1))
  fi
done < <(find "$ROOT/ws/src" "$ROOT/subprojects" -name '*.launch' -not -path '*/archive/*' 2>/dev/null)
echo "[launch-xml] 检查 $n 份，失败 $bad 份"
[ "$bad" = "0" ] || exit 1
