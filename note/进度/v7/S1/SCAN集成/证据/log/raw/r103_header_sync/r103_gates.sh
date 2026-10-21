#!/usr/bin/env bash
# R103（00_总览.md 档头同步批）的门禁件：只读巡检，零改动、零链窗、零端口、零 sudo。
# 重放：cd /home/ros1-slam && bash note/进度/SCAN集成/证据/log/raw/r103_header_sync/r103_gates.sh
# rc：0 三闸全过；6 环境件缺失；其余透传首个非 0 闸的 rc。
set -u
ROOT=/home/ros1-slam
SELF=$(readlink -f "$0")
HERE=$(dirname "$SELF")
cd "$ROOT" || exit 6

# 环境前提先落：ISO 的 [4] 依赖 rospack，裸跑（只 source foxy）会把环境缺失常读成被测面红
# —— 见 02 坑册「本轮做法」句（R102 自曝），本件把前提写成代码而不是靠自觉。
[ -r /opt/ros/noetic/setup.bash ] || { echo "NOETIC SETUP MISSING"; exit 6; }
[ -r ws/devel/setup.bash ] || { echo "WS DEVEL SETUP MISSING"; exit 6; }
# shellcheck disable=SC1091
set +u; source /opt/ros/noetic/setup.bash; set -u
# shellcheck disable=SC1091
set +u; source ws/devel/setup.bash; set -u
echo "ENV ROS_DISTRO=${ROS_DISTRO:-unset} rospack=$(command -v rospack || echo NONE)"

bash scripts/verify/v_harness_generation_table.sh > "$HERE/gtc.out" 2>&1
rc_gtc=$?
echo "GTC rc=$rc_gtc H_rows=$(grep -c '^| H' "$HERE/gtc.out") last=$(tail -1 "$HERE/gtc.out")"

bash scripts/verify/v_sh_py_syntax.sh "$SELF" > "$HERE/syn.out" 2>&1
rc_syn=$?
echo "SYN rc=$rc_syn 被检件=1（本件自身；空集不判绿，故不点空集）"

bash scripts/verify/v3_isolation.sh > "$HERE/iso.out" 2>&1
rc_iso=$?
echo "ISO rc=$rc_iso verdict=$(grep -m1 'ISOLATION' "$HERE/iso.out") ws_src_files=$(grep -oE 'ws_src_files[=: ]+[0-9]+' "$HERE/iso.out" | tail -1)"

for r in "$rc_gtc" "$rc_syn" "$rc_iso"; do [ "$r" -ne 0 ] && exit "$r"; done
echo "R103 GATES=PASS"
