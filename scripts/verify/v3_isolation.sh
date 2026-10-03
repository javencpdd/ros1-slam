#!/usr/bin/env bash
FAIL=0
TMP=$(mktemp)

say() { echo "$1"; }

say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
if echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc"; then echo "  违规"; FAIL=1; fi

say "[2] 自研壳中是否含 /home/ysc"
if grep -rn "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* >/dev/null 2>&1; then
  grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null; FAIL=1
fi

say "[3] .bashrc 是否 source 了其他 devel"
if grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null; then echo "  违规"; FAIL=1; fi

say "[4] rospack 解析路径是否都在 /home/ros1-slam（teb 例外：系统包）"
for p in faster_lio hdl_localization lite3_bringup lite3_safety; do
  rospack find "$p" 2>/dev/null | grep -q "/home/ros1-slam" || { echo "  BAD: $p"; FAIL=1; }
done

say "[5] ★ 模块间是否互相 include launch（20 号 K3）—— v3 修复子 shell 丢 FAIL bug"
: > "$TMP"
for L in /home/ros1-slam/ws/src/*/lite3_*/launch/*.launch; do
  [ -e "$L" ] || continue
  SELF=$(basename "$(dirname "$(dirname "$L")")")
  grep -o 'find lite3_[a-z_]*' "$L" 2>/dev/null | sed 's/find //;s/)//' | sort -u | \
  while read -r n; do [ "$n" != "$SELF" ] && echo "$L -> $n" >> "$TMP"; done
done
if [ -s "$TMP" ]; then cat "$TMP"; FAIL=1; fi

say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug"
: > "$TMP"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" >> "$TMP" || true
if [ -s "$TMP" ]; then cat "$TMP"; FAIL=1; fi

say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }

say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
if grep -rn "/home/ysc" /home/ros1-slam/ws/src >/dev/null 2>&1; then
  grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null; FAIL=1
fi

rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
exit $FAIL
