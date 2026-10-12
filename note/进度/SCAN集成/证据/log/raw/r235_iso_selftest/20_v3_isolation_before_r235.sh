#!/usr/bin/env bash
FAIL=0
TMP=$(mktemp)
say() { echo "$1"; }
say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc" && { echo "  违规"; FAIL=1; }
say "[2] 自研壳中是否含 /home/ysc"
grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null && FAIL=1
say "[3] .bashrc 是否 source 了其他 devel"
grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null && { echo "  违规"; FAIL=1; }
say "[4] rospack 解析路径是否都在 /home/ros1-slam（teb 例外：系统包）"
for p in faster_lio hdl_localization lite3_bringup lite3_safety; do
  rospack find "$p" 2>/dev/null | grep -q "/home/ros1-slam" || { echo "  BAD: $p"; FAIL=1; }
done
say "[5] ★ 平级模块间是否互相 include launch（20 号 K3）—— P7 修订（主理人裁决，04 台账）：豁免 lite3_bringup（TOP 编排层 = include 合法主体，mode 级组合 launch include 各平级模块属 TOP 职责）；K3 语义 = 平级 lite3_* 模块禁互 include（TOP 下行不豁免）—— v3 修复子 shell 丢 FAIL bug 口径不变 —— SCAN 精度修订（主理人裁决 2026-10-08，04 台账，留底 raw/v3_isolation.sh.before_scanC5.bak）：旧实现 grep -o 'find lite3_*' 会把「rosparam file=\$(find lite3_X)/config/... 加载他包配置」也计为命中，与本段标题及 K3 语义（禁互 include **launch**）不符，实测致 m1_mapping/m2_mapping 两条纯 config 加载假阳性；新实现只匹配 <include file=\"\$(find lite3_X)\" 形态，真 include 命中数不变、反向探针必抓（见 04 登记）。已知局限：跨行书写的 <include\\n file=...> 不在扫描面内（沙箱现存 launch 均单行写法）"
: > "$TMP"
for L in /home/ros1-slam/ws/src/*/lite3_*/launch/*.launch; do
  [ -e "$L" ] || continue
  SELF=$(basename "$(dirname "$(dirname "$L")")")
  [ "$SELF" = "lite3_bringup" ] && continue  # P7 修订：TOP 编排层豁免（语义变化见上方 say 行与 04 台账）
  grep -oE '<include[^>]*file="\$\(find (lite3_[a-z_]*)\)' "$L" 2>/dev/null | sed 's/.*find //;s/)//' | sort -u | \
  while read -r n; do [ "$n" != "$SELF" ] && echo "$L -> $n" >> "$TMP"; done
done
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug；v4 语义注记：lite3_supervisor 亦在扫描范围（SUP 红线：绝不写 /cmd_vel，N5/C13）"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" > "$TMP" || true
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }
say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null && FAIL=1
say "[9] ★★ 40_navigation 壳包禁算法源码（v4 新增，N2/§2.6 硬规则 b：policy 层零 .cpp/.cc/.h）"
HITS=$(find /home/ros1-slam/ws/src/40_navigation \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" \) 2>/dev/null)
[ -n "$HITS" ] && { echo "  命中算法源码文件："; echo "$HITS"; FAIL=1; }
# 自证行（小美 M2R6 路由项 R25 二）：旧版只打结论、无时间与退出码 ⇒ 归档件里「谁在跑、跑出几」
#   要反推（与「跑的是哪份 harness」同族的自证缺口）。日期＋扫描面＋rc 三者入正文，跑完即自解释。
SCOPE_N=$(find /home/ros1-slam/ws/src -type f 2>/dev/null | wc -l)
rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
echo "# self-attest: at=$(date '+%Y-%m-%d %H:%M:%S') script=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}') ws_src_files=$SCOPE_N rc=$FAIL"
exit $FAIL
