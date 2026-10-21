#!/usr/bin/env bash
# R276 施工侧对小张评审"承重事实"的现跑复验（只读；每行 = 命令 + 实测输出）
# 派生：本脚本输出即 10_recheck_raw.txt；引用规矩 = 版本+节次 / 锚文本+键=值，不写 件名:行号
set -u
cd /home/ros1-slam || exit 9
V53="note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md"
V45="note/计划书v4/22_ros1-slam隔离部署计划书-v4.5-落册候选稿（并v4.3+v4.4，X1X2X5待复核不落）.md"
V41="note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md"
SLH="/opt/ros/noetic/include/costmap_2d/static_layer.h"
DRM="ws/src/30_perception/deeprobotics_local_height_map"

echo "== date =="; date '+%F %H:%M'
echo
echo "== R1 static_layer.h 内 combination_method 命中数 =="
echo "\$ grep -c combination_method $SLH"; grep -c combination_method "$SLH"; echo "  rc=$?"
echo
echo "== R2 v5.3 全文 elevation_grid 命中数 =="
echo "\$ grep -c elevation_grid <v5.3>"; grep -c elevation_grid "$V53"
echo
echo "== R3 v5.3 全文 blind 命中数 =="
echo "\$ grep -c blind <v5.3>"; grep -c blind "$V53"
echo "-- R3b 近场过滤行原文（锚文本：§0.12 表内『近场过滤』那一行）--"
grep -o "近场过滤.\{0,150\}" "$V53" | head -1
echo
echo "== R4 v5.3 生效序句原文（锚文本：§0.6『生效序（写死…）』那句）=="
grep -o "生效序（写死[^*]*" "$V53" | head -1
echo
echo "== R5 v5.3 是否存在 §0.11/§0.12/§0.13 三章标题 =="
for s in 0.11 0.12 0.13; do printf '  §%s 章标题命中数=%s\n' "$s" "$(grep -cE "^##+ .*§?$s " "$V53")"; done
echo
echo "== R6 §16.9 的 X1/X2 行现文字（锚文本：§16.9 二期路线图那节的 X1/X2 两条）=="
awk '/^### §16\.9/,/^### §16\.10/' "$V53" | grep -o "^- \*\*X[12][^|]\{0,120\}" | sed 's/^/  /'
echo
echo "== R7 v5.3 内『出处类别』『未标定施工值』通则句命中数 =="
echo -n "  出处类别="; grep -c "出处类别" "$V53"
echo -n "  未标定施工值|未标定的施工值="; grep -cE "未标定施工值|未标定的施工值" "$V53"
echo
echo "== R8 高程包文件计数（小张 X5 前提复核）=="
echo "\$ find $DRM -type f | wc -l"; find "$DRM" -type f | wc -l
echo -n "  其中 .cpp/.h 数="; find "$DRM" -type f \( -name '*.cpp' -o -name '*.h' \) | wc -l
echo -n "  CATKIN_IGNORE 在位="; [ -f "$DRM/CATKIN_IGNORE" ] && echo yes || echo no
echo
echo "== R9 slope_threshold 0.577 命中数（全仓 yaml）=="
echo -n "  slope_threshold: 0.577 命中="; grep -rl "slope_threshold: 0\.577" ws/src 2>/dev/null | wc -l
echo -n "  slope_method: 1 命中文件数="; grep -rl "slope_method: 1" ws/src 2>/dev/null | wc -l
echo
echo "== R10 v4.1 与 v4.3 的 X 编号是否同号不同事 =="
echo -n "  v4.1 内 X 编号种类数="; grep -oE "\*\*X[0-9]+" "$V41" | sort -u | wc -l
echo -n "  note/ 内『同号不同事』命中文件数（本批前应为 0）="; grep -rl "同号不同事" note/ 2>/dev/null | wc -l
echo
echo "== R11 v4.5 候选稿 Part 5.1/5.2 是否已成文（判『重复』的依据）=="
echo -n "  Part 5.1 内 use_maximum 成文="; awk '/^### 5\.1/,/^### 5\.2/' "$V45" | grep -c "use_maximum"
echo -n "  Part 5.1 内 combination_method 成文="; awk '/^### 5\.1/,/^### 5\.2/' "$V45" | grep -c "combination_method"
echo -n "  Part 4.2 内 三值成文="; awk '/^### 4\.2/,/^### 4\.3/' "$V45" | grep -c "三值"
echo -n "  Part 7 内『剩下的随 1 一起落』="; grep -c "剩下的随 1 一起落" "$V45"
echo
echo "== R12 load1（本批零窗，仅记环境态）=="
cat /proc/loadavg
echo "== end =="
