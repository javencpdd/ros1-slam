#!/usr/bin/env bash
# R277 四格待裁项（A40–A43）现状实证：施工侧现跑，只读；输出＝10_measures_raw.txt
set -u
cd /home/ros1-slam || exit 9
V53="note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md"
V41="note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md"
V43="note/计划书v4/21_ros1-slam隔离部署计划书-v4.3-楼梯通行与计划书同步纪律补丁.md"
V45="note/计划书v4/22_ros1-slam隔离部署计划书-v4.5-落册候选稿（并v4.3+v4.4，X1X2X5待复核不落）.md"
CC="ws/src/40_navigation/lite3_navigation/config/costmap_common.yaml"
echo "== date =="; date '+%F %H:%M'
echo
echo "== M-A40-1 v5.3 内 elevation_grid 命中数（禁令句是否在正文）=="; grep -c elevation_grid "$V53"
echo "== M-A40-2 v5.3 内 'ws/README.md' 指针句命中数 =="; grep -c "ws/README.md" "$V53"
echo "== M-A40-3 ws/README.md 内 §6.3 锚命中数与件长 =="; echo -n "  6.3 命中="; grep -c "6\.3" ws/README.md; echo -n "  件长行="; wc -l < ws/README.md
echo "== M-A40-4 v4.5 Part 4.1 单点裁定句是否在册 =="; echo -n "  '单点落址' 命中="; grep -c "单点落址" "$V45"
echo
echo "== M-A41-1 v5.3 §4.3 表行数（'加两行'无处可加的实证）=="; awk '/^### 4\.3/,/^## §5/' "$V45" >/dev/null 2>&1; awk '/^### 4\.3/,/^## §5/' "$V53" | grep -c "^|"
echo "== M-A41-2 v5.3 内 '三层仿真能力边界' 命中数（§0.8 那一行）=="; grep -c "三层仿真能力边界" "$V53"
echo "== M-A41-3 '不得优于' 在册性（HEAD 面，小美第⑥句是否净新增）=="; echo -n "  命中件数="; git grep -l "不得优于" HEAD -- note/ | wc -l
echo "== M-A41-4 '出处类别' 与 '未标定施工值' 在 v5.3 内命中数 =="; echo -n "  出处类别="; grep -c "出处类别" "$V53"; echo -n "  未标定施工值|未标定的施工值="; grep -cE "未标定施工值|未标定的施工值" "$V53"
echo "== M-A41-5 在册三键现值（取信面与探针面之差的根）=="; grep -hoE "use_maximum: *[a-z]+|first_map_only: *[a-z]+|trinary_costmap: *[a-z]+" "$CC" | sort -u | tr '\n' ' '; echo
echo
echo "== M-A42-1 §0.6 生效序句现文字（是否只覆盖 §0.6–§0.10）=="; grep -o "生效序（写死[^*]*" "$V53" | head -1
echo "== M-A42-2 §0.11／§0.12／§0.13 章标题在位数 =="; for s in 0.11 0.12 0.13; do printf '  §%s=%s\n' "$s" "$(grep -cE "^##+ .*§?$s " "$V53")"; done
echo "== M-A42-3 §0.11 章标题自述（驳'纯施工志'前提）=="; grep -oE "^## §0\.11.{0,60}" "$V53"
echo
echo "== M-A43-1 v4.1 与 v4.3 的 X 编号种类数（同号不同事的范围）=="; echo -n "  v4.1="; grep -oE "\*\*X[0-9]+" "$V41" | sort -u | wc -l; echo -n "  v4.3="; grep -oE "\*\*X[0-9]+" "$V43" | sort -u | wc -l
echo "== M-A43-2 '同号不同事' 在册性（HEAD 面）＋阳性对照 =="; echo -n "  同号不同事="; git grep -l "同号不同事" HEAD -- note/ | wc -l; echo -n "  坑60（阳性对照）="; git grep -l "坑60" HEAD -- note/ | wc -l
echo "== M-A43-3 v4.1 与 v4.3 编号交集大小 =="; python3 -c "
import re,subprocess,io
def codes(p):
    s=io.open(p,encoding='utf-8',errors='replace').read()
    return set(re.findall(r'\*\*X(\d+)', s))
a=codes('note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md')
b=codes('note/计划书v4/21_ros1-slam隔离部署计划书-v4.3-楼梯通行与计划书同步纪律补丁.md')
print('  交集个数=%d 交集=%s' % (len(a&b), ','.join(sorted(a&b,key=int))))
"
echo "== end =="
