#!/usr/bin/env bash
# v_scan_relay_equiv.sh —— 壳包自持 relay 与上游 lio_relay 的**零节点静态对账**（批2 前置，小明 2026-10-08）
# 判据（小张 fd 矩阵 §10.4 批2、设计件表 FD）：
#   ① 输出话题集与 arg 默认值与上游逐字相同（前馈通道不得被顺手改掉）
#   ② 唯一允许的差异 = 少掉静态边 body->base_link（FD-G 双父源）；多删或改动其余节点即 FAIL
#   ③ 壳包件内不得存在任何以 base_link 为**子帧**的静态边（U1 单父不变式）
#   ④ 30_3rd 基线双条件不被触动（本件只读上游件）
# 零节点、零 roslaunch：纯 XML 解析与集合比对，可在任何窗执行（含真机采集时）。
# ★R606／批258 核出、批259 落裁★＝原 ROOT 写死容器面路径，宿主面恒不可达；按 L1 问三裁甲改成 git 派生＋缺根即退（与 D7 件第 45 行同形）。
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=/home/ros1-slam
set -u
UP=$ROOT/ws/src/30_3rd/plan_manage/launch/lio_relay.launch
SH=$ROOT/ws/src/40_navigation/lite3_scan/launch/lio_relay_sim.launch
FAIL=0
chk() { if [ "$1" = "0" ]; then echo "[PASS] $2"; else echo "[FAIL] $2"; FAIL=1; fi; }
[ -f "$SH" ]; chk $? "壳包 relay 件在位 $SH"
python3 - "$UP" "$SH" <<'PYEOF'
import sys, xml.etree.ElementTree as ET
def sig(p):
    r = ET.parse(p).getroot()
    n = {x.get('name'): (x.get('pkg'), x.get('type'), (x.get('args') or '').strip()) for x in r.iter('node')}
    a = {x.get('name'): x.get('default') for x in r.iter('arg')}
    return n, a
un, ua = sig(sys.argv[1]); sn, sa = sig(sys.argv[2])
bad = 0
if ua != sa:
    print('[FAIL] arg 默认值不一致 %s vs %s' % (ua, sa)); bad = 1
else:
    print('[PASS] arg 默认值逐字一致（%d 项）' % len(ua))
only_up = sorted(set(un) - set(sn)); only_sh = sorted(set(sn) - set(un))
if only_up == ['body_to_base_link'] and not only_sh:
    print('[PASS] 唯一差异 = 去掉静态边 body_to_base_link（共同节点 %d 个逐一比对）' % len(set(un) & set(sn)))
else:
    print('[FAIL] 差异面越界：仅上游有=%s 仅壳包有=%s' % (only_up, only_sh)); bad = 1
for k in sorted(set(un) & set(sn)):
    if un[k] != sn[k]:
        print('[FAIL] 共同节点被改动 %s: %s vs %s' % (k, un[k], sn[k])); bad = 1
if not bad:
    print('[PASS] 共同节点 pkg/type/args 全部逐字相同')
for k, v in sn.items():
    if v[1] == 'static_transform_publisher' and v[2].split()[-1] == 'base_link':
        print('[FAIL] 壳包件仍发以 base_link 为子帧的静态边：%s %s' % (k, v[2])); bad = 1
print('[PASS] U1 单父不变式：壳包件无以 base_link 为子帧的静态边' if not bad else '')
raise SystemExit(bad)
PYEOF
chk $? "relay 等价与单父静态断言（上列四项）"
rm -f /tmp/relay_equiv_base.txt   # ★乙档在场断言（问一乙档欠的那半、批263 落）★＝先删陈旧证人，防止校验行没跑成时读到上一次的文件给绿
LC_ALL=C; cd "$ROOT/ws/src/30_3rd" && LC_ALL=C sha256sum -c "$ROOT/note/进度/v7/S1/SCAN集成/证据/scan_src_baseline_49.sha256" > /tmp/relay_equiv_base.txt 2>&1; LC_ALL=C
[ -s /tmp/relay_equiv_base.txt ]; chk $? "乙档在场断言：证人文件本轮真被生成（非陈旧）"
N=$(grep -vc ': OK$' /tmp/relay_equiv_base.txt); [ "$N" = 0 ] && [ "$(find . -type f | wc -l)" = 49 ]
chk $? "30_3rd 基线双条件未被触动（非 OK=$N，计数=$(find . -type f | wc -l)）"
# ---- 批2 接线回归断言：70_scan.launch 必须指壳包自持件，且不得再指上游件（两者同时成立才算接线生效）----
L7=$ROOT/ws/src/40_navigation/lite3_scan/launch/70_scan.launch
grep -q "find lite3_scan)/launch/lio_relay_sim.launch" "$L7" && ! grep -q "find scan_planner)/launch/lio_relay.launch" "$L7"
chk $? "批2 接线在位：70_scan.launch 的 include 目标＝壳包 lio_relay_sim.launch，且上游件引用已消失"
[ "$FAIL" = 0 ] && echo "RELAY EQUIV VERDICT: PASS" || echo "RELAY EQUIV VERDICT: FAIL"
exit "$FAIL"
