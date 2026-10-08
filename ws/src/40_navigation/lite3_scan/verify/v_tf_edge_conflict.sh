#!/usr/bin/env bash
# v_tf_edge_conflict.sh —— 反向静态边与单父不变式的零节点静态防线（小明 2026-10-08，FD-I 处置第 3 项）
# 为何存在：小美 R31 把「同两端点、反方向的两条静态边一旦同窗 ⇒ TF 成环＋base_link 双父」写成升阻断条件，
#   但只靠活体窗发现就太晚（M2R4/U1 那次已吃了 7 跑全 HIT）。本件在零节点态复核：谁在发以 base_link 为子的
#   静态边、反向对是否共存、共存时 base_link→body 一侧是否已被缺省关闭并注记、30_3rd 基线是否被动。
# C1 是**阳性对照**（05 B2／C4b）：扫描面必须先认出在册已知的 body→base_link 那条边，才有资格说「名册外零命中」。
#   首版没有这条，实测在 sed 括号写错致边表为空的情况下 C1 直接绿——空集真命题，正是成员对账被否证过的那一型。
# 边界：本件不进门禁 v3_isolation.sh（把新判据塞进共同门禁属判据面变更，依 05 A3／A3b 需验收先落笔）。
# 性质：零 ROS 节点、零 roslaunch、零活体、零 ws 写、无 sudo。二态 rc（0 全过／5 有 FAIL），件内无有效性门可折。
set -u
ROOT=/home/ros1-slam
cd "$ROOT" || exit 5
NP=0; NF=0
mark() { if [ "$1" = "1" ]; then NP=$((NP+1)); echo "[PASS] $2"; else NF=$((NF+1)); echo "[FAIL] $2"; fi; }
MINI=ws/src/30_perception/deeprobotics_local_height_map/launch/height_map_mini.launch
RELAY_SIM=ws/src/40_navigation/lite3_scan/launch/lio_relay_sim.launch
RELAY_3R=ws/src/30_3rd/plan_manage/launch/lio_relay.launch
EDGES=$(mktemp /tmp/tf_edges.XXXXXX)

python3 - > "$EDGES" <<'PYEOF'
import os, re
pat = re.compile(r'args\s*=\s*"([^"]*)"')
for dp, dn, fn in os.walk("ws/src"):
    if "/build/" in dp or "/devel/" in dp:
        continue
    for f in fn:
        if not f.endswith(".launch"):
            continue
        p = os.path.join(dp, f)
        try:
            txt = open(p, encoding="utf-8", errors="ignore").read()
        except Exception:
            continue
        # node 标签常跨行（30_3rd 的 body_to_base_link 就是 args 在第二行），按整元素匹配而非逐行；
        # 首版逐行扫正因这条把在册已知边扫漏，被本件 C1 阳性对照当场判废（05 B2 的效力实证）。
        for m in re.finditer(r"<node[^>]*?args\s*=\s*\"([^\"]*)\"[^>]*?>", txt, re.S):
            tag = m.group(0)
            if "static_transform_publisher" not in tag:
                continue
            tok = m.group(1).split()
            if len(tok) < 2:
                continue
            parent, child = tok[-2], tok[-1]
            lineno = txt[:m.start()].count("\n") + 1
            print("\t".join([parent, child, p, str(lineno)]))
PYEOF
[ -s "$EDGES" ] || { echo "扫描面为空 ⇒ 扫描器无分辨力"; rm -f "$EDGES"; exit 5; }
_norm() { sed 's|^/||'; }   # TF frame_id 的 /foo 与 foo 同义

# ---- C1 阳性对照：必须认出在册已知的 body→base_link ----
_hit=$(awk -F'\t' '{p=$1; c=$2; gsub(/^\//,"",p); gsub(/^\//,"",c); if (p=="body" && c=="base_link" && $3=="'"$RELAY_3R"'") print}' "$EDGES" | wc -l)
[ "$_hit" -ge 1 ] && mark 1 "C1 阳性对照：扫描面认出 30_3rd/lio_relay.launch 的 body→base_link（实测 $_hit 条）⇒ 后续「零命中」类结论有资格成立" \
                  || mark 0 "C1 阳性对照不过：扫描面认不出在册已知边 ⇒ 本件所有零命中结论作废（扫描器无分辨力）"

# ---- C2 名册：以 base_link 为子的静态边只允许在册那一条 ----
BLIN=$(awk -F'\t' '{c=$2; gsub(/^\//,"",c); if (c=="base_link") print $3":"$4" "$1"->"$2}' "$EDGES")
echo "-- 以 base_link 为子的静态边（本件扫得）--"
printf '%s\n' "$BLIN" | sed 's/^/  /'
_bad=$(printf '%s\n' "$BLIN" | grep -v '^$' | grep -vc "^$RELAY_3R" )
_cnt=$(printf '%s\n' "$BLIN" | grep -c . )
if [ "$_cnt" -ge 1 ] && [ "$_bad" = "0" ]; then mark 1 "C2 名册内：base_link 为子的静态边共 $_cnt 条，全属 30_3rd/lio_relay.launch（批2 用壳包 relay 绕开的那条）"
else mark 0 "C2 名册外或空集：扫得 $_cnt 条，其中名册外 $_bad 条 ⇒ 双父新源，按 R31 升阻断条件停手上报"; fi

# ---- C3 反向对：存在 base_link→body 时，该侧必须缺省关闭且带互斥注记 ----
REV=$(awk -F'\t' '{p=$1; c=$2; gsub(/^\//,"",p); gsub(/^\//,"",c); if (p=="base_link" && c=="body") print $3}' "$EDGES" | sort -u)
if [ -z "$REV" ]; then
  mark 0 "C3 未扫到任何 base_link→body 边 ⇒ 与本件立项事实（height_map_mini.launch:64）不符，扫描器口径待复核"
else
  _det=1
  for f in $REV; do
    d=$(sed -n 's/.*<arg name="enable_urdf"[^>]*default="\([a-z]*\)".*/\1/p' "$f" 2>/dev/null | head -1)
    n=$(grep -c '不得与 lio_relay 同窗' "$f" 2>/dev/null)
    echo "  $f：base_link→body 在位，enable_urdf 缺省=${d:-未声明}，互斥注记行数=$n"
    [ "$d" = "false" ] || _det=0
    [ "$n" -ge 1 ] || _det=0
  done
  mark "$_det" "C3 反向对静态等价式：同仓共存 base_link→body 与 body→base_link 时，前者所在件必须缺省关闭且有互斥注记（R31 升阻断的零节点前置形态）"
fi

# ---- C4 壳包自持 relay 不得引入以 base_link 为子的静态边 ----
if [ -f "$RELAY_SIM" ]; then
  _bad2=$(awk -F'\t' -v r="$RELAY_SIM" '{c=$2; gsub(/^\//,"",c); if ($3==r && c=="base_link") print}' "$EDGES" | wc -l)
  _tot=$(grep -c 'static_transform_publisher' "$RELAY_SIM")
  [ "$_bad2" = "0" ] && mark 1 "C4 壳包 relay 零条 base_link 为子的静态边（该件静态边共 $_tot 条，全为可视化用途）" \
                    || mark 0 "C4 壳包 relay 出现 $_bad2 条 base_link 为子的静态边 ⇒ FD-G 双父复发"
else
  mark 0 "C4 壳包 relay 件不在位（$RELAY_SIM）⇒ 批2 前置件缺失"
fi

# ---- C5 30_3rd 基线双条件未被触动（动 30_perception 侧不得波及 30_3rd）----
_BASE="$ROOT/note/进度/SCAN集成/证据/scan_src_baseline_49.sha256"
_ok=$(cd ws/src/30_3rd && sha256sum -c "$_BASE" 2>&1 | grep -vc ": OK$")
_cnt2=$(find ws/src/30_3rd -type f ! -name 'scan_src_baseline_49.sha256' | wc -l)
[ "$_ok" = "0" ] && [ "$_cnt2" = "49" ] && mark 1 "C5 30_3rd 基线双条件全过（非 OK 行数=$_ok，计数=$_cnt2）" \
                  || mark 0 "C5 30_3rd 基线双条件不过（非 OK 行数=$_ok，计数=$_cnt2，应为 0 与 49）"

rm -f "$EDGES"
echo "=================================================="
echo "TF EDGE CONFLICT VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
