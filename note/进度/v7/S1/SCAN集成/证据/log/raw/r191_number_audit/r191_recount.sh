#!/usr/bin/env bash
# r191_recount.sh —— #81 数字账实核对的复跑驱动（只读被试仓；零开窗、零节点、无 sudo）
# 产物全部落在本目录（同批留底）。任何一步取不到数就打印 FAIL 行并非 0 退出——
# 空读数不得当"复现成功"（在册 05 §十二补：判据必须能失败）。
set -u
cd /home/ros1-slam || exit 9
E=note/进度/SCAN集成/证据/log/raw/r191_number_audit
VCC=scripts/verify/v_citation_check.sh
R163=$(git log --format='%H %s' | grep -m1 'R163' | cut -d' ' -f1)
R134=36dafc6
FACE=/tmp/r191_face_r163
SNAP=/tmp/r191_snap_r134
rc_all=0

say() { printf '%s\n' "$*" | tee -a "$E/RUN.log"; }

# ── 面 A：R163 提交态整树 + 【git 忽略件补回】──────────────────────────────
# git archive 只收 git 收录的文件，而 VCC 把每条引用回整树解析 ⇒ 被 .gitignore 挡掉的
# opt/upstream/（含在册那条 lvr2 CMakeLists）一进临时面就没了，RED 列当场多 3 条"路径失效"。
# 这一步不是修饰，是本次核对抓到的第一条面条件。
rm -rf "$FACE"; mkdir -p "$FACE"
git archive "$R163" | tar -x -C "$FACE"
mkdir -p "$FACE/opt/upstream/lvr2"
cp -p /home/ros1-slam/opt/upstream/lvr2/CMakeLists.txt "$FACE/opt/upstream/lvr2/CMakeLists.txt"
say "面 A 建好：$FACE（R163=$(git log -1 --format=%h $R163) ＋补回 git 忽略件 1 条）"

# ── 组1／组3：A25 六档窗宽 ＋ R163 普查数（面 A＋当年件）───────────────────
: > "$E/tiers_A_r163face_with_r163piece.txt"
for cfg in "20:" "40:" "60:" "100:" "200:" ":same_line"; do
  CH="${cfg%%:*}"; MO="${cfg##*:}"
  o=$(cd "$FACE" && VCC_ROOT="$FACE" VCC_FIXUP_CHARS="$CH" VCC_FIXUP_MODE="$MO" bash scripts/verify/v_citation_check.sh 2>&1)
  c=$(cd "$FACE" && VCC_ROOT="$FACE" VCC_FIXUP_CHARS="$CH" VCC_FIXUP_MODE="$MO" bash scripts/verify/v_citation_check.sh --census 2>&1)
  printf '%s｜scan：%s｜census：%s\n' "窗=${CH:-整行} mode=${MO:-跨行}" "$(printf '%s' "$o" | grep '^\[计数\]')" "$(printf '%s' "$c" | grep -oE '目标=[0-9]+ 处.*' | head -1)" >> "$E/tiers_A_r163face_with_r163piece.txt"
done
printf '%s\n' "（对照在册件 r163_vcc_census_first_run/fixup_window_sweep.txt 与 summary.txt）" >> "$E/tiers_A_r163face_with_r163piece.txt"

# 面 A＋今天的件：证 R175/R181/R190 三次改件没动默认判定（表在面 A 里不存在 ⇒ 应与当年逐字同值）
: > "$E/tiers_A_r163face_with_todaypiece.txt"
for cfg in "20:" "40:" "60:" "100:" "200:" ":same_line"; do
  CH="${cfg%%:*}"; MO="${cfg##*:}"
  o=$(VCC_ROOT="$FACE" VCC_FIXUP_CHARS="$CH" VCC_FIXUP_MODE="$MO" bash "$VCC" 2>&1)
  printf '%s｜%s\n' "窗=${CH:-整行} mode=${MO:-跨行}" "$(printf '%s' "$o" | grep '^\[计数\]')" >> "$E/tiers_A_r163face_with_todaypiece.txt"
done

# 现役面（今天的工作树、今天的件、默认 27 件文档面）
: > "$E/tiers_B_live_default_face.txt"
for cfg in "20:" "40:" "60:" "100:" "200:" ":same_line"; do
  CH="${cfg%%:*}"; MO="${cfg##*:}"
  o=$(VCC_FIXUP_CHARS="$CH" VCC_FIXUP_MODE="$MO" bash "$VCC" 2>&1)
  c=$(VCC_FIXUP_CHARS="$CH" VCC_FIXUP_MODE="$MO" bash "$VCC" --census 2>&1)
  printf '%s｜scan：%s｜census：%s\n' "窗=${CH:-整行} mode=${MO:-跨行}" "$(printf '%s' "$o" | grep '^\[计数\]')" "$(printf '%s' "$c" | grep -oE '目标=[0-9]+ 处.*' | head -1)" >> "$E/tiers_B_live_default_face.txt"
done

# ── 组4：R173 豁免效果（志文面 52／计划书面 4 与 5／失败方向）──────────────
DOCS=$(sed -n 's/^# 被检面：//p' "$E"/../r182_citation_baseline_first/baseline_note52.tsv)
[ -n "$DOCS" ] || { say "FAIL：52 件志文面取不出来"; exit 9; }
bash "$VCC" $DOCS > "$E/g4_zhiven52_with_table.txt" 2>&1
VCC_EXEMPT=/nonexistent bash "$VCC" $DOCS > "$E/g4_zhiven52_table_missing.txt" 2>&1
P4123=$(ls note/计划书v4/*.md | grep -v 'v4\.4')
bash "$VCC" $P4123 > "$E/g4_planbook_r173face4.txt" 2>&1
bash "$VCC" note/计划书v4/*.md > "$E/g4_planbook_today5.txt" 2>&1

# ── 组5：R168 双时点（表关＝回到"豁免档不存在时"的判定本体）────────────────
rm -rf "$SNAP"; mkdir -p "$SNAP"; git archive "$R134" note | tar -x -C "$SNAP"
sed -n '/^VCC_MAX=400/,$p' "$E"/../r168_amber_attribution/summary.txt \
  | grep -oE '/tmp/snap_r134/[^ ]+\.md' | sed "s|^/tmp/snap_r134/|$SNAP/|" > /tmp/r191_docs_r134.txt
[ -s /tmp/r191_docs_r134.txt ] || { say "FAIL：R168 那 49 件名单取不出来"; exit 9; }
VCC_MAX=400 VCC_EXEMPT=/nonexistent bash "$VCC" $(cat /tmp/r191_docs_r134.txt) > "$E/g5_timepoint_A_r134snap.txt" 2>&1
VCC_MAX=400 VCC_EXEMPT=/nonexistent bash "$VCC" $DOCS > "$E/g5_timepoint_B_worktree.txt" 2>&1
: > "$E/g5_comm_two_way.txt"
for t in AMBER RED; do
  grep "^\[$t\]" "$E/g5_timepoint_A_r134snap.txt" | sort > /tmp/g5_A_$t.txt
  grep "^\[$t\]" "$E/g5_timepoint_B_worktree.txt" | sort > /tmp/g5_B_$t.txt
  say "$t：A=$(wc -l < /tmp/g5_A_$t.txt) B=$(wc -l < /tmp/g5_B_$t.txt)"
  printf '%s 只在A：%s｜只在B：%s\n' "$t" "$(comm -23 /tmp/g5_A_$t.txt /tmp/g5_B_$t.txt | tr '\n' ' ')" "$(comm -13 /tmp/g5_A_$t.txt /tmp/g5_B_$t.txt | tr '\n' ' ')" >> "$E/g5_comm_two_way.txt"
done

# ── 组2：A23 clearance 全枚举（判据行取真判，不读 json 的死字段——缺口B）───
python3 - > "$E/g2_clearance_full_census.txt" <<'PY'
# -*- coding: utf-8 -*-
import json,glob,re,os
def find(o,k):
    if isinstance(o,dict):
        for a,b in o.items():
            if a==k: return b
            r=find(b,k)
            if r is not None: return r
    elif isinstance(o,list):
        for b in o:
            r=find(b,k)
            if r is not None: return r
    return None
rows=[]
for f in sorted(glob.glob('note/进度/SCAN集成/证据/log/raw/**/p4.json',recursive=True)):
    t=open(f,encoding='utf-8',errors='ignore').read()
    md=None
    for line in t.splitlines():
        s=line.strip()
        if s.startswith('{'):
            try: d=json.loads(s)
            except Exception: continue
            md=find(d,'min_dist_to_any_occ')
            if md is not None: break
    if md is None:
        m=re.search(r'"min_dist_to_any_occ"\s*:\s*([0-9.]+)',t)
        if m: md=float(m.group(1))
    v=None
    for cand in (f, os.path.join(os.path.dirname(f),'harness.log'), os.path.join(os.path.dirname(f),'p4_stdout.txt')):
        if v: break
        if os.path.exists(cand):
            mm=re.search(r'P4 重规划结论\s*=\s*([A-Z]+)',open(cand,encoding='utf-8',errors='ignore').read())
            if mm: v=mm.group(1)
    rows.append((f.replace('note/进度/SCAN集成/证据/log/raw/',''),md,v))
print("# 盘上全部带 min_dist_to_any_occ 的 P4 窗（判取自判据行，非 json 的 invalid/fail 死字段）")
for f,md,v in rows: print("  %-8s %-7s %s"%(str(md) if md is not None else '—',v,f))
vals=[r[1] for r in rows if r[2]=='PASS' and isinstance(r[1],(int,float))]
allv=[r[1] for r in rows if isinstance(r[1],(int,float))]
print("非空读数共 %d 条｜最小=%.3f"%(len(allv),min(allv)))
print("判 PASS 者共 %d 窗｜其 clearance 最小=%.3f｜<0.10 的 PASS=%s｜<0.20 的 PASS=%s"%(len(vals),min(vals),[x for x in vals if x<0.10],[x for x in vals if x<0.20]))
PY

# ── 自证：每件产物都必须真有读数（空文件不算复现成功）──────────────────────
for f in tiers_A_r163face_with_r163piece tiers_A_r163face_with_todaypiece tiers_B_live_default_face \
         g4_zhiven52_with_table g4_zhiven52_table_missing g4_planbook_r173face4 g4_planbook_today5 \
         g5_timepoint_A_r134snap g5_timepoint_B_worktree; do
    grep -q '\[计数\]' "$E/$f.txt" || { say "FAIL：$f.txt 里没有 [计数] 行 ⇒ 这一组没跑出来"; rc_all=5; }
done
grep -q '判 PASS 者共' "$E/g2_clearance_full_census.txt" || { say "FAIL：clearance 全枚举没出总结行"; rc_all=5; }
[ -s "$E/g5_comm_two_way.txt" ] || { say "FAIL：双时点 comm 对照为空"; rc_all=5; }

say "驱动跑完 rc_all=$rc_all"
exit $rc_all
