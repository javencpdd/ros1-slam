#!/usr/bin/env bash
# v_dwr_exitclean_selftest.sh —— v_dwa_recheck.sh 退净段「自匹配剔除」的零节点自测（R40 后续修订项的验收面）
#
# 要证的是一件事的两面：那一行 pgrep -af 模式串不再把**调用者自己**列成命中，而**真残留仍必须被列出来**
#   ——否则就是拿假绿灯换假红灯。
# 两条剔除规则各有各的活体来源，分开测、不许互相顶包：
#   规则一（按名剔）的排除串是 "v_dwa_recheck.sh" ⇒ 探针件名必须带这个字面串（少个 v_ 就是测一个不存在的口径，
#     本自测首版正栽在这里）；规则二（按祖先链剔）用**不带**该字面串的父件来验。
# 纪律同 STS／RAS／BRS：被检片段用 awk 从**现役件**抽出后 eval，不在本件复制实现；
#   片段不完整（缺 pgrep 主体或缺零命中分支）即判前置失效 rc=9，绝不在残缺片段上做判定（坑29／R40 那一族）。
# 助手件一律用**带引号的 heredoc** 生成 ⇒ 写盘时不做变量展开，运行时才展开（坑18）。
#
# 用法：bash v_dwr_exitclean_selftest.sh     退出码 0=全过；5=有 FAIL；9=现役件形态变了
set -u
ROOT=/home/ros1-slam
DWR=$ROOT/ws/src/40_navigation/lite3_navigation/verify/v_dwa_recheck.sh
NP=0; NF=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
[ -r "$DWR" ] || { echo "[FAIL] 现役 DWR 件不可读：$DWR"; echo "DWRS VERDICT: rc=9"; exit 9; }
# 开局先扫掉往轮残留的假残留件：23:0x 那轮 T2/T3 就是被上一轮没死透的 fake（PID 38401）污染的——
#   表现是「T2 说漏列、T3 说夹带一行」，看着像被检件过宽，其实是我自己的测试留下脏环境。测试也得退净。
pkill -f '^fake_roslaunch_residual_for_dwrs' 2>/dev/null; sleep 1

HELP=/tmp/dwrs_helper_v_dwa_recheck.sh
PAR=/tmp/dwrs_parent_plain.sh
CHILD=/tmp/dwrs_child_plain.sh
cat > "$HELP" <<'HELPEOF'
#!/usr/bin/env bash
DWR="${DWRPATH:?需要 DWRPATH}"
SEG=$(awk '/_anc=\$\$; while/{p=1} p{print} p&&/echo "  （零命中）"; fi/{exit}' "$DWR")
printf '%s\n' "$SEG" | grep -q 'pgrep -af' || { echo "SEGMISS"; exit 9; }
printf '%s\n' "$SEG" | grep -q '（零命中）"; fi' || { echo "SEGMISS"; exit 9; }
echo "SELF=$$"
eval "$SEG"
HELPEOF
chmod 755 "$HELP"
cat > "$PAR" <<'PAREOF'
#!/usr/bin/env bash
# 父件名里没有 v_dwa_recheck.sh 字面串，只有 dwa_recheck 字样 ⇒ 这时只有祖先链规则能把它剔掉
bash "$CHILD"
PAREOF
chmod 755 "$PAR"
cat > "$CHILD" <<'CHILDEOF'
#!/usr/bin/env bash
echo "CHILD_SELF=$$"
echo "CHILD_PARENT=$PPID"
DWRPATH="$DWRPATH" bash "$HELP"
CHILDEOF
chmod 755 "$CHILD"

# ---- T1a 阳性对照：旧式（无剔除）必自匹配——不先复现，就不许宣称修好了 ----
cat > /tmp/dwrs_oldform_dwa_recheck.sh <<'OLDEOF'
#!/usr/bin/env bash
pgrep -af 'roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|sim_adapter|dwa_recheck' 2>/dev/null \
  | awk -v me="$$" '$1==me {print "SELFLINE"}'
echo "ME=$$"
OLDEOF
chmod 755 /tmp/dwrs_oldform_dwa_recheck.sh
bash /tmp/dwrs_oldform_dwa_recheck.sh > /tmp/dwrs_t1a.txt 2>&1
grep -q 'SELFLINE' /tmp/dwrs_t1a.txt && mark 0 "T1a 阳性对照：旧式确实自匹配（$(grep -o 'ME=[0-9]*' /tmp/dwrs_t1a.txt)）" \
                                    || mark 1 "T1a 阳性对照不过：旧式没复现出自匹配 ⇒ 探针看不见这一形态，后面全部断言不可信"

# ---- T1b 规则一（按名）----
DWRPATH="$DWR" bash "$HELP" > /tmp/dwrs_t1b.txt 2>&1
if grep -q 'SEGMISS' /tmp/dwrs_t1b.txt; then echo "[FAIL] 现役片段抽得不全（SEGMISS）⇒ 不在残缺片段上判定"; echo "DWRS VERDICT: rc=9"; exit 9; fi
ME=$(grep -oE '^SELF=[0-9]+' /tmp/dwrs_t1b.txt | cut -d= -f2)
H1=$(awk -v m="$ME" 'NR>1 && $1==m' /tmp/dwrs_t1b.txt | wc -l)
N1=$(awk 'NR>1 && $0 ~ /dwrs_helper_v_dwa_recheck.sh/' /tmp/dwrs_t1b.txt | wc -l)
[ -n "$ME" ] && [ "$H1" = "0" ] && [ "$N1" = "0" ] && mark 0 "T1b 规则一（按名）：自调用者 PID=$ME 与同名助手件均未列成命中" \
             || mark 1 "T1b 规则一失效（PID 命中=$H1 名命中=$N1）"

# ---- T2 阳性对照：真残留必须仍被列出 ----
setsid bash -c 'exec -a "fake_roslaunch_residual_for_dwrs" sleep 25' </dev/null >/dev/null 2>&1 &
disown    # 不收 job：否则 kill 时 bash 往 stderr 打「N Killed ...」的作业报告，混进证据里像缺陷（其实是作业控制）
sleep 2
# 取**这一轮新起的那个** PID（用 pgrep 全集减去开局已存在的），不接受 head -1 撞上往轮残留
FAKE=$(pgrep -f '^fake_roslaunch_residual_for_dwrs' | tail -1)
NFAKE=$(pgrep -cf '^fake_roslaunch_residual_for_dwrs')
if [ -z "$FAKE" ]; then
  mark 1 "T2 前置不成立：假残留没起来（本条不给绿）"
else
  DWRPATH="$DWR" bash "$HELP" > /tmp/dwrs_t2.txt 2>&1
  H2=$(awk -v f="$FAKE" 'NR>1 && $1==f' /tmp/dwrs_t2.txt | wc -l)
  if [ "${NFAKE:-1}" != "1" ]; then mark 1 "T2 前置不成立：场上有 $NFAKE 个假残留（应 1），环境脏了不自测"
  elif [ "$H2" = "1" ]; then mark 0 "T2 阳性对照：真残留 PID=$FAKE 仍被列出（剔除面没盖住它）"; else
    mark 1 "T2 真残留被漏列（PID=$FAKE 命中=$H2）⇒ 规则过宽＝假绿灯"
    echo "      [读数] 助手件输出全文（$(wc -l < /tmp/dwrs_t2.txt) 行）:"; sed 's/^/        /' /tmp/dwrs_t2.txt | head -6
    echo "      [读数] 现役片段行数=$(awk '/_anc=\$\$; while/{p=1} p{print} p&&/echo "  （零命中）"; fi/{exit}' "$DWR" | wc -l)"
  fi
fi

# ---- T3 零命中形态可指认 ----
pkill -KILL -f '^fake_roslaunch_residual_for_dwrs' 2>/dev/null; sleep 1
DWRPATH="$DWR" bash "$HELP" > /tmp/dwrs_t3.txt 2>&1
Z3=$(grep -c '（零命中）' /tmp/dwrs_t3.txt)
X3=$(awk 'NR>1 && !/（零命中）/ && $1 ~ /^[0-9]+$/' /tmp/dwrs_t3.txt | wc -l)
if [ "$Z3" = "1" ] && [ "$X3" = "0" ]; then mark 0 "T3 零命中形态可指认（打出（零命中），夹带命中行=$X3）"; else
  mark 1 "T3 零命中形态不可指认（零命中行=$Z3 夹带行=$X3）⇒ 空输出冒充零命中那一型"
  echo "      [读数] 输出全文:"; sed 's/^/        /' /tmp/dwrs_t3.txt | head -6
  echo "      [读数] 该时刻原始 pgrep 命中数=$(pgrep -af 'roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|sim_adapter|dwa_recheck' 2>/dev/null | wc -l)"
fi

# ---- T4 规则二（祖先链）----
HELP="$HELP" CHILD="$CHILD" DWRPATH="$DWR" bash "$PAR" > /tmp/dwrs_t4.txt 2>&1
CS=$(grep -oE 'CHILD_SELF=[0-9]+' /tmp/dwrs_t4.txt | cut -d= -f2)
CP=$(grep -oE 'CHILD_PARENT=[0-9]+' /tmp/dwrs_t4.txt | cut -d= -f2)
HS=$(awk -v m="$CS" '$1==m' /tmp/dwrs_t4.txt | wc -l)
HP=$(awk -v m="$CP" '$1==m' /tmp/dwrs_t4.txt | wc -l)
PN=$(awk '$1 ~ /^[0-9]+$/ && /dwrs_parent_plain/' /tmp/dwrs_t4.txt | wc -l)
[ -n "$CP" ] && [ "$HP" = "0" ] && [ "$PN" = "0" ] && [ "$HS" = "0" ] \
  && mark 0 "T4 规则二（祖先链）：父件 PID=$CP 与子件 PID=$CS 都没被列成命中（父件名里无 v_dwa_recheck.sh 字面串，救它的只有祖先链规则）" \
  || mark 1 "T4 规则二失效（父命中=$HP 父名命中=$PN 子命中=$HS，父 PID=${CP:-不可测}）"

rm -f "$HELP" "$PAR" "$CHILD" /tmp/dwrs_oldform_dwa_recheck.sh
echo "=================================================="
echo "DWRS VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
