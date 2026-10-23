#!/usr/bin/env bash
# _batch_readout.sh —— ★批末读数面五枚的可复跑采集件★（登记面纪律件、★不是闸★：不进任何件的 rc、不入门禁串、不改成员与语义）
# 起意＝追加七十九③ 立的「批末读数面固定五枚、缺一即自曝」连犯四次（批137—139 漏 P8、批142 漏两枚、批150 取的是提交前的丁案源、
#         ④ 甲/乙累计 自批143 起一律报"未动"却从没跑过式）⇒ 把"当次同调用现取"从脑子里的动作变成一件能跑、会翻红的东西。
# 用法＝bash _batch_readout.sh [--phase pre|post] [--skip-chain] [--batch N] [--BR_SKIP=<①..⑤|6=附二|7=附三|8=附四|9=附五|10=附六|9=附五>]
#   --phase post 只影响⑤的措辞（★提交前跑⇒本批那次提交不在丁案源内★）；--skip-chain 用于同一批内早先已跑过串的那一遍；
#   ★--batch N＝把本批批号显式传给附三的 firing 探测件（★批201 补★：此前本件不接 --batch ⇒ BR_BATCH 恒空
#     ⇒ 附三只能收到 --batch "?" ⇒ 状态面印出批号「?」两行；★现由调用方显式给★、不给则缺省仍是 ?＝向后兼容）★
#   --BR_SKIP=3 是★负例开关★：故意让第③枚取不到 ⇒ 本件必须翻红（rc=5）并点名缺哪枚。
# 附面（★全部只报不拦＝不进 MISS、不进任何件的 rc★）＝附 清盘计数 ‖ 附二 清单覆盖面（第六枚）‖ 附三 firing 探测（第九枚）
#            ‖ ★附四 governance 漂移闸（第十枚＝2026-10-21 真人点出四处陈旧后建）★＝红档面必须归零、笔权与历史面走旁报★
#            ‖ ★附五 志面与标题禁形闸（第十一枚＝同批反引号第八次复发后建）★＝只检本批新增行＝历史行照留★
# 面（★两面分别写死★）＝①②③④⑤ 全在★宿主面★跑（账构与引用件必须显式传根 VLB_ROOT／VCC_ROOT，容器内无 .git ⇒ 账构恒 rc=9）；
#              容器面只跑门禁串与清盘计数。
set -u
# ★R589／批242 修 ROOT★＝本件随 SCAN集成 于批208 搬入 note/进度/v7/S1/… ⇒ 实际深 6 级，原式四跳只到 note/进度
#   ⇒ 该件自批208 起★恒报「前置缺失＝不在位／不是 git 仓」★，批末读数面四枚附面同步恒报「件不在位」（本批现跑抓到）
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=$(cd "$(dirname "$0")/../../../../.." && pwd)
PHASE=pre; SKIPCHAIN=0; BRSKIP=""; BR_BATCH=""
while [ $# -gt 0 ]; do
  case "$1" in
    --phase) PHASE="$2"; shift 2 ;;
    --batch) BR_BATCH="$2"; shift 2 ;;
    --skip-chain) SKIPCHAIN=1; shift ;;
    --BR_SKIP=*) BRSKIP="${1#--BR_SKIP=}"; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
cd "$ROOT" || exit 9
MISS=""
note(){ MISS="$MISS $1"; }

echo "== 批末读数面五枚（★当批现取、非跨批抄★）phase=$PHASE =="

echo
echo "── ① 容器面门禁串逐件 ＋ 窗数回执 ＋ 宿主面引用面十字段 ──"
if [ "$BRSKIP" = "1" ]; then note "①"; echo "  （负例开关跳过）";
elif [ "$SKIPCHAIN" = "1" ]; then echo "  串＝本批早先已跑（★同批引用、不跨批抄★）；窗数回执现取＝$(grep -c '^R' scripts/verify/_window_receipt.tsv) 行";
else
  docker exec ros1-slam-sim bash -lc "cd /opt/ros1-slam && bash scripts/verify/run_gate_chain.sh" 2>&1 |
    awk '/^ *(SYN|GTC|ISO|DC|PYST|VAR|PARITY|账构|VCC|规矩⑨) +rc=/ {v=$2; sub(/^rc=/,"",v); printf "  %s rc=%s\n", $1, v}
         /窗数回执/ {l=$0; sub(/^.*窗数回执＝/,"",l); print "  窗数回执＝" l}
         /面：/ {print "  "$0}'
fi
BASE=note/进度/v7/S1/SCAN集成/证据/log/raw/r447_真人六句到签落地批/04_baseline_note71w40_r447.tsv
if [ -f "$BASE" ]; then
  VCC_ROOT=$PWD bash scripts/verify/v_ledger_citation_baseline.sh --check "$BASE" $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) 2>&1 |
    awk '/^ +(文档|单点引用|OK|RED|AMBER|勘误句|非单点形状|豁免红|豁免表命中|豁免表未命中) /{printf "%s%s",sep,$4; sep="/"} END{printf "\n"}' |
    sed 's/^/  引用面十字段（现值＝基线→现值箭头后那枚；基线是 r447 的陈旧在册值、比的是与批138—N 逐项同值）＝/'
else note "①"; echo "  ★引用面基线件不在位＝不可现取★"; fi

echo
echo "── ② 账构（宿主面显式 VLB_ROOT）──"
if [ "$BRSKIP" = "2" ]; then note "②"; echo "  （负例开关跳过）"; else
  OUT=$(VLB_ROOT=$PWD bash scripts/verify/v_ledger_batch_consistency.sh 2>&1); RC=$?
  echo "  rc=$RC ‖ $(printf '%s\n' "$OUT" | grep -oE 'C4 滞后\]? [0-9]+ 批' | head -1) ‖ $(printf '%s\n' "$OUT" | grep -E '^\[C1 ' | head -1)"
  # ★R510 批166 新增枚＝「档头时间枚」形检＋与当刻钟面差（只报不拦、不进任何件的 rc、不入门禁串）★
  #   起意＝批163—165 三批的档头时间枚被模板里 T[-5:] 的切片改成「分:秒」而无人报错（28:58 ‖ 03:35 ‖ 11:51）；
  #   姿势＝凡从钟面串里截一段，必须同句声明截的是哪几枚字符并打印一次对表。
  HEADTM=$(sed -n '2p' "$ROOT/note/进度/00_总览.md" | grep -oE '更新时间：[0-9]{1,2}:[0-9]{2}' | head -1 | sed 's/更新时间：//')
  NOWHM=$(date '+%H:%M')
  SHAPEOK=$(printf '%s' "$HEADTM" | grep -cE '^[0-2][0-9]:[0-5][0-9]$')
  DIFFMIN=$(awk -v a="$HEADTM" -v b="$NOWHM" 'BEGIN{split(a,x,":");split(b,y,":");m=(y[1]*60+y[2])-(x[1]*60+x[2]); if(m<0)m=-m; print m}')
  if [ "$SHAPEOK" != "1" ]; then echo "  ★档头时间枚形检不过＝现取[$HEADTM] ‖ 当刻 HH:MM＝$NOWHM★＝本批该行可能被抄歪（只报不拦）";
  elif [ "$DIFFMIN" -gt 90 ]; then echo "  ★档头时间枚＝[$HEADTM] ‖ 当刻＝$NOWHM ‖ 差 ${DIFFMIN} 分＞90★＝疑滞批或被改坏（只报不拦；★本枚不覆盖 C1 的档头账实核对★）";
  else echo "  档头时间枚＝[$HEADTM] ‖ 当刻＝$NOWHM ‖ 差 ${DIFFMIN} 分 ‖ 形检过＝1（只报不拦）"; fi
  echo "  ★读法照坑79＝rc=0 只覆盖 C2＋C4＋C5、不覆盖 C1 的档头账实核对★（本仓提交标题形「x86侧批N（…」不被 GITSUB 的 ^[a-z+]+\( 抓到 ⇒ C1 恒报「实动档头＝R未取到」＝在册已知态）"
  [ "$RC" = "9" ] && note "②"
fi

echo
echo "── ③ 回执册三数 ＋ 两条机检 ──"
if [ "$BRSKIP" = "3" ]; then note "③"; echo "  （负例开关跳过）"; else
  R=scripts/verify/_window_receipt.tsv
  echo "  工作树行数=$(wc -l < "$R") ‖ HEAD 行数=$(git show HEAD:"$R" 2>/dev/null | wc -l) ‖ ^R 行数=$(grep -c '^R' "$R")"
  python3 - "$R" <<'PY'
import sys,os,re
rows=[l.rstrip("\n").split("\t") for l in open(sys.argv[1],encoding="utf-8") if l.strip()]
R=[r for r in rows if r[0].startswith("R")]
bad=[r[0] for r in R if len(r)<2 or not r[1].startswith("note/")]
miss=[r[0] for r in R if len(r)>1 and not os.path.isdir(r[1])]
print("  第2字段全路径机检＝%s ‖ 目录在位机检＝%s ‖ R 行=%d" % (bad,miss,len(R)))
PY
fi

echo
echo "── ④ 甲/乙累计（★本 boot 第一次把它做成可跑式★）──"
if [ "$BRSKIP" = "4" ]; then note "④"; echo "  （负例开关跳过）"; else
  python3 - <<'PY'
import re
ov="note/进度/00_总览.md"; o4="note/04_配置改动diff.md"
L=open(ov,encoding="utf-8").read().splitlines()
jia=sum(1 for l in L if l.startswith("- 20") and "x86侧批" in l)
tot=sum(1 for l in L if l.startswith("- 20"))
yi=sum(1 for l in open(o4,encoding="utf-8").read().splitlines() if re.match(r"^## R\d+〔x86〕",l))
first=[l.split("〔")[0] for l in open(o4,encoding="utf-8").read().splitlines() if re.match(r"^## R\d+〔x86〕",l)]
print("  甲案＝条目区主行（行首「- 20」）里带 `x86侧批` 的条数＝%d ‖ 主行总数＝%d（差＝那些条目行未带侧别形）" % (jia,tot))
print("  乙案＝04 段标题匹配 ^## R<号>〔x86〕 的段数＝%d ‖ 首段＝%s" % (yi,first[0] if first else "无"))
print("  ★辖域＝两枚都是「自生效日起的累计」：侧别形自 R481（批131）才起用 ⇒ 不得读成 x86 侧全部批数★")
print("  ★时点＝本批装配前★：若本批会新增一条条目行与一段 04 段标题，则★落笔后各 +1★ ⇒ 落册须两数并打（照 R488「均含本批」口径），不得只印装配前那一枚")
PY
fi

echo
echo "── ⑤ 领先批数（★一次性正则捕获组；distinct 不是批数★）──"
if [ "$BRSKIP" = "5" ]; then note "⑤"; echo "  （负例开关跳过）"; else
  python3 - <<'PY'
import re,glob,subprocess
pat=re.compile(r"（x86侧批(\d{1,4})、")
lite=re.compile(r"（Lite3侧批(\d{1,4})")
ids=[];lt=[]
for f in ["note/进度/00_总览.md"]+sorted(glob.glob("note/进度/总览分册/*.md")):
    t=open(f,encoding="utf-8").read(); ids+=pat.findall(t); lt+=lite.findall(t)
n=sorted(set(map(int,ids)))
ae=subprocess.run(["git","log","--format=%ae"],capture_output=True,text=True).stdout.split()
print("  甲案源 distinct=%d min=%d max=%d ‖ Lite3 侧 distinct=%d" % (len(n),n[0],n[-1],len(set(lt))))
print("  丁案源 x86 侧邮箱条数=%d（★时点重要：提交前跑⇒本批那次提交不在内★）" % sum(1 for x in ae if x=="1904650862@qq.com"))
print("  ★口径限定照 R481 ⑤＝「x86 侧自编号 vs Lite3 侧在本仓条目里的枚数」、不得读成两侧产能比★")
PY
fi

echo
echo "── 附：清盘计数 ──"
if [ "$BRSKIP" = "6" ]; then echo "  （负例开关跳过）"; else
  # ★先清产物再计数★（追加三十一①；R281／R282／R284 三次实测同源：SYN／PYST 跑过 py_compile 会在被检件旁留 __pycache__，
  #   直接 find 计数会把 ws_src 报成 4614 而在册基线是 4580＝差 34 枚产物＝假高读数）
  # ★清产物动作不得被 --skip-chain 门住★（R507 批163 自曝：落册后复跑时串已在早先跑过、本遍走 --skip-chain ⇒ 旧形跳过了清理
  #   ⇒ 读数虚高＝pyc 48／__pycache__ 19／ws_src 4614 对不上 ISO 自报 4580＝假高。清理动作与串跑没跑无关，故写死无条件。）
  docker exec ros1-slam-sim bash -lc 'cd /opt/ros1-slam && find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null; true'
# ★R509 批165 改形＝ROS 进程两口径并报★（R508 批164 自曝：`pgrep -f`／`ps -ef | grep -E` 这类★argv 口径★会把承载命令自身的 bash 数进来＝给出假残影；
#   在册退净用的是★名册＋甲/乙差集★口径，采集件此前混用了 argv 口径 ⇒ 现在★精确名(comm)为主、argv 为辅★，两枚并报、缺一枚即自曝）
  docker exec ros1-slam-sim bash -lc "cd /opt/ros1-slam && echo \"  容器面 pyc=\$(find . -name '*.pyc' | wc -l) ‖ __pycache__ 目录=\$(find . -type d -name '__pycache__' | wc -l) ‖ ROS 进程(★主＝名册 argv＋自剔承载者★)=\$(ps -eo args= | grep -E '(/opt/ros/[^ ]*(roslaunch|rosmaster|roscore|gzserver|move_base|sim_adapter|costmap_2d))' | grep -vE 'grep -E|bash -lc|ps -eo' | wc -l) ‖ ROS 进程(辅＝精确名 comm，★对 shebang 脚本假阴★)=\$(ps -eo comm= | grep -cE '^(roslaunch|rosmaster|rosout|gzserver|gzclient|move_base|costmap_2d|sim_adapter)') ‖ ROS 进程(argv 含前缀)=\$(ps -eo args= | grep -E 'roslaunch|rosmaster|gzserver|move_base_flex' | grep -v 'grep -E' | grep -vc 'batch_readout') ‖ TCP=\$(ss -ltng 2>/dev/null | grep -c LISTEN)\""
  echo "  宿主面 note_files=$(find note -type f | wc -l) ‖ ws_src=$(find ws/src -type f | wc -l) ‖ tooling=$(find tooling -type f | wc -l)（★ws_src 与 ISO 自报串同值才算没被产物污染；★note_files 这枚是「与清理同一条命令」取的瞬时值★——★下一次跑门禁串会在被检件旁留 __pycache__，它必然回潮，不得当末态引用＝r511q 新六：这句此前只在 04 的志文里、现在写进件里★）"
fi

echo
echo "── 附二：留底清单覆盖面（★第六枚·只报不拦＝跑不到才算缺一枚；漏件本身不拦★）──"
if [ "$BRSKIP" = "6" ]; then note "附二"; echo "  （负例开关跳过）"; else
  MCC="$ROOT/note/进度/v7/S1/SCAN集成/证据/_manifest_coverage_check.sh"   # ★R589／批242＝三处路径随批208 搬迁改指（原写死旧路径⇒附二三四恒报「不在位」）★
  if [ -f "$MCC" ]; then
    bash "$MCC" 2>&1 | awk '/漏件|★无清单★|★目录不在位★/ {c++; if (c<=6) print "   " $0} /全部覆盖OK|★有留底目录漏件|小结＝/ {print "  " $0} END {printf "  （★未覆盖OK 的行共 %d 枚、只列前 6；★历史目录不回填＝R506 全仓盘点在册★）\n", c}'
  else note "附二"; echo "  ★覆盖面自检件不在位＝不可现取★"; fi
fi

echo
echo "── 附三：firing 探测（★第九枚·丙案口径＝正文齐而清单缺席跨过一整批★；★只报不拦、不入门禁串★）──"
if [ "$BRSKIP" = "7" ]; then note "附三"; echo "  （负例开关跳过）"; else
  RQ="$ROOT/note/进度/v7/S1/SCAN集成/证据/_rqq_watch.sh"
  if [ -f "$RQ" ]; then
    bash "$RQ" --batch "${BR_BATCH:-?}" --phase "$PHASE" 2>&1 | sed 's/^/  /'
    echo "  ★本枚的 rc 只作登记面读数、不进 MISS、不改任何在册件的 rc★（★红＝人要看一眼：按 R355 同规格扩、不等句★）"
  else note "附三"; echo "  ★firing 探测件不在位＝不可现取★"; fi
fi

echo
echo "── 附四：governance 漂移闸（★第十枚＝真人 2026-10-21 点出四处陈旧后建的格子★；★只报不拦、不入门禁串★）──"
if [ "$BRSKIP" = "8" ]; then note "附四"; echo "  （负例开关跳过）"; else
  GDC="$ROOT/note/进度/v7/S1/SCAN集成/证据/_governance_drift_check.sh"
  if [ -f "$GDC" ]; then
    bash "$GDC" 2>&1 | sed 's/^/  /'
    echo "  ★红档＝施工侧★有写入通道★的面（技能包／仓根 README／门面现行指针节）＝必须归零；旁报＝笔权面（计划书正文）与不回改面（门面历史行）＝只点名★不记红★"
  else note "附四"; echo "  ★漂移闸件不在位＝不可现取★"; fi
fi

echo
echo "── 附五：志面与标题禁形闸（★第十一枚＝本 boot 反引号第八次复发后建的格子★；★只报不拦、不入门禁串★）──"
if [ "$BRSKIP" = "9" ]; then note "附五"; echo "  （负例开关跳过）"; else
  LFC="$ROOT/note/进度/v7/S1/SCAN集成/证据/_ledger_form_check.sh"
  if [ -f "$LFC" ]; then
    bash "$LFC" --tasks 2>&1 | sed 's/^/  /'
    echo "  ★只检★本批新增行★（历史行照留＝不破「旧文不回改」）；提交标题那一枚形检★另跑一次★＝bash 本件 --msg 文件（★写在落盘之前、也在 commit 之前★）"
  else note "附五"; echo "  ★禁形闸件不在位＝不可现取★"; fi
fi

echo
if [ -n "$MISS" ]; then echo "★缺一枚＝本批自曝★ 缺＝$MISS"; exit 5; fi
echo "── 附六：提交声称与成形闸（★第十二枚＝把「声称的动作也是读数」从人眼落成件★；★只报不拦★）──"
if [ "$BRSKIP" = "10" ]; then note "附六"; echo "  （负例开关跳过）"; else
  CC="$ROOT/note/进度/v7/S1/SCAN集成/证据/_commit_title_numstat.sh"   # ★第十二枚（批201 建、批208 搬深后本批修好 ROOT）＝标题声称动作 vs numstat★
  if [ -f "$CC" ]; then
    bash "$CC" HEAD 2>&1 | tail -12 | sed 's/^/  /'
    CRC=${PIPESTATUS[0]:-0}
    echo "  ★本枚判据＝对上一枚提交（HEAD）跑四级＝A 行数级（标题「追加 N 行」须＝numstat additions）‖ B 面级存在性（04 段／00_总览 档头与条目区）‖ C 未动形（声称未动的面不得出现在 numstat）‖ D Y15 三枚尺（首句≤40 ‖ 正文≤400 ‖ 三段且按序；域下限＝5ce95b2＝Y15 落册批，早于它的史文不判）★"
    [ "$CRC" = 0 ] || echo "  ★红档＝上一枚提交有声称未落地或不合 Y15 形＝本批收尾前必须补登记或另起句更正★"
  else note "附六"; echo "  ★第十二枚不在位＝不可现取★"; fi
fi
echo "  ★辖域分句★＝禁形（反引号／ASCII 双引号）＋标题五谓词由第十一枚管；动作对表四级由第十二枚管；★Y15 三枚字数与段数自批247 起＝第十二枚 D 级（域下限 5ce95b2＝Y15 落册批，史文不回填）★"
echo "五枚齐（①—⑤ 全部现取到数）；★本件只报不拦、不进任何件的 rc★"
