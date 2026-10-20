#!/usr/bin/env bash
# _batch_readout.sh —— ★批末读数面五枚的可复跑采集件★（登记面纪律件、★不是闸★：不进任何件的 rc、不入门禁串、不改成员与语义）
# 起意＝追加七十九③ 立的「批末读数面固定五枚、缺一即自曝」连犯四次（批137—139 漏 P8、批142 漏两枚、批150 取的是提交前的丁案源、
#         ④ 甲/乙累计 自批143 起一律报"未动"却从没跑过式）⇒ 把"当次同调用现取"从脑子里的动作变成一件能跑、会翻红的东西。
# 用法＝bash _batch_readout.sh [--phase pre|post] [--skip-chain] [--BR_SKIP=<①..⑤>]
#   --phase post 只影响⑤的措辞（★提交前跑⇒本批那次提交不在丁案源内★）；--skip-chain 用于同一批内早先已跑过串的那一遍；
#   --BR_SKIP=3 是★负例开关★：故意让第③枚取不到 ⇒ 本件必须翻红（rc=5）并点名缺哪枚。
# 面（★两面分别写死★）＝①②③④⑤ 全在★宿主面★跑（账构与引用件必须显式传根 VLB_ROOT／VCC_ROOT，容器内无 .git ⇒ 账构恒 rc=9）；
#              容器面只跑门禁串与清盘计数。
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
PHASE=pre; SKIPCHAIN=0; BRSKIP=""
while [ $# -gt 0 ]; do
  case "$1" in
    --phase) PHASE="$2"; shift 2 ;;
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
    awk '/^ *(SYN|GTC|ISO|DC|PYST|VAR|PARITY|账构|VCC|规矩⑨) +rc=/ {printf "  %s rc=%s\n", $1, $2} /窗数回执/ {print "  窗数回执＝"$0} /面：/ {print "  "$0}'
fi
BASE=note/进度/SCAN集成/证据/log/raw/r447_真人六句到签落地批/04_baseline_note71w40_r447.tsv
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
  docker exec ros1-slam-sim bash -lc "cd /opt/ros1-slam && echo \"  容器面 pyc=\$(find . -name '*.pyc' | wc -l) ‖ __pycache__ 目录=\$(find . -type d -name '__pycache__' | wc -l) ‖ ROS 进程(精确名)=\$(ps -eo comm= | grep -cE '^(roslaunch|rosmaster|roscore|rosout|gzserver|gzclient|move_base|costmap_2d|sim_adapter)') ‖ ROS 进程(argv 含前缀)=\$(ps -eo args= | grep -E 'roslaunch|rosmaster|gzserver|move_base_flex' | grep -v 'grep -E' | grep -vc 'batch_readout') ‖ TCP=\$(ss -ltng 2>/dev/null | grep -c LISTEN)\""
  echo "  宿主面 note_files=$(find note -type f | wc -l) ‖ ws_src=$(find ws/src -type f | wc -l) ‖ tooling=$(find tooling -type f | wc -l)（★ws_src 与 ISO 自报串同值才算没被产物污染★）"
fi

echo
echo "── 附二：留底清单覆盖面（★第六枚·只报不拦＝跑不到才算缺一枚；漏件本身不拦★）──"
if [ "$BRSKIP" = "6" ]; then note "附二"; echo "  （负例开关跳过）"; else
  MCC="$ROOT/note/进度/SCAN集成/证据/_manifest_coverage_check.sh"
  if [ -f "$MCC" ]; then
    bash "$MCC" 2>&1 | awk '/漏件|★无清单★|★目录不在位★/ {c++; if (c<=6) print "   " $0} /全部覆盖OK|★有留底目录漏件|小结＝/ {print "  " $0} END {printf "  （★未覆盖OK 的行共 %d 枚、只列前 6；★历史目录不回填＝R506 全仓盘点在册★）\n", c}'
  else note "附二"; echo "  ★覆盖面自检件不在位＝不可现取★"; fi
fi

echo
if [ -n "$MISS" ]; then echo "★缺一枚＝本批自曝★ 缺＝$MISS"; exit 5; fi
echo "五枚齐（①—⑤ 全部现取到数）；★本件只报不拦、不进任何件的 rc★"
