#!/usr/bin/env bash
# 01_独立复跑.sh —— 批153（R501，零窗）＝落 r499q 裁定前的★施工侧独立复跑★（不抄判据面的数）
# 姿势＝零窗、只读：不起节点、不 kill、不编译、不 sudo、不 git 写；scratch 全在 /tmp
# 用法＝bash 01_独立复跑.sh [仓根]（缺省＝本件所在 raw 目录上溯六级）
set -u
ROOT="${1:-$(cd "$(dirname "$0")"/../../../../../../.. && pwd)}"   # ★七级上溯：r501→raw→log→证据→SCAN集成→进度→note→仓根★
RAW="$ROOT/note/进度/SCAN集成/证据/log/raw"
W498=$(ls -d "$RAW"/r498_* | head -1)
W499=$(ls -d "$RAW"/r499_* | head -1)
S=$(mktemp -d /tmp/r501_scr.XXXXXX)
echo "★仓根＝$ROOT ‖ scratch＝$S★"

echo
echo "===〔一〕七枚 cm_A.csv 的世界对齐重算（锚 x=y=-50.0、列 476..534 × 行 523..543）==="
echo "  并★同把不带架的那一版也打出来★＝按下标硬比（不乘 shift）＝上一轮栽过的 L-C 同名不同架"
SECT=""
python3 - "$RAW" <<'PY' || SECT="$SECT 〔一〕"
import sys,os,glob,itertools
RAW=sys.argv[1]; ANCHOR=-50.0; TC=(476,534); TR=(523,543)
def load(p):
    ln=open(p,encoding="utf-8",errors="surrogateescape").read().split("\n")
    m=ln[0].split(",")
    return {"w":int(m[1]),"res":float(m[3]),"ox":float(m[4]),"oy":float(m[5]),
            "d":[int(x) for x in ln[1].split(",") if x!=""]}
def at(g,c,r,aligned=True):
    sc=int(round((g["ox"]-ANCHOR)/g["res"])); sr=int(round((g["oy"]-ANCHOR)/g["res"]))
    fc,fr=(c-sc,r-sr) if aligned else (c,r)
    h=len(g["d"])//g["w"]
    return g["d"][fr*g["w"]+fc] if (0<=fc<g["w"] and 0<=fr<h) else None
W=[d for d in sorted(glob.glob(RAW+"/r4*")) if os.path.exists(d+"/run/csv/cm_A.csv")]
if len(W) < 7:
    print("    ★面不可信：只找到 %d 枚 cm_A.csv（应 ≥7）⇒ 本段任何 0 一律不作数★"%len(W)); sys.exit(7)
G={d:load(d+"/run/csv/cm_A.csv") for d in W}
print("    面＝%d 枚 cm_A.csv"%len(W))
for tag,al in (("★aligned（世界对齐）★",True),("raw（同下标硬比）",False)):
    print("  ── %s ──"%tag)
    for d in W:
        t=[at(G[d],c,r,al) for r in range(TR[0],TR[1]+1) for c in range(TC[0],TC[1]+1)]
        print("    %s origin=%.2f n_T=%d X_A[P100]=%d X_A[P90]=%d v99=%d"%(
            os.path.basename(d)[:4],G[d]["ox"],len(t),
            sum(1 for v in t if v==100),sum(1 for v in t if v is not None and v>=90),
            sum(1 for v in t if v==99)))
    if al:
        TA={d:[at(G[d],c,r,al) for r in range(TR[0],TR[1]+1) for c in range(TC[0],TC[1]+1)] for d in W}
        nz=0; tot=0
        for a,b in itertools.combinations(W,2):
            for f in (lambda v:v==100, lambda v:v is not None and v>=90, lambda v:v==99):
                tot+=1
                sa={i for i,v in enumerate(TA[a]) if f(v)}; sb={i for i,v in enumerate(TA[b]) if f(v)}
                dd=len(sa-sb)+len(sb-sa)
                if dd: nz+=1; print("      ★非零★",os.path.basename(a)[:4],os.path.basename(b)[:4],dd)
        print("    两两真对称差（★不除 2★）＝%d 对 × 3 谓词＝%d 枚 ⇒ 非零枚数＝★%d★"%(len(list(itertools.combinations(W,2))),tot,nz))
    else:
        print("    ⇒ 上面那行与 aligned 不一致＝★同一枚 T 在两种架下给出两套数；不带架的复现本身是错的★")
PY

echo
echo "===〔二〕cm_C 跨模块第四通路（T 内 v==100 ‖ 全图 v==100）==="
python3 - "$RAW" <<'PY' || SECT="$SECT 〔二〕"
import sys,os,glob
RAW=sys.argv[1]; ANCHOR=-50.0; TC=(476,534); TR=(523,543)
def load(p):
    ln=open(p,encoding="utf-8",errors="surrogateescape").read().split("\n")
    m=ln[0].split(",")
    return {"w":int(m[1]),"res":float(m[3]),"ox":float(m[4]),"oy":float(m[5]),"d":[int(x) for x in ln[1].split(",") if x!=""]}
CW=[d for d in sorted(glob.glob(RAW+"/r4*")) if os.path.exists(d+"/run/csv/cm_C.csv")]
if len(CW) < 7:
    print("  ★面不可信：只找到 %d 枚 cm_C.csv（应 ≥7）⇒ 本段不作数★"%len(CW)); sys.exit(7)
print("  面＝%d 枚 cm_C.csv"%len(CW))
for d in CW:
    g=load(d+"/run/csv/cm_C.csv"); h=len(g["d"])//g["w"]
    sc=int(round((g["ox"]-ANCHOR)/g["res"])); sr=int(round((g["oy"]-ANCHOR)/g["res"]))
    t=[g["d"][(r-sr)*g["w"]+(c-sc)] if (0<=c-sc<g["w"] and 0<=r-sr<h) else None for r in range(TR[0],TR[1]+1) for c in range(TC[0],TC[1]+1)]
    print("  %s T内v100=%d ‖ 全图v100=%d"%(os.path.basename(d)[:4],sum(1 for v in t if v==100),sum(1 for v in g["d"] if v==100)))
PY

echo
echo "===〔三〕同秒连跑两遍＝让路件被下一遍静默覆盖（★本批最该记的一格★）==="
EX="$S/ex.sh"; sed -n '31,41p;84p' "$W498/00_r498_window.sh" > "$EX"
( R="$S"; RUN="$R/run"; DUMP="$RUN/dump"; CSV="$RUN/csv"
  printf 'STATE=REF s1\nSTATE=A s1\nSTATE=Ap s1\nSTATE=C s1\n' > "$R/00_window_log.txt"; mkdir -p "$RUN/dump"
  . "$EX"
  printf 'STATE=REF s2\nSTATE=A s2\n' > "$R/00_window_log.txt"; mkdir -p "$RUN"
  . "$EX"
  echo "  让路件现读＝"
  for f in "$R"/.prev_*_00_window_log.txt; do
    echo "    $(basename "$f") ‖ s1 行=$(grep -ac s1 "$f") ‖ s2 行=$(grep -ac s2 "$f") ‖ ^STATE=$(grep -ac '^STATE=' "$f")"
  done
  echo "  ★s1 行＝0 ⇒ 第一遍那枚 s1×4 的证据已被第二遍的同名 mv 静默覆盖★（改名戳只到秒）"
  echo "  .prev_ 总数=$(ls -1d "$R"/.prev_* | wc -l)（★两遍共让开 2＋2 枚，总数却不增＝撞名实锤★）"
  echo "  嵌套目录现读＝$(find "$R" -maxdepth 3 -type d -name 'run' | sed "s#$R#.#" | tr '\n' ' ')"
)

echo
echo "===〔四〕三处互斥字面 ‖ 消费面缺席 ===="
printf '  甲（断言两枚独立来源）＝%s\n' "$(sed -n '75p' "$W498/pub_rect_probe_r498.py" | cut -c1-96)"
printf '  乙（自陈同源两派生）＝%s\n' "$(sed -n '76p' "$W498/pub_rect_probe_r498.py" | cut -c1-96)"
printf '  丙（缺② 自打＝成立）＝%s\n' "$(sed -n '202,203p' "$W498/_cmp_r498.py" | tr '\n' ' ' | cut -c1-96)"
printf '  对照件读 sub_elev／SECSUB 的命中枚数＝%s（★0＝第三枚未进任何合取项★）\n' "$(grep -c -E 'sub_elev|SECSUB' "$W498/_cmp_r498.py")"

echo
echo "===〔五〕遍次这一格的三个载体细节 ===="
printf '  窗志含遍次的行号现读＝%s（★在册 04 与 W498/06 都写「第 12 行」＝引错★）\n' "$(grep -an '步0 重跑让路' "$W498/00_window_log.txt" | head -1 | cut -d: -f1)"
printf '  第 12 行实为＝%s\n' "$(sed -n '12p' "$W498/00_window_log.txt" | cut -c25-88)"
for f in 05_cmp.txt 07_复读.txt run/sub_elev_C.txt run/pub_C.txt; do
  printf '  读数件 %s 含「遍」=%s ‖ 含「共几遍」=%s\n' "$f" "$(grep -ac '遍' "$W498/$f")" "$(grep -ac '共几遍' "$W498/$f")"
done
printf '  件内次序＝第 19 行 mkdir(%s) 早于第 37—39 行让路(%s) ⇒ ★首遍必有 run 可让、与跨 boot 无关★\n' \
  "$(sed -n '19p' "$W498/00_r498_window.sh" | cut -c1-14)" "$(sed -n '37p' "$W498/00_r498_window.sh" | cut -c1-14)"
printf '  .runs 自增（第 40 行）早于预检（第 86 行）＝%s ‖ %s ⇒ 遍次数含未起窗的尝试\n' \
  "$(sed -n '40p' "$W498/00_r498_window.sh" | cut -c1-20)" "$(sed -n '86p' "$W498/00_r498_window.sh" | cut -c1-30)"

echo
echo "===〔六〕订阅腿的字节级取证（坏字节来自截断、不来自 tee）==="
printf '  全件 run/sub_elev_C.txt 字节=%s ‖ 整件 UTF-8 解码=%s\n' "$(wc -c < "$W498/run/sub_elev_C.txt")" \
  "$(python3 -c "d=open('$W498/run/sub_elev_C.txt','rb').read(); d.decode('utf-8'); print('OK')")"
python3 -c "
l=open('$W498/00_window_log.txt','rb').read().split(b'\n')[38]
print('  窗志第 39 行字节=%d ‖ 该行解码='%len(l), end='')
try: l.decode('utf-8'); print('OK')
except Exception as e: print('FAIL（截断在一个码点中间）＝%s'%e)
"
printf '  窗脚本第 162 行（★截断源★）＝%s\n' "$(sed -n '162p' "$W498/00_r498_window.sh" | grep -o 'cut -c[0-9-]*')"

echo
echo "===〔七〕复读件宿主面重跑（★现算、非手抄★）＋ --swap 分支的退出码 ===="
SRC=$(ls -d "$RAW"/r472_* | xargs basename); LIV=$(ls -d "$W498" | xargs basename)
python3 "$W498/01_复读件_r498.py" "$ROOT" "$SRC" "$LIV" > "$S/pos.txt" 2>&1; RC=$?
printf '  正测退出码＝%s ‖ 差值枚＝0 的行数＝%s ‖ 复读时刻＝%s\n' "$RC" "$(grep -ac '差值枚＝0' "$S/pos.txt")" \
  "$(grep -a '复读时刻' "$S/pos.txt" | grep -o '[0-9:.]* UTC[^‖]*' | head -1 | cut -c1-46)"
python3 "$W498/01_复读件_r498.py" "$ROOT" "$SRC" "$LIV" --swap > "$S/swap.txt" 2>&1; RC2=$?
printf '  ★--swap 分支：进程退出码＝%s，而打印串里写「被引件不在位 ⇒ rc_复读=5」＝红不在退出码★\n' "$RC2"
grep -a '被引件不在位' "$S/swap.txt" | head -1 | cut -c1-90

echo
echo "===〔八〕追加八十三 是不是空号（登记面新格）==="
printf '  05 里 ^## 追加八十三 命中＝%s ‖ 05 全文提「追加八十三」＝%s ‖ 04 提「追加八十三」＝%s\n' \
  "$(grep -c '^## 追加八十三' "$ROOT/note/05_复验作业单.md")" \
  "$(grep -c '追加八十三' "$ROOT/note/05_复验作业单.md")" \
  "$(grep -c '追加八十三' "$ROOT/note/04_配置改动diff.md")"
[ -d "$W498" ] || SECT="$SECT W498不在位"
[ -d "$W499" ] || SECT="$SECT W499不在位"
rm -rf "$S"
if [ -n "$SECT" ]; then echo "★有段没跑到＝本批自曝★ 缺＝$SECT"; exit 5; fi
echo "★全段跑到、scratch 已清★"
