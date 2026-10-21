set -u
cd /home/ros1-slam
D=note/进度/v6/v6零窗批1_证据/R333_unit正文入仓批
B=note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv
echo "# R333 收尾定稿记录（含一次调用式自曝）｜时刻 $(date '+%F %T')｜身份 $(id -un)"
echo "## 一、清盘三条核对式（★K2 首跑式子取错、此处用正确式复跑）"
echo "K1 git 跟踪内 .pyc／__pycache__ 条目=$(git ls-files | grep -cE '\.pyc$|__pycache__')（应为 0）"
echo "K2 首跑错式＝git ls-files | xargs grep -l '__pycache__|\\.pyc' → 得 15（★病＝把志文里\"谈 .pyc 的句子\"当成\"清单收录了 .pyc\"＝假阳性 15）"
echo "K2 正确式＝四张面清单非注释条目里含 .pyc／__pycache__ 的行数：$(cat scripts/verify/syn_face.txt scripts/verify/pyst_face.txt scripts/verify/parity_face.txt scripts/verify/claim_face.txt | grep -vE '^ *(#|$)' | grep -cE '\.pyc|__pycache__')（应为 0）"
echo "K3 ws/src/30_3rd 内 .pyc=$(find ws/src/30_3rd -name '*.pyc' 2>/dev/null | grep -c .)（应为 0）"
echo "清盘时刻＝04:56:20 前后（在册手工动作、K1=0 且 K3=0 已达标；★K2 那一格当时用的是错式、值为 15＝假阳性 ⇒ 本行以正确式复跑＝0，三条现全过＝清盘这个动作按正确式复算仍是达标的，但★\"先过三条再删\"这个顺序我当时没照做到，属流程偏差、照实记）"
echo "清后 __pycache__ 目录在场数=$(find . -path ./.git -prune -o -type d -name __pycache__ -print | grep -c .)（R332 在册产物目录数＝19，本批清前＝23＝覆盖面与运行次数变化、非新疑点）"
echo
echo "## 二、★调用式自曝（一次跑法把四件跑错，四个错读数方向全是"报不出数"或"跨面混引"、无一个被当成绿）"
echo "①契约件首跑＝件名写成 scripts/verify/v_w3_terrain_design_contract.sh → rc=127（命令不存在）；真身＝ws/src/95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh → 现跑 rc=0 ｜ $(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+ ABSENT=[0-9]+' "$D/38_契约件.txt" | tail -1)（★与在册逐值同）"
echo "②VCC 首跑＝未喂门用面 → 得 文档=43 单点引用=530 OK=429 AMBER=31（＝件内缺省面、与存量表不同面＝★在册写死的"跨面混引"那一格）；按在册调用式（运行器同款＝显式喂 claim_face.txt）重跑 → $(grep -E '单点引用=' "$D/39_VCC明细.txt" | tail -1)"
echo "③规矩⑨ 首跑＝bash 直跑 .py → rc=2；正确式＝python3 件 → $(grep -E '含结论强度词行=' "$D/40_规矩九明细.txt" | tail -1)｜$(grep -E 'VERDICT' "$D/40_规矩九明细.txt" | tail -1)"
echo "④LCB 首跑＝缺 --check 与基线参数 → rc=9 前置缺失；正确式＝--check 基线＋70 面条目 → rc=0 ｜ $(grep -E 'VERDICT' "$D/41_LCB.txt" | tail -1)（同值项数=$(grep -c '同值' "$D/41_LCB.txt")）"
echo "⑤30_3rd 首跑＝路径写成 ws/src/40_navigation/lite3_scan/30_3rd → 0；真身＝ws/src/30_3rd 全树文件数=$(find ws/src/30_3rd -type f | grep -c .)（★与在册 49 同）"
echo
echo "## 三、批末十件串与仓态（引用同批留底 36 号文件、不重打）"
grep -E '^(SYN|GTC|ISO|DC|PYST|VAR|PARITY|账构|VCC|规矩⑨)' "$D/36_十件串_批末.txt" | cut -c1-60
echo "串成员=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt) 窗回执=$(grep -vE '^ *(#|$)' scripts/verify/_window_receipt.tsv | grep -c .) 面=$(cat scripts/verify/syn_face.txt scripts/verify/pyst_face.txt scripts/verify/parity_face.txt scripts/verify/claim_face.txt | grep -vcE '^ *(#|$)')（四张合计、逐张＝87/57/54/70）"
echo "load1=$(awk '{print $1}' /proc/loadavg)"
