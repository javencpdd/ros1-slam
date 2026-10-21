set -u
cd /home/ros1-slam
D=note/进度/v6/v6零窗批1_证据/R333_unit正文入仓批
echo "# R333 收尾跑序（05 追加三十一①）｜起 $(date '+%F %T')"
echo "## 清前三条核对式（缺一条就不清、公开留名）"
K1=$(git ls-files | grep -cE '\.pyc$|__pycache__')
K2=$(git ls-files | xargs -r grep -lE '__pycache__|\.pyc' 2>/dev/null | grep -c .)
K3=$(find ws/src/40_navigation/lite3_scan/30_3rd -name '*.pyc' 2>/dev/null | grep -c .)
PC=$(find . -path ./.git -prune -o -type d -name __pycache__ -print 2>/dev/null | grep -c .)
echo "K1 git 跟踪内的 pyc/pycache=$K1 ｜ K2 在册清单引用 pyc 的行所在件数=$K2 ｜ K3 30_3rd 内 .pyc=$K3 ｜ 产物目录在场数=$PC"
if [ "$K1" = 0 ] && [ "$K3" = 0 ]; then
  find . -path ./.git -prune -o -type d -name __pycache__ -print0 2>/dev/null | xargs -0 -r rm -rf
  echo "已清（K1=0 且 K3=0 达标后执行在册手工动作）；清后产物目录=$(find . -path ./.git -prune -o -type d -name __pycache__ -print 2>/dev/null | grep -c .)"
else
  echo "★三条核对式未全过 ⇒ 本批不清盘、公开留名（在册规矩）"
fi
echo
echo "## 契约件（清盘后跑）"
bash scripts/verify/v_w3_terrain_design_contract.sh > "$D/38_契约件.txt" 2>&1
CR=$?
echo "契约件 rc=$CR ｜ $(grep -E 'PASS=|结论' "$D/38_契约件.txt" | tail -1)"
echo
echo "## 30_3rd 计数与 ISO 面"
echo "30_3rd 件数=$(find ws/src/40_navigation/lite3_scan/30_3rd -maxdepth 1 -type f 2>/dev/null | grep -c .)"
echo
echo "## VCC 与 规矩⑨ 逐件明细（在册调用式＝双 source）"
( set +u; unset ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_IP ROS_HOSTNAME 2>/dev/null || true
  export PYTHONNOUSERSITE=1
  source /opt/ros/noetic/setup.bash >/dev/null 2>&1
  source ws/devel/setup.bash >/dev/null 2>&1
  bash scripts/verify/v_citation_check.sh > "$D/39_VCC明细.txt" 2>&1; echo "VCC rc=$? ｜ $(grep -E '单点引用=' "$D/39_VCC明细.txt" | tail -1)"
  bash scripts/verify/v_claim_field_audit.py > "$D/40_规矩九明细.txt" 2>&1; echo "规矩⑨ rc=$? ｜ $(grep -E '含结论强度词行=|VERDICT' "$D/40_规矩九明细.txt" | tail -2 | tr '\n' ' ')"
  echo "账构与 LCB 见串；此处补跑 tooling 豁免面复验（式子从表头现取）"
  sed -n '8p' tooling/_forbid_str_exempt.tsv | sed 's/^#   //' > "$D/09_复验式_现取.sh"
  bash "$D/09_复验式_现取.sh"; echo "豁免面 rc=$?" )
echo
echo "## 改后反扫（证零新号形）"
for f in tooling/_forbid_str_exempt.tsv tooling/_sha256_manifest.txt note/03_外部文件来源清单.md note/04_配置改动diff.md note/00_README.md "note/进度/00_总览.md"; do
  out=$(bash scripts/verify/v_citation_refscan.sh "$f" 2>&1); r=$?
  n=$(printf '%s\n' "$out" | grep -oE '合计行号引用=[0-9]+' | head -1)
  echo "$f ｜ rc=$r ｜ $n"
done
echo
echo "## 仓态与面数（现跑）"
echo "串成员=$(grep -vc '^#' scripts/verify/gate_chain.txt) 面=syn $(grep -vE '^ *(#|$)' scripts/verify/syn_face.txt | grep -c .)/pyst $(grep -vE '^ *(#|$)' scripts/verify/pyst_face.txt | grep -c .)/parity $(grep -vE '^ *(#|$)' scripts/verify/parity_face.txt | grep -c .)/claim $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt | grep -c .)"
echo "窗回执非注释行=$(grep -vE '^ *(#|$)' scripts/verify/_window_receipt.tsv | grep -c .)"
echo "tooling 文件数=$(find tooling -type f | grep -c .) 顶层清单条目=$(grep -vc '^#' tooling/_sha256_manifest.txt) 顶层自验 rc=$(cd tooling && sha256sum -c _sha256_manifest.txt >/dev/null 2>&1; echo $?)"
echo "gate_chain.txt diff 行数=$(git diff --numstat scripts/verify/gate_chain.txt | grep -c .)"
echo "load1=$(awk '{print $1}' /proc/loadavg) 止 $(date '+%F %T')"
