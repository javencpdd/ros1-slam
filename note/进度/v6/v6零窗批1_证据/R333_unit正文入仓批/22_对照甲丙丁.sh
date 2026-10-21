set -u
D=$(cd "$(dirname "$0")" && pwd)
R=$(cd "$D/../../../.." && pwd)
cd "$R"
echo
echo "## 补齐三段阳性对照（同一式子、T 改指对照件）｜时刻 $(date '+%F %T')"
echo "对照甲（从定稿表里删掉一行 systemd 键＝模拟表外真命中新件，期望 新增=1 rc=5）"
grep -v '^tooling/systemd/lite3-monitor\.service' tooling/_forbid_str_exempt.tsv > "$D/11_对照甲.tsv"
echo "  对照甲数据行数=$(grep -vc '^#' "$D/11_对照甲.tsv")（基线 16－1＝15）"
sed "s|T=tooling/_forbid_str_exempt.tsv|T=$D/11_对照甲.tsv|" "$D/09_复验式_现取.sh" > "$D/11_跑甲.sh"
bash "$D/11_跑甲.sh"; echo "  rc=$?"
echo "对照丙（把一处 1 改 9＝处数不符，期望 rc=5）"
sed "s|^tooling/systemd/transfer_ros2.service\t1\t|tooling/systemd/transfer_ros2.service\t9\t|" tooling/_forbid_str_exempt.tsv > "$D/13_对照丙.tsv"
echo "  改后该行前两字段=$(grep '^tooling/systemd/transfer_ros2' "$D/13_对照丙.tsv" | cut -f1,2)"
sed "s|T=tooling/_forbid_str_exempt.tsv|T=$D/13_对照丙.tsv|" "$D/09_复验式_现取.sh" > "$D/13_跑丙.sh"
bash "$D/13_跑丙.sh"; echo "  rc=$?"
echo "对照丁（T 指向不存在的表，期望 rc=9 不判）"
sed "s|T=tooling/_forbid_str_exempt.tsv|T=$D/无此件.tsv|" "$D/09_复验式_现取.sh" > "$D/14_跑丁.sh"
bash "$D/14_跑丁.sh"; echo "  rc=$?"
echo "收尾核＝定稿表未被对照件改动：行数=$(wc -l < tooling/_forbid_str_exempt.tsv) 数据行=$(grep -vc '^#' tooling/_forbid_str_exempt.tsv)；对照件全在留底目录、tooling/ 内零新增件（tooling 文件数现跑=$(find tooling -type f | grep -c .)）"
