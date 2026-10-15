set -u
D=$(cd "$(dirname "$0")" && pwd)
R=$(cd "$D/../../../.." && pwd)
cd "$R"
PAT="$(printf '%s' /home)""$(printf '%s' /ysc)"
echo "# R333 豁免面复验定稿跑（第二段·修正对照乙）｜时刻 $(date '+%F %T')｜身份 $(id -un)｜跑法＝先落脚本再跑、式子一律从表头第 8 行现取、T 路径改指对照件｜★首跑 ROOT 取多一层少一层之误（同 R332 探针 ROOT 那一族、本族第二次）已修为向上四层、下面读数为改后现跑"
sed -n '8p' tooling/_forbid_str_exempt.tsv | sed 's/^#   //' > "$D/09_复验式_现取.sh"
echo "表自禁串命中=$(grep -c "$PAT" tooling/_forbid_str_exempt.tsv)（应为 0）｜表行数=$(wc -l < tooling/_forbid_str_exempt.tsv)｜数据行=$(grep -vc '^#' tooling/_forbid_str_exempt.tsv)"
echo
echo "① 新式基线（期望 新增=0 陈旧=0 处数不符=0 rc=0）"
bash "$D/09_复验式_现取.sh"; echo "rc=$?"
echo
echo "② 旧式（git show HEAD: 版、带 systemd 过滤）在正文已落地的盘上（期望 新增=7 rc=5）"
git show HEAD:tooling/_forbid_str_exempt.tsv | sed -n '8p' | sed 's/^#   //' > "$D/10_复验式_旧式取自HEAD.sh"
bash "$D/10_复验式_旧式取自HEAD.sh"; echo "rc=$?"
echo
echo "③ 面尺寸：盘上命中件=$(grep -rl "$PAT" tooling/ | grep -c .) 行=$(grep -rc "$PAT" tooling/ | awk -F: '{s+=$2} END{print s}')｜表内数据行=$(grep -vc '^#' tooling/_forbid_str_exempt.tsv)"
echo
echo "对照乙（先拷全表 16 行、再补一条盘上不存在的键 tooling/systemd/ghost_R333.service 处数写 1 ⇒ 期望 新增=0 陈旧=1 处数不符=0 rc=0＝只报不拦兑现）"
cp tooling/_forbid_str_exempt.tsv "$D/12_对照乙.tsv"
printf 'tooling/systemd/ghost_R333.service\t1\t甲-镜像随真身\tR333 阳性对照用键、盘上无此件（期望只报陈旧、不拦）\n' >> "$D/12_对照乙.tsv"
echo "对照乙文件：数据行数=$(grep -vc '^#' "$D/12_对照乙.tsv")（基线 16＋1＝17）；补入那行的键与处数=$(grep '^tooling/systemd/ghost' "$D/12_对照乙.tsv" | cut -f1,2)"
sed "s|T=tooling/_forbid_str_exempt.tsv|T=$D/12_对照乙.tsv|" "$D/09_复验式_现取.sh" > "$D/12_跑乙.sh"
bash "$D/12_跑乙.sh"; echo "rc=$?"
echo
echo "★本路自曝（不删前一次读数、只在此更正）＝第一次跑对照乙时「printf 键与处数」那半句漏了重定向符号、只打到屏幕没进文件 ⇒ 那次对照件只剩 1 行、跑出 新增=16 陈旧=1 rc=5，没单独证到「陈旧⇒只报不拦」；第一次读数原样留在 15 号文件与 19 号文件里。同段第二条自曝＝在同一条双引号 echo 里放了反引号 ⇒ 反引号内容被当命令执行（打出两行 command not found、正文被吃掉一截）＝动作㉕ 第十八次；本文件改用「先落脚本、再 bash 跑」的写法。"
