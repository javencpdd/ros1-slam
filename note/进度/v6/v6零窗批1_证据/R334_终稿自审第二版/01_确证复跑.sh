set -u
cd /home/ros1-slam
R=note/进度/SCAN集成/证据/log/raw
echo "# R334 确证复跑（★独立一路、不接前档结论；先核清单 sha256 再 grep 字段）｜时刻 $(date '+%F %T')｜全程只读、未跑节点、无 sudo"
for d in r138_p4_newshape_r1_pass r149_p1p2_newshape r150_p3_triple_newshape r85_b1_345 r154_m52_live_newshape r155_m52_live_goalB r156_m52_live_goalC r331_breach_first_live r332_x2_fourcells; do
  if [ -d "$R/$d" ]; then
    if [ -f "$R/$d/_sha256_manifest.txt" ]; then
      cd "$R/$d"; ok=$(sha256sum -c _sha256_manifest.txt 2>/dev/null | grep -c ': OK$'); bad=$(sha256sum -c _sha256_manifest.txt 2>/dev/null | grep -vc ': OK$'); cd /home/ros1-slam
      printf '%-32s 清单在位 OK=%s 非OK=%s\n' "$d" "$ok" "$bad"
    else
      printf '%-32s ★无清单（按令⑤＝不补、只能记 N3）\n' "$d"
    fi
  else
    printf '%-32s ★目录不存在\n' "$d"
  fi
done
echo
echo "## 逐条字段复跑（判据式取自 ap 稿、值取原件现跑）"
echo "#1 P1 收敛 ｜ $(grep -rhoE '判据① 结论 = [A-Z]+' "$R/r149_p1p2_newshape" 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
echo "#2 P2 出轨迹 ｜ $(grep -rhoE '判据② 结论 = [A-Z]+' "$R/r149_p1p2_newshape" 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
echo "#3 到点偏差 ｜ $(grep -rhoE '判据③[^|]{0,46}' "$R/r150_p3_triple_newshape" 2>/dev/null | head -2 | tr '\n' ' ')"
echo "#4 全程时延 ｜ $(grep -rhoE '判据④[^|]{0,40}' "$R/r150_p3_triple_newshape" 2>/dev/null | head -2 | tr '\n' ' ')"
echo "#5 三连红线 ｜ $(grep -rhoE 'P3 判据③ 结论 = PASS' "$R/r150_p3_triple_newshape" 2>/dev/null | grep -c .) 处"
echo "#6 避让门值 ｜ $(grep -rhoE 'avoid@[0-9.]+=[0-9.]+' "$R/r138_p4_newshape_r1_pass" 2>/dev/null | sort -u | tr '\n' ' ')"
echo "#7 breach  ｜ 历史 $(grep -rhoE 'breach@[0-9.]+=[0-9.]+' "$R/r138_p4_newshape_r1_pass" 2>/dev/null | sort -u | tr '\n' ' ')｜R331 整链首读 $(grep -rhoE '"breach_gate": *[0-9.]+|"breach": *(true|false)|breach@[0-9.]+=[0-9.]+' "$R/r331_breach_first_live" 2>/dev/null | sort -u | tr '\n' ' ')"
echo "#8 A① Hz   ｜ $(grep -rhoE 'A①[^|]{0,26}' "$R/r85_b1_345" 2>/dev/null | sort -u | head -2 | tr '\n' ' ')"
echo "#9 A② Hz   ｜ $(grep -rhoE 'A②[^|]{0,26}' "$R/r85_b1_345" 2>/dev/null | sort -u | head -2 | tr '\n' ' ')"
echo "#18 唯一写者｜ $(grep -rhoE 'mux 命中=[0-9]+|写者总数=[0-9]+' "$R/r150_p3_triple_newshape" 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
echo "#23 outcome ｜ $(for x in r154_m52_live_newshape r155_m52_live_goalB r156_m52_live_goalC; do printf '%s=%s ' "$x" "$(grep -rho 'outcome=succeeded' "$R/$x" 2>/dev/null | grep -c .)"; done)"
echo "#24 dt≤60s ｜ $(grep -rhoE '总时长=[0-9.]+s|dt=[0-9.]+s' "$R/r154_m52_live_newshape" "$R/r155_m52_live_goalB" "$R/r156_m52_live_goalC" 2>/dev/null | sort -u | tr '\n' ' ')"
echo "#22 X2 四格｜ R332 两扇目录内 cost 原始件：$(find "$R/r332_x2_fourcells" -name 'cost_g*.txt' 2>/dev/null | grep -c .) 件；data 行数合计=$(grep -rho '^data:' "$R/r332_x2_fourcells" 2>/dev/null | grep -c .)；VERDICT 行＝$(grep -rhoE 'X2_FOURCELL VERDICT rc=[0-9]+' "$R/r332_x2_fourcells" 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
echo
echo "## 目录全貌与分母（现跑）"
echo "raw 目录数=$(find "$R" -maxdepth 1 -type d | tail -n +2 | grep -c .) 其中无清单目录=$(for x in "$R"/*/; do [ -f "${x}_sha256_manifest.txt" ] || echo x; done | grep -c .)"
echo "ap 表体行数=$(grep -cE '^\| *[0-9]+ *\|' note/进度/SCAN集成/设计/ap_在册判据清单_A54扫完定义输入.md)｜丙=#27—#32"
