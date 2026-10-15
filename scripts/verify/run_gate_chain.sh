#!/usr/bin/env bash
# run_gate_chain.sh —— 门禁串的**只报不判**运行器（R274，#130；施工侧-小明）
#
# 为何存在：`05` 追十六把"八件串当代读数"写成散文，而成员名单此前**没有文件**——
#   佳文原话「下一位无法原样复现」。R273 建了 gate_chain.txt（成员）＋syn_face.txt／pyst_face.txt（面），
#   本件把"按名单跑一遍、逐件打印 rc 与关键行"变成一条命令，于是那句散文有了可复现的载体。
#
# ★本件**不做任何判定**：不内置"什么算绿"，不比较历史值，不决定谁的 rc 可接受
#   （**R278 佳文裁定＝丙修正调用式**：ISO 真态为 `rc=0`（须带 `PYTHONNOUSERSITE=1`，与 `env.sh`、
#    `v_window_preflight.sh` 同姿势；此前长期报 `rc=1` 是本件少这个 export 所致，非隔离不过）；
#    DC 的 `rc=1` 仍属在册已知态，见 `05` 追加二十三——**看到具体行才可定性**，本件只把行打出来）。
#   所以本件**退出码恒 0**；真要拦人，靠的是各件自身的 rc 与在册调用式，不是本件。
#   ⇒ 本件**不是第九件门禁件**，也不改任何件的 rc 语义；它只是一个跑法。
#
# 用法：bash scripts/verify/run_gate_chain.sh          # 跑全部八件
#       GATE_ONLY=ISO,DC bash scripts/verify/run_gate_chain.sh   # 只跑名单里的某几件（代号逗号分隔）
# 依赖：gate_chain.txt（成员名单）、syn_face.txt、pyst_face.txt、parity_face.txt（面清单，R272／R273 建）
set -u
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
CHAIN="$ROOT/scripts/verify/gate_chain.txt"

if [ ! -f "$CHAIN" ]; then
  echo "[前置缺失] 成员名单不在位：$CHAIN（⇒ 不猜名单、不判绿，退出 9）"
  exit 9
fi

face_count() { grep -cvE '^ *(#|$)' "$1" 2>/dev/null || echo "?"; }

run_one() { # 代号 件路径 rc语义
  local tag="$1" path="$2" sem="$3" out rc
  out=$(mktemp /tmp/gate_out.XXXXXX)
  # 每件都在同一姿势下跑：先 source noetic 再 source ws/devel（set +u 是因为 setup.bash 在 -u 下会炸）
  case "$tag" in
    SYN)   ( cd "$ROOT" && set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash
             grep -vE '^ *(#|$)' scripts/verify/syn_face.txt | xargs bash "$path" ) > "$out" 2>&1 ;;
    PYST)  ( cd "$ROOT" && set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash
             grep -vE '^ *(#|$)' scripts/verify/pyst_face.txt | xargs bash "$path" ) > "$out" 2>&1 ;;
    PARITY) ( cd "$ROOT" && set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash
              VP_FULLFACE=1 bash "$path" $(grep -vE '^ *(#|$)' scripts/verify/parity_face.txt) ) > "$out" 2>&1 ;;
    ISO)   ( cd "$ROOT" && set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash; export PYTHONNOUSERSITE=1; bash "$path" ) > "$out" 2>&1 ;;
    # ★R326（主理人 2026-10-15 16:05 明令成员 8→10）新增两条**跑法**分支：不改任何件、不改任何件的 rc 语义，
    #   只是让第 9／10 件能被串跑到、且跑在写死的面上（VCC 件内缺省面＝42 件≠存量表的 claim_face 70 件 ⇒ 必须显式喂，否则跨面混引）。
    VCC)   ( cd "$ROOT" && bash "$path" $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) ) > "$out" 2>&1 ;;
    规矩⑨) ( cd "$ROOT" && python3 "$path" ) > "$out" 2>&1 ;;
    *)     ( cd "$ROOT" && bash "$path" ) > "$out" 2>&1 ;;
  esac
  rc=$?
  # ★不按字节截行：`cut -c` 在本 locale 下按**字节**数，会把中文砍半截、令本件自己吐出无效 UTF-8（R299 实测）
  printf "%-7s rc=%-3s ｜ %s\n" "$tag" "$rc" "$(tail -1 "$out")"
  rm -f "$out"
}

echo "== 门禁串逐件（$(date '+%F %H:%M')，身份 $(whoami)）｜成员名单＝$CHAIN（$(face_count "$CHAIN") 行）=="
echo "   面：syn_face=$(face_count "$ROOT/scripts/verify/syn_face.txt") pyst_face=$(face_count "$ROOT/scripts/verify/pyst_face.txt") parity_face=$(face_count "$ROOT/scripts/verify/parity_face.txt") claim_face=$(face_count "$ROOT/scripts/verify/claim_face.txt")"
echo "   窗数回执＝$(face_count "$ROOT/scripts/verify/_window_receipt.tsv") 行（A27 口径）"
echo "   调用式（R278 佳文丙）＝ISO：noetic→ws/devel＋PYTHONNOUSERSITE=1（与 env.sh／v_window_preflight 同姿势）｜SYN／PYST／PARITY：同双 source＋各自面清单｜GTC／DC／VAR／账构：直接 bash 件｜R326 起两件＝VCC：bash 件＋显式喂 claim_face.txt（门用面写死，件内缺省面只有 42 件）｜规矩⑨：python3 件（件内缺省面＝claim_face.txt）"
while IFS=$'\t' read -r n tag path sem; do
  case "$n" in ''|'#'*) continue ;; esac
  if [ -n "${GATE_ONLY:-}" ] && ! printf '%s' ",$GATE_ONLY," | grep -q ",$tag,"; then continue; fi
  if [ ! -f "$ROOT/$path" ]; then printf "%-7s %s\n" "$tag" "［件不在位 ⇒ 报缺失，不当绿］$path"; continue; fi
  run_one "$tag" "$ROOT/$path" "$sem"
done < "$CHAIN"
echo "== 本件只报不判（退出码恒 0）；判定看各件自身 rc 与 05 的在册调用式，成员增减与 rc 语义变更须裁定者明令 =="
echo "   ★收尾跑序提示（05 追加三十一②；只提示、不动盘、不判定）：本串内 PYST／SYN 对 .py 跑过 py_compile ⇒ 源件旁会留下 __pycache__/*.pyc；接下来若跑任何按 find 计数的件，先按追加三十一①的三条核对式清产物，否则计数会虚高（R281／R282／R284 三次实测同源，产物在场数 19）"
exit 0
