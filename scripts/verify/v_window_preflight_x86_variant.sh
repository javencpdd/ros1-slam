#!/usr/bin/env bash
# v_window_preflight_x86_variant.sh —— x86 侧**联调**预检变体（R356 到签①，2026-10-18）
#
# 身份与边界（★先读再跑）：
#   · 在册 `v_window_preflight.sh`（A35＝丙，主理人-佳文 2026-10-14 裁）**一字不动**，
#     仍是唯一在册许可式；本件是 x86 容器路径形差下的**联调变体**。
#   · 按真人到签①：本件跑通（rc=0）即**视为预检 rc=0**（联调口径）。
#   · 本件**不重抄任何判据**：三条判据、rc 语义（0 放行／5 拒窗／9 前置缺失）、负载门的
#     核归一化基（R355 改判）**全部走在册本体**——唯一差别是把本体的 VPF_ISO 换成同目录
#     乙式包装件 `_iso_alt_wrapper.sh`（它接住"预检自带 source 顶掉 /home 形"这一跳）。
#     ★单一真身仍在在册件；在册件若改，本件自动跟随（无第二份实现、无漂移面）。
#   · ★本件不自设门值、不改判据语义、不入门禁串、不申请入任何面清单（入面归主理人）。
#
# 与在册本体的分界＝只此两处（★R475 起由 1 处变 2 处，见下「差异面账」）：
#   ① 在册本体：. source（noetic→devel）→ bash v3_isolation.sh      ⇒ x86 上 [4] 项 /home 形败（甲式已知态）
#      本件：    . source（同在册本体）→ bash _iso_alt_wrapper.sh     ⇒ 包装件把 /home 形插回队头＋ISO_ALLOW 补全 ⇒ 乙式口径
#   ② 在册本体：不打印勾对行                  ⇒ 本件跑完后补打 `[勾对]` 一行＋`rc_勾对`
#
# ── ★R475 追加＝补 R472q §四丙② 缺③「预检件勾对行」★（真人 2026-10-20 第①节拍板：走变体件、不动在册件）──
#   · 依据＝`05` 追加六十四 姿势④（新增一形 `rc_勾对`＝预检件里留一行「清单枚数／在场枚数／缺项名」，缺＝红）。
#   · ★勾对的两个来源都必须是外部物、不许自我循环★（追加六十五 姿势① 的教训：R472 那枚 `rc_勾对`
#     实际比的是脚本内手抄的 `COLS` 字典键 ⇒ 既不读在册件、也不读打印行 ⇒ 该 rc 只算自报）：
#       清单来源＝**在册本体头部注释里逐条声明的判据**（现读 `$BASE` 的第 ①／②／③ 三条）；
#       在场来源＝**本体这一跑实际打印出来的 `[①…]`／`[②…]`／`[③…]` 行**（不信变量、不信记忆）。
#   · ★rc 语义不动★：仍是 0／5／9（与在册本体逐字同）。★`rc_勾对` 只报不拦★——它**不参与 exit code**；
#     「勾对不齐要不要拒窗」＝新增一档＝判据面权，本件不自升、也不许被下一棒读成已拦。
#
# ── 差异面账（08_技术债台账 §二「变体债：上限＝2 处差异／触发器＝第 3 处」）──
#   R475 前＝1 处（VPF_ISO）；R475 起＝**2 处**（①VPF_ISO ②勾对行）⇒ ★已到上限★。
#   ⇒ 若再出现第 3 处差异，必须带「合并在册件」方案呈裁（合并走另起句修订 R356 旧句、★不回改原文★），
#     不得静默增殖。本账记在 `note/04` R475 段与请求单，本件只留指针。
#
# 用法（★x86 容器内）：
#   bash v_window_preflight_x86_variant.sh              # 真跑（放行句形状与在册本体逐字同形＋末尾一行勾对）
#   bash v_window_preflight_x86_variant.sh --selftest    # 透传在册本体五例自测（T0 的 ISO 走本件包装件）
#   bash v_window_preflight_x86_variant.sh --selftest-hg # 勾对双向自测（正例 rc_勾对=0／反例 rc_勾对=1，两例都必须能单独失败）
# rc 语义与在册本体同：0＝放行；5＝拒窗（任一判据不成立）；9＝前置缺失。
set -u
SELF=$(readlink -f "${BASH_SOURCE[0]}")
HERE=$(cd "$(dirname "$SELF")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BASE=$ROOT/scripts/verify/v_window_preflight.sh
WRAP=$HERE/_iso_alt_wrapper.sh
[ -f "$BASE" ] || { echo "[前置缺失] 在册预检本体不在位＝$BASE"; exit 9; }
[ -f "$WRAP" ] || { echo "[前置缺失] 乙式包装件不在位＝$WRAP"; exit 9; }
export VPF_ISO="$WRAP"

# 在册本体头部声明的一条判据标签（如 `①ISO_RC`）；取不到＝空串（调用方必须判空，不得当已勾）
decl_label() {  # $1＝标记（①②③）
  grep -m1 "^#   ${1}" "$BASE" 2>/dev/null | sed 's/^#   //' | sed 's/＝.*$//'
}

# 勾对行：清单 vs 在场。**只报不拦**（不改 exit code）。返回 0＝齐／1＝不齐
audit_line() {  # $1＝本体本跑输出文件
  local out=$1 decl=0 present=0 miss="" lbl m
  for m in ① ② ③; do
    [ -n "$(decl_label "$m")" ] && decl=$((decl + 1))
  done
  for m in ① ② ③; do
    if grep -q "^\[${m}" "$out" 2>/dev/null; then
      present=$((present + 1))
    else
      lbl=$(decl_label "$m")
      miss="${miss}${lbl:-${m}（本体头部未声明）}、"
    fi
  done
  [ -z "$miss" ] || miss=${miss%、}
  echo "[勾对] 清单枚数=${decl}／在场枚数=${present}／缺项=${miss:-无}｜清单来源＝在册本体头部注释逐条／在场来源＝本跑实际打印行"
  if [ "$decl" -eq 3 ] && [ "$present" -eq "$decl" ] && [ -z "$miss" ]; then
    echo "rc_勾对=0"
    return 0
  fi
  echo "rc_勾对=1（★只报不拦：不改 exit code，「勾对不齐要不要拒窗」归判据面★）"
  return 1
}

# ── 勾对双向自测：两例都必须能单独失败 ───────────────────────────────
if [ "${1:-}" = "--selftest-hg" ]; then
  W=$(mktemp -d /tmp/vpfx_selftest.XXXXXX)
  HP=0; HF=0
  # 例一（★正例★）：真跑 ⇒ 本体 rc=0、三条判据行齐 ⇒ rc_勾对 必须 =0
  bash "$SELF" > "$W/p.txt" 2>&1; R1=$?
  grep -q '^rc_勾对=0' "$W/p.txt" && H1=PASS || H1=FAIL
  grep -q '^\[勾对\] 清单枚数=3／在场枚数=3／缺项=无' "$W/p.txt" && H2=PASS || H2=FAIL
  echo "[$H1] 例一 真跑 ⇒ 勾对必须齐（本体 rc=$R1／须同时 rc_勾对=0）"
  echo "[$H2] 例一 勾对行逐字（清单枚数=3／在场枚数=3／缺项=无）"
  [ "$H1" = "PASS" ] && [ "$H2" = "PASS" ] && [ "$R1" -eq 0 ] && HP=$((HP + 1)) || HF=$((HF + 1))
  # 例二（★反例：把"在场"打到 0★）：用**桩树**造一次真前置缺失——把在册本体复制一份、
#   只把它的 `VPF_ISO=` 那行改成不存在件 ⇒ 本体走 `[前置缺失]` 分支、三条判据行一枚都不印
#   ⇒ 勾对**必须**报 1（否则这枚 rc 就是恒 0 的死格）。
#   ★为什么不直接 `VPF_ISO=/nonexistent` 注入★＝本件第 30 行就 `export VPF_ISO="$WRAP"`（第一处差异的本意），
#   外部注入会被无条件覆写 ⇒ 那样写出来的"反例"实测仍 rc=0（本批第一版就这么假绿过一次、已改）。
  W2=$W/tree
  mkdir -p "$W2/scripts/verify"
  cp -f "$BASE" "$W2/scripts/verify/v_window_preflight.sh"
  sed -i 's|^VPF_ISO=.*|VPF_ISO=/nonexistent/iso_hg.sh|' "$W2/scripts/verify/v_window_preflight.sh"
  printf '#!/usr/bin/env bash\nexit 0\n' > "$W2/scripts/verify/_iso_alt_wrapper.sh"
  cp -f "$SELF" "$W2/scripts/verify/v_window_preflight_x86_variant.sh"
  bash "$W2/scripts/verify/v_window_preflight_x86_variant.sh" > "$W/n.txt" 2>&1; R2=$?
  grep -q '^rc_勾对=1' "$W/n.txt" && H3=PASS || H3=FAIL
  grep -q '^\[勾对\] 清单枚数=3／在场枚数=0' "$W/n.txt" && H4=PASS || H4=FAIL
  echo "[$H3] 例二 桩树造前置缺失 ⇒ 勾对必须报 1（本体 rc=$R2／须 =9）"
  echo "[$H4] 例二 勾对行逐字（清单枚数=3／在场枚数=0）"
  [ "$H3" = "PASS" ] && [ "$H4" = "PASS" ] && [ "$R2" -eq 9 ] && HP=$((HP + 1)) || HF=$((HF + 1))
  rm -rf "$W"
  echo "PREFLIGHT-HG-SELFTEST VERDICT: PASS=$HP FAIL=$HF（2 例；任一例单独失败即 FAIL）"
  [ "$HF" -eq 0 ] && exit 0 || exit 5
fi

if [ "${1:-}" = "--selftest" ]; then
  exec bash "$BASE" --selftest
fi

# ── 真跑：本体输出原样透传 → 补打勾对行 → 以**本体 rc** 退出 ─────────
OUT=$(mktemp /tmp/vpfx_out.XXXXXX)
bash "$BASE" "$@" > "$OUT" 2>&1
RC=$?
cat "$OUT"
audit_line "$OUT" || :                          # 只报不拦：返回值不参与 exit code
rm -f "$OUT"
exit "$RC"
