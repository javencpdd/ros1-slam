#!/usr/bin/env bash
# v_window_preflight.sh —— 活体窗的**唯一许可式**（A35＝丙，主理人-佳文 2026-10-14 00:35 裁；施工侧不自设条件）
#
# 要灭的是这一型：`v3_isolation.sh` 长期 `rc=1`（第 [10] 项逮到 `import numpy` 解析进运行账户的 user-site），
#   于是"能不能开窗"变成每个人各自解释——A35 未裁的这十几批里，所有活体窗都被这一格挡死，
#   而**挡它的其实不是环境、是没有一个写死的许可式**。本件把许可式变成一个数：三条全绿才放。
#
# 三条判据（缺一即拒窗，逐条印字段）：
#   ①ISO_RC＝0 —— 本件自带 source（noetic → ws/devel），调用者不必也不得先手 source（防跨态混引）；
#   ②numpy 的**解析路径前缀**＝/usr/local —— 把库来源钉成唯一态（`PYTHONNOUSERSITE=1` 由本件 export，
#     也写进 scripts/env.sh；1.23.1 那个 user-site 态自此不再是默认态，旧留底属另一态、不可与新窗混引）；
#   ③load1 ≤ VPF_LOAD_MAX（缺省 4.0，在册开窗规矩）。
#
# rc 语义：0＝放行；5＝拒窗（任一判据不成立）；9＝前置缺失（ISO 件不在位／source 后仍取不到 numpy 字段／
#   **空读数不得当绿读**，坑29 同族）。判定不改任何判据与门值：ISO 本体、harness、门禁串成员一字不动。
#
# --selftest 四例（每例必须能单独失败；照 ISO 的在册教训：**基线不绿即 exit 9**，否则所有断言失去意义）：
#   T0 真跑＝期望 0；T1 阈值收到 0.0001＝期望 5（load1 必超）；T2 路径前缀改成不存在＝期望 5；
#   T3 ISO 件不可达＝期望 9。T1/T2/T3 都只动本件自己的旋钮，不改任何在册件。

SELF=$(readlink -f "${BASH_SOURCE[0]}")
ROOT=$(cd "$(dirname "$SELF")/../.." && pwd)
VPF_ISO=${VPF_ISO:-$ROOT/scripts/verify/v3_isolation.sh}
VPF_LOAD_MAX=${VPF_LOAD_MAX:-4.0}
VPF_NUMPY_PREFIX=${VPF_NUMPY_PREFIX:-/usr/local}

emit_fail() { echo "[拒窗] $1"; VERDICT_FAIL=$((VERDICT_FAIL + 1)); }

run_checks() {
  VERDICT_FAIL=0
  MISSING=0
  if [ ! -f "$VPF_ISO" ]; then
    echo "[前置缺失] ISO 件不在位＝$VPF_ISO"
    MISSING=1
    return
  fi

  local out
  out=$(mktemp /tmp/preflight_iso.XXXXXX)
  # 子 shell 内 source，不污染调用方；set +u 是因为 noetic 的 setup.bash 在 -u 下会炸（在册规矩）
  set +u
  export PYTHONNOUSERSITE=1
  # shellcheck disable=SC1091
  . /opt/ros/noetic/setup.bash >/dev/null 2>&1 || echo "[前置缺失] noetic source 失败" 1>&2
  # shellcheck disable=SC1091
  . "$ROOT/ws/devel/setup.bash" >/dev/null 2>&1 || echo "[前置缺失] ws/devel source 失败" 1>&2
  bash "$VPF_ISO" > "$out" 2>&1
  local iso_rc=$?
  set -u
  ISO_RC=$iso_rc
  ISO_TAIL=$(tail -1 "$out")
  rm -f "$out"

  echo "[①ISO] ISO_RC=$ISO_RC 尾行＝${ISO_TAIL:0:150}"
  [ "$ISO_RC" -eq 0 ] || emit_fail "ISO_RC=$ISO_RC≠0（第 [10] 项或其余判据未过）"

  NUMPY_LINE=$(python3 -c 'import numpy;print(numpy.__version__, numpy.__file__)' 2>/dev/null)
  if [ -z "$NUMPY_LINE" ]; then
    echo "[前置缺失] numpy 两字段取不到（import 失败或 python 不可用）"
    MISSING=1
    return
  fi
  NUMPY_VER=${NUMPY_LINE%% *}
  NUMPY_PATH=${NUMPY_LINE#* }
  echo "[②库来源] numpy=$NUMPY_VER path=$NUMPY_PATH 要求前缀=$VPF_NUMPY_PREFIX"
  case "$NUMPY_PATH" in
    "$VPF_NUMPY_PREFIX"*) : ;;
    *) emit_fail "numpy 解析到 $NUMPY_PATH（不在 $VPF_NUMPY_PREFIX 下＝跨态风险）" ;;
  esac

  LOAD1=$(awk '{print $1}' /proc/loadavg)
  echo "[③机况] load1=$LOAD1 阈值=$VPF_LOAD_MAX 门槛来源＝在册开窗规矩"
  if awk -v a="$LOAD1" -v b="$VPF_LOAD_MAX" 'BEGIN{exit !(a>b)}'; then
    emit_fail "load1=$LOAD1 > 阈值 $VPF_LOAD_MAX"
  fi
}

if [ "${1:-}" = "--selftest" ]; then
  NPASS=0; NFAIL=0
  check() { # 名称 期望rc 实际rc
    if [ "$2" = "$3" ]; then echo "[PASS] $1（期望 rc=$2 实=$3）"; NPASS=$((NPASS+1));
    else echo "[FAIL] $1（期望 rc=$2 实=$3）"; NFAIL=$((NFAIL+1)); fi
  }
  VPF_ISO="$VPF_ISO" VPF_LOAD_MAX="$VPF_LOAD_MAX" VPF_NUMPY_PREFIX="$VPF_NUMPY_PREFIX" bash "$SELF" >/tmp/vpf_t0.txt 2>&1; T0=$?
  check "T0 真跑必须放" 0 "$T0"
  if [ "$T0" -ne 0 ]; then
    # 基线不绿 ⇒ 后面三例的"拦得住"全不可信（ISO 自测的第一课），直接 exit 9
    echo "[中止] T0 基线不绿（rc=$T0），四例不作数——见 /tmp/vpf_t0.txt 末三行"
    tail -3 /tmp/vpf_t0.txt
    exit 9
  fi
  VPF_LOAD_MAX=0.0001 bash "$SELF" >/tmp/vpf_t1.txt 2>&1; check "T1 阈值压到 0.0001 必须拦" 5 $?
  VPF_NUMPY_PREFIX=/nonexistent-prefix bash "$SELF" >/tmp/vpf_t2.txt 2>&1; check "T2 路径前缀改错必须拦" 5 $?
  VPF_ISO=/nonexistent/iso.sh bash "$SELF" >/tmp/vpf_t3.txt 2>&1; check "T3 ISO 件不在位必须 rc=9（不当绿）" 9 $?
  echo "PREFLIGHT-SELFTEST VERDICT: PASS=$NPASS FAIL=$NFAIL（T0 未绿即已 exit 9，不留假全过）"
  [ "$NFAIL" -eq 0 ] && exit 0 || exit 5
fi

VERDICT_FAIL=0
run_checks
if [ "$MISSING" -eq 1 ]; then
  echo "PREFLIGHT VERDICT rc=9（前置缺失＝不放行也不判绿；把缺的那项补齐再跑）"
  exit 9
fi
if [ "$VERDICT_FAIL" -gt 0 ]; then
  echo "PREFLIGHT VERDICT rc=5（拒窗：$VERDICT_FAIL 项不成立，逐条见上）"
  exit 5
fi
echo "PREFLIGHT VERDICT rc=0（放行：ISO_RC=0／numpy 在 $VPF_NUMPY_PREFIX 下／load1=$LOAD1≤$VPF_LOAD_MAX）"
exit 0
