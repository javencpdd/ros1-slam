#!/usr/bin/env bash
# witness_units.sh —— 活体窗「红线见证面」取件件（小明 2026-10-13，R238）
#
# 为何存在：v5.3 §0.12.3（主理人裁 S2＝乙 ⇒ 覆盖面 4→6，又裁 W10①＝乙′）要求 A1 基准改取
#   「六个 unit 的 systemd 态断言」——旧形状「扫 foxy 进程取 N 行、起终逐位一致」在 2026-10-12 停单后
#   永久失去对象（四进程不再存在 ⇒ 三窗 WITNESS 全 = INVALID-空集，见 R232）。本件把新形状成件，
#   并保留 foxy 残留计数作**辅助断言**（只报，不再当主判据）。
#
# 用法（harness 内）：
#   . "$DIR/witness_units.sh"                       # 只定义函数，不执行
#   witness_snapshot "$RUN" start                   # 起前
#   witness_verdict "$RUN"                          # 退后自定结论（内部先取 end 快照）
# 自测：bash witness_units.sh --selftest            # 三态各能单独失败
# 覆盖面可旋钮（WIT_UNITS）：默认六件＝§0.12.3 在册名单；改名单＝改覆盖面，须主理人明文。
WIT_UNITS_DEFAULT="lite3-monitor.service lite3-ros-bridge.service transfer_ros2.service whoopsie.service apport-autoreport.path apport-forward.socket"
# 名单与期望行数**一次算好并导出**（第一版在 harness 的 `set -u` 下用嵌套 $( [ -n "$WIT_UNITS" ] && … ) 取数，
# 结果拿到空串 ⇒ 把一致的一窗误判成 INVALID-行数不符；故这里绝不再用行内三元式算数）
if [ -n "${WIT_UNITS:-}" ]; then WIT_EXPECT_N=$(printf '%s\n' "$WIT_UNITS" | wc -w); else WIT_UNITS="$WIT_UNITS_DEFAULT"; WIT_EXPECT_N=6; fi
export WIT_UNITS WIT_EXPECT_N

witness_snapshot() {  # $1=RUN $2=start|end
  local f="$1/wit_$2.txt" u
  : > "$f" 2>/dev/null || return 1
  for u in ${WIT_UNITS:-$WIT_UNITS_DEFAULT}; do
    printf '%s|%s|%s\n' "$u" "$(systemctl is-enabled "$u" 2>&1)" "$(systemctl is-active "$u" 2>&1)" >> "$f"
  done
  wc -l < "$f" | tr -d ' '
}

witness_verdict() {  # $1=RUN —— 打印 WITNESS 结论并回显给调用方（stdout 末行＝结论词）
  local n
  n=$(witness_snapshot "$1" end)
  if [ ! -f "$1/wit_start.txt" ]; then echo "INVALID-缺起前快照"; return 1; fi
  if [ ! -s "$1/wit_start.txt" ]; then echo "INVALID-形状不符(起0行/终${n:-0}行)"; return 1; fi
  if [ "${n:-0}" != "$(wc -l < "$1/wit_start.txt" | tr -d ' ')" ]; then echo "INVALID-形状不符(起$(wc -l < "$1/wit_start.txt"|tr -d ' ')行/终${n}行)"; return 1; fi
  if [ "${n:-0}" != "$WIT_EXPECT_N" ]; then
    echo "INVALID-行数与覆盖面名单数不符(应 $WIT_EXPECT_N，实 ${n:-0})"; return 1
  fi
  if diff -q "$1/wit_start.txt" "$1/wit_end.txt" >/dev/null 2>&1; then echo "OK"; return 0
  else echo "DISTURBED"; diff "$1/wit_start.txt" "$1/wit_end.txt" | sed 's/^/    /'; return 1; fi
}

witness_foxy_aux() {  # 辅助断言：foxy／生产侧残留进程计数（只报，不判）
  local n
  # 模式里用字符类（[f]oxy）自我排除——直写 /opt/ros/foxy 会把"执行这条查询的 grep 自身"数进去
  # （本会话第三次踩同一形状：断言⑤、账构自测、本辅助项；见 05 追加十四）
  n=$(ps -eo args 2>/dev/null | grep -cE '/opt/ros/[f]oxy|run_[m]apping_online|livox_[r]os_driver' || true)
  echo "foxy 残留进程（辅助，非判据）=$n"
}

if [ "${1:-}" = "--selftest" ]; then
  W=$(mktemp -d /tmp/witst.XXXXXX); NP=0; NT=0
  chk() { NT=$((NT+1)); if [ "$2" = "$3" ]; then NP=$((NP+1)); echo "[PASS] $1（期望 $2／实得 $3）"; else echo "[FAIL] $1（期望 $2／实得 $3）"; fi; }
  N=$(witness_snapshot "$W" start); chk "T1 起前快照行数须等于覆盖面名单数 6" "6" "$N"
  cp "$W/wit_start.txt" "$W/wit_end.txt"; V=$(witness_verdict "$W"); chk "T2 起终一致须 OK" "OK" "$V"
  # 注意：一个 sed 里放两条互相覆盖的表达式会把改动**自己抵消掉**（本件第一版就栽在这），故用追加哨兵做变异
  sed -i '1s/$/|MUTATED/' "$W/wit_start.txt"
  V=$(witness_verdict "$W" | head -1); chk "T3 起终不一致必须 DISTURBED（这条不绿＝见证面是橡皮图章）" "DISTURBED" "$V"
  rm -f "$W/wit_start.txt"; V=$(witness_verdict "$W"); chk "T4 缺起前快照须 INVALID-缺起前快照（不得当作 OK）" "INVALID-缺起前快照" "$V"
  : > "$W/wit_start.txt"; V=$(witness_verdict "$W" | head -1); chk "T5 起前空文件须报『形状不符』而不是『缺快照』（0 行也是形状问题）" "INVALID-形状不符(起0行/终6行)" "$V"
  echo "WITNESS SELFTEST VERDICT: PASS=$NP/$NT"
  rm -rf "$W"; [ "$NP" = "$NT" ] && exit 0 || exit 9
fi
