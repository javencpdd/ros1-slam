#!/usr/bin/env bash
# `_pub_count` 的自测（三连 + 反控）。
#
# 被测缺陷（2026-11-26 实测）：原实现是
#     rostopic info "$1" | sed -n '/Publishers/,$p' | grep -c '^ \*'
# `$p` 是"到文件尾"，把 **Subscribers 段**的条目一起算进发布者 ⇒ 只要有人订阅，
# 计数就 ≥2。于是"单发布者守卫"恒假：`/rslidar_points`、`/hdl_odom` 永远被判"双发布者"。
# 现场代价：两次把"一发一订"读成"两个 gzserver / 两份 HDL"，排查方向被带偏。
#
# 做法：stub 一个 `rostopic`，喂三种 `rostopic info` 形状，只验计数函数。
#   主测：一发一订 ⇒ 1；双发一订 ⇒ 2；无发有订 ⇒ 0
#   反控：同一份"一发一订"夹具喂**旧实现**必须得 2（证明本测试真能抓住这个 bug，
#         而不是永远绿的空控）
set -uo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SIM=$REPO/docker/sim.sh

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
PASS=0; FAIL=0
say() { printf '%s\n' "$*"; }
ok()   { PASS=$((PASS+1)); say "PASS $*"; }
bad()  { FAIL=$((FAIL+1)); say "FAIL $*"; }

# ---- 被测函数：从 sim.sh 里**原样抽取**，不复制粘贴 ----
eval "$(sed -n '/^_pub_count() {/,/^}/p' "$SIM")"
if ! declare -F _pub_count >/dev/null; then
  say "FAIL 无法从 $SIM 抽取 _pub_count"; exit 1
fi

# ---- stub rostopic：按 FIXTURE 环境变量输出 ----
cat > "$TMP/rostopic" <<'SH'
#!/usr/bin/env bash
cat "$FIXTURE"
SH
chmod +x "$TMP/rostopic"

fixture_one_pub_one_sub() {
  cat > "$TMP/f1" <<'TXT'
Type: sensor_msgs/PointCloud2

Publishers: 
 * /sim_adapter (http://h:43633/)

Subscribers: 
 * /relay (http://h:41767/)

TXT
}
fixture_two_pub_one_sub() {
  cat > "$TMP/f2" <<'TXT'
Type: sensor_msgs/PointCloud2

Publishers: 
 * /a (http://h:1/)
 * /b (http://h:2/)

Subscribers: 
 * /relay (http://h:3/)

TXT
}
fixture_no_pub() {
  cat > "$TMP/f3" <<'TXT'
Type: sensor_msgs/PointCloud2

Publishers: None

Subscribers: 
 * /relay (http://h:3/)

TXT
}

run_count() {  # $1=夹具路径
  PATH="$TMP:$PATH" FIXTURE="$1" _pub_count /whatever 2>/dev/null | tail -1
}

# ---- 主测 ----
fixture_one_pub_one_sub
n=$(run_count "$TMP/f1")
[ "$n" = "1" ] && ok "一发一订 ⇒ 1（实际 $n）" || bad "一发一订应得 1，实际 $n（把订阅者算成发布者了）"

fixture_two_pub_one_sub
n=$(run_count "$TMP/f2")
[ "$n" = "2" ] && ok "双发一订 ⇒ 2（实际 $n）" || bad "双发一订应得 2，实际 $n"

fixture_no_pub
n=$(run_count "$TMP/f3")
[ "$n" = "0" ] && ok "无发有订 ⇒ 0（实际 $n）" || bad "无发有订应得 0，实际 $n"

# ---- 反控：旧实现必须在这份夹具上翻车 ----
old_pub_count() {  # 历史写法，故意保留
  rostopic info "$1" 2>/dev/null | sed -n '/Publishers/,$p' | grep -c '^ \*' || true
}
fixture_one_pub_one_sub
n_old=$(PATH="$TMP:$PATH" FIXTURE="$TMP/f1" old_pub_count /whatever 2>/dev/null | tail -1)
if [ "$n_old" = "2" ]; then
  ok "反控：旧实现把一发一订算成 2（证明本测试能抓住该 bug）"
else
  bad "反控失效：旧实现得 $n_old，说明夹具与本测试都测不出原缺陷"
fi

say "----"
say "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = "0" ] || exit 1
say "VERDICT=PASS"
