#!/usr/bin/env bash
# v_b3_timebase_contract.sh —— 批3（坑25 时基根除）的**改点对账单＋验收件**：施工未开工前预置（小明 2026-10-09）
#
# 为什么现在做这件：批3 的硬前置（属主侧 chmod 类修复）还没落，本体不能开工；但**验收面不该等施工那天现场再想**。
#   在册修法（02 坑25 处置条）就一句：「把插件的打戳源与节流源统一到同一个 SimTime ⇒ 每 tick 时间戳严格单调、无同戳重复」。
#   这件把那句变成可执行的断言：现在跑必然判 **FAIL（rc=1，现状未修）**——这条红不是件坏，是**被检件的真实态**，
#   也是 05 C4a 要的「新 gate 先真的红一次」；批3 落地后同一件必须转 rc=0，转不了就是没修对。
#   本件**不进任何标准回归集**（避免未开工期间把共同门禁常项打红），只在批3 的施工/验收两刻各跑一次并归档。
#
# 锚点纪律（02 坑42 追加④ 的教训）：**一律按符号在文件里发现，不硬编码行号**；发现不了或形态与在册不符 ⇒ rc=9 不判被试
#   （那说明有人动过这文件，得先重新对账，而不是让我按旧行号猜一个新位置）。
# 范围纪律：在册修法只覆盖**发布打戳**（odometry header.stamp 与 TF sendTransform）。命令时效判据那一对
#   （last_cmd_time_ 与比较处的 ros::Time::now()）**同基成对、修法没要求改**；若被顺手改成 SimTime，
#   cmd_timeout_ 的语义会随 /clock 快慢漂移＝**行为变更**（超出批准面）⇒ B4 专门挡这一条。
#
# 用法：bash v_b3_timebase_contract.sh        退出码（本件自有层，不与 G35／W3／ISO 各层互认）：
#   0＝时基已统一（批3 施工后可验收）；1＝未统一（现状，本件今日即判此）；9＝锚点缺失或形态与在册不符（不判被试）
set -u
ROOT=/home/ros1-slam
LIVE_SRC=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp
SRC=${B3_SRC:-$LIVE_SRC}

if [ "${1:-}" = "--posctl" ]; then
  # 只证明"今日必红"还不够：那只能说明这件会红。这条把两处发布打戳**按在册修法**改成 SimTime 面的临时副本，
  #   拿它作被试重跑本体 ⇒ 必须转 rc=0；同时 B4（命令时效那一对）须仍 PASS，证明我的改写没越到批准面之外。
  #   注意：这是**本件的绿色可达成性见证（fixture）**，不是批3 的施工成品，也不是编译验证——真正的 .so 要等前置落地后重编译。
  [ -f "$SRC" ] || { echo "缺被测件：$SRC"; exit 9; }
  T=$(mktemp -d /tmp/b3pos.XXXXXX); trap 'rm -rf "$T"' EXIT
  sed -e 's/o\.header\.stamp = ros::Time::now();/o.header.stamp = ros::Time(now.Double());/' \
      -e 's/sendTransform(tf::StampedTransform(tf, ros::Time::now(),/sendTransform(tf::StampedTransform(tf, ros::Time(now.Double()),/' \
      "$SRC" > "$T/drive_fixture.cpp"
  a0=$(grep -cF 'ros::Time::now()' "$SRC"); a1=$(grep -cF 'ros::Time::now()' "$T/drive_fixture.cpp")
  echo "=== 阳性对照（fixture 副本）：在册件含 ros::Time::now() $a0 处 ⇒ 副本含 $a1 处（应少 2）==="
  [ "$((a0 - a1))" = 2 ] || { echo "改写处数不是 2 ⇒ 对照空转，判 9"; exit 9; }
  B3_SRC="$T/drive_fixture.cpp" bash "$0" > "$T/green.log" 2>&1; r=$?
  echo "  本体对 fixture 实得 rc=$r（期望 0＝时基已统一），FAIL 行数=$(grep -c '^  \[FAIL\]' "$T/green.log")"
  grep -E '^  \[(PASS|FAIL)\] B[2345]' "$T/green.log" | sed 's/^/      /'
  if [ "$r" = 0 ] && grep -q 'B4 命令时效那一对仍同基成对' "$T/green.log"; then
    echo "POSCTL VERDICT: 修法形态可使本件转绿且未越批准面 rc=0"; exit 0
  fi
  echo "POSCTL VERDICT: 按在册修法改完仍不绿（或越到 B4）⇒ 本件判据写错了，rc=5"; exit 5
fi

[ -f "$SRC" ] || { echo "缺被测件：$SRC"; exit 9; }
echo "=== 批3 时基对账（被测件 sha=$(sha256sum "$SRC" | cut -c1-12)）==="

python3 - "$SRC" <<'PYEOF'
import re, sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
lines = t.splitlines()

def where(pat, label, need=True):
    hits = [i + 1 for i, l in enumerate(lines) if re.search(pat, l)]
    if not hits:
        if need:
            print("  [ABSENT-ANCHOR] %s 的锚点找不到（%s）⇒ 形态与在册不符" % (label, pat)); sys.exit(9)
        return []
    if len(hits) > 1:
        print("  [ABSENT-ANCHOR] %s 的锚点不唯一（%s 命中 %s）⇒ 不硬猜，判 9" % (label, pat, hits)); sys.exit(9)
    return hits

NF = 0; NP = 0
def bad(m):
    global NF; NF += 1; print("  [FAIL] %s" % m)
def ok(m):
    global NP; NP += 1; print("  [PASS] %s" % m)

# B1 三处锚点：节流源、两个发布打戳
h_thr  = where(r'^\s*const common::Time now = parent_->GetWorld\(\)->SimTime\(\);', '节流源（SimTime）')
h_hd   = where(r'^\s*o\.header\.stamp\s*=', 'odometry 打戳')
h_tf   = where(r'^\s*tf_broadcaster_->sendTransform\(tf::StampedTransform\(', 'TF 打戳')
for h, n in ((h_thr, '节流源'), (h_hd, 'odometry 打戳'), (h_tf, 'TF 打戳')):
    ok("B1 %s 锚点唯一在位：L%d ⇒ %s" % (n, h[0], lines[h[0] - 1].strip()))

def stamp_src(h):
    s = lines[h - 1]
    if 'ros::Time::now()' in s: return 'ros::Time::now()'
    if re.search(r'\bnow\b', s): return 'SimTime(now)'
    if 'last_odom_time_' in s: return 'SimTime(last_odom_time_)'
    return '未知时基:' + s.strip()

s_hd, s_tf = stamp_src(h_hd[0]), stamp_src(h_tf[0])
print("  现场读数：odometry 打戳=%s ｜ TF 打戳=%s ｜ 节流源=%s" % (s_hd, s_tf, 'SimTime(now)'))

# B2/B3 在册判据：两处发布打戳必须与节流源同一时基
if s_hd == 'SimTime(now)': ok("B2 odometry 打戳已与节流源同基（SimTime）")
else: bad("B2 odometry 打戳仍用 %s ⇒ 与节流源（SimTime）不同基＝坑25 的本体（同戳重复发布）" % s_hd)
if s_tf == 'SimTime(now)': ok("B3 TF 打戳已与节流源同基（SimTime）")
else: bad("B3 TF 打戳仍用 %s ⇒ 同上，且这条正是 tf2::BufferCore 报 TF_REPEATED_DATA 的那一路" % s_tf)

# B4 范围护栏：命令时效那一对必须**同基成对**（在册修法没要求改它，改了＝超批准面的行为变更）
h_c1 = where(r'^\s*cmd_ = \*msg; last_cmd_time_\s*=\s*ros::Time::now\(\);', 'cmd 时效写入')
h_c2 = where(r'^\s*.*\(ros::Time::now\(\) - last_cmd_time_\)\.toSec\(\) > cmd_timeout_', 'cmd 时效比较')
ok("B4 命令时效那一对仍同基成对（L%d 写 / L%d 比，皆 ros::Time::now）⇒ 未被顺手改动" % (h_c1[0], h_c2[0]))

# B5 读数一致性：节流早退必须用同一 now（防止有人只改打戳、把节流换成墙钟，那会改 odom_rate 的物理语义）
#   首版在这里犯过一条**假红**：我按「离 L61 定义处 12 行内」去找早退式，而它在 L106（相隔 45 行）——
#   "离锚点 N 行内"不是结构事实（与坑42 追加④ 同源：把没写的当已知）。改为全文按语义找，并核它不含墙钟。
h_ret = where(r'\(now - last_odom_time_\)\.Double\(\) < period', '节流早退式')
_wall = [i + 1 for i in range(h_ret[0] - 1, min(len(lines), h_ret[0] + 1)) if 'ros::Time::now()' in lines[i]]
if not _wall:
    ok("B5 节流早退式在 L%d 且不含墙钟调用＝SimTime 面（未被换成墙钟）：%s" % (h_ret[0], lines[h_ret[0] - 1].strip()))
else:
    bad("B5 节流早退式附近（L%s）出现 ros::Time::now() ⇒ 节流面被换成墙钟，odom_rate 的物理语义已变，须公开回报" % _wall)

print("==================================================")
print("B3 TIMEBASE VERDICT: PASS=%d FAIL=%d ｜ %s" % (NP, NF,
      "时基已统一 ⇒ 批3 可结案（还须 5 连重新起算＋07 表加行）" if NF == 0 else
      "时基未统一＝现状（未施工）⇒ 本件今日必然判 1，这条红是被检件真实态、不是件坏"))
sys.exit(0 if NF == 0 else 1)
PYEOF
rc=$?
echo "  在册依据（引用须连件名＋代际）：02 坑25 处置条＝修法；小美 M2R4 复核节＝「被测件即变 ⇒ 5 连重新起算」；"
echo "    代际现值 SM/LIB 见 scripts/verify/v_harness_generation_table.sh --check（本批落地后跳 H15）。"
echo "B3 rc=$rc"
exit $rc
