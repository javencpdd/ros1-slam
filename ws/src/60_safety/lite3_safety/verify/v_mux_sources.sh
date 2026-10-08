#!/usr/bin/env bash
# v_mux_sources.sh —— M6 源表与 /cmd_vel 唯一写者的零节点静态防线（小明 2026-10-08，批1 行 H／J3 的落地配套）
# 为何要一件而不靠门禁：v3_isolation 的 [6] 只扫 ws/src/*/lite3_*/launch/，config/*.yaml 不在扫描面内
#   （小张 o1 设计件 §4.2 β 明写「不能把它当已过检的证据」）⇒ 源表加一行这件事在旧门禁下是**静默的**。
#   而 /cmd_vel 唯一写者是 §2.6-a 红线，M6 联锁（坑22 处置二）也挂在这张表上，必须有机判件。
# 顺序纪律（02 坑29）：先阳性对照（表必须解析出 ≥1 源），再零命中断言；空表或解析失败一律判废不判绿。
# 性质：零 ROS 节点、零活体、零 ws 写、无 sudo；二态 rc（0 全过／5 有 FAIL），件内无有效性门可折（06 普查口径）。
set -u
ROOT=/home/ros1-slam
Y=${MUX_Y:-$ROOT/ws/src/60_safety/lite3_safety/config/cmd_vel_mux.yaml}   # MUX_Y 仅供阴性对照用临时表跑同一份判据（05 C4a），默认值是在册真件
M=$ROOT/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py
S=$ROOT/ws/src/90_supervisor/lite3_supervisor
L7=$ROOT/ws/src/40_navigation/lite3_scan/launch/70_scan.launch
cd "$ROOT" || exit 5
NP=0; NF=0
mark() { if [ "$1" = "1" ]; then NP=$((NP+1)); echo "[PASS] $2"; else NF=$((NF+1)); echo "[FAIL] $2"; fi; }

# ---- M1 阳性对照＋M2 表结构＋M3 J3 行＋M4 红线守恒（一次解析，逐项判）----
python3 - "$Y" > /tmp/mux_read.txt <<'PYEOF'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
ss = c.get("sources") or []
ps = sorted(x["priority"] for x in ss)
print("N=%d" % len(ss))
print("PRI=%s" % ",".join(str(p) for p in ps))
print("UNIQ=%d" % (1 if len(set(ps)) == len(ps) else 0))
sc = [x for x in ss if x.get("name") == "scan_planner"]
print("SCAN=%s|%s|%s" % (len(sc), sc[0]["topic"] if sc else "-", sc[0]["priority"] if sc else "-"))
print("OUT=%s" % c.get("output_topic"))
print("RATE=%s" % c.get("output_rate_hz"))
print("HOLD=%s" % c.get("hold_topic"))
PYEOF
[ -s /tmp/mux_read.txt ] || { echo "[FAIL] M1 源表解析失败（yaml 不可读或不可解析）"; exit 5; }
g() { grep -m1 "^$1=" /tmp/mux_read.txt | cut -d= -f2-; }
[ "$(g N)" -ge 1 ] && mark 1 "M1 阳性对照：源表解析出 N=$(g N) 个源 ⇒ 后续「零命中/唯一」类断言有资格成立" \
                   || mark 0 "M1 阳性对照不过：源表解析为空 ⇒ 本件所有结论作废（扫描器无分辨力，坑29 同型）"
if [ "$(g N)" = "7" ] && [ "$(g PRI)" = "1,2,3,4,5,6,7" ] && [ "$(g UNIQ)" = "1" ]; then
  mark 1 "M2 表结构：七源、优先级恰为 1..7 且互不重复（并列 5 会把 teb 与 SCAN 同时活跃变成时序竞态，小张 §4.2 β 已排除）"
else mark 0 "M2 表结构不符：N=$(g N) PRI=$(g PRI) UNIQ=$(g UNIQ)"; fi
if [ "$(g SCAN)" = "1|/scan_planner/cmd_vel|7" ]; then mark 1 "M3 J3 行在位：scan_planner → /scan_planner/cmd_vel，priority=7（最低，只在无其他导航源时被选中）"
else mark 0 "M3 J3 行不符：$(g SCAN)"; fi
if [ "$(g OUT)" = "/cmd_vel" ] && [ "$(g RATE)" = "20.0" ] && [ "$(g HOLD)" = "/lite3/supervisor/hold" ]; then
  mark 1 "M4 红线守恒：output_topic=/cmd_vel、output_rate_hz=20.0（Lite3 250ms 超时不能断）、hold 源=SUP"
else mark 0 "M4 红线被改：$(g OUT) | $(g RATE) | $(g HOLD)"; fi

# ---- M5 唯一出口由构造：mux 脚本内 Publisher 只有一处 ----
_p=$(grep -c 'rospy.Publisher' "$M")
[ "$_p" = "1" ] && mark 1 "M5 唯一写者由构造：mux 脚本内 rospy.Publisher 计数=1（J3 加的是订阅面，发布出口未增）" \
                || mark 0 "M5 mux 脚本出现 $_p 处 Publisher ⇒ /cmd_vel 唯一写者红线受疑（§2.6-a）"

# ---- M6 SUP 侧零 cmd_vel 发布者（只 hold）----
# 只取 Publisher 的话题名字面量来判，不整行匹配：SUP 侧注释里本就写着「绝无 /cmd_vel」，
#   整行匹配会把这句红线声明读成一次违例（与坑29「匹配散文而非令牌」同型，首版就在这里假红过一次）。
_t=$(grep -rhoE "rospy\.Publisher\(\s*['\"][^'\"]+" "$S" 2>/dev/null | sed "s/.*['\"]//" | grep -c .)
_s=$(grep -rhoE "rospy\.Publisher\(\s*['\"][^'\"]+" "$S" 2>/dev/null | sed "s/.*['\"]//" | grep -c 'cmd_vel')
[ "$_s" = "0" ] && mark 1 "M6 SUP 侧零 cmd_vel 类发布者（Publisher 话题名共 $_t 个，全部非速度话题；红线：SUP 绝不写 /cmd_vel）" \
                || mark 0 "M6 SUP 侧出现 $_s 个 cmd_vel 类话题发布者 ⇒ 违 §2.6-a 与 N5"

# ---- M7 话题名对账：源表 scan_planner 的 topic 必须与 70_scan 的 remap 目标逐字一致（R15 教训）----
if [ "$(g SCAN)" = "1|/scan_planner/cmd_vel|7" ] && grep -q 'to="/scan_planner/cmd_vel"' "$L7"; then
  mark 1 "M7 话题名逐字对账：源表 topic 与 70_scan.launch 的 remap 目标同为 /scan_planner/cmd_vel（改任一侧不同步即红）"
else mark 0 "M7 话题名不对账：源表=$(g SCAN) 而 70_scan 侧=$(grep -c 'to="/scan_planner/cmd_vel"' "$L7") 处命中"; fi

# ---- M8 文案一致性：代码里不得残留未限定的「六源」----
_w=$(grep -c '六源' "$M")
[ "$_w" = "0" ] && mark 1 "M8 文案一致：mux 脚本内「六源」字样已清零（源数在代码里本就动态读取，无硬编码）" \
                || mark 0 "M8 mux 脚本仍有 $_w 处「六源」未加限定语 ⇒ 引用者会误读源数"

echo "=================================================="
echo "MUX SOURCES VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
