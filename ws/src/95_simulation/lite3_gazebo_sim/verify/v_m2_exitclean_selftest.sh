#!/usr/bin/env bash
# v_m2_exitclean_selftest.sh —— 退净判据的零节点已知态标定件（小明 2026-10-08）
#
# 为何存在（小美 M2R7 结案 六节 R26 验收判据 / R27 三，G05 C4a L32、C4b L33、B2）：
#   R22 的退净判据到 M2R7 为止从未被一次真反例证过——四口在全部在册跑里都是零，
#   差集判据又因「须命中 OWN」而对本跑派生残留放过（她 #8–#10：O-3 那型 rosout 残留用旧判据打不出 FAIL）。
#   C4a 要求新门必须真实 FAIL 一次，C4b 要求探针必须对已知态标定；本件把两件事一次做完。
# 关键设计：判据逻辑住 m2_exitclean_lib.sh，本件与 m2_smoke.sh 同 source 该库（05 B2：
#   另抄一份去测＝测的不是在跑的那份）。所以本件通过即证明活体判据的分辨力，不只是证明一份副本。
# 性质：零 ROS 节点、零 roslaunch、零在册端口（11311／11312／11313／11345 全程零接触，
#   端口用例只用 127.0.0.1 随机高口的自绑自关）、零 ws/ 与 scripts/ 改动、无 sudo、不写证据目录。
set -u
LIB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB="$LIB_DIR/m2_exitclean_lib.sh"
[ -f "$LIB" ] || { echo "标定件缺库件：$LIB"; exit 2; }
source "$LIB"

LOGDIR=$(mktemp -d /tmp/m2_ec_selftest.XXXXXX)
REC="$LOGDIR/.marks"
PAT='roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|topic_tools|sim_adapter|m2_free_map|m2_cmd_relay|m2_tf_fresh|hdl_nodelet|run_mapping_online'
OWN='m2_tf_fresh|m2_free_map|m2_cmd_relay|gzserver|rosmaster --core -p 11313'
PORTS="11311 11312 11313 11345"
STAGE=selftest
SEG=pre
CHECKED_SEG=""
NP=0; NF=0; T=0; CP=0; CF=0

mark() { if [ "$1" = "1" ]; then NP=$((NP+1)); printf 'P|%s\n' "$2" >>"$REC"; echo "    [mkPASS] $2";
         else NF=$((NF+1)); printf 'F|%s\n' "$2" >>"$REC"; echo "    [mkFAIL] $2"; fi; }
HELD=""
hold() { HELD="$HELD $1"; }        # 累积登记：逐用例覆盖会让上一轮的残留漏杀
killall_hold() { local p; for p in $HELD; do kill "$p" 2>/dev/null; done; HELD=""; }
cleanup_self() { killall_hold; rm -rf "$LOGDIR"; }
trap cleanup_self EXIT
# 开局清扫：上一轮未收口的本用例残留若进基线，U3/U4 的「既存」与「新增」两面就都不再可信。
#   只扫本件专用的三个 argv0；本件自身命令行是 bash + 件名，不含这三个串，故不会自杀或误杀他人。
for _p in $(pgrep -u "$(id -u)" -f "rosout_selftest_u3|rosout_selftest_u4|m2_tf_fresh_selftest_u8" 2>/dev/null); do kill "$_p" 2>/dev/null; done
sleep 0.5

begin() { SEG="$1"; CHECKED_SEG=""; NP=0; NF=0; : >"$REC"; rm -f "$LOGDIR/exitbase.txt"; }
got() { local g; g=$(grep -F "|$1" "$REC" | tail -1 | cut -d'|' -f1); [ -n "$g" ] || g=NONE
        case "$g" in P) echo PASS;; F) echo FAIL;; *) echo NONE;; esac; }
chk() { T=$((T+1)); local a; a=$(got "$2")
        if [ "$a" = "$3" ]; then CP=$((CP+1)); echo "[PASS] $1 ⇒ 判据「$2」实得 $a（期望 $3）"
        else CF=$((CF+1)); echo "[FAIL] $1 ⇒ 判据「$2」实得 $a（期望 $3）"; fi; }
has_txt() { T=$((T+1)); local f="$LOGDIR/$1"
        if grep -qF "$3" "$f"; then CP=$((CP+1)); echo "[PASS] $2 ⇒ 跑内回显 $1 含「$3」"
        else CF=$((CF+1)); echo "[FAIL] $2 ⇒ $1 未见「$3」"; fi; }
assert_gone() { T=$((T+1)); local a; a=$(ps -o args= --no-headers -p "$2" 2>/dev/null)
        if [ -z "$a" ]; then CP=$((CP+1)); echo "[PASS] $1 ⇒ PID $2 已净（下一用例基线不会被本用例残留污染）"
        else CF=$((CF+1)); echo "[FAIL] $1 ⇒ PID $2 仍在位：$a"; fi; }
fake_proc() { ( exec -a "$1" sleep 999 ) </dev/null >/dev/null 2>&1 & echo $!; }   # argv0 可控的单 PID 残留
    # 三个 fd 都要断开：后台残留若继承 stdout，命令替换会等管道 EOF 而把调用方挂死（坑20 同族，实测踩过一次）
free_port() { local p i; for i in 1 2 3 4 5; do
        p=$(python3 -c 'import socket;s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(("127.0.0.1",0));print(s.getsockname()[1])' 2>/dev/null)
        [ -n "$p" ] && [ "$(port_cnt "$p" | awk '{print $1}')" = "0" ] && { echo "$p"; return 0; }; done; echo ""; }
listen_hold() { python3 -c 'import socket,sys,time
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", int(sys.argv[1]))); s.listen(1); time.sleep(float(sys.argv[2]))' "$1" 300 </dev/null >/dev/null 2>&1 & echo $!; }

echo "=== 退净判据零节点标定（库件 sha=$(sha256sum "$LIB" | awk '{print $1}' | cut -c1-12)）==="

# --- U1 探针敏感度阳性：真临时口必须被读成非零 ---
echo "--- U1 探针敏感度自检（期望 PASS：探针对非零态有分辨力）---"
begin probe; probe_sens_check; chk "U1 阳性对照" "探针敏感度自检" PASS

# --- U2 探针敏感度可 FAIL：桩掉查询式 ⇒ 必须打 FAIL（C4a 新门真实 FAIL 一次）---
echo "--- U2 探针敏感度阴性对照（桩 port_cnt 恒零，期望 FAIL：证该门能真 FAIL 而非恒绿）---"
begin probe_neg
port_cnt() { echo "0 0"; }
probe_sens_check; chk "U2 恒零探针必被识破" "探针敏感度自检" FAIL
source "$LIB"   # 还原真实定义

# --- U3 既存不误判：基线里就有的 rosout 型命中不得判死 ---
echo "--- U3 既存 rosout 型命中（在基线内，期望 进程面 PASS）---"
begin u3; _p1=$(fake_proc rosout_selftest_u3); hold "$_p1"
exitbase_snapshot >"$LOGDIR/u3.base" 2>&1; exitclean_check >"$LOGDIR/u3.out" 2>&1
sed 's/^/    /' "$LOGDIR/u3.out"
chk "U3 既存不入差集" "退净/进程面" PASS
killall_hold
assert_gone u3 "$_p1"

# --- U4 差集非空即判死：基线后新起的 rosout 型残留（命令行不含 OWN）必须打 FAIL ---
#     旧式判据（须命中 OWN 才判死）在此打不出 FAIL，正是她 #8/#9/#10 的漏判型，也是 O-3 的真实形态。
echo "--- U4 基线后新起 rosout 型残留（不含 OWN，期望 进程面 FAIL：R26 一改式生效）---"
begin u4
exitbase_snapshot >"$LOGDIR/u4.base" 2>&1
_p2=$(fake_proc rosout_selftest_u4); hold "$_p2"; sleep 1
exitclean_check >"$LOGDIR/u4.out" 2>&1
sed 's/^/    /' "$LOGDIR/u4.out"
chk "U4 差集非空即判死" "退净/进程面" FAIL
has_txt u4.out "U4 other 分栏可指认" "=other"
killall_hold
assert_gone u4 "$_p2"
# --- U5 缺基线判不洁：不把「没快照」当「干净」 ---
echo "--- U5 缺基线快照（期望 退净整体判不洁）---"
begin u5; exitclean_check >"$LOGDIR/u5.out" 2>&1
sed 's/^/    /' "$LOGDIR/u5.out"
chk "U5 缺基线不判洁" "退净：缺起跑前基线快照" FAIL

# --- U6/U7 端口面：计数与身份两面都要能真 FAIL，且既存监听者不误伤 ---
TP=$(free_port)
if [ -z "$TP" ]; then echo "[FAIL] U6/U7 取不到空闲口 ⇒ 端口面用例未跑"; CF=$((CF+1)); T=$((T+2));
else
echo "--- U6 基线后自绑监听口 $TP（期望 端口面与端口身份面双 FAIL）---"
begin u6; PORTS="$TP"
exitbase_snapshot >"$LOGDIR/u6.base" 2>&1
_p3=$(listen_hold "$TP"); hold "$_p3"; sleep 1
exitclean_check >"$LOGDIR/u6.out" 2>&1
sed 's/^/    /' "$LOGDIR/u6.out"
chk "U6 计数面能真 FAIL" "退净/端口面" FAIL
chk "U6 身份面能真 FAIL" "退净/端口身份面" FAIL
has_txt u6.out "U6 身份面打印新增三元组" "port-ident"

echo "--- U7 同一监听者已在基线内（期望 两面 PASS：不误伤）---"
begin u7; PORTS="$TP"
exitbase_snapshot >"$LOGDIR/u7.base" 2>&1
exitclean_check >"$LOGDIR/u7.out" 2>&1
sed 's/^/    /' "$LOGDIR/u7.out"
chk "U7 既存监听者不判死（计数面）" "退净/端口面" PASS
chk "U7 既存监听者不判死（身份面）" "退净/端口身份面" PASS
killall_hold; PORTS="11311 11312 11313 11345"
fi

# --- U8 OWN 分栏：含自有标识的残留同样判死，且分栏可读作 own ---
echo "--- U8 含 OWN 的残留（期望 进程面 FAIL 且分栏标 own=）---"
begin u8
exitbase_snapshot >"$LOGDIR/u8.base" 2>&1
_p4=$(fake_proc m2_tf_fresh_selftest_u8); hold "$_p4"; sleep 1
exitclean_check >"$LOGDIR/u8.out" 2>&1
sed 's/^/    /' "$LOGDIR/u8.out"
chk "U8 含 OWN 残留同样判死" "退净/进程面" FAIL
has_txt u8.out "U8 own 分栏可指认" "=own"
killall_hold

# --- U10 身份探针失能必须被判死（她 R36：port_ids 恒空 ⇒ 旧形态会静默 PASS）---
echo "--- U10 计数面有监听者而身份面恒空（期望 端口身份面 FAIL：一致性硬闸生效）---"
TP2=$(free_port)
if [ -z "$TP2" ]; then echo "[FAIL] U10 取不到空闲口 ⇒ 该用例未跑"; CF=$((CF+1)); T=$((T+2));
else
begin u10; PORTS="$TP2"
_p5=$(listen_hold "$TP2"); hold "$_p5"; sleep 1
port_ids() { :; }                       # 桩：身份面恒空，但计数面仍能看见监听者
begin u10b; PORTS="$TP2"; CHECKED_SEG=""
exitbase_snapshot >"$LOGDIR/u10.base" 2>&1
exitclean_check >"$LOGDIR/u10.out" 2>&1
sed 's/^/    /' "$LOGDIR/u10.out"
chk "U10 身份面失能不得判洁" "退净/端口身份面" FAIL
has_txt u10.out "U10 失能可指认" "身份探针失能"
source "$LIB"; PORTS="11311 11312 11313 11345"; killall_hold
fi

# --- U11 同一口换一个新 socket 必须被认作新增身份（她 R33 的阳性对照：PID 不可解析态也要抓得住）---
echo "--- U11 同口换 socket（PID 解析得到与否都算，期望 端口身份面 FAIL＝R33 inode 键生效）---"
TP3=$(free_port)
if [ -z "$TP3" ]; then echo "[FAIL] U11 取不到空闲口 ⇒ 该用例未跑"; CF=$((CF+1)); T=$((T+2));
else
begin u11; PORTS="$TP3"
_p6=$(listen_hold "$TP3"); hold "$_p6"; sleep 1
exitbase_snapshot >"$LOGDIR/u11.base" 2>&1
echo "  基线身份：$(port_ids "$TP3" | paste -sd, -)"
kill "$_p6" 2>/dev/null; HELD=""; sleep 1
_p7=$(listen_hold "$TP3"); hold "$_p7"; sleep 1        # 同一口，新进程新 socket（新 PID 与新 inode）
echo "  退净身份：$(port_ids "$TP3" | paste -sd, -)"
exitclean_check >"$LOGDIR/u11.out" 2>&1
sed 's/^/    /' "$LOGDIR/u11.out"
chk "U11 换 socket 必须算新增身份" "退净/端口身份面" FAIL
has_txt u11.out "U11 新增身份可指认" "port-ident"
killall_hold; PORTS="11311 11312 11313 11345"
fi

# --- U9 段令牌：同段幂等不重复打 mark，换段必须重检（她 #13 的正反两面）---
echo "--- U9 段令牌（期望 同段二次调用零新增 mark、换段后必重检）---"
begin u9; exitbase_snapshot >"$LOGDIR/u9.base" 2>&1
exitclean_check >"$LOGDIR/u9a.out" 2>&1; _n1=$((NP+NF))
exitclean_check >"$LOGDIR/u9b.out" 2>&1; _n2=$((NP+NF))
SEG=z; exitclean_check >"$LOGDIR/u9c.out" 2>&1; _n3=$((NP+NF))
echo "    首检 mark 数=$_n1；同段二次后=$_n2（应仍为 $_n1）；换段后=$_n3（应大于 $_n2）"
T=$((T+1)); if [ "$_n2" = "$_n1" ]; then CP=$((CP+1)); echo "[PASS] U9 同段幂等：trap 不会把同一段 mark 计两遍"; else CF=$((CF+1)); echo "[FAIL] U9 同段幂等失效：$_n1 → $_n2"; fi
T=$((T+1)); if [ "$_n3" -gt "$_n2" ]; then CP=$((CP+1)); echo "[PASS] U9 换段必重检：stage=all 三段各检一次（闭 #13）"; else CF=$((CF+1)); echo "[FAIL] U9 换段未重检：$_n2 → $_n3"; fi

echo "=================================================="
echo "EXITCLEAN SELFTEST: 用例 PASS=$CP FAIL=$CF / $T"
echo "  覆盖面：探针敏感度（U1 阳性／U2 阴性）、进程面差集（U3 不误判／U4 真判死／U8 分栏）、"
echo "  缺基线（U5）、端口计数与身份两面（U6 真判死／U7 不误伤）、段令牌幂等（U9）"
echo "=================================================="
[ "$CF" = "0" ] && exit 0 || exit 5
