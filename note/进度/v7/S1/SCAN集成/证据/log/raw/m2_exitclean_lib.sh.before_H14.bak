#!/usr/bin/env bash
# m2_exitclean_lib.sh —— 退净核验判据库（小明 2026-10-08；判据式子逐条出自小美 M2R7 结案六节 R26/R27）
#
# 为何要把判据从 m2_smoke.sh 抽出来：她 R26 的验收判据要求「改件后随批做一次已知态标定」，
#   而标定必须跑在**活体同一份判据代码**上（另抄一份去测＝测的不是在跑的那份，违 05 B2）。
#   故判据逻辑住本件：m2_smoke.sh（活体跑）与 v_m2_exitclean_selftest.sh（零节点标定件）同 source 本件。
#   代际后果：harness 的指纹不再覆盖退净判据 ⇒ m2_smoke.sh 跑首行须同时打本件 sha（R28 二 同批处理）。
#
# 依赖：source 方须先定义 PAT OWN PORTS LOGDIR STAGE SEG CHECKED_SEG 并提供 mark()。
# 本件零副作用：只定义函数，不 source ROS、不启停进程、不占任何在册端口。

port_cnt() { # $1=port -> "TCP UDP" 监听计数（只比数量；身份面见 port_ids，R26 二）
  local t u
  t=$(ss -lnt 2>/dev/null | grep -c ":$1\b"); u=$(ss -lnu 2>/dev/null | grep -c ":$1\b")
  echo "$t $u"
}

port_ids() { # R26 二：$1=port -> 每行一个监听身份三元组 "proto/port/pid/user"
  # 非 root 取不到他人口子的 PID 时记 pid=- user=hidden；进程刚好消失记 user=gone。
  # 三个值都进集合，故「他人换了 PID」与「本跑在同一口留下无人可查的监听者」都会成为新增身份（方向判死）。
  local p="$1" rec proto pid us
  while IFS= read -r rec; do
    [ -z "$rec" ] && continue
    proto=${rec%% *}; pid=${rec##* }
    if [ "$pid" = "-" ]; then echo "$proto/p$p/-/hidden"; continue; fi
    us=$(ps -o user= --no-headers -p "$pid" 2>/dev/null | tr -d ' '); [ -n "$us" ] || us=gone
    echo "$proto/p$p/$pid/$us"
  done < <(
    { ss -lantp 2>/dev/null | awk -v re=":$p\$" '$1=="LISTEN" && ($4 ~ re || $3 ~ re) {
           pid="-"; if (match($0, /pid=[0-9]+/)) pid=substr($0, RSTART+4, RLENGTH-4); print "TCP", pid }';
      ss -lanup 2>/dev/null | awk -v re=":$p\$" '$1=="UNCONN" && ($4 ~ re || $3 ~ re) {
           pid="-"; if (match($0, /pid=[0-9]+/)) pid=substr($0, RSTART+4, RLENGTH-4); print "UDP", pid }'; } | sort -u
  )
}

probe_sens_check() { # R27 三：端口查询式探针的阳性（非零）自检常驻入件
  # 闭 #12：本仓至今的端口面判据只在「四口全零」的跑里被读过，从未证明它对非零态有分辨力；
  #   一个恒读零的探针会让「四口零新增」的 PASS 成为假判据（坑27 同型）。
  # 用一个一次性临时监听口做阳性对照：绑 127.0.0.1:0 取随机口，读得出非零且身份可解析才算探针有效。
  #   随机口不在 PORTS 内 ⇒ 对在册四口的基线与退净两面零接触；自检本身自绑自关，零节点零 sudo。
  local _sf _ppid _pp _i _tv _uv _ids
  _sf="$LOGDIR/.probe_port"
  python3 -c 'import socket,sys,time
s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", 0)); s.listen(1)
open(sys.argv[1], "w").write(str(s.getsockname()[1]))
time.sleep(float(sys.argv[2]))' "$_sf" 6 >/dev/null 2>&1 &
  _ppid=$!
  for _i in 1 2 3 4 5 6 7 8 9 10; do [ -s "$_sf" ] && break; sleep 0.2; done
  _pp=$(cat "$_sf" 2>/dev/null)
  if [ -z "$_pp" ]; then
    kill "$_ppid" 2>/dev/null; wait "$_ppid" 2>/dev/null; rm -f "$_sf"
    mark 0 "探针敏感度自检：临时监听口未起成（python3 或 socket 不可用）⇒ 端口面自证缺件，本跑退净端口面不可信（R27 三）"
    return 0
  fi
  set -- $(port_cnt "$_pp"); _tv=$1; _uv=$2
  _ids=$(port_ids "$_pp" | paste -sd, -)
  kill "$_ppid" 2>/dev/null; wait "$_ppid" 2>/dev/null; rm -f "$_sf"
  if [ "${_tv:-0}" -ge 1 ] && [ -n "$_ids" ]; then
    mark 1 "探针敏感度自检：临时口 $_pp 读出 TCP=$_tv UDP=$_uv 身份=${_ids}（R27 三 阳性对照命中，探针对非零态有分辨力）"
  else
    mark 0 "探针敏感度自检：临时口 $_pp 读出 TCP=${_tv:-?} 身份=${_ids:-空} ⇒ 端口查询式探针无分辨力，本跑退净端口面不可信（R27 三）"
  fi
}

exitbase_snapshot() { # R21 二 基线快照 ＋ R26 二 身份三元组入基线
  { echo "# exitbase @$(date '+%Y-%m-%d %H:%M:%S') stage=$STAGE seg=$SEG"
    echo "PIDS $(pgrep -f "$PAT" 2>/dev/null | sort -n | tr '\n' ',' | sed 's/,$//')"
    for pt in $PORTS; do set -- $(port_cnt "$pt"); echo "PORT $pt TCP=$1 UDP=$2"; done
    for pt in $PORTS; do
      while IFS= read -r id; do [ -n "$id" ] && echo "IDENT $pt $id"; done < <(port_ids "$pt")
    done
  } > "$LOGDIR/exitbase.txt" 2>&1
  sed 's/^/[exitbase] /' "$LOGDIR/exitbase.txt"
}

exitclean_check() { # R21 形态 ＋ R22 部分阻断 ＋ R26 三条改式（差集非空即判死／端口面比身份／逐段各检一次）
  # R26 三：旧写法 CHECK_DONE=1 让 stage=all 只检第一段（她 #13 的洞，出自她 #22(四) 条目写法，她自记）。
  #   改为按段令牌：每段检一次且只检一次 ⇒ 既不覆写，也不会让 trap 把同一段的 mark 计两遍。
  [ -n "$CHECKED_SEG" ] && [ "$CHECKED_SEG" = "$SEG" ] && return 0
  CHECKED_SEG="$SEG"
  local _ecx _pid _bf _bp _bp2 _pfail _ifail _dfail _dtag _keep _cl _id _rec _bids _bt _bu _ct _cu
  _ecx="$LOGDIR/exitclean.$SEG.txt"
  { echo "# 退净核验 @$(date '+%Y-%m-%d %H:%M:%S')  stage=$STAGE seg=$SEG"
    echo "-- 命中进程（ps -o pid,user,ppid,lstart,args，R21 一：属主与起时落成件字段）--"
    _pid=$(pgrep -f "$PAT" 2>/dev/null | tr '\n' ',' | sed 's/,$//')
    if [ -n "$_pid" ]; then
      ps -o pid,user,ppid,lstart,args --no-headers -p $(echo "$_pid" | tr ',' ' ') 2>/dev/null | grep -v "pgrep -f" || echo "  （零命中）"
    else echo "  （零命中）"; fi
    echo "-- 监听端口分查（计数面）--"
    for _bf in $PORTS; do set -- $(port_cnt "$_bf"); echo "  port $_bf TCP=$1 UDP=$2"; done
    echo "-- 监听端口分查（身份三元组面，R26 二）--"
    for _bf in $PORTS; do
      while IFS= read -r _id; do [ -n "$_id" ] && echo "  cur $_id"; done < <(port_ids "$_bf")
    done
  } > "$_ecx" 2>&1
  sed 's/^/[exitclean] /' "$_ecx"

  _bf="$LOGDIR/exitbase.txt"
  # ---- 缺基线不判洁（R22 一 原样保留：不把「没快照」当「干净」）----
  if [ ! -f "$_bf" ]; then
    mark 0 "退净：缺起跑前基线快照（exitbase.txt 不在位）⇒ 无法按增量判定，本项按不洁处理"
    return 0
  fi
  _bp=$(grep '^PIDS ' "$_bf" | tail -1 | cut -d' ' -f2 | tr ',' ' ')
  # ---- R22 一：端口计数面（按基线增量，非零增即判死）----
  _pfail=0
  for _id in $PORTS; do
    set -- $(awk -v pt="$_id" '$1=="PORT" && $2==pt {gsub(/TCP=/,"",$3); gsub(/UDP=/,"",$4); print $3, $4}' "$_bf" | tail -1)
    _bt=${1:-0}; _bu=${2:-0}
    set -- $(port_cnt "$_id"); _ct=$1; _cu=$2
    if [ $((_ct + _cu)) -gt $(( ${_bt:-0} + ${_bu:-0} )) ]; then
      echo "  [port-delta] port $_id 基线=$(( ${_bt:-0} + ${_bu:-0} )) → 退净=$((_ct + _cu))"; _pfail=1
    fi
  done
  [ "$_pfail" = "0" ] && mark 1 "退净/端口面：四口相对基线零新增（R22 一）" \
                      || mark 0 "退净/端口面：本跑新增端口占用（见上方 port-delta，R22 一）"
  # ---- R26 二：端口身份面（端口号＋PID＋属主 集合比对，不只比计数）闭 #11 ----
  #   计数面放过的那一型：基线某口有既存监听者、本跑期间它消亡而本跑在同一口留下残留 ⇒ 计数相等判洁。
  _bids=$(awk '$1=="IDENT" {sub(/^IDENT /,""); print}' "$_bf")
  _ifail=0
  for _id in $PORTS; do
    while IFS= read -r _rec; do
      [ -z "$_rec" ] && continue
      _rec="$_id $_rec"
      if ! echo "$_bids" | grep -Fxq "$_rec"; then
        echo "  [port-ident] 新增监听身份: $_rec"; _ifail=1
      fi
    done < <(port_ids "$_id")
  done
  [ "$_ifail" = "0" ] && mark 1 "退净/端口身份面：相对基线零新增监听身份（R26 二）" \
                      || mark 0 "退净/端口身份面：出现基线中没有的监听身份（见上方 port-ident，R26 二）"
  # ---- R26 一：进程面差集非空即判死；OWN 只用于打印分栏，不再作豁免（闭 #8 #9 #10）----
  #   旧式「差集命中且命令行含 OWN 才判死」放过了 rosout／roslaunch mode_sim_*／mbf_costmap_nav／
  #   sim_adapter 这类本跑派生但不含自有标识的残留，而 O-3 那型真实残留恰好就是这一型（她 #10）。
  _dfail=""; _dtag=""
  for _pid in $(pgrep -f "$PAT" 2>/dev/null); do
    _keep=0
    for _bp2 in $_bp; do [ "$_pid" = "$_bp2" ] && _keep=1 && break; done
    [ "$_keep" = "1" ] && continue
    _cl=$(ps -o args= --no-headers -p "$_pid" 2>/dev/null)
    _dfail="$_dfail $_pid"
    if echo "$_cl" | grep -qE "$OWN"; then _dtag="$_dtag $_pid=own"; else _dtag="$_dtag $_pid=other"; fi
  done
  if [ -z "$_dfail" ]; then
    mark 1 "退净/进程面：本跑派生零残留（差集为空；既存命中不入判据，R26 一）"
  else
    mark 0 "退净/进程面：本跑派生未退净 PID:$_dfail 分栏:${_dtag}（R26 一：差集非空即判死，OWN 只分栏不豁免；other=不含自有标识的旧式漏判型）"
  fi
}
