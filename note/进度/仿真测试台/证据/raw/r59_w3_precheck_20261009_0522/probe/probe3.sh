#!/usr/bin/env bash
# w3 预检 ③④ 活体探针 v3：参数装到 /costmap_probe/costmap（Costmap2DROS 自己的 ns），读 /costmap_probe/costmap/costmap
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
RUN=/tmp/w3probe; LOG=$RUN/run3.log; : > "$LOG"
say(){ echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
P="";T="";C="";RS=""
port_pid(){ ss -lntp 2>/dev/null | grep -o ":$1.*pid=[0-9]*" | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u; }
cleanup(){ say "-- teardown（只按 PID 与端口持有者收，不用未锚定的 -f 全局匹配）--"
  for p in "$C" "$T" "$P" "$RS"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  for _ in 1 2 3 4 5 6 7 8 9 10; do n=$(port_pid 11313); [ -z "$n" ] && break
    for q in $n; do cl=$(tr '\0' ' ' < /proc/$q/cmdline 2>/dev/null)
      case "$cl" in *rosmaster*|*roscore*|*rosout*) say "  端口持有者 $q 确认为 ROS 件（${cl:0:44}）⇒ 收"; kill "$q" 2>/dev/null;; *) say "  端口持有者 $q 不是 ROS 件（${cl:0:44}）⇒ 不收";; esac; done
    sleep 2; done
  say "退净：11313=$(ss -lnt 2>/dev/null|grep -c ':11313') 11311=$(ss -lnt 2>/dev/null|grep -c ':11311')（均须 0）"; }
trap cleanup EXIT
roscore -p 11313 > "$RUN/roscore3.log" 2>&1 & RS=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
python3 $RUN/pub.py > "$RUN/pub3.log" 2>&1 & P=$!
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf3.log" 2>&1 & T=$!
sleep 5
rosparam load ${PF:-$RUN/params.yaml} /costmap_probe/costmap
say "参数回读（装在 costmap 子 ns）：global_frame=$(rosparam get /costmap_probe/costmap/global_frame 2>&1|tail -1) plugins=$(rosparam get /costmap_probe/costmap/plugins 2>&1 | grep -c 'name:')"
rosrun costmap_2d costmap_2d_node __name:=costmap_probe > "$RUN/costmap3.log" 2>&1 & C=$!
sleep 14
say "插件装载行：$(grep -aoE 'static_a|static_b|pre-Hydro' "$RUN/costmap3.log" | sort -u | tr '\n' ' ')"
say "话题面：$(rostopic list 2>/dev/null | grep -c 'costmap_probe/costmap/costmap$') 个主话题"
timeout 30 rostopic echo -n1 /costmap_probe/costmap/costmap > "$RUN/cost3.txt" 2>&1
say "采到 data 行=$(grep -c 'data:' "$RUN/cost3.txt")"
python3 - <<'PY'
import re
t=open('/tmp/w3probe/cost3.txt',encoding='utf-8',errors='ignore').read()
if 'data:' not in t: print("  [FAIL] 未采到 data 面"); raise SystemExit(1)
vals=[int(x) for x in re.findall(r'-?\d+',t.split('data:',1)[1])][:1600]
g=lambda r,c: vals[r*40+c] if len(vals)>r*40+c else None
print("  格数=%d（源图 40x40）"%(len(vals)))
print("  ③ 双 StaticLayer 同栈：map_a 的 100 区读回=%s；map_a 的 50 点读回=%s"%([g(10,10+i) for i in range(5)],g(20,20)))
print("  ④ unknown 面：map_b 的 -1 区读回=%s｜自由区(5,5)=%s｜未被两图覆盖的(35,35)=%s"%([g(30,i) for i in range(5)],g(5,5),g(35,35)))
print("  distinct 值=%s（254＝costmap 的 NO_INFORMATION；0＝自由；100＝必杀）"%(sorted(set(vals))))
PY
