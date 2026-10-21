#!/usr/bin/env bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
RUN=/tmp/w3probe; LOG=$RUN/run2.log; : > "$LOG"
say(){ echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
P="";T="";C="";RS=""
cleanup(){ say "-- teardown --"; for p in "$P" "$T" "$C"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done; sleep 3
  say "退净：11313 监听=$(ss -lnt 2>/dev/null|grep -c ':11313')（须 0）"; }
trap cleanup EXIT
roscore -p 11313 > "$RUN/roscore2.log" 2>&1 & RS=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
python3 $RUN/pub.py > "$RUN/pub2.log" 2>&1 & P=$!
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf.log" 2>&1 & T=$!
sleep 5
say "TF 面：$(timeout 6 rosrun tf tf_echo map base_link 2>/dev/null | head -2 | tr "\n" " ")"
rosparam load $RUN/params.yaml /costmap_probe; say "参数回读 global_frame=$(rosparam get /costmap_probe/global_frame 2>&1|tail -1) plugins 条数=$(rosparam get /costmap_probe/plugins 2>&1 | grep -c 'name:')"
rosrun costmap_2d costmap_2d_node __name:=costmap_probe > "$RUN/costmap2.log" 2>&1 & C=$!
sleep 14
say "话题面：$(rostopic list 2>/dev/null | grep 'costmap_probe' | tr '\n' ' ')"
timeout 30 rostopic echo -n1 /costmap_probe/costmap > "$RUN/cost2.txt" 2>&1
say "采到 data 行=$(grep -c 'data:' "$RUN/cost2.txt") 行数=$(wc -l < "$RUN/cost2.txt")"
python3 - <<'PY'
import re
t=open('/tmp/w3probe/cost2.txt',encoding='utf-8',errors='ignore').read()
if 'data:' not in t: print("  [FAIL] 仍没采到 data 面"); raise SystemExit(1)
vals=[int(x) for x in re.findall(r'-?\d+',t.split('data:',1)[1])][:1600]
g=lambda r,c: vals[r*40+c] if len(vals)>r*40+c else None
print("  格数=%d；map_a 的 100 区读回=%s；map_a 的 50 点读回=%s"%(len(vals),[g(10,10+i) for i in range(5)],g(20,20)))
print("  map_b 的 -1 区读回=%s；自由区(5,5)=%s；未标注(35,35)=%s"%([g(30,i) for i in range(5)],g(5,5),g(35,35)))
print("  distinct 值=%s"%sorted(set(vals)))
PY
say "costmap 日志关键行：$(grep -aiE 'layer|plugin|unknown|err' "$RUN/costmap2.log" | head -3 | tr '\n' '|')"
