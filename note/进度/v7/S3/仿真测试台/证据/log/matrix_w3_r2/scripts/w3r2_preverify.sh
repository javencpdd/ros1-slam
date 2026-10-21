#!/usr/bin/env bash
# w3r2_preverify.sh —— W3-R2 goals 环带候选离线 getpath 预验 v3（小明 2026-10-07，方案 A）
# 判据（manifest，加严以对冲实测双峰性）：每候选 2 轮×4 次点单——GlobalPlanner 正/反向 → NavfnROS 正/反向，
# 全部 outcome=0 且 rosdt≤1.0s 才入集（预验 run1/2 + probe-rev 实证：6m 环带对在 stvl 动态 costmap 下
# GP 规划耗时双峰 0.06s↔>3s，单轮过关不足以保 T1 8/8；已知好对 5.6m/7.8m 弧长多轮 0.05–0.15s=存在可靠快对）。
# 防毒化（run1 mbf 段错误 / run2 cycle1 wedge 实证）：调用间隔 1.0s + 每调用后 mbf 存活探针 + 任一 NO-RESULT
# 整栈重启并同候选公平重试 1 次（重试再败才弃）；其余失败（55/超 1s）弃候选进下一个。
# 零文件改动、零 dynparam；远角对被环带 [4,8] 结构性排除（逐候选断言）。
LOGDIR=/tmp/matrix_w3/logs
STATE=/tmp/matrix_w3/w3r2_preverify_state.json
RESULT=/tmp/matrix_w3/w3r2_preverify_result.json
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313

cleanup() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f "roslaunch.*mode_nav_sim" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null; sleep 2; }

[ -f "$STATE" ] || echo '{"next":[0,0,0,0],"retrying":[false,false,false,false],"attempts":[],"cycles_no_progress":0}' > "$STATE"
FINAL_RC=5
for CYCLE in $(seq 1 80); do
  echo "[w3r2-pre] === cycle $CYCLE 起栈 ==="
  setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
    map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
    perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
    map_grid:=40 map_traj:=/tmp/matrix_w3/traj_perlin3d.yaml > "$LOGDIR/w3r2_preverify_launch_c$CYCLE.log" 2>&1 &
  sleep 2
  CY=$CYCLE python3 - <<'PYEOF' 2>&1 | tee "$LOGDIR/w3r2_preverify_getpath_c$CYCLE.log"
import json, math, os, time, subprocess, rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf as tf_py
from mbf_msgs.msg import GetPathAction, GetPathGoal

STATE_F = '/tmp/matrix_w3/w3r2_preverify_state.json'
RESULT_F = '/tmp/matrix_w3/w3r2_preverify_result.json'
ROUNDS = 2
st = json.load(open(STATE_F))
if 'retrying' not in st: st['retrying'] = [False, False, False, False]
C = json.load(open('/tmp/matrix_w3/w3r2_ring_candidates.json'))
S = C['start']; RING_LO, RING_HI = C['ring_m']

rospy.init_node('w3r2_preverify')
budget, t0 = 240.0, time.time()
tf_ = tf_py.TransformListener()
def gate(fn):
    while time.time() - t0 < budget and not rospy.is_shutdown():
        try:
            if fn(): return True
        except Exception: pass
        time.sleep(0.5)
    return False
g1 = gate(lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0).info.width > 0)
g2 = gate(lambda: tf_.canTransform('map', 'base_link', rospy.Time(0)))
g3 = gate(lambda: actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction).wait_for_server(rospy.Duration(2.0)))
print('W3R2-PRE gates map=%s tf=%s srv=%s t=%.1f' % (g1, g2, g3, time.time() - t0), flush=True)
if not (g1 and g2 and g3):
    st['cycles_no_progress'] = st.get('cycles_no_progress', 0) + 1
    json.dump(st, open(STATE_F, 'w'), ensure_ascii=False, indent=1)
    raise SystemExit(2 if st['cycles_no_progress'] < 3 else 5)

def mbf_alive():
    try:
        return b'move_base_flex' in subprocess.run(['rosnode', 'list'], capture_output=True, timeout=15).stdout
    except Exception:
        return False
def ps(x, y):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p
ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
def call(a, b, planner):
    g = GetPathGoal(); g.use_start_pose = True; g.start_pose = ps(*a); g.target_pose = ps(*b); g.tolerance = 0.0
    g.planner = planner
    rt0 = rospy.Time.now(); ac.send_goal(g)
    ok = ac.wait_for_result(rospy.Duration(8))
    dt = (rospy.Time.now() - rt0).to_sec()
    if not ok:
        ac.cancel_goal(); return dict(outcome=-1, dt=dt)
    r = ac.get_result()
    return dict(outcome=r.outcome, dt=dt)

def save(): json.dump(st, open(STATE_F, 'w'), ensure_ascii=False, indent=1)

for sec in C['sectors']:
    k = sec['sector']
    while st['next'][k] < len(sec['ranked']):
        if 'selected' in sec: break
        cand = sec['ranked'][st['next'][k]]
        c = (cand[0], cand[1])
        d = math.hypot(c[0] - S[0], c[1] - S[1])
        assert RING_LO <= d <= RING_HI, '候选出环带 (%.2f)' % d   # 远角结构性排除断言
        ok_all, nores = True, False
        for rd in range(ROUNDS):
            for a, b, planner, direction in [(S, c, 'GlobalPlanner', 'fwd'), (c, S, 'GlobalPlanner', 'rev'),
                                             (S, c, 'NavfnROS', 'fwd'), (c, S, 'NavfnROS', 'rev')]:
                time.sleep(1.0)
                r = call(a, b, planner)
                ok = (r['outcome'] == 0 and r['dt'] <= 1.0)
                if r['outcome'] == -1: nores = True
                st.setdefault('attempts', []).append(dict(sector=k, cand=[cand[0], cand[1]], d=round(d, 2),
                    round_n=rd + 1, a=list(a), b=list(b), planner=planner, direction=direction,
                    outcome=r['outcome'], dt=round(r['dt'], 3), ok=ok, selected=False, cycle=os.environ.get('CY')))
                print('W3R2-PRE s%d#%d r%d (%.2f,%.2f)->(%.2f,%.2f) %s %s outcome=%d dt=%.3fs ok=%s mbf_alive=%s'
                      % (k, st['next'][k], rd + 1, a[0], a[1], b[0], b[1], planner, direction,
                         r['outcome'], r['dt'], ok, mbf_alive()), flush=True)
                if not ok:
                    ok_all = False; break
            if not ok_all: break
        if ok_all:
            for at in st['attempts'][-ROUNDS * 4:]: at['selected'] = True
            sec['selected'] = [cand[0], cand[1]]
            st['retrying'][k] = False
            st['next'][k] += 1
            save()
            print('W3R2-PRE sector%d SELECTED (%.2f,%.2f) d=%.2fm' % (k, cand[0], cand[1], d), flush=True)
            break
        # 失败处置：NO-RESULT（wedge/死）→ 整栈重启同候选公平重试 1 次；
        # 其余失败（55/超1s，探针实证不毒化栈：P4 在两次 55 后仍 0.09s 过）→ 弃候选原地续测下一个
        st['next'][k] += 1
        if nores and not st['retrying'][k]:
            st['retrying'][k] = True
            st['next'][k] -= 1
            save()
            print('W3R2-PRE s%d 候选#%d NO-RESULT → 整栈重启重试同候选' % (k, st['next'][k]), flush=True)
            raise SystemExit(3)   # 退栈重启（bash cycle 驱动）
        st['retrying'][k] = False
        save()
        print('W3R2-PRE s%d 候选#%d 未过（nores=False）→ 弃用原地续测下一候选' % (k, st['next'][k]), flush=True)
    save()
    if 'selected' not in sec:
        # W3-R2 接手修订（小明 2026-10-07）：扇区回炉耗尽只记 exhausted 并续跑下一扇区，
        # 终判挪到全部扇区处理完后（原版首扇区耗尽即 rc=4 退出，sectors1-3 从未live测过）。
        # 判据（2轮×4点单 outcome=0 且 ≤1s、NO-RESULT 整栈重试1次）一字未动。
        sec['exhausted'] = True
        print('W3R2-PRE sector%d 回炉耗尽（候选 %d 个全未过）→ 续测下一扇区' % (k, len(sec['ranked'])), flush=True)
        save()

chosen = [s.get('selected') for s in C['sectors']]
n_sel = sum(1 for c in chosen if c)
json.dump(dict(start=S, ring_m=[RING_LO, RING_HI], chosen=chosen, n_selected=n_sel, attempts=st['attempts']),
          open(RESULT_F, 'w'), ensure_ascii=False, indent=1)
if n_sel == len(C['sectors']):
    print('W3R2-PRE ALL-SECTORS-VERIFIED chosen=%s' % chosen, flush=True)
    raise SystemExit(0)
print('W3R2-PRE PARTIAL n_selected=%d/%d chosen=%s' % (n_sel, len(C['sectors']), chosen), flush=True)
raise SystemExit(4)
PYEOF
  RC=${PIPESTATUS[0]}
  echo "[w3r2-pre] cycle $CYCLE tester rc=$RC，退净..."
  cleanup
  if [ "$RC" -eq 0 ]; then FINAL_RC=0; break; fi
  if [ "$RC" -eq 4 ] || [ "$RC" -eq 5 ]; then FINAL_RC=$RC; break; fi
  FINAL_RC=3
done
echo "[w3r2-pre] 终态 rc=$FINAL_RC"
exit $FINAL_RC
