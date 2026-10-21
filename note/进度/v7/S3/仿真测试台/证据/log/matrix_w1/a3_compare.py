# A3 全局规划对比：GlobalPlanner vs NavfnROS，10 起终点对（maze2d 基线图，traj_maze 30 格走廊）
# 判据（C 文 D 表 全局规划行）：outcome=0 / 路径弧长 / 耗时 / 成功率≥90% / 无穿墙（路径点在 /map free 内）
import rospy, yaml, math, time, actionlib, csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from mbf_msgs.msg import GetPathAction, GetPathGoal

rospy.init_node('a3_compare')
traj = yaml.safe_load(open('/home/ros1-slam/ws/src/00_drivers/lite3_drivers/config/traj_maze.yaml'))
CELLS = traj['waypoints'][:30]           # 去程 30 格（回程为镜像）
PAIRS = [(0,29),(29,0),(3,25),(25,3),(10,17),(17,10),(5,27),(27,5),(12,22),(22,12)]

m = rospy.wait_for_message('/map', OccupancyGrid, timeout=20)
res_, ox, oy, w_, h_ = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.info.width, m.info.height
def occ_at(x, y):
    i, j = int((x-ox)/res_), int((y-oy)/res_)
    if not (0 <= i < w_ and 0 <= j < h_): return 100
    return m.data[j*w_+i]

def ps(x, y):
    p = PoseStamped(); p.header.frame_id='map'; p.header.stamp=rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p

ac = {'GlobalPlanner': actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction),
      'NavfnROS': actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)}
for k in ac: ac[k].wait_for_server(rospy.Duration(20))

def call(planner, a, b):
    g = GetPathGoal(); g.use_start_pose=True; g.start_pose=ps(*CELLS[a]); g.target_pose=ps(*CELLS[b])
    g.tolerance=0.0; g.planner=planner
    t0=time.time(); rt0=rospy.Time.now(); ac[planner].send_goal(g)
    ok = ac[planner].wait_for_result(rospy.Duration(30))
    wall = time.time()-t0; rosdt = (rospy.Time.now()-rt0).to_sec()
    if not ok:
        ac[planner].cancel_goal(); return dict(outcome=-1, message='30s 无 result', L=0.0, dt=rosdt, wall=wall, wallhits=None, n=0)
    r = ac[planner].get_result(); p = r.path
    L = sum(math.dist((p.poses[k].pose.position.x,p.poses[k].pose.position.y),
                      (p.poses[k+1].pose.position.x,p.poses[k+1].pose.position.y))
            for k in range(len(p.poses)-1)) if len(p.poses)>1 else 0.0
    hits = sum(1 for q in p.poses if occ_at(q.pose.position.x, q.pose.position.y) >= 90)
    return dict(outcome=r.outcome, message=r.message, L=L, dt=rosdt, wall=wall, wallhits=hits, n=len(p.poses))

rows=[]; wins={'GlobalPlanner':0,'NavfnROS':0}
for i,(a,b) in enumerate(PAIRS):
    order = ['GlobalPlanner','NavfnROS'] if i%2==0 else ['NavfnROS','GlobalPlanner']  # 交替先后抗次序效应
    for pl in order:
        r = call(pl, a, b); r.update(pair=i, a=a, b=b, planner=pl)
        rows.append(r)
        print('PAIR%02d cells(%02d->%02d) %-14s outcome=%2d %-28s 弧长=%6.2fm poses=%3d 耗时=%.3fs 穿墙点=%s'
              % (i,a,b,pl,r['outcome'],r['message'][:28],r['L'],r['n'],r['dt'],r['wallhits']), flush=True)
with open('/tmp/matrix_w1/a3_results.csv','w',newline='') as f:
    wcsv = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wcsv.writeheader(); wcsv.writerows(rows)
print('---- 汇总 ----')
for pl in ('GlobalPlanner','NavfnROS'):
    rs=[r for r in rows if r['planner']==pl]
    ok=sum(1 for r in rs if r['outcome']==0)
    Ls=[r['L'] for r in rs if r['outcome']==0]; dts=[r['dt'] for r in rs if r['outcome']==0]
    print('%-14s 成功=%d/10 (%.0f%%) 弧长均值=%.2fm 范围=[%.2f,%.2f] 耗时均值=%.3fs 最大=%.3fs 穿墙合计=%s'
          % (pl, ok, 10*ok/len(rs), sum(Ls)/len(Ls) if Ls else -1, min(Ls) if Ls else -1, max(Ls) if Ls else -1,
             sum(dts)/len(dts) if dts else -1, max(dts) if dts else -1, sum(r['wallhits'] or 0 for r in rs)))
