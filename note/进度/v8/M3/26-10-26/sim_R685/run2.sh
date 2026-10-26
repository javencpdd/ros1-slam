source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 HAVE_GAZEBO=0
mkdir -p /work/logs /work/ros; export ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
export PYTHONPATH=/work/planner:/work/contracts:$PYTHONPATH
roscore -p 11311 >/tmp/rc.log 2>&1 &
python3 - <<'P'
import time, xmlrpc.client
for _ in range(60):
    try:
        if xmlrpc.client.ServerProxy("http://127.0.0.1:11311").getPid("/wait")[0] == 1:
            print("MASTER OK"); break
    except OSError: pass
    time.sleep(0.5)
else: print("MASTER FAIL")
P
rosparam set /v8_support_planner/map_json /work/map.json
rosparam set /v8_support_planner/start "[0.0, 0.0, 0.0]"
rosparam set /v8_support_planner/goal "[1.0, 0.0, 0.35]"
rosparam set /v8_support_planner/budget 2000
rosparam set /v8_support_planner/keep_alive_s 300
rosparam set /v8_support_planner/make_plan_service /make_plan
python3 /work/planner/ros_node.py __name:=v8_support_planner >/work/node.log 2>&1 & sleep 2
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom >/dev/null 2>&1 & sleep 1
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link >/dev/null 2>&1 & sleep 1
rosparam load /work/mb.yaml /move_base
( source /work/ws/devel/setup.bash; exec rosrun move_base move_base __name:=move_base ) >/work/mb.log 2>&1 &
for i in $(seq 1 30); do rosservice list 2>/dev/null | grep -q '/move_base/make_plan' && break; sleep 1; done
echo "--- node ---"; tail -5 /work/node.log; rosservice list 2>/dev/null | grep -c "move_base/make_plan" | sed "s/^/SERVICE_COUNT /"
rosservice call /move_base/make_plan "start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
goal: {header: {frame_id: 'map'}, pose: {position: {x: 99.0, y: 99.0, z: 9.0}, orientation: {w: 1.0}}}
tolerance: 0.1" > /work/plan.out 2>/work/plan.err
python3 - <<'P'
import re, json
plan=open('/work/plan.out').read(); err=open('/work/plan.err').read()
log=open('/work/mb.log').read()
print("RESULT "+json.dumps({"plan_poses": len(re.findall(r'seq:', plan)), "refused_line": [l for l in log.splitlines() if 'refused' in l][-1:] ,
 "delegated": re.findall(r'\[v8 planner\] delegated plan: (\d+) poses', log)[:2],
 "v8_lines": [l for l in log.splitlines() if '[v8 planner]' in l][:3],
 "err": err[-160:], "mb_tail": log.splitlines()[-3:]}))
P
