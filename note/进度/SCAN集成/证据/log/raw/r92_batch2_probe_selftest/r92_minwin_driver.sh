#!/usr/bin/env bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11313
export ROS_LOG_DIR=/tmp/r92_roslog
roscore -p 11313 >/tmp/r92_core.log 2>&1 &
CORE=$!
for i in $(seq 1 25); do ss -lnt 2>/dev/null | grep -q ':11313' && break; sleep 1; done
echo "core_listening=$(ss -lnt 2>/dev/null | grep -c ':11313')"
python3 /tmp/r92_fake_tf.py >/tmp/r92_fake.log 2>&1 &
FAKE=$!
sleep 3
bash /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/v_scan_tf_parent_probe.sh 最小图阳性对照 > /tmp/r92_probe.txt 2>&1
echo "probe_rc=$?"
RC2=$?
STP_CHILD=no_such_frame bash /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/v_scan_tf_parent_probe.sh 目标帧缺席自检 >/tmp/r92_probe2.txt 2>&1
echo "probe2_rc=$?"
kill $FAKE $CORE 2>/dev/null
sleep 2
pkill -f "roscore -p 11313" 2>/dev/null; pkill -f "rosmaster --core -p 11313" 2>/dev/null
echo "after_ports=$(ss -lnt 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b')"
echo "after_procs=$(pgrep -f 'roscore|rosmaster|fake_tf' | wc -l)"
