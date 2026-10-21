#!/usr/bin/env python3
# /home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/scripts/health_monitor.py
# SUP 一期最小监控骨架（C13，N5）。红线：本脚本绝不发布任何 cmd_vel 类话题（唯一输出 = hold）。
import rospy, yaml, threading
from std_msgs.msg import Bool, Empty

class HealthMonitor:
    def __init__(self):
        mode = rospy.get_param('~mode', 'mode_nav_stub')
        with open(rospy.get_param('~health_watch')) as f:
            cfg = yaml.safe_load(f)
        self.mode_cfg = cfg['modes'][mode]
        self.timeout = rospy.Duration(self.mode_cfg.get('timeout_s', cfg['defaults']['timeout_s']))
        self.last_seen, self.lock = {}, threading.Lock()
        self.hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)  # 唯一输出，绝无 /cmd_vel
        for w in self.mode_cfg['watch']:
            self.last_seen[w['module']] = rospy.Time.now()
            rospy.Subscriber(w['topic'], Empty, self._beat, callback_args=w['module'])
        rospy.Timer(rospy.Duration(1.0), self._check)

    def _beat(self, _msg, module):
        with self.lock:
            self.last_seen[module] = rospy.Time.now()

    def _check(self, _event):
        now = rospy.Time.now()
        lost = [m for m, t in self.last_seen.items() if now - t > self.timeout]
        if lost:
            rospy.logfatal('[SUP] 心跳失联: %s —— hold=true（通知 M6 立即零速）；待降级关停 = 本 mode watch 全表', ','.join(lost))
            self.hold_pub.publish(Bool(data=True))

if __name__ == '__main__':
    rospy.init_node('lite3_supervisor')
    HealthMonitor()
    rospy.spin()
