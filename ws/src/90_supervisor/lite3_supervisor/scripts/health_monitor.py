#!/usr/bin/env python3
# /home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/scripts/health_monitor.py
# SUP 一期最小监控骨架（C13，N5）。红线：本脚本绝不发布任何 cmd_vel 类话题（唯一输出 = hold）。
# P7 修正批（D-P7-1，04 台账）：启动宽限参数化 startup_grace_s（详见 __init__ 内注释）。
# L1 修复（G6-③，04 台账 D-L1-1）：hold 自动复位语义——FATAL 后该模块心跳恢复且连续
#   recover_beats 拍稳定 → hold=false + FATAL 降级 INFO；恢复观察期内再失联 → 计数清零、
#   保持 FATAL（hold 不翻转，防振荡）；auto_recover=false 保持人工 S-4 复位（详见 _check 注释）。
import rospy, yaml, threading
from std_msgs.msg import Bool, Empty

class HealthMonitor:
    def __init__(self):
        mode = rospy.get_param('~mode', 'mode_nav_stub')
        with open(rospy.get_param('~health_watch')) as f:
            cfg = yaml.safe_load(f)
        self.mode_cfg = cfg['modes'][mode]
        self.timeout = rospy.Duration(self.mode_cfg.get('timeout_s', cfg['defaults']['timeout_s']))
        # P7 修正批（D-P7-1）：启动宽限独立于稳态看门狗——startup_grace_s 仅约束各 watch「首包到达前」
        # 的宽限期（mode_nav_stub 一窗实测末位 include 的 m4/m6 心跳首包 ~init+3.4s，旧实现与
        # timeout_s 共用 3.0s 截止 → 启动竞态 hold 误 latch，P7/证据/修正批_D-P7-1.log）。
        # 实现 = 初始化 last_seen 前移 (startup_grace_s - timeout_s)，等效 deadline = init + startup_grace_s；
        # 首包到达后 _beat 以真实时间覆盖 last_seen，稳态失联判定仍为 timeout_s（看门狗不钝化）。
        self.startup_grace = rospy.Duration(self.mode_cfg.get(
            'startup_grace_s', cfg['defaults'].get('startup_grace_s', self.timeout.to_sec())))
        # L1 修复（G6-③）：恢复语义参数——外置 health_watch.yaml（mode 可覆盖 defaults，同
        # timeout_s 模式）。auto_recover=false 时 FATAL 不自动解除（人工 S-4：SUP+M6 双重启）。
        # recover_beats = 解除前需连续观察到「非失联」的看门狗拍数（Timer 1 Hz），3 拍 ≈ 3 s
        # 稳定观察窗；观察期内任一拍判失联即清零计数（防振荡，见 _check 状态机注释）。
        self.auto_recover = bool(self.mode_cfg.get('auto_recover', cfg['defaults'].get('auto_recover', True)))
        self.recover_beats = int(self.mode_cfg.get('recover_beats', cfg['defaults'].get('recover_beats', 3)))
        self.last_seen, self.lock = {}, threading.Lock()
        self.lost = set()          # L1：处于 FATAL 的模块集合（hold=true 的唯一驱动源）
        self.recover_cnt = {}      # L1：恢复观察计数（仅 lost 内模块有意义）
        self.hold_on = False       # L1：hold 输出边缘触发状态
        self.hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)  # 唯一输出，绝无 /cmd_vel
        for w in self.mode_cfg['watch']:
            self.last_seen[w['module']] = rospy.Time.now() + (self.startup_grace - self.timeout)
            rospy.Subscriber(w['topic'], Empty, self._beat, callback_args=w['module'])
        rospy.Timer(rospy.Duration(1.0), self._check)

    def _beat(self, _msg, module):
        with self.lock:
            self.last_seen[module] = rospy.Time.now()

    def _check(self, _event):
        # L1 修复（G6-③）逐模块状态机：
        #   NORMAL --(now-last_seen > timeout_s)--> FATAL（入 self.lost，边缘发布 hold=true + logfatal）
        #   FATAL --(auto_recover 且连续 recover_beats 拍非失联)--> NORMAL（出 self.lost，
        #           边缘发布 hold=false + FATAL 降级 INFO/恢复事件日志）
        #   恢复观察期内任一拍再失联 --> recover_cnt 清零、留在 FATAL（hold 不变，mux 无感，防振荡）
        # 发布与日志均边缘触发：旧实现每拍重复 logfatal+publish(True)，本版仅在状态翻转时动作，
        # 消除失联期每秒刷屏；hold_pub 仍 latch=True，边缘一拍即对后接入者生效。
        now = rospy.Time.now()
        with self.lock:
            snapshot = dict(self.last_seen)
        newly_lost, recovered = [], []
        for m, t in snapshot.items():
            if (now - t) > self.timeout:
                self.recover_cnt[m] = 0                       # 恢复观察中断（防振荡）
                if m not in self.lost:
                    self.lost.add(m)
                    newly_lost.append(m)
            elif m in self.lost and self.auto_recover:
                self.recover_cnt[m] = self.recover_cnt.get(m, 0) + 1
                if self.recover_cnt[m] >= self.recover_beats:
                    self.lost.discard(m)
                    recovered.append(m)
        for m in newly_lost:
            rospy.logfatal('[SUP] 心跳失联: %s —— hold=true（通知 M6 立即零速）；待降级关停 = 本 mode watch 全表；自动恢复=%s（需连续 %d 拍稳定）',
                           m, self.auto_recover, self.recover_beats)
        for m in recovered:
            rospy.loginfo('[SUP][RECOVER] 心跳恢复: %s 连续 %d 拍稳定 —— hold=false，FATAL 降级解除',
                          m, self.recover_beats)
        want_hold = bool(self.lost)
        if want_hold != self.hold_on:
            self.hold_pub.publish(Bool(data=want_hold))
            self.hold_on = want_hold

if __name__ == '__main__':
    rospy.init_node('lite3_supervisor')
    HealthMonitor()
    rospy.spin()
