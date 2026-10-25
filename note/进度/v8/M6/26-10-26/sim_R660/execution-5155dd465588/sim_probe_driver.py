"""R660a 仿真基线采集驱动（容器内）：rospy 订阅 /clock 证流，不与 bash 管道搏斗。

就绪＝launch 内置断言参数（stairs: /world_load_assert/ok；m1: /tf_frame_assert/ok）；
物理推进门＝/clock 消息的 sim 戳实际前进（RTF 依负载浮动，不猜固定等待）；
模型表＝/world_load_assert/loaded（launch 断言已缓存）；传感流＝话题在发布表。
"""
import json
import sys
import time
from pathlib import Path

import rospy

CASE = sys.argv[1] if len(sys.argv) > 1 else 'stairs'
WAIT_PARAM = '/tf_frame_assert/ok' if CASE == 'm1' else '/world_load_assert/ok'


def main():
    rospy.init_node('sim_baseline_driver', anonymous=True, disable_signals=True)
    clocks = []

    def on_clock(msg):
        if len(clocks) < 2 or clocks[-1] != msg.clock.to_sec():
            clocks.append(msg.clock.to_sec())
        if len(clocks) > 200:
            clocks.pop(0)

    rospy.Subscriber('/clock', rospy.AnyMsg, lambda m: on_clock(_clock(m)))
    t0 = time.time()
    # ①就绪：断言参数出现
    while time.time() - t0 < 90 and not rospy.is_shutdown():
        try:
            if rospy.get_param(WAIT_PARAM, None) is not None:
                break
        except Exception:
            pass
        time.sleep(1)
    # ②物理推进门：sim 戳推进 ≥3.5 s（覆盖 tf 断言 3 s 观察窗；RTF 依负载 0.02—0.3，
    # 低 RTF 时需 ~150 s 墙钟——不猜固定等待，认条件），最多 420 s 墙钟
    deadline = time.time() + 420
    first = None
    advanced_to = None
    while time.time() < deadline and not rospy.is_shutdown():
        if clocks:
            if first is None:
                first = clocks[0]
            for value in clocks[1:]:
                if value > first:
                    advanced_to = value
                    break
        if advanced_to is not None and advanced_to - first >= 3.5:
            break
        time.sleep(2)
    time.sleep(2)

    tf_ok = rospy.get_param('/tf_frame_assert/ok', None)
    world_ok = rospy.get_param('/world_load_assert/ok', None)
    models = rospy.get_param('/world_load_assert/loaded', [])
    if isinstance(models, str):
        models = [models]
    # spawn 证据以日志为准（loaded 是断言节点早期缓存，与 spawn 完成有竞态）
    log = Path('/opt/ros1-slam/r660/logs/roslaunch-%s.log' % CASE)
    log_text = log.read_text(errors='replace') if log.exists() else ''
    spawn_done = ('spawn_lite3_planar' in log_text
                  and 'finished cleanly' in log_text.split('spawn_lite3_planar')[-1])
    published = [t for t, _ in rospy.get_published_topics()]
    result = {
        'stage': 'sim_baseline', 'case': CASE,
        'clock_first': first, 'clock_advanced_to': advanced_to,
        'clock_advancing': advanced_to is not None,
        'clock_samples_tail': clocks[-3:],
        'tf_frame_assert_ok': tf_ok, 'world_load_assert_ok': world_ok,
        'models_loaded': models,
        'models_assert_cached': models,
        'spawn_present': spawn_done,
        'world_models_present': all(m in models for m in ('ground_plane', 'stairs_easy')),
        'lidar_topic_present': '/rslidar_points' in published,
        'wall_duration_s': round(time.time() - t0, 1),
    }
    if CASE == 'm1':
        result['accepted'] = bool(result['clock_advancing']
                                  and result['tf_frame_assert_ok'] is True
                                  and result['lidar_topic_present'])
    else:
        result['accepted'] = bool(result['clock_advancing']
                                  and result['tf_frame_assert_ok'] is True
                                  and result['world_load_assert_ok'] is True
                                  and result['spawn_present']
                                  and result['world_models_present'])
    Path('/opt/ros1-slam/r660/sim-baseline.json').write_text(json.dumps(result, indent=2) + '\n')
    print('SIM_BASELINE', json.dumps(result, ensure_ascii=False))
    return 0 if result['accepted'] else 6


def _clock(raw):
    from rosgraph_msgs.msg import Clock
    m = Clock()
    m.deserialize(raw._buff)
    return m


if __name__ == '__main__':
    sys.exit(main())
