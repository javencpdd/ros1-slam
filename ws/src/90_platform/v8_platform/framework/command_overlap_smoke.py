#!/usr/bin/env python3
"""Real ROS smoke: zero-gap command handover with the real mux priority table."""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/tmp/r1034_smoke.json')
MUX_CONFIG = HERE / 'fixtures' / 'mux' / 'dual_cmd_mux.yaml'
PORT = 11520
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1034_ros',
                  ROS_LOG_DIR='/tmp/r1034_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1034_ros').mkdir(exist_ok=True)
Path('/tmp/r1034_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def main():
    import rospy
    from geometry_msgs.msg import Twist
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_command_overlap.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    plan['instances'][0]['params']['mux_config'] = str(MUX_CONFIG)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1034_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    runner = None
    running = threading.Event()
    try:
        time.sleep(3)
        rospy.init_node('r1034_smoke', anonymous=True, disable_signals=True)
        instance = plan['instances'][0]
        command, marker = runtime.default_command(
            instance, catalog['components'][instance['component']])
        spec = runtime.build_specs(plan, catalog)[0]
        runner = runtime.ProcessRunner(spec, command, ready_marker=marker)
        runner.start()
        result['command'] = command[:2]
        os.environ['ROSCONSOLE_FORMAT'] = '${message}'
        pub_a = rospy.Publisher('/v8/handover/cmd_a', Twist, queue_size=1)
        pub_b = rospy.Publisher('/v8/handover/cmd_b', Twist, queue_size=1)
        outputs = []
        rospy.Subscriber('/cmd_vel', Twist,
                         lambda message: outputs.append((time.monotonic(),
                                                         message.linear.x)),
                         queue_size=20)
        feed = {'a': 0.3, 'b': None}
        lock = threading.Lock()
        running.set()

        def message(value):
            twist = Twist()
            twist.linear.x = value
            return twist

        def feed_loop():
            while running.is_set() and not rospy.is_shutdown():
                with lock:
                    a, b = feed['a'], feed['b']
                pub_a.publish(message(a))
                if b is not None:
                    pub_b.publish(message(b))
                time.sleep(.05)

        threading.Thread(target=feed_loop, daemon=True).start()
        time.sleep(1.5)
        result['phase_a'] = round(outputs[-1][1], 4) if outputs else None
        with lock:
            feed['b'] = 0.15
        time.sleep(.3)
        switch_start = len(outputs)
        time.sleep(.5)
        switched = outputs[switch_start:]
        result['phase_overlap'] = round(outputs[-1][1], 4) if outputs else None
        result['overlap_min'] = round(min(item[1] for item in switched), 4) if switched else None
        with lock:
            feed['b'] = None
        time.sleep(1.0)
        fallback_start = len(outputs)
        time.sleep(.6)
        fallback = outputs[fallback_start:]
        result['phase_fallback'] = round(outputs[-1][1], 4) if outputs else None
        result['fallback_min'] = round(min(item[1] for item in fallback), 4) if fallback else None
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        running.clear()
        if runner is not None and not runner.is_stopped():
            runner.stop('process_group_signal')
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and not runner.is_stopped():
                time.sleep(.1)
            if not runner.is_stopped():
                runner.force()
        result['stopped'] = bool(runner and runner.is_stopped())
        os.killpg(master.pid, signal.SIGTERM)
        master.wait(timeout=10)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'safe_cmd_vel_mux.py' in row[2]
                    and 'r1034_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('phase_a') == .3
                        and result.get('phase_overlap') == .15
                        and (result.get('overlap_min') or 0) > .1
                        and result.get('phase_fallback') == .3
                        and (result.get('fallback_min') or 0) > .1
                        and result.get('stopped') and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
