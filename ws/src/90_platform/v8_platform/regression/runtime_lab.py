"""Own an isolated ROS master and optional Gazebo stack; always reap children.

Run in the fixed Noetic image after the approved preflight. Artifacts go to an
explicit writable directory, never into the read-only source tree. No hardware
bridge is launched. Backend tuning is experimental and does not modify locks.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import random
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

import yaml

BASE = Path(__file__).resolve().parents[1]
REPO = BASE.parents[3]
SIM = REPO / 'ws/src/95_simulation/lite3_gazebo_sim'
FLAT = BASE / 'calibration/flat2d'
sys.path.insert(0, str(FLAT))
from geometry import box_dist
sys.path.insert(0, str(BASE / 'orchestrator'))
from process_runner import process_group_members


class Processes:
    def __init__(self, output):
        self.output = output
        self.children = []

    def start(self, name, args, extra_env=None):
        stream = open(self.output / (name + '.log'), 'wb')
        env = dict(os.environ, **(extra_env or {}))
        process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        self.children.append((name, process, stream))
        return process

    def stop(self, process):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                process.poll()
                if not process_group_members(process.pid):
                    process.wait(timeout=5)
                    return []
                time.sleep(.02)
        process.wait(timeout=5)
        return process_group_members(process.pid)

    def close(self):
        survivors = {}
        for _name, process, stream in reversed(self.children):
            try:
                survivors[_name] = self.stop(process)
            finally:
                stream.close()
        return survivors


def wait_for(predicate, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except Exception:
            pass
        time.sleep(0.1)
    raise RuntimeError('readiness timeout: ' + label)


def make_map(output):
    model = json.loads((FLAT / 'world_model.json').read_text())
    width, height = model['size']
    resolution = model['resolution']
    ox, oy = model['origin']
    image = bytearray()
    for iy in range(height - 1, -1, -1):
        for ix in range(width):
            x, y = ox + (ix + 0.5) * resolution, oy + (iy + 0.5) * resolution
            image.append(0 if any(box_dist(x, y, *box) <= 0 for box in model['boxes']) else 254)
    (output / 'map.pgm').write_bytes(('P5\n%d %d\n255\n' % (width, height)).encode() + image)
    (output / 'map.yaml').write_text(
        'image: %s\nresolution: %s\norigin: [%s, %s, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' %
        (output / 'map.pgm', resolution, ox, oy))


def backend_launch(output, backend, global_backend='global_planner'):
    source = FLAT / ('flat2d_stack_sc09_f64nli25%s.launch' % ('_teb' if backend == 'teb' else ''))
    tree = ET.parse(source)
    root = tree.getroot()
    for node in list(root):
        if node.get('name') == 'map_to_odom':
            root.remove(node)  # Gazebo launch already owns this exact TF child.
        if node.get('name') == 'map_server':
            node.set('args', str(output / 'map.yaml'))
    move = next(node for node in root if node.get('name') == 'move_base')
    planner = ('navfn/NavfnROS' if global_backend == 'navfn'
               else 'global_planner/GlobalPlanner')
    existing = next((p for p in move.findall('param')
                     if p.get('name') == 'base_global_planner'), None)
    if existing is None:
        existing = ET.SubElement(move, 'param', {'name': 'base_global_planner'})
    existing.set('value', planner)
    if global_backend == 'navfn':
        for key, value in (('allow_unknown', 'false'), ('default_tolerance', '0.0'),
                           ('planner_window_x', '0.0'), ('planner_window_y', '0.0')):
            param = ET.SubElement(move, 'param', {'name': 'NavfnROS/' + key,
                                                  'value': value})
    if backend == 'teb':
        values = json.loads((FLAT / 'teb_runtime.json').read_text())
        for key, value in values.items():
            name = 'TebLocalPlannerROS/' + key
            param = next((p for p in move.findall('param') if p.get('name') == name), None)
            if param is None:
                param = ET.SubElement(move, 'param', {'name': name})
            param.set('value', str(value).lower())
        (output / 'teb_experimental_parameters.json').write_text(json.dumps(values, indent=2))
    path = output / (backend + '.launch')
    tree.write(path, encoding='unicode')
    return path


def manifest_for_backend(manifest, backend, global_backend='global_planner'):
    """Keep the copied v12 manifest honest about which planners ran."""
    result = copy.deepcopy(manifest)
    result['backend'] = backend
    result['global_backend'] = global_backend
    result['combo'] = 'lite3_%s_%s_mux' % (global_backend, backend)
    lock = result.setdefault('config_lock', {})
    if global_backend == 'navfn':
        result['global_planner'] = 'navfn/NavfnROS'
        lock['planner'] = lock.get('planner', '').replace(
            'global_planner/GlobalPlanner', 'navfn/NavfnROS')
    else:
        result['global_planner'] = 'global_planner/GlobalPlanner'
    if backend == 'teb':
        result['local_planner'] = 'teb_local_planner/TebLocalPlannerROS'
        lock['planner'] = lock.get('planner', '').replace(
            'dwa_local_planner/DWAPlannerROS', 'teb_local_planner/TebLocalPlannerROS')
        stack = lock.get('stack', '')
        lock['stack'] = stack.replace(
            'flat2d_stack_sc09_f64nli25.launch',
            'flat2d_stack_sc09_f64nli25_teb.launch').replace(
            'DWA 参数在 DWAPlannerROS ns', 'TEB 参数在 TebLocalPlannerROS ns').replace(
            'DWA 默认块', 'TEB 参数块')
    else:
        result['local_planner'] = 'dwa_local_planner/DWAPlannerROS'
    return result


def _pid_state(pid):
    try:
        return (Path('/proc') / str(pid) / 'stat').read_text().rpartition(')')[2].split()[0]
    except (OSError, IndexError):
        return 'gone'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--port', type=int, default=11329)
    parser.add_argument('--interfaces', action='store_true')
    parser.add_argument('--a6', action='store_true')
    parser.add_argument('--a5', action='store_true')
    parser.add_argument('--ndt-probe', action='store_true')
    parser.add_argument('--gazebo', action='store_true')
    parser.add_argument('--backend', choices=['dwa', 'teb'])
    parser.add_argument('--global-backend', choices=['global_planner', 'navfn'],
                        default='global_planner')
    parser.add_argument('--graph', help='v2 graph YAML; Assembly starts nav+mux instead of backend_launch')
    parser.add_argument('--scenario', choices=['SC01', 'SC02', 'SC09'], default='SC02')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--repetition', type=int, default=1)
    parser.add_argument('--stop-test', action='store_true')
    parser.add_argument('--local-gazebo', action='store_true')
    parser.add_argument('--global-route', action='store_true')
    parser.add_argument('--localization', choices=['truth', 'noisy'], default='truth')
    parser.add_argument('--noise-seed', type=int, default=0)
    parser.add_argument('--local-cases', default='arrival,sensed_no_route,localization,sensor_freshness')
    args = parser.parse_args()
    output = Path(args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                      ROS_IP='127.0.0.1', ROS_HOME=str(output / 'ros'),
                      ROS_LOG_DIR=str(output / 'logs'), PYTHONDONTWRITEBYTECODE='1')
    processes = Processes(output)
    results = {}
    graph_assembly = None
    graph_status = {}
    try:
        processes.start('master', ['roscore', '-p', str(args.port)])
        wait_for(lambda: xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/lab')[0] == 1,
                 20, 'master')
        if args.interfaces:
            for command in [('a3_runtime.py', []), ('a4_runtime.py', ['clearance'])]:
                name = command[0][:-3]
                proc = processes.start(name, [sys.executable, '-B', str(BASE / 'regression' / command[0]),
                                             *command[1], '--out', str(output / (name + '.json'))])
                results[name] = proc.wait(timeout=180)
        for enabled, name, script in ((args.a6, 'a6_runtime', BASE / 'regression/a6_runtime.py'),
                                      (args.a5, 'a5_runtime', BASE / 'local_planner/runtime_probe.py'),
                                      (args.ndt_probe, 'ndt_monitor', BASE.parent / 'v8_ndt_odom/test/ros_monitor_probe.py')):
            if enabled:
                proc = processes.start(name, [sys.executable, '-B', str(script),
                                              '--out', str(output / (name + '.json'))])
                results[name] = proc.wait(timeout=150)
        if args.gazebo:
            gazebo = ['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                      'payload_dir:=' + str(SIM), 'gui:=false']
            if args.localization == 'noisy':
                gazebo.append('map_to_odom:=false')
            processes.start('gazebo', gazebo)
            import rospy
            from gazebo_msgs.msg import ModelStates
            rospy.init_node('runtime_lab', anonymous=True, disable_signals=True)
            rospy.wait_for_service('/gazebo/get_model_state', timeout=90)
            wait_for(lambda: 'lite3_planar' in rospy.wait_for_message(
                '/gazebo/model_states', ModelStates, timeout=2).name, 90, 'spawned model')
            if args.localization == 'noisy':
                processes.start('estimated', [sys.executable, '-B',
                                              str(BASE / 'regression/noisy_localization.py'),
                                              '--seed', str(args.noise_seed)])
            if args.backend or args.graph:
                backend, global_backend = args.backend, args.global_backend
                make_map(output)
                if args.graph:
                    sys.path.insert(0, str(BASE / 'framework'))
                    import config as fw_config
                    import graph as fw_graph
                    import launch as fw_launch
                    import render as fw_render
                    import runtime as fw_runtime
                    raw, source_sha = fw_config.load_yaml(args.graph)
                    for inst in raw['instances']:
                        if inst['id'] == 'nav':
                            inst.setdefault('params', {})['map_yaml'] = str(output / 'map.yaml')
                    materialized = output / 'graph.materialized.yaml'
                    materialized.write_text(yaml.safe_dump(raw, sort_keys=False,
                                                           allow_unicode=True))
                    catalog = fw_config.load_components(BASE / 'framework/fixtures/manifests')
                    launch_check = fw_launch.check_components(
                        catalog, fw_launch.load_index(BASE / 'framework/fixtures/package_index.json'))
                    graph = fw_config.load_graph(materialized)
                    report = fw_graph.check(graph, catalog)
                    plan = fw_render.render_plan(graph, catalog, report)
                    (output / 'plan.json').write_text(
                        json.dumps(plan, ensure_ascii=False, indent=1) + '\n')
                    (output / 'graph.launch').write_text(
                        fw_render.render_launch(graph, catalog, report))
                    nav_params = graph['instances']['nav']['params']
                    global_backend = ('navfn' if 'navfn' in nav_params.get(
                        'global_planner', '').lower() else 'global_planner')
                    backend = ('teb' if 'teb' in nav_params.get(
                        'local_planner', '').lower() else 'dwa')
                    graph_assembly = fw_runtime.Assembly(plan, catalog)
                    graph_assembly.start()
                    graph_status.update({
                        'source': str(Path(args.graph).resolve()),
                        'source_sha256': source_sha,
                        'materialized': str(materialized),
                        'digest': report['digest'],
                        'startup_order': report['startup_order'],
                        'command_writers': report['command_writers'],
                        'launch_check': launch_check,
                        'planners': {'global': global_backend, 'local': backend},
                        'ready_after_start': graph_assembly.ready(),
                        'commands': {iid: list(runner.cmd) for iid, runner
                                     in graph_assembly.runners.items()},
                    })
                else:
                    cfg = output / 'mux.yaml'
                    cfg.write_text('output_topic: /cmd_vel\noutput_rate_hz: 20\n'
                                   'hold_topic: /lite3/supervisor/hold\nsources:\n'
                                   '  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}\n')
                    processes.start('mux', [sys.executable, str(REPO / 'ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py'),
                                            '__name:=mux', '_mux_config:=' + str(cfg)])
                    launch = backend_launch(output, args.backend, args.global_backend)
                    processes.start('navigation', ['roslaunch', str(launch)])
                processes.start('relay', [sys.executable, str(FLAT / 'scan_relay.py')],
                                {'RELAY_MODE': 'volume'})
                from nav_msgs.msg import OccupancyGrid
                wait_for(lambda: rospy.wait_for_message('/map', OccupancyGrid, timeout=2).info.width == 200,
                         60, 'map')
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=60)
                if graph_status:
                    graph_status['planner_params'] = {
                        'base_global_planner': rospy.get_param('/move_base/base_global_planner', None),
                        'base_local_planner': rospy.get_param('/move_base/base_local_planner', None)}
                    graph_status['nodes'] = sorted(subprocess.check_output(
                        ['rosnode', 'list'], text=True).split())
                    graph_status['topics'] = {
                        topic: subprocess.run(['rostopic', 'info', topic],
                                              capture_output=True, text=True).stdout
                        for topic in ('/cmd_vel', '/cmd_vel_raw')}
                    graph_status['health_before_goal'] = graph_assembly.health()
                from gazebo_msgs.srv import SetModelState
                from gazebo_msgs.msg import ModelState
                manifest_dir = FLAT / ('runs_v12' if args.scenario == 'SC09' else 'runs_v12_sc0102')
                template = next(manifest_dir.glob('*_%s_g0_r1.json' % args.scenario))
                manifest = manifest_for_backend(json.loads(template.read_text()), backend,
                                                global_backend)
                generator = random.Random(args.seed)
                perturbation = [0., 0., 0.] if args.seed == 0 else [
                    generator.uniform(-.03, .03), generator.uniform(-.03, .03),
                    generator.uniform(-.02, .02)]
                state = ModelState(model_name='lite3_planar', reference_frame='world')
                state.pose.position.x, state.pose.position.y = perturbation[:2]
                state.pose.position.z = 0.112
                state.pose.orientation.z = math.sin(perturbation[2] / 2)
                state.pose.orientation.w = math.cos(perturbation[2] / 2)
                rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
                localization_source = (
                    'Gazebo ground-truth TF; not LIO or NDT' if args.localization == 'truth'
                    else 'seeded map->odom perturbation (amp 0.05 m / 0.03 rad); '
                         'truth /sim/odom_gt used for evaluation')
                seed_effect = {'start_xy_yaw': perturbation}
                if args.localization == 'noisy':
                    seed_effect['estimated_track'] = {
                        'seed': args.noise_seed, 'amp_xy_m': 0.05, 'amp_yaw_rad': 0.03}
                track_suffix = '' if args.localization == 'truth' else '_' + args.localization
                manifest.update(run_id='runtime_%s_%s_%s_s%d%s_r%d' % (
                    global_backend, backend, args.scenario, args.seed,
                    track_suffix, args.repetition),
                                run_class='runtime_experimental', calibration_only=False,
                                seed=args.seed, repetition=args.repetition,
                                start=perturbation, seed_effect=seed_effect,
                                track=('truth' if args.localization == 'truth' else 'estimated'),
                                localization_source=localization_source,
                                localization_track=args.localization,
                                source_manifest_sha256=hashlib.sha256(template.read_bytes()).hexdigest(),
                                telemetry_json=str(output / 'trajectory.json'))
                if graph_status:
                    manifest['assembly'] = {
                        'graph': graph_status['source'],
                        'graph_sha256': graph_status['source_sha256'],
                        'digest': graph_status['digest'],
                        'startup_order': graph_status['startup_order']}
                if manifest.get('inject') and args.seed != 0:
                    manifest['inject']['at_progress_m'] += generator.uniform(-.03, .03)
                    manifest['seed_effect']['inject_at_progress_m'] = manifest['inject']['at_progress_m']
                manifest['criteria']['deadline_s'] = 120
                path = output / 'manifest.json'
                path.write_text(json.dumps(manifest, indent=2))
                driver = processes.start('driver', [sys.executable, str(FLAT / 'driver.py'),
                                                   str(path), str(output / 'driver.json')])
                results['navigation'] = driver.wait(timeout=230)
                if graph_assembly is not None:
                    graph_status['health_after_goal'] = graph_assembly.health()
                    if graph_status['health_after_goal']:
                        raise RuntimeError('graph components died: %r'
                                           % graph_status['health_after_goal'])
            if args.stop_test:
                proc = processes.start('stop', [sys.executable, str(BASE / 'regression/a2_gazebo_stop.py'),
                                                '--out', str(output / 'stop.json')])
                results['stop'] = proc.wait(timeout=100)
            if args.local_gazebo:
                command = [sys.executable,
                    str(BASE / 'regression/a5_gazebo_runtime.py'),
                    '--out', str(output / 'a5_gazebo.json'), '--cases', args.local_cases]
                if args.global_route:
                    command.append('--global-route')
                proc = processes.start('local_gazebo', command)
                results['local_gazebo'] = proc.wait(timeout=500)
        return 0 if all(code == 0 for code in results.values()) else 6
    finally:
        unstopped = {}
        try:
            if graph_assembly is not None:
                graph_assembly.stop()
                graph_status['stop_record'] = graph_assembly.stop_record
                unstopped = {iid: not runner.is_stopped()
                             for iid, runner in graph_assembly.runners.items()}
                graph_status['unstopped'] = unstopped
                graph_status['runner_state'] = {iid: {
                    'alive': runner.is_alive(), 'groups': runner.group_members(),
                    'tracked_pids': sorted(runner._tracked_pids),
                    'tracked_pgids': sorted(runner._tracked_pgids),
                    'reader_alive': bool(runner._reader and runner._reader.is_alive()),
                    'pid_states': {str(pid): _pid_state(pid) for pid in runner._tracked_pids}}
                    for iid, runner in graph_assembly.runners.items()}
        finally:
            if graph_status:
                (output / 'graph_runtime.json').write_text(
                    json.dumps(graph_status, ensure_ascii=False, indent=1) + '\n')
            survivors = processes.close()
            results['process_group_survivors'] = survivors
            results['cleaned_processes'] = [name for name, members in survivors.items()
                                            if not members]
            (output / 'lab.json').write_text(json.dumps(results, indent=2))
            print(json.dumps(results), flush=True)
        if any(survivors.values()) or any(unstopped.values()):
            raise RuntimeError('process groups survived cleanup: %r %r'
                               % (survivors, unstopped))


if __name__ == '__main__':
    sys.exit(main())
