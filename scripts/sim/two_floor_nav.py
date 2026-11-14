#!/usr/bin/env python3
"""Plan/check a complete two-floor scene; explicitly execute only after preflight."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = Path(os.environ.get('V8_SIM_ROOT') or (ROOT / 'sim_ws'))
SCENE = (SIM_ROOT / 'ws/src/lite3_gazebo_sim/scenarios/two_floor_nav'
         if os.environ.get('V8_SIM_ROOT')
         else SIM_ROOT / 'src/lite3_gazebo_sim/scenarios/two_floor_nav')
sys.path.insert(0, str(SCENE))
import scene
import validation


def validate_window(root, window_id, case, orchestrator='direct'):
    """Check the original receipt, never create or modify a registration."""
    if not window_id:
        raise ValueError('--execute requires a registered --window-id')
    root = Path(root).resolve()
    receipt = root / 'scripts/verify/_window_receipt.tsv'
    with receipt.open(encoding='utf-8', newline='') as stream:
        rows = [row for row in csv.reader(stream, delimiter='\t')
                if row and not row[0].lstrip().startswith('#') and row[0] == window_id]
    if len(rows) != 1 or len(rows[0]) != 4:
        raise ValueError('window ID must match exactly one four-column receipt')
    row = rows[0]
    if row[2] != 'sim':
        raise ValueError('window is not a simulation registration')
    declared = Path(row[1])
    if declared.is_absolute() or '..' in declared.parts:
        raise ValueError('window directory must be relative without escape')
    directory = (root / declared).resolve()
    try:
        directory.relative_to(root)
    except ValueError as error:
        raise ValueError('window directory resolves outside repository') from error
    if not directory.is_dir():
        raise ValueError('registered window directory does not exist')
    if not re.search(r'\btwo_floor_nav\b', row[3]) or not re.search(r'\bcase=' + re.escape(case) + r'(?:\s|$)', row[3]):
        raise ValueError('window receipt must name two_floor_nav and the selected case')
    if orchestrator == 'bt' and not re.search(r'\borchestrator=bt\b', row[3]):
        raise ValueError('window receipt must name orchestrator=bt')
    return {'id': window_id, 'directory': str(declared), 'kind': row[2], 'content': row[3]}


def ports_idle(port):
    if type(port) is not int or not 1024 <= port <= 65534 or port in (11310, 11311, 11312):
        raise ValueError('invalid/production ROS master port')
    for selected in (port, port + 1):
        with socket.socket() as probe:
            probe.settimeout(.2)
            if probe.connect_ex(('127.0.0.1', selected)) == 0:
                raise ValueError('requested port already has a listener: ' + str(selected))


def app_path_for(case, orchestrator):
    suffix = '_bt' if orchestrator == 'bt' else ''
    return SCENE / 'apps' / (case + suffix + '.yaml')


def plan_case(case, orchestrator='direct'):
    different = scene.check_artifacts()
    if different:
        raise ValueError('scene artifacts differ from source: ' + ', '.join(different))
    sys.path.insert(0, str(scene.PLATFORM / 'framework'))
    from nodes import app_runner
    import support_planner
    app_path = app_path_for(case, orchestrator)
    prepared = app_runner.prepare_execution(app_path, with_gazebo=True)
    task = prepared['resolved']['task_resolved']
    map_value = prepared['resolved']['map_value']
    result = {'ok': True, 'scope': 'offline configuration/global planning only; no ROS or Gazebo started',
              'case': case, 'app': str(app_path), 'map_sha256': prepared['resolved']['map_sha256'],
              'execution_mode': prepared['plan']['execution_mode'], 'robot_id': prepared['plan']['robot_id'],
              'task': task, 'four_foot_physics': 'NOT_TESTED',
              'global_corridor_following': 'NOT_ESTABLISHED: adapter forwards global final goal only'}
    try:
        route = support_planner.plan(map_value, task['start']['xyz'], task['goal']['xyz'], budget=50000,
                    robot_radius_m=scene.RADIUS + scene.CLEARANCE, robot_height_m=scene.HEIGHT,
                    min_clearance_m=scene.HEIGHT + scene.CLEARANCE)
        if case == 'blocked':
            raise ValueError('blocked fixture unexpectedly produced a route')
        result['global_route'] = route
    except support_planner.PlannerError as error:
        if case != 'blocked' or error.code != 'E_PLAN_NO_ROUTE':
            raise
        result['expected_refusal'] = error.code
    return result


def _preflight_env():
    """Alias root for the preflight only; the run keeps the canonical workspace.

    rospack must see /home/ros1-slam for item [4], but rosrun finds duplicate
    executables when both repo aliases sit in ROS_PACKAGE_PATH.  Keep the alias
    override local to the preflight subprocess.
    """
    env = dict(os.environ)
    alias = os.environ.get('V8_FORMAL_ALIAS_ROOT')
    if alias:
        env['ROS_PACKAGE_PATH'] = alias + '/ws/src:/opt/ros/noetic/share'
        env['ISO_ALLOW'] = alias + ':/opt/ros1-slam:/opt/ros/noetic:/usr:/opt/3rdparty:/etc'
    return env


def execute_case(case, output, port=11657, timeout_s=None, gui=False, interactive=False,
                 rviz=False, window_id=None, orchestrator='direct'):
    # All fail-closed checks precede process creation; plan is allowed anywhere.
    plan = plan_case(case, orchestrator)
    receipt = validate_window(ROOT, window_id, case, orchestrator)
    ports_idle(port)
    output = Path(output).absolute()
    if not output.parent.is_dir():
        raise ValueError('output parent directory must already exist')
    paths = {'summary': output, 'app': output.with_suffix('.app.json'),
             'trace': output.with_suffix('.trace.json'), 'preflight': output.with_suffix('.preflight.log')}
    if len(set(paths.values())) != len(paths) or any(p.exists() or p.is_symlink() for p in paths.values()):
        raise ValueError('output files must be new, distinct paths; preserve previous evidence')
    for name in ('roscore', 'roslaunch', 'gzserver'):
        if shutil.which(name) is None:
            raise ValueError('prepared x86 ROS1/Gazebo environment missing: ' + name)
    timeout_s = (20. if case == 'blocked' else 180.) if timeout_s is None else timeout_s
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('timeout must be positive finite seconds')
    preflight = ROOT / 'scripts/verify/v8_window_preflight.sh'
    checked = subprocess.run(['bash', str(preflight)], cwd=ROOT, text=True, env=_preflight_env(),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
    paths['preflight'].write_text(checked.stdout, encoding='utf-8'); scene.acl(paths['preflight'])
    if checked.returncode:
        raise ValueError('original v8 preflight refused: rc=%d; see %s' % (checked.returncode, paths['preflight']))
    # Recheck after the preflight to reject a new competing listener.
    ports_idle(port)
    command = [sys.executable, str(scene.PLATFORM / 'framework/nodes/app_runner.py'),
               '--app', str(app_path_for(case, orchestrator)), '--execute',
               '--out', str(paths['app']), '--trace-out', str(paths['trace']),
               '--port', str(port), '--timeout-s', str(timeout_s)]
    if gui: command.append('--gui')
    if interactive: command.append('--interactive')
    if rviz: command.append('--rviz')
    process = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
    try:
        app_rc = process.wait()
    except KeyboardInterrupt:
        import signal
        os.killpg(process.pid, signal.SIGINT)
        app_rc = process.wait(timeout=90)
    for key in ('app', 'trace'):
        if paths[key].is_file(): scene.acl(paths[key])
    app_result = json.loads(paths['app'].read_text(encoding='utf-8'))
    trace = json.loads(paths['trace'].read_text(encoding='utf-8'))
    result = {'schema': 'two_floor_run/v1', 'case': case, 'window': receipt, 'ok': False,
              'preflight_rc': checked.returncode, 'app_rc': app_rc,
              'files': {key: str(value) for key, value in paths.items()},
              'offline_plan': {'map_sha256': plan['map_sha256'], 'task': plan['task']}}
    try:
        if app_rc not in (0, 6) or (case != 'blocked' and app_rc != 0):
            raise validation.ValidationError('unexpected app subprocess return code: ' + str(app_rc))
        result['validation'] = validation.validate_trace(trace, case, app_result, interactive or rviz)
        result['ok'] = result['validation']['ok']
    except (validation.ValidationError, KeyError, TypeError, ValueError) as error:
        result['validation_error'] = str(error)
    output.write_bytes(scene.json_bytes(result)); scene.acl(output)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=scene.CASES, default='upstairs')
    parser.add_argument('--execute', action='store_true', help='requires registered window, prepared x86 env and original preflight rc0')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--port', type=int, default=11657)
    parser.add_argument('--timeout-s', type=float)
    parser.add_argument('--window-id')
    parser.add_argument('--orchestrator', choices=('direct', 'bt'), default='direct')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--interactive', action='store_true')
    parser.add_argument('--rviz', action='store_true', help='implies interactive')
    args = parser.parse_args()
    try:
        if args.execute:
            if args.out is None:
                raise ValueError('--execute requires a new --out path')
            result = execute_case(args.case, args.out, args.port, args.timeout_s, args.gui,
                                  args.interactive, args.rviz, args.window_id, args.orchestrator)
        else:
            if args.gui or args.interactive or args.rviz or args.window_id or args.timeout_s is not None:
                raise ValueError('runtime options require --execute')
            result = plan_case(args.case, args.orchestrator)
            if args.out:
                if args.out.exists() or args.out.is_symlink(): raise ValueError('output already exists')
                args.out.write_bytes(scene.json_bytes(result)); scene.acl(args.out)
        print(json.dumps({'ok': result['ok'], 'case': args.case, 'scope': result.get('scope', 'observed Gazebo run'),
                          'expected_refusal': result.get('expected_refusal'),
                          'validation_error': result.get('validation_error'),
                          'output': str(args.out) if args.out else None}, ensure_ascii=False))
        return 0 if result['ok'] else 6
    except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as error:
        sys.stderr.write('ERROR: %s\n' % error)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
