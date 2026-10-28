"""A4 运行面 cell：把 support_planner 作为 ROS1 节点实跑（真值/扰动双轨），
独立评价器按真值图判分。

与 regression.py（设计层矩阵）的分工：本件只把「规划调用」换成真实 ROS 节点进程
（planner/ros_node.py 的 make_plan 服务），判分口径与 _run_cell 一致——规划图随轨、
评价恒按真值图；另加一条运行面一致性：节点返回的路径点列须与设计层同图同参的
结果逐点一致（A4「小图与参考解一致」的运行面落点）。

ROS 只在 call_node 内导入：纯逻辑可在无 ROS 的宿主上被测试。
用法（容器内，master 已起）：
    python3 -m a4_runtime run [--out /work/a4_runtime.json]
"""
import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'calibration', 'planner', 'contracts'):
    sys.path.insert(0, str(ROOT / sub))

import calibration as cal  # noqa: E402
import regression as reg  # noqa: E402
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402

NODE_PATH = str(ROOT / 'planner' / 'ros_node.py')


def planning_ids():
    return [sid for sid in cal.FROZEN['denominators']['official_scenarios']['value']
            if scn.SCENARIOS[sid].get('kind', 'planning') == 'planning']


def judge(case, route=None, error_code=None, evaluation=None):
    """与 regression._run_cell 同口径；route=(x,y,z) 列表，evaluation 可注入。"""
    expected = case['truth'].get('expected_reason')
    if route is None:
        reason = error_code or 'E_PLAN_UNKNOWN'
        return {'stage': 'plan', 'reason': reason,
                'outcome': 'expected_failure' if reason == expected else 'unexpected_failure'}
    verdict = evaluation if evaluation is not None else scn.evaluate(case, route)
    if verdict['ok']:
        return {'stage': 'eval', 'reason': 'ok', 'outcome': 'pass'}
    reason = ';'.join(verdict['failures']) or 'checker'
    if expected and expected in reason:
        return {'stage': 'eval', 'reason': reason, 'outcome': 'expected_failure'}
    return {'stage': 'eval', 'reason': reason, 'outcome': 'unexpected_failure'}


def same_points(a, b):
    """逐点一致性：A4 运行面返回路径须与设计层参考解完全同点；双双拒绝也算一致。"""
    if a is None or b is None:
        return a is None and b is None
    if len(a) != len(b):
        return False
    return all(abs(x1 - x2) <= 1e-9 and abs(y1 - y2) <= 1e-9 and abs(z1 - z2) <= 1e-9
               for (x1, y1, z1), (x2, y2, z2) in zip(a, b))


def call_node(map_value, start, goal, budget, name, service_timeout=30.0, keep_alive_s=5.0,
              cancel_first=False, min_clearance_m=None):
    """起一个 ros_node 子进程，调 make_plan；返回 (route|None, error_code|None, info)。

    cancel_first=True 时先调节点的 Trigger 取消服务（预取消），期望本次规划返回 E_PLAN_CANCELLED。
    """
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.srv import GetPlan

    handle, map_path = tempfile.mkstemp(prefix='a4_', suffix='.json')
    with os.fdopen(handle, 'w', encoding='utf-8') as stream:
        json.dump(map_value, stream)
    cmd = [sys.executable, NODE_PATH, '__name:=' + name,
           '_map_json:=' + map_path,
           '_start:=' + json.dumps(list(start)),
           '_goal:=' + json.dumps(list(goal)),
           '_budget:=' + str(int(budget)),
           '_frame:=map',
           '_make_plan_service:=/' + name + '/make_plan',
           '_status_topic:=/' + name + '/support_route_status',
           '_keep_alive_s:=%g' % keep_alive_s]
    if min_clearance_m is not None:
        cmd.append('_min_clearance_m:=%g' % min_clearance_m)
    env = dict(os.environ)
    env['PYTHONPATH'] = str(ROOT / 'planner') + os.pathsep + env.get('PYTHONPATH', '')
    try:
        rospy.delete_param('/' + name + '/support_route_status')
    except Exception:
        pass
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        # 负例：节点在启动阶段就拒绝（无路/预算），不挂服务直接退出；轮询进程存活，
        # 以它写下的 status 参数为准（退出后参数仍留在 master 上）。
        deadline = time.monotonic() + service_timeout
        ready = False
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                info = rospy.get_param('/' + name + '/support_route_status', None)
                if info is not None and not info.get('ok'):
                    return None, info.get('reason'), info
                return None, 'E_RUNTIME_NODE_EXIT', info
            try:
                rospy.wait_for_service('/' + name + '/make_plan', timeout=0.5)
                ready = True
                break
            except rospy.exceptions.ROSException:
                pass
        if not ready:
            return None, 'E_RUNTIME_SERVICE_TIMEOUT', None
        proxy = rospy.ServiceProxy('/' + name + '/make_plan', GetPlan)
        if cancel_first:
            from std_srvs.srv import Trigger
            rospy.wait_for_service('/' + name + '/cancel', timeout=service_timeout)
            rospy.ServiceProxy('/' + name + '/cancel', Trigger)()
        request_start, request_goal = PoseStamped(), PoseStamped()
        for msg, xyz in ((request_start, start), (request_goal, goal)):
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
            msg.pose.orientation.w = 1.0
        response = proxy(request_start, request_goal, 0.0)
        info = rospy.get_param('/' + name + '/support_route_status', None)
        if info is None:
            return None, 'E_RUNTIME_NO_STATUS', None
        if not info.get('ok'):
            return None, info.get('reason'), info
        route = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                 for p in response.plan.poses]
        if not route:
            return None, 'E_RUNTIME_EMPTY_PATH', info
        return route, None, info
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        os.unlink(map_path)


def run(tracks=('truth', 'estimate'), ids=None, call_node_fn=None):
    """跑 A4 运行面 cell 表；call_node_fn 供测试注入。"""
    cells = []
    call = call_node_fn or call_node
    for index, sid in enumerate(ids or planning_ids()):
        case = scn.scenario(sid)
        budget = case.get('planner_budget') or \
            cal.FROZEN['budgets']['planner_max_expansions']['value']
        for track in tracks:
            map_for_plan = reg._perturbed(case['map']) if track == 'estimate' else case['map']
            name = 'a4_%s_%s' % (sid.lower(), track)
            route, error_code, info = call(map_for_plan, case['start'], case['goal'],
                                           budget, name)
            # 设计层同图同参参考解（运行面一致性对照；同用本轨输入图，评价仍按真值图）
            design = None
            try:
                planned = pln.plan(map_for_plan, case['start'], case['goal'], budget=budget)
                design = [(x, y, z) for x, y, z, _sid in planned['path']]
            except pln.PlannerError:
                design = None
            cell = {'scenario': sid, 'track': track, 'budget': budget,
                    'points': None if route is None else len(route),
                    'design_points': None if design is None else len(design),
                    'same_as_design': same_points(route, design),
                    'expansions': None if not info else info.get('expansions'),
                    'info': info}
            cell.update(judge(case, route, error_code))
            cells.append(cell)
    return cells


def run_cancel(ids=('SC01', 'SC02'), call_node_fn=None):
    """A4 取消可区分：预取消后规划必须返回 E_PLAN_CANCELLED，而不是路径/无路/超预算。"""
    cells = []
    call = call_node_fn or call_node
    for sid in ids:
        case = scn.scenario(sid)
        budget = cal.FROZEN['budgets']['planner_max_expansions']['value']
        name = 'a4_%s_cancel' % sid.lower()
        route, error_code, info = call(case['map'], case['start'], case['goal'], budget, name,
                                       cancel_first=True)
        outcome = 'pass' if (route is None and error_code == 'E_PLAN_CANCELLED') \
            else 'unexpected_failure'
        cells.append({'scenario': sid, 'track': 'cancel', 'budget': budget,
                      'points': None if route is None else len(route),
                      'stage': 'plan', 'reason': error_code or 'route_returned',
                      'outcome': outcome, 'info': info})
    return cells


def run_clearance(call_node_fn=None):
    """SC09 原几何安全规划＋派生低顶目标请求负例；不改旧冻结场景的预期。

    原场景有合法绕行，因此不能将其整幅图硬判无路。负例只改变请求目标到低顶下方，
    保持原始地图与冻结净空值，必须在 plan 阶段拒绝，不由评价器代判。
    """
    case = scn.scenario('SC09')
    call = call_node_fn or call_node
    limit = cal.FROZEN['tolerances']['min_clearance_m']['value']
    budget = cal.FROZEN['budgets']['planner_max_expansions']['value']
    cells = []
    for track in ('truth', 'estimate'):
        map_value = reg._perturbed(case['map']) if track == 'estimate' else case['map']
        for label, goal, expected in (('official_geometry_safe_route', case['goal'], None),
                                      ('derived_low_roof_goal', (0.0, 0.0, 0.0),
                                       'E_PLAN_CLEARANCE')):
            route, reason, info = call(map_value, case['start'], goal, budget,
                                      'a4_clearance_%s_%s' % (track, label),
                                      min_clearance_m=limit)
            checked = None
            if expected:
                good = route is None and reason == expected
            else:
                checked = scn.evaluate(case, route) if route else None
                good = bool(checked and checked['ok'])
            cells.append({'scenario': 'SC09', 'request': label, 'track': track,
                          'min_clearance_m': limit, 'stage': 'plan' if route is None else 'eval',
                          'reason': reason or 'ok', 'points': 0 if route is None else len(route),
                          'info': info, 'evaluation': checked,
                          'outcome': 'pass' if good else 'unexpected_failure'})
    return cells


def _call_versions(name, scenario_id):
    """起一个节点，在同一临时地图文件上跑 v1→v2（删边）→v3（恢复），记录状态里的版本。"""
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.srv import GetPlan

    case = scn.scenario(scenario_id)
    v1 = case['map']
    v2 = copy.deepcopy(v1)
    v2['map_version'] = v1.get('map_version', 1) + 1
    # 合法更新：分辨率 0.1→0.2（不破坏契约连通/足迹），路径应随之变化
    v2['resolution'] = 0.2
    v3 = copy.deepcopy(v1)
    v3['map_version'] = v1.get('map_version', 1) + 2

    handle, map_path = tempfile.mkstemp(prefix='a3_', suffix='.json')
    with os.fdopen(handle, 'w', encoding='utf-8') as stream:
        json.dump(v1, stream)
    cmd = [sys.executable, NODE_PATH, '__name:=' + name,
           '_map_json:=' + map_path,
           '_start:=' + json.dumps(list(case['start'])),
           '_goal:=' + json.dumps(list(case['goal'])),
           '_budget:=' + str(int(cal.FROZEN['budgets']['planner_max_expansions']['value'])),
           '_frame:=map',
           '_make_plan_service:=/' + name + '/make_plan',
           '_status_topic:=/' + name + '/support_route_status',
           '_keep_alive_s:=60']
    env = dict(os.environ)
    env['PYTHONPATH'] = str(ROOT / 'planner') + os.pathsep + env.get('PYTHONPATH', '')
    try:
        rospy.delete_param('/' + name + '/support_route_status')
    except Exception:
        pass
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    observed = []
    try:
        rospy.wait_for_service('/' + name + '/make_plan', timeout=30.0)
        proxy = rospy.ServiceProxy('/' + name + '/make_plan', GetPlan)
        start, goal = PoseStamped(), PoseStamped()
        for msg, xyz in ((start, case['start']), (goal, case['goal'])):
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
            msg.pose.orientation.w = 1.0
        for label, value in (('v1', None), ('v2', v2), ('v3', v3)):
            if value is not None:
                with open(map_path, 'w', encoding='utf-8') as stream:
                    json.dump(value, stream)
            response = proxy(start, goal, 0.0)
            info = rospy.get_param('/' + name + '/support_route_status', {}) or {}
            observed.append((label, bool(info.get('ok')),
                             info.get('reason') or 'ok',
                             info.get('map_version'),
                             len(response.plan.poses)))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        os.unlink(map_path)
    return observed


def run_version(scenario_id='SC02', call_versions_fn=None):
    """A3 版本追踪：v1 可规划→v2 无路（E_PLAN_NO_ROUTE）→v3 恢复；状态须带对应版本号。"""
    observed = call_versions_fn() if call_versions_fn else _call_versions('a4_version', scenario_id)
    base = scn.scenario(scenario_id)['map'].get('map_version', 1)
    expect = {'v1': ('ok', base), 'v2': ('ok', base + 1), 'v3': ('ok', base + 2)}
    cells = []
    for label, ok, reason, version, points in observed:
        want_ok, want_version = expect[label]
        good = (ok if want_ok == 'ok' else (not ok and reason == want_ok)) \
            and version == want_version
        cells.append({'label': label, 'ok': ok, 'reason': reason, 'map_version': version,
                      'points': points, 'expected_version': want_version,
                      'outcome': 'pass' if good else 'unexpected_failure'})
    # 版本更新必须真的生效：v2 的路径形状应与 v1 不同，v3 回到 v1
    if len(cells) == 3 and cells[0]['points'] is not None:
        if cells[1]['points'] == cells[0]['points']:
            cells[1]['outcome'] = 'unexpected_failure'
            cells[1]['reason'] = 'v2_no_effect'
        if cells[2]['points'] != cells[0]['points']:
            cells[2]['outcome'] = 'unexpected_failure'
            cells[2]['reason'] = 'v3_not_restored'
    return cells


def summary(cells):
    return {'cells': len(cells),
            'passed': sum(1 for c in cells if c['outcome'] == 'pass'),
            'expected_failures': sum(1 for c in cells if c['outcome'] == 'expected_failure'),
            'unexpected_failures': sum(1 for c in cells if c['outcome'] == 'unexpected_failure'),
            'same_as_design': sum(1 for c in cells if c.get('same_as_design'))}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['run', 'cancel', 'version', 'clearance'])
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    import rospy
    rospy.init_node('a4_runtime_runner')
    cells = {'cancel': run_cancel, 'version': run_version,
             'clearance': run_clearance, 'run': run}[args.command]()
    result = {'summary': summary(cells), 'cells': cells}
    print(json.dumps(result['summary'], ensure_ascii=False))
    for cell in cells:
        print('%-10s %-8s %-18s %s %s' % (cell.get('scenario', cell.get('label')),
                                          cell.get('track', '-'), cell['outcome'],
                                          cell.get('stage', '-'), cell.get('reason', '-')))
    if args.out:
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    return 0 if result['summary']['unexpected_failures'] == 0 else 6


if __name__ == '__main__':
    sys.exit(main())
