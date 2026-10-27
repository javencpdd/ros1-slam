"""V8-05 组合求解首切片：角色依赖图与结构冲突检测（22 号 §组合流程 步骤2＋步骤5 的检测面）。

分层：本件属领域层，消费注册表（../registry/registry.py）与契约层（经注册表透传），
不 import ROS、不读写运行目录。兼容比对（流程3：维度/地图/消息语义逐对匹配）与
确定性锁文件（流程6）归后续切片。

角色依赖图（ROLE_DEPS，按 22 号 §1 组件职责表；「任一满足」集）：
  global_localization → {odometry}；mapping → {odometry}；representation → {odometry}；
  global_planner → {map_server, representation, mapping}（任一）；
  controller → {local_planner}；trajectory_validator → {local_planner}；safety_arbiter → {controller}。
local_planner 的地图依赖由具体 manifest 的 io 声明表达，归兼容比对切片（流程3），本层不预设。
22 号铁则提醒：未验证能力值禁入组合（robot 层 composition_use=prohibited 由契约层强制）；
本层结果如实携带各组件证据级别，供上层锁文件标注。
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'registry'))
import registry as reg  # noqa: E402  领域层消费注册表

ROLE_DEPS = {
    'odometry': set(),
    'global_localization': {'odometry'},
    'mapping': {'odometry'},
    'representation': {'odometry'},
    'map_server': set(),
    'global_planner': {'map_server', 'representation', 'mapping'},
    'local_planner': set(),
    'controller': {'local_planner'},
    'trajectory_validator': {'local_planner'},
    'safety_arbiter': {'controller'},
    'simulator': set(),
    'evaluator': set(),
}

# 流程3 兼容比对用：地图供需与运动学配对（22 号「比对地图类型、位姿维度、消息语义、控制接口和运动能力」）
MAP_PRODUCER_ROLES = {'map_server', 'representation', 'mapping'}
# 运动链＝输出机体运动指令/轨迹的角色（controller/local_planner）；
# global_planner 产出地图级路径、不执行运动，kinematics=not_applicable 合法（R647 修正）
MOTION_ROLES = {'controller', 'local_planner'}
# (规划器运动学, 控制器运动学) 允许配对：同型可配；四足规划器在 cmd_vel 抽象层可由
# differential/holonomic 控制器执行（Lite3 实栈即 quadruped 规划 × dwa 控制）；
# 其余错配（如 holonomic 规划 × differential 控制）拒绝。
PLANNER_CONTROLLER_OK = {
    ('quadruped_kinematic', 'differential'), ('quadruped_kinematic', 'holonomic'),
    ('quadruped_kinematic', 'quadruped_kinematic'),
    ('differential', 'differential'), ('holonomic', 'holonomic'),
}


class ComposerError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _topo_cycle(roles):
    """角色依赖图（限选中角色）的环检测；返回首个环（空列表＝无环）。"""
    graph = {role: [dep for dep in ROLE_DEPS.get(role, set()) if dep in roles] for role in roles}
    state = {}

    def visit(node, path):
        state[node] = 1
        for nxt in graph[node]:
            if state.get(nxt) == 1:
                return path[path.index(nxt):] + [nxt]
            if state.get(nxt) is None:
                found = visit(nxt, path + [nxt])
                if found:
                    return found
        state[node] = 2
        return []

    for role in sorted(roles):
        if state.get(role) is None:
            found = visit(role, [role])
            if found:
                return found
    return []


def compatibility(store, entries):
    """流程3 兼容比对（本切片面）：地图供需＋运动学配对；冲突抛错，通过返回报告条目。

    消息级语义匹配（io.inputs/outputs 逐对）归 V8-06/07 适配器层，报告留 adapters 槽位。
    """
    report = []

    def raw(entry):
        return _raw_manifest(store, entry)

    # 地图供需：声明真实地图类型且**非生产者角色**的组件构成需求（生产者的 map_schema
    # 自述产出类型）；须有同类生产者在组合内（按 io.map_schema 精确匹配，生产者角色
    # ∈ MAP_PRODUCER_ROLES；dimensions 一致性由契约层 E_MANIFEST_DIM 逐件保证）
    for entry in entries:
        schema = raw(entry)['io']['map_schema']
        if schema not in ('occupancy_grid/v1', 'support_map/v1'):
            continue
        if set(entry['roles']) & MAP_PRODUCER_ROLES:
            continue
        providers = [other['id'] for other in entries
                     if other['id'] != entry['id']
                     and (set(other['roles']) & MAP_PRODUCER_ROLES)
                     and raw(other)['io']['map_schema'] == schema]
        if not providers:
            raise ComposerError('E_COMP_MAP_SUPPLY',
                                '%s 需 %s，组合内无同类生产者' % (entry['id'], schema))
        report.append({'check': 'map_supply', 'component': entry['id'],
                       'schema': schema, 'providers': sorted(providers)})

    # 运动链运动学：运动角色不得 not_applicable；规划器×控制器配对查表
    for entry in entries:
        if (set(entry['roles']) & MOTION_ROLES
                and raw(entry)['capability']['kinematics'] == 'not_applicable'):
            raise ComposerError('E_COMP_KINEMATICS',
                                '%s 属运动链角色，kinematics 不得 not_applicable' % entry['id'])
    planner = next((e for e in entries if 'local_planner' in e['roles']), None)
    controller = next((e for e in entries if 'controller' in e['roles']), None)
    if planner is not None and controller is not None:
        pair = (raw(planner)['capability']['kinematics'],
                raw(controller)['capability']['kinematics'])
        if pair not in PLANNER_CONTROLLER_OK:
            raise ComposerError('E_COMP_KINEMATICS',
                                '规划器 %s(%s) × 控制器 %s(%s) 运动学错配' % (
                                    planner['id'], pair[0], controller['id'], pair[1]))
        report.append({'check': 'kinematics_pair', 'planner': planner['id'],
                       'controller': controller['id'], 'pair': list(pair)})
    return report


def compose(store, component_ids):
    """对选中组件做结构检测；通过返回组合摘要，否则抛 ComposerError（fail-closed）。"""
    component_ids = list(dict.fromkeys(component_ids))  # 去重保序
    if not component_ids:
        raise ComposerError('E_COMP_EMPTY', '未选择任何组件')
    entries = []
    for component_id in component_ids:
        entry = store.get(component_id)
        if entry is None:
            raise ComposerError('E_COMP_UNKNOWN_ID', component_id)
        entries.append(entry)

    # 角色重复（22 号：组合图不能重复分配同一职责）
    role_owner = {}
    for entry in entries:
        for role in entry['roles']:
            if role in role_owner:
                raise ComposerError('E_COMP_DUP_ROLE',
                                    '%s 与 %s 重复角色 %s' % (role_owner[role], entry['id'], role))
            role_owner[role] = entry['id']
    roles = set(role_owner)

    # 缺失角色（任一满足集）
    for role in sorted(roles):
        deps = [dep for dep in ROLE_DEPS.get(role, set())]
        if deps and not (set(deps) & roles):
            raise ComposerError('E_COMP_MISSING_ROLE',
                                '%s 依赖 %s（任一），选中集未提供' % (role, '/'.join(sorted(deps))))

    # 环（角色图；固定 DAG 下防御性，monkeypatch 测试覆盖）
    cycle = _topo_cycle(roles)
    if cycle:
        raise ComposerError('E_COMP_TF_CYCLE', '角色依赖环: %s' % '->'.join(cycle))

    # TF：child 唯一 parent、无环（22 号：整棵树无环且每个 child 只有一个 parent）
    parent_of = {}
    for entry in entries:
        manifest = _raw_manifest(store, entry)
        for edge in manifest['ownership']['tf_edges']:
            if edge['child'] in parent_of and parent_of[edge['child']] != edge['parent']:
                raise ComposerError('E_COMP_TF_DUP',
                                    'child %s 双 parent: %s 与 %s' % (edge['child'],
                                                                      parent_of[edge['child']], edge['parent']))
            parent_of[edge['child']] = edge['parent']
    tf_state = {}

    def tf_visit(node):
        tf_state[node] = 1
        parent = parent_of.get(node)
        if parent is not None:
            if tf_state.get(parent) == 1:
                return True
            if tf_state.get(parent) is None and tf_visit(parent):
                return True
        tf_state[node] = 2
        return False

    for child in sorted(parent_of):
        if tf_state.get(child) is None and tf_visit(child):
            raise ComposerError('E_COMP_TF_CYCLE', 'TF 边成环')

    # 命令话题唯一写者（22 号 §5：命令唯一所有权；建议话题与 /cmd_vel 仲裁分层）
    topic_owner = {}
    for entry in entries:
        manifest = _raw_manifest(store, entry)
        for topic in manifest['ownership']['command_topics']:
            topic_name = topic.split('（')[0]
            if topic_name in topic_owner:
                raise ComposerError('E_COMP_CMD_DUP',
                                    '%s 双写者: %s 与 %s' % (topic_name,
                                                             topic_owner[topic_name], entry['id']))
            topic_owner[topic_name] = entry['id']

    declared = sorted(e['id'] for e in entries if e['evidence']['claimed_only'])
    report = compatibility(store, entries)
    return {
        'ok': True,
        'components': sorted(component_ids),
        'roles': sorted(roles),
        'declared_only': declared,
        'tested': sorted(set(component_ids) - set(declared)),
        'command_topics': {t: topic_owner[t] for t in sorted(topic_owner)},
        'tf_children': sorted(parent_of),
        'compatibility': report,
        'adapters': [],  # 流程4 适配器槽位：明确登记的适配器与信息损失声明，后续切片填
    }


def lock(store, component_ids):
    """流程6 确定性锁文件：compose+compat 全过后生成；digest＝规范序列化的 sha256。

    declared_only 组件在锁内如实标注（22 号铁则的登记面：未验证组件不得宣称支持）。
    """
    result = compose(store, component_ids)
    component_ids = list(dict.fromkeys(component_ids))  # 与 compose 同口径去重，锁摘要不受重复选择影响
    payload = {
        'lock_version': 1,
        'components': sorted(component_ids),
        'roles': result['roles'],
        'command_topics': result['command_topics'],
        'tf_children': result['tf_children'],
        'compatibility': result['compatibility'],
        'adapters': result['adapters'],
        'manifests': {e['id']: {'version': e['version'], 'evidence': e['evidence']['level'],
                                'declared_only': e['evidence']['claimed_only'],
                                'sha256': e['sha256']}
                      for e in (store.get(cid) for cid in sorted(component_ids))},
    }
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    payload['digest'] = hashlib.sha256(canonical.encode()).hexdigest()
    return payload


def _raw_manifest(store, entry):
    """注册表条目不含 ownership/io 原文；从 load() 时随 store 缓存的原始 manifest 回读。

    缓存随 store 实例（store._composer_raws），不同 manifest 目录的 store 互不污染。
    """
    return store._composer_raws[entry['id']]


def load(manifest_dir, provenance=None):
    store = reg.Registry().load_dir(manifest_dir)
    if provenance:
        store.load_provenance(provenance)
    store._composer_raws = {}
    for entry in store.all():
        store._composer_raws[entry['id']] = json.loads(
            (Path(manifest_dir) / entry['file']).read_text())
    return store


def main(argv):
    parser = argparse.ArgumentParser(prog='composer.py', add_help=False)
    parser.add_argument('directory')
    parser.add_argument('--select', required=True)
    parser.add_argument('--provenance')
    parser.add_argument('--lock', help='组合通过后写锁文件到该路径')
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        print('usage: composer.py MANIFEST_DIR --select id1,id2 [--provenance P] [--lock OUT]',
              file=sys.stderr)
        return 9
    try:
        store = load(args.directory, args.provenance)
        ids = [part.strip() for part in args.select.split(',') if part.strip()]
        result = lock(store, ids) if args.lock else compose(store, ids)
    except (ComposerError, reg.RegistryError) as error:
        print(json.dumps({'ok': False, 'reason': error.code if isinstance(error, ComposerError)
                          else 'E_REG_INVALID', 'detail': str(error)}, ensure_ascii=False))
        return 5
    if args.lock:
        Path(args.lock).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({'ok': True, 'lock': args.lock, 'digest': result['digest'],
                          'declared_only': result['manifests'] and
                          [i for i, m in result['manifests'].items() if m['declared_only']]},
                         ensure_ascii=False))
    else:
        print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
