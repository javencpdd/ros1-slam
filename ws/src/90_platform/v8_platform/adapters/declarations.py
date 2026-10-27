"""V8-07 次切片：适配器信息损失声明（22 号 流程4）＋组合样例矩阵（R648）。

流程4 原文：「选择明确登记的适配器；显示其信息损失与有效域，不擅自插入有损转换。」
本件把各适配器的**已证损失**显式声明（证据指针回指 V8-03 探针与 R616 历史判读），
并枚举合法组合样例矩阵（定位×控制×全局全链过筛：compose＋wiring＋lock）。
估计与真值分开（A6）：损失声明只描述适配面，不混入真值通道语义。
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'composer'))
sys.path.insert(0, str(HERE.parent / 'registry'))
import composer as comp  # noqa: E402
import io_match  # noqa: E402

# 逐适配器损失声明（kind/description/domain/evidence 四要素齐；空列表＝接口同形无损）。
# 证据指针只引已证事实：V8-03 探针（probe_mode3/R633—R635）与 R616 历史判读。
ADAPTER_LOSSES = {
    'v8.global_planner.apt': [
        {'kind': 'dimension_loss',
         'description': 'NavFn 纯 2D：z／楼层信息丢弃，多楼层结构上无解',
         'domain': '单层平面导航',
         'evidence': 'R616 历史判读（实栈 /planners=[GlobalPlanner]）'},
    ],
    'v8.representation.costmap_layers': [
        {'kind': 'aggregation_loss',
         'description': '多高度列／层间信息聚合压缩为单一代价层',
         'domain': '平面代价决策（多层表征归 V8-08）',
         'evidence': '22 号 §1 representation 职责表'},
    ],
    'v8.local_planner.scan_planner': [
        {'kind': 'transition_loss',
         'description': '0.5 m 3D 抽稀吞掉短于 0.5 m 的转层（离线弦悬空 0.2 m；运行时坡脚悬空 0.298 m）',
         'domain': '转场 3D 段长 ≥0.5 m',
         'evidence': 'probe_mode3 B2／R633 烟测实测'},
        {'kind': 'profile_loss',
         'description': 'A* 搜索平面只表达单调高度剖面；同高天桥／同 XY 双层列不可表达，优化器兜底会掩盖失败',
         'domain': '单调段内（支撑图切段为主路线）',
         'evidence': 'probe_mode3 C2/C3／R634—R635 运行时实证'},
    ],
    'v8.controller.dwa_adapter': [
        {'kind': 'dimension_loss',
         'description': '3D 轨迹→平面速度指令，z 不受控',
         'domain': '平面运动',
         'evidence': 'DWAPlannerROS 语义（R616 配置面十六件引用）'},
    ],
    'v8.controller.teb_adapter': [
        {'kind': 'dimension_loss',
         'description': '3D 轨迹→平面速度指令，z 不受控；TEB 2D 刚体假设（台阶前沿被当可绕障碍）',
         'domain': '平面运动（台阶场景失守，历史「优化三」注记）',
         'evidence': '历史证据面「TEB 2D 刚体假设／台阶两处失守」'},
    ],
    'v8.odometry.faster_lio': [],
    'v8.odometry.lite3_slam': [],
    'v8.global_localization.hdl': [],
    'v8.global_localization.lite3': [],
    'v8.map_server.apt': [],
    'v8.safety_arbiter.lite3': [],
    'v8.local_planner.lite3_scan': [],
    'v8.simulator.lite3_gazebo': [],
}


def losses(store, component_ids):
    """选中组合的逐适配器损失声明；未声明组件＝E_ADAPTER_UNDECLARED（不许静默无声明）。"""
    report = []
    for component_id in component_ids:
        entry = store.get(component_id)
        if entry is None:
            raise io_match.AdapterError('E_ADAPTER_UNKNOWN_ID', component_id)
        if component_id not in ADAPTER_LOSSES:
            raise io_match.AdapterError('E_ADAPTER_UNDECLARED',
                                        '%s 无损失声明（新适配器必须显式登记，空列表＝声明无损）' % component_id)
        for record in ADAPTER_LOSSES[component_id]:
            for field in ('kind', 'description', 'domain', 'evidence'):
                if not record.get(field, '').strip():
                    raise io_match.AdapterError('E_ADAPTER_UNDECLARED',
                                                '%s 损失声明缺 %s' % (component_id, field))
        report.append({'component': component_id, 'losses': ADAPTER_LOSSES[component_id]})
    return report


# 组合样例矩阵：八件套基底上，定位 {hdl, lite3} × 控制 {dwa, teb} × 全局 {GlobalPlanner.apt}
BASE = ['v8.odometry.faster_lio', 'v8.global_localization.lite3', 'v8.map_server.apt',
        'v8.representation.costmap_layers', 'v8.local_planner.scan_planner',
        'v8.global_planner.apt', 'v8.controller.dwa_adapter', 'v8.safety_arbiter.lite3']
AXIS_LOCALIZATION = ('v8.global_localization.lite3', 'v8.global_localization.hdl')
AXIS_CONTROLLER = ('v8.controller.dwa_adapter', 'v8.controller.teb_adapter')


def matrix(store):
    """合法组合枚举＋全链过筛（compose＋wiring＋losses＋lock）；负例（双控制器）记拒绝。"""
    combos = []
    for localization in AXIS_LOCALIZATION:
        for controller in AXIS_CONTROLLER:
            ids = [localization if cid == AXIS_LOCALIZATION[0]
                   and cid in BASE else cid for cid in BASE]
            ids = [controller if cid == AXIS_CONTROLLER[0] else cid for cid in ids]
            try:
                result = comp.compose(store, ids)
                wires = io_match.wiring(store, ids)
                declared = losses(store, ids)
                digest = comp.lock(store, ids)['digest']
                combos.append({'ok': True, 'ids': sorted(ids), 'digest': digest,
                               'roles': result['roles'], 'wires': len(wires),
                               'loss_records': sum(len(d['losses']) for d in declared)})
            except (comp.ComposerError, io_match.AdapterError) as error:
                combos.append({'ok': False, 'ids': sorted(ids),
                               'reason': getattr(error, 'code', 'ERROR')})
    negative = list(BASE) + [AXIS_CONTROLLER[1]]  # 双控制器＝天然负例
    try:
        comp.compose(store, negative)
        negative_result = {'ok': True, 'unexpected': True}
    except comp.ComposerError as error:
        negative_result = {'ok': False, 'reason': error.code}
    return {'combos': combos, 'negative_dual_controller': negative_result}


def main(argv):
    if len(argv) != 2 or argv[0] != 'matrix':
        print('usage: declarations.py matrix MANIFEST_DIR', file=sys.stderr)
        return 9
    try:
        store = comp.load(argv[1])
        result = matrix(store)
    except (comp.ComposerError, io_match.AdapterError) as error:
        print(json.dumps({'ok': False, 'reason': getattr(error, 'code', 'E_REG_INVALID'),
                          'detail': str(error)}, ensure_ascii=False))
        return 5
    ok_count = sum(1 for c in result['combos'] if c['ok'])
    print(json.dumps({'ok': True, 'matrix': result, 'combo_count': len(result['combos']),
                      'ok_count': ok_count}, ensure_ascii=False, indent=1))
    return 0 if ok_count == len(result['combos']) else 5


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
