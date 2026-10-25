"""V8-07 适配器契约面：消息级 io 匹配（V8-05 流程3 的留白在此闭合，R647）。

规则：
- manifest 的 io.inputs/outputs 是「消息类型（注释）」形字符串；取注释前的消息类型为匹配键。
- 输入满足＝组合内有选中组件产出同消息类型；或属外部输入白名单：传感器类（sensor_msgs/*，
  驱动/仿真供给）、任务级输入（/initial_path 等）、平台基础设施（liveness/health 心跳）、
  静态文件（地图文件）。白名单是显式枚举，不静默吞缺口。
- 不满足＝E_ADAPTER_NO_SOURCE（fail-closed，接线报告不静默留空）。
- 产出＝接线报告（consumer.input → producer），填 composer 结果的 adapters 槽位（流程4 前置）。
- 估计与真值分开（A6 面）：适配器不混入真值通道，本层只登记接线，不复制数据语义。
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'registry'))
sys.path.insert(0, str(HERE.parent / 'composer'))
import composer as comp  # noqa: E402  消费组合层与注册表

SENSOR_PREFIX = 'sensor_msgs/'
EXTERNAL_MARKS = ('/initial_path', '任务级', '（任务', 'liveness', 'health', '静态地图文件', '地图文件')


class AdapterError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def msg_type(io_string):
    """『nav_msgs/Path（全局路径）』→ ('nav_msgs/Path', '全局路径')。"""
    text = io_string.split('（', 1)
    return text[0].strip(), (text[1].rstrip('）') if len(text) > 1 else '')


def is_external(io_string):
    """传感器类（驱动/仿真供给）与任务级输入不算组合内缺口。"""
    kind, note = msg_type(io_string)
    return kind.startswith(SENSOR_PREFIX) or any(mark in io_string for mark in EXTERNAL_MARKS)


def wiring(store, component_ids):
    """接线报告：每个非外部输入找到生产者；缺源＝E_ADAPTER_NO_SOURCE。"""
    entries = [store.get(cid) for cid in component_ids]
    if any(e is None for e in entries):
        missing = [cid for cid, e in zip(component_ids, entries) if e is None]
        raise AdapterError('E_ADAPTER_UNKNOWN_ID', ','.join(missing))
    producers = {}
    for entry in entries:
        raw = store._composer_raws[entry['id']]
        for output in raw['io']['outputs']:
            kind, _ = msg_type(output)
            producers.setdefault(kind, []).append(entry['id'])
    report = []
    for entry in entries:
        raw = store._composer_raws[entry['id']]
        for item in raw['io']['inputs']:
            kind, note = msg_type(item)
            record = {'component': entry['id'], 'input': kind}
            if kind in producers:
                record['providers'] = sorted(set(producers[kind]) - {entry['id']})
                if not record['providers']:
                    record['providers'] = sorted(producers[kind])  # 自产自销如实登记
            elif is_external(item):
                record['external'] = 'sensor' if kind.startswith(SENSOR_PREFIX) else 'task'
            else:
                raise AdapterError('E_ADAPTER_NO_SOURCE',
                                   '%s 需 %s，组合内无生产者且非外部输入' % (entry['id'], kind))
            report.append(record)
    return report


def main(argv):
    if len(argv) != 3 or argv[0] not in ('wiring',):
        print('usage: io_match.py wiring MANIFEST_DIR --select=id1,id2', file=sys.stderr)
        return 9
    if not argv[2].startswith('--select='):
        return 9
    try:
        store = comp.load(argv[1])
        result = wiring(store, [p.strip() for p in argv[2][9:].split(',') if p.strip()])
    except (AdapterError, comp.ComposerError) as error:
        code = error.code if isinstance(error, AdapterError) else 'E_COMP_' + getattr(error, 'code', 'X')
        print(json.dumps({'ok': False, 'reason': getattr(error, 'code', 'E_REG_INVALID'),
                          'detail': str(error)}, ensure_ascii=False))
        return 5
    print(json.dumps({'ok': True, 'wires': result}, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
