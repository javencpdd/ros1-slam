"""Assembly plan and ROS1 launch skeleton rendering (no ROS import)."""
import copy
import json
import os
import re
from pathlib import Path

import execution
from xml.sax.saxutils import escape, quoteattr

PLAN_SCHEMA = 'assembly_plan/v2'
_NAME = re.compile(r'[^A-Za-z0-9_]')


REPO_ROOT = Path(__file__).resolve().parents[5]      # …/framework→v8_platform→90_platform→src→ws→repo


def _pkg_rel(value):
    """把「仓库相对、且落在某个 catkin 包内」的资产路径改写成 $(find pkg)/子路径。

    背景（2026-11-23）：graph 里的 `tree_file: ws/src/.../lite3_bt/trees/x.xml` 是**仓库相对**的，
    节点以 devel 前缀为 cwd 运行时解析不到；而 `$(find pkg)` 由 roslaunch 解析，两处都成立。
    非路径、绝对路径、找不到对应包的一律原样返回（不猜）。
    """
    if not isinstance(value, str) or value.startswith(('$(', '/')) or os.sep in value[:1]:
        return value
    if '/' not in value:
        return value
    target = (REPO_ROOT / value)
    if not target.exists():
        return value
    d = target.parent
    while d != REPO_ROOT and d != d.parent:
        if (d / 'package.xml').exists():
            return '$(find %s)/%s' % (d.name, target.relative_to(d).as_posix())
        d = d.parent
    return value


def _name(instance_id):
    return _NAME.sub('_', instance_id)


def _value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float, str)):
        return str(value)
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def nodelet_group_lines(instance_id, spec, namespace, params):
    """Manager + one `nodelet load` per plugin; no remap (nodelet ignores it)."""
    manager = spec.get('manager') or _name(instance_id) + '_manager'
    attrs = ['pkg="nodelet"', 'type="nodelet"', 'name=%s' % quoteattr(manager),
             'args="manager"']
    if namespace:
        attrs.append('ns=%s' % quoteattr(namespace))
    lines = ['  <node %s/>' % ' '.join(attrs)]
    for plugin in spec['plugins']:
        attrs = ['pkg="nodelet"', 'type="nodelet"',
                 'name=%s' % quoteattr(plugin['name']),
                 'args=%s' % quoteattr('load %s %s --no-bond'
                                       % (plugin['type'], manager))]
        if namespace:
            attrs.append('ns=%s' % quoteattr(namespace))
        lines.append('  <node %s>' % ' '.join(attrs))
        merged = dict(plugin.get('params', {}))
        merged.update(params)
        for key in sorted(merged):
            lines.append('    <param name=%s value=%s/>'
                         % (quoteattr(str(key)), quoteattr(_value(merged[key]))))
        lines.append('  </node>')
    return lines


def node_lines(instance_id, spec, namespace, params, config_path=None, remap=None):
    """Render one node/plugin element, including config, params and remaps."""
    if spec['kind'] == 'plugin':
        attrs = ['pkg=%s' % quoteattr(spec['package']),
                 'type=%s' % quoteattr('nodelet'),
                 'name=%s' % quoteattr(_name(instance_id)),
                 'args=%s' % quoteattr('standalone ' + spec['plugin'])]
    else:
        attrs = ['pkg=%s' % quoteattr(spec['package']),
                 'type=%s' % quoteattr(spec['type']),
                 'name=%s' % quoteattr(_name(instance_id))]
    if namespace:
        attrs.append('ns=%s' % quoteattr(namespace))
    lines = ['  <node %s>' % ' '.join(attrs)]
    if config_path is not None:
        lines.append('    <rosparam command="load" file=%s/>' % quoteattr(config_path))
    else:
        for param_file in spec.get('param_files', []):
            lines.append('    <rosparam command="load" file=%s/>'
                         % quoteattr('$(find %s)/%s' % (spec['package'], param_file)))
    for key in sorted(params):
        # 仓库相对资产（如 tree_file）换成 $(find pkg)/…，否则节点以 devel 前缀为 cwd 时解析不到
        lines.append('    <param name=%s value=%s/>'
                     % (quoteattr(str(key)), quoteattr(_value(_pkg_rel(params[key])))))
    for source, target in sorted(set(remap or ())):
        lines.append('    <remap from=%s to=%s/>' % (quoteattr(source), quoteattr(target)))
    lines.append('  </node>')
    return lines


def render_plan(graph, catalog, report):
    execution.check_report(graph, catalog, report)
    instances = []
    for iid in report['startup_order']:
        rec = report['instances'][iid]
        comp = catalog['components'][rec['component']]
        config = report.get('configs', {}).get(iid)
        instances.append({
            'id': iid,
            'component': rec['component'],
            'launch': copy.deepcopy(comp['launch']),
            'component_sha256': report['checked_components'][iid]['sha256'],
            'namespace': rec['namespace'],
            'params': copy.deepcopy(rec['params']),
            'config': None if config is None else config['path'],
            'config_sha256': None if config is None else config['sha256'],
            'inputs': copy.deepcopy(report['bindings'][iid]),
        })
    plan = dict({
        'schema': PLAN_SCHEMA,
        'execution_origin': report['execution_origin'],
        'name': report['name'],
        'digest': report['digest'],
        'catalog': {'path': catalog['path'], 'sha256': catalog['sha256']},
        'instances': instances,
        'edges': [{'from': e['from'], 'to': e['to'],
                   'feedback': e['feedback'], 'policy': copy.deepcopy(e['policy'])}
                  for e in report['edges']],
        'external': [{'to': e['to'], 'type': e['type'], 'source': e['source']}
                     for e in report['external']],
        'startup_order': list(report['startup_order']),
        'feedback': copy.deepcopy(report['feedback']),
        'command_writers': dict(report['command_writers']),
    }, **(execution.context(report) or {}))
    plan['execution_sha256'] = execution.plan_binding(plan)
    return plan


def render_launch(graph, catalog, report):
    from config import load_yaml
    plan = render_plan(graph, catalog, report)
    execution.check_plan(plan, catalog, load_yaml)
    executable_instances = {item['id']: item for item in plan['instances']}
    lines = ['<?xml version="1.0"?>',
             '<launch>',
             '  <!-- generated by v8 framework; graph=%s digest=%s -->'
             % (escape(plan['name']), report['digest']),
             '  <arg name="graph_digest" default=%s/>' % quoteattr(report['digest'])]
    for iid in report['startup_order']:
        rec = dict(report['instances'][iid], params=executable_instances[iid]['params'])
        spec = executable_instances[iid]['launch']
        if spec['kind'] == 'include':
            target = '$(find %s)/%s' % (spec['package'], spec['launch_file'])
            lines.append('  <include file=%s>' % quoteattr(target))
            args = dict(spec.get('args', {}))
            args.update(rec['params'])
            for key in sorted(args):
                lines.append('    <arg name=%s value=%s/>'
                             % (quoteattr(str(key)), quoteattr(_value(args[key]))))
            lines.append('  </include>')
            continue
        if spec['kind'] == 'nodelet_group':
            lines.extend(nodelet_group_lines(iid, spec, rec['namespace'], rec['params']))
            continue
        # 端口→ROS 参数名：本渲染器只认 `<port>_topic`。已核的例外（2026-11-23）：
        #   v8.fw.local_native_raw：端口 cmd      → 参数 command_topic
        #   v8.fw.local_native_raw：端口 cmd_twist → 参数 twist_topic
        # 这两处若在 graph 里被改成与 manifest 默认不同的值，remap 会按默认名生成而空转；
        # 由 tests/test_graph_wiring.py::test_known_param_name_exceptions_do_not_drift 守。
        remaps = [(item['from'], item['to']) for item in spec.get('remap', [])]
        for edge in report['edges']:
            if edge['to_instance'] != iid:
                continue
            # ★用**有效话题**而不是 manifest 默认值★：graph 会用 `<port>_topic` 覆盖参数，
            # 若 remap 仍按 manifest 默认名生成，就会把消费者重定向到一个没人发的话题
            # （2026-11-23 实测：bt.out_goal 参数为 /v8/local/goal，而 local 被 remap 到
            #  manifest 默认的 /v8/bt/goal_request ⇒ 执行目标永远收不到）。
            # ★必须按最后一个点号切★：INSTANCE_ID 允许点号（config.py:21），
            # graph._endpoint 也用 rsplit('.', 1)；用 split 会把 bt.a.out_goal 切成 a.out_goal。
            to_port = edge['to'].rsplit('.', 1)[-1]
            from_port = edge['from'].rsplit('.', 1)[-1]
            producer = executable_instances[edge['from_instance']]['params']
            target_topic = rec['params'].get(to_port + '_topic') or edge['to_port'].get('topic')
            source_topic = producer.get(from_port + '_topic') or edge['from_port'].get('topic')
            if source_topic and target_topic:
                remaps.append((target_topic, source_topic))  # 同名也保留一条：旧契约按「每边一条」断言
        config = report.get('configs', {}).get(iid)
        lines.extend(node_lines(iid, spec, rec['namespace'], rec['params'],
                                None if config is None else config['path'], remaps))
    lines.extend(['</launch>', ''])
    return '\n'.join(lines)
