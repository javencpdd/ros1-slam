#!/usr/bin/env python3
"""V8-X3 v0.1 graphical selector: one HTML page per graph, variants prechecked.

The page never becomes a second validator: each single-instance replacement is
checked here with graph.check + safety.check_plan, and the selected combination
is exported as JSON (YAML-compatible) for the normal CLI check.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import safety  # noqa: E402


def _signature(component):
    return tuple(sorted(component['roles']))


def _trial(graph, catalog, instance_id, component_id):
    trial = copy.deepcopy(graph)
    trial['instances'][instance_id]['component'] = component_id
    try:
        report = graph_mod.check(trial, catalog)
        plan = render.render_plan(trial, catalog, report)
        checked = safety.check_plan(plan, catalog)
    except config.FrameworkError as error:
        return {'component': component_id, 'ok': False,
                'reason': error.code, 'detail': error.detail}
    return {'component': component_id, 'ok': True, 'reason': None, 'detail': None,
            'digest': report['digest'],
            'command_writers': report['command_writers'],
            'safety': checked}


def variant_matrix(graph, catalog):
    instances = []
    for item in graph['instances'].values():
        current = catalog['components'][item['component']]
        candidates = sorted(cid for cid, component in catalog['components'].items()
                            if _signature(component) == _signature(current)
                            and cid != item['component'])
        variants = [_trial(graph, catalog, item['id'], cid) for cid in candidates]
        variants.insert(0, _trial(graph, catalog, item['id'], item['component']))
        instances.append({'id': item['id'], 'current': item['component'],
                          'roles': list(current['roles']),
                          'namespace': item.get('namespace'),
                          'params': item.get('params', {}),
                          'variants': variants})
    report = graph_mod.check(graph, catalog)
    return {'schema': 'selector/v1', 'graph': graph['name'], 'digest': report['digest'],
            'catalog': catalog['path'], 'instances': instances,
            'edges': [{'from': e['from'], 'to': e['to'], 'feedback': e['feedback'],
                       'policy': e['policy']} for e in report['edges']],
            'external': [{'to': e['to'], 'type': e['type'], 'source': e['source']}
                         for e in report['external']]}


def selected_graph(graph, choices):
    result = copy.deepcopy(graph)
    for instance_id, component_id in choices.items():
        if instance_id in result['instances']:
            result['instances'][instance_id]['component'] = component_id
    return result


def _html(data, manifest_dir, graph_path):
    payload = json.dumps(data, ensure_ascii=False).replace('</', '<\\/')
    return '''<!doctype html>
<html lang="zh"><head><meta charset="utf-8"><title>V8 组件选配 · %(name)s</title>
<style>
:root{color-scheme:light dark;font-family:system-ui,sans-serif}
body{margin:24px;max-width:1100px;background:#101418;color:#e6edf3}
h1{font-size:20px} .muted{color:#8b949e}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:14px}
.card{border:1px solid #30363d;border-radius:10px;padding:14px;background:#161b22}
.card.bad{border-color:#f85149}.card.ok{border-color:#3fb950}
.card h2{font-size:15px;margin:0 0 6px}
select{width:100%%;padding:6px;margin:6px 0;background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:6px}
.status{font-size:12px;min-height:32px}.bad .status{color:#f85149}.ok .status{color:#3fb950}
.edge{font-family:ui-monospace,monospace;font-size:12px;color:#8b949e}
button{padding:8px 12px;border-radius:6px;border:1px solid #30363d;background:#238636;color:white;cursor:pointer}
textarea{width:100%%;height:220px;background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:6px;font-family:ui-monospace,monospace;font-size:12px}
</style></head><body>
<h1>V8 组件选配 · %(name)s</h1>
<p class="muted">单实例替换已由 graph.check + safety.check_plan 预检；组合结果导出后用 CLI 复核。
当前基线 digest: <code>%(digest)s</code>。manifest: <code>%(manifest)s</code></p>
<div class="grid" id="cards"></div>
<h2 style="font-size:15px;margin-top:22px">连线</h2><div id="edges"></div>
<p><button id="export">导出选配图（JSON，可被 YAML 读取）</button>
<span class="muted"> 保存为 graph_selected.json 后运行：
<code>python3 -B framework/cli.py check %(manifest)s graph_selected.json</code></span></p>
<textarea id="out" readonly></textarea>
<script id="data" type="application/json">%(payload)s</script>
<script>
const DATA = JSON.parse(document.getElementById('data').textContent);
const choices = {};
for (const inst of DATA.instances) choices[inst.id] = inst.current;
function render() {
  const cards = document.getElementById('cards'); cards.innerHTML = '';
  for (const inst of DATA.instances) {
    const card = document.createElement('div');
    const variant = inst.variants.find(v => v.component === choices[inst.id]) || inst.variants[0];
    card.className = 'card ' + (variant.ok ? 'ok' : 'bad');
    const title = document.createElement('h2'); title.textContent = inst.id + ' · ' + inst.roles.join('/');
    const select = document.createElement('select');
    for (const v of inst.variants) {
      const option = document.createElement('option');
      option.value = v.component; option.textContent = v.component;
      select.appendChild(option);
    }
    select.value = choices[inst.id];
    select.onchange = () => { choices[inst.id] = select.value; render(); };
    const status = document.createElement('div'); status.className = 'status';
    status.textContent = variant.ok ? '单实例替换可校验（digest ' + variant.digest.slice(0, 12) + '…）'
      : '拒绝：' + variant.reason + ' — ' + variant.detail;
    card.append(title, select, status); cards.appendChild(card);
  }
  const edges = document.getElementById('edges'); edges.innerHTML = '';
  for (const e of DATA.edges) {
    const row = document.createElement('div'); row.className = 'edge';
    row.textContent = (e.feedback ? '[反馈] ' : '') + e.from + ' → ' + e.to +
      (e.policy && Object.keys(e.policy).length ? '  ' + JSON.stringify(e.policy) : '');
    edges.appendChild(row);
  }
}
document.getElementById('export').onclick = () => {
  const out = {schema: 'pipeline_graph/v1', name: DATA.graph,
    instances: DATA.instances.map(i => ({id: i.id, component: choices[i.id]})),
    edges: DATA.edges, external: DATA.external};
  const text = JSON.stringify(out, null, 2);
  document.getElementById('out').value = text;
  const blob = new Blob([text], {type: 'application/json'});
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob); link.download = 'graph_selected.json';
  link.click(); URL.revokeObjectURL(link.href);
};
render();
</script></body></html>
''' % {'name': data['graph'], 'digest': data['digest'], 'manifest': manifest_dir,
       'payload': payload}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests')
    parser.add_argument('graph')
    parser.add_argument('--out-html', required=True)
    parser.add_argument('--out-json')
    args = parser.parse_args(argv)
    catalog = config.load_components(args.manifests)
    graph = config.load_graph(args.graph)
    data = variant_matrix(graph, catalog)
    Path(args.out_html).write_text(_html(data, args.manifests, args.graph), encoding='utf-8')
    if args.out_json:
        Path(args.out_json).write_text(json.dumps(data, ensure_ascii=False, indent=1) + '\n',
                                       encoding='utf-8')
    rejected = sum(1 for inst in data['instances'] for v in inst['variants'] if not v['ok'])
    print(json.dumps({'ok': True, 'html': args.out_html, 'instances': len(data['instances']),
                      'variant_rejections': rejected}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
