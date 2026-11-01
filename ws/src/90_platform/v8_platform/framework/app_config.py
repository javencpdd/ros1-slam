"""Navigation app config: graph + registry map + navigation_task + algorithm configs."""
import json
from pathlib import Path

import yaml

import map_registry
import surface_resolver

ROOT = Path(__file__).resolve().parents[5]
APP_SCHEMA = 'navigation_app/v1'
LOCALIZATION_SOURCES = ('carrier', 'sim_gt', 'faster_lio', 'ndt')
CARRIER_BACKENDS = ('gazebo', 'kinematic')


class AppError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _path(value, root):
    path = Path(value)
    return path if path.is_absolute() else (root / path)


def load_app(path, root=None):
    root = Path(root or ROOT)
    path = Path(path)
    if not path.is_file():
        raise AppError('E_APP_MISSING', str(path))
    app = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(app, dict) or app.get('schema') != APP_SCHEMA:
        raise AppError('E_APP_SCHEMA', str(path))
    for key in ('name', 'graph', 'map', 'task', 'localization', 'carrier'):
        if key not in app:
            raise AppError('E_APP_FIELD', 'missing ' + key)
    source = app['localization'].get('source')
    if source not in LOCALIZATION_SOURCES:
        raise AppError('E_APP_LOCALIZATION', repr(source))
    backend = app['carrier'].get('backend')
    if backend not in CARRIER_BACKENDS:
        raise AppError('E_APP_CARRIER', repr(backend))
    map_spec = app['map']
    entry, map_value, map_path, map_sha = map_registry.get_map(
        map_spec.get('id') or map_spec.get('map_id'), map_spec.get('version'),
        registry_path=_path(app['map_registry'], root) if app.get('map_registry') else None,
        root=root)
    task_path = _path(app['task'], root)
    task = json.loads(task_path.read_text(encoding='utf-8'))
    task_map = task.get('map') or {}
    if task_map.get('map_id') != entry['map_id'] or int(task_map.get('map_version')) != int(entry['version']):
        raise AppError('E_APP_TASK_MAP', 'task map does not match app map')
    resolved_task, meta = surface_resolver.resolve_task(task, map_value)
    graph_path = _path(app['graph'], root)
    if not graph_path.is_file():
        raise AppError('E_APP_GRAPH', str(graph_path))
    loc_graph = app['localization'].get('graph')
    loc_graph_path = _path(loc_graph, root) if loc_graph else None
    if source != 'carrier' and loc_graph_path is not None and not loc_graph_path.is_file():
        raise AppError('E_APP_LOC_GRAPH', str(loc_graph_path))
    return {
        'app': app, 'app_path': path, 'root': root,
        'map_entry': entry, 'map_value': map_value, 'map_path': map_path, 'map_sha256': map_sha,
        'task': task, 'task_resolved': resolved_task, 'task_meta': meta, 'task_path': task_path,
        'graph_path': graph_path if source == 'carrier' or loc_graph_path is None else loc_graph_path,
        'carrier_graph_path': graph_path, 'localization_graph_path': loc_graph_path,
    }
