"""Launch-adapter checks against a declared package index (no ROS import).

The index is a JSON snapshot (package -> launches/files/executables). It is
checked in for tests; a deployment can generate it from rospack/rospkg.
"""
import json
from pathlib import Path

from config import FrameworkError

INDEX_SCHEMA = 'package_index/v1'


def load_index(path):
    path = Path(path)
    if not path.is_file():
        raise FrameworkError('E_LAUNCH_INDEX', str(path))
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as error:
        raise FrameworkError('E_LAUNCH_INDEX', '%s: %s' % (path.name, error))
    if value.get('schema') != INDEX_SCHEMA or not isinstance(value.get('packages'), dict):
        raise FrameworkError('E_LAUNCH_INDEX', 'schema/packages missing')
    for name, entry in value['packages'].items():
        if not isinstance(entry, dict):
            raise FrameworkError('E_LAUNCH_INDEX', '%s must be an object' % name)
        for key in ('launches', 'files', 'executables'):
            items = entry.get(key, [])
            if not isinstance(items, list) or not all(isinstance(i, str) for i in items):
                raise FrameworkError('E_LAUNCH_INDEX', '%s.%s must be string list' % (name, key))
    return value['packages']


def check_components(catalog, packages):
    """Reject unknown package / launch file / executable / param file."""
    report = []
    for cid, comp in sorted(catalog['components'].items()):
        spec = comp['launch']
        package = spec['package']
        entry = packages.get(package)
        if entry is None:
            raise FrameworkError('E_LAUNCH_PACKAGE', '%s needs package %s' % (cid, package))
        if spec['kind'] == 'include':
            if spec['launch_file'] not in entry.get('launches', []):
                raise FrameworkError('E_LAUNCH_FILE',
                                     '%s: %s/%s not in index'
                                     % (cid, package, spec['launch_file']))
        elif spec['kind'] == 'plugin':
            executables = entry.get('executables')
            if executables is not None and 'nodelet' not in executables:
                raise FrameworkError('E_LAUNCH_TYPE',
                                     '%s: %s/nodelet not in index' % (cid, package))
            plugin_package = spec['plugin'].split('/')[0]
            if plugin_package not in packages:
                raise FrameworkError('E_LAUNCH_PACKAGE',
                                     '%s needs plugin package %s'
                                     % (cid, plugin_package))
        else:
            executables = entry.get('executables')
            if executables is not None and spec['type'] not in executables:
                raise FrameworkError('E_LAUNCH_TYPE',
                                     '%s: %s/%s not in index'
                                     % (cid, package, spec['type']))
        for param_file in spec.get('param_files', []):
            if param_file not in entry.get('files', []):
                raise FrameworkError('E_LAUNCH_FILE',
                                     '%s: %s/%s not in index'
                                     % (cid, package, param_file))
        report.append({'component': cid, 'kind': spec['kind'],
                       'package': package, 'test_double': spec.get('test_double', False)})
    real = sum(1 for row in report if not row['test_double'])
    return {'components': len(report), 'real': real,
            'test_double': len(report) - real, 'rows': report}
