"""Read-only map registry: map_id/version -> support_map JSON path."""
import hashlib
import json
import os
from pathlib import Path


class RegistryError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


ROOT = Path(__file__).resolve().parents[5]
DEFAULT_PATH = ROOT / 'data' / 'maps' / 'registry.json'


def load_registry(path=None):
    path = Path(path or DEFAULT_PATH)
    if not path.is_file():
        raise RegistryError('E_MAP_REGISTRY', 'registry missing: %s' % path)
    value = json.loads(path.read_text(encoding='utf-8'))
    if value.get('schema') != 'map_registry/v1' or not isinstance(value.get('maps'), list):
        raise RegistryError('E_MAP_REGISTRY_SCHEMA', str(path))
    ids = set()
    for entry in value['maps']:
        for key in ('map_id', 'version', 'path', 'sha256'):
            if key not in entry:
                raise RegistryError('E_MAP_REGISTRY_ENTRY', '%s missing %s' % (entry, key))
        key = (entry['map_id'], int(entry['version']))
        if key in ids:
            raise RegistryError('E_MAP_REGISTRY_DUP', '%s@%s' % key)
        ids.add(key)
    return value


def resolve_entry(registry, map_id, version=None):
    matches = [entry for entry in registry['maps'] if entry['map_id'] == map_id]
    if version is not None:
        matches = [entry for entry in matches if int(entry['version']) == int(version)]
    if not matches:
        raise RegistryError('E_MAP_NOT_FOUND', '%s@%s' % (map_id, version))
    return sorted(matches, key=lambda item: int(item['version']))[-1]


def load_map(entry, root=None):
    root = Path(root or ROOT)
    path = Path(entry['path'])
    sim_root = os.environ.get('V8_SIM_ROOT')
    if sim_root and path.parts[:1] == ('sim_ws',):
        path = Path(sim_root) / 'ws' / Path(*path.parts[1:])
    else:
        path = root / path
    path = path.resolve()
    if not path.is_file():
        raise RegistryError('E_MAP_FILE', str(path))
    value = json.loads(path.read_text(encoding='utf-8'))
    if value.get('map_id') != entry['map_id'] or int(value.get('map_version')) != int(entry['version']):
        raise RegistryError('E_MAP_ID_VERSION',
                            'registry %s@%s != file %s@%s'
                            % (entry['map_id'], entry['version'],
                               value.get('map_id'), value.get('map_version')))
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if entry.get('sha256') and sha != entry['sha256']:
        raise RegistryError('E_MAP_SHA', '%s sha256 mismatch' % entry['map_id'])
    return value, path, sha


def get_map(map_id, version=None, registry_path=None, root=None):
    registry = load_registry(registry_path)
    entry = resolve_entry(registry, map_id, version)
    value, path, sha = load_map(entry, root)
    return dict(entry), value, path, sha
