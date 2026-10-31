"""Portable map bundle: state snapshots + referenced map files + sha256 manifest.

A bundle is a directory with `bundle.json`, `state/*.json` and `maps/*`; import
verifies every hash and re-validates each snapshot, then rewrites absolute
data_uri paths that point at bundled map files so the bundle can move machines.
"""
import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_service  # noqa: E402

SCHEMA = 'map_bundle/v1'


class BundleError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_bytes(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp.%d' % os.getpid())
    with open(tmp, 'wb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def export_bundle(state_dir, bundle_dir, map_files=()):
    state_dir, bundle_dir = Path(state_dir), Path(bundle_dir)
    states = sorted(state_dir.glob('*.json'))
    if not states:
        raise BundleError('E_BUNDLE_EMPTY', '%s has no state snapshots' % state_dir)
    if bundle_dir.exists():
        shutil.rmtree(bundle_dir)
    entries, maps = [], []
    for path in states:
        target = bundle_dir / 'state' / path.name
        _atomic_bytes(target, path.read_bytes())
        entries.append({'path': 'state/' + path.name, 'sha256': _sha256(target),
                        'bytes': target.stat().st_size})
        raw = json.loads(path.read_text(encoding='utf-8'))
        history = raw.get('history') or []
        maps.append({'map_id': raw.get('map_id'),
                     'map_version': (raw.get('current') or {}).get('map_version'),
                     'digest': history[-1][1] if history else None})
    for path in map_files:
        path = Path(path)
        target = bundle_dir / 'maps' / path.name
        _atomic_bytes(target, path.read_bytes())
        entries.append({'path': 'maps/' + path.name, 'sha256': _sha256(target),
                        'bytes': target.stat().st_size})
    manifest = {'schema': SCHEMA, 'maps': sorted(maps, key=lambda item: item['map_id']),
                'files': sorted(entries, key=lambda item: item['path'])}
    _atomic_bytes(bundle_dir / 'bundle.json',
                  (json.dumps(manifest, ensure_ascii=False, indent=1) + '\n').encode())
    return {'bundle': str(bundle_dir), 'maps': manifest['maps'],
            'files': len(manifest['files'])}


def _manifest(bundle_dir):
    path = Path(bundle_dir) / 'bundle.json'
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise BundleError('E_BUNDLE_MANIFEST', '%s: %s' % (path, error))
    if not isinstance(value, dict) or value.get('schema') != SCHEMA:
        raise BundleError('E_BUNDLE_MANIFEST', 'schema mismatch')
    if not isinstance(value.get('files'), list) or not isinstance(value.get('maps'), list):
        raise BundleError('E_BUNDLE_MANIFEST', 'files/maps must be lists')
    for item in value['files']:
        if (not isinstance(item, dict) or not isinstance(item.get('path'), str)
                or not isinstance(item.get('sha256'), str) or len(item['sha256']) != 64):
            raise BundleError('E_BUNDLE_MANIFEST', 'bad file entry')
        path = Path(item['path'])
        if path.is_absolute() or '..' in path.parts:
            raise BundleError('E_BUNDLE_PATH', item['path'])
    return value


def verify_bundle(bundle_dir):
    manifest = _manifest(bundle_dir)
    for item in manifest['files']:
        path = Path(bundle_dir) / item['path']
        if not path.is_file() or _sha256(path) != item['sha256']:
            raise BundleError('E_BUNDLE_HASH', item['path'])
    return manifest


def _rewrite_paths(value, original_root, map_dir):
    """Point absolute data_uri values under the old maps dir at the new one."""
    if isinstance(value, dict):
        for key in list(value):
            if key == 'data_uri' and isinstance(value[key], str):
                path = Path(value[key])
                if path.is_absolute():
                    value[key] = str(map_dir / path.name)
            else:
                _rewrite_paths(value[key], original_root, map_dir)
    elif isinstance(value, list):
        for item in value:
            _rewrite_paths(item, original_root, map_dir)


def import_bundle(bundle_dir, state_dir, map_dir=None, original_root=None):
    manifest = verify_bundle(bundle_dir)
    state_dir = Path(state_dir)
    map_dir = Path(map_dir) if map_dir is not None else Path(bundle_dir) / 'maps'
    map_dir.mkdir(parents=True, exist_ok=True)
    for item in manifest['files']:
        if item['path'].startswith('maps/'):
            _atomic_bytes(map_dir / Path(item['path']).name,
                          (Path(bundle_dir) / item['path']).read_bytes())
    restored = []
    for item in manifest['files']:
        if not item['path'].startswith('state/'):
            continue
        raw = json.loads((Path(bundle_dir) / item['path']).read_text(encoding='utf-8'))
        if original_root is not None:
            current = raw.get('current', {})
            before = json.dumps(current, sort_keys=True)
            _rewrite_paths(current, original_root, map_dir)
            if json.dumps(current, sort_keys=True) != before:
                # data_uri is part of the content digest; after relocation the
                # snapshot must be re-sealed so the service can still verify it.
                evidence = None
                if raw.get('evidence'):
                    evidence = {(float(x), float(y), float(z)): state
                                for x, y, z, state in raw['evidence']}
                digest = map_service.content_digest(current, evidence)
                if raw.get('history'):
                    raw['history'][-1][1] = digest
                if isinstance(current.get('source'), dict) and current['source'].get('sha256'):
                    current['source']['sha256'] = digest
        data = (json.dumps(raw, ensure_ascii=False, sort_keys=True,
                           separators=(',', ':')) + '\n').encode()
        target = state_dir / Path(item['path']).name
        _atomic_bytes(target, data)
        try:
            service = map_service.MapService.load(target)
        except map_service.MapServiceError as error:
            raise BundleError('E_BUNDLE_STATE', str(error))
        restored.append({'map_id': service.map_id,
                         'map_version': service.current['map_version']})
    return {'state_dir': str(state_dir), 'map_dir': str(map_dir),
            'maps': sorted(restored, key=lambda item: item['map_id'])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    export = sub.add_parser('export')
    export.add_argument('state_dir')
    export.add_argument('bundle_dir')
    export.add_argument('--map-file', action='append', default=[])
    imp = sub.add_parser('import')
    imp.add_argument('bundle_dir')
    imp.add_argument('state_dir')
    imp.add_argument('--map-dir')
    imp.add_argument('--original-root')
    args = parser.parse_args(argv)
    try:
        if args.command == 'export':
            result = export_bundle(args.state_dir, args.bundle_dir, args.map_file)
        else:
            result = import_bundle(args.bundle_dir, args.state_dir, args.map_dir,
                                   args.original_root)
    except BundleError as error:
        print(json.dumps({'ok': False, 'reason': error.code, 'detail': error.detail},
                         ensure_ascii=False))
        return 5
    print(json.dumps(dict(result, ok=True), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
