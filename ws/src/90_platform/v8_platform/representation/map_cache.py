"""Map cache and merge core over map_delta/v1.

No ROS import and no v8_msgs dependency: caches buffer surface operations for a
known base version, and the merger combines deltas from several caches before
the MapService applies one atomic delta. The ROS message layer remains a test
double until v8_msgs exists.
"""
import copy
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402


class MapCacheError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, ensure_ascii=False,
                   separators=(',', ':')).encode()).hexdigest()


class MapCache:
    def __init__(self, map_id, base_version):
        if not isinstance(map_id, str) or not map_id:
            raise MapCacheError('E_CACHE_MAP_ID', 'map_id required')
        if not isinstance(base_version, int) or isinstance(base_version, bool) \
                or base_version < 1:
            raise MapCacheError('E_CACHE_BASE', 'base_version must be >=1')
        self.map_id = map_id
        self.base_version = base_version
        self.ops = {}

    def upsert(self, surface):
        if not isinstance(surface, dict) or not isinstance(surface.get('surface_id'), str) \
                or not surface['surface_id']:
            raise MapCacheError('E_CACHE_SURFACE', 'surface_id required')
        surface_id = surface['surface_id']
        self.ops.pop(surface_id, None)
        self.ops[surface_id] = {'op': 'upsert_surface',
                                'surface': copy.deepcopy(surface)}
        return self

    def remove(self, surface_id):
        if not isinstance(surface_id, str) or not surface_id:
            raise MapCacheError('E_CACHE_SURFACE', 'surface_id required')
        self.ops.pop(surface_id, None)
        self.ops[surface_id] = {'op': 'remove_surface', 'surface_id': surface_id}
        return self

    def pending(self):
        return [copy.deepcopy(op) for op in self.ops.values()]

    def make_delta(self, delta_id, source_uri='cache://local'):
        if not self.ops:
            raise MapCacheError('E_CACHE_EMPTY', 'no pending operations')
        ops = self.pending()
        delta = {'schema_version': 1, 'map_id': self.map_id,
                 'base_version': self.base_version, 'delta_id': delta_id,
                 'source': {'uri': source_uri, 'sha256': _digest(ops)},
                 'ops': ops}
        try:
            contract.validate('map_delta', delta)
        except contract.ContractError as error:
            raise MapCacheError('E_CACHE_DELTA', '%s: %s'
                                % (error.code, error.detail))
        return delta

    def ack(self, version):
        if not isinstance(version, int) or version != self.base_version + 1:
            raise MapCacheError('E_CACHE_ACK',
                                'expected %s, got %r'
                                % (self.base_version + 1, version))
        self.base_version = version
        self.ops = {}
        return self.base_version


def merge_deltas(deltas, delta_id=None):
    """Combine compatible deltas; later operations win per surface_id."""
    if not isinstance(deltas, list) or not deltas:
        raise MapCacheError('E_MERGE_EMPTY', 'at least one delta required')
    first = deltas[0]
    merged = {}
    for delta in deltas:
        try:
            contract.validate('map_delta', delta)
        except contract.ContractError as error:
            raise MapCacheError('E_MERGE_DELTA', '%s: %s'
                                % (error.code, error.detail))
        if delta['map_id'] != first['map_id']:
            raise MapCacheError('E_MERGE_MAP_ID', delta['map_id'])
        if delta['base_version'] != first['base_version']:
            raise MapCacheError('E_MERGE_BASE', '%s vs %s'
                                % (delta['base_version'], first['base_version']))
        for op in delta['ops']:
            if op['op'] == 'upsert_surface':
                surface_id = op['surface']['surface_id']
            else:
                surface_id = op['surface_id']
            merged.pop(surface_id, None)
            merged[surface_id] = copy.deepcopy(op)
    ops = list(merged.values())
    result = {'schema_version': 1, 'map_id': first['map_id'],
              'base_version': first['base_version'],
              'delta_id': delta_id or 'merge-' + _digest(ops)[:16],
              'source': {'uri': 'merge://' + ','.join(
                  delta['delta_id'] for delta in deltas),
                  'sha256': _digest(ops)},
              'ops': ops}
    try:
        contract.validate('map_delta', result)
    except contract.ContractError as error:
        raise MapCacheError('E_MERGE_DELTA', '%s: %s'
                            % (error.code, error.detail))
    return result
