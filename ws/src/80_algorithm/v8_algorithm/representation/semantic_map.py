"""Semantic map extension (semantic_map/v1); independent of frozen map.v1/A1."""
import json
import math
import re
from pathlib import Path

SCHEMA = 'semantic_map/v1'
MAP_ID = re.compile(r'^[a-z0-9_]+$')
KINDS = ('point', 'line', 'polygon', 'box')


class SemanticMapError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _vec(value, where):
    if (not isinstance(value, list) or len(value) != 3
            or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                       and math.isfinite(float(item)) for item in value)):
        raise SemanticMapError('E_SEMANTIC_GEOMETRY', where)
    return [float(item) for item in value]


def validate(value):
    if not isinstance(value, dict) or value.get('schema') != SCHEMA:
        raise SemanticMapError('E_SEMANTIC_SCHEMA', 'schema must be %s' % SCHEMA)
    if not isinstance(value.get('map_id'), str) or not MAP_ID.match(value['map_id']):
        raise SemanticMapError('E_SEMANTIC_MAP_ID', 'map_id pattern')
    if (not isinstance(value.get('map_version'), int)
            or isinstance(value.get('map_version'), bool) or value['map_version'] < 1):
        raise SemanticMapError('E_SEMANTIC_VERSION', 'map_version must be >=1')
    if not isinstance(value.get('frame'), str) or not value['frame']:
        raise SemanticMapError('E_SEMANTIC_FRAME', 'frame missing')
    features = value.get('features')
    if not isinstance(features, list):
        raise SemanticMapError('E_SEMANTIC_FEATURES', 'features must be a list')
    seen = set()
    for index, feature in enumerate(features):
        where = 'features[%d]' % index
        if not isinstance(feature, dict):
            raise SemanticMapError('E_SEMANTIC_FEATURE', where)
        feature_id = feature.get('id')
        if not isinstance(feature_id, str) or not feature_id or feature_id in seen:
            raise SemanticMapError('E_SEMANTIC_ID', where + '.id')
        seen.add(feature_id)
        if not isinstance(feature.get('kind'), str) or not feature['kind']:
            raise SemanticMapError('E_SEMANTIC_KIND', where + '.kind')
        geometry = feature.get('geometry')
        if not isinstance(geometry, dict) or geometry.get('type') not in KINDS:
            raise SemanticMapError('E_SEMANTIC_GEOMETRY', where + '.geometry')
        kind = geometry['type']
        if kind == 'point':
            _vec(geometry.get('point'), where + '.point')
        elif kind in ('line', 'polygon'):
            points = geometry.get('points')
            minimum = 2 if kind == 'line' else 3
            if not isinstance(points, list) or len(points) < minimum:
                raise SemanticMapError('E_SEMANTIC_GEOMETRY', where + '.points')
            for point in points:
                _vec(point, where + '.points')
        else:
            low = _vec(geometry.get('min'), where + '.min')
            high = _vec(geometry.get('max'), where + '.max')
            if any(a > b for a, b in zip(low, high)):
                raise SemanticMapError('E_SEMANTIC_GEOMETRY', where + '.box')
        if 'attributes' in feature and not isinstance(feature['attributes'], dict):
            raise SemanticMapError('E_SEMANTIC_ATTRIBUTES', where + '.attributes')
    return value


def load(path):
    path = Path(path)
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise SemanticMapError('E_SEMANTIC_JSON', '%s: %s' % (path, error))
    return validate(value)


def summary(value):
    counts = {}
    for feature in value['features']:
        counts[feature['kind']] = counts.get(feature['kind'], 0) + 1
    return {'map_id': value['map_id'], 'map_version': value['map_version'],
            'frame': value['frame'], 'features': len(value['features']),
            'kinds': dict(sorted(counts.items()))}


class SemanticCatalog:
    """Read-only collection of semantic maps keyed by map_id."""

    def __init__(self, maps=()):
        self.maps = {}
        for value in maps:
            self.add(value)

    def add(self, value):
        validate(value)
        if value['map_id'] in self.maps:
            raise SemanticMapError('E_SEMANTIC_DUPLICATE', value['map_id'])
        self.maps[value['map_id']] = value

    def ids(self):
        return sorted(self.maps)

    def get(self, map_id):
        if map_id not in self.maps:
            raise SemanticMapError('E_SEMANTIC_MAP_ID', 'unknown %s' % map_id)
        return self.maps[map_id]

    def load_files(self, paths):
        for path in paths:
            self.add(load(path))
        return self

    def status(self):
        return {map_id: summary(value) for map_id, value in sorted(self.maps.items())}
