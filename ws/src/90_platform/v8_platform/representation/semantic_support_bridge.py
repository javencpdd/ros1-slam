"""Semantic map -> support map bridge (semantic_map/v1 -> map.v1).

Features marked `attributes.role == "support_surface"` are interpreted as
axis-aligned slabs: the box top is the walking height, its x/y extent is the
footprint, and `attributes.edges` uses the map.v1 edge shape. The resulting map
goes through the same contract validation as any other support map.
"""
import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import semantic_map  # noqa: E402
import support_map  # noqa: E402

ROLE = 'support_surface'
SURFACE_ID = re.compile(r'^[a-z0-9_]+$')


class SemanticBridgeError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _surface(feature, map_id):
    attributes = feature.get('attributes') or {}
    geometry = feature['geometry']
    if geometry.get('type') != 'box':
        raise SemanticBridgeError('E_SEMANTIC_BRIDGE_GEOMETRY',
                                  '%s: support_surface needs box geometry' % feature['id'])
    if not SURFACE_ID.match(feature['id']):
        raise SemanticBridgeError('E_SEMANTIC_BRIDGE_ID',
                                  '%s: surface id must match [a-z0-9_]+' % feature['id'])
    low, high = geometry['min'], geometry['max']
    if high[0] <= low[0] or high[1] <= low[1] or high[2] < low[2]:
        raise SemanticBridgeError('E_SEMANTIC_BRIDGE_GEOMETRY',
                                  '%s: box has no footprint or negative thickness' % feature['id'])
    return {'surface_id': feature['id'],
            'z_reference': float(high[2]),
            'thickness_m': float(high[2] - low[2]),
            'representation': 'mesh',
            'data_uri': 'semantic://%s/%s' % (map_id, feature['id']),
            'traversability': attributes.get('traversability', 'supported'),
            'clearance_min_m': attributes.get('clearance_min_m', 0.4),
            'footprint': {'min_x': float(low[0]), 'min_y': float(low[1]),
                          'max_x': float(high[0]), 'max_y': float(high[1])},
            'edges': list(attributes.get('edges', []))}


def convert(value, resolution=0.1, source_uri=None):
    """Convert a validated semantic map into a contract-checked support map."""
    try:
        semantic_map.validate(value)
    except semantic_map.SemanticMapError as error:
        raise SemanticBridgeError(error.code, str(error))
    if not isinstance(resolution, (int, float)) or isinstance(resolution, bool) or resolution <= 0:
        raise SemanticBridgeError('E_SEMANTIC_BRIDGE_RESOLUTION', 'resolution must be positive')
    surfaces = [_surface(feature, value['map_id'])
                for feature in value['features']
                if (feature.get('attributes') or {}).get('role') == ROLE]
    if not surfaces:
        raise SemanticBridgeError('E_SEMANTIC_BRIDGE_EMPTY',
                                  '%s has no %s features' % (value['map_id'], ROLE))
    origin = [min(item['footprint']['min_x'] for item in surfaces),
              min(item['footprint']['min_y'] for item in surfaces), 0.0]
    digest = hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                       separators=(',', ':')).encode()).hexdigest()
    source = {'uri': source_uri or 'semantic://%s@%d' % (value['map_id'], value['map_version']),
              'sha256': digest}
    try:
        return support_map.build_support_map(value['map_id'], resolution, origin,
                                             surfaces, None, source,
                                             map_version=value['map_version'])
    except support_map.RepresentationError as error:
        raise SemanticBridgeError(error.code, str(error))
