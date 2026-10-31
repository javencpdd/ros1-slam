"""Semantic map -> planner obstacle list (semantic_map/v1 boxes)."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_map  # noqa: E402

ROLE = 'obstacle'


class SemanticObstacleError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _vec(value, where):
    if (not isinstance(value, list) or len(value) != 3
            or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                       for item in value)):
        raise SemanticObstacleError('E_SEMANTIC_OBSTACLE_VECTOR', where)
    return [float(item) for item in value]


def convert(value):
    """Return planner obstacle dicts for box features marked role=obstacle."""
    try:
        semantic_map.validate(value)
    except semantic_map.SemanticMapError as error:
        raise SemanticObstacleError(error.code, str(error))
    obstacles = []
    for feature in value['features']:
        attributes = feature.get('attributes') or {}
        if attributes.get('role') != ROLE:
            continue
        geometry = feature['geometry']
        if geometry.get('type') != 'box':
            raise SemanticObstacleError('E_SEMANTIC_OBSTACLE_GEOMETRY',
                                        '%s: obstacle needs box geometry' % feature['id'])
        low, high = _vec(geometry['min'], feature['id'] + '.min'), \
            _vec(geometry['max'], feature['id'] + '.max')
        if any(low[i] >= high[i] for i in range(3)):
            raise SemanticObstacleError('E_SEMANTIC_OBSTACLE_GEOMETRY',
                                        '%s: empty box' % feature['id'])
        item = {'id': feature['id'], 'min': low, 'max': high}
        if 'velocity' in attributes:
            item['velocity'] = _vec(attributes['velocity'], feature['id'] + '.velocity')
        for key in ('stamp', 'active_from', 'active_until'):
            if key in attributes:
                if not isinstance(attributes[key], (int, float)) or isinstance(attributes[key], bool):
                    raise SemanticObstacleError('E_SEMANTIC_OBSTACLE_TIME',
                                                feature['id'] + '.' + key)
                item[key] = float(attributes[key])
        obstacles.append(item)
    return obstacles
