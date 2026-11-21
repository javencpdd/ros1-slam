"""Conversions between map_delta/v1 JSON and v8_msgs ROS messages."""
import v8_msgs.msg as msgs


def surface_msg(value):
    message = msgs.SupportSurface()
    message.surface_id = value['surface_id']
    message.z_reference = float(value.get('z_reference', 0.0))
    message.thickness_m = float(value.get('thickness_m', 0.0))
    message.representation = value.get('representation', '')
    message.data_uri = value.get('data_uri', '')
    message.traversability = value.get('traversability', '')
    message.clearance_min_m = float(value.get('clearance_min_m', 0.0))
    footprint = value.get('footprint', {})
    message.min_x = float(footprint.get('min_x', 0.0))
    message.min_y = float(footprint.get('min_y', 0.0))
    message.max_x = float(footprint.get('max_x', 0.0))
    message.max_y = float(footprint.get('max_y', 0.0))
    for edge in value.get('edges', []):
        item = msgs.SurfaceEdge()
        item.to = edge['to']
        item.kind = edge.get('kind', '')
        item.max_slope_deg = float(edge.get('max_slope_deg', 0.0))
        item.bidirectional = bool(edge.get('bidirectional', False))
        message.edges.append(item)
    return message


def surface_value(message):
    value = {'surface_id': message.surface_id,
             'z_reference': message.z_reference,
             'thickness_m': message.thickness_m,
             'representation': message.representation,
             'data_uri': message.data_uri,
             'traversability': message.traversability,
             'clearance_min_m': message.clearance_min_m,
             'footprint': {'min_x': message.min_x, 'min_y': message.min_y,
                           'max_x': message.max_x, 'max_y': message.max_y},
             'edges': [{'to': edge.to, 'kind': edge.kind,
                        'max_slope_deg': edge.max_slope_deg,
                        'bidirectional': edge.bidirectional}
                       for edge in message.edges]}
    return value


def support_map_msg(value):
    message = msgs.SupportMap()
    message.schema = value.get('schema', 'support_map/v1')
    message.map_version = int(value.get('map_version', 1))
    message.frame = value.get('frame', 'map')
    message.kind = value.get('kind', 'support_map')
    message.resolution = float(value.get('resolution', 0.0))
    origin = value.get('origin', [0.0, 0.0, 0.0])
    message.origin = [float(item) for item in origin[:3]]
    extent = value.get('extent', {})
    message.width = int(extent.get('width', 0))
    message.height = int(extent.get('height', 0))
    message.cell_states = list(value.get('cell_states', []))
    message.unknown_policy = value.get('unknown_policy', '')
    message.multiple_surfaces_per_xy = bool(value.get('multiple_surfaces_per_xy', False))
    source = value.get('source', {})
    message.source_uri = source.get('uri', '')
    message.source_sha256 = source.get('sha256', '')
    for surface in value.get('surfaces', []):
        message.surfaces.append(surface_msg(surface))
    return message


def support_map_value(message):
    return {'schema': message.schema, 'map_version': message.map_version,
            'frame': message.frame, 'kind': message.kind,
            'resolution': message.resolution, 'origin': list(message.origin),
            'extent': {'width': message.width, 'height': message.height},
            'cell_states': list(message.cell_states),
            'unknown_policy': message.unknown_policy,
            'multiple_surfaces_per_xy': message.multiple_surfaces_per_xy,
            'source': {'uri': message.source_uri,
                       'sha256': message.source_sha256},
            'surfaces': [surface_value(item) for item in message.surfaces]}


def surfaces_from_support_map(message):
    return [surface_value(item) for item in message.surfaces]


def delta_msg(value, header=None):
    message = msgs.MapDelta()
    if header is not None:
        message.header = header
    message.schema_version = int(value['schema_version'])
    message.map_id = value['map_id']
    message.base_version = int(value['base_version'])
    message.delta_id = value['delta_id']
    message.source_uri = value['source']['uri']
    message.source_sha256 = value['source']['sha256']
    for op in value['ops']:
        item = msgs.MapOp()
        item.op = op['op']
        if op['op'] == 'upsert_surface':
            item.surface = surface_msg(op['surface'])
            item.surface_id = op['surface']['surface_id']
            item.has_surface = True
        else:
            item.surface_id = op['surface_id']
            item.has_surface = False
        message.ops.append(item)
    return message


def delta_value(message):
    ops = []
    for item in message.ops:
        if item.op == 'upsert_surface':
            ops.append({'op': 'upsert_surface',
                        'surface': surface_value(item.surface)})
        else:
            ops.append({'op': 'remove_surface',
                        'surface_id': item.surface_id})
    return {'schema_version': message.schema_version,
            'map_id': message.map_id,
            'base_version': message.base_version,
            'delta_id': message.delta_id,
            'source': {'uri': message.source_uri,
                       'sha256': message.source_sha256},
            'ops': ops}
