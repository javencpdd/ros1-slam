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
