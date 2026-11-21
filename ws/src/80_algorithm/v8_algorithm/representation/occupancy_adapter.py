"""PGM -> OccupancyGrid data for 2D maps; no ROS import.

The map JSON owns metadata; the PGM is the occupancy raster. Rows are flipped
to ROS bottom-up order; 0 is occupied, 254 is free, other values unknown by the
standard map_server thresholds.
"""
import math
from pathlib import Path

OCCUPIED, FREE = 100, 0
UNKNOWN = -1


def read_pgm(path):
    path = Path(path)
    raw = path.read_bytes()
    tokens, index = [], 0
    while index < len(raw):
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        if index >= len(raw):
            break
        if raw[index:index + 1] == b'#':
            while index < len(raw) and raw[index:index + 1] != b'\n':
                index += 1
            continue
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        tokens.append(raw[start:index])
    if len(tokens) < 4:
        raise ValueError('E_PGM_HEADER')
    magic = tokens[0]
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise ValueError('E_PGM_HEADER')
    body_start = 0
    for _ in range(4):
        body_start = raw.find(tokens[_], body_start) + len(tokens[_])
    if magic == b'P5':
        data_start = body_start
        if raw[data_start:data_start + 1] == b'\r':
            data_start += 1
        if raw[data_start:data_start + 1] == b'\n':
            data_start += 1
        elif raw[data_start:data_start + 1].isspace():
            data_start += 1
        size = width * height * (1 if maxval < 256 else 2)
        pixels = raw[data_start:data_start + size]
        if len(pixels) != size:
            raise ValueError('E_PGM_BODY')
        values = list(pixels) if maxval < 256 else [
            (pixels[i] << 8) | pixels[i + 1] for i in range(0, len(pixels), 2)]
    elif magic == b'P2':
        rest = raw[body_start:].decode('ascii', errors='ignore').split()
        values = [int(value) for value in rest[:width * height]]
        if len(values) != width * height:
            raise ValueError('E_PGM_BODY')
    else:
        raise ValueError('E_PGM_MAGIC')
    if any(value < 0 or value > maxval for value in values):
        raise ValueError('E_PGM_BODY')
    return width, height, maxval, values


def occupancy_grid(map_value, pgm_path):
    if map_value.get('kind') != 'occupancy_grid':
        raise ValueError('E_OCCUPANCY_KIND')
    width, height, maxval, values = read_pgm(pgm_path)
    extent = map_value.get('extent') or {}
    if (extent.get('width'), extent.get('height')) != (width, height):
        raise ValueError('E_OCCUPANCY_EXTENT')
    occupied_cut = maxval * (1. - .65)
    free_cut = maxval * (1. - .196)
    data = []
    for y in range(height - 1, -1, -1):
        row = values[y * width:(y + 1) * width]
        data.extend(OCCUPIED if value <= occupied_cut else
                    FREE if value >= free_cut else UNKNOWN for value in row)
    origin = map_value.get('origin') or [0., 0., 0.]
    return {'frame_id': map_value.get('frame', 'map'),
            'resolution': float(map_value['resolution']),
            'width': width, 'height': height,
            'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
            'data': data}
