"""Frozen support-cell contract used by the simulation carrier.

Canonical implementation: v8_platform/representation/support_map.py.  The
simulation package must run without the navigation platform's source, so this
file keeps the three pure helpers the carrier needs.  test_support_cells.py
compares both implementations and fails on drift; change both together.
"""
import math


def decode_cells(payload):
    """Runs payload -> set of (i, j). Empty/None -> None (bbox fallback)."""
    if not payload or not payload.get('runs'):
        return None
    cells = set()
    for j, start, count in payload['runs']:
        for i in range(start, start + count):
            cells.add((int(i), int(j)))
    return cells


def surface_cells(surface):
    """Decoded cell set attached to a surface, or None when it carries none."""
    return decode_cells(surface.get('cells'))


def contains_xy(surface, resolution, x, y, margin=0.0, cells=None):
    """Is (x, y) inside the surface, shrunk by ``margin`` metres?

    With a real cell set (map v3) the answer is the pose's own cell; negative
    margins fall back to the expanded box.  Identical semantics to support_map.
    """
    box = surface['footprint']
    if cells is None:
        return (box['min_x'] + margin - 1e-9 <= x <= box['max_x'] - margin + 1e-9
                and box['min_y'] + margin - 1e-9 <= y <= box['max_y'] - margin + 1e-9)
    if margin >= 0:
        return (math.floor(x / resolution + 1e-9),
                math.floor(y / resolution + 1e-9)) in cells
    padding = -margin
    i0 = math.floor((x - padding) / resolution + 1e-9)
    i1 = math.floor((x + padding) / resolution + 1e-9)
    j0 = math.floor((y - padding) / resolution + 1e-9)
    j1 = math.floor((y + padding) / resolution + 1e-9)
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            if (i, j) not in cells:
                continue
            dx = max(i * resolution - x, 0.0, x - (i + 1) * resolution)
            dy = max(j * resolution - y, 0.0, y - (j + 1) * resolution)
            if math.hypot(dx, dy) <= padding:
                return True
    return False
