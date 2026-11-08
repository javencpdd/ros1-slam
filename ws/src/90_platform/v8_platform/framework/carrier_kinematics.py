"""Pure support-aware kinematics for the Gazebo pose-sink carrier."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'representation'))
import support_map as rep


def _finite_vector(point):
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in point[:3])


def advance(position, velocity, dt):
    return tuple(position[i] + velocity[i] * dt for i in range(3))


def _candidate_indices(position, resolution):
    axes = []
    for axis in (0, 1):
        axes.append({math.floor((position[axis] + delta) / resolution + 1e-9)
                     for delta in (-1e-9, 0., 1e-9)})
    return {(i, j) for i in axes[0] for j in axes[1]}


def _eligible_supports(position, surfaces, snap_m, resolution=None):
    choices = []
    use_cells = resolution is not None and math.isfinite(resolution) and resolution > 0.
    for surface in surfaces:
        if surface.get('traversability') != 'supported':
            continue
        dz = abs(position[2] - surface['z_reference'])
        if dz > snap_m:
            continue
        cells = rep.surface_cells(surface) if use_cells else None
        if cells is not None:
            if not (_candidate_indices(position, resolution) & cells):
                continue
        elif not rep.contains_xy(surface, 1., position[0], position[1], cells=None):
            continue
        choices.append((dz, surface['z_reference'], surface))
    return choices


def snap_position(position, velocity, surfaces, snap_m):
    if not _finite_vector(position):
        return position
    choices = _eligible_supports(position, surfaces, snap_m)
    if velocity[2] > 1e-9:
        choices = [item for item in choices if item[1] >= position[2] - 1e-9]
    elif velocity[2] < -1e-9:
        choices = [item for item in choices if item[1] <= position[2] + 1e-9]
    if not choices:
        return position
    return (position[0], position[1],
            min(choices, key=lambda item: (item[0], item[1]))[1])


def _movement_limit(position, velocity, surface, resolution):
    """Return (limit_dt, edge_clamped_position) for this support, or None."""
    use_cells = resolution is not None and math.isfinite(resolution) and resolution > 0.
    cells = rep.surface_cells(surface) if use_cells else None
    clamped = list(position)
    if cells is not None:
        indices = [index for index in _candidate_indices(position, resolution)
                   if index in cells]
        limits = []
        for index in indices:
            bounds = []
            for axis in (0, 1):
                speed = velocity[axis]
                if abs(speed) <= 1e-15:
                    continue
                step = 1 if speed > 0. else -1
                edge_index = index[axis]
                while True:
                    probe = list(index)
                    probe[axis] = edge_index + step
                    if (probe[0], probe[1]) not in cells:
                        break
                    edge_index = probe[axis]
                boundary = ((edge_index + 1) if step > 0 else edge_index) * resolution
                travel = (boundary - position[axis]) / speed
                if travel >= -1e-9:
                    bounds.append(max(0., travel))
                    if travel < 0.:
                        clamped[axis] = boundary
            if bounds:
                limits.append(min(bounds))
        return (min(limits), tuple(clamped)) if limits else None
    footprint = surface['footprint']
    bounds = []
    for axis, (lo, hi) in enumerate(((footprint['min_x'], footprint['max_x']),
                                     (footprint['min_y'], footprint['max_y']))):
        speed = velocity[axis]
        if abs(speed) <= 1e-15:
            continue
        boundary = hi if speed > 0. else lo
        travel = (boundary - position[axis]) / speed
        if travel >= -1e-9:
            bounds.append(max(0., travel))
            if travel < 0.:
                clamped[axis] = boundary
    return (min(bounds), tuple(clamped)) if bounds else None


def piecewise_advance(position, velocity, dt, surfaces, resolution=None, snap_m=.03,
                      z_transfer_speed=1e-9):
    """Advance horizontally only while the same-height support continues.

    A nonzero vertical command is an explicit stair transfer and may cross the
    XY boundary.  With zero vertical speed, the carrier never integrates past
    the current v3 cell run / footprint edge; contiguous cells remain one run.
    """
    if not _finite_vector(position):
        return position
    if (not isinstance(dt, (int, float)) or isinstance(dt, bool) or not math.isfinite(dt)
            or not _finite_vector(velocity)):
        return position
    if dt <= 0. or all(abs(value) <= 1e-15 for value in velocity):
        return position
    if abs(velocity[2]) > z_transfer_speed:
        return advance(position, velocity, dt)
    supports = _eligible_supports(position, surfaces, snap_m, resolution)
    limits = [_movement_limit(position, velocity, surface, resolution)
              for _, _, surface in supports]
    limits = [limit for limit in limits if limit is not None]
    if not limits:
        return advance(position, velocity, dt)
    limit, clamped = min(limits, key=lambda item: item[0])
    if limit <= 0.:
        return clamped
    return advance(position, velocity, min(dt, limit))
