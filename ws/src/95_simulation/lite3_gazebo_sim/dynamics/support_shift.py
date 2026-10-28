"""Minimum-motion CoM shift inside a triangle eroded by a fixed margin.

Weighting longitudinal motion avoids the unnecessary backward travel of a
triangle incenter target. Feasibility always uses the same geometric margin.
"""
import math
import numpy as np


def closest_shift_target(point, supports, margin, longitudinal_weight=9.0):
    point = np.asarray(point, dtype=float)
    feet = np.asarray(supports, dtype=float)
    if (point.shape != (2,) or feet.shape != (3, 2)
            or not np.isfinite(point).all() or not np.isfinite(feet).all()
            or not math.isfinite(margin) or margin < 0
            or not math.isfinite(longitudinal_weight) or longitudinal_weight <= 0):
        raise ValueError('finite 2D support triangle, margin and positive weight required')
    normals, offsets = [], []
    for index in range(3):
        a, b, c = feet[index], feet[(index + 1) % 3], feet[(index + 2) % 3]
        edge = b - a
        length = np.linalg.norm(edge)
        if length < 1e-10:
            return None
        normal = np.array([-edge[1], edge[0]]) / length
        if np.dot(normal, c - a) < 0:
            normal = -normal
        normals.append(normal)
        offsets.append(np.dot(normal, a) + margin)
    normals, offsets = np.asarray(normals), np.asarray(offsets)
    feasible = lambda q: bool(np.all(normals @ q >= offsets - 1e-10))
    inverse_weight = np.diag([1.0 / longitudinal_weight, 1.0])
    candidates = [point] if feasible(point) else []
    for index, normal in enumerate(normals):
        displacement = (offsets[index] - np.dot(normal, point)) / (normal @ inverse_weight @ normal)
        candidate = point + displacement * (inverse_weight @ normal)
        if feasible(candidate):
            candidates.append(candidate)
        for other in range(index):
            matrix = normals[[index, other]]
            if abs(np.linalg.det(matrix)) > 1e-10:
                corner = np.linalg.solve(matrix, offsets[[index, other]])
                if feasible(corner):
                    candidates.append(corner)
    if not candidates:
        return None
    return min(candidates, key=lambda q: longitudinal_weight * (q[0] - point[0]) ** 2
               + (q[1] - point[1]) ** 2).copy()
