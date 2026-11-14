"""Coverage patterns for CoRAL boat surveys and AUV volume search. ROS-free."""
from __future__ import annotations

import math
from typing import Optional


def lawnmower(center_n: float, center_e: float, width_m: float, length_m: float, swath_m: float,
              heading_deg: float = 0.0, start: Optional[tuple] = None, max_points: int = 100) -> list:
    """Boustrophedon waypoints [(north, east), ...] covering a width x length rectangle."""
    for name, v in (("width_m", width_m), ("length_m", length_m), ("swath_m", swath_m)):
        if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
            raise ValueError(f"{name} must be a positive number, got {v!r}")
    lanes = max(1, math.ceil(width_m / swath_m - 1e-9))
    if 2 * lanes > max_points:
        raise ValueError(f"{lanes} lanes ({2 * lanes} waypoints) exceeds the limit of {max_points}; "
                         "use a wider swath or a smaller area")
    h = math.radians(heading_deg)
    along, across = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
    span = (lanes - 1) * swath_m

    def point(a: float, x: float) -> tuple:
        return (round(center_n + a * along[0] + x * across[0], 2),
                round(center_e + a * along[1] + x * across[1], 2))

    best = None
    for flip_lanes in (False, True):
        for flip_dir in (False, True):
            pts = []
            for i in range(lanes):
                x = -span / 2 + i * swath_m
                if flip_lanes:
                    x = -x
                a0, a1 = -length_m / 2, length_m / 2
                if (i % 2 == 1) != flip_dir:
                    a0, a1 = a1, a0
                pts += [point(a0, x), point(a1, x)]
            d = math.hypot(pts[0][0] - start[0], pts[0][1] - start[1]) if start else 0.0
            if best is None or d < best[0] - 1e-9:
                best = (d, pts)
    return best[1]


def depth_layers(z_top: float, z_bottom: float, layers: int) -> list:
    """Evenly spaced depths (world z, negative down). layers >= 1."""
    n = max(1, int(layers))
    if n == 1:
        return [0.5 * (z_top + z_bottom)]
    return [z_top + (z_bottom - z_top) * i / (n - 1) for i in range(n)]


def volume_lawnmower(width_m: float, length_m: float, swath_m: float, depths: list,
                     heading_deg: float = 0.0, max_points_per_layer: int = 80) -> list:
    """3D boustrophedon: horizontal lawnmower at each depth, stacked.

    Returns waypoints [(x, y, z), ...] in a path frame where +x is along
    ``heading_deg`` (0 = +north) and +y is to the left — same convention as
    ``lawnmower``'s (north, east) with heading 0. Centre is the origin.
    Consecutive layers reverse lane order so the vertical hop is short.
    """
    if not depths:
        raise ValueError("depths must be non-empty")
    out = []
    for i, z in enumerate(depths):
        pts2d = lawnmower(0.0, 0.0, width_m, length_m, swath_m, heading_deg=heading_deg,
                          max_points=max_points_per_layer)
        layer = [(n, e, float(z)) for n, e in pts2d]
        if i % 2 == 1:
            layer.reverse()
        out.extend(layer)
    return out


def path_length_3d(points: list) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def polyline_pos_vel(waypoints: list, speed_mps: float, t: float):
    """Constant-speed interpolation along a 3D polyline.

    Returns ((x,y,z), (vx,vy,vz)). After the end, holds the last point with zero vel.
    """
    if not waypoints:
        raise ValueError("waypoints empty")
    if len(waypoints) == 1 or speed_mps <= 0:
        p = tuple(waypoints[-1])
        return p, (0.0, 0.0, 0.0)
    seg_lens = [math.dist(a, b) for a, b in zip(waypoints, waypoints[1:])]
    total = sum(seg_lens)
    if total < 1e-9:
        p = tuple(waypoints[-1])
        return p, (0.0, 0.0, 0.0)
    s = min(max(0.0, t * speed_mps), total)
    acc = 0.0
    for i, L in enumerate(seg_lens):
        if acc + L >= s - 1e-12:
            a, b = waypoints[i], waypoints[i + 1]
            u = 0.0 if L < 1e-9 else (s - acc) / L
            pos = tuple(a[j] + u * (b[j] - a[j]) for j in range(3))
            direction = tuple((b[j] - a[j]) / L for j in range(3)) if L >= 1e-9 else (0.0, 0.0, 0.0)
            vel = tuple(speed_mps * d for d in direction)
            return pos, vel
        acc += L
    p = tuple(waypoints[-1])
    return p, (0.0, 0.0, 0.0)
