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


def world_offset_to_path(de: float, dn: float, yaw: float) -> tuple:
    """World east/north offset → path frame (forward, left) at heading yaw.

    Gazebo yaw 0 points along +X, and +X is east. Path x is forward along yaw,
    path y is to the left. This is the inverse of the helix controller's
    path-frame rotation.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * de + s * dn, -s * de + c * dn)


def swim_path_end(
    yaw: float,
    relative_to: str,
    north_m: float = 0.0,
    east_m: float = 0.0,
    along_heading_m: Optional[float] = None,
    x0: float = 0.0,
    y0: float = 0.0,
) -> tuple:
    """End point (forward, left) of one straight leg in the thrust-on path frame.

    relative_to:
      current — north_m / east_m are an ENU offset from the arm pose
      world   — north_m / east_m are absolute ENU; x0, y0 are the arm pose
      heading — along_heading_m (or north_m) is forward, east_m is starboard
    """
    rel = (relative_to or "current").lower()
    if rel == "absolute":
        rel = "world"
    if rel == "heading":
        fwd = float(along_heading_m if along_heading_m is not None else north_m)
        return (fwd, -float(east_m))
    if rel == "world":
        de, dn = float(east_m) - float(x0), float(north_m) - float(y0)
    else:
        de, dn = float(east_m), float(north_m)
    return world_offset_to_path(de, dn, yaw)


def curve_speed_cap(radius_m: float, mass_kg: float = 13.666, force_n: float = 144.0) -> float:
    """Pace at which a circular turn's m v²/r equals full forward thrust."""
    return math.sqrt(force_n * max(float(radius_m), 0.05) / mass_kg)


def _eased_pieces(waypoints: list, speed_mps: float, fillet_m: float):
    """Straight runs joined by circular fillets. Constant pace, slow enough for the tightest fillet."""
    pts = [tuple(float(c) for c in p) for p in waypoints]
    clean = []
    for p in pts:
        if not clean or math.dist(p, clean[-1]) > 1e-4:
            clean.append(p)
    if len(clean) < 2:
        return []
    n = len(clean)
    arcs = [None] * n
    for i in range(1, n - 1):
        a, b, c = clean[i - 1], clean[i], clean[i + 1]
        v1 = tuple(b[k] - a[k] for k in range(3))
        v2 = tuple(c[k] - b[k] for k in range(3))
        l1 = math.sqrt(sum(x * x for x in v1))
        l2 = math.sqrt(sum(x * x for x in v2))
        if l1 < 1e-4 or l2 < 1e-4:
            continue
        d1 = tuple(x / l1 for x in v1)
        d2 = tuple(x / l2 for x in v2)
        if abs(d1[2]) > 0.5 or abs(d2[2]) > 0.5:
            continue
        dot = max(-1.0, min(1.0, sum(d1[k] * d2[k] for k in range(3))))
        alpha = math.acos(dot)
        if alpha < 0.08 or alpha > 2.8:
            continue
        half = math.tan(alpha / 2.0)
        r = float(fillet_m)
        trim_len = r * half
        limit = 0.45 * min(l1, l2)
        if trim_len > limit:
            trim_len = limit
            r = trim_len / half
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        sign = 1.0 if cross >= 0.0 else -1.0
        p_in = tuple(b[k] - d1[k] * trim_len for k in range(3))
        p_out = tuple(b[k] + d2[k] * trim_len for k in range(3))
        nx, ny = -d1[1] * sign, d1[0] * sign
        center = (p_in[0] + nx * r, p_in[1] + ny * r, p_in[2])
        a0 = math.atan2(p_in[1] - center[1], p_in[0] - center[0])
        a1 = math.atan2(p_out[1] - center[1], p_out[0] - center[0])
        sweep = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        arcs[i] = {
            "center": center, "r": r, "a0": a0, "sweep": sweep,
            "p_in": p_in, "p_out": p_out, "z0": p_in[2], "z1": p_out[2],
        }
    speed = max(float(speed_mps), 0.05)
    pieces = []
    for i in range(n - 1):
        a, b = clean[i], clean[i + 1]
        p0 = arcs[i]["p_out"] if arcs[i] else a
        p1 = arcs[i + 1]["p_in"] if arcs[i + 1] else b
        length = math.dist(p0, p1)
        if length > 1e-4:
            direction = tuple((p1[k] - p0[k]) / length for k in range(3))
            pieces.append({
                "kind": "line", "p0": p0, "d": direction, "length": length,
                "speed": speed, "dt": length / speed,
            })
        arc = arcs[i + 1]
        if arc:
            alen = abs(arc["sweep"]) * arc["r"]
            pieces.append({
                "kind": "arc", **arc, "length": alen, "speed": speed,
                "dt": alen / max(speed, 1e-6),
            })
    return pieces


def _sample_piece(piece: dict, tau: float, held: bool):
    if piece["kind"] == "line":
        s = piece["length"] if held else piece["speed"] * tau
        pos = tuple(piece["p0"][k] + piece["d"][k] * s for k in range(3))
        if held:
            return pos, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        vel = tuple(piece["d"][k] * piece["speed"] for k in range(3))
        return pos, vel, (0.0, 0.0, 0.0)
    if piece["dt"] <= 1e-9:
        theta = piece["a0"] + piece["sweep"]
        w = 0.0
        u = 1.0
    else:
        u = 1.0 if held else min(1.0, max(0.0, tau / piece["dt"]))
        theta = piece["a0"] + piece["sweep"] * u
        w = 0.0 if held else piece["sweep"] / piece["dt"]
    c, s = math.cos(theta), math.sin(theta)
    r = piece["r"]
    z = piece["z0"] + (piece["z1"] - piece["z0"]) * u
    pos = (piece["center"][0] + r * c, piece["center"][1] + r * s, z)
    if held:
        return pos, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    vz = (piece["z1"] - piece["z0"]) / piece["dt"] if piece["dt"] > 1e-9 else 0.0
    vel = (-r * s * w, r * c * w, vz)
    acc = (-r * c * w * w, -r * s * w * w, 0.0)
    return pos, vel, acc


def eased_duration(waypoints: list, speed_mps: float, fillet_m: float = 0.6) -> float:
    """Seconds to swim a filleted polyline at the curvature-limited pace."""
    return sum(p["dt"] for p in _eased_pieces(waypoints, speed_mps, fillet_m))


def eased_polyline(waypoints: list, speed_mps: float, t: float, fillet_m: float = 0.6):
    """Position, velocity, and acceleration along a filleted polyline.

    Corners become circular arcs. The pace stays at the requested speed.
    After the end, the last point is held with zero velocity.
    """
    pieces = _eased_pieces(waypoints, speed_mps, fillet_m)
    if not pieces:
        p = tuple(waypoints[-1]) if waypoints else (0.0, 0.0, 0.0)
        return p, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    remain = max(0.0, float(t))
    for i, piece in enumerate(pieces):
        last = i == len(pieces) - 1
        if remain <= piece["dt"] or last:
            held = last and remain >= piece["dt"]
            tau = piece["dt"] if held else remain
            return _sample_piece(piece, tau, held)
        remain -= piece["dt"]
    return _sample_piece(pieces[-1], pieces[-1]["dt"], True)


def trapezoid(length_m: float, vmax: float, amax: float, t: float):
    """(s, v, a) along a straight run that speeds up, cruises, and slows to a stop.

    A run too short to reach vmax peaks lower (a triangle profile).
    """
    length = max(0.0, float(length_m))
    amax = max(1e-3, float(amax))
    vmax = max(1e-3, float(vmax))
    t_acc = vmax / amax
    if amax * t_acc * t_acc > length:
        t_acc = math.sqrt(length / amax)
        vmax = amax * t_acc
    d_acc = 0.5 * amax * t_acc * t_acc
    t_cruise = (length - 2.0 * d_acc) / vmax
    total = 2.0 * t_acc + t_cruise
    t = max(0.0, float(t))
    if t >= total:
        return length, 0.0, 0.0
    if t < t_acc:
        return 0.5 * amax * t * t, amax * t, amax
    if t < t_acc + t_cruise:
        return d_acc + vmax * (t - t_acc), vmax, 0.0
    r = total - t
    return length - 0.5 * amax * r * r, amax * r, -amax


def trapezoid_time(length_m: float, vmax: float, amax: float) -> float:
    length = max(0.0, float(length_m))
    amax = max(1e-3, float(amax))
    vmax = max(1e-3, float(vmax))
    t_acc = vmax / amax
    if amax * t_acc * t_acc > length:
        return 2.0 * math.sqrt(length / amax)
    return 2.0 * t_acc + (length - amax * t_acc * t_acc) / vmax


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
