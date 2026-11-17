"""Kiwanis hula hoop geometry. ROS-free.

The ring is vertical: a 1 m opening in the north-up plane, centred at ENU
(6, 0, -4.626), the midpoint of the 9.25 m water column there. The axis
runs east-west. The flight does not use that coordinate. It fits an ellipse
to the orange tube in the front camera and back-projects the centre with
the pinhole. The major axis is the 1.09 m outer diameter at any viewing angle.
"""
from __future__ import annotations

import math

HOOP_CENTER = (6.0, 0.0, -4.626)  # authored ring, east, north, up
HOOP_AXIS = (1.0, 0.0, 0.0)     # +east through the opening
HOOP_RADIUS = 0.50              # 1 m diameter, tube centreline
RING_OUTER_M = 1.09             # outer diameter: 1.0 m plus the 0.045 m tube
INNER_PASS_M = 0.40             # inside the opening, clear of the tube
CAM_FORWARD_M = 0.185           # front camera, metres ahead of the hull origin
SIDEWAYS_M = 0.28               # baseline before a ring fix may confirm itself
STABLE_M = 4.0                  # inside this range the final is frozen
LOCALIZER_MPS = 0.08            # how fast that trim may slide the line
LOCALIZER_GAIN = 0.35           # fraction of the image error taken while still far
LOCALIZER_LAT_M = 0.28          # cap on that sideways trim
LOCALIZER_VERT_M = 0.18         # cap on the vertical trim


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def through_ends(vehicle_enu, center, axis=HOOP_AXIS, lead_m: float = 2.4, exit_m: float = 2.0):
    """Standoff on the vehicle's side of the ring, and the point past the opening.

    Both lie on the hoop axis, so the segment between them passes through the centre.
    """
    ax = tuple(float(v) for v in axis)
    n = math.sqrt(_dot(ax, ax))
    ax = tuple(v / n for v in ax)
    rel = tuple(float(vehicle_enu[i]) - float(center[i]) for i in range(3))
    side = 1.0 if _dot(rel, ax) >= 0.0 else -1.0
    start = tuple(float(center[i]) + side * lead_m * ax[i] for i in range(3))
    end = tuple(float(center[i]) - side * exit_m * ax[i] for i in range(3))
    return start, end


def in_opening(pos, center, axis=HOOP_AXIS, radius: float = INNER_PASS_M, plane_tol: float = 0.30) -> bool:
    """True when pos is in the disk of the opening, within plane_tol of the ring plane."""
    ax = tuple(float(v) for v in axis)
    n = math.sqrt(_dot(ax, ax))
    ax = tuple(v / n for v in ax)
    d = tuple(float(pos[i]) - float(center[i]) for i in range(3))
    along = _dot(d, ax)
    radial2 = _dot(d, d) - along * along
    return abs(along) <= plane_tol and radial2 <= radius * radius


def level_feature(x_n, y_n, pitch):
    """Calibrated image point with pitch taken out.

    ``pitch`` is the body angle from ``rpy_from_q``, positive nose-down.
    ``y_n`` is positive below the optical axis. A target at the same depth
    then sits near ``y_level`` of 0 while the hull holds depth nose-down.
    """
    ang = math.atan(float(y_n)) + float(pitch)
    if ang > 1.2:
        ang = 1.2
    elif ang < -1.2:
        ang = -1.2
    return float(x_n), math.tan(ang)


def sweep_points(vehicle, center, lead_m: float = 2.4, exit_m: float = 2.0):
    """Intercept the centerline, then fly it straight and level through the ring.

    The first leg leaves the present pose and does most of the depth change.
    The rollout, the opening, and the exit are one straight line at the ring
    depth, so the short final does not pitch or turn. Along-track distance
    increases the whole way, and the rollout stays in front of the ring.
    """
    de = float(center[0]) - float(vehicle[0])
    dn = float(center[1]) - float(vehicle[1])
    dist = math.hypot(de, dn)
    if dist < 0.4:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = de / dist, dn / dist
    lead = min(float(lead_m), max(0.6, dist * 0.55))
    z = pass_depth(center[2])
    stand = (float(center[0]) - ux * lead, float(center[1]) - uy * lead, z)
    ease = (
        float(vehicle[0]) + 0.45 * (stand[0] - float(vehicle[0])),
        float(vehicle[1]) + 0.45 * (stand[1] - float(vehicle[1])),
        float(vehicle[2]) + 0.85 * (z - float(vehicle[2])),
    )
    opening = (float(center[0]), float(center[1]), z)
    exit_pt = (float(center[0]) + ux * exit_m, float(center[1]) + uy * exit_m, z)
    start = (float(vehicle[0]), float(vehicle[1]), float(vehicle[2]))
    return [start, ease, stand, opening, exit_pt], (ux, uy, 0.0)


def _hermite(s, p0, t0, p1, t1):
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return tuple(
        h00 * p0[i] + h10 * t0[i] + h01 * p1[i] + h11 * t1[i]
        for i in range(3)
    )


def _sample_line(a, b, step):
    dist = math.dist(a, b)
    n = max(1, int(math.ceil(dist / max(step, 0.05))))
    return [
        tuple(a[i] + (k / n) * (b[i] - a[i]) for i in range(3))
        for k in range(n + 1)
    ]


def _peak_turn(points):
    """Largest heading change, in radians, between successive steps."""
    dirs = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        if n < 1e-4:
            continue
        dirs.append((dx / n, dy / n))
    worst = 0.0
    for a, b in zip(dirs, dirs[1:]):
        dot = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1]))
        worst = max(worst, math.acos(dot))
    return worst


def fly_trajectory(pos, heading, center, lead_m: float = 1.62, exit_m: float = 2.0,
                   through: bool = True):
    """One smooth path onto the ring, then — when ``through`` — straight out the far side.

    The curve is a Hermite join from the present nose onto the centerline.
    Tangent length is chosen from a few candidates so the peak turn stays
    small and the path does not loop. The rollout, the opening, and the exit
    are then one straight level line. The result is a polyline the force
    loop can track without further reshaping.
    """
    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
    hx, hy = float(heading[0]), float(heading[1])
    hn = math.hypot(hx, hy)
    if hn < 0.2:
        hx, hy = 1.0, 0.0
    else:
        hx, hy = hx / hn, hy / hn
    cx, cy = float(center[0]), float(center[1])
    de, dn = cx - px, cy - py
    dist = math.hypot(de, dn)
    if dist < 0.4:
        ax, ay = hx, hy
    else:
        ax, ay = de / dist, dn / dist
    z = pass_depth(center[2])
    if dist < 0.9:
        lead = max(0.35, dist * 0.40)
    else:
        lead = min(float(lead_m), max(0.9, dist * 0.50))
    lead = min(lead, max(0.3, dist - 0.35))
    gate = (cx - ax * lead, cy - ay * lead, z)
    opening = (cx, cy, z)
    exit_pt = (cx + ax * float(exit_m), cy + ay * float(exit_m), z)
    chord = math.dist((px, py, pz), gate)
    dot = hx * ax + hy * ay
    if dot > 0.75:
        bx, by = hx, hy
    else:
        bn = math.hypot(hx + ax, hy + ay)
        if bn < 1e-3:
            bx, by = ax, ay
        else:
            bx, by = (hx + ax) / bn, (hy + ay) / bn

    def curve(scale):
        tan = max(0.35, min(1.8, chord * scale))
        t0 = (bx * tan, by * tan, 0.0)
        t1 = (ax * tan, ay * tan, 0.0)
        n = max(6, int(math.ceil(max(chord, 0.3) / 0.25)))
        return [_hermite(k / n, (px, py, pz), t0, gate, t1) for k in range(n + 1)]

    def cost(pts):
        along = [(p[0] - px) * ax + (p[1] - py) * ay for p in pts]
        back = max(0.0, -min(along))
        return _peak_turn(pts) + 3.0 * back

    best = min((curve(scale) for scale in (0.25, 0.40, 0.55)), key=cost)
    if not through:
        return best, (ax, ay, 0.0)
    rest = _sample_line(gate, opening, 0.25)[1:]
    rest += _sample_line(opening, exit_pt, 0.25)[1:]
    return best + rest, (ax, ay, 0.0)


def look_step(pos, center, distance: float = 1.6):
    """A short straight step toward the ring. Depth stays where the hull is."""
    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
    de, dn = float(center[0]) - px, float(center[1]) - py
    dist = math.hypot(de, dn)
    if dist < 0.3:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = de / dist, dn / dist
    step = min(float(distance), max(0.4, dist * 0.35))
    end = (px + ux * step, py + uy * step, pz)
    return _sample_line((px, py, pz), end, 0.4), (ux, uy, 0.0)


def localizer_shift(x_l, y_l, z_opt, left, up, stable_m: float = STABLE_M,
                    gain: float = LOCALIZER_GAIN, lat_max: float = LOCALIZER_LAT_M,
                    vert_max: float = LOCALIZER_VERT_M):
    """ENU trim that recenters the ring while the final is still long.

    Returns ``(shift, freeze)``. ``shift`` is None to hold the line already
    being flown: either the nose is still turning onto it, or the range is
    inside ``stable_m`` and the final is frozen. Image +x is hull −Y and
    image +y is hull −Z, so a ring right of the optical axis moves the line
    along −left.
    """
    z = float(z_opt)
    if not math.isfinite(z) or not math.isfinite(float(x_l)) or not math.isfinite(float(y_l)):
        return None, False
    if z <= float(stable_m):
        return None, True
    if abs(float(x_l)) > 0.22 or abs(float(y_l)) > 0.28:
        return None, False
    lat = max(-float(lat_max), min(float(lat_max), float(x_l) * z * float(gain)))
    vert = max(-float(vert_max), min(float(vert_max), float(y_l) * z * float(gain)))
    shift = tuple(-lat * float(left[i]) - vert * float(up[i]) for i in range(3))
    return shift, False


def slew_toward(current, target, rate_mps: float, dt: float):
    """Step ``current`` toward ``target`` by at most ``rate_mps * dt``."""
    if target is None:
        return current
    if current is None:
        current = (0.0, 0.0, 0.0)
    dt = max(0.0, min(0.2, float(dt)))
    step = max(0.0, float(rate_mps)) * dt
    delta = tuple(float(target[i]) - float(current[i]) for i in range(3))
    dist = math.sqrt(sum(v * v for v in delta))
    if dist <= step or dist < 1e-9:
        return tuple(float(v) for v in target)
    scale = step / dist
    return tuple(float(current[i]) + scale * delta[i] for i in range(3))


def debug_caption(stage, kind, n_pts, remain_m, center, slide_m,
                  real_miss, surrounded, crossed):
    """Three short lines for the hoop debug image.

    The first names the plan and how far is left. The second is the fix and
    the cross-track already covered. During the pass, the third is the
    distance to the placed hoop and whether the opening has been met.
    """
    name = stage if stage == "through" or not kind else f"{stage} {kind}"
    if n_pts:
        lines = [f"{name}  {int(n_pts)} pts  remain {float(remain_m):.1f}m"]
    else:
        lines = [name]
    if center is not None:
        lines.append(
            f"fix {center[0]:.1f},{center[1]:.1f},{center[2]:.1f}  "
            f"slide {float(slide_m):.2f}m"
        )
    elif float(slide_m) > 0.0:
        lines.append(f"slide {float(slide_m):.2f}m")
    if stage == "through":
        miss = float(real_miss)
        miss_s = f"{miss:.1f}m" if math.isfinite(miss) else "inf"
        lines.append(
            f"real {miss_s}  surround {int(bool(surrounded))}  "
            f"opening {int(bool(crossed))}"
        )
    return lines


def horizontal_axis(fwd):
    """Camera forward flattened into the horizontal plane, as a unit vector."""
    ax = (float(fwd[0]), float(fwd[1]), 0.0)
    n = math.hypot(ax[0], ax[1])
    if n < 0.2:
        return (1.0, 0.0, 0.0)
    return (ax[0] / n, ax[1] / n, 0.0)


def sideways_baseline(sides, min_m: float = SIDEWAYS_M) -> bool:
    """True once the fixes span a cross-track move, so one viewpoint cannot agree with itself."""
    if len(sides) < 2:
        return False
    return max(sides) - min(sides) >= float(min_m)


def ring_center(u, v, major_px, fx, fy, cx, cy, veh_pos, fwd, left, up,
                diameter_m: float = RING_OUTER_M, cam_forward_m: float = CAM_FORWARD_M):
    """Ring-centre ENU and optical range from the fitted outer ellipse.

    The Gazebo front camera looks along the hull +X. Image +x is right, which is
    hull -Y (left is +Y), and image +y is down, which is hull -Z. A circle's
    major axis keeps the true diameter at any viewing angle, so the range is
    fx * outer_diameter / major_px.
    """
    major = float(major_px)
    if major < 8.0 or float(fx) <= 1.0:
        raise ValueError("ellipse or focal length is not usable")
    z_opt = float(fx) * float(diameter_m) / major
    x_n = (float(u) - float(cx)) / float(fx)
    y_n = (float(v) - float(cy)) / float(fy)
    bx = float(cam_forward_m) + z_opt
    by = -x_n * z_opt
    bz = -y_n * z_opt
    center = tuple(
        float(veh_pos[i]) + bx * fwd[i] + by * left[i] + bz * up[i]
        for i in range(3)
    )
    return center, z_opt


def orange_ring(bgr):
    """Largest orange ring in a BGR image, or None.

    The measurement is an ellipse fit to the outer contour of the orange
    pixels. The major axis is the outer diameter at any viewing angle; the
    upright bounding box is only the fallback when the fit fails. ``closed``
    is true when those pixels fill a thin ring rather than an arc or a solid
    blob.

    The tube in a real frame is about BGR (40, 120, 180). The lake bed is
    about (32, 66, 86) and a loose red-over-green test paints the whole
    bottom as one blob, so the gate stays on the saturated tube.
    """
    import cv2
    import numpy as np

    if bgr is None or getattr(bgr, "ndim", 0) != 3 or bgr.shape[2] < 3:
        return None
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    mask = (
        (r > 150) & (r > g + 45) & (r > b + 100) & (g > b + 25)
    ).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    height, width = bgr.shape[:2]
    count, _labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    best_score = 0.0
    for i in range(1, int(count)):
        x, y, w, h, area = (float(v) for v in stats[i])
        major = max(w, h)
        minor = min(w, h)
        if major < 18.0 or area < 40.0 or minor < 2.0:
            continue
        # A blob cut by the frame edge is often bed/pontoon clutter. Near the
        # ring the ellipse fills the frame and clips too — keep large clipped
        # blobs (was 160; that dropped FOV ~2–4 m out and aborted through).
        clipped = x <= 1.0 or y <= 1.0 or x + w >= width - 1.0 or y + h >= height - 1.0
        if clipped and major < 90.0:
            continue
        ell_area = math.pi * (major / 2.0) * (minor / 2.0)
        filled = area / max(ell_area, 1.0)
        aspect = minor / major
        eu, ev, emajor, eminor = _fit_outer_ellipse(int(i), _labels)
        if emajor is not None:
            major, minor = emajor, eminor
            aspect = minor / max(major, 1.0)
            cu, cv = eu, ev
        else:
            cu, cv = x + w / 2.0, y + h / 2.0
        # A tube across the box fills about a third of the disk. An arc is
        # thinner in that ratio or too flat to be the whole ring.
        closed = 0.10 <= filled <= 0.70 and aspect >= 0.35 and major >= 24.0
        score = major * (1.5 if closed else 0.6)
        if score > best_score:
            best_score = score
            best = {
                "u": cu,
                "v": cv,
                "w": w,
                "h": h,
                "major": major,
                "minor": minor,
                "aspect": aspect,
                "closed": closed,
            }
    return best


def _fit_outer_ellipse(label, labels):
    """Centre and full axes of the component's outer contour, or Nones."""
    import cv2
    import numpy as np

    comp = np.ascontiguousarray((labels == label).astype(np.uint8))
    contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None, None, None
    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 5:
        return None, None, None, None
    try:
        (cu, cv), (aw, ah), _angle = cv2.fitEllipse(contour)
    except cv2.error:
        return None, None, None, None
    major, minor = max(float(aw), float(ah)), min(float(aw), float(ah))
    if major < 8.0 or not all(math.isfinite(v) for v in (cu, cv, major, minor)):
        return None, None, None, None
    return float(cu), float(cv), major, minor


# Coverage grid for the hoop search. World-aligned 2.07 m cells. A cell is a
# glance at 1 and searched at 2, and only the front-camera wedge at hoop
# depth can raise the count. The wedge is half of a 640-wide frame at
# fx 381 (about ±40°), not a narrower glance. The box is 20 m on a side around the dock at
# the origin. A vehicle that arms outside it is sent to the nearest open cell.
CELL_M = 2.07
MAP_HALF_M = 10.0
WEDGE_NEAR_M = 1.5
WEDGE_FAR_M = 6.0
WEDGE_HALF_RAD = math.atan(320.0 / 381.46)
DEPTH_LO = -5.15
DEPTH_HI = -4.10
SEARCH_HITS = 2
LOOK_S = 1.5
SEARCH_Z = -4.626
# Below this the hull has left the ring and is heading for the bed.
HOOP_FLOOR = -6.2
VIEW_STAND_M = 3.31
VIEW_NEAR_M = 2.0
PASS_TOP = -0.5


def pass_depth(z: float) -> float:
    """Depth to fly through a ring seen at z, kept off the surface and the floor."""
    return min(PASS_TOP, max(HOOP_FLOOR + 0.5, float(z)))


# ---------------------------------------------------------------------------
# Approach corridor (rendezvous-style): station → gate → through → exit.
# Wide at the approach station, narrow at the gate; HUD draws the funnel.
# ---------------------------------------------------------------------------
APPROACH_STANDOFF_M = 3.5
GATE_STANDOFF_M = 1.20
EXIT_STANDOFF_M = 2.0
APPROACH_ARRIVE_M = 0.70
GATE_ARRIVE_M = 0.50
CORRIDOR_HALF_APPROACH_M = 1.20
CORRIDOR_HALF_GATE_M = 0.32
GLIDE_GATE_FRACS = (0.0, 0.33, 0.66, 1.0)


def _unit_xy(axis):
    ax, ay = float(axis[0]), float(axis[1])
    n = math.hypot(ax, ay)
    if n < 1e-9:
        return (1.0, 0.0, 0.0)
    return (ax / n, ay / n, 0.0)


def _left_xy(fwd):
    return (-float(fwd[1]), float(fwd[0]), 0.0)


def approach_side_sign(vehicle, center, axis=HOOP_AXIS) -> float:
    """+1 if the vehicle sits on the +axis side of the ring, else -1."""
    fwd = _unit_xy(axis)
    along = (
        (float(vehicle[0]) - float(center[0])) * fwd[0]
        + (float(vehicle[1]) - float(center[1])) * fwd[1]
    )
    return 1.0 if along >= 0.0 else -1.0


def through_dir(vehicle, center, axis=HOOP_AXIS):
    """Unit horizontal direction from the approach side through the opening."""
    fwd = _unit_xy(axis)
    side = approach_side_sign(vehicle, center, axis)
    # Station is on the vehicle's side; fly toward the opposite side.
    return (-side * fwd[0], -side * fwd[1], 0.0)


def approach_station(center, axis, vehicle, standoff_m: float = APPROACH_STANDOFF_M):
    """Centreline hold on the vehicle's side of the ring (like the aft station)."""
    fwd = _unit_xy(axis)
    side = approach_side_sign(vehicle, center, axis)
    z = pass_depth(center[2])
    stand = float(standoff_m)
    return (
        float(center[0]) + side * stand * fwd[0],
        float(center[1]) + side * stand * fwd[1],
        z,
    )


def gate_station(center, through, standoff_m: float = GATE_STANDOFF_M):
    """Threshold short of the opening on the approach centreline."""
    th = _unit_xy(through)
    z = pass_depth(center[2])
    stand = float(standoff_m)
    return (
        float(center[0]) - stand * th[0],
        float(center[1]) - stand * th[1],
        z,
    )


def exit_station(center, through, exit_m: float = EXIT_STANDOFF_M):
    """Rollout past the opening on the through axis."""
    th = _unit_xy(through)
    z = pass_depth(center[2])
    return (
        float(center[0]) + float(exit_m) * th[0],
        float(center[1]) + float(exit_m) * th[1],
        z,
    )


def hoop_frame(pos, center, through):
    """(s, y, z) with s along through (0 at opening; approach s < 0)."""
    th = _unit_xy(through)
    left = _left_xy(th)
    v = (
        float(pos[0]) - float(center[0]),
        float(pos[1]) - float(center[1]),
        float(pos[2]) - float(center[2]),
    )
    s = v[0] * th[0] + v[1] * th[1]
    y = v[0] * left[0] + v[1] * left[1]
    z = v[2]
    return s, y, z, th, left


def hoop_corridor(
    auv_pos,
    center,
    axis=HOOP_AXIS,
    prev_phase: str | None = None,
    approach_m: float = APPROACH_STANDOFF_M,
    gate_m: float = GATE_STANDOFF_M,
    exit_m: float = EXIT_STANDOFF_M,
    approach_arrive_m: float = APPROACH_ARRIVE_M,
    gate_arrive_m: float = GATE_ARRIVE_M,
    half_approach: float = CORRIDOR_HALF_APPROACH_M,
    half_gate: float = CORRIDOR_HALF_GATE_M,
):
    """Progress along the hoop approach corridor (rendezvous analogue).

    ``phase``:
      - ``approach``: swim to the approach station
      - ``commit``: fly the tapering funnel toward the gate
      - ``through``: at the gate, on-axis enough to punch through
    """
    th = through_dir(auv_pos, center, axis)
    appr = approach_station(center, axis, auv_pos, standoff_m=approach_m)
    gate = gate_station(center, th, standoff_m=gate_m)
    exit_pt = exit_station(center, th, exit_m=exit_m)
    s, y, z, fwd, left = hoop_frame(auv_pos, center, th)
    dist_appr = math.hypot(
        float(auv_pos[0]) - appr[0], float(auv_pos[1]) - appr[1],
    )
    dist_gate = math.hypot(
        float(auv_pos[0]) - gate[0], float(auv_pos[1]) - gate[1],
    )
    lat = math.hypot(y, z)
    # Corridor half-width at this along-track (lerp approach→gate).
    # s_appr ≈ -approach_m, s_gate ≈ -gate_m.
    span = max(1e-3, float(approach_m) - float(gate_m))
    t = max(0.0, min(1.0, (float(approach_m) + s) / span))
    half = float(half_approach) + t * (float(half_gate) - float(half_approach))
    in_funnel = lat <= half * 1.15 + 1e-9
    prev = (prev_phase or "").lower()
    sticky_commit = prev in ("commit", "through", "approach")
    on_gate = dist_gate <= float(gate_arrive_m) and lat <= float(half_gate) * 1.25
    if on_gate or (prev == "through" and s > -float(gate_m) - 0.4 and lat <= half_gate * 1.5):
        phase = "through"
    elif prev == "commit" and not (dist_appr > float(approach_m) + 1.5 and not in_funnel):
        phase = "commit"
    elif dist_appr <= float(approach_arrive_m) and in_funnel:
        phase = "commit"
    elif sticky_commit and s > -float(approach_m) - 0.5 and in_funnel:
        phase = "commit"
    else:
        phase = "approach"
    return {
        "phase": phase,
        "center": tuple(float(c) for c in center),
        "approach": appr,
        "gate": gate,
        "exit": exit_pt,
        "through": th,
        "fwd": fwd,
        "left": left,
        "s": s,
        "y": y,
        "z": z,
        "lat": lat,
        "half_m": half,
        "in_funnel": in_funnel,
        "dist_approach": dist_appr,
        "dist_gate": dist_gate,
    }


def _shift3(p, left, side: float, half_m: float):
    w = float(side) * float(half_m)
    return tuple(float(p[i]) + w * float(left[i]) for i in range(3))


def _lerp3(a, b, t: float):
    t = float(t)
    return tuple(float(a[i]) + t * (float(b[i]) - float(a[i])) for i in range(3))


def hoop_glide_path(
    center,
    axis=HOOP_AXIS,
    vehicle=None,
    approach_m: float = APPROACH_STANDOFF_M,
    gate_m: float = GATE_STANDOFF_M,
    exit_m: float = EXIT_STANDOFF_M,
    half_approach: float = CORRIDOR_HALF_APPROACH_M,
    half_gate: float = CORRIDOR_HALF_GATE_M,
    in_funnel: bool = False,
    through_ok: bool = False,
):
    """HUD funnel for the hoop corridor (ILS-style rails + range gates)."""
    veh = vehicle if vehicle is not None else (
        float(center[0]) - float(approach_m), float(center[1]), float(center[2]),
    )
    th = through_dir(veh, center, axis)
    left = _left_xy(th)
    appr = approach_station(center, axis, veh, standoff_m=approach_m)
    gate = gate_station(center, th, standoff_m=gate_m)
    exit_pt = exit_station(center, th, exit_m=exit_m)
    opening = (float(center[0]), float(center[1]), pass_depth(center[2]))

    ap_l = _shift3(appr, left, 1.0, half_approach)
    ap_r = _shift3(appr, left, -1.0, half_approach)
    gt_l = _shift3(gate, left, 1.0, half_gate)
    gt_r = _shift3(gate, left, -1.0, half_gate)
    ex_l = _shift3(exit_pt, left, 1.0, half_gate)
    ex_r = _shift3(exit_pt, left, -1.0, half_gate)

    approach = (0, 180, 80)
    tube = (255, 255, 255) if through_ok else ((0, 220, 0) if in_funnel else (0, 200, 255))
    paths = [
        {
            "pts": [ap_l, gt_l, gt_r, ap_r],
            "color": approach,
            "width": 1,
            "closed": True,
            "fill": True,
            "fill_alpha": 0.22,
            "label": "",
        },
        {"pts": [ap_l, gt_l], "color": approach, "width": 2, "label": ""},
        {"pts": [ap_r, gt_r], "color": approach, "width": 2, "label": ""},
        {"pts": [appr, gate], "color": approach, "width": 2, "label": "approach"},
        {"pts": [gate, opening, exit_pt], "color": tube, "width": 2, "label": "through"},
        {"pts": [gt_l, ex_l], "color": tube, "width": 1, "label": ""},
        {"pts": [gt_r, ex_r], "color": tube, "width": 1, "label": ""},
    ]
    for t in GLIDE_GATE_FRACS:
        c = _lerp3(appr, gate, t)
        w = float(half_approach) + t * (float(half_gate) - float(half_approach))
        paths.append({
            "pts": [_shift3(c, left, 1.0, w), _shift3(c, left, -1.0, w)],
            "color": approach,
            "width": 1,
            "label": (
                "" if t not in (0.0, 1.0)
                else (f"{(1.0 - t) * approach_m:.0f}m" if t < 1.0 else "gate")
            ),
        })
    markers = [
        {"pos": appr, "label": "approach", "color": approach, "shape": "diamond"},
        {"pos": gate, "label": "gate", "color": approach, "shape": "circle"},
        {"pos": opening, "label": "opening", "color": tube, "shape": "circle"},
        {"pos": exit_pt, "label": "exit", "color": tube, "shape": "diamond"},
    ]
    return {
        "paths": paths,
        "markers": markers,
        "approach": appr,
        "gate": gate,
        "exit": exit_pt,
        "opening": opening,
        "through": th,
        "left": left,
    }


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _columns(lo: float, hi: float, res: float):
    c0 = math.ceil(lo / res - 0.5 - 1e-9)
    c1 = math.floor(hi / res - 0.5 + 1e-9)
    return range(c0, c1 + 1)


class CoverageMap:
    """Hit count per cell. 0 unseen, 1 glanced, 2 searched."""

    def __init__(self, east0: float, north0: float, half_m: float = MAP_HALF_M, res: float = CELL_M):
        self.res = float(res)
        self.half = float(half_m)
        self.east0 = float(east0)
        self.north0 = float(north0)
        self.hits = {}
        self.since = {}
        lo_e, hi_e = -self.half, self.half
        lo_n, hi_n = -self.half, self.half
        self.cells = [
            (c, r)
            for c in _columns(lo_e, hi_e, self.res)
            for r in _columns(lo_n, hi_n, self.res)
        ]

    def center(self, cell) -> tuple:
        c, r = cell
        return ((c + 0.5) * self.res, (r + 0.5) * self.res)

    def in_wedge(self, east, north, yaw, cell_e, cell_n) -> bool:
        de = float(cell_e) - float(east)
        dn = float(cell_n) - float(north)
        dist = math.hypot(de, dn)
        if dist < WEDGE_NEAR_M or dist > WEDGE_FAR_M:
            return False
        bearing = math.atan2(dn, de)
        return abs(_wrap_pi(bearing - float(yaw))) <= WEDGE_HALF_RAD

    def mark_wedge(self, east, north, yaw, z, now) -> int:
        """Count cells the forward wedge is covering. Out of the depth band, the look timer resets."""
        in_band = DEPTH_LO <= float(z) <= DEPTH_HI
        looking = set()
        if in_band:
            for cell in self.cells:
                if self.hits.get(cell, 0) >= SEARCH_HITS:
                    continue
                ce, cn = self.center(cell)
                if not self.in_wedge(east, north, yaw, ce, cn):
                    continue
                looking.add(cell)
                started = self.since.get(cell)
                if started is None:
                    self.since[cell] = float(now)
                    if self.hits.get(cell, 0) < 1:
                        self.hits[cell] = 1
                elif float(now) - started >= LOOK_S:
                    self.hits[cell] = SEARCH_HITS
        for cell in list(self.since):
            if cell not in looking:
                self.since.pop(cell, None)
        return len(looking)

    def nearest_open(self, east, north, yaw, skip=(), min_dist: float = 0.0):
        """Nearest open cell centre. Distance is bucketed by 2 m, then the smallest turn wins.

        Cells closer than min_dist are skipped: the camera cannot cover a centre inside the
        near edge of the wedge, so those are looked at from farther out.
        """
        skipped = set(skip)
        best = None
        best_key = None
        floor = float(min_dist)
        for cell in self.cells:
            if self.hits.get(cell, 0) >= SEARCH_HITS:
                continue
            if cell in skipped:
                continue
            ce, cn = self.center(cell)
            de, dn = ce - float(east), cn - float(north)
            dist = math.hypot(de, dn)
            if dist < floor:
                continue
            bearing = math.atan2(dn, de) if dist > 0.05 else float(yaw)
            off = abs(_wrap_pi(bearing - float(yaw)))
            key = (int(dist / 2.0), off, cell[1], cell[0])
            if best_key is None or key < best_key:
                best_key = key
                best = (ce, cn, cell, dist)
        return best

    def done_count(self) -> int:
        return sum(1 for c in self.cells if self.hits.get(c, 0) >= SEARCH_HITS)

    def open_count(self) -> int:
        return len(self.cells) - self.done_count()


def look_leg(east, north, cell_e, cell_n, stand: float = VIEW_STAND_M, near: float = VIEW_NEAR_M):
    """Swim points that put a cell centre in the forward wedge.

    From outside the wedge: stop short of the cell, then creep in.
    Already in the wedge: creep toward the inner edge.
    On top of the cell: back out to the stand-off, then creep in facing it.
    """
    de = float(cell_e) - float(east)
    dn = float(cell_n) - float(north)
    dist = math.hypot(de, dn)
    if dist < 0.4:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = de / dist, dn / dist
    view = (float(cell_e) - ux * stand, float(cell_n) - uy * stand)
    close = (float(cell_e) - ux * near, float(cell_n) - uy * near)
    if dist > stand + 0.8:
        return [view, close]
    if dist > near + 0.4:
        return [close]
    # Too close to see the centre. Back out to the stand, then creep in facing it.
    return [view, close]
