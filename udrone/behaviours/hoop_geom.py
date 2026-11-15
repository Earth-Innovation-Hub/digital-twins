"""Kiwanis hula hoop geometry. ROS-free.

The ring is vertical: a 1 m opening in the north-up plane, centred at ENU
(6, 0, -1), so the axis runs east-west. The flight does not use that
coordinate. It fits an ellipse to the orange tube in the front camera and
back-projects the centre with the pinhole. The major axis is the 1.09 m
outer diameter at any viewing angle.
"""
from __future__ import annotations

import math

HOOP_CENTER = (6.0, 0.0, -1.0)  # authored ring, east, north, up
HOOP_AXIS = (1.0, 0.0, 0.0)     # +east through the opening
HOOP_RADIUS = 0.50              # 1 m diameter, tube centreline
RING_OUTER_M = 1.09             # outer diameter: 1.0 m plus the 0.045 m tube
INNER_PASS_M = 0.40             # inside the opening, clear of the tube
CAM_FORWARD_M = 0.185           # front camera, metres ahead of the hull origin


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
    """A forward sweep onto the line through the ring, then straight through.

    Depth changes over the first half of the lead-in, so the hull is not
    asked to pitch onto the ring depth in one step. The standoff stays in
    front of the vehicle. Along-track distance increases the whole way.
    """
    de = float(center[0]) - float(vehicle[0])
    dn = float(center[1]) - float(vehicle[1])
    dist = math.hypot(de, dn)
    if dist < 0.4:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = de / dist, dn / dist
    lead = min(float(lead_m), max(0.6, dist * 0.55))
    z = min(-0.65, max(-1.35, float(center[2])))
    stand = (float(center[0]) - ux * lead, float(center[1]) - uy * lead, z)
    ease = (
        float(vehicle[0]) + 0.45 * (stand[0] - float(vehicle[0])),
        float(vehicle[1]) + 0.45 * (stand[1] - float(vehicle[1])),
        float(vehicle[2]) + 0.35 * (z - float(vehicle[2])),
    )
    opening = (float(center[0]), float(center[1]), z)
    exit_pt = (float(center[0]) + ux * exit_m, float(center[1]) + uy * exit_m, z)
    start = (float(vehicle[0]), float(vehicle[1]), float(vehicle[2]))
    return [start, ease, stand, opening, exit_pt], (ux, uy, 0.0)


def horizontal_axis(fwd):
    """Camera forward flattened into the horizontal plane, as a unit vector."""
    ax = (float(fwd[0]), float(fwd[1]), 0.0)
    n = math.hypot(ax[0], ax[1])
    if n < 0.2:
        return (1.0, 0.0, 0.0)
    return (ax[0] / n, ax[1] / n, 0.0)


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

    The measurement is the bounding box of the orange pixels, which is the
    pair of tangent limbs: width and height stay valid when the tube is
    broken into segments. ``closed`` is true when those pixels fill a thin
    ring rather than an arc or a solid blob.

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
        # A blob cut by the frame edge is a sliver of the bed or a pontoon,
        # and its size is not the ring diameter.
        clipped = x <= 1.0 or y <= 1.0 or x + w >= width - 1.0 or y + h >= height - 1.0
        if clipped and major < 160.0:
            continue
        ell_area = math.pi * (major / 2.0) * (minor / 2.0)
        filled = area / max(ell_area, 1.0)
        aspect = minor / major
        # A tube across the box fills about a third of the disk. An arc is
        # thinner in that ratio or too flat to be the whole ring.
        closed = 0.10 <= filled <= 0.70 and aspect >= 0.35 and major >= 24.0
        score = major * (1.5 if closed else 0.6)
        if score > best_score:
            best_score = score
            best = {
                "u": x + w / 2.0,
                "v": y + h / 2.0,
                "w": w,
                "h": h,
                "major": major,
                "minor": minor,
                "aspect": aspect,
                "closed": closed,
            }
    return best


# Coverage grid for the hoop search. World-aligned 2 m cells. A cell is a
# glance at 1 and searched at 2, and only the front-camera wedge at hoop
# depth can raise the count. The box is 20 m on a side around the dock at
# the origin. A vehicle that arms outside it is sent to the nearest open cell.
CELL_M = 2.0
MAP_HALF_M = 10.0
WEDGE_NEAR_M = 1.5
WEDGE_FAR_M = 6.0
WEDGE_HALF_RAD = math.radians(30.0)
DEPTH_LO = -1.40
DEPTH_HI = -0.70
SEARCH_HITS = 2
LOOK_S = 1.5
SEARCH_Z = -1.0
VIEW_STAND_M = 3.5
VIEW_NEAR_M = 2.0


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _columns(lo: float, hi: float, res: float):
    c0 = math.ceil(lo / res - 0.5 - 1e-9)
    c1 = math.floor(hi / res - 0.5 + 1e-9)
    return range(c0, c1 + 1)


class CoverageMap:
    """Hit count per 2 m ENU cell. 0 unseen, 1 glanced, 2 searched."""

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
