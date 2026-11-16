"""xudrone global 2D map: grid layers, sensor geometry, and message packing. ROS-free.

The grid is ENU (x east, y north) at the lake origin. Arrays are [ix, iy]
with ix growing east from x_min and iy growing north from y_min.

Layers kept per cell:
  bed_sum, bed_n   altimeter returns (bed elevation = sum / n)
  son_min, son_n   lowest sonar return, and how many returns landed here
  obs_n            sonar returns standing OBST_H_M or more above the bed
  cover            down-camera footprint passes
  visited          hull passes
"""
from __future__ import annotations

import math

import numpy as np

RES_M = 0.5
HALF_M = 160.0
OBST_H_M = 0.6          # a return this far above the bed is an obstacle
OBST_MIN_N = 3          # returns before a cell counts as occupied
SON_BED_MIN_N = 2       # sonar-only bed needs this many returns
DOWN_HFOV = 1.2         # down camera, rad
COVER_MAX_ALT_M = 8.0   # beyond this the down camera sees nothing useful
BOAT_CLEAR_M = 1.3      # returns this close to otter_k's hull are the boat

# Sensor mounts on base_link (model/xudrone/model.sdf): origin and unit beam.
ALTIMETER = ((0.0, 0.0, -0.12), (0.0, 0.0, -1.0))
SURFACE = ((0.0, 0.0, 0.12), (0.0, 0.0, 1.0))
SONAR_ORIGIN = (0.20, 0.0, -0.04)
SONAR_PITCH = math.radians(10.0)    # nose-down tilt of the fan


def rot(q):
    """3x3 body-to-world rotation for q = (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q[:4])
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def sonar_rot():
    """Sonar frame to body: pitch SONAR_PITCH about body y (positive is nose down)."""
    c, s = math.cos(SONAR_PITCH), math.sin(SONAR_PITCH)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def beam_hit(pos, q, mount, rng):
    """World point a single-beam ranger hit at `rng` metres, or None."""
    if rng is None or not math.isfinite(rng):
        return None
    r = rot(q)
    origin, beam = np.asarray(mount[0]), np.asarray(mount[1])
    return np.asarray(pos, float) + r @ (origin + float(rng) * beam)


def sonar_to_world(points, pos, q):
    """Nx3 sonar-frame points to world."""
    r = rot(q)
    body = points @ sonar_rot().T + np.asarray(SONAR_ORIGIN)
    return body @ r.T + np.asarray(pos, float)


def cloud_xyz(data: bytes, point_step: int, n: int, offsets=(0, 4, 8)):
    """float32 x, y, z from a PointCloud2 buffer, finite rows only."""
    if n <= 0 or point_step <= 0:
        return np.zeros((0, 3), np.float32)
    raw = np.frombuffer(data, dtype=np.uint8, count=n * point_step).reshape(n, point_step)
    cols = [raw[:, o:o + 4].copy().view(np.float32).reshape(n) for o in offsets]
    pts = np.stack(cols, axis=1)
    return pts[np.all(np.isfinite(pts), axis=1)]


def pack_xyz(points) -> bytes:
    return np.ascontiguousarray(points, dtype=np.float32).tobytes()


class Grid:
    def __init__(self, res=RES_M, half=HALF_M):
        self.res = float(res)
        self.half = float(half)
        self.n = int(round(2 * half / res))
        self.x_min = self.y_min = -self.half
        shape = (self.n, self.n)
        self.bed_sum = np.zeros(shape, np.float64)
        self.bed_n = np.zeros(shape, np.int32)
        self.son_min = np.full(shape, np.inf, np.float32)
        self.son_n = np.zeros(shape, np.int32)
        self.obs_n = np.zeros(shape, np.int32)
        self.cover = np.zeros(shape, np.int32)
        self.visited = np.zeros(shape, np.int32)

    # ---- indexing ------------------------------------------------------
    def index(self, x, y):
        ix = np.floor((np.asarray(x, float) - self.x_min) / self.res).astype(int)
        iy = np.floor((np.asarray(y, float) - self.y_min) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return ix, iy, ok

    def center(self, ix, iy):
        return (self.x_min + (ix + 0.5) * self.res, self.y_min + (iy + 0.5) * self.res)

    # ---- updates -------------------------------------------------------
    def add_bed(self, p):
        ix, iy, ok = self.index(p[0], p[1])
        if ok:
            self.bed_sum[ix, iy] += float(p[2])
            self.bed_n[ix, iy] += 1

    def bed_z(self):
        """Bed elevation: altimeter mean, else the lowest sonar return. NaN unknown."""
        out = np.full(self.bed_n.shape, np.nan, np.float32)
        son = self.son_n >= SON_BED_MIN_N
        out[son] = self.son_min[son]
        alt = self.bed_n > 0
        out[alt] = (self.bed_sum[alt] / self.bed_n[alt]).astype(np.float32)
        return out

    def add_sonar(self, pts, boat_xy=None):
        """World sonar returns: lowest per cell, and obstacles above the known bed."""
        if len(pts) == 0:
            return 0
        if boat_xy is not None:
            far = np.hypot(pts[:, 0] - boat_xy[0], pts[:, 1] - boat_xy[1]) > BOAT_CLEAR_M
            pts = pts[far | (pts[:, 2] < -1.0)]
        ix, iy, ok = self.index(pts[:, 0], pts[:, 1])
        ix, iy, z = ix[ok], iy[ok], pts[ok, 2].astype(np.float32)
        np.minimum.at(self.son_min, (ix, iy), z)
        np.add.at(self.son_n, (ix, iy), 1)
        alt_n = self.bed_n[ix, iy]
        bed = np.where(
            alt_n > 0,
            self.bed_sum[ix, iy] / np.maximum(alt_n, 1),
            np.where(self.son_n[ix, iy] >= SON_BED_MIN_N, self.son_min[ix, iy], np.nan),
        )
        tall = np.isfinite(bed) & (z > bed + OBST_H_M)
        np.add.at(self.obs_n, (ix[tall], iy[tall]), 1)
        return int(tall.sum())

    def add_visit(self, pos):
        ix, iy, ok = self.index(pos[0], pos[1])
        if ok:
            self.visited[ix, iy] += 1

    def add_cover(self, pos, altitude):
        """Down-camera footprint: a disc of altitude * tan(hfov / 2)."""
        if altitude is None or not (0.2 < altitude <= COVER_MAX_ALT_M):
            return
        r = altitude * math.tan(DOWN_HFOV / 2.0)
        k = int(math.ceil(r / self.res))
        cx, cy, ok = self.index(pos[0], pos[1])
        if not ok:
            return
        xs = np.arange(max(0, cx - k), min(self.n, cx + k + 1))
        ys = np.arange(max(0, cy - k), min(self.n, cy + k + 1))
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        px, py = self.center(gx, gy)
        inside = np.hypot(px - pos[0], py - pos[1]) <= r
        self.cover[gx[inside], gy[inside]] += 1

    # ---- products ------------------------------------------------------
    def occupancy(self):
        """nav_msgs/OccupancyGrid values [ix, iy]: 100 obstacle, 0 seen, -1 unknown."""
        occ = np.full(self.bed_n.shape, -1, np.int8)
        seen = (self.bed_n > 0) | (self.son_n > 0) | (self.cover > 0) | (self.visited > 0)
        occ[seen] = 0
        occ[self.obs_n >= OBST_MIN_N] = 100
        return occ

    def depth_dm(self):
        """Bed depth below the surface in decimetres (0-100), -1 unknown."""
        bed = self.bed_z()
        out = np.full(bed.shape, -1, np.int8)
        ok = np.isfinite(bed)
        out[ok] = np.clip(np.round(-bed[ok] * 10.0), 0, 100).astype(np.int8)
        return out

    def known_box(self, margin_m=10.0, min_half_m=20.0, around=None):
        """(x0, y0, x1, y1) around every seen cell (and `around`), for rendering."""
        seen = (self.bed_n > 0) | (self.son_n > 0) | (self.visited > 0)
        xs, ys = np.nonzero(seen)
        pts = []
        if len(xs):
            pts += [self.center(xs.min(), ys.min()), self.center(xs.max(), ys.max())]
        if around is not None:
            pts.append((around[0], around[1]))
        if not pts:
            return (-min_half_m, -min_half_m, min_half_m, min_half_m)
        x0 = min(p[0] for p in pts) - margin_m
        x1 = max(p[0] for p in pts) + margin_m
        y0 = min(p[1] for p in pts) - margin_m
        y1 = max(p[1] for p in pts) + margin_m
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        h = max(min_half_m, (x1 - x0) / 2.0, (y1 - y0) / 2.0)
        return (cx - h, cy - h, cx + h, cy + h)

    # ---- persistence ---------------------------------------------------
    LAYERS = ("bed_sum", "bed_n", "son_min", "son_n", "obs_n", "cover", "visited")

    def save(self, path):
        np.savez_compressed(path, res=self.res, half=self.half,
                            **{k: getattr(self, k) for k in self.LAYERS})

    @classmethod
    def load(cls, path):
        d = np.load(path)
        g = cls(float(d["res"]), float(d["half"]))
        for k in cls.LAYERS:
            setattr(g, k, d[k])
        return g


# Costmap (Nav2 values: 100 lethal, 99 inscribed, 1-98 inflated, 0 free, -1 unknown).
COST_LETHAL, COST_INSCRIBED = 100, 99
INSCRIBED_M = 0.35       # hull half-length
INFLATE_M = 2.0
INFLATE_K = 2.0          # cost decay per metre beyond the inscribed radius
BED_CLEAR_M = 0.5        # keep this much water under the hull
CRUISE_Z = -1.0          # depth the published costmap is drawn for
UNKNOWN_COST = 30        # what planners pay to cross unknown water


def costmap(grid, z=CRUISE_Z, boat_xy=None, unknown=-1):
    """Nav2-style costmap [ix, iy] for an AUV holding depth `z`.

    Lethal: sonar obstacles, the bed within BED_CLEAR_M of `z`, and otter_k's
    hull. Inflation decays from the inscribed radius out to INFLATE_M. Cells
    nobody has seen get `unknown`.
    """
    from scipy.ndimage import distance_transform_edt

    lethal = grid.obs_n >= OBST_MIN_N
    if z is not None:
        bed = grid.bed_z()
        with np.errstate(invalid="ignore"):
            lethal |= np.isfinite(bed) & (bed > z - BED_CLEAR_M)
    if boat_xy is not None:
        cx, cy, ok = grid.index(boat_xy[0], boat_xy[1])
        if ok:
            k = int(math.ceil(BOAT_CLEAR_M / grid.res))
            xs = slice(max(0, cx - k), min(grid.n, cx + k + 1))
            ys = slice(max(0, cy - k), min(grid.n, cy + k + 1))
            lethal[xs, ys] = True
    cost = np.zeros(lethal.shape, np.int8)
    if lethal.any():
        d = distance_transform_edt(~lethal) * grid.res
        infl = (d > 0) & (d <= INFLATE_M)
        c = np.where(d <= INSCRIBED_M, COST_INSCRIBED,
                     np.round(98.0 * np.exp(-INFLATE_K * (d - INSCRIBED_M))))
        cost[infl] = c[infl].astype(np.int8)
        cost[lethal] = COST_LETHAL
    seen = (grid.bed_n > 0) | (grid.son_n > 0) | (grid.cover > 0) | (grid.visited > 0)
    cost[~seen & (cost == 0)] = unknown
    return cost


def occupancy_data(arr):
    """[ix, iy] array to OccupancyGrid row-major data (row = y, origin at x_min, y_min)."""
    return np.ascontiguousarray(arr.T).reshape(-1)


def gridmap_data(arr):
    """[ix, iy] array to grid_map's Float32MultiArray order.

    grid_map index (0, 0) is the +x, +y corner, and the matrix is stored
    column-major, so the array is flipped on both axes and written in
    Fortran order.
    """
    return arr[::-1, ::-1].reshape(-1, order="F").astype(np.float32)
