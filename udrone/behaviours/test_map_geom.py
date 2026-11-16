"""Sensor geometry, grid layers, and message packing for the xudrone map."""
import math
import os
import tempfile

import numpy as np

import map_geom as mg
from hud_overlay import body_axes

LEVEL = (1.0, 0.0, 0.0, 0.0)


def yaw_q(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def close(a, b, tol=1e-6):
    return np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol)


def main():
    # rot() agrees with the HUD's body axes.
    q = (0.9, 0.1, -0.2, 0.3)
    n = math.sqrt(sum(v * v for v in q))
    q = tuple(v / n for v in q)
    r = mg.rot(q)
    for col, axis in zip(range(3), body_axes(q)):
        assert close(r[:, col], axis), (col, r[:, col], axis)

    # Altimeter straight down from the mount; yaw does not move it.
    hit = mg.beam_hit((1.0, 2.0, -1.0), yaw_q(0.7), mg.ALTIMETER, 3.0)
    assert close(hit, (1.0, 2.0, -1.0 - 0.12 - 3.0))
    assert mg.beam_hit((0, 0, 0), LEVEL, mg.ALTIMETER, float("inf")) is None

    # A sonar return dead ahead is pitched 10 degrees down; yaw turns it.
    p = mg.sonar_to_world(np.array([[10.0, 0.0, 0.0]]), (0.0, 0.0, -2.0), yaw_q(math.pi / 2))[0]
    c, s = math.cos(mg.SONAR_PITCH), math.sin(mg.SONAR_PITCH)
    assert close(p, (0.0, 0.20 + 10.0 * c, -2.0 - 0.04 - 10.0 * s)), p

    # PointCloud2 parsing: 16-byte points, non-finite rows dropped.
    pts = np.array([[1, 2, 3, 9], [np.inf, 0, 0, 9], [4, 5, 6, 9]], np.float32)
    got = mg.cloud_xyz(pts.tobytes(), 16, 3)
    assert close(got, [[1, 2, 3], [4, 5, 6]])
    assert close(np.frombuffer(mg.pack_xyz(got), np.float32).reshape(-1, 3), got)

    g = mg.Grid(res=1.0, half=10.0)
    assert g.n == 20
    ix, iy, ok = g.index(0.2, -0.2)
    assert ok and (ix, iy) == (10, 9)
    assert not g.index(10.5, 0.0)[2]

    # Bed from the altimeter; a return well above it is an obstacle.
    g.add_bed((3.2, 4.7, -6.0))
    g.add_bed((3.4, 4.1, -6.2))
    bed = g.bed_z()
    bx, by, _ = g.index(3.2, 4.7)
    assert abs(bed[bx, by] + 6.1) < 1e-6 and np.isnan(bed[0, 0])
    tall = g.add_sonar(np.array([[3.3, 4.5, -4.0]] * 3 + [[3.3, 4.5, -6.1]]))
    assert tall == 3 and g.occupancy()[bx, by] == 100
    # Returns near the boat hull are the boat, not the map.
    assert g.add_sonar(np.array([[0.5, 0.5, -0.3]] * 5), boat_xy=(0.0, 0.0)) == 0
    assert g.son_n[g.index(0.5, 0.5)[:2]] == 0
    # Sonar-only bed after two returns.
    g.add_sonar(np.array([[-4.5, -4.5, -8.0], [-4.2, -4.7, -7.9]]))
    sx, sy, _ = g.index(-4.5, -4.5)
    assert abs(g.bed_z()[sx, sy] + 8.0) < 1e-6
    assert g.depth_dm()[sx, sy] == 80 and g.depth_dm()[0, 19] == -1

    # Visits and the camera footprint.
    g.add_visit((-9.5, 9.5, -1.0))
    assert g.visited[0, 19] == 1 and g.occupancy()[0, 19] == 0
    g.add_cover((0.0, 0.0, -1.0), 2.0)
    radius = 2.0 * math.tan(mg.DOWN_HFOV / 2.0)
    assert g.cover.sum() > 0
    xs, ys = np.nonzero(g.cover)
    cx, cy = g.center(xs, ys)
    assert np.all(np.hypot(cx, cy) <= radius + 1e-9)
    before = g.cover.sum()
    g.add_cover((0.0, 0.0, -1.0), 20.0)
    assert g.cover.sum() == before

    # OccupancyGrid: row = y (north), column = x (east), origin at the SW corner.
    a = np.zeros((20, 20), np.int8)
    a[19, 0] = 7          # far east, far south
    a[0, 19] = 9          # far west, far north
    flat = mg.occupancy_data(a)
    assert flat[0 * 20 + 19] == 7 and flat[19 * 20 + 0] == 9

    # grid_map: index (0, 0) is the +x +y corner, column-major.
    b = np.arange(20 * 20, dtype=np.float32).reshape(20, 20)
    d = mg.gridmap_data(b)
    assert d[0] == b[19, 19] and d[1] == b[18, 19] and d[20] == b[19, 18]

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "m.npz")
        g.save(path)
        h = mg.Grid.load(path)
        for k in mg.Grid.LAYERS:
            assert np.array_equal(getattr(g, k), getattr(h, k)), k
    x0, y0, x1, y1 = g.known_box(margin_m=1.0, min_half_m=2.0)
    assert x0 < -9 and x1 > 3 and y1 > 9 and abs((x1 - x0) - (y1 - y0)) < 1e-9
    # Costmap: obstacle lethal, free bed zero, unknown -1.
    cost = mg.costmap(g, z=-1.0, boat_xy=(0.0, 0.0), unknown=-1)
    assert cost[bx, by] == mg.COST_LETHAL
    assert cost.dtype == np.int8
    print("map geometry ok")


if __name__ == "__main__":
    main()
