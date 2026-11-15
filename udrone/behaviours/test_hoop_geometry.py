"""Geometry of the Kiwanis hoop: an orange ellipse, and a line through the opening."""
import math

from hoop_geom import (
    HOOP_CENTER,
    RING_OUTER_M,
    SEARCH_HITS,
    CoverageMap,
    horizontal_axis,
    in_opening,
    level_feature,
    look_leg,
    orange_ring,
    ring_center,
    sweep_points,
    through_ends,
)


def main():
    # Vehicle west of the ring: the pass starts further west and finishes east.
    start, end = through_ends((2.0, 0.2, -0.4), HOOP_CENTER, lead_m=2.4, exit_m=2.0)
    assert start[0] < HOOP_CENTER[0] < end[0], (start, end)
    assert abs(start[0] - (HOOP_CENTER[0] - 2.4)) < 1e-9
    assert abs(end[0] - (HOOP_CENTER[0] + 2.0)) < 1e-9
    assert abs(start[1] - HOOP_CENTER[1]) < 1e-9 and abs(start[2] - HOOP_CENTER[2]) < 1e-9

    # The centre is in the opening; a point out at the rim is not.
    assert in_opening(HOOP_CENTER, HOOP_CENTER)
    assert in_opening((6.1, 0.1, -0.95), HOOP_CENTER)
    assert not in_opening((6.0, 0.45, -1.0), HOOP_CENTER)
    assert not in_opening((4.0, 0.0, -1.0), HOOP_CENTER)

    # Level hull at the origin, facing east. The major axis is the outer diameter.
    fx = fy = 400.0
    major = fx * RING_OUTER_M / 4.0
    enu, z_opt = ring_center(
        320, 240, major, fx, fy, 320, 240,
        (0.0, 0.0, -0.2), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    )
    assert abs(z_opt - 4.0) < 1e-6, z_opt
    assert abs(enu[0] - 4.185) < 1e-6, enu
    assert abs(enu[1]) < 1e-6 and abs(enu[2] + 0.2) < 1e-6, enu

    # Image right is hull -Y, so a pixel shift moves the centre south.
    enu_r, _z = ring_center(
        360, 240, major, fx, fy, 320, 240,
        (0.0, 0.0, -0.2), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    )
    assert enu_r[1] < -0.3, enu_r
    assert horizontal_axis((0.0, 1.0, -0.4)) == (0.0, 1.0, 0.0)

    # West of the ring, the sweep moves forward the whole way and exits past it.
    sweep, axis = sweep_points((2.0, 0.5, -0.8), (6.0, 0.0, -1.0))
    along = [(p[0] - 2.0) * axis[0] + (p[1] - 0.5) * axis[1] for p in sweep]
    assert all(along[i] <= along[i + 1] + 1e-9 for i in range(len(along) - 1)), along
    assert sweep[-1][0] > 6.0 and sweep[2][0] < 6.0
    assert abs(sweep[1][2] - (-0.8)) < abs(-1.0 - (-0.8))

    # rpy pitch is positive nose-down, which puts a same-depth target above
    # the optical axis. Levelling brings that vertical error back to zero.
    nose_down = 0.40
    _xl, yl = level_feature(0.05, math.tan(-nose_down), nose_down)
    assert abs(yl) < 1e-6, yl
    assert abs(_xl - 0.05) < 1e-9

    # Facing east at hoop depth, the wedge marks cells 1.5–6 m ahead and ignores the bed.
    grid = CoverageMap(0.0, 0.0)
    hoop_cell = (3, 0)  # east [6, 8), the cell that contains the hoop's east edge
    assert hoop_cell in grid.cells, len(grid.cells)
    grid.mark_wedge(2.0, 0.0, 0.0, -0.2, 0.0)
    assert grid.done_count() == 0 and all(v == 0 for v in grid.hits.values())
    grid.mark_wedge(2.0, 0.0, 0.0, -1.0, 0.0)
    assert grid.hits.get(hoop_cell) == 1, grid.hits
    assert grid.hits.get((2, 0)) == 1  # centre (5, 1), 3 m ahead
    assert grid.hits.get((1, 0), 0) == 0  # centre (3, 1) is inside the near cutoff
    grid.mark_wedge(2.0, 0.0, 0.0, -2.2, 1.0)  # a bed view drops the look timer
    grid.mark_wedge(2.0, 0.0, 0.0, -1.0, 1.2)
    assert grid.hits.get(hoop_cell) == 1
    grid.mark_wedge(2.0, 0.0, 0.0, -1.0, 1.2 + 1.5)
    assert grid.hits.get(hoop_cell) == SEARCH_HITS

    nxt = grid.nearest_open(2.0, 0.0, 0.0)
    assert nxt is not None and nxt[2] != hoop_cell

    # From the dock, the leg toward that cell centre approaches it from the west.
    far = look_leg(0.0, 0.0, 7.0, 1.0)
    assert len(far) == 2
    assert math.hypot(far[0][0] - 7.0, far[0][1] - 1.0) > math.hypot(far[1][0] - 7.0, far[1][1] - 1.0)
    close = look_leg(4.0, 1.0, 7.0, 1.0)
    assert len(close) == 1 and close[0][0] > 4.0
    # Sitting on a cell: back out, then creep in so the camera faces the centre.
    backed = look_leg(6.2, 1.0, 7.0, 1.0)
    assert len(backed) == 2
    assert math.hypot(backed[0][0] - 7.0, backed[0][1] - 1.0) > math.hypot(backed[1][0] - 7.0, backed[1][1] - 1.0)
    far_cell = grid.nearest_open(2.0, 0.0, 0.0, min_dist=1.5)
    assert far_cell is not None and far_cell[3] >= 1.5

    _orange_ellipse()
    print("hoop geometry ok")


def _orange_ellipse():
    import cv2
    import numpy as np

    img = np.zeros((480, 640, 3), np.uint8)
    # BGR orange, outer axes 126 x 100 px. fitEllipse reports the full axes.
    cv2.ellipse(img, (320, 240), (63, 50), 0, 0, 360, (20, 90, 230), thickness=6)
    hit = orange_ring(img)
    assert hit is not None, "orange ellipse was not found"
    assert abs(hit["u"] - 320) < 8 and abs(hit["v"] - 240) < 8, hit
    assert 110 < hit["major"] < 150, hit
    assert hit["aspect"] > 0.6, hit
    assert hit["closed"], hit

    # A limb without the rest of the ring is still a bearing, not a lock.
    limb = np.zeros((480, 640, 3), np.uint8)
    cv2.line(limb, (200, 240), (360, 240), (20, 90, 230), thickness=6)
    partial = orange_ring(limb)
    assert partial is not None and not partial["closed"], partial

    sand = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(sand, (320, 240), 40, (140, 180, 200), thickness=-1)
    assert orange_ring(sand) is None

    speck = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(speck, (100, 100), 4, (20, 90, 230), thickness=-1)
    assert orange_ring(speck) is None


if __name__ == "__main__":
    main()
