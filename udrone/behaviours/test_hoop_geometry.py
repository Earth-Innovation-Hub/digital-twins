"""Geometry of the Kiwanis hoop: an orange ellipse, and a line through the opening."""
import math

from hoop_geom import (
    HOOP_CENTER,
    RING_OUTER_M,
    SEARCH_HITS,
    SEARCH_Z,
    DEPTH_LO,
    CoverageMap,
    debug_caption,
    horizontal_axis,
    in_opening,
    level_feature,
    localizer_shift,
    fly_trajectory,
    look_leg,
    look_step,
    orange_ring,
    ring_center,
    slew_toward,
    sideways_baseline,
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
    z = HOOP_CENTER[2]
    assert in_opening((6.1, 0.1, z + 0.05), HOOP_CENTER)
    assert not in_opening((6.0, 0.45, z), HOOP_CENTER)
    assert not in_opening((4.0, 0.0, z), HOOP_CENTER)

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
    # The rollout, the opening, and the exit are one level line. The descent
    # is mostly done on the intercept leg and has not overshot the ring depth.
    assert abs(sweep[2][2] - sweep[3][2]) < 1e-9 and abs(sweep[3][2] - sweep[4][2]) < 1e-9
    assert abs(sweep[1][2] - sweep[2][2]) < abs(sweep[1][2] - (-0.8))
    leg_out = (sweep[4][0] - sweep[2][0], sweep[4][1] - sweep[2][1])
    leg_mid = (sweep[3][0] - sweep[2][0], sweep[3][1] - sweep[2][1])
    assert abs(leg_out[0] * leg_mid[1] - leg_out[1] * leg_mid[0]) < 1e-6

    # Lined up on the ring: the planned pass is straight, level after the join,
    # and finishes past the opening. A nose pointed off the line still joins it.
    planned, pax = fly_trajectory((2.0, 0.0, -0.8), (1.0, 0.0), (6.0, 0.0, -1.0), through=True)
    assert math.dist(planned[0], (2.0, 0.0, -0.8)) < 1e-6
    assert planned[-1][0] > 6.0 and abs(planned[-1][1]) < 0.05
    assert abs(planned[-1][2] - planned[-2][2]) < 1e-9
    palong = [(p[0] - 2.0) * pax[0] + p[1] * pax[1] for p in planned]
    assert palong[-1] > palong[0] + 4.0
    assert min(palong) > -0.4
    turned, tax = fly_trajectory((2.0, 1.5, -0.8), (0.0, 1.0), (6.0, 0.0, -1.0), through=True)
    assert turned[-1][0] > 6.0
    assert any(math.hypot(p[0] - 6.0, p[1]) < 0.2 for p in turned)
    talong = [(p[0] - 2.0) * tax[0] + (p[1] - 1.5) * tax[1] for p in turned]
    assert min(talong) > -0.5 and talong[-1] > 4.0
    hold, _hax = fly_trajectory((2.0, 0.4, -0.8), (1.0, 0.0), (6.0, 0.0, -1.0), through=False)
    assert hold[-1][0] < 6.0
    # The pass flies at the ring's own depth, also for the deep Kiwanis hoop.
    deep, _dax = fly_trajectory((2.0, 0.0, -4.0), (1.0, 0.0), (6.0, 0.0, SEARCH_Z), through=True)
    assert abs(deep[-1][2] - SEARCH_Z) < 1e-9 and abs(deep[-2][2] - SEARCH_Z) < 1e-9
    dsweep, _ = sweep_points((2.0, 0.0, -4.0), (6.0, 0.0, SEARCH_Z))
    assert abs(dsweep[3][2] - SEARCH_Z) < 1e-9
    cap = debug_caption("search", "lead", 20, 3.4, (6.0, 0.1, -1.0), 0.55, 9.0, False, False)
    assert cap[0].startswith("search lead") and "3.4" in cap[0] and "6.0" in cap[1]
    assert len(cap) == 2
    passed = debug_caption("through", None, 40, 1.2, (6.0, 0.0, -1.0), 0.5, 0.8, True, False)
    assert "surround 1" in passed[-1] and "opening 0" in passed[-1]
    step, _sax = look_step((2.0, 0.0, -0.8), (6.0, 0.0, -1.0))
    assert math.dist(step[0], (2.0, 0.0, -0.8)) < 1e-6
    assert 1.0 < math.dist(step[0], step[-1]) < 1.8
    assert abs(step[-1][2] - (-0.8)) < 1e-9

    # A ring just right of the axis slides the line that way. A large error
    # means the nose is still turning, so the line is held. Inside 4 m it freezes.
    shift, freeze = localizer_shift(0.10, 0.0, 6.0, (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert shift is not None and shift[1] < -0.15 and not freeze, shift
    held, still = localizer_shift(0.50, 0.0, 6.0, (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert held is None and not still
    frozen, locked = localizer_shift(0.05, 0.0, 3.0, (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert frozen is None and locked
    stepped = slew_toward((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.08, 0.1)
    assert abs(stepped[0] - 0.008) < 1e-9

    # rpy pitch is positive nose-down, which puts a same-depth target above
    # the optical axis. Levelling brings that vertical error back to zero.
    nose_down = 0.40
    _xl, yl = level_feature(0.05, math.tan(-nose_down), nose_down)
    assert abs(yl) < 1e-6, yl
    assert abs(_xl - 0.05) < 1e-9

    # Facing east at hoop depth, the wedge marks cells 1.5–6 m ahead and ignores the bed.
    # The behaviour default cell is 2.07 m. This fixture stays on a 2 m grid so the
    # cell indexes below keep their meaning.
    grid = CoverageMap(0.0, 0.0, res=2.0)
    hoop_cell = (3, 0)  # east [6, 8), the cell that contains the hoop's east edge
    assert hoop_cell in grid.cells, len(grid.cells)
    grid.mark_wedge(2.0, 0.0, 0.0, -0.2, 0.0)
    assert grid.done_count() == 0 and all(v == 0 for v in grid.hits.values())
    grid.mark_wedge(2.0, 0.0, 0.0, SEARCH_Z, 0.0)
    assert grid.hits.get(hoop_cell) == 1, grid.hits
    assert grid.hits.get((2, 0)) == 1  # centre (5, 1), 3 m ahead
    assert grid.hits.get((3, 1)) == 1  # about 31° off the nose, inside the camera view
    assert grid.hits.get((1, 0), 0) == 0  # centre (3, 1) is inside the near cutoff
    grid.mark_wedge(2.0, 0.0, 0.0, DEPTH_LO - 0.8, 1.0)  # below the band drops the look timer
    grid.mark_wedge(2.0, 0.0, 0.0, SEARCH_Z, 1.2)
    assert grid.hits.get(hoop_cell) == 1
    grid.mark_wedge(2.0, 0.0, 0.0, SEARCH_Z, 1.2 + 1.5)
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
    assert not sideways_baseline([0.0, 0.2])
    assert sideways_baseline([0.0, 0.2, 0.55])

    # A tilted ring's major axis is the ellipse, which is shorter than the upright box.
    tilted = np.zeros((480, 640, 3), np.uint8)
    cv2.ellipse(tilted, (320, 240), (80, 25), 55, 0, 360, (20, 90, 230), thickness=5)
    spun = orange_ring(tilted)
    assert spun is not None, spun
    assert abs(spun["major"] - 160) < 30, spun
    assert spun["major"] > max(spun["w"], spun["h"]), spun

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
