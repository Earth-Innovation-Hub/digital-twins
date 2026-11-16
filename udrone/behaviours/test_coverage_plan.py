"""Costmap lanes, A*, and string-pull for map-aware coverage."""
import math

import numpy as np

import coverage_plan as cp
import map_geom as mg


def main():
    g = mg.Grid(res=1.0, half=20.0)
    # Known free corridor: bed along y=0 for x in [-8, 8]; a wall at x=0, y=5.
    for x in range(-8, 9):
        g.add_bed((x + 0.1, 0.1, -3.0))
        g.add_bed((x + 0.1, 2.1, -3.0))
        g.add_bed((x + 0.1, -2.1, -3.0))
    # Bed at the wall cell, then taller returns become a lethal obstacle.
    g.add_bed((0.2, 5.2, -3.0))
    for _ in range(mg.OBST_MIN_N):
        g.add_sonar(np.array([[0.2, 5.2, -1.0]]))
    cost = mg.costmap(g, z=-1.0, unknown=-1)
    wx, wy, _ = g.index(0.2, 5.2)
    assert cost[wx, wy] == mg.COST_LETHAL, cost[wx, wy]
    fx, fy, _ = g.index(0.1, 0.1)
    assert cost[fx, fy] == 0
    # Unknown water carries UNKNOWN_COST when requested.
    cost_u = mg.costmap(g, z=-1.0, unknown=mg.UNKNOWN_COST)
    ux, uy, _ = g.index(15.0, 15.0)
    assert cost_u[ux, uy] == mg.UNKNOWN_COST

    # Straight transit is free; a hop through the wall is not.
    assert cp.line_free(cost, g, (-4.0, 0.0), (4.0, 0.0))
    assert not cp.line_free(cost, g, (0.0, 0.0), (0.0, 8.0))
    path = cp.astar(cost, g, (-3.0, 0.0), (3.0, 0.0))
    assert path and math.dist(path[0], (-3.0, 0.0)) < 1.5
    pulled = cp.string_pull(cost, g, path)
    assert len(pulled) <= len(path)

    # Plan over a 12x12 region: unknown bed is needed, so lanes appear.
    out = cp.plan(g, cost, (0.0, 0.0), width=12.0, length=12.0, heading_deg=90.0,
                  swath=3.0, z=-1.0, start=(-5.0, 0.0), target="bed")
    assert out["waypoints"], out
    assert out["lanes"] >= 1 and out["length_m"] > 0
    # Camera target: everything is uncovered, so a plan still appears.
    cam = cp.plan(g, cost, (0.0, 0.0), 8.0, 8.0, 0.0, 2.0, -1.0, start=(0.0, 0.0),
                  target="camera")
    assert cam["waypoints"]
    print("coverage plan ok")


if __name__ == "__main__":
    main()
