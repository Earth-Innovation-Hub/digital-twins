"""The HUD projects the world the same way the servos read the cameras."""
import math

from hud_overlay import RED, CAMERAS, TaskHud, default_intrinsics, map_pixel, map_scale, project
from servo_geom import nadir_body
from track_geom import CAM_Z, up_camera_tag_body

LEVEL = (1.0, 0.0, 0.0, 0.0)


def yaw_q(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def main():
    assert abs(default_intrinsics("front")[0] - 381.46) < 0.1
    intr = (100.0, 100.0, 320.0, 240.0)
    pos = (2.0, 3.0, -1.0)

    # Front: straight ahead is the centre, left of the hull is image left, up is image up.
    ahead = (pos[0] + 5.0, pos[1], pos[2])
    assert close(project("front", pos, LEVEL, ahead, intr), (320.0, 240.0))
    u, v = project("front", pos, LEVEL, (pos[0] + 5.0, pos[1] + 1.0, pos[2] + 1.0), intr)
    assert u < 320.0 and v < 240.0
    assert project("front", pos, LEVEL, (pos[0] - 5.0, pos[1], pos[2]), intr) is None

    # Up: a tag the up camera reads at (x_n, y_n, z) projects back to the same pixel.
    for x_n, y_n, z in ((0.1, -0.2, 1.5), (-0.3, 0.25, 0.8)):
        b = up_camera_tag_body(x_n, y_n, z, CAM_Z)
        for yaw in (0.0, 1.1):
            q = yaw_q(yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            world = (pos[0] + c * b[0] - s * b[1], pos[1] + s * b[0] + c * b[1], pos[2] + b[2])
            pix = project("up", pos, q, world, intr)
            assert close(pix, (320.0 + 100.0 * x_n, 240.0 + 100.0 * y_n), 1e-6), (pix, x_n, y_n)
    assert project("up", pos, LEVEL, (pos[0], pos[1], pos[2] - 2.0), intr) is None

    # Down: the same round trip through nadir_body, from the down camera mount.
    off = CAMERAS["down"]["offset"]
    for x_n, y_n, z in ((0.2, 0.1, 1.2), (-0.15, -0.3, 2.0)):
        b = nadir_body(x_n, y_n, z)
        world = (pos[0] + off[0] + b[0], pos[1] + off[1] + b[1], pos[2] + off[2] + b[2])
        pix = project("down", pos, LEVEL, world, intr)
        assert close(pix, (320.0 + 100.0 * x_n, 240.0 + 100.0 * y_n), 1e-6), (pix, x_n, y_n)

    # Map: north-up, east-right, the farthest point fits inside the frame.
    centre = (0.0, 0.0)
    scale = map_scale(centre, [(10.0, 0.0, 0.0)])
    east = map_pixel(centre, (10.0, 0.0), scale)
    north = map_pixel(centre, (0.0, 10.0), scale)
    assert 85.0 < east[0] < 170.0 and abs(east[1] - 85.0) < 1e-9
    assert north[1] < 85.0 and abs(north[0] - 85.0) < 1e-9
    assert map_scale(centre, [(500.0, 0.0, 0.0)]) == map_scale(centre, [(40.0, 0.0, 0.0)])
    test_map_raster()
    test_map_status_lines()
    print("hud overlay ok")


def test_map_status_lines():
    hud = TaskHud.__new__(TaskHud)
    hud._map_status = {
        "map": {"bed_cells": 12, "bed_area_m2": 3.0, "obstacle_cells": 2, "cover_cells": 4},
        "plan": {"target": "bed", "waypoints": 8, "length_m": 40.0, "pct": 25.0},
        "graph": {"enabled": True, "keyframes": 5, "landmarks": [{"id": 0}],
                  "factors": {"odom": 4, "tag": 1}, "sigma_m": 0.2, "err_m": 0.1, "odom_err_m": 0.4},
    }
    lines = hud._map_status_lines()
    assert any("bed 12" in s for s in lines)
    assert any("plan bed 8 wps" in s for s in lines)
    assert any("graph kf 5" in s and "lm 1" in s for s in lines)


def test_map_raster():
    # OccupancyGrid rows are north (y), columns east (x); the raster is north-up.
    import cv2
    import numpy as np

    hud = TaskHud.__new__(TaskHud)
    hud._np, hud._cv2, hud._maps = np, cv2, {}
    occ = np.zeros((20, 20), np.int8)
    occ[15, 13] = 100                      # cell x 3..4, y 5..6
    dep = np.full((20, 20), -1, np.int8)
    dep[10, 10] = 50                       # cell x 0..1, y 0..1, 5 m deep
    hud._maps["grid"] = (occ, 1.0, -10.0, -10.0)
    hud._maps["bed_depth"] = (dep, 1.0, -10.0, -10.0)
    raster, seen = hud._map_raster((0.0, 0.0), 4.0, 170)
    assert tuple(raster[63, 97]) == RED
    assert seen[85, 84] and tuple(raster[85, 84]) == (60, 60, 60)
    assert tuple(raster[83, 87]) != (60, 60, 60)
    assert not seen[85, 145]


if __name__ == "__main__":
    main()
