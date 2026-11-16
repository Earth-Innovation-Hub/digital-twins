"""The upward camera maps a centred tag to a point above the hull."""
from track_geom import (
    CAM_Z,
    FACE_M,
    STANDOFF_M,
    BaselineVelocity,
    station_heading,
    station_under,
    up_camera_tag_body,
)


def main():
    body = up_camera_tag_body(0.0, 0.0, 1.0)
    assert abs(body[0]) < 1e-9 and abs(body[1]) < 1e-9
    assert abs(body[2] - (CAM_Z + 1.0)) < 1e-9
    # Image right is hull -Y, image down is hull -X.
    shifted = up_camera_tag_body(0.2, 0.1, 2.0)
    assert abs(shifted[0] - (-0.2)) < 1e-9
    assert abs(shifted[1] - (-0.4)) < 1e-9
    hold = station_under((6.0, 1.0, 0.4), STANDOFF_M, CAM_Z)
    assert abs(hold[0] - 6.0) < 1e-9 and abs(hold[1] - 1.0) < 1e-9
    assert abs(hold[2] - (0.4 - STANDOFF_M - CAM_Z)) < 1e-9

    # Boat velocity over a baseline: repeated snapshots do not halve or
    # double it, and a teleport is held to vmax.
    bv = BaselineVelocity(span_s=0.3, vmax=2.0)
    t, x = 0.0, 0.0
    for i in range(40):
        t += 0.05
        if i % 2 == 0:
            x += 0.05          # 0.5 m/s, but the pose only moves every other tick
        v = bv.update(t, x, 0.0)
    assert abs(v[0] - 0.5) < 0.1 and v[2] == 0.0, v
    v = bv.update(t + 0.3, x + 10.0, 0.0)
    assert abs(v[0] - 2.0) < 1e-9, v

    # Close in, the nose keeps the boat's heading; farther out it is free.
    assert station_heading(FACE_M - 0.1, 0.7) == 0.7
    assert station_heading(FACE_M + 0.1, 0.7) is None
    print("track geometry ok")


if __name__ == "__main__":
    main()
