"""The upward camera maps a centred tag to a point above the hull."""
from track_geom import CAM_Z, STANDOFF_M, station_under, up_camera_tag_body


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
    print("track geometry ok")


if __name__ == "__main__":
    main()
