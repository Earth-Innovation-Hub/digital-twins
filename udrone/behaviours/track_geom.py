"""Geometry for holding xudrone under the tag on otter_k. ROS-free.

The upward camera is pitched -90 deg, so its look axis is the hull +Z.
Image +x is hull -Y and image +y is hull -X.
"""
from __future__ import annotations

TAG_Z_BOAT = 0.254   # deck tag, metres above the Heron origin
CAM_Z = 0.13         # upward camera, metres above the xudrone origin
STANDOFF_M = 1.0     # camera-to-tag range to hold
TAG_ID = 10


def up_camera_tag_body(x_n, y_n, z_opt, cam_z: float = CAM_Z):
    """Tag position in the hull frame. x_n, y_n are normalised image coordinates."""
    z = float(z_opt)
    return (
        -float(y_n) * z,
        -float(x_n) * z,
        float(cam_z) + z,
    )


def station_under(tag, standoff: float = STANDOFF_M, cam_z: float = CAM_Z):
    """Hull position that puts the upward camera `standoff` metres under the tag."""
    return (
        float(tag[0]),
        float(tag[1]),
        float(tag[2]) - float(standoff) - float(cam_z),
    )


def clamp_depth(z, lo: float = -1.80, hi: float = -0.40):
    return min(hi, max(lo, float(z)))
