"""Geometry for holding xudrone under the tag on otter_k. ROS-free.

The upward camera is pitched -90 deg, so its look axis is the hull +Z.
Image +x is hull -Y and image +y is hull -X.
"""
from __future__ import annotations

import math

TAG_Z_BOAT = 0.254   # deck tag, metres above the Heron origin
CAM_Z = 0.13         # upward camera, metres above the xudrone origin
STANDOFF_M = 1.0     # camera-to-tag range to hold
TAG_ID = 10
FACE_M = 1.5         # inside this the nose keeps the boat's heading
STATION_PITCH_RAD = math.radians(20.0)   # keeps the deck tag in the up camera


class BaselineVelocity:
    """Horizontal velocity of a pose stream, over at least `span_s` of sim time.

    The pose snapshot can repeat between control ticks, so a one-tick
    difference alternates between zero and double. A longer baseline does not.
    """

    def __init__(self, span_s: float = 0.3, vmax: float = 2.0):
        self.span_s = float(span_s)
        self.vmax = float(vmax)
        self._ref = None
        self.vel = (0.0, 0.0, 0.0)

    def update(self, t: float, x: float, y: float):
        if self._ref is None:
            self._ref = (float(t), float(x), float(y))
            return self.vel
        dt = float(t) - self._ref[0]
        if dt < 0.0:
            self._ref = (float(t), float(x), float(y))
        elif dt >= self.span_s:
            vx = (float(x) - self._ref[1]) / dt
            vy = (float(y) - self._ref[2]) / dt
            n = math.hypot(vx, vy)
            if n > self.vmax:
                vx, vy = vx * self.vmax / n, vy * self.vmax / n
            self.vel = (vx, vy, 0.0)
            self._ref = (float(t), float(x), float(y))
        return self.vel


def station_heading(dist_m: float, boat_yaw: float, face_m: float = FACE_M):
    """Heading to hold near the station: the boat's inside face_m, else free (None)."""
    return float(boat_yaw) if float(dist_m) <= face_m else None


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
    from servo_geom import aim_point
    return aim_point(tag, "up", standoff, cam_offset=cam_z)


def clamp_depth(z, lo: float = -1.80, hi: float = -0.40):
    return min(hi, max(lo, float(z)))
