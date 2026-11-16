"""PX4 NED / FRD to the Gazebo ENU / FLU frame the force loop speaks. ROS-free.

PoseFuser is the pose the behaviours fly and the mapper maps with: PX4
odometry, held onto Gazebo by a slowly tracking offset, and replaced by
Gazebo when the two disagree.
"""
from __future__ import annotations

import math
import time

ODOM_AGREE_M = 0.5
ODOM_RESYNC_S = 3.0      # time constant of the odometry-to-Gazebo offset
ODOM_STALE_S = 0.4

# 180 deg about X: aircraft FRD (x forward, y right, z down) to FLU.
_Q_FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)
# NED world to ENU world (mavros / px4_ros_com).
_Q_NED_TO_ENU = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)


def ned_to_enu(north: float, east: float, down: float):
    """(east, north, up)."""
    return (float(east), float(north), -float(down))


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def px4_q_to_enu(q):
    """Body-FRD-to-NED quaternion (w, x, y, z) to body-FLU-to-ENU."""
    w, x, y, z = (float(v) for v in q[:4])
    # q_enu_flu = Q_ned_to_enu * q_ned_frd * q_frd_to_flu^-1
    return _qmul(_qmul(_Q_NED_TO_ENU, (w, x, y, z)), _qconj(_Q_FRD_TO_FLU))


class PoseFuser:
    """Control pose in ENU. Gazebo is the check that rejects a bad estimate.

    `vpos` is PX4 local position (n, e, d, vn, ve, vd, wall time) or None.
    After `fuse`, `source` is "odom" or "gazebo", `vel` is the ENU velocity
    from PX4 (None when Gazebo is flown), `gz_miss` is the estimate's
    distance from Gazebo, and `rejected` says this call refused odometry.
    """

    def __init__(self, agree_m=ODOM_AGREE_M, resync_s=ODOM_RESYNC_S, stale_s=ODOM_STALE_S):
        self.agree_m = float(agree_m)
        self.resync_s = float(resync_s)
        self.stale_s = float(stale_s)
        self.offset = None
        self.offset_t = 0.0
        self.source = "gazebo"
        self.vel = None
        self.gz_miss = 0.0
        self.rejects = 0
        self.rejected = False

    def fuse(self, gz_pos, gz_q, vpos, odom_q, now=None):
        now = time.time() if now is None else float(now)
        self.rejected = False
        fresh = vpos is not None and (now - vpos[6]) < self.stale_s
        q = gz_q if gz_q is not None else odom_q
        if not fresh:
            self.source = "gazebo"
            self.vel = None
            return gz_pos, q
        n, e, d, vn, ve, vd, _wall = vpos
        odom_enu = ned_to_enu(n, e, d)
        odom_vel = ned_to_enu(vn, ve, vd)
        if gz_pos is None:
            self.source = "odom"
            self.vel = odom_vel
            return odom_enu, q or odom_q
        target = tuple(g - o for g, o in zip(gz_pos, odom_enu))
        if self.offset is None:
            self.offset = target
            self.offset_t = now
        est = tuple(o + b for o, b in zip(odom_enu, self.offset))
        self.gz_miss = math.dist(est, gz_pos)
        # Estimator drift is slow, so the offset follows Gazebo slowly. A jump
        # still fails the agreement check below.
        k = min(1.0, max(0.0, now - self.offset_t) / self.resync_s)
        self.offset = tuple(b + k * (g - b) for b, g in zip(self.offset, target))
        self.offset_t = now
        if self.gz_miss > self.agree_m:
            self.rejects += 1
            self.rejected = True
            self.source = "gazebo"
            self.vel = None
            return gz_pos, q
        self.source = "odom"
        self.vel = odom_vel
        return est, q
