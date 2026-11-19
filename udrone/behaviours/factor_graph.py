"""xudrone pose graph on GTSAM iSAM2. ROS-free.

Variables
  X(k)   keyframe pose (Pose3, ENU map frame, body FLU)
  L(id)  static AprilTag landmark (Point3)

Factors
  prior       first keyframe, at the fused pose
  odom        BetweenFactorPose3 from PX4 odometry deltas (the dead-reckoning chain)
  depth       pressure depth on z (CustomFactor), every keyframe
  gnss        GPSFactor on position, only at the surface
  tag         BearingRangeFactor3D from a keyframe to a tag, in the body frame

A keyframe is cut after KF_DIST_M of travel, KF_YAW_RAD of turn, KF_MAX_S of
time, or when a tag needs a fresh pose to hang on. Tags in DYNAMIC_TAGS ride on
a vehicle (tag 10 is under the Heron deck, 11 and 12 are on its pontoon
sterns) and stay out of the static map.

GTSAM comes from a private directory (UDRONE_GTSAM_PATH, default
/opt/udrone_py) so the system Python packages stay untouched. Without it,
`available()` is False and the mapper runs without the graph.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from collections import Counter

import numpy as np

_GTSAM_PATH = os.environ.get("UDRONE_GTSAM_PATH", "/opt/udrone_py")
if os.path.isdir(_GTSAM_PATH) and _GTSAM_PATH not in sys.path:
    sys.path.append(_GTSAM_PATH)
try:
    import gtsam
    from gtsam.symbol_shorthand import L, X
except ImportError:          # mapper runs without the graph
    gtsam = None

KF_DIST_M = 1.0
KF_YAW_RAD = math.radians(15.0)
KF_MAX_S = 10.0
TAG_KF_S = 0.25              # a tag sighting cuts a keyframe if the last is older
TAG_MAX_RANGE_M = 12.0
DYNAMIC_TAGS = frozenset(
    int(t) for t in os.environ.get("UDRONE_GRAPH_DYNAMIC_TAGS", "10,11,12").split(",") if t.strip()
)

SIG_PRIOR_ROT, SIG_PRIOR_T = 0.05, 0.2
SIG_ODOM_ROT, SIG_ODOM_T = 0.01, 0.05       # per keyframe, plus ODOM_T_PER_M of travel
ODOM_T_PER_M = 0.03
SIG_DEPTH = 0.05
SIG_GNSS = 0.7
SIG_BEARING, SIG_RANGE, RANGE_PER_M = 0.03, 0.05, 0.03
SURFACE_Z = -0.3             # above this the GNSS antenna is out of the water


def available() -> bool:
    return gtsam is not None


def pose3(pos, q):
    """gtsam.Pose3 from ENU position and quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    return gtsam.Pose3(gtsam.Rot3.Quaternion(w, x, y, z), np.asarray(pos, float))


def _depth_error(z_meas):
    def err(this, values, jac):
        p = values.atPose3(this.keys()[0])
        if jac is not None:
            r = p.rotation().matrix()
            jac[0] = np.array([[0.0, 0.0, 0.0, r[2, 0], r[2, 1], r[2, 2]]])
        return np.array([p.translation()[2] - z_meas])
    return err


def optical_to_body(t_optical, mount_t, mount_q):
    """Tag position from apriltag (camera optical: x right, y down, z ahead) to body."""
    from map_geom import rot   # local import keeps this module importable alone

    x, y, z = (float(v) for v in t_optical)
    link = np.array([z, -x, -y])          # camera link: x ahead, y left, z up
    return rot(mount_q) @ link + np.asarray(mount_t, float)


class PoseGraph:
    def __init__(self):
        p = gtsam.ISAM2Params()
        p.setRelinearizeThreshold(0.05)
        p.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(p)
        self.k = -1
        self.est = None              # gtsam.Values
        self._new = gtsam.NonlinearFactorGraph()
        self._init = gtsam.Values()
        self.kf_t = 0.0
        self.kf_odom = None          # Pose3 of the last keyframe in the odometry frame
        self.kf_pos = []             # optimised keyframe positions (refreshed on update)
        self.counts = Counter()
        self.landmarks = {}          # id -> observation count
        self._tag_kf = {}            # id -> keyframe index of its last factor
        self.log = []                # factor records for the JSON dump
        self.align = None            # world <- odom at the first keyframe (for drift)
        self.sigma_m = None
        self.cov = None              # 6x6 marginal of the newest keyframe (rot, trans)
        self.last_update_ms = 0.0

    # ---- keyframes -----------------------------------------------------
    def need_keyframe(self, odom_pose, now, tag=False):
        if self.k < 0:
            return True
        d = self.kf_odom.between(odom_pose)
        if np.linalg.norm(d.translation()) >= KF_DIST_M:
            return True
        if abs(d.rotation().yaw()) >= KF_YAW_RAD:
            return True
        if tag:
            return now - self.kf_t >= TAG_KF_S
        return now - self.kf_t >= KF_MAX_S

    def add_keyframe(self, odom_pose, fused_pose, now, depth_z=None, gnss=None):
        """New X(k): odometry from the last keyframe, plus depth and (at the surface) GNSS."""
        k = self.k + 1
        if k == 0:
            noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([SIG_PRIOR_ROT] * 3 + [SIG_PRIOR_T] * 3))
            self._new.add(gtsam.PriorFactorPose3(X(0), fused_pose, noise))
            self._init.insert(X(0), fused_pose)
            self.align = fused_pose.compose(odom_pose.inverse())
            self._record("prior", [0], fused_pose.translation())
        else:
            d = self.kf_odom.between(odom_pose)
            dist = float(np.linalg.norm(d.translation()))
            st = SIG_ODOM_T + ODOM_T_PER_M * dist
            noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([SIG_ODOM_ROT] * 3 + [st] * 3))
            self._new.add(gtsam.BetweenFactorPose3(X(k - 1), X(k), d, noise))
            self._init.insert(X(k), self.pose(k - 1).compose(d))
            self._record("odom", [k - 1, k], d.translation())
        if depth_z is not None:
            self._new.add(gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(1, SIG_DEPTH),
                                             [X(k)], _depth_error(float(depth_z))))
            self._record("depth", [k], [float(depth_z)])
        if gnss is not None:
            self._new.add(gtsam.GPSFactor(X(k), np.asarray(gnss, float),
                                          gtsam.noiseModel.Isotropic.Sigma(3, SIG_GNSS)))
            self._record("gnss", [k], gnss)
        self.k = k
        self.kf_t = now
        self.kf_odom = odom_pose
        return k

    def add_tag(self, tag_id, body_vec):
        """Bearing-range from the current keyframe to static tag `tag_id`. False if skipped.

        Call only for a keyframe cut in the same tick as the measurement. A
        bearing hung on an older keyframe after the hull has moved pulls the
        landmark to the wrong place.
        """
        tag_id = int(tag_id)
        if self.k < 0 or tag_id in DYNAMIC_TAGS:
            return False
        v = np.asarray(body_vec, float)
        rng = float(np.linalg.norm(v))
        if not (0.2 < rng <= TAG_MAX_RANGE_M) or self._tag_kf.get(tag_id) == self.k:
            return False
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([SIG_BEARING, SIG_BEARING, SIG_RANGE + RANGE_PER_M * rng]))
        self._new.add(gtsam.BearingRangeFactor3D(X(self.k), L(tag_id), gtsam.Unit3(v), rng, noise))
        if tag_id not in self.landmarks and not self._has(L(tag_id)):
            self._init.insert(L(tag_id), self.pose(self.k).transformFrom(v))
        self.landmarks[tag_id] = self.landmarks.get(tag_id, 0) + 1
        self._tag_kf[tag_id] = self.k
        self._record("tag", [self.k], v, tag=tag_id)
        return True

    # ---- solve ---------------------------------------------------------
    def update(self):
        if self._new.size() == 0:
            return False
        t0 = time.perf_counter()
        self.isam.update(self._new, self._init)
        self._new = gtsam.NonlinearFactorGraph()
        self._init = gtsam.Values()
        self.est = self.isam.calculateEstimate()
        self.kf_pos = [self.est.atPose3(X(i)).translation() for i in range(self.k + 1)]
        self.last_update_ms = 1000.0 * (time.perf_counter() - t0)
        return True

    def refresh_sigma(self):
        """1-sigma position uncertainty of the newest keyframe (m)."""
        if self.est is None or self.k < 0:
            return None
        cov = self.isam.marginalCovariance(X(self.k))
        self.cov = cov
        self.sigma_m = float(math.sqrt(max(0.0, np.trace(cov[3:, 3:]))))
        return self.sigma_m

    # ---- queries -------------------------------------------------------
    def _has(self, key):
        return (self.est is not None and self.est.exists(key)) or self._init.exists(key)

    def pose(self, k):
        if self._init.exists(X(k)):
            return self._init.atPose3(X(k))
        return self.est.atPose3(X(k))

    def now_pose(self, odom_pose):
        """Graph pose now: the newest keyframe carried forward by odometry since."""
        if self.k < 0 or self.est is None or not self.est.exists(X(self.k)):
            return None
        return self.est.atPose3(X(self.k)).compose(self.kf_odom.between(odom_pose))

    def odom_only(self, odom_pose):
        """Dead reckoning: odometry aligned once at the first keyframe."""
        return None if self.align is None else self.align.compose(odom_pose)

    def landmark_list(self):
        out = []
        if self.est is None:
            return out
        for tid, n in sorted(self.landmarks.items()):
            if self.est.exists(L(tid)):
                p = self.est.atPoint3(L(tid))
                out.append({"id": tid, "x": float(p[0]), "y": float(p[1]), "z": float(p[2]), "obs": n})
        return out

    def summary(self):
        return {
            "keyframes": self.k + 1,
            "factors": dict(self.counts),
            "landmarks": self.landmark_list(),
            "sigma_m": self.sigma_m,
            "update_ms": round(self.last_update_ms, 2),
        }

    def _record(self, kind, keys, meas, **extra):
        self.counts[kind] += 1
        rec = {"type": kind, "keys": keys, "meas": [round(float(v), 4) for v in meas]}
        rec.update(extra)
        self.log.append(rec)

    def dump(self, path):
        """Keyframes, landmarks, and every factor as JSON (for offline solvers)."""
        kfs = []
        if self.est is not None:
            for i in range(self.k + 1):
                if self.est.exists(X(i)):
                    p = self.est.atPose3(X(i))
                    q = p.rotation().toQuaternion()
                    kfs.append({"k": i, "t": [float(v) for v in p.translation()],
                                "q": [q.w(), q.x(), q.y(), q.z()]})
        with open(path, "w") as f:
            json.dump({"keyframes": kfs, "landmarks": self.landmark_list(),
                       "factors": self.log}, f)
