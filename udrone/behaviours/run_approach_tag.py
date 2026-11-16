#!/usr/bin/env python3
"""AUV front-camera IBVS approach (attitude-rate + thrust).

Uses /xudrone/detections (apriltag on the forward camera) and drives PX4
VehicleRatesSetpoint directly — the same offboard path as helix (airframe
60003 UUV_SKIP_CTRL). Not ENU waypoint chase.

Image features (normalized):
  e_y = (u - u*) / fx     tag right of centre  → yaw right (command -e_y)
  e_z = (v - v*) / fy     tag below centre     → nose down (+pitch)
  e_s = (s* - s) / s*     tag too small/far    → surge forward

s = mean tag side length in pixels; s* = fx * tag_size_m / z_des
(z_des = standoff along the optical axis, metres).

Stages: search (expanding spiral) → approach → settle → done.
On lost track: back to search (radius kept; grows until lock or budget).
"""
from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan

from udrone_sinusoidal_controller import (
    UdroneHelixController,
    clamp,
    rpy_from_q,
    spin_until_done,
    wrap_pi,
)

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # host without msgs
    AprilTagDetectionArray = None

# IBVS gains → normalized rate / thrust commands (see helix TORQUE_MAX / surge).
LAM_YAW = float(os.environ.get("UDRONE_IBVS_LAM_YAW", "0.9"))
LAM_PITCH = float(os.environ.get("UDRONE_IBVS_LAM_PITCH", "1.0"))
LAM_THRUST = float(os.environ.get("UDRONE_IBVS_LAM_THRUST", "0.12"))
THRUST_BIAS = float(os.environ.get("UDRONE_IBVS_THRUST_BIAS", "0.02"))
RATE_MAX = float(os.environ.get("UDRONE_IBVS_RATE_MAX", "0.25"))
THRUST_MAX = float(os.environ.get("UDRONE_IBVS_THRUST_MAX", "0.25"))
ARRIVE_UV = float(os.environ.get("UDRONE_IBVS_ARRIVE_UV", "0.14"))
ARRIVE_S = float(os.environ.get("UDRONE_IBVS_ARRIVE_S", "0.75"))
LOST_S = float(os.environ.get("UDRONE_IBVS_LOST_S", "2.5"))

# Expanding spiral search (attitude rates only).
SEARCH_YAW_RATE = float(os.environ.get("UDRONE_IBVS_SEARCH_YAW", "0.18"))
SEARCH_PITCH0 = float(os.environ.get("UDRONE_IBVS_SEARCH_PITCH", "0.14"))
SEARCH_PITCH_BOB = float(os.environ.get("UDRONE_IBVS_SEARCH_PITCH_BOB", "0.06"))
SEARCH_R_GROW = float(os.environ.get("UDRONE_IBVS_SEARCH_R_GROW", "0.08"))  # m/s toward R_max
SEARCH_ORBIT_BUMP = float(os.environ.get("UDRONE_IBVS_SEARCH_ORBIT_BUMP", "0.6"))  # m per orbit

# Ground / surface awareness (EchoRanger + depth from GZ pose z).
AGL_MIN_M = float(os.environ.get("UDRONE_IBVS_AGL_MIN", "0.45"))       # keep off the bed
AGL_SOFT_M = float(os.environ.get("UDRONE_IBVS_AGL_SOFT", "0.90"))
SURF_MIN_M = float(os.environ.get("UDRONE_IBVS_SURF_MIN", "0.35"))     # keep below free surface
DEPTH_SHALLOW = float(os.environ.get("UDRONE_IBVS_DEPTH_SHALLOW", "-0.35"))  # world z
DEPTH_DEEP = float(os.environ.get("UDRONE_IBVS_DEPTH_DEEP", "-1.85"))
RANGE_STALE_S = float(os.environ.get("UDRONE_IBVS_RANGE_STALE", "0.8"))


def _tag_side_px(corners) -> float:
    """Mean side length of the tag quadrilateral in pixels."""
    if not corners or len(corners) < 4:
        return 0.0
    pts = [(float(c.x), float(c.y)) for c in corners]
    sides = [
        math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
        for i in range(4)
    ]
    return sum(sides) / 4.0


class UdroneApproachController(UdroneHelixController):
    def __init__(
        self,
        tag_id=0,
        standoff_m=1.5,
        depth_m=-1.2,
        arrive_m=0.5,
        depth_tol_m=0.25,
        speed_mps=0.25,
        settle_s=3.0,
        timeout_s=90.0,
        acquire_s=25.0,
        search_budget_s=None,
        search_r0_m=1.0,
        search_r_max_m=8.0,
        tag_size_m=0.4,
        detections_topic="/xudrone/detections",
        camera_info_topic="/xudrone/camera_front/camera_info",
        bottom_range_topic="/xudrone/range/bottom",
        surface_range_topic="/xudrone/range/surface",
        center_east_m=None,
        center_north_m=None,
        **kwargs,
    ):
        self.tag_id = int(tag_id)
        self.z_des = max(0.4, float(standoff_m) if float(standoff_m) > 1e-3 else 1.5)
        self.depth_m = float(depth_m)
        self.arrive_m = float(arrive_m)
        self.depth_tol_m = float(depth_tol_m)
        self.speed_mps = float(speed_mps)
        self.settle_s = float(settle_s)
        self.timeout_s = float(timeout_s)
        # acquire_s kept for CLI compat; search_budget is the real fail timer.
        self.acquire_s = float(acquire_s)
        self.search_budget_s = float(
            search_budget_s if search_budget_s is not None else max(acquire_s, 45.0)
        )
        self.search_r0 = max(0.3, float(search_r0_m))
        self.search_r_max = max(self.search_r0, float(search_r_max_m))
        self.tag_size_m = float(tag_size_m)

        self.fx = 381.46
        self.fy = 381.46
        self.cx = 320.0
        self.cy = 240.0
        self._have_info = False

        self._det_u = None
        self._det_v = None
        self._det_s = None
        self._det_t = 0.0
        self._det_id = None

        # EchoRanger (bottom AGL) + upward surface clearance (metres).
        self._agl_m = None
        self._agl_t = 0.0
        self._surf_m = None
        self._surf_t = 0.0

        self.arrived = False
        self._settle_t0 = None
        self._stage = "search"
        self._had_tag = False
        self.tag_source = None

        # Search spiral state (ENU centre + growing radius).
        self._seed_e = float(center_east_m) if center_east_m is not None else 5.0
        self._seed_n = float(center_north_m) if center_north_m is not None else 4.0
        self.tag_e = self._seed_e
        self.tag_n = self._seed_n
        self._lock_e = self._seed_e
        self._lock_n = self._seed_n
        self._search_r = self.search_r0
        self._search_t0 = None          # sim-tm when current search bout began
        self._search_total = 0.0        # accumulated search time (sim)
        self._orbit_acc = 0.0           # integrated |yaw_rate| * dt → orbits
        self._last_orbit_n = 0
        self._search_yaw_sign = 1.0
        self._last_search_tm = None
        self._orbit_tm = None

        kwargs.setdefault("duration_s", self.timeout_s + 5.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.speed_mps)
        kwargs.setdefault("z0", self.depth_m)
        super().__init__(**kwargs)

        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, bottom_range_topic, self._on_bottom_range, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, surface_range_topic, self._on_surface_range, qos_profile_sensor_data
        )
        if AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray,
                detections_topic,
                self._on_apriltag,
                qos_profile_sensor_data,
            )
        else:
            self.get_logger().error("apriltag_msgs missing — IBVS cannot run")

        self.get_logger().info(
            f"approach_tag IBVS+search: id={self.tag_id} z_des={self.z_des:g} m "
            f"R0={self.search_r0:g} Rmax={self.search_r_max:g} "
            f"budget={self.search_budget_s:g}s seed=({self._seed_e:g},{self._seed_n:g}) "
            f"agl_min={AGL_MIN_M:g} surf_min={SURF_MIN_M:g} topic={detections_topic}"
        )

    # ---- vision ---------------------------------------------------------
    def _on_info(self, m: CameraInfo):
        k = list(m.k) if m.k is not None else []
        if len(k) >= 6:
            self.fx = float(k[0]) or self.fx
            self.fy = float(k[4]) or self.fy
            self.cx = float(k[2]) if k[2] else (float(m.width) * 0.5)
            self.cy = float(k[5]) if k[5] else (float(m.height) * 0.5)
            self._have_info = True

    @staticmethod
    def _scan_range_m(msg: LaserScan):
        if not msg.ranges:
            return None
        r = float(msg.ranges[0])
        if not math.isfinite(r):
            return None
        if r < msg.range_min or r > msg.range_max:
            return None
        return r

    def _on_bottom_range(self, m: LaserScan):
        r = self._scan_range_m(m)
        if r is None:
            return
        self._agl_m = r
        self._agl_t = time.time()

    def _on_surface_range(self, m: LaserScan):
        r = self._scan_range_m(m)
        if r is None:
            return
        self._surf_m = r
        self._surf_t = time.time()

    def _fresh_agl(self):
        if self._agl_m is None or (time.time() - self._agl_t) > RANGE_STALE_S:
            return None
        return self._agl_m

    def _fresh_surf(self):
        if self._surf_m is None or (time.time() - self._surf_t) > RANGE_STALE_S:
            return None
        return self._surf_m

    def _apply_clearance(self, pitch_u, thrust, pos):
        """Keep off the bed and below the free surface using EchoRanger + depth."""
        x, y, z = pos
        agl = self._fresh_agl()
        surf = self._fresh_surf()
        note = None

        # Depth band from world z (GZ pose; baro feeds PX4 EKF separately).
        if z > DEPTH_SHALLOW:
            # Too shallow → pitch nose down a bit, ease surge.
            pitch_u = max(pitch_u, 0.12)
            thrust = min(thrust, 0.06)
            note = f"shallow z={z:.2f}"
        elif z < DEPTH_DEEP:
            # Too deep → lift nose (negative pitch), cut surge.
            pitch_u = min(pitch_u, -0.08)
            thrust = min(thrust, 0.04)
            note = f"deep z={z:.2f}"

        if agl is not None:
            if agl < AGL_MIN_M:
                pitch_u = min(pitch_u, -0.12)  # nose up off the bed
                thrust = min(abs(thrust), 0.03) * (1.0 if thrust >= 0 else -1.0)
                if thrust > 0:
                    thrust = 0.0
                note = f"agl={agl:.2f}<{AGL_MIN_M:g}"
            elif agl < AGL_SOFT_M:
                pitch_u = min(pitch_u, 0.06)  # limit nose-down
                thrust = min(thrust, 0.08)
                note = note or f"agl_soft={agl:.2f}"

        if surf is not None and surf < SURF_MIN_M:
            pitch_u = max(pitch_u, 0.10)  # nose down away from surface
            thrust = min(thrust, 0.05)
            note = note or f"surf={surf:.2f}"

        return pitch_u, thrust, note, agl, surf

    def _on_apriltag(self, m: AprilTagDetectionArray):
        if not m.detections:
            return
        best = None
        for d in m.detections:
            if int(d.id) != self.tag_id:
                continue
            best = d
            break
        if best is None:
            return
        u, v = float(best.centre.x), float(best.centre.y)
        s = _tag_side_px(best.corners)
        if s < 1.0:
            return
        self._det_u, self._det_v, self._det_s = u, v, s
        self._det_t = time.time()
        self._det_id = int(best.id)
        self.tag_source = "front_ibvs"
        self._had_tag = True
        if self._stage == "search":
            self._stage = "approach"
            # Keep R for next loss; do not reset to R0 on flicker.
            self._search_t0 = None
            print(
                f"APPROACH_ACQUIRED IBVS u={u:.1f} v={v:.1f} s={s:.1f}px "
                f"z_est={self._z_est(s):.2f}m R={self._search_r:.1f}",
                flush=True,
            )

    def _z_est(self, s_px: float) -> float:
        if s_px <= 1e-3:
            return float("inf")
        return self.fx * self.tag_size_m / s_px

    def _s_des(self) -> float:
        return self.fx * self.tag_size_m / self.z_des

    def _fresh_det(self) -> bool:
        return (
            self._det_u is not None
            and (time.time() - self._det_t) <= LOST_S
        )

    def _enter_search(self, tm: float, reason: str, pos=None):
        """Drop to expanding spiral; centre on last lock / seed / current pose."""
        if pos is not None:
            x, y, _z = pos
            # If we had a detection recently, keep seed; else nudge center toward us.
            if self._had_tag and self._lock_e is not None:
                pass
            elif self.tag_e is not None:
                self._lock_e, self._lock_n = self.tag_e, self.tag_n
            else:
                self._lock_e, self._lock_n = x, y
        self._stage = "search"
        self._settle_t0 = None
        self._search_t0 = tm
        self._last_search_tm = tm
        self._orbit_acc = 0.0
        self._last_orbit_n = 0
        self._orbit_tm = tm
        # Mild bump on each re-entry so lost-track expands coverage.
        if self._had_tag:
            self._search_r = min(self.search_r_max, self._search_r + 0.5 * SEARCH_ORBIT_BUMP)
        print(
            f"APPROACH_SEARCH {reason} R={self._search_r:.1f}/{self.search_r_max:.1f} "
            f"center=({self._lock_e:.1f},{self._lock_n:.1f}) "
            f"budget_left={max(0.0, self.search_budget_s - self._search_total):.0f}s",
            flush=True,
        )

    def _update_lock_enu(self, pos, q):
        """Rough ENU of tag from vehicle pose + optical range estimate.

        Only refresh when the tag is reasonably centred — early/edge detections
        otherwise drag the search seed off the pad.
        """
        if not self._fresh_det() or pos is None or q is None:
            return
        ey = (self._det_u - self.cx) / max(self.fx, 1e-3)
        ez = (self._det_v - self.cy) / max(self.fy, 1e-3)
        if max(abs(ey), abs(ez)) > 0.35:
            return
        x, y, _z = pos
        yaw = rpy_from_q(q)[2]
        z_est = self._z_est(self._det_s)
        if not math.isfinite(z_est) or z_est > 20.0:
            z_est = self.z_des
        bearing = yaw - ey  # yaw_u = -ey convention
        self._lock_e = x + z_est * math.cos(bearing)
        self._lock_n = y + z_est * math.sin(bearing)
        self.tag_e, self.tag_n = self._lock_e, self._lock_n

    # ---- control hooks --------------------------------------------------
    def _events(self, tm: float):
        if self._stage == "search":
            if self._search_t0 is None:
                self._search_t0 = tm
                self._last_search_tm = tm
            # Accumulate search time only while searching.
            if self._last_search_tm is not None and tm > self._last_search_tm:
                self._search_total += tm - self._last_search_tm
            self._last_search_tm = tm

            at_rmax = self._search_r >= self.search_r_max - 1e-3
            if self._search_total >= self.search_budget_s and at_rmax and not self._fresh_det():
                self.failed = "lost_tag"
                print(
                    f"FAIL lost_tag (search budget {self.search_budget_s:.0f}s "
                    f"and Rmax={self.search_r_max:g} exhausted)",
                    flush=True,
                )
                self.done = True
                return
            # Soft fail: budget exhausted even if not at Rmax (avoid infinite grow).
            if self._search_total >= self.search_budget_s and not self._fresh_det():
                self.failed = "lost_tag"
                print(
                    f"FAIL lost_tag (search budget {self.search_budget_s:.0f}s exhausted, "
                    f"R={self._search_r:.1f})",
                    flush=True,
                )
                self.done = True
                return
            return

        if not self._fresh_det():
            if self._stage == "settle" and self._settle_t0 is not None:
                held = tm - self._settle_t0
                if held >= max(1.0, 0.5 * self.settle_s):
                    self.arrived = True
                    self._stage = "done"
                    print(
                        f"APPROACH_ARRIVED IBVS (settle held {held:.1f}s; det dropout) "
                        f"u={self._det_u:.1f} v={self._det_v:.1f} s={self._det_s:.1f}",
                        flush=True,
                    )
                    return
            if self._stage in ("approach", "settle") and (time.time() - self._det_t) > LOST_S:
                pos = self._prev[1] if self._prev else None
                self._enter_search(tm, "lost_track", pos=pos)
            return

        # Fresh detection — refresh lock ENU while approaching.
        if self._prev is not None:
            self._update_lock_enu(self._prev[1], self._prev[2])

        ey = (self._det_u - self.cx) / max(self.fx, 1e-3)
        ez = (self._det_v - self.cy) / max(self.fy, 1e-3)
        s_des = self._s_des()
        s_ratio = self._det_s / max(s_des, 1e-3)

        uv_lim = ARRIVE_UV * (1.6 if self._stage == "settle" else 1.0)
        aligned = abs(ey) < uv_lim and abs(ez) < uv_lim and s_ratio >= ARRIVE_S
        close_enough = s_ratio >= ARRIVE_S and max(abs(ey), abs(ez)) < ARRIVE_UV * 1.8
        if aligned or (self._stage == "settle" and close_enough):
            if self._settle_t0 is None:
                self._settle_t0 = tm
                self._stage = "settle"
                # Successful close approach — shrink search radius for next time.
                self._search_r = self.search_r0
                print(
                    f"APPROACH_SETTLE ey={ey:.3f} ez={ez:.3f} s_ratio={s_ratio:.2f} "
                    f"z_est={self._z_est(self._det_s):.2f}",
                    flush=True,
                )
            elif tm - self._settle_t0 >= self.settle_s:
                self.arrived = True
                self._stage = "done"
                print(
                    f"APPROACH_ARRIVED IBVS u={self._det_u:.1f} v={self._det_v:.1f} "
                    f"s={self._det_s:.1f} z_est={self._z_est(self._det_s):.2f}",
                    flush=True,
                )
        else:
            if self._stage == "settle" and s_ratio >= ARRIVE_S * 0.7:
                pass
            else:
                self._settle_t0 = None
                if self._stage == "settle":
                    self._stage = "approach"

    def _helix(self, tm: float):
        z = self.depth_m
        if self._prev is not None:
            z = self._prev[1][2]
        return (0.0, 0.0, z), (0.0, 0.0, 0.0)

    def _search_rates(self, tm: float, pos, q):
        """Expanding spiral: orbit yaw + surge to hold/grow radius around lock centre."""
        x, y, _z = pos
        yaw = rpy_from_q(q)[2]
        ce, cn = self._lock_e, self._lock_n
        dx, dy = ce - x, cn - y
        horiz = math.hypot(dx, dy)
        bearing_in = math.atan2(dy, dx)  # toward centre

        # Grow R with search time; bump after each full orbit.
        if self._search_t0 is not None:
            grown = self.search_r0 + SEARCH_R_GROW * max(0.0, tm - self._search_t0)
            # Also apply orbit bumps already folded into _search_r.
            self._search_r = min(self.search_r_max, max(self._search_r, grown))

        # Pitch: nose-down bias + slow bob for vertical FOV coverage.
        pitch_u = clamp(
            SEARCH_PITCH0
            + 0.04 * (self._search_r / max(self.search_r_max, 1e-3))
            + SEARCH_PITCH_BOB * math.sin(0.45 * tm),
            RATE_MAX,
        )

        # Tangential heading = centre bearing ± 90° (orbit).
        tangent = bearing_in + self._search_yaw_sign * (0.5 * math.pi)
        # Blend: when far outside R, head inward; inside R, head outward; on ring, orbit.
        if horiz > self._search_r + 0.6:
            desire = bearing_in  # go toward centre / ring
            thrust = 0.14
        elif horiz < max(0.4, self._search_r - 0.6):
            desire = wrap_pi(bearing_in + math.pi)  # outward
            thrust = 0.12
        else:
            desire = tangent
            # Circumference speed ~ yaw_rate * R → surge ≈ omega * R (capped).
            thrust = clamp(0.05 + 0.04 * self._search_r, 0.18)

        eyaw = wrap_pi(desire - yaw)
        yaw_u = clamp(0.9 * eyaw + self._search_yaw_sign * 0.3 * SEARCH_YAW_RATE, RATE_MAX)

        # Integrate orbit progress from commanded yaw rate magnitude.
        dt = 0.05
        if self._orbit_tm is not None and tm > self._orbit_tm:
            dt = tm - self._orbit_tm
        self._orbit_tm = tm
        self._orbit_acc += abs(yaw_u) * dt * 2.5  # scale normalized cmd → rad-ish
        orbit_n = int(self._orbit_acc / (2.0 * math.pi))
        if orbit_n > self._last_orbit_n:
            self._last_orbit_n = orbit_n
            old = self._search_r
            self._search_r = min(self.search_r_max, self._search_r + SEARCH_ORBIT_BUMP)
            # Alternate orbit direction every other bump for coverage.
            if orbit_n % 2 == 0:
                self._search_yaw_sign *= -1.0
            print(
                f"SEARCH_ORBIT n={orbit_n} R {old:.1f}->{self._search_r:.1f} "
                f"horiz={horiz:.1f} sign={self._search_yaw_sign:+.0f}",
                flush=True,
            )

        return pitch_u, yaw_u, thrust

    def _control(self, tm, auv, boat):
        """Search spiral or IBVS → body rates (pitch/yaw) + surge. Roll always 0."""
        pos, q = auv
        sim_t = self.t_thrust0 + tm
        if self._prev is None:
            self._prev = (sim_t, pos, q)
        elif sim_t > self._prev[0] + 1e-3:
            self._prev = (sim_t, pos, q)

        if self._stage == "search" and self._search_t0 is None:
            self._enter_search(tm, "start", pos=pos)

        thrust = 0.0
        pitch_u = 0.0
        yaw_u = 0.0

        if self._stage == "search" and not self._fresh_det():
            pitch_u, yaw_u, thrust = self._search_rates(tm, pos, q)
            if tm >= self.next_print:
                self.next_print = tm + 2.0
                x, y, _z = pos
                horiz = math.hypot(self._lock_e - x, self._lock_n - y)
                print(
                    f"t={tm:5.1f} SEARCH R={self._search_r:.1f}/{self.search_r_max:.1f} "
                    f"horiz={horiz:.1f} yaw_u={yaw_u:+.2f} pitch_u={pitch_u:+.2f} "
                    f"thrust={thrust:+.2f} tot={self._search_total:.0f}s",
                    flush=True,
                )
        elif self._fresh_det() and self._stage in ("approach", "settle", "done"):
            ey = (self._det_u - self.cx) / max(self.fx, 1e-3)
            ez = (self._det_v - self.cy) / max(self.fy, 1e-3)
            s_des = self._s_des()
            es = (s_des - self._det_s) / max(s_des, 1e-3)
            s_ratio = self._det_s / max(s_des, 1e-3)

            yaw_u = clamp(-LAM_YAW * ey, RATE_MAX)
            pitch_ff = 0.05 if 0.4 < s_ratio < 0.85 else 0.0
            pitch_u = clamp(LAM_PITCH * ez + pitch_ff, RATE_MAX)

            if self._stage in ("settle", "done"):
                yaw_u = clamp(-0.4 * LAM_YAW * ey, 0.12)
                pitch_u = clamp(0.4 * LAM_PITCH * ez, 0.12)
                thrust = 0.0 if es < 0.05 else clamp(0.5 * THRUST_BIAS, 0.05)
            else:
                align = max(abs(ey), abs(ez))
                surge_scale = 1.0
                if align > 0.12:
                    surge_scale = 0.25
                if s_ratio > 0.55:
                    surge_scale *= 0.4
                if s_ratio > 0.85:
                    surge_scale *= 0.25
                thrust = clamp(THRUST_BIAS + LAM_THRUST * es * surge_scale, THRUST_MAX)
                if es < -0.05:
                    thrust = clamp(LAM_THRUST * es, THRUST_MAX)

            if tm >= self.next_print:
                self.next_print = tm + 1.0
                print(
                    f"t={tm:5.1f} IBVS ey={ey:+.3f} ez={ez:+.3f} es={es:+.3f} "
                    f"s={self._det_s:.0f}/{s_des:.0f}px z≈{self._z_est(self._det_s):.2f} "
                    f"yaw_u={yaw_u:+.2f} pitch_u={pitch_u:+.2f} thrust={thrust:+.2f} "
                    f"stage={self._stage} latch={self.latch.state}",
                    flush=True,
                )

        if self.latch.state == "attached":
            pitch_u = 0.0
            yaw_u = 0.0
            thrust = 0.0
        else:
            pitch_u, thrust, clr, agl, surf = self._apply_clearance(pitch_u, thrust, pos)
            if clr is not None and tm >= self.next_print - 0.05:
                print(
                    f"t={tm:5.1f} CLEARANCE {clr} "
                    f"agl={agl if agl is not None else float('nan'):.2f} "
                    f"surf={surf if surf is not None else float('nan'):.2f} "
                    f"z={pos[2]:.2f} pitch_u={pitch_u:+.2f} thrust={thrust:+.2f}",
                    flush=True,
                )

        x, y, z = pos
        self.log.append(
            (
                tm, x, y, z,
                self._det_u or 0.0, self._det_v or 0.0, self._det_s or 0.0,
                thrust, yaw_u, pitch_u, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, float(self.latch.code()),
            )
        )
        return thrust, 0.0, pitch_u, yaw_u

    def _finished(self, tm: float) -> bool:
        if self.failed:
            return True
        if self.arrived:
            return True
        if tm >= self.timeout_s:
            if not self.arrived:
                self.failed = "timeout"
                print(f"FAIL timeout after {tm:.1f}s", flush=True)
            return True
        return False


def main():
    tag_id = int(float(os.environ.get("UDRONE_APPROACH_TAG_ID", "0")))
    standoff = float(os.environ.get("UDRONE_APPROACH_STANDOFF", "1.5"))
    depth = float(os.environ.get("UDRONE_APPROACH_DEPTH", "-1.2"))
    arrive = float(os.environ.get("UDRONE_APPROACH_ARRIVE", "0.5"))
    depth_tol = float(os.environ.get("UDRONE_APPROACH_DEPTH_TOL", "0.25"))
    speed = float(os.environ.get("UDRONE_APPROACH_SPEED", "0.25"))
    settle = float(os.environ.get("UDRONE_APPROACH_SETTLE", "3"))
    timeout = float(os.environ.get("UDRONE_APPROACH_TIMEOUT", "90"))
    acquire = float(os.environ.get("UDRONE_APPROACH_ACQUIRE", "25"))
    search_budget = os.environ.get("UDRONE_APPROACH_SEARCH_BUDGET")
    search_r0 = float(os.environ.get("UDRONE_APPROACH_SEARCH_R0", "1.0"))
    search_r_max = float(os.environ.get("UDRONE_APPROACH_SEARCH_RMAX", "8.0"))
    tag_size = float(os.environ.get("UDRONE_APPROACH_TAG_SIZE", "0.4"))
    det_topic = os.environ.get("UDRONE_APPROACH_DET_TOPIC", "/xudrone/detections")
    info_topic = os.environ.get("UDRONE_APPROACH_INFO_TOPIC", "/xudrone/camera_front/camera_info")
    ce = os.environ.get("UDRONE_APPROACH_CENTER_E")
    cn = os.environ.get("UDRONE_APPROACH_CENTER_N")
    center_e = float(ce) if ce not in (None, "") else None
    center_n = float(cn) if cn not in (None, "") else None

    rclpy.init()
    node = UdroneApproachController(
        tag_id=tag_id,
        standoff_m=standoff,
        depth_m=depth,
        arrive_m=arrive,
        depth_tol_m=depth_tol,
        speed_mps=speed,
        settle_s=settle,
        timeout_s=timeout,
        acquire_s=acquire,
        search_budget_s=float(search_budget) if search_budget not in (None, "") else None,
        search_r0_m=search_r0,
        search_r_max_m=search_r_max,
        tag_size_m=tag_size,
        detections_topic=det_topic,
        camera_info_topic=info_topic,
        center_east_m=center_e,
        center_north_m=center_n,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_approach_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.arrived and not node.failed:
            ok = True
            why = "ok"
        print(
            f"APPROACH_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why} "
            f"arrived={node.arrived} source={node.tag_source}",
            flush=True,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
