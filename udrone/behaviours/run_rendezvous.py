#!/usr/bin/env python3
"""Approach otter_k on a continuous 3D glide, lock deck tag early, weld-latch.

Geometry (see rendezvous_geom): nest at the Heron origin (between the
pontoons); continuous ILS-style glide from a deep aft tip up into the weld
standoff on boat −Z; hull plates parallel for ATTACH.

Default path: deep aft tip → climb the tapering glide → flare → fly-through
capture on the boat centreline, 0.25 m below the nest; ATTACH fires from any
stage once the hull is inside the weld gate. A capture that passes the nest
goes around to the tip. The front camera's view of tags 10 / 11 / 12
(``boat_fix``) corrects the boat position the guidance flies on; the fix is
held through the last metre, where no camera sees a tag.
``UDRONE_RDV_TERMINAL=mpc`` restores the old latch station + dock MPC.
A lost deck tag holds the last fix then rejoins the glide. After
``search_otter``, warm-starts mid-glide (skips cold tip). Phrase:
``xudrone, rendezvous and dock with otter_k``.
"""
from __future__ import annotations

import math
import os
import random
import subprocess
import sys
import time

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

from patterns import world_offset_to_path, slew_angle
from servo_geom import phase, relock_leg
import maneuver_power as mp
import dock_mpc
from boat_fix import BoatFix
from rendezvous_geom import (
    AFT_ARRIVE_M,
    AFT_STANDOFF_M,
    ATTACH_DIST_M,
    CAPTURE_ALONG_M,
    CAPTURE_HEADING_OFF,
    CAPTURE_MISS_FWD_M,
    CAPTURE_MPS,
    CONE_EXIT,
    CONE_HALF,
    CONE_RANGE_M,
    DOCK_CLIMB_MPS,
    DOCK_STANDOFF_M,
    GLIDE_FLARE_S,
    GLIDE_HEADING_OFF,
    GLIDE_LATCH_S,
    LAND_RANGE_M,
    LEARNED_BLEND,
    LEARNED_CHASE_MPS,
    LEARNED_CLOSE_RATE,
    SERVO_LOCK_CAP_M,
    axes,
    capture_missed,
    capture_reference,
    centreline_heading,
    closing_range,
    cone_rim,
    cone_state,
    dock_goal,
    gate_optical_range,
    glide_corridor,
    glide_path,
    glide_station,
    nest_body_offset,
    pull_to_nest_axis,
    servo_close_velocity,
    servo_station,
)
from track_geom import (
    CAM_Z,
    STATION_PITCH_RAD,
    TAG_ID,
    BaselineVelocity,
    station_heading,
    up_camera_tag_body,
)
from udrone_sinusoidal_controller import (
    PITCH_MAX_RAD,
    UdroneHelixController,
    body_axes,
    rpy_from_q,
    spin_until_done,
)

BUDGET_S = 200.0
LOST_S = 2.0
FOUND_HITS = 3
WELD_HOLD_S = 0.25   # inside the weld gate this long before ATTACH
ATTACH_RETRY_S = 1.5
DOCK_PITCH_RAD = math.radians(15.0)
DOCK_TOPIC = "/udrone_dock/command"
# Glide surge: allow full pitch so natural nose-down aims the up-cam forward-up.
# Flare / latch tighten to plates-parallel (DOCK_PITCH_RAD).
GLIDE_PITCH_RAD = PITCH_MAX_RAD
# Legacy alias (tests / logs); nose-up bias removed for early up-cam.
AFT_PITCH_RAD = GLIDE_PITCH_RAD
# When too shallow for the nest setpoint, open pitch and free yaw so surge
# can dive (yaw_hold + shallow pitch left thrust≈0 in the 2026-11-17 logs).
DIVE_PITCH_RAD = math.radians(42.0)
DIVE_DEPTH_ERR_M = 0.35
DIVE_VZ_MPS = -0.28
# Tag PASS / up-cam aim only once depth is in this band of the nest setpoint.
UNDER_DEPTH_BAND_M = 0.45
# Stuck steer: large horizontal error + near-zero surge → clear yaw_hold.
STUCK_HERR_M = 3.0
STUCK_THRUST = 0.025
STUCK_S = 5.0
AFT_FAR_M = 12.0
# After warm-start, hold briefly before a dark-tag spiral on the tip.
UNDER_HOLD_BEFORE_SPIRAL_S = 8.0
# Lead the glide setpoint slightly ahead of projected s (seconds of look-ahead).
GLIDE_LEAD_S = 0.35
# Keep the moving reference local and acceleration bounded.  The hull is
# underactuated; multi-metre position jumps become reverse-thrust turns.
GLIDE_CARROT_M = 0.45
GLIDE_LATERAL_K = 0.55
GLIDE_LATERAL_D = 0.35
GLIDE_LATERAL_V_MAX = 0.18
REFERENCE_ACCEL_MPS2 = 0.25
# Capture the nest axis before climbing the final few decimetres.
LATCH_CENTER_M = 0.10
LATCH_CAPTURE_ALONG_M = 0.65
# Boat tags logged per camera (VIS lines): deck 10, pontoon sterns 11 / 12.
# Measurement only; guidance still uses tag 10 on the up camera.
VIS_TAG_IDS = (10, 11, 12)
VIS_LOG_S = 0.5
TERMINAL_MPC = os.environ.get("UDRONE_RDV_TERMINAL", "flythrough").strip().lower() == "mpc"
# Front-camera boat fix. Image-to-receipt delay used to look up the hull pose
# the frame was taken from.
FRONT_TF_TOPIC = os.environ.get("UDRONE_RDV_FRONT_TF", "/xudrone/tf_front")
FRONT_LATENCY_S = float(os.environ.get("UDRONE_RDV_FRONT_LATENCY_S", "0.15"))
USE_BOAT_FIX = os.environ.get("UDRONE_RDV_BOAT_FIX", "1") != "0"
POSE_BUFFER_S = 1.0
# Test hook: constant east,north bias on the boat prior (stands in for boat GNSS).
_bias = [float(v) for v in os.environ.get("UDRONE_RDV_BOAT_BIAS", "0,0").split(",")[:2]]
BOAT_BIAS = (_bias + [0.0, 0.0])[:2]
# Centreline steering look-ahead (m) and lateral-rate damping (s).
GLIDE_LOOKAHEAD_M = 1.4
FLARE_LOOKAHEAD_M = 0.9
CAPTURE_LOOKAHEAD_M = 0.7
HEADING_DAMP_S = 0.8
FLARE_HEADING_OFF = math.radians(25.0)
CAPTURE_ENTRY_MPS = 0.25    # closing speed on the boat to start the capture
CAPTURE_ENTRY_LAT_M = 0.45  # weld sphere is 0.55 m; lateral must already fit
CAPTURE_PITCH_RAD = math.radians(20.0)
CAPTURE_PITCH_BOOST_RAD = math.radians(10.0)   # LinkAttacher gate is 30 deg
CAPTURE_MAX_S = 25.0
MAX_GOAROUNDS = 3


def _tag_side_px(corners) -> float:
    if not corners or len(corners) < 4:
        return 0.0
    pts = [(float(c.x), float(c.y)) for c in corners]
    sides = [
        math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
        for i in range(4)
    ]
    return sum(sides) / 4.0


class UdroneRendezvous(UdroneHelixController):
    def __init__(self, **kwargs):
        self.budget_s = float(os.environ.get("UDRONE_RENDEZVOUS_BUDGET", BUDGET_S))
        # Deck underside tag (up-cam) for search acquire, nest servo, and dock.
        self.tag_id = int(float(os.environ.get("UDRONE_RENDEZVOUS_TAG_ID", TAG_ID)))
        self._power = mp.from_env()
        # PPO residual defaults (env overrides for A/B). Still scaled by power.
        chase0 = float(os.environ.get("UDRONE_RDV_CHASE", LEARNED_CHASE_MPS))
        climb0 = float(os.environ.get("UDRONE_RDV_CLIMB", DOCK_CLIMB_MPS))
        close0 = float(os.environ.get("UDRONE_RDV_CLOSE_RATE", LEARNED_CLOSE_RATE))
        blend0 = float(os.environ.get("UDRONE_RDV_BLEND", LEARNED_BLEND))
        pace = max(0.08, mp.scale(chase0, self._power))
        # Fail budget and helix duration both stretch at low power so a 50%
        # rendezvous is not aborted at the full-power 200 s wall.
        self._fail_budget_s = self.budget_s / max(0.35, self._power)
        kwargs.setdefault("duration_s", self._fail_budget_s + 30.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", pace)
        kwargs.setdefault("z0", -1.0)
        super().__init__(**kwargs)
        self.power = self._power
        self._climb_mps = mp.scale(climb0, self._power)
        self._close_rate = mp.scale(close0, self._power)
        self._blend0 = max(0.35, min(1.0, blend0))
        self._spiral_omega = mp.scale(0.40, self._power)
        self._spiral_grow = mp.scale(0.12, self._power)
        self.fx = self.fy = 380.0
        self.cx, self.cy = 320.0, 240.0
        self._stage = "approach"
        self._warm_started = False
        self.get_logger().info(
            f"rendezvous power={100.0 * self._power:.0f}% "
            f"chase={pace:.2f} climb={self._climb_mps:.2f} blend0={self._blend0:.2f} "
            f"approach=glide-path"
        )
        self._search_t0 = None
        self._corridor = None
        self._des = None
        self._vel = (0.0, 0.0, 0.0)
        self._bearing = None
        self._bearing_tm = None
        self._transit_yaw = None
        self._glide_s_max = 0.0
        self._ref_rel_vel = (0.0, 0.0, 0.0)
        self._ref_vel_tm = None
        self._yaw_out = None
        self._yaw_out_tm = None
        self._boat_vel = BaselineVelocity()
        self._boat = None
        self._auv_pos = None
        self._det = None          # deck tag 10 (up-cam)
        self._hits = 0
        self._lock_range = None
        self._lock_tm = None
        self._lock_enu = None
        self._lock_enu_t = 0.0
        self._landed = False
        self._attach_t = 0.0
        self._attach_proc = None
        self._attach_n = 0
        self._cone = None
        self._dock_along = None
        self._dock_tm = None
        self._weld_tm = None
        self._dock_mpc = dock_mpc.enabled()
        self._mpc_warm = None
        self._mpc_rel_vel = None
        self._mpc_v_state = (0.0, 0.0, 0.0)  # lagged nest-frame vel for CEM plant
        self._mpc_rng = random.Random()  # one stream; Random(0) each tick repeats
        self._acc = None
        self._servo_prev = None   # (tm, station) for d(station)/dt feedforward
        self._under_hold_t0 = None
        self._stuck_t0 = None
        self._stuck_logged = False
        self._dive_logged = False
        self._depth_ok_logged = False
        self._log_t = 0.0
        self.hud_preview = False
        if self._dock_mpc:
            self.get_logger().info("rendezvous dock MPC on (nest-frame CEM, UDRONE_RDV_DOCK_MPC)")
        info = os.environ.get("UDRONE_RENDEZVOUS_INFO", "/xudrone/camera_up/camera_info")
        dets = os.environ.get("UDRONE_RENDEZVOUS_DET", "/xudrone/detections_up")
        self.create_subscription(CameraInfo, info, self._on_info, qos_profile_sensor_data)
        self._vis = {}            # (camera, id) -> [frames, side px, cx, cy]
        self._vis_t = 0.0
        self._boat_fix = BoatFix()
        self._boat_truth = None
        self._boat_prior = None
        self._capture_yaw = None
        self._pose_buf = []       # (wall time, pos, q) for frame-time lookup
        self._front_frame = None  # (wall time, [(id, optical xyz)])
        self._capture_t0 = None
        self._goarounds = 0
        self._weld_any_tm = None
        front = os.environ.get("UDRONE_RENDEZVOUS_FRONT_DET", "/xudrone/detections")
        if USE_BOAT_FIX:
            self.create_subscription(TFMessage, FRONT_TF_TOPIC, self._on_front_tf, 20)
        if AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray, dets, self._on_tag, qos_profile_sensor_data,
            )
            self.create_subscription(
                AprilTagDetectionArray, front,
                lambda m: self._note_vis("front", m), qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"rendezvous otter_k: deck_tag={self.tag_id} (up-cam) "
            f"budget={self._fail_budget_s:g}s (base={self.budget_s:g}s)"
        )
        self.get_logger().info(
            f"rendezvous terminal={'mpc' if TERMINAL_MPC else 'flythrough'} "
            f"boat_fix={'front tags 10/11/12' if USE_BOAT_FIX else 'off'} "
            f"bias=({BOAT_BIAS[0]:+.2f},{BOAT_BIAS[1]:+.2f})"
        )

    def _on_front_tf(self, m):
        """Front-camera tag poses (optical frame) from apriltag_ros."""
        now = time.time()
        seen = []
        for tr in m.transforms:
            name = tr.child_frame_id
            if not name.startswith("tag36h11:"):
                continue
            try:
                tid = int(name.split(":", 1)[1])
            except ValueError:
                continue
            if tid in VIS_TAG_IDS:
                t = tr.transform.translation
                seen.append((tid, (float(t.x), float(t.y), float(t.z))))
        if not seen:
            return
        # apriltag_ros sends one TFMessage per frame; merge a split frame.
        prev = self._front_frame
        if prev is not None and now - prev[0] < 0.05:
            ids = {s[0] for s in seen}
            seen += [s for s in prev[1] if s[0] not in ids]
        self._front_frame = (now, seen)

    def _pose_at(self, t):
        best = None
        for rec in self._pose_buf:
            if best is None or abs(rec[0] - t) < abs(best[0] - t):
                best = rec
        return best

    def _boat_estimate(self, tm, auv, boat_truth):
        """Boat pose the guidance flies on: prior (+ test bias) + front-camera offset."""
        now = time.time()
        self._boat_truth = boat_truth
        self._pose_buf.append((now, auv[0], auv[1]))
        while self._pose_buf and now - self._pose_buf[0][0] > POSE_BUFFER_S:
            self._pose_buf.pop(0)
        bp, bq = boat_truth
        prior = (float(bp[0]) + BOAT_BIAS[0], float(bp[1]) + BOAT_BIAS[1], float(bp[2]))
        self._boat_prior = prior
        frame = self._front_frame
        if USE_BOAT_FIX and frame is not None:
            self._front_frame = None
            rec = self._pose_at(frame[0] - FRONT_LATENCY_S)
            if rec is not None:
                self._boat_fix.update(tm, frame[1], rec[1], rec[2], prior, bq)
        return self._boat_fix.apply(prior), bq

    def _on_info(self, m: CameraInfo):
        k = list(m.k)
        if len(k) >= 6 and k[0] > 1.0 and k[4] > 1.0:
            self.fx, self.fy = float(k[0]), float(k[4])
            self.cx = float(k[2]) or self.cx
            self.cy = float(k[5]) or self.cy

    def _note_vis(self, cam, m):
        """Count which boat tags a camera decoded since the last VIS line."""
        for d in m.detections:
            tid = int(d.id)
            if tid not in VIS_TAG_IDS:
                continue
            rec = self._vis.setdefault((cam, tid), [0, 0.0, 0.0, 0.0])
            rec[0] += 1
            rec[1] = _tag_side_px(d.corners)
            rec[2], rec[3] = float(d.centre.x), float(d.centre.y)

    def _log_vis(self, cone):
        now = time.time()
        if now < self._vis_t:
            return
        self._vis_t = now + VIS_LOG_S
        seen = " ".join(
            f"{cam}:{tid}={rec[1]:.0f}px@({rec[2]:.0f},{rec[3]:.0f})x{rec[0]}"
            for (cam, tid), rec in sorted(self._vis.items())
        )
        self._vis.clear()
        corr = self._corridor or {}
        s_txt = f"{corr['s']:.2f}" if "s" in corr else "-"
        fix_txt = ""
        bt, be = self._boat_truth, self._boat
        if bt is not None and be is not None:
            # Guidance boat position minus Gazebo truth, boat frame (fwd, left, up).
            ef, el, eu = nest_body_offset(be[0], bt[0], bt[1])
            bf = self._boat_fix
            fix_txt = (
                f" fix_err=({ef:+.2f},{el:+.2f},{eu:+.2f}) frames={bf.frames}"
                f" tags={bf.last_tags}"
            )
            if bf.last_innovation is not None:
                fix_txt += f" innov={bf.last_innovation:.2f}"
        print(
            f"VIS {self._stage} s={s_txt} range={cone['range']:.2f} "
            f"lateral={cone['lateral']:.2f} along={cone['along']:.2f} "
            f"yaw={math.degrees(cone['yaw']):+.0f} tilt={math.degrees(cone['tilt']):.0f}"
            f"{fix_txt} | {seen or 'none'}",
            flush=True,
        )

    def _on_tag(self, m):
        """Deck underside tag (up-cam) — search / nest servo / dock."""
        self._note_vis("up", m)
        best = None
        for d in m.detections:
            if int(d.id) == self.tag_id:
                best = d
                break
        if best is None:
            return
        side = _tag_side_px(best.corners)
        if side < 6.0 or self.fx <= 1.0:
            return
        x_n = (float(best.centre.x) - self.cx) / self.fx
        y_n = (float(best.centre.y) - self.cy) / self.fy
        z_opt = self.fx * 0.40 / side
        if not math.isfinite(z_opt) or z_opt < 0.08 or z_opt > 6.0:
            return
        if not self._fresh():
            self._hits = 0      # FOUND_HITS counts one unbroken sighting
        self._det = (x_n, y_n, z_opt, time.time())
        self._hits += 1

    def _fresh(self):
        return self._det is not None and time.time() - self._det[3] <= LOST_S

    def _visual_tag(self, pos, q):
        if not self._fresh():
            return None
        x_n, y_n, z_opt, _t = self._det
        bx, by, bz = up_camera_tag_body(x_n, y_n, z_opt, CAM_Z)
        fwd, left, up = body_axes(q)
        tag = tuple(
            float(pos[i]) + bx * fwd[i] + by * left[i] + bz * up[i]
            for i in range(3)
        )
        self._lock_enu = tag
        self._lock_enu_t = time.time()
        return tag

    def _relock_goal(self, auv):
        """Step under the last tag. None once that fix is too old to trust."""
        if self._lock_enu is None:
            return None
        age = time.time() - self._lock_enu_t
        if phase(age) != "relock":
            return None
        if self._stage != "relock":
            self._stage = "relock"
            fix = self._lock_enu
            print(
                f"RENDEZVOUS_RELOCK age={age:.1f}s "
                f"tag=({fix[0]:.1f},{fix[1]:.1f},{fix[2]:.2f})",
                flush=True,
            )
        pos = auv[0]
        leg = relock_leg(pos, self._lock_enu, "up")
        z = min(-0.55, max(-1.60, float(pos[2])))
        return (leg[0], leg[1], z), (0.0, 0.0, 0.0)

    def _corridor_prev(self):
        """Map runner stage → glide sticky phase."""
        s = (self._stage or "").lower()
        if s in ("latch", "dock"):
            return "latch"
        if s in ("flare", "servo"):
            return "flare"
        if s in ("glide", "commit", "under", "relock"):
            return "glide"
        if s in ("approach", "aft"):
            return "approach"
        return None

    def _ready_to_dock(self, cone=None):
        """Dock MPC / ATTACH once flared/latched with up-cam (or already docking)."""
        if self._stage == "dock":
            return True
        if self._stage == "latch":
            c = cone or {}
            # The acceptance cone is intentionally broad for capture, but
            # entering dock MPC at 0.3--0.4 m lateral locks yaw to the boat
            # before a body-x-only hull can center.  Finish capture first.
            return (
                float(c.get("lateral", 99.0)) <= 0.12
                and abs(float(c.get("yaw", math.pi))) <= math.radians(28.0)
                and float(c.get("tilt", math.pi)) <= math.radians(35.0)
            )
        if self._stage == "flare" and self._hits >= FOUND_HITS:
            return True
        # Legacy servo name still counts if a warm container maps it.
        return self._stage == "servo" and self._hits >= FOUND_HITS

    def _tag_weld_ok(self, cone) -> bool:
        """Deck-tag relative pose already inside the LinkAttacher ATTACH gate.

        Nest-frame ``cone['weld']`` plus a fresh up-cam lock. Not from far tip.
        """
        if self._stage in ("approach", "aft"):
            return False
        if self._hits < FOUND_HITS or not self._fresh() or self._det is None:
            return False
        if not cone.get("weld"):
            return False
        # Deck tag sits ~0.25 m above the nest; optical should match weld range.
        return float(self._det[2]) <= ATTACH_DIST_M + 0.25

    def _z_des(self, fallback=None):
        if self._des is not None:
            return float(self._des[2])
        return None if fallback is None else float(fallback)

    def _depth_ok(self, pos=None, z_des=None) -> bool:
        """True when hull depth is close enough to the nest setpoint for up-cam."""
        p = pos if pos is not None else self._auv_pos
        zd = self._z_des(z_des)
        if p is None or zd is None:
            return False
        return abs(float(p[2]) - zd) <= UNDER_DEPTH_BAND_M

    def _dive_needed(self, pos, z_des=None) -> bool:
        """Too shallow vs nest setpoint — need open pitch / free yaw to dive."""
        zd = self._z_des(z_des)
        if pos is None or zd is None:
            return False
        return float(pos[2]) - zd >= DIVE_DEPTH_ERR_M

    def _dive_rel_vel(self, pos, z_des=None):
        """Extra downward nest-relative velocity while climbing down to depth."""
        if not self._dive_needed(pos, z_des=z_des):
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, DIVE_VZ_MPS)

    def _apply_stuck_steer(self, tm, prefer_yaw):
        """If large horizontal error and near-zero surge, free the nose."""
        herr = float(getattr(self, "_last_herr", 0.0) or 0.0)
        thrust = float(getattr(self, "_last_thrust", 1.0) or 0.0)
        if herr >= STUCK_HERR_M and abs(thrust) < STUCK_THRUST:
            if self._stuck_t0 is None:
                self._stuck_t0 = tm
            elif tm - self._stuck_t0 >= STUCK_S:
                if not self._stuck_logged:
                    print(
                        f"STUCK_STEER herr={herr:.1f}m thrust={thrust:+.3f} "
                        f"→ clear yaw_hold open pitch",
                        flush=True,
                    )
                    self._stuck_logged = True
                return None, PITCH_MAX_RAD
        else:
            self._stuck_t0 = None
            self._stuck_logged = False
        return prefer_yaw

    def _nest_attitude(self, tm, pos, boat, *, prefer_boat_yaw: bool, cruise_pitch):
        """Yaw/pitch for corridor and under-nest: dive authority + stuck steer.

        Returns nothing; sets ``self.yaw_hold`` / ``self.pitch_limit``.
        """
        prefer = None
        pitch = cruise_pitch
        if prefer_boat_yaw and boat is not None and not self._dive_needed(pos):
            prefer = rpy_from_q(boat[1])[2]
            pitch = cruise_pitch
        elif self._dive_needed(pos):
            prefer = None
            pitch = DIVE_PITCH_RAD
            if not self._dive_logged:
                derr = float(pos[2]) - float(self._des[2]) if self._des is not None else 0.0
                print(
                    f"NEST_DIVE derr={derr:+.2f}m pitch≤{math.degrees(DIVE_PITCH_RAD):.0f}deg "
                    f"yaw free",
                    flush=True,
                )
                self._dive_logged = True
        else:
            self._dive_logged = False

        stuck = self._apply_stuck_steer(tm, prefer)
        if isinstance(stuck, tuple):
            self.yaw_hold, self.pitch_limit = stuck
        else:
            self.yaw_hold = stuck
            self.pitch_limit = pitch
        if self._depth_ok(pos) and not self._depth_ok_logged:
            print(
                f"NEST_DEPTH_OK z={pos[2]:.2f} des={self._des[2]:.2f} "
                f"(band±{UNDER_DEPTH_BAND_M:g}m)",
                flush=True,
            )
            self._depth_ok_logged = True
        elif not self._depth_ok(pos):
            self._depth_ok_logged = False

    def _drop_to_search(self, tm, boat):
        """Lost the up-cam: rejoin the glide (tip if far), boat-frame loiter."""
        self._lock_range = None
        self._lock_tm = None
        self._hits = 0
        self._servo_prev = None
        self._search_t0 = tm
        self._under_hold_t0 = tm
        tip = glide_station(0.0, boat[0], boat[1])["point"]
        corr = glide_corridor(
            self._auv_pos or tip, boat[0], boat[1],
            prev_phase=self._corridor_prev(),
        )
        self._stage = "approach" if corr["phase"] == "approach" else "glide"
        print(f"RENDEZVOUS_LOST back to {self._stage}", flush=True)
        return self._corridor_goal(tm, boat)

    def _glide_setpoint(self, corr, auv_pose, boat, tm):
        """Lead along the glide centreline; blend toward tag when up-cam is fresh."""
        s_meas = float(corr["s"])
        self._glide_s_max = max(self._glide_s_max, s_meas)
        # Boat motion can move the projection slightly backwards.  Preserve
        # progress, with only a tiny allowance for local tube recapture.
        s = max(s_meas, self._glide_s_max - 0.02)
        speed = float(corr["speed_mps"])
        half = max(0.05, float(corr.get("half_m", 0.5)))
        lat = float(corr.get("lat", 0.0))
        if lat > half:
            # Spend thrust recapturing the tube instead of charging forward.
            speed *= max(0.25, min(1.0, half / max(lat, 1e-3)))
        lead = min(0.15, GLIDE_LEAD_S * speed / max(0.2, corr["length_m"]))
        s_cmd = min(1.0, s + lead)
        st = glide_station(s_cmd, boat[0], boat[1])
        point = st["point"]
        tangent = corr["tangent"]
        rel_vel = tuple(speed * float(tangent[i]) for i in range(3))
        pos = auv_pose[0] if auv_pose is not None else self._auv_pos
        # Damped vector-field capture of the local centreline.  This acts at
        # the current glide progress; it never sends the hull back to the tip.
        if pos is not None:
            closest = corr.get("closest", glide_station(s, boat[0], boat[1])["point"])
            lat_vec = tuple(float(pos[i]) - float(closest[i]) for i in range(3))
            world_v = tuple(float(v) for v in getattr(self, "vel", (0.0, 0.0, 0.0)))
            boat_v = tuple(float(v) for v in self._boat_vel.vel)
            rel_meas = tuple(world_v[i] - boat_v[i] for i in range(3))
            along_v = sum(rel_meas[i] * float(tangent[i]) for i in range(3))
            lat_v = tuple(rel_meas[i] - along_v * float(tangent[i]) for i in range(3))
            correction = tuple(
                -GLIDE_LATERAL_K * lat_vec[i] - GLIDE_LATERAL_D * lat_v[i]
                for i in range(3)
            )
            cn = math.sqrt(sum(c * c for c in correction))
            if cn > GLIDE_LATERAL_V_MAX:
                correction = tuple(c * GLIDE_LATERAL_V_MAX / cn for c in correction)
            rel_vel = tuple(rel_vel[i] + correction[i] for i in range(3))
        # Early up-cam: soft-blend nest pull / servo station while still gliding.
        if (
            self._hits >= FOUND_HITS and self._fresh() and self._det is not None
            and auv_pose is not None
        ):
            tag = self._visual_tag(auv_pose[0], auv_pose[1])
            if tag is not None:
                if self._lock_range is None:
                    seen = gate_optical_range(
                        float(self._det[2]),
                        (self._cone or {}).get("along"),
                    )
                    self._lock_range = max(LAND_RANGE_M, min(SERVO_LOCK_CAP_M, seen))
                    self._lock_tm = tm
                    print(
                        f"RENDEZVOUS_LOCK early s={s:.2f} range={self._lock_range:.2f} "
                        f"seen={float(self._det[2]):.2f}",
                        flush=True,
                    )
                elapsed = tm - (self._lock_tm or tm)
                optical = closing_range(
                    self._lock_range, elapsed, LAND_RANGE_M, rate=self._close_rate,
                )
                closed = max(0.0, self._lock_range - optical)
                span = max(0.5, self._lock_range - LAND_RANGE_M)
                blend = min(
                    1.0,
                    self._blend0 + (1.0 - self._blend0) * min(1.0, closed / span),
                )
                # Weight tag residual more as s grows (still on the glide).
                w_tag = min(0.85, 0.25 + 0.75 * s)
                tag_st = servo_station(
                    tag, optical, boat[0], boat[1], CAM_Z, blend=blend,
                )
                point = tuple(
                    (1.0 - w_tag) * point[i] + w_tag * float(tag_st[i])
                    for i in range(3)
                )
                point = pull_to_nest_axis(point, boat[0], boat[1], blend=0.35 + 0.4 * s)
                tag_v = servo_close_velocity(
                    boat[1], self._lock_range, elapsed, LAND_RANGE_M, rate=self._close_rate,
                )
                rel_vel = tuple(
                    (1.0 - w_tag) * rel_vel[i] + w_tag * float(tag_v[i])
                    for i in range(3)
                )
        if pos is not None:
            delta = tuple(float(point[i]) - float(pos[i]) for i in range(3))
            dn = math.sqrt(sum(d * d for d in delta))
            if dn > GLIDE_CARROT_M:
                point = tuple(
                    float(pos[i]) + delta[i] * GLIDE_CARROT_M / dn
                    for i in range(3)
                )
        dive = self._dive_rel_vel(pos, z_des=point[2])
        rel_vel = tuple(rel_vel[i] + dive[i] for i in range(3))
        return point, rel_vel

    def _approach_transit(self, pos, tip, tm):
        """Surge toward the glide tip with the nose held on that bearing.

        The old command was the tip itself plus a pure dive. Horizontal motion
        then came from the position correction, and the free nose followed
        whichever way that small horizontal force pointed.
        """
        dx = float(tip[0]) - float(pos[0])
        dy = float(tip[1]) - float(pos[1])
        dist_h = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if dist_h >= 0.40 or self._bearing is None else self._bearing
        dt = 0.05 if self._bearing_tm is None else float(tm) - float(self._bearing_tm)
        self._bearing_tm = float(tm)
        self._bearing = slew_angle(self._bearing, bearing, dt, rate=0.45)
        yaw = self._bearing
        vh = min(0.32, dist_h * 0.40)
        vz = self._dive_rel_vel(pos, z_des=float(tip[2]))[2]
        dz = float(tip[2]) - float(pos[2])
        if abs(vz) < 1e-6 and dz > 0.15:
            # Below the tip (e.g. leaving the hoop deep): climb in the
            # velocity, not only through the position correction.
            vz = min(0.28, 0.7 * dz)
        # Never put the carrot past the tip.  Keep both horizontal and vertical
        # references local: the old full-depth setpoint made the first error
        # jump above 2 m and produced a 40 N launch impulse.
        lead = min(dist_h, 0.65, max(0.25, dist_h * 0.25))
        z_carrot = float(pos[2]) + max(-0.35, min(0.35, dz))
        point = (
            float(pos[0]) + lead * math.cos(yaw),
            float(pos[1]) + lead * math.sin(yaw),
            z_carrot,
        )
        self._transit_yaw = yaw
        return point, (vh * math.cos(yaw), vh * math.sin(yaw), vz)

    def _corridor_goal(self, tm, boat, auv_pose=None):
        """Deep tip → continuous glide → flare hold; dark-tag spiral near tip."""
        pos = self._auv_pos
        if pos is None:
            pos = glide_station(0.0, boat[0], boat[1])["point"]
        corr = glide_corridor(pos, boat[0], boat[1], prev_phase=self._corridor_prev())
        self._corridor = corr
        phase = corr["phase"]
        if not TERMINAL_MPC:
            if self._stage == "latch":
                return self._capture_goal(tm, boat, corr)
            f_h, l_h, _u = nest_body_offset(pos, boat[0], boat[1])
            if self._stage in ("glide", "flare"):
                if f_h > CAPTURE_MISS_FWD_M:
                    return self._go_around(tm, boat, "overshot")
                heading_ok = abs(float((self._cone or {}).get("yaw", 0.0))) <= math.radians(45.0)
                # Enter only slow and near the line: at glide speed the hull
                # crosses the weld sphere before the gate can hold.
                if (
                    heading_ok
                    and (phase == "latch" or float(corr["s_raw"]) >= GLIDE_LATCH_S)
                    and abs(l_h) <= CAPTURE_ENTRY_LAT_M
                    and f_h <= 0.2
                    and self._closing_rate(boat) <= CAPTURE_ENTRY_MPS
                ):
                    return self._capture_goal(tm, boat, corr)
                if phase == "latch":
                    # Not slow or centred enough yet: keep flying the flare.
                    return self._glide_setpoint(corr, auv_pose, boat, tm)
        if phase == "approach":
            if self._stage != "approach":
                self._stage = "approach"
                self._search_t0 = None
                self._under_hold_t0 = None
                print(
                    f"RENDEZVOUS_APPROACH tip=({corr['tip'][0]:.1f},{corr['tip'][1]:.1f},"
                    f"{corr['tip'][2]:.2f}) dist={corr['dist_tip']:.1f}m",
                    flush=True,
                )
            tip = corr["tip"]
            return self._approach_transit(pos, tip, tm)
        if phase == "glide":
            if self._stage != "glide":
                self._stage = "glide"
                self._search_t0 = None
                self._under_hold_t0 = None
                print(
                    f"RENDEZVOUS_GLIDE s={corr['s']:.2f} lat={corr['lat']:.2f} "
                    f"speed={corr['speed_mps']:.2f}",
                    flush=True,
                )
            return self._glide_setpoint(corr, auv_pose, boat, tm)
        if phase == "flare":
            if self._stage not in ("flare", "latch", "dock"):
                self._stage = "flare"
                print(
                    f"RENDEZVOUS_FLARE s={corr['s']:.2f} lat={corr['lat']:.2f}",
                    flush=True,
                )
            return self._glide_setpoint(corr, auv_pose, boat, tm)
        # latch phase geometrically — hold threshold until dock MPC takes over.
        if self._stage not in ("latch", "dock"):
            self._stage = "latch"
            print(
                f"RENDEZVOUS_LATCH s={corr['s']:.2f} dist_thr={corr['dist_threshold']:.2f}",
                flush=True,
            )
        thr = corr["threshold"]
        cone = self._cone or {}
        lateral = float(cone.get("lateral", corr.get("lat", 0.0)))
        along_live = max(DOCK_STANDOFF_M, float(cone.get("along", DOCK_STANDOFF_M)))
        # Do not climb into the deck while still displaced sideways.  Blend
        # continuously between a 0.65 m axis-capture station and the weld
        # standoff, then let dock MPC take over once the cone is valid.
        capture_w = max(
            0.0,
            min(1.0, (lateral - 0.04) / max(0.01, LATCH_CENTER_M - 0.04)),
        )
        along_goal = DOCK_STANDOFF_M + capture_w * (
            LATCH_CAPTURE_ALONG_M - DOCK_STANDOFF_M
        )
        along_goal = max(along_goal, min(along_live, LATCH_CAPTURE_ALONG_M) * capture_w)
        nest = corr["nest"]
        down = cone.get("down")
        if down is None:
            v = tuple(float(thr[i]) - float(nest[i]) for i in range(3))
            vn = math.sqrt(sum(c * c for c in v)) or 1.0
            down = tuple(c / vn for c in v)
        point = tuple(float(nest[i]) + along_goal * float(down[i]) for i in range(3))
        err = tuple(point[i] - float(pos[i]) for i in range(3))
        rel_vel = tuple(0.55 * e for e in err)
        vn = math.sqrt(sum(v * v for v in rel_vel))
        vmax = 0.16 if lateral > LATCH_CENTER_M else 0.10
        if vn > vmax:
            rel_vel = tuple(v * vmax / vn for v in rel_vel)
        return point, rel_vel

    def _closing_rate(self, boat):
        """Hull speed along the boat's heading, relative to the boat (m/s)."""
        boat_v = self._boat_vel.vel
        fwd = axes(boat[1])[0]
        return sum((float(self.vel[i]) - float(boat_v[i])) * fwd[i] for i in range(2))

    def _centreline_yaw(self, pos, boat, lookahead, max_off):
        """Bounded heading onto the boat centreline, damped by the sideways rate."""
        boat_v = self._boat_vel.vel
        rel = tuple(float(self.vel[i]) - float(boat_v[i]) for i in range(2))
        left = axes(boat[1])[1]
        lat_rate = rel[0] * left[0] + rel[1] * left[1]
        yaw, _l = centreline_heading(
            pos, boat[0], boat[1], lookahead, max_off,
            lat_rate=lat_rate, damp_s=HEADING_DAMP_S,
        )
        return yaw

    def _capture_goal(self, tm, boat, corr):
        """Fly through the weld sphere on the centreline; go around on a miss."""
        pos = self._auv_pos
        if self._stage != "latch" or self._capture_t0 is None:
            self._stage = "latch"
            self._capture_t0 = tm
            f_h, l_h, u_h = nest_body_offset(pos, boat[0], boat[1])
            print(
                f"RENDEZVOUS_LATCH capture s={corr['s']:.2f} fwd={f_h:+.2f} "
                f"left={l_h:+.2f} below={-u_h:.2f} dist_thr={corr['dist_threshold']:.2f}",
                flush=True,
            )
        missed = capture_missed(pos, boat[0], boat[1])
        if missed is None and self._capture_t0 is not None and tm - self._capture_t0 > CAPTURE_MAX_S:
            missed = "slow"
        if missed is not None:
            return self._go_around(tm, boat, missed)
        yaw = self._centreline_yaw(pos, boat, CAPTURE_LOOKAHEAD_M, CAPTURE_HEADING_OFF)
        self._capture_yaw = yaw
        return capture_reference(pos, boat[0], boat[1], yaw, speed=CAPTURE_MPS * self._power_scale())

    def _power_scale(self):
        return max(0.35, min(1.0, float(getattr(self, "power", 1.0) or 1.0)))

    def _go_around(self, tm, boat, why):
        f_h, l_h, u_h = nest_body_offset(self._auv_pos, boat[0], boat[1])
        self._goarounds += 1
        print(
            f"RENDEZVOUS_GOAROUND #{self._goarounds} {why} fwd={f_h:+.2f} "
            f"left={l_h:+.2f} below={-u_h:.2f}",
            flush=True,
        )
        if self._goarounds > MAX_GOAROUNDS:
            self._fail(f"missed the nest {self._goarounds} times ({why})")
        self._stage = "approach"
        self._capture_t0 = None
        self._glide_s_max = 0.0
        self._weld_any_tm = None
        self._bearing = None
        corr = glide_corridor(self._auv_pos, boat[0], boat[1], prev_phase="approach")
        self._corridor = corr
        return self._approach_transit(self._auv_pos, corr["tip"], tm)

    def _slew_reference_velocity(self, tm, target):
        """Acceleration-limit the nest-relative reference across all glide phases."""
        target = tuple(float(v) for v in target)
        if self._ref_vel_tm is None:
            self._ref_vel_tm = float(tm)
            # Start from rest after arming/hand-off.  Publishing the full
            # 0.32/-0.28 m/s command on the first tick still made a ~42 N
            # impulse even after the position carrot was shortened.
            dt = 0.05
        else:
            dt = min(0.25, max(1e-3, float(tm) - self._ref_vel_tm))
            self._ref_vel_tm = float(tm)
        delta = tuple(target[i] - self._ref_rel_vel[i] for i in range(3))
        dn = math.sqrt(sum(d * d for d in delta))
        cap = REFERENCE_ACCEL_MPS2 * dt
        if dn > cap > 0.0:
            delta = tuple(d * cap / dn for d in delta)
        new = tuple(self._ref_rel_vel[i] + delta[i] for i in range(3))
        acc = tuple((new[i] - self._ref_rel_vel[i]) / dt for i in range(3))
        self._ref_rel_vel = new
        return new, acc
    def _servo_goal(self, tm, auv, boat):
        pos, q = auv
        tag = self._visual_tag(pos, q)
        if tag is None or boat is None:
            return None
        if self._lock_range is None:
            seen = float(self._det[2])
            # Gate foreshortened optical range with nest-axis along when known.
            along = None
            if self._cone is not None:
                along = self._cone.get("along")
            elif self._auv_pos is not None:
                along = cone_state(pos, q, boat[0], boat[1]).get("along")
            seen = gate_optical_range(seen, along)
            self._lock_range = max(LAND_RANGE_M, min(SERVO_LOCK_CAP_M, seen))
            self._lock_tm = tm
            self._servo_prev = None
            self._stage = "servo"
            print(
                f"RENDEZVOUS_LOCK range={self._lock_range:.2f} "
                f"seen={float(self._det[2]):.2f} gated={seen:.2f} "
                f"level=({self._det[0]:+.2f},{self._det[1]:+.2f})",
                flush=True,
            )
        elapsed = tm - self._lock_tm
        optical = closing_range(
            self._lock_range, elapsed, LAND_RANGE_M, rate=self._close_rate,
        )
        # Tag finds the deck; nest axis keeps the hull between the pontoons.
        # PPO preferred ~0.72 nest pull early; rise to 1.0 as optical range closes.
        # Heading alignment is yaw_hold in _control — do not lead off the nest axis.
        closed = max(0.0, self._lock_range - optical)
        span = max(0.5, self._lock_range - LAND_RANGE_M)
        blend = min(1.0, self._blend0 + (1.0 - self._blend0) * min(1.0, closed / span))
        station = servo_station(tag, optical, boat[0], boat[1], CAM_Z, blend=blend)
        # Velocity feedforward: prefer finite-difference of the station; fall
        # back to the optical close schedule along boat −Z. FD is world-frame
        # and already includes nest translation with the boat — subtract the
        # boat baseline so _update's boat_v + rel_vel does not double-count.
        rel_vel = servo_close_velocity(
            boat[1], self._lock_range, elapsed, LAND_RANGE_M, rate=self._close_rate,
        )
        if self._servo_prev is not None:
            tm0, st0 = self._servo_prev
            dt = tm - tm0
            if 1e-3 < dt < 0.5:
                fd = tuple((station[i] - st0[i]) / dt for i in range(3))
                bv = self._boat_vel.vel
                rel_vel = tuple(fd[i] - float(bv[i]) for i in range(3))
                n = math.sqrt(sum(c * c for c in rel_vel))
                if n > 0.85:
                    rel_vel = tuple(c * 0.85 / n for c in rel_vel)
        self._servo_prev = (tm, station)
        return station, rel_vel

    def _dock_goal(self, tm, cone):
        """Climb the nest axis (boat -Z); ATTACH in the weld gate, plates parallel.

        With dock MPC (default): receding nest-frame plan for position + velocity.
        Fallback: classical gated ``dock_goal`` climb.
        """
        if self._stage != "dock":
            prev = self._stage
            self._stage = "dock"
            self._dock_along = max(DOCK_STANDOFF_M, cone.get("along", cone["range"]))
            self._dock_tm = tm
            self._weld_tm = None
            self._mpc_warm = None
            self._acc = None
            print(
                f"RENDEZVOUS_CONE from={prev} range={cone['range']:.2f} "
                f"along={cone.get('along', 0):.2f} lateral={cone.get('lateral', 0):.2f} "
                f"off={math.degrees(cone['off_axis']):.0f}deg "
                f"yaw={math.degrees(cone['yaw']):+.0f}deg "
                f"tilt={math.degrees(cone['tilt']):.0f}deg "
                f"mpc={'on' if self._dock_mpc else 'off'}",
                flush=True,
            )
        dt = max(1e-3, tm - self._dock_tm) if self._dock_tm is not None else 0.05
        rel_vel = (0.0, 0.0, 0.0)
        if self._dock_mpc and cone.get("pos") is not None:
            step = dock_mpc.plan(
                cone, climb_mps=self._climb_mps, warm=self._mpc_warm,
                v_state=self._mpc_v_state, rng=self._mpc_rng, dt=dt,
            )
            point = step["point"]
            rel_vel = step["vel"]
            self._dock_along = step["along"]
            self._mpc_warm = step["warm"]
            self._mpc_v_state = step.get("v_lag", self._mpc_v_state)
            # Accel feedforward from lagged velocity change (world, nest-relative).
            if self._acc is None:
                self._acc = (0.0, 0.0, 0.0)
            else:
                prev = getattr(self, "_mpc_rel_vel", (0.0, 0.0, 0.0))
                self._acc = tuple(
                    (rel_vel[i] - prev[i]) / max(dt, 1e-3) for i in range(3)
                )
            self._mpc_rel_vel = rel_vel
            if time.time() >= self._log_t:
                va, vf, vl = step["u0"]
                print(
                    f"RENDEZVOUS_MPC along={step['along']:.2f} "
                    f"u=({va:+.2f},{vf:+.2f},{vl:+.2f}) "
                    f"vlag=({step['v_lag'][0]:+.2f},{step['v_lag'][1]:+.2f},"
                    f"{step['v_lag'][2]:+.2f}) cost={step['cost']:.1f}",
                    flush=True,
                )
        else:
            point, self._dock_along = dock_goal(
                cone, self._dock_along, dt, climb_mps=self._climb_mps,
            )
            self._acc = None
            self._mpc_v_state = (0.0, 0.0, 0.0)
        self._dock_tm = tm
        if cone["weld"]:
            if self._weld_tm is None:
                self._weld_tm = tm
            elif tm - self._weld_tm >= WELD_HOLD_S:
                self._request_attach(cone)
        else:
            self._weld_tm = None
        return point, rel_vel

    def _leave_dock(self, cone):
        print(
            f"RENDEZVOUS_CONE out range={cone['range']:.2f} "
            f"off={math.degrees(cone['off_axis']):.0f}deg "
            f"yaw={math.degrees(cone['yaw']):+.0f}deg",
            flush=True,
        )
        # Rejoin flare (up-cam still fresh) or relock — no hard servo mode flip.
        self._stage = "flare" if self._fresh() else "relock"
        self._weld_tm = None
        self._mpc_warm = None
        self._acc = None
        self._mpc_rel_vel = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._servo_prev = None

    def _request_attach(self, cone):
        """Send ATTACH without blocking the control loop; retry while still in the gate."""
        if self._landed or time.time() < self._attach_t:
            return
        if self._attach_proc is not None and self._attach_proc.poll() is None:
            return
        self._attach_t = time.time() + ATTACH_RETRY_S
        self._attach_n += 1
        env = {**os.environ, "GZ_PARTITION": os.environ.get("GZ_PARTITION", "aquatic")}
        try:
            self._attach_proc = subprocess.Popen(
                ["gz", "topic", "-t", DOCK_TOPIC, "-m", "gz.msgs.StringMsg",
                 "-p", 'data: "ATTACH"'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
            )
        except OSError as ex:
            print(f"RENDEZVOUS_ATTACH failed {ex}", flush=True)
            return
        print(
            f"RENDEZVOUS_ATTACH try={self._attach_n} range={cone['range']:.2f} "
            f"tilt={math.degrees(cone['tilt']):.0f}deg",
            flush=True,
        )

    def _approach_goal(self, tm, auv, boat):
        """Glide/flare with early up-cam blend, or relock on the last fix."""
        seen = self._fresh()
        if self._stage in ("approach", "aft", "glide", "flare", "commit", "under", "latch"):
            # Up-cam residual is blended inside _glide_setpoint — no stage flip.
            return self._corridor_goal(tm, boat, auv_pose=auv)
        if seen:
            if self._stage == "relock":
                self._stage = "flare"
            # Brief tag reacquire after relock: still fly the glide.
            return self._corridor_goal(tm, boat, auv_pose=auv)
        return self._relock_goal(auv) or self._drop_to_search(tm, boat)

    def _update(self, tm, auv, boat):
        if self.latch.state == "attached":
            if not self._landed:
                print(f"RENDEZVOUS_LANDED after {self._attach_n} ATTACH", flush=True)
            self._landed = True
            return
        if boat is None:
            return
        self._auv_pos = auv[0]
        self._boat = boat
        # Refresh boat baseline before servo FD so nest-relative feedforward
        # can subtract the same boat_v that _vel will add below. The prior,
        # not the vision-corrected pose: offset steps are not boat motion.
        bp = self._boat_prior or boat[0]
        self._boat_vel.update(tm, bp[0], bp[1])
        # Chaining after search_otter: land mid-glide only when actually in the
        # tube. Clamped projection s≈1 with huge lat must NOT skip to latch
        # (live bug: WARM start=latch s=1.00 dist_thr=21m).
        if not self._warm_started:
            self._warm_started = True
            corr0 = glide_corridor(auv[0], boat[0], boat[1], prev_phase=None)
            dist_thr = float(corr0["dist_threshold"])
            on_glide = (
                (
                    corr0.get("near_tube")
                    and corr0["phase"] in ("glide", "flare", "latch")
                )
                # Immediately after detach the hull can be centimetres from
                # the weld but outside the narrow projected glide tube.  A
                # cold-tip restart here produced a 2.2 m reference jump.
                or dist_thr <= 0.75
            ) and dist_thr <= AFT_STANDOFF_M + 1.5
            if on_glide:
                # Latch only when already in the weld tube; flare only near the
                # nest (≤2.5 m). Far "near_tube" warm-starts mid-glide only.
                if corr0["phase"] == "latch" or (
                    corr0["s"] >= GLIDE_LATCH_S and dist_thr <= 1.0
                ):
                    self._stage = "latch"
                elif dist_thr <= 2.5 and (
                    corr0["phase"] == "flare" or corr0["s"] >= GLIDE_FLARE_S
                ):
                    self._stage = "flare"
                else:
                    self._stage = "glide"
                self._corridor = corr0
                self._under_hold_t0 = tm
                print(
                    f"RENDEZVOUS_WARM start={self._stage} s={corr0['s']:.2f} "
                    f"lat={corr0['lat']:.2f} dist_thr={corr0['dist_threshold']:.1f}m "
                    f"(skip tip)",
                    flush=True,
                )
            else:
                print(
                    f"RENDEZVOUS_WARM skip s={corr0['s']:.2f} lat={corr0['lat']:.2f} "
                    f"dist_thr={corr0['dist_threshold']:.1f}m phase={corr0['phase']} "
                    f"→ cold tip",
                    flush=True,
                )
        docking = self._stage == "dock"
        cone = cone_state(auv[0], auv[1], boat[0], boat[1], CONE_EXIT if docking else 1.0)
        cone["pos"] = auv[0]
        self._cone = cone
        self._log_vis(cone)
        if not TERMINAL_MPC:
            self._weld_attach(tm, cone)
            goal = self._approach_goal(tm, auv, boat)
            self._finish_update(tm, cone, boat, goal)
            return
        # Tag already in weld gate → dock. Else flare/latch + cone → dock MPC.
        tag_weld = self._tag_weld_ok(cone)
        corr_now = self._corridor or glide_corridor(
            auv[0], boat[0], boat[1], prev_phase=self._corridor_prev(),
        )
        flare_ready = (
            corr_now.get("near_tube")
            and corr_now.get("s", 0.0) >= GLIDE_FLARE_S
            and float(corr_now.get("dist_threshold", 99.0)) <= 2.5
            and self._hits >= FOUND_HITS
            and cone["inside"]
        )
        if tag_weld or (cone["inside"] and self._ready_to_dock(cone)) or flare_ready:
            if tag_weld and self._stage != "dock":
                print(
                    f"RENDEZVOUS_WELD tag pose in latch gate → dock "
                    f"range={cone['range']:.2f} opt={float(self._det[2]):.2f} "
                    f"yaw={math.degrees(cone['yaw']):+.0f}deg "
                    f"tilt={math.degrees(cone['tilt']):.0f}deg",
                    flush=True,
                )
            goal = self._dock_goal(tm, cone)
        else:
            if docking:
                self._leave_dock(cone)
            goal = self._approach_goal(tm, auv, boat)
            self._acc = None
        self._finish_update(tm, cone, boat, goal)

    def _weld_attach(self, tm, cone):
        """ATTACH whenever the weld gate holds, whatever the stage."""
        if self._stage in ("approach", "aft") or not cone["weld"]:
            self._weld_any_tm = None
            return
        if self._weld_any_tm is None:
            self._weld_any_tm = tm
            print(
                f"RENDEZVOUS_WELD gate stage={self._stage} range={cone['range']:.2f} "
                f"lateral={cone['lateral']:.2f} yaw={math.degrees(cone['yaw']):+.0f}deg "
                f"tilt={math.degrees(cone['tilt']):.0f}deg",
                flush=True,
            )
        elif tm - self._weld_any_tm >= WELD_HOLD_S:
            self._request_attach(cone)

    def _finish_update(self, tm, cone, boat, goal):
        if goal is None:
            return
        station, rel_vel = goal
        self._des = station
        if self._stage != "dock":
            rel_vel, self._acc = self._slew_reference_velocity(tm, rel_vel)
        else:
            self._ref_rel_vel = tuple(float(v) for v in rel_vel)
            self._ref_vel_tm = float(tm)
        # Nest rides with the boat; dock MPC / servo add nest-relative velocity.
        bp = self._boat_prior or boat[0]
        boat_v = self._boat_vel.update(tm, bp[0], bp[1])
        self._vel = tuple(boat_v[i] + float(rel_vel[i]) for i in range(3))
        if time.time() >= self._log_t:
            self._log_t = time.time() + 2.0
            print(
                f"RENDEZVOUS {self._stage} des=({self._des[0]:.1f},{self._des[1]:.1f},{self._des[2]:.2f}) "
                f"hits={self._hits} cone_range={cone['range']:.2f} "
                f"lateral={cone['lateral']:.2f} "
                f"off={math.degrees(cone['off_axis']):.0f}deg",
                flush=True,
            )

    def _hud_layers(self):
        out = UdroneHelixController._hud_layers(self)
        # Front camera while it can see the boat's tags; up camera under the boat.
        out["camera"] = "front" if self._stage in ("approach", "aft", "glide") else "up"
        out["stage"] = self._stage
        s_txt = ""
        if self._corridor is not None and "s" in self._corridor:
            s_txt = f"  s={self._corridor['s']:.2f}"
        out["lines"].append(
            f"deck tag {self.tag_id} hits {self._hits}  "
            f"attach tries {self._attach_n}{s_txt}"
        )
        if self._des is not None:
            out["markers"].append({"pos": self._des, "label": "goal", "color": (0, 200, 255)})
        if self._lock_enu is not None:
            out["markers"].append({"pos": self._lock_enu, "label": "tag fix", "color": (255, 200, 0),
                                   "shape": "diamond"})
        boat = getattr(self, "_boat", None)
        cone = self._cone
        if boat is not None:
            gp = glide_path(
                boat[0], boat[1],
                inside=bool(cone and cone.get("inside")),
                weld=bool(cone and cone.get("weld")),
            )
            out["paths"].extend(gp["paths"])
            out["markers"].extend(gp["markers"])
            out["cone"] = gp["cone"]
        elif cone is not None:
            # Fallback if boat pose was never cached: rim only.
            ok = (0, 220, 0) if cone["inside"] else (0, 200, 255)
            if cone["weld"]:
                ok = (255, 255, 255)
            rim = cone_rim(cone["nest"], cone["down"])
            out["paths"].append({"pts": rim, "color": ok, "width": 1})
            out["markers"].append({"pos": cone["nest"], "label": "nest", "color": ok, "shape": "circle"})
            out["cone"] = {
                "center": cone["nest"], "radius": CONE_RANGE_M * math.sin(CONE_HALF),
                "inside": cone["inside"],
            }
        if cone is not None:
            state = "WELD GATE" if cone["weld"] else ("IN CONE" if cone["inside"] else "outside cone")
            out["lines"].append(
                f"{state}  range {cone['range']:.2f} m  lateral {cone['lateral']:.2f} m  "
                f"off {math.degrees(cone['off_axis']):.0f}deg  "
                f"yaw {math.degrees(cone['yaw']):+.0f}deg  tilt {math.degrees(cone['tilt']):.0f}deg"
            )
        return out

    def _control(self, tm, auv, boat):
        boat_truth = boat
        if boat is not None:
            boat = self._boat_estimate(tm, auv, boat)
        self._update(tm, auv, boat)
        self.yaw_hold = None
        pos = auv[0]
        if not TERMINAL_MPC and boat is not None and self._stage in ("latch", "flare"):
            if self._stage == "latch":
                yaw = self._capture_yaw
                if yaw is None:
                    yaw = self._centreline_yaw(pos, boat, CAPTURE_LOOKAHEAD_M, CAPTURE_HEADING_OFF)
                self.yaw_hold = self._slew_yaw(yaw, tm)
                # Climb per metre forward is tan(pitch) for a body-x thruster,
                # so a deep entry needs pitch, not time, to reach the sphere.
                _f, _l, u_n = nest_body_offset(pos, boat[0], boat[1])
                deep = max(0.0, min(1.0, (-u_n - CAPTURE_ALONG_M) / 0.35))
                self.pitch_limit = CAPTURE_PITCH_RAD + deep * CAPTURE_PITCH_BOOST_RAD
            else:
                self._nest_attitude(
                    tm, pos, boat, prefer_boat_yaw=True, cruise_pitch=DOCK_PITCH_RAD,
                )
                if self.yaw_hold is not None:
                    self.yaw_hold = self._slew_yaw(
                        self._centreline_yaw(pos, boat, FLARE_LOOKAHEAD_M, FLARE_HEADING_OFF), tm,
                    )
        elif self._stage == "dock" and boat is not None:
            # Plates parallel for weld / last-metre MPC.
            self.yaw_hold = rpy_from_q(boat[1])[2]
            self.pitch_limit = DOCK_PITCH_RAD
            if self._dive_needed(pos):
                self.pitch_limit = max(DOCK_PITCH_RAD, DIVE_PITCH_RAD * 0.6)
        elif self._stage == "latch" and boat is not None:
            lateral = float((self._cone or {}).get("lateral", 99.0))
            if lateral <= LATCH_CENTER_M:
                self.yaw_hold = self._slew_yaw(rpy_from_q(boat[1])[2], tm)
                self.pitch_limit = DOCK_PITCH_RAD
            elif self._des is not None:
                # A body-x-only hull cannot remove lateral error while its nose
                # is rigidly parallel to the boat. Point into the capture
                # correction, then align with the boat only after centering.
                dx = float(self._des[0]) - float(pos[0])
                dy = float(self._des[1]) - float(pos[1])
                if math.hypot(dx, dy) > 0.03:
                    self.yaw_hold = self._slew_yaw(math.atan2(dy, dx), tm)
                self.pitch_limit = max(DOCK_PITCH_RAD, math.radians(24.0))
        elif self._stage == "flare" and boat is not None:
            # Flare: boat yaw, tighten pitch toward level.
            self._nest_attitude(
                tm, pos, boat,
                prefer_boat_yaw=True,
                cruise_pitch=DOCK_PITCH_RAD,
            )
        elif self._stage in ("approach", "aft"):
            corr = self._corridor
            dist_tip = float(corr.get("dist_tip", corr.get("dist_aft", 99.0))) if corr else 99.0
            near = dist_tip <= max(1.2, 1.5 * AFT_ARRIVE_M)
            far = dist_tip >= AFT_FAR_M
            # Full pitch authority — nose-down under surge aims the up-cam.
            cruise = GLIDE_PITCH_RAD
            self._nest_attitude(
                tm, pos, boat, prefer_boat_yaw=near and not far, cruise_pitch=cruise,
            )
            # Swim to the tip nose-first. Free yaw here followed the small
            # horizontal part of the dive force and wagged the hull. Near the
            # tip, turn onto the boat heading the glide will hold.
            if self._stage == "approach" and self._transit_yaw is not None:
                target = self._transit_yaw
                if near and not far and self.yaw_hold is not None:
                    target = self.yaw_hold
                self.yaw_hold = self._slew_yaw(target, tm)
                if self._dive_needed(pos):
                    self.pitch_limit = max(float(self.pitch_limit or 0.0), DIVE_PITCH_RAD)
        elif self._stage in ("glide", "commit", "under", "relock"):
            # Boat yaw on the slope; allow nose-down for early up-cam FOV.
            self._nest_attitude(
                tm, pos, boat,
                prefer_boat_yaw=not self._dive_needed(pos),
                cruise_pitch=GLIDE_PITCH_RAD,
            )
            if (
                self._stage == "glide"
                and self.yaw_hold is not None
                and boat is not None
                and not TERMINAL_MPC
            ):
                # The single body-x thruster cannot side-slip, so steer onto
                # the centreline with a bounded offset from the boat heading.
                # The old pursuit of the setpoint circled it near the nest.
                self.yaw_hold = self._centreline_yaw(
                    pos, boat, GLIDE_LOOKAHEAD_M, GLIDE_HEADING_OFF,
                )
            elif self._stage == "glide" and self._des is not None and not self._dive_needed(pos):
                corr = self._corridor or {}
                half = max(0.05, float(corr.get("half_m", 0.5)))
                if float(corr.get("lat", 0.0)) > 0.65 * half:
                    dx = float(self._des[0]) - float(pos[0])
                    dy = float(self._des[1]) - float(pos[1])
                    if math.hypot(dx, dy) > 0.04:
                        self.yaw_hold = math.atan2(dy, dx)
            # Turn from the transit bearing onto the boat heading at a rate,
            # not in one tick. Under/relock keep the direct hold.
            if self._stage in ("glide", "commit") and self.yaw_hold is not None:
                self.yaw_hold = self._slew_yaw(self.yaw_hold, tm)
        elif self._des is not None and boat is not None:
            dist = math.hypot(self._des[0] - pos[0], self._des[1] - pos[1])
            prefer = station_heading(dist, rpy_from_q(boat[1])[2])
            cruise = STATION_PITCH_RAD if prefer is not None else GLIDE_PITCH_RAD
            self._nest_attitude(
                tm, pos, boat,
                prefer_boat_yaw=prefer is not None and self._depth_ok(pos),
                cruise_pitch=cruise,
            )
        if self.yaw_hold is None or self._stage not in ("approach", "glide", "commit"):
            # Slewed stages start from the heading last held, or the hull's.
            self._yaw_out = (
                self.yaw_hold if self.yaw_hold is not None else rpy_from_q(auv[1])[2]
            )
            self._yaw_out_tm = float(tm)
        return UdroneHelixController._control(self, tm, auv, boat_truth)

    def _slew_yaw(self, target, tm):
        dt = 0.05 if self._yaw_out_tm is None else float(tm) - float(self._yaw_out_tm)
        self._yaw_out_tm = float(tm)
        self._yaw_out = slew_angle(self._yaw_out, target, dt, rate=0.45)
        return self._yaw_out

    def _helix(self, tm: float):
        if self._des is None or self.anchor is None:
            z = self._prev[1][2] if self._prev else -1.0
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(self._des[0] - x0, self._des[1] - y0, yaw0)
        c, s = math.cos(yaw0), math.sin(yaw0)
        vx, vy, vz = self._vel
        path_v = (c * vx + s * vy, -s * vx + c * vy, vz)
        if self._acc is not None:
            ax, ay, az = self._acc
            path_a = (c * ax + s * ay, -s * ax + c * ay, az)
            return (fwd, left, self._des[2]), path_v, path_a
        return (fwd, left, self._des[2]), path_v

    def _finished(self, tm: float) -> bool:
        if self._landed or self.failed:
            return True
        if tm >= self._fail_budget_s:
            if self._hits < FOUND_HITS:
                self._fail(f"did not see tag {self.tag_id} under otter_k")
            else:
                self._fail("saw the deck tag but did not latch")
            return True
        return False


def main():
    rclpy.init()
    node = UdroneRendezvous()
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_rendezvous_ctrl_log.txt")
        why = node.failed or "no samples"
        if node._landed and not node.failed:
            ok = True
            why = "latched under otter_k"
        elif not node.failed:
            why = "did not land under otter_k"
        print(f"RENDEZVOUS_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
