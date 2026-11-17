#!/usr/bin/env python3
"""Search for the Kiwanis hula hoop and swim through the opening.

Search covers a dock-centred grid until the front camera locks the orange
ring. Then the hull flies a rendezvous-style corridor: approach station on
the centreline → tapering glide path to the gate → hoop-axis CEM tube
through the opening (``hoop_mpc``; ``UDRONE_HOOP_MPC=0`` falls back to a
polyline). If through fails (off the tube / miss), the corridor is
reacquired at the approach station rather than restarting the whole search.
HUD draws the glide-path funnel on ``/xudrone/hud/image``. Coral ``hoop``.
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
from sensor_msgs.msg import CameraInfo, Image

import hoop_mpc
from hoop_geom import (
    HOOP_AXIS,
    HOOP_CENTER,
    HOOP_FLOOR,
    DEPTH_HI,
    DEPTH_LO,
    INNER_PASS_M,
    RING_OUTER_M,
    SEARCH_HITS,
    SEARCH_Z,
    SIDEWAYS_M,
    WEDGE_NEAR_M,
    CoverageMap,
    debug_caption,
    fly_trajectory,
    hoop_corridor,
    hoop_glide_path,
    horizontal_axis,
    in_opening,
    level_feature,
    look_leg,
    look_step,
    orange_ring,
    pass_depth,
    ring_center,
    sideways_baseline,
)
from patterns import path_length_3d, polyline_pos_vel, world_offset_to_path
from servo_geom import relock_leg
from udrone_sinusoidal_controller import (
    UdroneHelixController,
    body_axes,
    rpy_from_q,
    spin_until_done,
)

SPEED = 0.32          # trial 6 of the hoop search rollout
LEAD_M = 1.62
EXIT_M = 2.0
SEARCH_BUDGET_S = 200.0
THROUGH_BUDGET_S = 80.0
SEARCH_RETRY_S = 40.0   # extra search after a rare through abort near the budget
COMMIT_RANGE = (1.15, 8.0)  # allow commit once close (was 2.4 — rejected last-metre locks)
CENTER_PX = 0.35            # levelled lateral error before the sweep is planned
CENTER_PY = 0.45            # levelled vertical error
COMMIT_ASPECT = 0.55        # minor/major; near enough to the ring axis to pass
# Well-centered radial approach may never accumulate SIDEWAYS_M — waive it.
COMMIT_CENTER_PX = 0.18
COMMIT_CENTER_PY = 0.22
COMMIT_CENTER_ASPECT = 0.60
LOST_S = 1.5
# Within this range of a known centre, never look-step toward orange (rim chase).
CENTERLINE_HOLD_M = 5.5
NOSE_DOWN_CAP = 0.85        # rad; past this, do not command a deeper setpoint
THROUGH_PITCH_RAD = math.radians(28.0)
APPROACH_PITCH_RAD = math.radians(35.0)
MAX_REACQUIRE = 3
OFF_TUBE_S = 2.0          # sustained off-tube before reacquire
OFF_TUBE_R = 0.55         # metres off axis (hoop frame) to count as failed through


def _cv2_ok() -> bool:
    try:
        import cv2  # noqa: F401
        import numpy as np
        np.zeros(1)
        return True
    except Exception:
        return False


class UdroneHoopController(UdroneHelixController):
    def __init__(self, **kwargs):
        kwargs.setdefault("duration_s", SEARCH_BUDGET_S + THROUGH_BUDGET_S + 30.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", SPEED)
        kwargs.setdefault("z0", -1.0)
        super().__init__(**kwargs)
        # The search dives about 4.5 m to the ring, so allow a steep nose.
        self.pitch_limit = math.radians(70.0)
        self.stage = "search"
        self.fx = 381.46
        self.fy = 381.46
        self.cx = 320.0
        self.cy = 240.0
        self._img = None
        self._sight_stamp = None
        self._sight_hit = None
        self._map = None
        self._goal_cell = None
        self._goal_xy = None
        self._search_wps = []
        self._search_end = None
        self._leg_t0 = None
        self._leg_time = 0.0
        self._leg_limit = 0.0
        self._map_print = 0.0
        self._ring_log_t = 0.0
        self._none_log_t = 0.0
        self._fixes = []
        self._fix_stamp = None
        self._fix_origin = None
        self._fix_samples = []
        self._slides = []
        self._slide_stamp = None
        self._planned_center = None
        self._plan_kind = None
        self._wps = []
        self._through_t0 = None
        self._center = None
        self._axis = (1.0, 0.0, 0.0)
        self._exit = None
        self._crossed = False
        self._real_crossed = False
        self._surrounded = False
        self._miss = float("inf")
        self._real_miss = float("inf")
        self._servo = None
        self._approach = None
        self._approach_wps = []
        self._approach_t0 = None
        self._approach_stand = None
        self._track_log_t = 0.0
        self._last_hit_t = -LOST_S
        self._through_seen = None
        self._relock = False
        self._tracking = False
        self._deep_t = None
        self._dumped = False
        self._last_center = None
        self._sweep_world = []
        self._hud_hit = None
        self._hoop_mpc = hoop_mpc.enabled()
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._des = None
        self._vel = (0.0, 0.0, 0.0)
        self._mpc_log_t = 0.0
        self._mpc_tm = None
        self._through_blind_logged = False
        self._search_extra = 0.0
        self._corridor = None
        self._reacquire_n = 0
        self._off_tube_t0 = None
        self.hud_preview = False
        self._use_cv2 = _cv2_ok()
        info_topic = os.environ.get("UDRONE_HOOP_INFO_TOPIC", "/xudrone/camera_front/camera_info")
        image_topic = os.environ.get("UDRONE_HOOP_IMAGE_TOPIC", "/xudrone/camera_front/image")
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"hoop: opencv ring outer={RING_OUTER_M:g} m "
            f"cv2={'yes' if self._use_cv2 else 'MISSING'} "
            f"mpc={'on' if self._hoop_mpc else 'off'} corridor=on"
        )

    def _on_info(self, m: CameraInfo):
        k = list(m.k)
        if len(k) >= 6 and k[0] > 1.0 and k[4] > 1.0:
            self.fx, self.fy = float(k[0]), float(k[4])
            self.cx, self.cy = float(k[2]), float(k[5])

    def _on_image(self, m: Image):
        self._img = (m.encoding or "", int(m.width), int(m.height), bytes(m.data), time.time())

    def _bgr(self):
        if self._img is None or not self._use_cv2:
            return None
        enc, w, h, data, stamp = self._img
        if time.time() - stamp > 0.7 or w < 2 or h < 2:
            return None
        import cv2
        import numpy as np
        arr = np.frombuffer(data, dtype=np.uint8)
        key = (enc or "").lower()
        frame = None
        if key in ("rgb8", "") and arr.size >= w * h * 3:
            rgb = arr[: w * h * 3].reshape(h, w, 3)
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif key == "bgr8" and arr.size >= w * h * 3:
            frame = arr[: w * h * 3].reshape(h, w, 3).copy()
        elif key == "rgba8" and arr.size >= w * h * 4:
            rgba = arr[: w * h * 4].reshape(h, w, 4)
            frame = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        elif key == "bgra8" and arr.size >= w * h * 4:
            bgra = arr[: w * h * 4].reshape(h, w, 4)
            frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return frame

    def _events(self, tm: float):
        if not self._use_cv2:
            self._fail("hoop rejected: opencv is not importable")
            return
        if self.stage == "search" and self.latch.state == "attached":
            self._fail("hoop rejected: xudrone is latched; detach first")

    def _sight(self, tm, pos, q):
        """Latest orange-ring centre, or None. One fit per camera frame."""
        bgr = self._bgr()
        if bgr is None:
            return None
        stamp = self._img[4]
        if stamp == self._sight_stamp:
            return self._sight_hit
        self._sight_stamp = stamp
        hit = orange_ring(bgr)
        if hit is None:
            self._sight_hit = None
            if time.time() >= self._none_log_t:
                self._none_log_t = time.time() + 4.0
                import numpy as np
                pix = bgr.astype(np.int16)
                r, g, b = pix[:, :, 2], pix[:, :, 1], pix[:, :, 0]
                orange = int(((r > 150) & (r > g + 45) & (r > b + 100) & (g > b + 25)).sum())
                print(f"HOOP_RING none orange={orange}", flush=True)
            return None
        fwd, left, up = body_axes(q)
        try:
            center, z_opt = ring_center(
                hit["u"], hit["v"], hit["major"],
                self.fx, self.fy, self.cx, self.cy,
                pos, fwd, left, up,
            )
        except ValueError:
            self._sight_hit = None
            return None
        if not all(math.isfinite(v) for v in center) or not math.isfinite(z_opt):
            self._sight_hit = None
            return None
        _roll, pitch, _yaw = rpy_from_q(q)
        x_n = (hit["u"] - self.cx) / max(self.fx, 1.0)
        y_n = (hit["v"] - self.cy) / max(self.fy, 1.0)
        x_l, y_l = level_feature(x_n, y_n, pitch)
        hit["center"] = center
        hit["z_opt"] = z_opt
        hit["fwd"] = fwd
        hit["left"] = left
        hit["up"] = up
        hit["x_n"] = x_n
        hit["y_n"] = y_n
        hit["x_l"] = x_l
        hit["y_l"] = y_l
        self._sight_hit = hit
        if not self._dumped:
            self._dumped = True
            try:
                import cv2
                cv2.imwrite("/tmp/hoop_ring.jpg", bgr)
            except Exception:
                pass
        return hit

    def _commit(self, tm, pos, hit):
        if time.time() >= self._ring_log_t:
            self._ring_log_t = time.time() + 1.0
            c = hit["center"]
            print(
                f"HOOP_RING u={hit['u']:.0f} v={hit['v']:.0f} "
                f"major={hit['major']:.0f} aspect={hit['aspect']:.2f} "
                f"level=({hit['x_l']:+.2f},{hit['y_l']:+.2f}) "
                f"range={hit['z_opt']:.2f} closed={int(bool(hit['closed']))} "
                f"center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})",
                flush=True,
            )
        if not hit["closed"]:
            return
        if abs(hit["x_l"]) > CENTER_PX or abs(hit["y_l"]) > CENTER_PY:
            return
        if hit["aspect"] < COMMIT_ASPECT:
            return
        if not (COMMIT_RANGE[0] <= hit["z_opt"] <= COMMIT_RANGE[1]):
            return
        center = hit["center"]
        if center[2] > -0.35 or center[2] < HOOP_FLOOR:
            return
        stamp = self._img[4] if self._img is not None else None
        if stamp == self._fix_stamp:
            return
        self._fix_stamp = stamp
        self._fix_samples.append(center)
        self._fix_samples = self._fix_samples[-5:]
        if len(self._fix_samples) < 3:
            return
        well_centered = (
            abs(hit["x_l"]) <= COMMIT_CENTER_PX
            and abs(hit["y_l"]) <= COMMIT_CENTER_PY
            and hit["aspect"] >= COMMIT_CENTER_ASPECT
        )
        if not well_centered and not sideways_baseline(self._slides, SIDEWAYS_M):
            return
        pts = self._fix_samples
        med = tuple(sorted(p[i] for p in pts)[len(pts) // 2] for i in range(3))
        if max(math.dist(p, med) for p in pts[-3:]) > 0.40:
            return
        if self.anchor is None:
            return
        heading = horizontal_axis(hit["fwd"])
        _points, axis = fly_trajectory(pos, heading, med, LEAD_M, EXIT_M, through=True)
        self._fix_samples.clear()
        self._fix_origin = None
        self._relock = False
        self._servo = None
        self._enter_corridor(tm, pos, med, axis, why=f"centered={int(well_centered)}")

    def _centerline_gate(self, pos, center, axis_xy=None):
        """Standoff on the opening axis — never an orange-rim aim point."""
        if axis_xy is None:
            de = float(center[0]) - float(pos[0])
            dn = float(center[1]) - float(pos[1])
            dist = math.hypot(de, dn)
            if dist < 0.3:
                ax, ay = 1.0, 0.0
            else:
                ax, ay = de / dist, dn / dist
        else:
            ax, ay = float(axis_xy[0]), float(axis_xy[1])
            n = math.hypot(ax, ay) or 1.0
            ax, ay = ax / n, ay / n
            de = float(center[0]) - float(pos[0])
            dn = float(center[1]) - float(pos[1])
            dist = math.hypot(de, dn)
        back = min(LEAD_M, max(0.45, dist - 0.55))
        z = pass_depth(center[2])
        return (float(center[0]) - ax * back, float(center[1]) - ay * back, z), (ax, ay, 0.0)

    def _aim(self, pos, hit):
        """World point the force loop should chase.

        A closed ring uses the estimated centre and stops LEAD_M short of it
        on the opening axis (not the orange tube). An arc only gives a bearing
        when no centre fix exists yet; once a centre is known nearby, hold that
        centerline gate instead of stepping toward orange pixels (rim chase).
        """
        known = self._planned_center or self._last_center
        if known is not None:
            dist = math.hypot(known[0] - pos[0], known[1] - pos[1])
            if dist <= CENTERLINE_HOLD_M:
                gate, _ax = self._centerline_gate(pos, known, self._axis)
                return gate
        fwd = hit["fwd"]
        ax = horizontal_axis(fwd)
        left_h = (-ax[1], ax[0])
        if hit["closed"] and hit["z_opt"] <= 12.0 and "center" in hit:
            c = hit["center"]
            gate, _ax = self._centerline_gate(pos, c, ax)
            # Small image trim only while still far — never when the ring fills
            # the frame (clipped orange is the rim, not the opening).
            if hit["z_opt"] > 3.2:
                lat = max(-0.30, min(0.30, float(hit["x_l"]) * min(hit["z_opt"], 5.0) * 0.25))
                return (gate[0] - lat * left_h[0], gate[1] - lat * left_h[1], gate[2])
            return gate
        # Far arc with no centre yet: short bearing step (search only).
        step = 1.5
        z = pos[2]
        return (
            pos[0] + step * ax[0] - hit["x_n"] * step * left_h[0],
            pos[1] + step * ax[1] - hit["x_n"] * step * left_h[1],
            z,
        )

    def _plan_once(self, tm, pos, hit, closed_line):
        """Plan one lead-in and keep tracking it.

        A closed ring joins the centerline and stops at the rollout. Near a
        known centre, arcs do **not** look-step toward orange (that chases the
        rim); they hold the centerline gate. Far arcs without a centre still
        take a short bearing step. Plans replace only after three seconds and
        only when the aim has moved by more than a metre.
        """
        if self.anchor is None:
            return
        known = None
        if "center" in hit and HOOP_FLOOR <= hit["center"][2] <= -0.35:
            known = hit["center"]
        elif self._planned_center is not None:
            known = self._planned_center
        elif self._last_center is not None:
            known = self._last_center
        if known is None:
            return
        dist = math.hypot(known[0] - pos[0], known[1] - pos[1])
        hold_line = dist <= CENTERLINE_HOLD_M and (
            self._planned_center is not None or self._last_center is not None
        )
        kind = "lead"
        if closed_line and HOOP_FLOOR <= known[2] <= -0.35:
            moved = (
                self._planned_center is None
                or math.dist(self._planned_center, known) > 1.2
            )
            aim = known
        elif hold_line:
            # Centerline hold — do not chase a clipped orange arc (the rim).
            kind = "gate"
            gate, axis = self._centerline_gate(pos, known, self._axis)
            points = [
                (float(pos[0]), float(pos[1]), float(pos[2])),
                gate,
            ]
            aim = gate
            moved = (
                self._planned_center is None
                or math.dist(self._planned_center, known) > 1.0
            )
        else:
            kind = "look"
            points, axis = look_step(pos, known)
            aim = points[-1]
            moved = (
                self._planned_center is None
                or math.dist(self._planned_center, known) > 1.5
            )
        age = 0.0 if self._approach_t0 is None else tm - self._approach_t0
        if self._plan_kind in ("lead", "gate") and kind == "look" and self._approach_wps:
            self._tracking = True
            return
        if self._approach_wps and self._plan_kind == kind and (age < 3.0 or not moved):
            self._tracking = True
            return
        if kind == "lead":
            heading = horizontal_axis(hit["fwd"]) if hit.get("fwd") else (1.0, 0.0, 0.0)
            points, axis = fly_trajectory(pos, heading, known, LEAD_M, EXIT_M, through=False)
        self._tracking = True
        self._plan_kind = kind
        self._planned_center = known
        self._approach_stand = aim
        self._approach_t0 = tm
        self._approach_wps = [self._path_of(p) for p in points]
        self._sweep_world = list(points)
        self._axis = axis
        end = points[-1]
        print(
            f"HOOP_PLAN {kind} points={len(points)} "
            f"end=({end[0]:.2f},{end[1]:.2f},{end[2]:.2f}) "
            f"range={hit.get('z_opt', float('nan')):.1f}",
            flush=True,
        )

    def _track(self, tm, pos, hit):
        """Track the planned lead-in. The path is not rebuilt from this frame."""
        self._plan_once(tm, pos, hit, True)
        if self._sweep_world and time.time() >= self._track_log_t:
            self._track_log_t = time.time() + 2.0
            ap = self._sweep_world[-1]
            print(
                f"HOOP_TRACK to=({ap[0]:.1f},{ap[1]:.1f},{ap[2]:.2f}) "
                f"range={float(hit.get('z_opt', float('nan'))):.1f} "
                f"aspect={float(hit.get('aspect', float('nan'))):.2f} "
                f"level=({float(hit.get('x_l', 0.0)):+.2f},"
                f"{float(hit.get('y_l', 0.0)):+.2f})",
                flush=True,
            )

    def _drop_track(self):
        self._approach = None
        self._approach_wps = []
        self._approach_t0 = None
        self._approach_stand = None
        self._tracking = False
        self._fix_samples.clear()
        self._fix_origin = None
        self._fix_stamp = None
        self._slides = []
        self._slide_stamp = None
        self._planned_center = None
        self._plan_kind = None
        self._search_wps = []
        self._leg_t0 = None
        self._goal_cell = None
        self._prev = None
        self.vel = (0.0, 0.0, 0.0)

    def _align(self, tm, pos, hit):
        """Track one short step toward an orange arc. The step is not chased."""
        self._plan_once(tm, pos, hit, False)
        if self._sweep_world and time.time() >= self._track_log_t:
            self._track_log_t = time.time() + 2.0
            ap = self._sweep_world[-1]
            print(
                f"HOOP_ALIGN to=({ap[0]:.1f},{ap[1]:.1f},{ap[2]:.2f}) "
                f"aspect={hit['aspect']:.2f} level=({hit['x_l']:+.2f},{hit['y_l']:+.2f})",
                flush=True,
            )

    def _note_real(self, pos):
        miss = math.dist(pos, HOOP_CENTER)
        if miss < self._real_miss:
            self._real_miss = miss
        if self._real_crossed or not in_opening(pos, HOOP_CENTER, HOOP_AXIS, INNER_PASS_M):
            return
        self._real_crossed = True
        print(
            f"HOOP_REAL xyz=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) real_miss={miss:.2f}",
            flush=True,
        )

    def _note_surround(self, hit):
        if self._surrounded or hit is None or not hit.get("closed"):
            return
        if abs(hit["x_l"]) > 0.22 or abs(hit["y_l"]) > 0.28 or hit["z_opt"] > 1.6:
            return
        self._surrounded = True
        print(
            f"HOOP_SURROUND range={hit['z_opt']:.2f} "
            f"level=({hit['x_l']:+.2f},{hit['y_l']:+.2f})",
            flush=True,
        )

    def _note_slide(self, pos, hit):
        """Cross-track since the intercept started. The turn counts as the baseline."""
        fwd = hit.get("fwd")
        if not fwd:
            return
        stamp = self._img[4] if self._img is not None else None
        if stamp is not None and stamp == self._slide_stamp:
            return
        self._slide_stamp = stamp
        left = (-float(fwd[1]), float(fwd[0]))
        ln = math.hypot(left[0], left[1]) or 1.0
        left = (left[0] / ln, left[1] / ln)
        if self._fix_origin is None:
            self._fix_origin = (float(pos[0]), float(pos[1]), left[0], left[1])
            side = 0.0
        else:
            ox, oy, lx, ly = self._fix_origin
            side = (float(pos[0]) - ox) * lx + (float(pos[1]) - oy) * ly
        self._slides.append(side)
        self._slides = self._slides[-12:]

    def _path_of(self, enu):
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(enu[0] - x0, enu[1] - y0, yaw0)
        return (fwd, left, enu[2])

    def _corridor_prev(self):
        s = (self.stage or "").lower()
        if s in ("through",):
            return "through"
        if s in ("commit",):
            return "commit"
        if s in ("approach", "reacquire"):
            return "approach"
        return None

    def _enter_corridor(self, tm, pos, center, axis, why: str = ""):
        """Lock the fix and start the approach-station → gate glide path."""
        self._center = tuple(float(c) for c in center)
        self._axis = (
            float(axis[0]), float(axis[1]),
            float(axis[2]) if len(axis) > 2 else 0.0,
        )
        corr = hoop_corridor(pos, self._center, self._axis, prev_phase=None)
        self._corridor = corr
        self._axis = corr["through"]
        self._exit = corr["exit"]
        self._through_t0 = None
        self._through_seen = None
        self._through_blind_logged = False
        self._crossed = False
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._vel = (0.0, 0.0, 0.0)
        self._off_tube_t0 = None
        self._wps = []
        self._tracking = False
        self._approach_wps = []
        self.stage = "approach"
        self._des = corr["approach"]
        self._sweep_world = [pos, corr["approach"], corr["gate"], corr["exit"]]
        print(
            f"HOOP_CORRIDOR enter approach "
            f"center=({self._center[0]:.2f},{self._center[1]:.2f},{self._center[2]:.2f}) "
            f"through=({corr['through'][0]:+.2f},{corr['through'][1]:+.2f}) "
            f"station=({corr['approach'][0]:.2f},{corr['approach'][1]:.2f}) "
            f"gate=({corr['gate'][0]:.2f},{corr['gate'][1]:.2f}) {why}",
            flush=True,
        )

    def _reacquire_corridor(self, tm, pos, reason: str):
        """Through failed — return to the approach station and try again."""
        self._reacquire_n += 1
        self._search_extra = max(self._search_extra, SEARCH_RETRY_S)
        if self._center is None:
            self.stage = "search"
            self._des = None
            print(f"HOOP_REACQUIRE search (no centre) n={self._reacquire_n} {reason}", flush=True)
            return
        if self._reacquire_n > MAX_REACQUIRE:
            self._fail(f"through failed after {MAX_REACQUIRE} corridor reacquires ({reason})")
            return
        corr = hoop_corridor(pos, self._center, self._axis, prev_phase=None)
        self._corridor = corr
        self._axis = corr["through"]
        self._exit = corr["exit"]
        self.stage = "approach"
        self._des = corr["approach"]
        self._vel = (0.0, 0.0, 0.0)
        self._through_t0 = None
        self._through_seen = None
        self._through_blind_logged = False
        self._crossed = False
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._off_tube_t0 = None
        self._wps = []
        self.pitch_limit = math.radians(70.0)
        self._sweep_world = [pos, corr["approach"], corr["gate"], corr["exit"]]
        print(
            f"HOOP_REACQUIRE approach n={self._reacquire_n}/{MAX_REACQUIRE} "
            f"station=({corr['approach'][0]:.2f},{corr['approach'][1]:.2f}) "
            f"why={reason}",
            flush=True,
        )

    def _enter_through(self, tm, pos, corr):
        """Gate reached on-axis — punch the CEM tube (or Hermite fallback)."""
        self.stage = "through"
        self._through_t0 = tm
        self._through_seen = tm
        self._through_blind_logged = False
        self._axis = corr["through"]
        self._exit = corr["exit"]
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._off_tube_t0 = None
        # Hermite fallback path if MPC is off.
        heading = (corr["through"][0], corr["through"][1], 0.0)
        points, axis = fly_trajectory(
            pos, heading, self._center, LEAD_M, EXIT_M, through=True,
        )
        self._axis = axis
        self._wps = [self._path_of(p) for p in points]
        self._sweep_world = points
        if self._hoop_mpc:
            self._step_mpc(pos, tm)
        print(
            f"HOOP_SWEEP corridor→through "
            f"gate=({corr['gate'][0]:.2f},{corr['gate'][1]:.2f}) "
            f"exit=({self._exit[0]:.2f},{self._exit[1]:.2f}) "
            f"mpc={'on' if self._hoop_mpc else 'off'} "
            f"lat={corr['lat']:.2f}",
            flush=True,
        )

    def _update_corridor(self, tm, pos):
        """Advance approach → commit → through along the locked glide path."""
        if self._center is None:
            return
        corr = hoop_corridor(
            pos, self._center, self._axis, prev_phase=self._corridor_prev(),
        )
        self._corridor = corr
        self._axis = corr["through"]
        self._exit = corr["exit"]
        phase = corr["phase"]
        if phase == "through":
            self._enter_through(tm, pos, corr)
            return
        if phase == "commit":
            if self.stage != "commit":
                print(
                    f"HOOP_CORRIDOR commit "
                    f"dist_gate={corr['dist_gate']:.2f} lat={corr['lat']:.2f} "
                    f"half={corr['half_m']:.2f}",
                    flush=True,
                )
            self.stage = "commit"
            # Lead along the centreline toward the gate (not the rim).
            self._des = corr["gate"]
            self._vel = tuple(
                0.32 * float(corr["through"][i]) for i in range(3)
            )
        else:
            self.stage = "approach"
            self._des = corr["approach"]
            self._vel = (0.0, 0.0, 0.0)
        self._sweep_world = [pos, corr["approach"], corr["gate"], self._exit]
        # Yaw-hold the through axis once on the corridor.
        self.yaw_hold = math.atan2(float(corr["through"][1]), float(corr["through"][0]))
        self.pitch_limit = APPROACH_PITCH_RAD

    def _des_helix(self):
        if self._des is None or self.anchor is None:
            z = self._prev[1][2] if self._prev else SEARCH_Z
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(self._des[0] - x0, self._des[1] - y0, yaw0)
        c, s = math.cos(yaw0), math.sin(yaw0)
        vx, vy, vz = self._vel
        path_v = (c * vx + s * vy, -s * vx + c * vy, vz)
        return (fwd, left, self._des[2]), path_v

    def _step_mpc(self, pos, tm=None):
        """Receding hoop-axis tube setpoint for the force loop."""
        if self._center is None:
            return
        step = hoop_mpc.plan(
            pos, self._center, self._axis,
            speed_mps=SPEED, exit_m=EXIT_M,
            warm=self._mpc_warm, v_state=self._mpc_v_state,
        )
        self._des = step["point"]
        self._vel = step["vel"]
        self._mpc_warm = step["warm"]
        self._mpc_v_state = step.get("v_lag", self._mpc_v_state)
        if tm is not None:
            self._mpc_tm = tm
        # HUD: short forward look along the commanded tube.
        tip = tuple(
            float(self._des[i]) + float(self._vel[i]) * 1.2 for i in range(3)
        )
        self._sweep_world = [pos, self._des, tip, self._exit] if self._exit else [pos, self._des, tip]
        if time.time() >= self._mpc_log_t:
            self._mpc_log_t = time.time() + 1.0
            vs, vy, vz = step["u0"]
            s0, y0, z0 = step["state0"]
            print(
                f"HOOP_MPC s={s0:+.2f} y={y0:+.2f} z={z0:+.2f} "
                f"u=({vs:+.2f},{vy:+.2f},{vz:+.2f}) "
                f"tube={int(step['in_tube'])} cost={step['cost']:.1f}",
                flush=True,
            )

    def _helix(self, tm: float):
        if self.stage in ("approach", "commit", "reacquire"):
            return self._des_helix()
        if self.stage == "through":
            if self._hoop_mpc and self._des is not None and self.anchor is not None:
                return self._des_helix()
            if not self._wps or self._through_t0 is None:
                z = self._prev[1][2] if self._prev else SEARCH_Z
                return (0.0, 0.0, z), (0.0, 0.0, 0.0)
            return polyline_pos_vel(self._wps, SPEED, max(0.0, tm - self._through_t0))
        if self._tracking and self._approach_wps and self._approach_t0 is not None:
            return polyline_pos_vel(
                self._approach_wps, SPEED, max(0.0, tm - self._approach_t0),
            )
        if self._tracking and self._approach is not None and self.anchor is not None:
            return self._path_of(self._approach), (0.0, 0.0, 0.0)
        if self._search_wps and self._leg_t0 is not None:
            return polyline_pos_vel(self._search_wps, SPEED, max(0.0, tm - self._leg_t0))
        return (0.0, 0.0, SEARCH_Z), (0.0, 0.0, 0.0)

    def _leg_done(self, tm, pos) -> bool:
        if self._leg_t0 is None or not self._search_wps or self._search_end is None:
            return True
        elapsed = tm - self._leg_t0
        if elapsed > self._leg_limit:
            return True
        if elapsed < max(2.5, self._leg_time * 0.8):
            return False
        horiz = math.hypot(self._search_end[0] - pos[0], self._search_end[1] - pos[1])
        deep = abs(pos[2] - SEARCH_Z) <= 0.45
        return horiz <= 0.7 and deep

    def _set_leg(self, tm, pos, ce, cn, cell):
        pts = look_leg(pos[0], pos[1], ce, cn)
        world = [(pos[0], pos[1], SEARCH_Z)]
        for east, north in pts:
            world.append((east, north, SEARCH_Z))
        self._search_wps = [self._path_of(p) for p in world]
        self._search_end = world[-1]
        self._goal_cell = cell
        self._goal_xy = (ce, cn)
        self._leg_t0 = tm
        length = path_length_3d(self._search_wps)
        self._leg_time = length / SPEED
        self._leg_limit = max(8.0, self._leg_time + 18.0)
        print(
            f"HOOP_LEG cell=({ce:.1f},{cn:.1f}) "
            f"to=({world[-1][0]:.1f},{world[-1][1]:.1f}) "
            f"len={length:.1f} open={self._map.open_count()}",
            flush=True,
        )

    def _abort_through(self, tm, pos):
        """Drop the blind line and point the front camera at the last ring."""
        center = self._last_center or self._center
        self.stage = "search"
        self._wps = []
        self._through_t0 = None
        self._through_seen = None
        self._sweep_world = []
        self._tracking = False
        self._approach_wps = []
        self._approach = None
        self._approach_t0 = None
        self._crossed = False
        self._fix_samples.clear()
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._des = None
        self._vel = (0.0, 0.0, 0.0)
        self._through_blind_logged = False
        self._search_extra = max(self._search_extra, SEARCH_RETRY_S)
        self.pitch_limit = math.radians(70.0)
        where = ""
        if center is not None:
            where = f" center=({center[0]:.2f},{center[1]:.2f},{center[2]:.2f})"
        print(f"HOOP_RELOCK ring left the front camera{where}", flush=True)
        if center is None or self.anchor is None:
            self._relock = False
            return
        self._last_center = center
        leg = relock_leg(pos, center, "front")
        z = min(DEPTH_HI, max(DEPTH_LO, leg[2]))
        target = (leg[0], leg[1], z)
        self._search_wps = [
            self._path_of((float(pos[0]), float(pos[1]), float(pos[2]))),
            self._path_of(target),
        ]
        self._search_end = target
        self._leg_t0 = tm
        length = path_length_3d(self._search_wps)
        self._leg_time = length / max(SPEED, 0.05)
        self._leg_limit = max(6.0, self._leg_time + 8.0)
        self._goal_cell = ("relock", round(center[0], 1), round(center[1], 1))
        self._goal_xy = (float(center[0]), float(center[1]))
        self._relock = True

    def _steer(self, tm, pos, yaw):
        if self.anchor is None:
            return
        if self._relock:
            end = self._search_end
            held = self._leg_t0 is not None and tm - self._leg_t0 < self._leg_limit
            if end is not None and math.dist(pos, end) > 0.6 and held:
                return
            self._relock = False
            self._search_wps = []
            self._leg_t0 = None
            self._goal_cell = None
        if self._map is None:
            x0, y0, _yaw0 = self.anchor
            self._map = CoverageMap(x0, y0)
            print(
                f"HOOP_MAP cells={len(self._map.cells)} res={self._map.res:g} "
                f"arm=({x0:.1f},{y0:.1f})",
                flush=True,
            )
        self._map.mark_wedge(pos[0], pos[1], yaw, pos[2], tm)
        if self._last_center is not None:
            ce, cn = self._last_center[0], self._last_center[1]
            dist = math.hypot(ce - pos[0], cn - pos[1])
            if dist > 2.8:
                same = (
                    self._goal_xy is not None
                    and math.hypot(self._goal_xy[0] - ce, self._goal_xy[1] - cn) < 0.8
                )
                if not same or self._leg_done(tm, pos):
                    self._set_leg(tm, pos, ce, cn, ("sight", round(ce, 1), round(cn, 1)))
                return
            if tm - self._last_hit_t > 8.0:
                self._last_center = None
        goal_hits = 0 if self._goal_cell is None else self._map.hits.get(self._goal_cell, 0)
        goal_done = self._goal_cell is not None and goal_hits >= SEARCH_HITS
        leg_done = self._leg_done(tm, pos)
        if self._goal_cell is None or goal_done or leg_done:
            skip = set()
            if self._goal_cell is not None and leg_done and not goal_done:
                skip.add(self._goal_cell)
            nxt = self._map.nearest_open(
                pos[0], pos[1], yaw, skip=skip, min_dist=WEDGE_NEAR_M,
            )
            if nxt is None:
                nxt = self._map.nearest_open(pos[0], pos[1], yaw, skip=skip)
            if nxt is None:
                nxt = self._map.nearest_open(pos[0], pos[1], yaw)
            if nxt is None:
                self._fail("searched the box without seeing the hoop")
                return
            ce, cn, cell, _dist = nxt
            if cell != self._goal_cell or not self._search_wps or goal_done:
                self._set_leg(tm, pos, ce, cn, cell)
        if tm >= self._map_print:
            self._map_print = tm + 4.0
            ge, gn = self._goal_xy if self._goal_xy else (float("nan"), float("nan"))
            print(
                f"t={tm:5.1f} HOOP_MAP done={self._map.done_count()} "
                f"open={self._map.open_count()} goal=({ge:.1f},{gn:.1f}) "
                f"z={pos[2]:.2f}",
                flush=True,
            )

    def _hud_layers(self):
        """Ring box, planned path, end, fix, and the plan caption for the HUD."""
        out = UdroneHelixController._hud_layers(self)
        out["camera"] = "front"
        out["stage"] = self.stage
        pos = self._prev[1] if self._prev is not None else None
        hit = self._hud_hit
        world = list(self._sweep_world) if self._sweep_world else []
        if not world and self._search_end is not None:
            world = [self._search_end]
        center = self._center if self._center is not None else self._planned_center
        slides = self._slides
        slide_m = (max(slides) - min(slides)) if len(slides) >= 2 else 0.0
        remain = math.dist(pos, world[-1]) if (world and pos is not None) else 0.0
        out["lines"] += list(debug_caption(
            self.stage, self._plan_kind, len(world), remain, center, slide_m,
            self._real_miss, self._surrounded, self._real_crossed,
        ))
        if hit is not None:
            out["lines"].append(
                f"ring {hit['z_opt']:.1f}m asp {hit['aspect']:.2f} "
                f"lvl {hit['x_l']:+.2f},{hit['y_l']:+.2f}"
            )
        if center is not None:
            out["markers"].append({"pos": center, "label": "fix", "color": (255, 255, 255), "shape": "diamond"})
        # Glide-path funnel (rendezvous-style ILS rails).
        if self._center is not None:
            corr = self._corridor
            gp = hoop_glide_path(
                self._center, self._axis, vehicle=pos,
                in_funnel=bool(corr and corr.get("in_funnel")),
                through_ok=bool(self._crossed or self.stage == "through"),
            )
            out["paths"].extend(gp["paths"])
            out["markers"].extend(gp["markers"])
            if corr is not None:
                out["lines"].append(
                    f"corridor {corr['phase']}  s {corr['s']:+.2f}  "
                    f"lat {corr['lat']:.2f}/{corr['half_m']:.2f}  "
                    f"reacq {self._reacquire_n}"
                )
        if world:
            near_i = 0
            if pos is not None:
                near_i = min(range(len(world)), key=lambda i: math.dist(pos, world[i]))
            if near_i > 0:
                out["paths"].append({"pts": world[: near_i + 1], "color": (0, 90, 140), "width": 1})
            out["paths"].append({"pts": world[near_i:], "color": (0, 200, 255), "width": 2})
            out["markers"].append({"pos": world[-1], "label": "end", "color": (0, 200, 255), "shape": "circle"})
        if self._des is not None:
            out["markers"].append({"pos": self._des, "label": "goal", "color": (0, 200, 255)})

        def draw(img, _proj, clip):
            import cv2

            if hit is None:
                return
            bw = float(hit.get("w", hit["major"]))
            bh = float(hit.get("h", hit["minor"]))
            x0 = int(hit["u"] - bw / 2.0)
            y0 = int(hit["v"] - bh / 2.0)
            color = (0, 220, 0) if hit.get("closed") else (0, 180, 255)
            cv2.rectangle(img, (x0, y0), (x0 + int(bw), y0 + int(bh)), color, 2)
            cv2.drawMarker(img, clip((hit["u"], hit["v"])), color, cv2.MARKER_CROSS, 18, 2)

        out["draw"] = draw
        return out

    def _act(self, tm, auv, boat, hit, cmd=None):
        if cmd is None:
            cmd = UdroneHelixController._control(self, tm, auv, boat)
        self._hud_hit = hit
        return cmd

    def _control(self, tm, auv, boat):
        pos, q = auv
        self.yaw_hold = None
        if self._relock and self._search_end is not None:
            dx = self._search_end[0] - pos[0]
            dy = self._search_end[1] - pos[1]
            if math.hypot(dx, dy) > 0.60:
                self.yaw_hold = math.atan2(dy, dx)
        if pos[2] < HOOP_FLOOR:
            if self._deep_t is None:
                self._deep_t = tm
            elif tm - self._deep_t > 4.0:
                self._fail("depth ran away from the hoop")
                return self._act(tm, auv, boat, self._sight_hit, (0.0, 0.0, 0.0, 0.0))
        else:
            self._deep_t = None
        self._note_real(pos)
        if self.stage in ("approach", "commit", "reacquire"):
            hit = self._sight(tm, pos, q)
            if hit is not None and hit.get("closed") and "center" in hit:
                self._last_hit_t = tm
                self._last_center = hit["center"]
            self._update_corridor(tm, pos)
            if self.stage == "through":
                return self._act(tm, auv, boat, hit)
            return self._act(tm, auv, boat, hit)

        if self.stage == "through":
            # Hold yaw on the ring axis. Fly the tube blind after FOV loss;
            # off-tube for long enough reacquires the approach corridor.
            ax = self._axis
            self.yaw_hold = math.atan2(float(ax[1]), float(ax[0]))
            self.pitch_limit = THROUGH_PITCH_RAD
            hit = self._sight(tm, pos, q)
            self._note_surround(hit)
            inside = (
                self._center is not None
                and in_opening(pos, self._center, self._axis, INNER_PASS_M)
            )
            if hit is not None:
                self._through_seen = tm
            elif (
                not self._through_blind_logged
                and self._through_t0 is not None
                and tm - self._through_t0 > 0.4
            ):
                self._through_blind_logged = True
                print(
                    f"HOOP_COMMIT_HOLD blind through "
                    f"s_est={hoop_mpc.hoop_state(pos, self._center, self._axis)[0]:+.2f}m "
                    f"(no FOV abort)",
                    flush=True,
                )
            if self._hoop_mpc:
                self._step_mpc(pos, tm)
            # Off-tube watchdog → reacquire corridor (do not chase the rim).
            if self._center is not None and not inside and not self._crossed:
                _s, y, z, *_ = hoop_mpc.hoop_state(pos, self._center, self._axis)
                if math.hypot(y, z) > OFF_TUBE_R and _s < 0.4:
                    if self._off_tube_t0 is None:
                        self._off_tube_t0 = tm
                    elif tm - self._off_tube_t0 >= OFF_TUBE_S:
                        self._reacquire_corridor(
                            tm, pos, f"off-tube lat={math.hypot(y, z):.2f} s={_s:+.2f}",
                        )
                        return self._act(tm, auv, boat, hit)
                else:
                    self._off_tube_t0 = None
            if inside and not self._crossed:
                self._crossed = True
                print(
                    f"HOOP_THROUGH xyz=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                    f"real_miss={self._real_miss:.2f}",
                    flush=True,
                )
            return self._act(tm, auv, boat, hit)

        hit = self._sight(tm, pos, q)
        if hit is not None:
            self._note_slide(pos, hit)
        usable = (
            hit is not None
            and hit["closed"]
            and hit["aspect"] >= 0.50
            and 1.6 <= hit["z_opt"] <= 10.0
            and "center" in hit
            and HOOP_FLOOR <= hit["center"][2] <= -0.35
        )
        if usable:
            self._last_hit_t = tm
            self._last_center = hit["center"]
            self._relock = False
            self._commit(tm, pos, hit)
            if self.stage == "through":
                return self._act(tm, auv, boat, hit)
            self._track(tm, pos, hit)
            return self._act(tm, auv, boat, hit)
        # Near a known centre: hold the opening axis. Do not align toward
        # orange arcs — that steers the nose onto the rim in the last metres.
        known = self._planned_center or self._last_center
        near_known = (
            known is not None
            and math.hypot(known[0] - pos[0], known[1] - pos[1]) <= CENTERLINE_HOLD_M
        )
        if hit is not None and near_known:
            self._last_hit_t = tm
            self._track(tm, pos, hit)
            return self._act(tm, auv, boat, hit)
        if hit is not None and not near_known:
            self._last_hit_t = tm
            self._align(tm, pos, hit)
            return self._act(tm, auv, boat, hit)
        if near_known and self._tracking:
            # Blind hold on the last centerline gate (ring left the frame).
            # Keep the dict complete — _track logging used to KeyError on aspect
            # and abort the whole behaviour mid-approach.
            self._track(tm, pos, {
                "fwd": self._axis or (1.0, 0.0, 0.0),
                "z_opt": max(1.2, math.hypot(known[0] - pos[0], known[1] - pos[1])),
                "center": known,
                "aspect": 1.0,
                "closed": True,
                "x_l": 0.0,
                "y_l": 0.0,
                "x_n": 0.0,
                "y_n": 0.0,
            })
            return self._act(tm, auv, boat, hit)

        hold = 6.0 if self._approach_wps else LOST_S
        if self._tracking and tm - self._last_hit_t <= hold:
            return self._act(tm, auv, boat, hit)
        if self._tracking:
            self._drop_track()
        self._steer(tm, pos, rpy_from_q(q)[2])
        if self.failed or self.done:
            return self._act(tm, auv, boat, hit, (0.0, 0.0, 0.0, 0.0))
        return self._act(tm, auv, boat, hit)

    def _finished(self, tm: float) -> bool:
        if self.failed:
            return True
        if self.stage in ("search",):
            deadline = SEARCH_BUDGET_S + self._search_extra
            if tm >= deadline:
                self._fail(f"hoop not found in {deadline:.0f}s")
                return True
            return False
        if self.stage in ("approach", "commit", "reacquire"):
            deadline = SEARCH_BUDGET_S + THROUGH_BUDGET_S + self._search_extra
            if tm >= deadline:
                self._fail(f"corridor not completed in {deadline:.0f}s")
                return True
            return False
        pos = self._prev[1] if self._prev else None
        if pos is not None and self._exit is not None:
            self._miss = math.dist(pos, self._exit)
        real = math.dist(pos, HOOP_CENTER) if pos is not None else float("inf")
        elapsed = tm - (self._through_t0 or tm)
        vision_ok = self._crossed and self._miss <= 1.2 and elapsed > 2.0
        real_ok = self._real_crossed and self._surrounded and elapsed > 2.0
        if vision_ok or real_ok:
            print(
                f"HOOP_ARRIVE vision_miss={self._miss:.2f} m real_miss={real:.2f} m "
                f"vision_crossed={int(self._crossed)} real_crossed={int(self._real_crossed)} "
                f"surrounded={int(self._surrounded)}",
                flush=True,
            )
            return True
        if elapsed >= THROUGH_BUDGET_S:
            # Prefer corridor reacquire over a hard fail on the first miss.
            if self._reacquire_n < MAX_REACQUIRE and pos is not None:
                why = (
                    f"through budget latched miss={self._miss:.1f} "
                    f"real={real:.1f}"
                )
                self._reacquire_corridor(tm, pos, why)
                return False
            if not self._real_crossed:
                self._fail(
                    f"missed the opening (vision_miss {self._miss:.1f} m real_miss {real:.1f} m)"
                )
            elif not self._surrounded:
                self._fail(
                    f"crossed the ring without the opening around the camera "
                    f"(real_miss {real:.1f} m)"
                )
            elif self._miss > 1.2:
                self._fail(f"through the hoop but {self._miss:.1f} m short of the exit")
            return True
        return False


def main():
    rclpy.init()
    node = UdroneHoopController()
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_hoop_ctrl_log.txt")
        why = node.failed or "no samples"
        vision = bool(node._crossed or node._surrounded)
        # Align with _finished: vision through-to-exit or real cross + surround.
        vision_pass = bool(node._crossed and node._miss <= 1.2)
        real_pass = bool(node._real_crossed and node._surrounded)
        through = (
            node.armed_ok and node.log and not node.failed
            and (vision_pass or real_pass)
        )
        if through:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            else:
                ok = True
                why = (
                    f"ok through vision_miss={node._miss:.2f} m "
                    f"real_miss={node._real_miss:.2f} m "
                    f"vision={int(vision_pass)} real={int(real_pass)}"
                )
        elif node.armed_ok and node.log and not node.failed and vision and not node._real_crossed:
            why = (
                f"vision pass outside the ring vision_miss={node._miss:.2f} m "
                f"real_miss={node._real_miss:.2f} m"
            )
        elif not node.failed:
            why = "did not pass through the opening"
        print(f"HOOP_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
