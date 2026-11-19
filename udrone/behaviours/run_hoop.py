#!/usr/bin/env python3
"""Search for the Kiwanis hula hoop and swim through the opening.

Search covers a dock-centred grid. The grid finishes its dive before it
translates, so the nose does not pitch while sliding sideways. A closed ring
that matches the authored hoop starts one pursuit reference: dive in place,
slide onto the centreline, then cruise at 0.32 m/s and brake to a stop at
the exit. The nose stays free. The through-direction is frozen at entry.
Vision loss in the opening is expected. A pass is an authored-plane crossing
plus arrival at that exit, written once as HOOP_VERDICT.
If the open-loop punch misses the exit after the through budget, the
corridor can be reacquired at the approach station (capped). HUD draws the
glide-path funnel on ``/xudrone/hud/image``. Coral ``hoop``.
"""
from __future__ import annotations

import json
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
    WEDGE_NEAR_M,
    APPROACH_STANDOFF_M,
    CoverageMap,
    approach_point,
    authored_ring_lock,
    corridor_lead_point,
    debug_caption,
    fly_trajectory,
    hoop_corridor,
    hoop_glide_path,
    horizontal_axis,
    hoop_pass,
    in_opening,
    leg_clock,
    level_feature,
    look_leg,
    look_step,
    orange_ring,
    pass_depth,
    plane_sample,
    ready_to_punch,
    surge_speed,
    through_dir,
    ring_center,
)
from patterns import path_length_3d, polyline_pos_vel, slew_angle, world_offset_to_path
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
LOST_S = 1.5
# Within this range of a known centre, never look-step toward orange (rim chase).
CENTERLINE_HOLD_M = 5.5
NOSE_DOWN_CAP = 0.85        # rad; past this, do not command a deeper setpoint
THROUGH_PITCH_RAD = math.radians(40.0)  # enough nose-up to climb onto the opening
APPROACH_PITCH_RAD = math.radians(40.0)
MAX_REACQUIRE = 3
GOAROUND_MPS = 0.25         # loop beside the ring after a miss
GOAROUND_SIDE_M = 1.6       # clear of the ring (outer radius ~0.6 m) plus the hull
GOAROUND_BACK_M = 3.5       # rejoin the axis this far in front of the plane
# Last metre: hold the gate until centered, then surge open-loop to the exit.
# A combined lat gate let a deep, off-axis hull freeze the exit (false PASS).
OPEN_LOOP_M = 1.2           # start centering when this close to the gate
EXIT_ARRIVE_M = 0.60        # arrive at the virtual exit behind the ring
PUNCH_SPEED = 0.60          # committed surge once centered; approach stays SPEED
PUNCH_CLIMB_MPS = 0.22      # extra +Z while centering, or if still slightly deep
# Dock-glide capture. Lead is a fraction of a second along the centreline.
GLIDE_LEAD_S = 0.35
DIVE_DEPTH_ERR_M = 0.35     # shallower than the ring by this much → yaw stays free
DIVE_VZ_MPS = 0.24
STATION_LOCK_M = 1.2        # lock the through-axis only this close to the station
LOCK_FRAMES = 3             # unique camera frames, not repeated control ticks
RING_FILTER_ALPHA = 0.25
RING_FILTER_STEP_M = 0.40   # cap one-frame world-centre innovation
VISUAL_TRIM_ALPHA = 0.10
VISUAL_TRIM_MAX_M = 0.08    # guidance only; pass score stays on authored geometry


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
        self._sight_new = False
        self._ring_center_filt = None
        self._vision_trim = (0.0, 0.0)
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
        self._plane = None
        self._exit_logged = False
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
        self._acc = (0.0, 0.0, 0.0)
        self._slew_v = (0.0, 0.0, 0.0)
        self._slew_arm = None
        self._slew_tm = None
        self._slew_prev_tm = None
        self._yaw_cmd = None
        self._yaw_tm = None
        self._track_mode = None
        self._leg_held_deep = False
        self._leg_shallow_t = None
        self._leg_rebased = False
        self._mpc_log_t = 0.0
        self._mpc_tm = None
        self._through_blind_logged = False
        self._search_extra = 0.0
        self._corridor = None
        self._lock_hits = 0
        self._reacquire_n = 0
        self._off_tube_t0 = None
        # Hold the vehicle armed for the next plan step instead of disarming
        # when the pass fails (set by coral_behaviours when a rendezvous follows).
        self._fail_hold = os.environ.get("UDRONE_HOOP_FAIL_HOLD", "") == "1"
        self._held = False
        self._ga_wps = []
        self._ga_t0 = None
        self._ga_end = None
        self._ga_time = 0.0
        self._tick_tm = 0.0
        self.elev_from_command = True
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
        self._sight_new = False
        bgr = self._bgr()
        if bgr is None:
            return None
        stamp = self._img[4]
        if stamp == self._sight_stamp:
            return self._sight_hit
        self._sight_stamp = stamp
        self._sight_new = True
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
        hit["center_raw"] = center
        if hit.get("closed"):
            if self._ring_center_filt is None:
                self._ring_center_filt = center
            else:
                prev = self._ring_center_filt
                self._ring_center_filt = hoop_mpc.filter_center(
                    prev, center,
                    alpha=RING_FILTER_ALPHA,
                    step_m=RING_FILTER_STEP_M,
                )
            center = self._ring_center_filt
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

    def _update_visual_trim(self, hit):
        """Small filtered y/z correction; never let vision move the ring plane.

        The authored model remains the primary geometry. A complete ring at
        useful range may remove a few centimetres of camera/map bias, but a
        clipped near-field rim cannot retarget the corridor.
        """
        if (
            not self._sight_new
            or hit is None
            or not hit.get("closed")
            or "center" not in hit
            or not (2.0 <= float(hit.get("z_opt", 0.0)) <= 8.0)
            or not authored_ring_lock(hit["center"])
        ):
            return
        _s, y, z, _fwd, _left, _up = hoop_mpc.hoop_state(
            hit["center"], HOOP_CENTER, HOOP_AXIS,
        )
        target = (
            max(-VISUAL_TRIM_MAX_M, min(VISUAL_TRIM_MAX_M, float(y))),
            max(-VISUAL_TRIM_MAX_M, min(VISUAL_TRIM_MAX_M, float(z))),
        )
        self._vision_trim = tuple(
            (1.0 - VISUAL_TRIM_ALPHA) * float(self._vision_trim[i])
            + VISUAL_TRIM_ALPHA * target[i]
            for i in range(2)
        )

    def _guidance_center(self):
        """Authored centre plus bounded filtered lateral/depth visual trim."""
        fwd, left, up = hoop_mpc.hoop_axes(HOOP_AXIS)
        del fwd
        y, z = self._vision_trim
        return tuple(
            float(HOOP_CENTER[i]) + y * float(left[i]) + z * float(up[i])
            for i in range(3)
        )

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
        if self.stage in ("approach", "commit", "reacquire", "center", "through"):
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
        self._leg_rebased = False
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

    def _note_plane(self, pos):
        """First sample inside the authored opening. Later grazes do not replace it."""
        if self._plane is not None:
            return
        if not in_opening(pos, HOOP_CENTER, HOOP_AXIS, INNER_PASS_M):
            return
        sample = plane_sample(pos, HOOP_CENTER, HOOP_AXIS)
        self._plane = sample
        print(
            f"HOOP_PLANE s={sample['s']:+.2f} y={sample['y']:+.2f} "
            f"z={sample['z']:+.2f} radial={sample['radial']:.2f}",
            flush=True,
        )

    def _clear_pass(self):
        """A retry must earn the opening and the exit again."""
        self._plane = None
        self._real_crossed = False
        self._surrounded = False
        self._crossed = False
        self._real_miss = float("inf")
        self._miss = float("inf")
        self._exit_logged = False

    def _note_real(self, tm, pos):
        miss = math.dist(pos, HOOP_CENTER)
        if miss < self._real_miss:
            self._real_miss = miss
        self._note_plane(pos)
        if self._real_crossed or not in_opening(pos, HOOP_CENTER, HOOP_AXIS, INNER_PASS_M):
            return
        self._real_crossed = True
        print(
            f"HOOP_REAL xyz=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) real_miss={miss:.2f}",
            flush=True,
        )
        if self.stage not in ("approach", "commit", "reacquire", "center", "goaround"):
            self._enter_corridor(float(tm), pos, HOOP_CENTER, HOOP_AXIS, why="opening")

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
        if s in ("commit", "center"):
            return "commit"
        if s in ("approach", "reacquire"):
            return "approach"
        return None

    def _enter_corridor(self, tm, pos, center, axis, why: str = ""):
        """Lock the fix. Join the tube if already on it; otherwise go to the station.

        The through-direction is taken once, from which side the hull is on,
        and is not recomputed if a later graze crosses the ring plane.
        """
        self._center = tuple(float(c) for c in center)
        sample = approach_point(pos, self.vel)
        th = through_dir(sample, self._center, axis if axis is not None else HOOP_AXIS)
        self._axis = (float(th[0]), float(th[1]), 0.0)
        corr = hoop_corridor(
            pos, self._center, self._axis, prev_phase=None, through=self._axis,
        )
        self._corridor = corr
        self._exit = corr["exit"]
        self._through_t0 = None
        self._through_seen = None
        self._through_blind_logged = False
        self._crossed = False
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._vel = (0.0, 0.0, 0.0)
        self._acc = (0.0, 0.0, 0.0)
        self._track_mode = None
        self._off_tube_t0 = None
        self._wps = []
        self._tracking = False
        self._approach_wps = []
        self.stage = "approach"
        print(
            f"HOOP_CORRIDOR track "
            f"center=({self._center[0]:.2f},{self._center[1]:.2f},{self._center[2]:.2f}) "
            f"through=({self._axis[0]:+.2f},{self._axis[1]:+.2f}) {why}",
            flush=True,
        )
        self._apply_track(pos)

    def _reacquire_corridor(self, tm, pos, reason: str):
        """Through failed — return to the approach station and try again."""
        self._reacquire_n += 1
        self._search_extra = max(self._search_extra, SEARCH_RETRY_S)
        if self._reacquire_n > MAX_REACQUIRE:
            self._soft_fail(f"through failed after {MAX_REACQUIRE} corridor reacquires ({reason})")
            return
        self._clear_pass()
        if self._center is None:
            self.stage = "search"
            self._des = None
            print(f"HOOP_REACQUIRE search (no centre) n={self._reacquire_n} {reason}", flush=True)
            return
        corr = hoop_corridor(
            pos, self._center, self._axis, prev_phase=None, through=self._axis,
        )
        self._corridor = corr
        self._exit = corr["exit"]
        self.stage = "approach"
        self.yaw_hold = None
        self._track_mode = None
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
        self._apply_track(pos)

    def _enter_through(self, tm, pos, corr):
        """Gate on-axis — latch a world exit behind the ring and punch open-loop.

        Rim tracking dies in the last metre; do not run MPC or reacquire from
        vision. The force loop chases the frozen ENU exit on the through axis.
        """
        self.stage = "through"
        self._through_t0 = tm
        self._through_seen = tm
        self._through_blind_logged = True  # open-loop: rim loss is expected
        th = corr["through"]
        nxy = math.hypot(float(th[0]), float(th[1])) or 1.0
        ax = (float(th[0]) / nxy, float(th[1]) / nxy, float(th[2]) if len(th) > 2 else 0.0)
        # Freeze centre / axis / exit at punch entry (global ENU).
        if self._center is not None:
            self._center = tuple(float(c) for c in self._center)
        self._axis = ax
        self._exit = tuple(float(c) for c in corr["exit"])
        self._des = self._exit
        self._vel = self._punch_vel(pos, ax)
        self._mpc_warm = None
        self._mpc_v_state = (0.0, 0.0, 0.0)
        self._off_tube_t0 = None
        self._wps = []
        self._sweep_world = [pos, corr.get("gate", pos), self._exit]
        self.yaw_hold = math.atan2(ax[1], ax[0])
        self.pitch_limit = THROUGH_PITCH_RAD
        print(
            f"HOOP_PUNCH open-loop speed={PUNCH_SPEED:.2f} "
            f"gate=({corr['gate'][0]:.2f},{corr['gate'][1]:.2f}) "
            f"exit=({self._exit[0]:.2f},{self._exit[1]:.2f},{self._exit[2]:.2f}) "
            f"axis=({ax[0]:+.2f},{ax[1]:+.2f}) "
            f"y={float(corr['y']):+.2f} z={float(corr['z']):+.2f} "
            f"dist_gate={corr['dist_gate']:.2f}",
            flush=True,
        )

    def _punch_vel(self, pos, ax):
        """Surge along the frozen axis. Speed falls to zero at the exit.

        A constant 0.6 m/s is faster than the position loop can cancel, so
        the hull kept going after the exit. Climb bias only while still short.
        """
        along = 0.0
        if self._exit is not None:
            delta = tuple(float(self._exit[i]) - float(pos[i]) for i in range(3))
            along = sum(delta[i] * float(ax[i]) for i in range(3))
        speed = surge_speed(along, PUNCH_SPEED, taper_m=1.0)
        vx = speed * float(ax[0])
        vy = speed * float(ax[1])
        vz = speed * float(ax[2])
        if self._exit is not None and along > 0.0:
            dz = float(self._exit[2]) - float(pos[2])  # + = need to climb (ENU)
            if dz > 0.12:
                vz = max(vz, min(PUNCH_CLIMB_MPS, 0.8 * dz))
            elif dz < -0.12:
                vz = min(vz, max(-PUNCH_CLIMB_MPS, 0.8 * dz))
        return (vx, vy, vz)

    def _hold_gate(self, pos, corr):
        """Last metre, not yet centered: stop on the gate with the nose free.

        Along-track feedforward here is what carried the hull through the
        ring. The position loop closes the gate on its own.
        """
        entered = self.stage != "center"
        self.stage = "center"
        gate = tuple(float(c) for c in corr["gate"])
        self._des = gate
        dz = float(gate[2]) - float(pos[2])
        vz = 0.0
        if abs(dz) > 0.08:
            vz = max(-PUNCH_CLIMB_MPS, min(PUNCH_CLIMB_MPS, 0.7 * dz))
        self._vel = (0.0, 0.0, vz)
        self.yaw_hold = None
        self.pitch_limit = APPROACH_PITCH_RAD
        self._sweep_world = [pos, gate]
        if entered:
            print(
                f"HOOP_CENTER gate=({gate[0]:.2f},{gate[1]:.2f},{gate[2]:.2f}) "
                f"y={float(corr['y']):+.2f} z={float(corr['z']):+.2f} "
                f"dz={dz:+.2f}",
                flush=True,
            )

    def _shallow(self, pos, z_des) -> bool:
        """True when the hull is shallower than the ring setpoint."""
        return float(pos[2]) - float(z_des) >= DIVE_DEPTH_ERR_M

    def _approach_vel(self, pos, station):
        """Downward feedforward while joining the station. No along-track surge yet."""
        dz = float(station[2]) - float(pos[2])
        if abs(dz) <= DIVE_DEPTH_ERR_M:
            return (0.0, 0.0, 0.0)
        vz = max(-DIVE_VZ_MPS, min(DIVE_VZ_MPS, 0.7 * dz))
        return (0.0, 0.0, vz)

    def _lead_command(self, pos, corr):
        """Setpoint a short lead ahead on the station→gate centreline, at cruise speed."""
        point, _s_cmd, _lead = corridor_lead_point(
            corr["approach"], corr["gate"], corr["s_prog"], corr["length_m"],
            speed_mps=SPEED, lead_s=GLIDE_LEAD_S,
        )
        th = corr["through"]
        vz = 0.0
        dz = float(point[2]) - float(pos[2])
        if abs(dz) > 0.12:
            vz = max(-0.25, min(0.25, 0.7 * dz))
        vel = (SPEED * float(th[0]), SPEED * float(th[1]), vz)
        return point, vel

    def _corridor_yaw(self, pos, corr, *, on_glide: bool):
        """Free the nose until the hull is on the station and at ring depth.

        Yaw lock throws away lateral force. Off the tube, or still shallow,
        the nose has to point at the setpoint or a north/south start never
        reaches the line.
        """
        z_des = float(corr["gate"][2])
        near_station = float(corr["dist_approach"]) <= STATION_LOCK_M
        # Lock only once the hull is actually on the centreline. A wide
        # "near the station" ball still has to steer, and yaw lock cannot.
        on_line = (
            not self._shallow(pos, z_des)
            and abs(float(corr["y"])) <= 0.32
        )
        if on_line and (on_glide or near_station):
            th = corr["through"]
            self.yaw_hold = math.atan2(float(th[1]), float(th[0]))
        else:
            self.yaw_hold = None
        self.pitch_limit = APPROACH_PITCH_RAD

    def _apply_track(self, pos):
        """Pursuit reference: dive, join the centreline, cruise, brake at the exit.

        The nose stays free so the force can point along that curve. Yaw lock
        would throw the lateral part away.
        """
        if self._center is None:
            return
        guidance_center = self._guidance_center()
        step = hoop_mpc.plan(
            pos, guidance_center, self._axis or HOOP_AXIS,
            speed_mps=SPEED, exit_m=EXIT_M, v_state=self.vel,
        )
        self._des = step["point"]
        self._vel = step["vel"]
        self._acc = step.get("acc") or (0.0, 0.0, 0.0)
        self._exit = step["exit"]
        mode = step["mode"]
        if mode != self._track_mode:
            s0, y0, z0 = step["state0"]
            print(
                f"HOOP_TRACK mode={mode} s={s0:+.2f} y={y0:+.2f} z={z0:+.2f} "
                f"v={math.hypot(*self._vel):.2f}",
                flush=True,
            )
            self._track_mode = mode
        s_miss, y_miss, _z_miss = step["state0"]
        if float(s_miss) > hoop_mpc.MISS_S_M and self._plane is None and not self.failed:
            self._start_goaround(self._tick_tm, pos, y_miss)
            if self.stage == "goaround":
                return
        self.stage = {
            "dive": "approach",
            "join": "approach",
            "center": "center",
            "through": "commit",
        }.get(mode, "approach")
        # Short of the plane the nose follows the commanded horizontal vector,
        # slewed. This preserves the planner's continuous lateral closure;
        # the old axis-heading branch discarded it and toggled at |y|=0.40 m.
        s0, y0, z0 = step["state0"]
        fwd = step["fwd"]
        vx, vy, vz = self._vel
        vs = vx * fwd[0] + vy * fwd[1]
        speed_h = math.hypot(vx, vy)
        target = None
        speed = None
        if float(s0) < 0.0:
            axis_yaw = math.atan2(float(fwd[1]), float(fwd[0]))
            if speed_h > 0.02:
                raw = math.atan2(vy, vx)
                offset = (raw - axis_yaw + math.pi) % (2.0 * math.pi) - math.pi
                offset = max(math.radians(-55.0), min(math.radians(55.0), offset))
                target = axis_yaw + offset
                speed = speed_h
            else:
                target = axis_yaw
                speed = 0.0
        if target is None:
            self.yaw_hold = None
        else:
            self.yaw_hold = self._hold_yaw(target, self._slew_arm)
            self._vel = (
                speed * math.cos(self.yaw_hold),
                speed * math.sin(self.yaw_hold),
                vz,
            )
        # Same curve the planner used for the cone, so the nose and the
        # command open together as the depth error grows.
        self.pitch_limit = hoop_mpc.pitch_limit_rad(z0)
        self._corridor = hoop_corridor(
            pos, guidance_center, self._axis,
            prev_phase=self._corridor_prev(),
            through=self._axis,
        )
        self._sweep_world = [pos, self._des, self._exit]

    def _soft_fail(self, why: str):
        """The pass failed but the vehicle is fine.

        Under a plan that continues (``UDRONE_HOOP_FAIL_HOLD``), finish armed
        so the base hands the vehicle to the hold runner for the next step.
        Otherwise disarm as before.
        """
        if self._fail_hold and self.keep_armed:
            print(f"FAIL {why} (holding armed for the next step)", flush=True)
            self.failed = why
            self._held = True
            return
        self._fail(why)

    def _start_goaround(self, tm, pos, y0):
        """Past the plane with no opening sample: loop beside the ring and retry.

        Backing straight up the axis would hit the ring, and waiting at the
        exit only runs out the budget.
        """
        self._reacquire_n += 1
        if self._reacquire_n > MAX_REACQUIRE:
            self._soft_fail(f"missed the opening {MAX_REACQUIRE} times")
            return
        axis = self._axis or HOOP_AXIS
        center = self._center or HOOP_CENTER
        fwd, left, up = hoop_mpc.hoop_axes(axis)
        side = 1.0 if float(y0) >= 0.0 else -1.0
        s_now = max(hoop_mpc.MISS_S_M, hoop_mpc.hoop_state(pos, center, axis)[0])
        world = [
            tuple(float(p) for p in pos),
            hoop_mpc._world(center, fwd, left, up, s_now, side * GOAROUND_SIDE_M, 0.0),
            hoop_mpc._world(center, fwd, left, up, -GOAROUND_BACK_M, side * GOAROUND_SIDE_M, 0.0),
            hoop_mpc._world(center, fwd, left, up, -GOAROUND_BACK_M, 0.0, 0.0),
        ]
        self._ga_wps = [self._path_of(p) for p in world]
        self._ga_end = world[-1]
        self._ga_t0 = float(tm)
        length = path_length_3d(self._ga_wps)
        self._ga_time = length / GOAROUND_MPS
        self._search_extra += self._ga_time + 30.0
        self._clear_pass()
        self.stage = "goaround"
        self._track_mode = None
        self._sweep_world = list(world)
        print(
            f"HOOP_GOAROUND n={self._reacquire_n}/{MAX_REACQUIRE} side={side:+.0f} "
            f"len={length:.1f} m",
            flush=True,
        )

    def _goaround_tick(self, tm, pos):
        """Nose along the loop; back into the corridor at the approach station."""
        ref = polyline_pos_vel(self._ga_wps, GOAROUND_MPS, max(0.0, tm - self._ga_t0))
        v_fwd, v_left = ref[1][0], ref[1][1]
        if math.hypot(v_fwd, v_left) > 0.05:
            yaw0 = self.anchor[2]
            self.yaw_hold = self._hold_yaw(math.atan2(v_left, v_fwd) + yaw0, tm)
        center = self._center or HOOP_CENTER
        self.pitch_limit = hoop_mpc.pitch_limit_rad(float(pos[2]) - float(center[2]))
        if math.dist(pos, self._ga_end) < 0.7 or tm - self._ga_t0 > self._ga_time + 20.0:
            self._enter_corridor(
                tm, pos, center, self._axis or HOOP_AXIS,
                why=f"go-around {self._reacquire_n}",
            )

    def _update_corridor(self, tm, pos):
        """The through-direction stays at the value frozen on entry."""
        del tm
        self._apply_track(pos)

    def _des_helix(self):
        if self._des is None or self.anchor is None:
            z = self._prev[1][2] if self._prev else SEARCH_Z
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(self._des[0] - x0, self._des[1] - y0, yaw0)
        c, s = math.cos(yaw0), math.sin(yaw0)
        vx, vy, vz = self._vel
        path_v = (c * vx + s * vy, -s * vx + c * vy, vz)
        ax, ay, az = self._acc if self._acc is not None else (0.0, 0.0, 0.0)
        path_a = (c * ax + s * ay, -s * ax + c * ay, az)
        return (fwd, left, self._des[2]), path_v, path_a

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
        ref = self._helix_raw(tm)
        # The HUD previews _helix at future times, and again at this tm.
        # Only the live control sample may advance the slew.
        if self._slew_arm != tm or self._slew_tm == tm:
            return ref
        self._slew_tm = tm
        return self._slew(ref, tm)

    def _slew(self, ref, tm):
        """Rate-limit the velocity the force loop will track."""
        pos, vel = ref[0], ref[1]
        dt = 0.05
        if self._slew_prev_tm is not None:
            dt = float(tm) - self._slew_prev_tm
        self._slew_prev_tm = float(tm)
        new_v, acc = hoop_mpc.slew_velocity(self._slew_v, vel, dt)
        self._slew_v = new_v
        return pos, new_v, acc

    def _hold_yaw(self, target, tm) -> float:
        """Slew the held heading so a new bearing does not snap the nose."""
        dt = 0.05
        if self._yaw_tm is not None and tm is not None:
            dt = float(tm) - float(self._yaw_tm)
        if tm is not None:
            self._yaw_tm = float(tm)
        self._yaw_cmd = slew_angle(self._yaw_cmd, target, dt, rate=0.45)
        return self._yaw_cmd

    def _helix_raw(self, tm: float):
        if self.stage == "goaround" and self._ga_wps and self._ga_t0 is not None:
            return polyline_pos_vel(self._ga_wps, GOAROUND_MPS, max(0.0, tm - self._ga_t0))
        if self.stage in ("approach", "commit", "reacquire", "center", "through"):
            return self._des_helix()
        if self._tracking and self._approach_wps and self._approach_t0 is not None:
            return polyline_pos_vel(
                self._approach_wps, SPEED, max(0.0, tm - self._approach_t0),
            )
        if self._tracking and self._approach is not None and self.anchor is not None:
            return self._path_of(self._approach), (0.0, 0.0, 0.0)
        if self._search_wps and self._leg_t0 is not None:
            # Finish the dive before the leg walks forward. A time-based
            # polyline otherwise moves sideways while the hull is still shallow
            # and the nose pitches through the ring.
            z_now = self._prev[1][2] if self._prev else 0.0
            hold, self._leg_held_deep, self._leg_t0, self._leg_shallow_t = leg_clock(
                z_now, SEARCH_Z, 0.40, self._leg_held_deep, self._leg_t0, tm,
                self._leg_shallow_t,
            )
            if hold:
                # Nose-down glide toward the active leg while depth closes.
                # Pure vertical thrust is impossible on this body-x-only hull;
                # extending it caused 6.6 m of uncontrolled horizontal drift.
                if self.anchor is not None and self._prev is not None:
                    hx, hy = self._path_xy(self._prev[1][0], self._prev[1][1])
                    return hoop_mpc.search_glide_reference(
                        (hx, hy, z_now), self._search_wps[-1], SEARCH_Z,
                        max_vz=DIVE_VZ_MPS,
                    )
                return self._search_wps[0], (0.0, 0.0, 0.0)
            if not self._leg_rebased and self._prev is not None:
                # Begin the timed leg exactly where the glide ended. The old
                # polyline still began at the arm pose and made a 2.61 m jump.
                hx, hy = self._path_xy(self._prev[1][0], self._prev[1][1])
                self._search_wps = [
                    (hx, hy, float(z_now)),
                    self._search_wps[-1],
                ]
                self._leg_t0 = float(tm)
                self._leg_rebased = True
                length = path_length_3d(self._search_wps)
                self._leg_time = length / SPEED
                self._leg_limit = max(8.0, self._leg_time + 18.0)
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
        self._leg_held_deep = False
        self._leg_shallow_t = None
        self._leg_rebased = False
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
        self._clear_pass()
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
        self._leg_rebased = False
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
                through=self._axis,
            )
            out["paths"].extend(gp["paths"])
            out["markers"].extend(gp["markers"])
            if corr is not None:
                out["lines"].append(
                    f"corridor {corr['phase']}  s {corr['s']:+.2f}  "
                    f"lat {corr['lat']:.2f}/{corr['half_m']:.2f}  "
                    f"reacq {self._reacquire_n}"
                )
            if self._plane is not None:
                p = self._plane
                out["lines"].append(
                    f"plane s {p['s']:+.2f} y {p['y']:+.2f} "
                    f"z {p['z']:+.2f} r {p['radial']:.2f}"
                )
            if self.stage == "center" and corr is not None:
                out["lines"].append(
                    f"centering y {float(corr['y']):+.2f} z {float(corr['z']):+.2f}"
                )
            elif self.stage == "through":
                out["lines"].append("open-loop exit")
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
        self._slew_arm = tm
        self._tick_tm = float(tm)
        pos, q = auv
        self.yaw_hold = None
        if self._relock and self._search_end is not None:
            dx = self._search_end[0] - pos[0]
            dy = self._search_end[1] - pos[1]
            if math.hypot(dx, dy) > 0.60:
                self.yaw_hold = math.atan2(dy, dx)
        elif self.stage == "search" and self._search_end is not None:
            # Point along the leg for the whole dive, so the nose does not
            # follow a sideways correction.
            dx = self._search_end[0] - pos[0]
            dy = self._search_end[1] - pos[1]
            if math.hypot(dx, dy) > 0.40:
                self.yaw_hold = self._hold_yaw(math.atan2(dy, dx), tm)
        if pos[2] < HOOP_FLOOR:
            # Climbing from a deep prior leg (e.g. bed-tag approach) is allowed;
            # only fail if we stay deep without rising toward the ring band.
            climbing = (
                self._prev is not None
                and pos[2] > float(self._prev[1][2]) + 0.05
            )
            if climbing:
                self._deep_t = None
            elif self._deep_t is None:
                self._deep_t = tm
            elif tm - self._deep_t > 4.0:
                self._fail("depth ran away from the hoop")
                return self._act(tm, auv, boat, self._sight_hit, (0.0, 0.0, 0.0, 0.0))
        else:
            self._deep_t = None
        self._note_real(tm, pos)
        if self.stage == "goaround":
            hit = self._sight(tm, pos, q)
            self._goaround_tick(tm, pos)
            return self._act(tm, auv, boat, hit)
        if self.stage in ("approach", "commit", "reacquire", "center", "through"):
            hit = self._sight(tm, pos, q)
            self._update_visual_trim(hit)
            if hit is not None and hit.get("closed") and "center" in hit:
                self._last_hit_t = tm
                self._last_center = hit["center"]
            self._update_corridor(tm, pos)
            if self.stage == "through":
                return self._act(tm, auv, boat, hit)
            return self._act(tm, auv, boat, hit)

        if self.stage == "through":
            # Open-loop punch: frozen ENU exit + axis. No MPC, no rim update,
            # no off-tube reacquire — chase _des via _des_helix until exit.
            ax = self._axis or (1.0, 0.0, 0.0)
            self.yaw_hold = math.atan2(float(ax[1]), float(ax[0]))
            self.pitch_limit = THROUGH_PITCH_RAD
            if self._exit is not None:
                self._des = self._exit
                self._vel = self._punch_vel(pos, ax)
                self._sweep_world = [pos, self._exit]
            hit = self._sight(tm, pos, q)  # surround / HUD only — never retarget
            self._note_surround(hit)
            inside = (
                self._center is not None
                and in_opening(pos, self._center, self._axis, INNER_PASS_M)
            )
            if inside and not self._crossed:
                self._crossed = True
                sample = plane_sample(pos, self._center, self._axis)
                print(
                    f"HOOP_THROUGH s={sample['s']:+.2f} y={sample['y']:+.2f} "
                    f"z={sample['z']:+.2f} radial={sample['radial']:.2f}",
                    flush=True,
                )
            return self._act(tm, auv, boat, hit)

        hit = self._sight(tm, pos, q)
        self._update_visual_trim(hit)
        if hit is not None:
            self._note_slide(pos, hit)
        # Any closed detection of the authored ring starts the corridor,
        # including inside 1.6 m where the old commit gate had already given
        # up. Do not keep the vision lead on that frame — its endpoint walks
        # through and below the opening.
        lock_match = (
            hit is not None
            and hit.get("closed")
            and "center" in hit
            and authored_ring_lock(hit["center"])
        )
        if lock_match:
            if self._sight_new:
                self._lock_hits += 1
            if self._lock_hits >= LOCK_FRAMES and self.anchor is not None:
                self._fix_samples.clear()
                self._enter_corridor(
                    tm, pos, HOOP_CENTER, HOOP_AXIS,
                    why=f"authored lock ({LOCK_FRAMES} frames)",
                )
            return self._act(tm, auv, boat, hit)
        if self._sight_new:
            self._lock_hits = 0
        if hit is not None and hit.get("closed"):
            # Not the authored hoop. Do not aim at this vision centre: the
            # median used to start a corridor through and below the opening.
            self._steer(tm, pos, rpy_from_q(q)[2])
            if self.failed or self.done:
                return self._act(tm, auv, boat, hit, (0.0, 0.0, 0.0, 0.0))
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

    def _verdict(self, pos=None) -> dict:
        """One score for the flight stopper and the process exit.

        Pass means the authored opening was crossed and the hull later
        reached the frozen exit. A camera surround and punch time do not.
        """
        if pos is None and self._prev is not None:
            pos = self._prev[1]
        exit_miss = None
        if pos is not None and self._exit is not None:
            dist = math.dist(pos, self._exit)
            if dist < self._miss:
                self._miss = dist
            exit_miss = self._miss
        elif math.isfinite(self._miss):
            exit_miss = self._miss
        frame = None
        if pos is not None:
            frame = plane_sample(pos, self._center or HOOP_CENTER, self._axis or HOOP_AXIS)
        passed = (not self.failed) and hoop_pass(self._plane, exit_miss, exit_m=EXIT_ARRIVE_M)
        plane = self._plane or {}
        if self.failed:
            why = str(self.failed)
        elif passed:
            why = (
                f"crossed radial={float(plane.get('radial', float('nan'))):.2f} m "
                f"exit_miss={float(exit_miss):.2f} m"
            )
        elif not self._plane:
            why = "did not cross the opening"
        else:
            shown = float(exit_miss) if exit_miss is not None else float("inf")
            why = f"crossed the opening but {shown:.2f} m short of the exit"

        def _num(value):
            if value is None:
                return None
            try:
                number = float(value)
            except (TypeError, ValueError):
                return None
            return None if not math.isfinite(number) else round(number, 3)

        return {
            "pass": bool(passed),
            "why": why,
            "stage": self.stage,
            "plane": self._plane,
            "exit_miss_m": _num(exit_miss),
            "exit_s": None if frame is None else _num(frame["s"]),
            "exit_y": None if frame is None else _num(frame["y"]),
            "exit_z": None if frame is None else _num(frame["z"]),
            "reacquire_n": int(self._reacquire_n),
            "real_crossed": bool(self._real_crossed),
            "vision_crossed": bool(self._crossed),
            "real_miss_m": _num(self._real_miss),
            "held": bool(self._held),
        }

    def _log_stop(self, verdict: dict):
        if self._exit_logged:
            return
        self._exit_logged = True
        print(
            f"HOOP_EXIT exit_miss={verdict['exit_miss_m']} "
            f"s={verdict['exit_s']} y={verdict['exit_y']} z={verdict['exit_z']} "
            f"reacq={verdict['reacquire_n']} stage={verdict['stage']}",
            flush=True,
        )

    def _write_verdict(self, verdict: dict):
        path = os.environ.get("UDRONE_HOOP_VERDICT", "/tmp/coral_hoop_verdict.json")
        try:
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(verdict, handle)
        except OSError as ex:
            print(f"HOOP_VERDICT write failed {ex}", flush=True)
        print("HOOP_VERDICT " + json.dumps(verdict), flush=True)

    def _finished(self, tm: float) -> bool:
        if self.failed:
            return True
        pos = self._prev[1] if self._prev else None
        verdict = self._verdict(pos)
        if verdict["pass"]:
            self._log_stop(verdict)
            print(f"HOOP_ARRIVE {verdict['why']}", flush=True)
            return True
        if self.stage == "search":
            deadline = SEARCH_BUDGET_S + self._search_extra
            if tm >= deadline:
                self._soft_fail(f"hoop not found in {deadline:.0f}s")
                return True
            return False
        if self.stage in ("approach", "commit", "reacquire", "center", "goaround"):
            deadline = SEARCH_BUDGET_S + THROUGH_BUDGET_S + self._search_extra
            if tm >= deadline:
                self._soft_fail(verdict["why"])
                return True
            return False
        elapsed = tm - (self._through_t0 or tm)
        if elapsed >= THROUGH_BUDGET_S:
            if self._reacquire_n < MAX_REACQUIRE and pos is not None:
                self._reacquire_corridor(tm, pos, verdict["why"])
                return False
            self._soft_fail(verdict["why"])
            return True
        return False


def main():
    rclpy.init()
    node = UdroneHoopController()
    ok = False
    verdict_path = os.environ.get("UDRONE_HOOP_VERDICT", "/tmp/coral_hoop_verdict.json")
    try:
        os.remove(verdict_path)
    except OSError:
        pass
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_hoop_ctrl_log.txt")
        verdict = node._verdict()
        if verdict["pass"]:
            if not (node.armed_ok and node.log):
                verdict["pass"] = False
                verdict["why"] = "did not pass through the opening"
            else:
                rotor = [r[18] for r in node.log]
                mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
                if mean_rotor < 0.2:
                    verdict["pass"] = False
                    verdict["why"] = "thrusters were not driven"
        node._log_stop(verdict)
        node._write_verdict(verdict)
        ok = bool(verdict["pass"])
        print(f"HOOP_BEHAVIOUR {'PASS' if ok else 'FAIL'} {verdict['why']}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
