#!/usr/bin/env python3
"""Search for the Kiwanis hula hoop and swim through the opening.

The search keeps a 2 m coverage grid of a 20 m box around the dock until the
front camera sees the orange ring. The measurement is the bounding box of
the orange pixels (the tangent limbs). Pitch is removed before the centre
test. Three agreeing fixes become one forward sweep on the force loop:
ease onto the line through the ring, then straight through the opening.
A debug image on ``/xudrone/hoop_debug/image`` draws the ring, any AprilTag
in the frame, and the command arrow. Coral command type ``hoop``.
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

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

from hoop_geom import (
    INNER_PASS_M,
    RING_OUTER_M,
    SEARCH_HITS,
    SEARCH_Z,
    WEDGE_NEAR_M,
    CoverageMap,
    horizontal_axis,
    in_opening,
    level_feature,
    look_leg,
    orange_ring,
    ring_center,
    sweep_points,
)
from patterns import path_length_3d, polyline_pos_vel, world_offset_to_path
from udrone_sinusoidal_controller import (
    UdroneHelixController,
    body_axes,
    rpy_from_q,
    spin_until_done,
)

SPEED = 0.22
LEAD_M = 2.4
EXIT_M = 2.0
SEARCH_BUDGET_S = 200.0
THROUGH_BUDGET_S = 80.0
COMMIT_RANGE = (2.4, 7.0)   # start the sweep while still in front of the ring
CENTER_PX = 0.35            # levelled lateral error before the sweep is planned
CENTER_PY = 0.45            # levelled vertical error
COMMIT_ASPECT = 0.55        # minor/major; near enough to the ring axis to pass
LOST_S = 1.5
SLEW_MPS = 0.25             # how fast the approach point may move
FILT_S = 0.30               # image-feature time constant
NOSE_DOWN_CAP = 0.85        # rad; past this, do not command a deeper setpoint


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
        self._wps = []
        self._through_t0 = None
        self._center = None
        self._axis = (1.0, 0.0, 0.0)
        self._exit = None
        self._crossed = False
        self._miss = float("inf")
        self._approach = None
        self._approach_wps = []
        self._approach_t0 = None
        self._approach_stand = None
        self._slew_tm = None
        self._filt = None
        self._track_log_t = 0.0
        self._last_hit_t = -LOST_S
        self._tracking = False
        self._deep_t = None
        self._dumped = False
        self._last_center = None
        self._sweep_world = []
        self._tags = []
        self._frame = None
        self._use_cv2 = _cv2_ok()
        info_topic = os.environ.get("UDRONE_HOOP_INFO_TOPIC", "/xudrone/camera/camera_info")
        image_topic = os.environ.get("UDRONE_HOOP_IMAGE_TOPIC", "/xudrone/camera/image")
        debug_topic = os.environ.get("UDRONE_HOOP_DEBUG_TOPIC", "/xudrone/hoop_debug/image")
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self._debug_pub = self.create_publisher(Image, debug_topic, 2)
        if AprilTagDetectionArray is not None:
            det_topic = os.environ.get("UDRONE_HOOP_DET_TOPIC", "/xudrone/detections")
            self.create_subscription(
                AprilTagDetectionArray, det_topic, self._on_tag, qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"hoop: opencv ring outer={RING_OUTER_M:g} m "
            f"cv2={'yes' if self._use_cv2 else 'MISSING'} debug={debug_topic}"
        )

    def _on_info(self, m: CameraInfo):
        k = list(m.k)
        if len(k) >= 6 and k[0] > 1.0 and k[4] > 1.0:
            self.fx, self.fy = float(k[0]), float(k[4])
            self.cx, self.cy = float(k[2]), float(k[5])

    def _on_image(self, m: Image):
        self._img = (m.encoding or "", int(m.width), int(m.height), bytes(m.data), time.time())

    def _on_tag(self, m):
        tags = []
        now = time.time()
        for d in m.detections:
            pts = [(float(c.x), float(c.y)) for c in d.corners]
            if len(pts) < 4:
                continue
            tags.append((int(d.id), pts, (float(d.centre.x), float(d.centre.y)), now))
        self._tags = tags

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
        if frame is not None:
            self._frame = frame
        return frame

    def _events(self, tm: float):
        if not self._use_cv2:
            self._fail("hoop rejected: opencv is not importable")
            return
        if self.stage == "search" and self.latch.state == "attached":
            self._fail("hoop rejected: xudrone is latched; detach first")

    def _smooth(self, hit, tm):
        """Low-pass the limb measurement so one frame cannot yank the setpoint."""
        now = float(tm)
        prev = self._filt
        if prev is None or now - prev["t"] > 1.0:
            hit["t"] = now
            self._filt = dict(hit)
            return hit
        dt = max(1e-3, now - prev["t"])
        a = 1.0 - math.exp(-dt / FILT_S)
        for key in ("u", "v", "major", "minor"):
            hit[key] = prev[key] + a * (hit[key] - prev[key])
        hit["aspect"] = hit["minor"] / max(hit["major"], 1.0)
        hit["t"] = now
        self._filt = dict(hit)
        return hit

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
        hit = self._smooth(dict(hit), tm)
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
        if center[2] > -0.35 or center[2] < -1.80:
            return
        stamp = self._img[4] if self._img is not None else None
        if stamp == self._fix_stamp:
            return
        self._fix_stamp = stamp
        self._fixes.append(center)
        self._fixes = self._fixes[-5:]
        if len(self._fixes) < 3:
            return
        med = tuple(sorted(p[i] for p in self._fixes)[len(self._fixes) // 2] for i in range(3))
        if max(math.dist(p, med) for p in self._fixes[-3:]) > 0.40:
            return
        if self.anchor is None:
            return
        points, axis = sweep_points(pos, med, LEAD_M, EXIT_M)
        self._center = med
        self._axis = axis
        self._exit = points[-1]
        self._sweep_world = points
        self._wps = [self._path_of(p) for p in points]
        self._through_t0 = tm
        self.stage = "through"
        self._fixes.clear()
        print(
            f"HOOP_SWEEP center=({med[0]:.2f},{med[1]:.2f},{med[2]:.2f}) "
            f"axis=({axis[0]:+.2f},{axis[1]:+.2f}) "
            f"ease=({points[1][0]:.2f},{points[1][1]:.2f},{points[1][2]:.2f}) "
            f"end=({points[-1][0]:.2f},{points[-1][1]:.2f},{points[-1][2]:.2f})",
            flush=True,
        )

    def _aim(self, pos, hit):
        """World point the force loop should chase, before the slew limit.

        A closed ring uses the estimated centre and stops LEAD_M short of it.
        An arc only gives a bearing, so the aim is a short step along the
        horizontal look direction, shifted by the lateral image error.
        Depth stays near the present depth when the nose is already steep
        or the ring is close to the top or bottom of the frame.
        """
        fwd = hit["fwd"]
        ax = horizontal_axis(fwd)
        left_h = (-ax[1], ax[0])
        hold_depth = abs(hit["y_n"]) > 0.50 or -fwd[2] > math.sin(NOSE_DOWN_CAP)
        if hit["closed"] and hit["z_opt"] <= 12.0:
            c = hit["center"]
            de = c[0] - pos[0]
            dn = c[1] - pos[1]
            dist = math.hypot(de, dn)
            if dist < 0.3:
                ux, uy = ax[0], ax[1]
            else:
                ux, uy = de / dist, dn / dist
            back = min(LEAD_M, max(0.0, dist - 0.6))
            z = pos[2] if hold_depth else min(-0.65, max(-1.35, c[2]))
            return (c[0] - ux * back, c[1] - uy * back, z)
        step = 1.5
        z = pos[2]
        return (
            pos[0] + step * ax[0] - hit["x_n"] * step * left_h[0],
            pos[1] + step * ax[1] - hit["x_n"] * step * left_h[1],
            z,
        )

    def _slew(self, tm, target):
        if self._approach is None:
            self._approach = target
            self._slew_tm = tm
            return
        dt = 0.05 if self._slew_tm is None else max(0.0, min(0.2, tm - self._slew_tm))
        self._slew_tm = tm
        step = SLEW_MPS * dt
        delta = tuple(target[i] - self._approach[i] for i in range(3))
        dist = math.sqrt(sum(v * v for v in delta))
        if dist <= step or dist < 1e-3:
            self._approach = target
            return
        scale = step / dist
        self._approach = tuple(self._approach[i] + scale * delta[i] for i in range(3))

    def _plan_approach(self, tm, pos, hit):
        """Replace the lead-in with a forward sweep. The path starts at the
        current pose, so a replan does not jump the setpoint.

        Only the run up to the standoff is flown here. The opening and the
        exit are added once _commit freezes the line.
        """
        center = hit["center"]
        if center[2] > -0.35 or center[2] < -1.80:
            return
        if self.anchor is None:
            return
        points, axis = sweep_points(pos, center, LEAD_M, EXIT_M)
        stand = points[2]
        age = 0.0 if self._approach_t0 is None else tm - self._approach_t0
        moved = (
            self._approach_stand is None
            or math.dist(self._approach_stand, stand) > 0.55
        )
        if self._approach_wps and age < 1.2:
            return
        if self._approach_wps and not moved:
            return
        self._approach_stand = stand
        self._approach_t0 = tm
        self._approach_wps = [self._path_of(p) for p in points[:3]]
        self._sweep_world = list(points[:3])
        self._axis = axis
        print(
            f"HOOP_APPROACH stand=({stand[0]:.2f},{stand[1]:.2f},{stand[2]:.2f}) "
            f"axis=({axis[0]:+.2f},{axis[1]:+.2f}) range={hit['z_opt']:.1f}",
            flush=True,
        )

    def _track(self, tm, pos, hit):
        """Follow a forward sweep on the force loop. No rate-mode servo."""
        if self.anchor is None:
            return
        if not self._tracking:
            self._tracking = True
            self._approach = (pos[0], pos[1], pos[2])
            self._slew_tm = tm
            self._prev = None
            self.vel = (0.0, 0.0, 0.0)
        self._plan_approach(tm, pos, hit)
        if time.time() >= self._track_log_t:
            self._track_log_t = time.time() + 2.0
            ap = self._approach_stand or self._approach
            print(
                f"HOOP_TRACK to=({ap[0]:.1f},{ap[1]:.1f},{ap[2]:.2f}) "
                f"range={hit['z_opt']:.1f} aspect={hit['aspect']:.2f} "
                f"level=({hit['x_l']:+.2f},{hit['y_l']:+.2f})",
                flush=True,
            )

    def _drop_track(self):
        self._approach = None
        self._approach_wps = []
        self._approach_t0 = None
        self._approach_stand = None
        self._tracking = False
        self._slew_tm = None
        self._filt = None
        self._search_wps = []
        self._leg_t0 = None
        self._goal_cell = None
        self._prev = None
        self.vel = (0.0, 0.0, 0.0)

    def _path_of(self, enu):
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(enu[0] - x0, enu[1] - y0, yaw0)
        return (fwd, left, enu[2])

    def _helix(self, tm: float):
        if self.stage == "through":
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

    def _steer(self, tm, pos, yaw):
        if self.anchor is None:
            return
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

    def _project(self, world_pt, pos, q):
        """Image pixel of a world point in front of the camera, or None."""
        fwd, left, up = body_axes(q)
        dx = float(world_pt[0]) - float(pos[0])
        dy = float(world_pt[1]) - float(pos[1])
        dz = float(world_pt[2]) - float(pos[2])
        ahead = dx * fwd[0] + dy * fwd[1] + dz * fwd[2]
        if ahead < 0.25:
            return None
        left_m = dx * left[0] + dy * left[1] + dz * left[2]
        up_m = dx * up[0] + dy * up[1] + dz * up[2]
        # Image +x is hull -Y, image +y is hull -Z.
        return (
            self.cx + self.fx * (-left_m / ahead),
            self.cy + self.fy * (-up_m / ahead),
        )

    def _publish_debug(self, pos, q, hit, cmd):
        """Camera image with the ring, any AprilTag, and the command arrow."""
        self._bgr()
        frame = self._frame
        if frame is None or not self._use_cv2 or self._img is None:
            return
        stamp = self._img[4]
        if stamp == getattr(self, "_debug_stamp", None):
            return
        self._debug_stamp = stamp
        import array
        import cv2
        import numpy as np

        img = frame.copy()
        height, width = img.shape[:2]

        def clip(pt):
            return (
                int(max(0, min(width - 1, pt[0]))),
                int(max(0, min(height - 1, pt[1]))),
            )

        now = time.time()
        for tid, pts, centre, seen in self._tags:
            if now - seen > 0.4 or len(pts) < 4:
                continue
            poly = np.array([clip(p) for p in pts], np.int32)
            cv2.polylines(img, [poly], True, (255, 200, 0), 2)
            cv2.putText(
                img, f"tag {tid}", clip(centre),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1, cv2.LINE_AA,
            )
        if hit is not None:
            bw = float(hit.get("w", hit["major"]))
            bh = float(hit.get("h", hit["minor"]))
            x0 = int(hit["u"] - bw / 2.0)
            y0 = int(hit["v"] - bh / 2.0)
            color = (0, 220, 0) if hit.get("closed") else (0, 180, 255)
            cv2.rectangle(img, (x0, y0), (x0 + int(bw), y0 + int(bh)), color, 2)
            cv2.drawMarker(img, clip((hit["u"], hit["v"])), color, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(
                img,
                f"ring {hit['z_opt']:.1f}m asp {hit['aspect']:.2f} "
                f"lvl {hit['x_l']:+.2f},{hit['y_l']:+.2f}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
            )
        aim = None
        if len(self._sweep_world) >= 4:
            aim = self._sweep_world[3]
        elif len(self._sweep_world) >= 3:
            aim = self._sweep_world[2]
        elif self._approach is not None:
            aim = self._approach
        if aim is not None:
            proj = self._project(aim, pos, q)
            if proj is not None:
                cv2.arrowedLine(
                    img, clip((self.cx, self.cy)), clip(proj),
                    (255, 0, 255), 2, tipLength=0.18,
                )
        thrust, _roll_u, pitch_u, yaw_u = cmd
        # +yaw command is nose left (image left). +pitch command is nose down.
        tip = clip((self.cx - float(yaw_u) * 140.0, self.cy + float(pitch_u) * 140.0))
        cv2.arrowedLine(img, clip((self.cx, self.cy)), tip, (0, 255, 255), 2, tipLength=0.28)
        cv2.putText(
            img,
            f"{self.stage} thrust {thrust:+.2f} pitch {pitch_u:+.2f} yaw {yaw_u:+.2f}",
            (8, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA,
        )
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "xudrone/front_camera"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = array.array("B", img.tobytes())
        self._debug_pub.publish(msg)
        if time.time() >= getattr(self, "_debug_jpg_t", 0.0):
            self._debug_jpg_t = time.time() + 1.0
            try:
                cv2.imwrite("/tmp/hoop_debug.jpg", img)
            except Exception:
                pass

    def _act(self, tm, auv, boat, hit, cmd=None):
        if cmd is None:
            cmd = UdroneHelixController._control(self, tm, auv, boat)
        try:
            self._publish_debug(auv[0], auv[1], hit, cmd)
        except Exception as exc:
            if not getattr(self, "_debug_err", False):
                self._debug_err = True
                print(f"HOOP_DEBUG {exc}", flush=True)
        return cmd

    def _control(self, tm, auv, boat):
        pos, q = auv
        if pos[2] < -1.80:
            if self._deep_t is None:
                self._deep_t = tm
            elif tm - self._deep_t > 4.0:
                self._fail("depth ran away from the hoop")
                return self._act(tm, auv, boat, self._sight_hit, (0.0, 0.0, 0.0, 0.0))
        else:
            self._deep_t = None
        if self.stage == "through":
            if self._center is not None and in_opening(pos, self._center, self._axis, INNER_PASS_M):
                if not self._crossed:
                    self._crossed = True
                    print(
                        f"HOOP_THROUGH xyz=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})",
                        flush=True,
                    )
            return self._act(tm, auv, boat, self._sight_hit)

        hit = self._sight(tm, pos, q)
        usable = (
            hit is not None
            and hit["closed"]
            and hit["aspect"] >= 0.50
            and 1.6 <= hit["z_opt"] <= 10.0
        )
        if usable:
            self._last_hit_t = tm
            self._last_center = hit["center"]
            self._commit(tm, pos, hit)
            if self.stage == "through":
                return self._act(tm, auv, boat, hit)
            self._track(tm, pos, hit)
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
        if self.stage != "through":
            if tm >= SEARCH_BUDGET_S:
                self._fail(f"hoop not found in {SEARCH_BUDGET_S:.0f}s")
                return True
            return False
        pos = self._prev[1] if self._prev else None
        if pos is not None and self._exit is not None:
            self._miss = math.dist(pos, self._exit)
        elapsed = tm - (self._through_t0 or tm)
        if self._crossed and self._miss <= 1.2 and elapsed > 2.0:
            print(f"HOOP_ARRIVE miss={self._miss:.2f} m crossed=1", flush=True)
            return True
        if elapsed >= THROUGH_BUDGET_S:
            if not self._crossed:
                self._fail(f"missed the opening (miss {self._miss:.1f} m)")
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
        if node.armed_ok and node.log and not node.failed and node._crossed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            else:
                ok = True
                why = f"ok through miss={node._miss:.2f} m"
        elif not node.failed:
            why = "did not pass through the opening"
        print(f"HOOP_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
