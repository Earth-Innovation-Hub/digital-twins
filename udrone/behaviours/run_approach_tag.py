#!/usr/bin/env python3
"""AUV approach: a level spiral while the down camera searches, then a
front-camera line-of-sight lock.

Spiral: one heading around the seed, radius growing from zero, nose near
level so the downward camera keeps the bed in view. The first down-camera
sighting of the tag ends the search.

Lock: the front camera takes over. Yaw and pitch bring the tag to the
centre of the front image and a steady surge closes along that line.
Within 2 m the hull levels and brakes, then climbs to about 6 m depth
with the nose near level. That is the handoff the hoop can leave from.
EchoRanger clearance is still the last gate on the way in.
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
    body_axes,
    clamp,
    rpy_from_q,
    spin_until_done,
    wrap_pi,
)
from servo_geom import nadir_body

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # host without msgs
    AprilTagDetectionArray = None

LAM_YAW = float(os.environ.get("UDRONE_IBVS_LAM_YAW", "1.4"))
LAM_PITCH = float(os.environ.get("UDRONE_IBVS_LAM_PITCH", "1.4"))
RATE_MAX = float(os.environ.get("UDRONE_IBVS_RATE_MAX", "0.30"))
LOS_THRUST = float(os.environ.get("UDRONE_IBVS_LOS_THRUST", "0.12"))
HOLD_THRUST = float(os.environ.get("UDRONE_IBVS_HOLD_THRUST", "0.08"))
ARRIVE_RANGE_M = float(os.environ.get("UDRONE_IBVS_ARRIVE_RANGE", "2.0"))
BRAKE_S = float(os.environ.get("UDRONE_IBVS_BRAKE_S", "0.5"))
BRAKE_THRUST = float(os.environ.get("UDRONE_IBVS_BRAKE_THRUST", "-0.12"))
# Hoop search floor is -6.2. Hand off above it, nose near level, so the
# ring pass can climb instead of starting on the bed.
EGRESS_Z = float(os.environ.get("UDRONE_IBVS_EGRESS_Z", "-6.0"))
EGRESS_S = float(os.environ.get("UDRONE_IBVS_EGRESS_S", "40"))
LOST_S = float(os.environ.get("UDRONE_IBVS_LOST_S", "2.5"))

# Nearly level, so the fixed down camera keeps looking at the bed.
SEARCH_PITCH0 = float(os.environ.get("UDRONE_IBVS_SEARCH_PITCH", "0.12"))
SEARCH_THRUST = float(os.environ.get("UDRONE_IBVS_SEARCH_THRUST", "0.10"))
SEARCH_R_GROW = float(os.environ.get("UDRONE_IBVS_SEARCH_R_GROW", "0.12"))  # m/s

AGL_MIN_M = float(os.environ.get("UDRONE_IBVS_AGL_MIN", "0.45"))
AGL_SOFT_M = float(os.environ.get("UDRONE_IBVS_AGL_SOFT", "0.90"))
SURF_MIN_M = float(os.environ.get("UDRONE_IBVS_SURF_MIN", "0.35"))
DEPTH_SHALLOW = float(os.environ.get("UDRONE_IBVS_DEPTH_SHALLOW", "-0.35"))
DEPTH_DEEP = float(os.environ.get("UDRONE_IBVS_DEPTH_DEEP", "-9.30"))
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
        search_r0_m=0.0,
        search_r_max_m=8.0,
        tag_size_m=0.4,
        detections_topic="/xudrone/detections",
        camera_info_topic="/xudrone/camera_front/camera_info",
        down_detections_topic="/xudrone/detections_down",
        down_info_topic="/xudrone/camera_down/camera_info",
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
        self.acquire_s = float(acquire_s)
        self.search_budget_s = float(
            search_budget_s if search_budget_s is not None else max(acquire_s, 45.0)
        )
        self.search_r0 = max(0.0, float(search_r0_m))
        self.search_r_max = max(self.search_r0, float(search_r_max_m))
        self.tag_size_m = float(tag_size_m)

        self.fx = self.fy = 381.46
        self.cx, self.cy = 320.0, 240.0
        # Down camera: 640 px across a 1.2 rad horizontal field.
        self.fx_d = self.fy_d = 467.8
        self.cx_d, self.cy_d = 320.0, 240.0

        self._det_u = None
        self._det_v = None
        self._det_s = None
        self._det_t = 0.0
        self._det_id = None
        self._down_u = None
        self._down_v = None
        self._down_s = None
        self._down_t = 0.0
        self.tag_source = None

        self._agl_m = None
        self._agl_t = 0.0
        self._surf_m = None
        self._surf_t = 0.0

        self.arrived = False
        self._stage = "search"
        self._seed_e = float(center_east_m) if center_east_m is not None else 5.0
        self._seed_n = float(center_north_m) if center_north_m is not None else 4.0
        self.tag_e = self._seed_e
        self.tag_n = self._seed_n
        self._search_t0 = None
        self._out_yaw = None
        self._search_r = 0.0
        self._los = None
        self._tag_xyz = None
        self._rng = None
        self._brake_t0 = None

        kwargs.setdefault("duration_s", self.timeout_s + 5.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.speed_mps)
        kwargs.setdefault("z0", self.depth_m)
        super().__init__(**kwargs)

        self.down_detections_topic = down_detections_topic
        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, down_info_topic, self._on_info_down, qos_profile_sensor_data
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
            self.create_subscription(
                AprilTagDetectionArray,
                down_detections_topic,
                self._on_down,
                qos_profile_sensor_data,
            )
        else:
            self.get_logger().error("apriltag_msgs missing — IBVS cannot run")

        self.get_logger().info(
            f"approach_tag spiral+front-lock: id={self.tag_id} "
            f"R0={self.search_r0:g} Rmax={self.search_r_max:g} "
            f"budget={self.search_budget_s:g}s seed=({self._seed_e:g},{self._seed_n:g}) "
            f"arrive={ARRIVE_RANGE_M:g} down={down_detections_topic}"
        )

    # ---- sensors --------------------------------------------------------
    def _on_info(self, m: CameraInfo):
        self._take_info(m, front=True)

    def _on_info_down(self, m: CameraInfo):
        self._take_info(m, front=False)

    def _take_info(self, m: CameraInfo, front: bool):
        k = list(m.k) if m.k is not None else []
        if len(k) < 6:
            return
        fx = float(k[0]) or (self.fx if front else self.fx_d)
        fy = float(k[4]) or (self.fy if front else self.fy_d)
        cx = float(k[2]) if k[2] else (float(m.width) * 0.5)
        cy = float(k[5]) if k[5] else (float(m.height) * 0.5)
        if front:
            self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        else:
            self.fx_d, self.fy_d, self.cx_d, self.cy_d = fx, fy, cx, cy

    @staticmethod
    def _scan_range_m(msg: LaserScan):
        if not msg.ranges:
            return None
        r = float(msg.ranges[0])
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
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

    def _fresh_range(self, value, stamp):
        if value is None or (time.time() - stamp) > RANGE_STALE_S:
            return None
        return value

    def _apply_clearance(self, pitch_u, thrust, pos, nose_down):
        """Keep off the bed and below the free surface.

        A nose that is still pointed down gets no forward thrust near the
        bed. A nose that is already up may climb off it.
        """
        _x, _y, z = pos
        agl = self._fresh_range(self._agl_m, self._agl_t)
        surf = self._fresh_range(self._surf_m, self._surf_t)
        note = None
        if z > DEPTH_SHALLOW and thrust >= 0.0:
            pitch_u = max(pitch_u, 0.12)
            thrust = min(thrust, 0.06)
            note = f"shallow z={z:.2f}"
        elif z < DEPTH_DEEP:
            pitch_u = min(pitch_u, -0.08)
            thrust = 0.0 if nose_down >= 0.0 else min(thrust, 0.12)
            note = f"deep z={z:.2f}"
        if agl is not None:
            if agl < AGL_MIN_M:
                pitch_u = min(pitch_u, -0.12)
                thrust = min(thrust, 0.08) if nose_down < 0.0 else min(thrust, 0.0)
                note = f"agl={agl:.2f}<{AGL_MIN_M:g}"
            elif agl < AGL_SOFT_M:
                # A nose that is down does not get to dive. Level flight may continue.
                if pitch_u > 0.05:
                    pitch_u = 0.0
                if nose_down > 0.25:
                    thrust = min(thrust, 0.0)
                note = note or f"agl_soft={agl:.2f}"
        if surf is not None and surf < SURF_MIN_M:
            pitch_u = max(pitch_u, 0.10)
            thrust = min(thrust, 0.05)
            note = note or f"surf={surf:.2f}"
        return pitch_u, thrust, note, agl, surf

    def _on_apriltag(self, m: AprilTagDetectionArray):
        if not m.detections:
            return
        best = None
        for d in m.detections:
            if int(d.id) == self.tag_id:
                best = d
                break
        if best is None:
            return
        side = _tag_side_px(best.corners)
        if side < 1.0:
            return
        self._det_u = float(best.centre.x)
        self._det_v = float(best.centre.y)
        self._det_s = side
        self._det_t = time.time()
        self._det_id = int(best.id)
        self.tag_source = "front_ibvs"

    def _on_down(self, m: AprilTagDetectionArray):
        if not m.detections:
            return
        best = None
        for d in m.detections:
            if int(d.id) == self.tag_id:
                best = d
                break
        if best is None:
            return
        side = _tag_side_px(best.corners)
        if side < 1.0:
            return
        self._down_u = float(best.centre.x)
        self._down_v = float(best.centre.y)
        self._down_s = side
        self._down_t = time.time()

    def _fresh_down(self) -> bool:
        return self._down_u is not None and (time.time() - self._down_t) <= LOST_S

    def _down_range(self):
        rng = self.fx_d * self.tag_size_m / max(self._down_s or 0.0, 1e-3)
        if not math.isfinite(rng) or rng < 0.3 or rng > 30.0:
            return None
        return rng

    def _down_ray(self, q):
        """World direction of the tag in the downward camera."""
        if q is None or not self._fresh_down():
            return None
        xn = (self._down_u - self.cx_d) / max(self.fx_d, 1e-3)
        yn = (self._down_v - self.cy_d) / max(self.fy_d, 1e-3)
        body = nadir_body(xn, yn, 1.0)
        fwd, left, up = body_axes(q)
        direction = tuple(
            body[0] * fwd[i] + body[1] * left[i] + body[2] * up[i] for i in range(3)
        )
        norm = math.sqrt(sum(v * v for v in direction))
        if norm < 1e-3:
            return None
        return tuple(v / norm for v in direction)

    def _aim_axis(self, pos):
        """Unit world direction from the hull to the stored tag.

        The lock ray is the fallback. The hull moves after that sighting,
        so the front camera follows the point.
        """
        tag = self._tag_xyz
        if tag is not None and pos is not None:
            delta = tuple(float(tag[i]) - float(pos[i]) for i in range(3))
            norm = math.sqrt(sum(v * v for v in delta))
            if norm > 0.05:
                return tuple(v / norm for v in delta)
        return self._los

    def _fresh_det(self) -> bool:
        return self._det_u is not None and (time.time() - self._det_t) <= LOST_S

    def _image_err(self):
        ey = (self._det_u - self.cx) / max(self.fx, 1e-3)
        ez = (self._det_v - self.cy) / max(self.fy, 1e-3)
        return ey, ez

    def _optical_range(self):
        rng = self.fx * self.tag_size_m / max(self._det_s, 1e-3)
        if not math.isfinite(rng) or rng < 0.3 or rng > 30.0:
            return None
        return rng

    def _aim_ray(self, q):
        """Unit world direction of the tag. Range cancels."""
        if q is None or not self._fresh_det():
            return None
        ey, ez = self._image_err()
        fwd, left, up = body_axes(q)
        # Image right is hull -Y, image down is hull -Z.
        body = (1.0, -ey, -ez)
        direction = tuple(
            body[0] * fwd[i] + body[1] * left[i] + body[2] * up[i] for i in range(3)
        )
        norm = math.sqrt(sum(v * v for v in direction))
        if norm < 1e-3:
            return None
        return tuple(v / norm for v in direction)

    def _refresh_tag(self, pos, q):
        rng = self._optical_range() if self._fresh_det() else None
        ray = self._aim_ray(q)
        if ray is None or rng is None or pos is None:
            return
        point, rng = self._place_tag(pos, ray, rng)
        # A flat tag seen at an angle looks small, so the range jumps long
        # and the stop point moves past the real tag. Keep the closer fix.
        if self._rng is not None and rng > self._rng * 1.35 and rng > self._rng + 0.4:
            return
        self._los = ray
        self._rng = rng
        self._tag_xyz = point
        self.tag_e, self.tag_n = point[0], point[1]

    def _tag_dist(self, pos):
        tag = self._tag_xyz
        if tag is None or pos is None:
            return None
        return math.sqrt(sum((float(pos[i]) - float(tag[i])) ** 2 for i in range(3)))

    def _arrive_px(self) -> float:
        """Pixel size of the tag at the arrive range, seen face-on."""
        return self.fx * self.tag_size_m / max(ARRIVE_RANGE_M, 0.3)

    def _near_tag(self, pos) -> bool:
        dist = self._tag_dist(pos)
        if dist is not None and dist <= ARRIVE_RANGE_M:
            return True
        return bool(self._fresh_det() and self._det_s >= self._arrive_px())

    def _surge(self, nose_down: float) -> float:
        """Forward thrust, limited so a steep nose cannot dive through the tag."""
        dive = max(0.20, math.sin(max(0.0, nose_down)))
        return min(LOS_THRUST, 0.04 / dive)

    def _place_tag(self, pos, ray, rng):
        """Back-project the tag. It sits on the bed, so the ray cannot go past it."""
        bed_z = -9.40
        if ray[2] < -0.05:
            rng_bed = (bed_z - float(pos[2])) / ray[2]
            if rng_bed > 0.3:
                rng = min(rng, rng_bed)
        return tuple(float(pos[i]) + ray[i] * rng for i in range(3)), rng

    # ---- spiral on the down camera, then front-camera lock -------------
    def _spiral_rates(self, tm, pos, q):
        """One level spiral. The down camera does the looking."""
        x, y, _z = pos
        dx, dy = x - self._seed_e, y - self._seed_n
        r = math.hypot(dx, dy)
        nose_down = rpy_from_q(q)[1]
        yaw = rpy_from_q(q)[2]
        if self._search_t0 is None:
            self._search_t0 = tm
            self._out_yaw = yaw
        grown = max(0.0, tm - self._search_t0)
        ring = min(self.search_r_max, SEARCH_R_GROW * grown)
        self._search_r = ring
        if r < 0.40:
            yaw_u = clamp(1.1 * wrap_pi(self._out_yaw - yaw), RATE_MAX)
        else:
            radial = (dx / r, dy / r)
            tangent = (-radial[1], radial[0])
            pull = clamp((ring - r) * 0.25, 0.7)
            desire = math.atan2(
                tangent[1] + pull * radial[1], tangent[0] + pull * radial[0]
            )
            yaw_u = clamp(1.1 * wrap_pi(desire - yaw), RATE_MAX)
        pitch_u = clamp(1.2 * (SEARCH_PITCH0 - nose_down), RATE_MAX)
        return pitch_u, yaw_u, SEARCH_THRUST

    def _egress_rates(self, pos, q):
        """Raise the nose and stop, then climb. No forward thrust while aimed down."""
        nose_down = rpy_from_q(q)[1]
        z = float(pos[2]) if pos is not None else EGRESS_Z
        if nose_down > 0.25:
            pitch_u = clamp(1.2 * (0.0 - nose_down), RATE_MAX)
            return pitch_u, 0.0, BRAKE_THRUST
        target = -0.35 if z < EGRESS_Z else 0.0
        pitch_u = clamp(1.2 * (target - nose_down), RATE_MAX)
        thrust = 0.10 if z < EGRESS_Z and nose_down < 0.0 else 0.0
        return pitch_u, 0.0, thrust

    def _los_rates(self, pos, q):
        """Hold the tag at the image centre and surge straight at it."""
        if self._stage == "egress":
            return self._egress_rates(pos, q)
        self._refresh_tag(pos, q)
        if self._brake_t0 is not None or self._near_tag(pos):
            nose_down = rpy_from_q(q)[1]
            pitch_u = clamp(1.2 * (0.0 - nose_down), RATE_MAX)
            return pitch_u, 0.0, BRAKE_THRUST
        nose_down = rpy_from_q(q)[1]
        if self._fresh_det():
            ey, ez = self._image_err()
            yaw_u = clamp(-LAM_YAW * ey, RATE_MAX)
            pitch_u = clamp(LAM_PITCH * ez, RATE_MAX)
            return pitch_u, yaw_u, self._surge(nose_down)
        axis = self._aim_axis(pos)
        if axis is None or q is None:
            return 0.0, 0.0, 0.0
        yaw = rpy_from_q(q)[2]
        bearing = math.atan2(axis[1], axis[0])
        horiz = math.hypot(axis[0], axis[1])
        elev = math.atan2(axis[2], max(horiz, 1e-3))
        # Point the front camera at the stored tag. No surge while that
        # line is still steep, so the handoff is not a dive.
        pitch_u = clamp(1.2 * (-elev - nose_down), RATE_MAX)
        yaw_u = clamp(1.2 * wrap_pi(bearing - yaw), RATE_MAX)
        if elev < -0.55 or nose_down > 0.70:
            return pitch_u, yaw_u, 0.0
        return pitch_u, yaw_u, min(HOLD_THRUST, self._surge(nose_down))

    def _events(self, tm: float):
        if self._stage == "done":
            return
        if self._los is None:
            if self._search_t0 is None:
                self._search_t0 = tm
            if tm - self._search_t0 >= self.search_budget_s:
                print(
                    f"FAIL lost_tag (search budget {self.search_budget_s:.0f}s exhausted)",
                    flush=True,
                )
                self._fail_and_disarm("lost_tag")
            return
        pos = self._prev[1] if self._prev else None
        q = self._prev[2] if self._prev else None
        if self._stage == "egress":
            nose = rpy_from_q(q)[1] if q is not None else 0.0
            z = float(pos[2]) if pos is not None else EGRESS_Z - 1.0
            if z >= EGRESS_Z and abs(nose) <= 0.25:
                self.arrived = True
                self._stage = "done"
                print(f"APPROACH_ARRIVED egress z={z:.2f}", flush=True)
            elif self._brake_t0 is not None and tm - self._brake_t0 >= EGRESS_S:
                print(f"APPROACH_EGRESS still z={z:.2f}", flush=True)
                self._brake_t0 = tm
            return
        if self._brake_t0 is None:
            if not self._near_tag(pos):
                return
            self._brake_t0 = tm
            dist = self._tag_dist(pos)
            span = f"{dist:.3f} m" if dist is not None else "px"
            print(f"APPROACH_BRAKE range={span}", flush=True)
            return
        if tm - self._brake_t0 >= BRAKE_S:
            nose = rpy_from_q(q)[1] if q is not None else 0.0
            z = float(pos[2]) if pos is not None else EGRESS_Z - 1.0
            if z >= EGRESS_Z and abs(nose) <= 0.25:
                self.arrived = True
                self._stage = "done"
                dist = self._tag_dist(pos)
                span = f"{dist:.3f} m" if dist is not None else "px"
                print(f"APPROACH_ARRIVED range={span}", flush=True)
            else:
                self._stage = "egress"
                print(f"APPROACH_EGRESS z={z:.2f}", flush=True)

    def _fail_and_disarm(self, why: str):
        self.failed = why
        try:
            self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
            self._stop_vehicle()
            self._drop_baton()
        except Exception:
            pass
        self.done = True

    def _helix(self, tm: float):
        z = self.depth_m
        if self._prev is not None:
            z = self._prev[1][2]
        return (0.0, 0.0, z), (0.0, 0.0, 0.0)

    def _hud_layers(self):
        lines = [f"tag {self.tag_id}  {self._stage}  R {self._search_r:.1f} m"]
        if self.tag_e is not None and self.tag_n is not None:
            lines.append(f"lock ENU ({self.tag_e:.1f}, {self.tag_n:.1f})")
        return {"camera": "down" if self._los is None else "front",
                "stage": self._stage, "lines": lines, "markers": [], "paths": []}

    def _control(self, tm, auv, boat):
        pos, q = auv
        sim_t = self.t_thrust0 + tm
        if self._prev is None:
            self._prev = (sim_t, pos, q)
        elif sim_t > self._prev[0] + 1e-3:
            self._prev = (sim_t, pos, q)

        if self._los is None and self._fresh_down():
            ray = self._down_ray(q)
            rng = self._down_range()
            if ray is not None and rng is not None:
                self._los = ray
                self._stage = "los"
                self.tag_source = "down"
                point, rng = self._place_tag(pos, ray, rng)
                self._rng = rng
                self._tag_xyz = point
                self.tag_e, self.tag_n = point[0], point[1]
                print(
                    f"LOS_LOCK down axis=({ray[0]:+.2f},{ray[1]:+.2f},{ray[2]:+.2f}) "
                    f"range={rng:.1f}",
                    flush=True,
                )

        thrust = pitch_u = yaw_u = 0.0
        if self.latch.state == "attached" or self._stage == "done":
            pass
        elif self._los is None:
            pitch_u, yaw_u, thrust = self._spiral_rates(tm, pos, q)
            if tm >= self.next_print:
                self.next_print = tm + 2.0
                print(
                    f"t={tm:5.1f} SPIRAL R={self._search_r:.1f} "
                    f"yaw_u={yaw_u:+.2f} pitch_u={pitch_u:+.2f} thrust={thrust:+.2f}",
                    flush=True,
                )
        else:
            pitch_u, yaw_u, thrust = self._los_rates(pos, q)
            if tm >= self.next_print:
                self.next_print = tm + 1.0
                if self._fresh_det():
                    ey, ez = self._image_err()
                    print(
                        f"t={tm:5.1f} LOS ey={ey:+.3f} ez={ez:+.3f} "
                        f"s={self._det_s:.0f}px yaw_u={yaw_u:+.2f} "
                        f"pitch_u={pitch_u:+.2f} thrust={thrust:+.2f}",
                        flush=True,
                    )
                else:
                    print(
                        f"t={tm:5.1f} LOS hold yaw_u={yaw_u:+.2f} "
                        f"pitch_u={pitch_u:+.2f} thrust={thrust:+.2f}",
                        flush=True,
                    )

        if self.latch.state != "attached" and self._stage not in ("done", "egress"):
            nose_down = rpy_from_q(q)[1]
            pitch_u, thrust, clr, agl, surf = self._apply_clearance(
                pitch_u, thrust, pos, nose_down
            )
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
    search_r0 = float(os.environ.get("UDRONE_APPROACH_SEARCH_R0", "0.0"))
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
