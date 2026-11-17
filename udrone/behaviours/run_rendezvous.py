#!/usr/bin/env python3
"""Search for the tag under otter_k, line up on the nest axis, and dock.

Geometry (see rendezvous_geom): nest at the Heron origin (between the
pontoons); approach along boat -Z (normal to the deck); hull attitude
parallel to the boat for the weld.

The spiral is centred on the nest. The upward camera (AprilTag 36h11 id 10)
finds the deck; the servo setpoint is then pulled onto the nest axis so the
hull stays between the pontoons. A lost tag holds the last fix and steps
under it before the spiral starts again. Inside the acceptance cone the
hull climbs the nest axis level, and ATTACH is sent (and retried) in the
weld gate. Leaving the cone hands back to the servo.
Phrase: ``xudrone, rendezvous and dock with otter_k``.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

from patterns import world_offset_to_path
from servo_geom import phase, relock_leg
import maneuver_power as mp
from rendezvous_geom import (
    CONE_EXIT,
    CONE_HALF,
    CONE_RANGE_M,
    DOCK_CLIMB_MPS,
    DOCK_STANDOFF_M,
    LAND_RANGE_M,
    LEARNED_BLEND,
    LEARNED_CHASE_MPS,
    LEARNED_CLOSE_RATE,
    closing_range,
    cone_rim,
    cone_state,
    dock_goal,
    nest_world,
    search_offset,
    servo_station,
    under_nest,
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
WELD_HOLD_S = 0.4    # inside the weld gate this long before ATTACH
ATTACH_RETRY_S = 1.5
DOCK_PITCH_RAD = math.radians(15.0)
TRANSIT_M = 5.0      # farther than this from the boat, swim straight under it first
DOCK_TOPIC = "/udrone_dock/command"


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
        self.tag_id = int(float(os.environ.get("UDRONE_RENDEZVOUS_TAG_ID", TAG_ID)))
        self._power = mp.from_env()
        # PPO residual defaults (env overrides for A/B). Still scaled by power.
        chase0 = float(os.environ.get("UDRONE_RDV_CHASE", LEARNED_CHASE_MPS))
        climb0 = float(os.environ.get("UDRONE_RDV_CLIMB", DOCK_CLIMB_MPS))
        close0 = float(os.environ.get("UDRONE_RDV_CLOSE_RATE", LEARNED_CLOSE_RATE))
        blend0 = float(os.environ.get("UDRONE_RDV_BLEND", LEARNED_BLEND))
        pace = max(0.08, mp.scale(chase0, self._power))
        kwargs.setdefault("duration_s", self.budget_s / max(0.35, self._power) + 30.0)
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
        self._stage = "search"
        self.get_logger().info(
            f"rendezvous power={100.0 * self._power:.0f}% "
            f"chase={pace:.2f} climb={self._climb_mps:.2f} blend0={self._blend0:.2f}"
        )
        self._search_t0 = None
        self._des = None
        self._vel = (0.0, 0.0, 0.0)
        self._boat_vel = BaselineVelocity()
        self._auv_pos = None
        self._transit = False
        self._det = None
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
        self._log_t = 0.0
        self.hud_preview = False
        info = os.environ.get("UDRONE_RENDEZVOUS_INFO", "/xudrone/camera_up/camera_info")
        dets = os.environ.get("UDRONE_RENDEZVOUS_DET", "/xudrone/detections_up")
        self.create_subscription(CameraInfo, info, self._on_info, qos_profile_sensor_data)
        if AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray, dets, self._on_tag, qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"rendezvous otter_k: tag={self.tag_id} budget={self.budget_s:g}s topic={dets}"
        )

    def _on_info(self, m: CameraInfo):
        k = list(m.k)
        if len(k) >= 6 and k[0] > 1.0 and k[4] > 1.0:
            self.fx, self.fy = float(k[0]), float(k[4])
            self.cx = float(k[2]) or self.cx
            self.cy = float(k[5]) or self.cy

    def _on_tag(self, m):
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

    def _drop_to_search(self, tm, boat):
        self._stage = "search"
        self._search_t0 = tm
        self._lock_range = None
        self._lock_tm = None
        print("RENDEZVOUS_LOST back to search", flush=True)
        return self._search_goal(tm, boat)

    def _search_goal(self, tm, boat):
        # Nest axis (between pontoons), not the tag — search must already be
        # geometry-aware so the spiral ends under the weld.
        nest = nest_world(boat[0], boat[1])
        station = under_nest(boat[0], boat[1])
        z = min(-0.55, max(-1.60, float(station[2])))
        on_axis = (float(station[0]), float(station[1]), z)
        auv = self._auv_pos
        far = auv is not None and math.hypot(auv[0] - nest[0], auv[1] - nest[1]) > TRANSIT_M
        if far:
            if not self._transit:
                self._transit = True
                print("RENDEZVOUS_TRANSIT straight under nest before the search", flush=True)
            self._search_t0 = None
            return on_axis, (0.0, 0.0, 0.0)
        self._transit = False
        if self._search_t0 is None:
            self._search_t0 = tm
        de, dn, _r = search_offset(
            tm - self._search_t0, r_rate=self._spiral_grow, omega=self._spiral_omega,
        )
        return (float(nest[0]) + de, float(nest[1]) + dn, z), (0.0, 0.0, 0.0)

    def _servo_goal(self, tm, auv, boat):
        pos, q = auv
        tag = self._visual_tag(pos, q)
        if tag is None or boat is None:
            return None
        if self._lock_range is None:
            self._lock_range = self._det[2]
            self._lock_tm = tm
            self._stage = "servo"
            print(
                f"RENDEZVOUS_LOCK range={self._lock_range:.2f} "
                f"level=({self._det[0]:+.2f},{self._det[1]:+.2f})",
                flush=True,
            )
        optical = closing_range(
            self._lock_range, tm - self._lock_tm, LAND_RANGE_M, rate=self._close_rate,
        )
        # Tag finds the deck; nest axis keeps the hull between the pontoons.
        # PPO preferred ~0.72 nest pull early; rise to 1.0 as optical range closes.
        # Heading alignment is yaw_hold in _control — do not lead off the nest axis.
        closed = max(0.0, self._lock_range - optical)
        span = max(0.5, self._lock_range - LAND_RANGE_M)
        blend = min(1.0, self._blend0 + (1.0 - self._blend0) * min(1.0, closed / span))
        station = servo_station(tag, optical, boat[0], boat[1], CAM_Z, blend=blend)
        return station, (0.0, 0.0, 0.0)

    def _dock_goal(self, tm, cone):
        """Climb the nest axis (boat -Z); ATTACH in the weld gate, plates parallel."""
        if self._stage != "dock":
            self._stage = "dock"
            self._dock_along = max(DOCK_STANDOFF_M, cone.get("along", cone["range"]))
            self._dock_tm = tm
            self._weld_tm = None
            print(
                f"RENDEZVOUS_CONE in range={cone['range']:.2f} "
                f"along={cone.get('along', 0):.2f} lateral={cone.get('lateral', 0):.2f} "
                f"off={math.degrees(cone['off_axis']):.0f}deg "
                f"yaw={math.degrees(cone['yaw']):+.0f}deg "
                f"tilt={math.degrees(cone['tilt']):.0f}deg",
                flush=True,
            )
        point, self._dock_along = dock_goal(
            cone, self._dock_along, tm - self._dock_tm, climb_mps=self._climb_mps,
        )
        self._dock_tm = tm
        if cone["weld"]:
            if self._weld_tm is None:
                self._weld_tm = tm
            elif tm - self._weld_tm >= WELD_HOLD_S:
                self._request_attach(cone)
        else:
            self._weld_tm = None
        return point, (0.0, 0.0, 0.0)

    def _leave_dock(self, cone):
        print(
            f"RENDEZVOUS_CONE out range={cone['range']:.2f} "
            f"off={math.degrees(cone['off_axis']):.0f}deg "
            f"yaw={math.degrees(cone['yaw']):+.0f}deg",
            flush=True,
        )
        self._stage = "servo" if self._fresh() else "relock"
        self._lock_range = None
        self._weld_tm = None

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
        """Search, servo under the tag, or relock on the last fix."""
        seen = self._fresh()
        if self._stage == "search":
            goal = None
            if self._hits >= FOUND_HITS and seen:
                goal = self._servo_goal(tm, auv, boat)
            return goal or self._search_goal(tm, boat)
        if seen:
            if self._stage == "relock":
                self._stage = "servo"
            return self._servo_goal(tm, auv, boat)
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
        docking = self._stage == "dock"
        cone = cone_state(auv[0], auv[1], boat[0], boat[1], CONE_EXIT if docking else 1.0)
        self._cone = cone
        if cone["inside"]:
            goal = self._dock_goal(tm, cone)
        else:
            if docking:
                self._leave_dock(cone)
            goal = self._approach_goal(tm, auv, boat)
        if goal is None:
            return
        station, _vel = goal
        self._des = station
        # The goal rides on the boat. Its jumps (search steps, a relock, a
        # new tag fix) are position steps for the loop, not velocity.
        self._vel = self._boat_vel.update(tm, boat[0][0], boat[0][1])
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
        out["camera"] = "up"
        out["stage"] = self._stage
        out["lines"].append(f"tag {self.tag_id} hits {self._hits}  attach tries {self._attach_n}")
        if self._des is not None:
            out["markers"].append({"pos": self._des, "label": "goal", "color": (0, 200, 255)})
        if self._lock_enu is not None:
            out["markers"].append({"pos": self._lock_enu, "label": "tag fix", "color": (255, 200, 0),
                                   "shape": "diamond"})
        cone = self._cone
        if cone is not None:
            ok = (0, 220, 0) if cone["inside"] else (0, 200, 255)
            if cone["weld"]:
                ok = (255, 255, 255)
            rim = cone_rim(cone["nest"], cone["down"])
            out["paths"].append({"pts": rim, "color": ok, "width": 1})
            for k in (0, 6, 12, 18):
                out["paths"].append({"pts": [cone["nest"], rim[k]], "color": ok, "width": 1})
            out["markers"].append({"pos": cone["nest"], "label": "nest", "color": ok, "shape": "circle"})
            out["cone"] = {
                "center": cone["nest"], "radius": CONE_RANGE_M * math.sin(CONE_HALF),
                "inside": cone["inside"],
            }
            state = "WELD GATE" if cone["weld"] else ("IN CONE" if cone["inside"] else "outside cone")
            out["lines"].append(
                f"{state}  range {cone['range']:.2f} m  lateral {cone['lateral']:.2f} m  "
                f"off {math.degrees(cone['off_axis']):.0f}deg  "
                f"yaw {math.degrees(cone['yaw']):+.0f}deg  tilt {math.degrees(cone['tilt']):.0f}deg"
            )
        return out

    def _control(self, tm, auv, boat):
        self._update(tm, auv, boat)
        self.yaw_hold = None
        pos = auv[0]
        if self._stage == "dock" and boat is not None:
            # Climb level with the nose on the boat's, so the weld's attitude
            # gate holds when the hull reaches the nest.
            self.yaw_hold = rpy_from_q(boat[1])[2]
            self.pitch_limit = DOCK_PITCH_RAD
        elif self._des is not None and boat is not None:
            dist = math.hypot(self._des[0] - pos[0], self._des[1] - pos[1])
            # Close in, the nose keeps the boat's heading (also lined up for
            # the latch) and small errors are closed with forward or reverse
            # thrust. Farther out the force loop turns toward the goal.
            self.yaw_hold = station_heading(dist, rpy_from_q(boat[1])[2])
            self.pitch_limit = STATION_PITCH_RAD if self.yaw_hold is not None else PITCH_MAX_RAD
        return UdroneHelixController._control(self, tm, auv, boat)

    def _helix(self, tm: float):
        if self._des is None or self.anchor is None:
            z = self._prev[1][2] if self._prev else -1.0
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(self._des[0] - x0, self._des[1] - y0, yaw0)
        c, s = math.cos(yaw0), math.sin(yaw0)
        vx, vy, vz = self._vel
        return (fwd, left, self._des[2]), (c * vx + s * vy, -s * vx + c * vy, vz)

    def _finished(self, tm: float) -> bool:
        if self._landed or self.failed:
            return True
        if tm >= self.budget_s:
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
