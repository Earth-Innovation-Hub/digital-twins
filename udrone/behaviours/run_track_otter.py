#!/usr/bin/env python3
"""Hold xudrone under otter_k, using the tag on the underside of the deck.

The boat pose predicts where the station point is. The upward camera's
AprilTag (36h11 id 10) corrects that point when it is in view. When that
tag goes stale and the boat pose disagrees with the last visual fix, one
relock slides back under the fix before the prediction takes over again.
The force loop tracks the point and is given the boat's horizontal velocity
so the AUV keeps up. Coral command type ``track``. Rejected while the latch is
attached. Phrase: ``xudrone: track otter_k``.
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
from sensor_msgs.msg import CameraInfo

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

from patterns import world_offset_to_path
from servo_geom import phase, prediction_disagrees
from track_geom import (
    CAM_Z,
    STANDOFF_M,
    STATION_PITCH_RAD,
    TAG_ID,
    TAG_Z_BOAT,
    BaselineVelocity,
    clamp_depth,
    station_heading,
    station_under,
    up_camera_tag_body,
)
from udrone_sinusoidal_controller import (
    PITCH_MAX_RAD,
    UdroneHelixController,
    body_axes,
    rpy_from_q,
    spin_until_done,
)

DURATION_S = 90.0
ACQUIRE_S = 40.0
HOLD_S = 5.0
HOLD_H_M = 0.80
LOST_S = 1.5


def _tag_side_px(corners) -> float:
    if not corners or len(corners) < 4:
        return 0.0
    pts = [(float(c.x), float(c.y)) for c in corners]
    sides = [
        math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
        for i in range(4)
    ]
    return sum(sides) / 4.0


class UdroneTrackOtter(UdroneHelixController):
    def __init__(self, **kwargs):
        self.duration_s = float(os.environ.get("UDRONE_TRACK_DURATION", DURATION_S))
        self.standoff = float(os.environ.get("UDRONE_TRACK_STANDOFF", STANDOFF_M))
        self.tag_id = int(float(os.environ.get("UDRONE_TRACK_TAG_ID", TAG_ID)))
        kwargs.setdefault("duration_s", self.duration_s + 20.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", 0.25)
        kwargs.setdefault("z0", -0.8)
        super().__init__(**kwargs)
        self.fx = self.fy = 380.0
        self.cx, self.cy = 320.0, 240.0
        self._des = None
        self._vel = (0.0, 0.0, 0.0)
        self._boat_vel = BaselineVelocity()
        self._det = None
        self._lock_enu = None
        self._lock_t = 0.0
        self._relocking = False
        self._saw = False
        self._hold = 0.0
        self._hold_tm = None
        self._log_t = 0.0
        self._src = "boat"
        self.hud_preview = False
        info = os.environ.get("UDRONE_TRACK_INFO_TOPIC", "/xudrone/camera_up/camera_info")
        dets = os.environ.get("UDRONE_TRACK_DET_TOPIC", "/xudrone/detections_up")
        self.create_subscription(CameraInfo, info, self._on_info, qos_profile_sensor_data)
        if AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray, dets, self._on_tag, qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"track otter_k: tag={self.tag_id} standoff={self.standoff:g} m "
            f"duration={self.duration_s:g}s topic={dets}"
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
            if int(d.id) != self.tag_id:
                continue
            best = d
            break
        if best is None:
            return
        side = _tag_side_px(best.corners)
        if side < 4.0 or self.fx <= 1.0:
            return
        x_n = (float(best.centre.x) - self.cx) / self.fx
        y_n = (float(best.centre.y) - self.cy) / self.fy
        z_opt = self.fx * 0.40 / side
        if not math.isfinite(z_opt) or z_opt < 0.2 or z_opt > 8.0:
            return
        self._det = (x_n, y_n, z_opt, time.time())
        if not self._saw:
            self._saw = True
            print(
                f"TRACK_LOCK range={z_opt:.2f} level=({x_n:+.2f},{y_n:+.2f})",
                flush=True,
            )

    def _events(self, tm: float):
        if self.latch.state == "attached":
            self._fail("track rejected: xudrone is latched; detach first")

    def _boat_tag(self, boat):
        if boat is None:
            return None
        pos, q = boat
        _fwd, _left, up = body_axes(q)
        return tuple(float(pos[i]) + TAG_Z_BOAT * up[i] for i in range(3))

    def _visual_tag(self, pos, q):
        if self._det is None or time.time() - self._det[3] > LOST_S:
            return None
        x_n, y_n, z_opt, _t = self._det
        bx, by, bz = up_camera_tag_body(x_n, y_n, z_opt, CAM_Z)
        fwd, left, up = body_axes(q)
        return tuple(
            float(pos[i]) + bx * fwd[i] + by * left[i] + bz * up[i]
            for i in range(3)
        )

    def _update_target(self, tm, auv, boat):
        pos, q = auv
        visual = self._visual_tag(pos, q)
        boat_tag = self._boat_tag(boat)
        self._relocking = False
        if visual is not None:
            self._lock_enu = visual
            self._lock_t = time.time()
            tag = visual
        elif (
            self._lock_enu is not None
            and boat_tag is not None
            and phase(time.time() - self._lock_t) == "relock"
            and prediction_disagrees(self._lock_enu, boat_tag)
        ):
            tag = self._lock_enu
            self._relocking = True
        else:
            tag = boat_tag
        if tag is None:
            return
        station = station_under(tag, self.standoff, CAM_Z)
        self._des = (station[0], station[1], clamp_depth(station[2]))
        # The station moves with the boat. Its jumps (a new tag fix, a relock)
        # are position steps for the loop, not velocity.
        self._vel = self._boat_vel.update(tm, boat[0][0], boat[0][1])
        horiz = math.hypot(pos[0] - float(boat[0][0]), pos[1] - float(boat[0][1])) if boat else 99.0
        fresh = self._det is not None and time.time() - self._det[3] <= LOST_S
        if fresh and horiz <= HOLD_H_M and abs(pos[2] - self._des[2]) <= 0.45:
            if self._hold_tm is None:
                self._hold_tm = tm
            else:
                self._hold += max(0.0, tm - self._hold_tm)
                self._hold_tm = tm
        else:
            self._hold_tm = tm
        self._src = src = "tag" if fresh else ("relock" if self._relocking else "boat")
        if time.time() >= self._log_t:
            self._log_t = time.time() + 2.0
            print(
                f"TRACK {src} des=({self._des[0]:.1f},{self._des[1]:.1f},{self._des[2]:.2f}) "
                f"boat_h={horiz:.2f} hold={self._hold:.1f}s",
                flush=True,
            )

    def _hud_layers(self):
        out = UdroneHelixController._hud_layers(self)
        out["camera"] = "up"
        out["stage"] = f"station from {self._src}"
        out["lines"].append(
            f"tag {self.tag_id}  standoff {self.standoff:.2f} m  held {self._hold:.1f}/{HOLD_S:.0f}s"
        )
        if self._des is not None:
            out["markers"].append({"pos": self._des, "label": "station", "color": (0, 200, 255)})
        if self._lock_enu is not None:
            out["markers"].append({"pos": self._lock_enu, "label": "tag fix", "color": (255, 200, 0),
                                   "shape": "diamond"})
        return out

    def _control(self, tm, auv, boat):
        if boat is not None:
            self._update_target(tm, auv, boat)
        self.yaw_hold = None
        if self._des is not None and boat is not None:
            pos = auv[0]
            dist = math.hypot(self._des[0] - pos[0], self._des[1] - pos[1])
            # Farther out the force loop turns the nose toward the station.
            # Close in it keeps the boat's heading and closes small errors
            # with forward or reverse thrust, so it does not turn round on
            # every overshoot.
            self.yaw_hold = station_heading(dist, rpy_from_q(boat[1])[2])
            self.pitch_limit = STATION_PITCH_RAD if self.yaw_hold is not None else PITCH_MAX_RAD
        return UdroneHelixController._control(self, tm, auv, boat)

    def _helix(self, tm: float):
        if self._des is None or self.anchor is None:
            z = self._prev[1][2] if self._prev else -0.8
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        x0, y0, yaw0 = self.anchor
        fwd, left = world_offset_to_path(self._des[0] - x0, self._des[1] - y0, yaw0)
        c, s = math.cos(yaw0), math.sin(yaw0)
        vx, vy, vz = self._vel
        # Path x is along the anchor heading, path y is to the left.
        v_fwd = c * vx + s * vy
        v_left = -s * vx + c * vy
        return (fwd, left, self._des[2]), (v_fwd, v_left, vz)

    def _finished(self, tm: float) -> bool:
        if self.failed:
            return True
        if not self._saw and tm >= ACQUIRE_S:
            self._fail(f"did not see tag {self.tag_id} under otter_k in {ACQUIRE_S:.0f}s")
            return True
        if tm >= self.duration_s:
            if self._hold < HOLD_S:
                self._fail(
                    f"saw the boat tag but held station for {self._hold:.1f}s "
                    f"(need {HOLD_S:.0f}s)"
                )
            else:
                print(f"TRACK_DONE hold={self._hold:.1f}s", flush=True)
            return True
        return False


def main():
    rclpy.init()
    node = UdroneTrackOtter()
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_track_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed and node._hold >= HOLD_S:
            ok = True
            why = f"ok under otter_k hold={node._hold:.1f}s"
        elif not node.failed:
            why = "did not hold station under otter_k"
        print(f"TRACK_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
