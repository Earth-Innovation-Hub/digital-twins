"""Fiducial detections -> supervisor detections, inside the twin.

apriltag_ros (apriltag_node) looks at the gimbal camera (/drone_camera) and
broadcasts one TF per visible tag, camera optical frame -> "tag36h11:<id>".
This node turns those into the supervisor's detection records:

  in   /tf                              apriltag_node's camera -> tag transforms
  in   /detections                      apriltag_msgs/AprilTagDetectionArray (decision margin)
  in   /fmu/out/vehicle_local_position  vehicle NED position + heading
  out  <supervisor>/detections          [{"label", "n", "e", "conf", "id", "alt_m"}]
  out  <supervisor>/tracks              same shape, tracks_hz while a tag is in view
  out  /model/<model>/command/gimbal_pitch   keeps the camera at nadir

One detection record per sighting: samples are collected for settle_sec after
a tag first appears (or until it disappears), their median is published once,
then that id is muted until it has been out of view for reset_sec. So an
overflight yields one detection, not 28 a second, and the supervisor's
bookkeeping stays sane. The first goto of a new mission (cycle 1 on
<supervisor>/command) clears the mutes. ~/tracks is the live estimate (median of the last few
samples) for closed-loop use such as re-centring during hover_above.

Geometry assumes the camera is at nadir. Measured on the PX4 x500_gimbal model
(2026-10-11, vehicle 5 m east of the tag, heading east: tag at image y = +5.2 m):
image-up is body-forward, image-right is body-right, so camera_yaw_deg = 0.

Gimbal: the Gazebo JointPositionController takes the latest command on the
gz topic. PX4's own gimbal module (MNT_MODE_OUT 2) publishes the QGC setpoint
there at ~5 Hz, so we publish at 20 Hz and win; set gimbal_pitch_rad to 0 to
leave the gimbal to QGC. Because this node owns the gimbal, the supervisor's
{"type": "camera", "pitch_deg"} on <supervisor>/command re-points it here
(-90 = nadir, 0 = horizontal); while the camera is more than
nadir_tolerance_deg off nadir the NED geometry above does not hold, so
sightings are not converted (the apriltag TFs still flow for anyone else).
"""

from __future__ import annotations

import json
import math
import os
import statistics
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleLocalPosition

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:          # tests / hosts without apriltag_msgs
    AprilTagDetectionArray = None

PX4_SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)


def tag_to_ned(tx: float, ty: float, veh_n: float, veh_e: float, heading: float,
               camera_yaw_deg: float = 0.0, mirror: bool = False) -> tuple:
    """Tag offset in the camera image plane (metres right, metres down) -> NED.

    Nadir camera: image "up" points camera_yaw_deg clockwise from body forward.
    """
    if mirror:
        tx = -tx
    th = math.radians(camera_yaw_deg)
    up_f, up_r = math.cos(th), math.sin(th)        # image-up axis in (forward, right)
    rt_f, rt_r = -up_r, up_f                       # image-right axis: up rotated 90 deg cw
    f = -ty * up_f + tx * rt_f
    r = -ty * up_r + tx * rt_r
    n = veh_n + f * math.cos(heading) - r * math.sin(heading)
    e = veh_e + f * math.sin(heading) + r * math.cos(heading)
    return n, e


class TagDetector(Node):
    def __init__(self) -> None:
        super().__init__("tag_detector")
        p = self.declare_parameter
        p("supervisor", "mission_supervisor")
        p("tag_prefix", "tag36h11:")
        p("label", "apriltag")
        p("camera_yaw_deg", 0.0)
        p("mirror", False)
        p("settle_sec", 1.0)          # collect samples this long after first sight
        p("gap_sec", 0.5)             # no sample for this long = out of view
        p("reset_sec", 5.0)           # out of view this long before the same id counts again
        p("min_samples", 3)
        p("max_range_m", 80.0)
        p("gimbal_pitch_topic", "gimbal/pitch")
        p("gimbal_pitch_rad", -1.5708)    # nadir; 0 = do not touch the gimbal
        p("gimbal_hz", 20.0)
        p("nadir_tolerance_deg", 5.0)     # convert sightings only this close to straight down
        p("tracks_hz", 2.0)               # live estimate while in view; 0 disables
        p("track_window", 8)              # samples in the running median
        p("gz_model", "")                 # if set, fall back to Gazebo ENU when PX4 pos is stale
        self._pitch_rad = float(self._p("gimbal_pitch_rad"))   # current gimbal command

        sup = self._p("supervisor").rstrip("/")
        self._det_pub = self.create_publisher(String, f"{sup}/detections", 10)
        self._track_pub = self.create_publisher(String, f"{sup}/tracks", 10)
        self.create_subscription(String, f"{sup}/command", self._on_command, 10)
        if float(self._p("tracks_hz")) > 0:
            self.create_timer(1.0 / float(self._p("tracks_hz")), self._publish_tracks)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        if AprilTagDetectionArray is not None:
            self.create_subscription(AprilTagDetectionArray, "detections", self._on_detections, 10)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position",
                                 self._on_pos, PX4_SUB_QOS)
        self._gimbal_pub = None
        if float(self._p("gimbal_pitch_rad")) != 0.0:
            self._gimbal_pub = self.create_publisher(Float64, self._p("gimbal_pitch_topic"), 10)
            self.create_timer(1.0 / float(self._p("gimbal_hz")), self._point_gimbal)

        self._pos = None                 # (n, e, d, heading)
        self._pos_wall = 0.0
        self._gz = None
        self._gz_model = str(self._p("gz_model") or "").strip()
        if self._gz_model:
            try:
                sys_path_src = os.environ.get("UDRONE_SRC", "/root/udrone_boat/src")
                if sys_path_src not in sys.path:
                    sys.path.insert(0, sys_path_src)
                from udrone_sinusoidal_controller import GzPoses, rpy_from_q  # noqa: WPS433
                self._gz = GzPoses()
                self._rpy_from_q = rpy_from_q
                self.create_timer(0.1, self._poll_gz_pose)
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warning(f"gz pose fallback unavailable: {ex}")
                self._gz = None
        self._margin = {}                # id -> latest decision margin
        self._sightings = {}             # id -> {"t0", "t_last", "samples": [(n, e, alt)], "published"}
        self._recent = {}                # id -> deque of the latest (n, e, alt), for ~/tracks
        self.create_timer(0.2, self._flush)
        self.published = 0
        extra = f" gz_model={self._gz_model}" if self._gz_model else ""
        self.get_logger().info(f"tag_detector: {self._p('tag_prefix')}* -> {sup}/detections{extra}")

    def _p(self, name):
        return self.get_parameter(name).value

    # -------------------------------------------------------------- inputs
    def _on_pos(self, m: VehicleLocalPosition) -> None:
        if m.xy_valid and m.z_valid:
            self._pos = (m.x, m.y, m.z, m.heading)
            self._pos_wall = time.monotonic()

    def _poll_gz_pose(self) -> None:
        """NED pose from Gazebo ENU when PX4 local position is missing/stale (typical for AUVs)."""
        if self._gz is None or not self._gz_model:
            return
        if self._pos is not None and (time.monotonic() - self._pos_wall) < 1.0:
            return
        fresh, _, poses = self._gz.read(self._gz_model)
        pose = poses[0] if poses else None
        if not (fresh and pose is not None):
            return
        (x, y, z), q = pose
        yaw = self._rpy_from_q(q)[2]
        # Gazebo ENU (x=east, y=north, z=up) -> NED (n, e, d).
        self._pos = (y, x, -z, yaw)
        self._pos_wall = time.monotonic()

    def _on_command(self, m: String) -> None:
        """A new mission (first goto, cycle 1) forgets earlier sightings, so a tag
        already in view is reported to the new mission instead of staying muted."""
        try:
            cmd = json.loads(m.data)
        except ValueError:
            return
        if cmd.get("type") == "goto" and cmd.get("cycle") == 1 and not cmd.get("repeat") and self._sightings:
            self._sightings.clear()
            self.get_logger().info("new mission: cleared earlier sightings")
        elif cmd.get("type") == "camera":
            self._set_camera(cmd.get("pitch_deg"))

    def _set_camera(self, pitch_deg) -> None:
        """Operator camera pointing; clamped to the gimbal's range."""
        if self._gimbal_pub is None:
            self.get_logger().warning("camera command ignored: gimbal left to QGC (gimbal_pitch_rad 0)")
            return
        try:
            pitch = min(45.0, max(-90.0, float(pitch_deg)))
        except (TypeError, ValueError):
            self.get_logger().warning(f"camera command without a numeric pitch_deg: {pitch_deg!r}")
            return
        self._pitch_rad = math.radians(pitch)
        self._point_gimbal()
        self.get_logger().info(f"camera pitch -> {pitch:g} deg"
                               + ("" if self.at_nadir() else " (off nadir: tag geometry paused)"))

    def at_nadir(self) -> bool:
        return abs(math.degrees(self._pitch_rad) + 90.0) <= float(self._p("nadir_tolerance_deg"))

    def _on_detections(self, m) -> None:
        for d in m.detections:
            self._margin[int(d.id)] = float(d.decision_margin)

    def _on_tf(self, m: TFMessage) -> None:
        if self._gimbal_pub is not None and not self.at_nadir():
            return                           # camera pointed elsewhere: geometry invalid
        prefix = self._p("tag_prefix")
        now = time.monotonic()
        for tr in m.transforms:
            if not tr.child_frame_id.startswith(prefix):
                continue
            try:
                tid = int(tr.child_frame_id[len(prefix):])
            except ValueError:
                continue
            t = tr.transform.translation
            if self._pos is None or t.z <= 0.0 or t.z > float(self._p("max_range_m")):
                continue
            vn, ve, vd, hdg = self._pos
            n, e = tag_to_ned(t.x, t.y, vn, ve, hdg, float(self._p("camera_yaw_deg")), bool(self._p("mirror")))
            alt = -vd - t.z                      # tag height above home
            self._recent.setdefault(tid, deque(maxlen=int(self._p("track_window")))).append((n, e, alt))
            s = self._sightings.get(tid)
            if s is None or (s["published"] and now - s["t_last"] > float(self._p("reset_sec"))):
                s = {"t0": now, "t_last": now, "samples": [], "published": False}
                self._sightings[tid] = s
            s["t_last"] = now
            if not s["published"]:
                s["samples"].append((n, e, alt))

    # -------------------------------------------------------------- output
    def _flush(self) -> None:
        now = time.monotonic()
        for tid, s in self._sightings.items():
            if s["published"] or len(s["samples"]) < int(self._p("min_samples")):
                continue
            settled = now - s["t0"] >= float(self._p("settle_sec"))
            gone = now - s["t_last"] >= float(self._p("gap_sec"))
            if not (settled or gone):
                continue
            k = len(s["samples"])
            n = statistics.median(x[0] for x in s["samples"])
            e = statistics.median(x[1] for x in s["samples"])
            alt = statistics.median(x[2] for x in s["samples"])
            margin = self._margin.get(tid, 0.0)
            conf = round(min(1.0, 0.5 + margin / 100.0), 2) if margin else 0.8
            rec = {"label": f"{self._p('label')} {tid}", "n": round(n, 1), "e": round(e, 1),
                   "conf": conf, "id": tid, "alt_m": round(alt, 1), "samples": k}
            self._det_pub.publish(String(data=json.dumps([rec])))
            s["published"] = True
            s["samples"] = []
            self.published += 1
            self.get_logger().info(f"tag {tid} at n {n:.1f} e {e:.1f} (alt {alt:.1f} m, {k} samples, margin {margin:.0f})")

    def _publish_tracks(self) -> None:
        now = time.monotonic()
        out = []
        for tid, s in self._sightings.items():
            if now - s["t_last"] > float(self._p("gap_sec")):
                continue                     # not in view right now
            q = self._recent.get(tid)
            if not q:
                continue
            margin = self._margin.get(tid, 0.0)
            out.append({"label": f"{self._p('label')} {tid}", "id": tid,
                        "n": round(statistics.median(x[0] for x in q), 2),
                        "e": round(statistics.median(x[1] for x in q), 2),
                        "alt_m": round(statistics.median(x[2] for x in q), 1),
                        "conf": round(min(1.0, 0.5 + margin / 100.0), 2) if margin else 0.8,
                        "samples": len(q)})
        if out:
            self._track_pub.publish(String(data=json.dumps(out)))

    def _point_gimbal(self) -> None:
        self._gimbal_pub.publish(Float64(data=self._pitch_rad))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
