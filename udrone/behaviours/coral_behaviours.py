#!/usr/bin/env python3
"""CoRAL behaviour library for digital-twin-aquatic (Kiwanis Lake).

One ROS 2 node that turns typed JSON commands into dock / boat-goto / helix /
coverage actions and reports completion on matching event/status topics. This
is the twin-side executor layer that a future mission_supervisor can drive; it
is not the LLM.

Topics (std_msgs/String, JSON):
  /coral/command   in
  /coral/event     out
  /coral/status    out  (~1 Hz)

Commands:
  {"type":"dock","action":"attach"|"detach","id"?}
  {"type":"goto","east_m":..,"north_m":..,"relative_to":"current"|"world"|"heading",
   "arrive_m"?, "timeout_s"?, "id"?}
      current  = ENU offset from present pose (east_m → +X, north_m → +Y)
      world    = absolute ENU target at (east_m, north_m)
      heading  = body offset (north_m forward along yaw, east_m to starboard)
  {"type":"survey","width_m":20,"length_m":20,"swath_m"?, "heading_deg"?,
   "center_east_m"?, "center_north_m"?, "relative_to":"current"|"world",
   "until"?: "tag_detected", "arrive_m"?, "id"?}
  {"type":"helix","duration_s"?, "radius_m"?, "omega"?, "vx"?, "z0"?, "id"?}
  {"type":"coverage","width_m"?, "length_m"?, "swath_m"?, "layers"?,
   "z_top"?, "z_bottom"?, "speed_mps"?, "heading_deg"?,
   "center_east_m"?, "center_north_m"?, "until"?: "tag_detected", "id"?}
      AUV 3D lawnmower (boustrophedon at several depths). Same force→hull
      loop as helix; path centred on thrust-on pose, or on world ENU centre.
      until tag_detected ends early when /coral/detections reports a tag
      (xudrone nadir camera + apriltag stack).
  {"type":"abort","id"?}
  {"type":"hold","id"?}          # disarm the boat if a goto is running

Events:
  {"type":"accepted"|"rejected"|"failed"|"arrived"|"survey_done"|"dock_done"|
   "helix_done"|"coverage_done"|"aborted", "id", ...}

Status (1 Hz): busy, phase, latch, boat/auv pose, active_id, last_event.

Env: ROS_DOMAIN_ID=84, GZ_PARTITION=aquatic (set by run.sh).
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition, VehicleStatus

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

# Shared Gazebo helpers + detect_target live with the helix test module.
sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
from udrone_sinusoidal_controller import (  # noqa: E402
    GZ_ENV,
    GzPoses,
    LatchState,
    detect_target,
    rpy_from_q,
)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from patterns import lawnmower  # noqa: E402

NS_BOAT = "/otter_k/fmu"
FORCE_ARM = 21196.0
WORLD_ENU = "world"
REL_CURRENT = "current"
CMD_TOPIC = "/coral/command"
EVT_TOPIC = "/coral/event"
STS_TOPIC = "/coral/status"
DET_TOPIC = "/coral/detections"
DOCK_CMD = "/udrone_dock/command"


def _new_id() -> str:
    return uuid.uuid4().hex[:10]


class CoralBehaviours(Node):
    def __init__(self):
        super().__init__("coral_behaviours")
        self.gz = GzPoses()
        self.latch = LatchState()
        self.boat_model = "heron_0"
        self.auv_model, self.auv_px4 = detect_target()

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.evt_pub = self.create_publisher(String, EVT_TOPIC, 10)
        self.sts_pub = self.create_publisher(String, STS_TOPIC, 10)
        self.create_subscription(String, CMD_TOPIC, self._on_command, 10)
        self.create_subscription(String, DET_TOPIC, self._on_detections, 10)
        # Live ENU refine while a tag is in view (hover_above analogue).
        self.create_subscription(String, "/coral/tracks", self._on_tracks, 10)
        # Raw apriltag arrays (sensor QoS): stop-on-tag without waiting for ENU
        # conversion. xudrone primary detections are the FRONT camera; down cam
        # is optional on /xudrone/detections_down.
        if AprilTagDetectionArray is not None:
            for topic in (
                "/xudrone/detections",
                "/xudrone/detections_down",
                "/otter_k/detections",
            ):
                self.create_subscription(
                    AprilTagDetectionArray, topic, self._on_apriltag_raw,
                    qos_profile_sensor_data,
                )

        self.boat_cmd = self.create_publisher(
            VehicleCommand, f"{NS_BOAT}/in/vehicle_command", pub_qos
        )
        self.boat_status = None
        self.boat_gpos = None
        self.boat_status_sub = None
        self.create_subscription(
            VehicleGlobalPosition,
            f"{NS_BOAT}/out/vehicle_global_position",
            lambda m: setattr(self, "boat_gpos", m),
            qos_profile_sensor_data,
        )

        self.phase = "idle"          # idle | dock | goto | survey | helix | coverage | approach | hold
        self.active_id = None
        self.busy = False
        self.last_event = None
        self.reject_reason = None
        self.last_detection = None

        # goto state
        self._goto = None
        # survey state (lawnmower of gotos)
        self._survey = None
        # dock state
        self._dock = None
        # AUV swim subprocess (helix / coverage / approach)
        self._helix_proc = None
        self._helix_meta = None
        self._auv_kind = None        # "helix" | "coverage" | "approach"

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"coral behaviours up: auv={self.auv_model} px4={self.auv_px4} "
            f"cmd={CMD_TOPIC} evt={EVT_TOPIC} det={DET_TOPIC}"
        )

    # ---- messaging ------------------------------------------------------
    def _emit(self, event: dict):
        event.setdefault("t_wall", time.time())
        self.last_event = event
        self.evt_pub.publish(String(data=json.dumps(event)))
        self.get_logger().info(f"event {event.get('type')} {event}")

    def _publish_status(self):
        fresh, sim_t, (boat, auv) = self.gz.read(self.boat_model, self.auv_model)
        boat_s = auv_s = None
        if boat is not None:
            (x, y, z), q = boat
            boat_s = {"x": x, "y": y, "z": z, "yaw": rpy_from_q(q)[2]}
        if auv is not None:
            (x, y, z), q = auv
            auv_s = {"x": x, "y": y, "z": z, "yaw": rpy_from_q(q)[2]}
        msg = {
            "busy": self.busy,
            "phase": self.phase,
            "latch": self.latch.state,
            "active_id": self.active_id,
            "boat": boat_s,
            "auv": auv_s,
            "auv_model": self.auv_model,
            "sim_t": sim_t if fresh else None,
            "last_event": (self.last_event or {}).get("type"),
            "last_detection": self.last_detection,
        }
        self.sts_pub.publish(String(data=json.dumps(msg)))

    def _on_detections(self, m: String):
        try:
            dets = json.loads(m.data)
        except ValueError:
            return
        if not dets:
            return
        rec = dets[0] if isinstance(dets, list) else dets
        if not isinstance(rec, dict):
            return
        self._handle_tag_sighting(rec)

    def _on_tracks(self, m: String):
        """Refresh last_detection from live ENU tracks (no until-stop side effects)."""
        try:
            dets = json.loads(m.data)
        except ValueError:
            return
        if not dets:
            return
        rec = dets[0] if isinstance(dets, list) else dets
        if not isinstance(rec, dict) or rec.get("e") is None or rec.get("n") is None:
            return
        rec = dict(rec)
        rec.setdefault("source", "tracks")
        self.last_detection = rec

    def _on_apriltag_raw(self, m):
        if not m.detections:
            return
        d = m.detections[0]
        # Pose estimate from current AUV/boat ENU (good enough for until-stop).
        n = e = None
        fresh, _, (boat, auv) = self.gz.read(self.boat_model, self.auv_model)
        pose = auv if (fresh and auv is not None) else (boat if (fresh and boat is not None) else None)
        if pose is not None:
            (x, y, z), _q = pose
            e, n = float(x), float(y)
        rec = {
            "label": f"apriltag {int(d.id)}", "id": int(d.id),
            "n": None if n is None else round(n, 1),
            "e": None if e is None else round(e, 1),
            "conf": 0.9, "alt_m": None, "source": "apriltag_raw",
        }
        self._handle_tag_sighting(rec)

    def _handle_tag_sighting(self, rec: dict):
        self.last_detection = rec
        if self.phase == "survey" and self._survey and self._survey.get("until") == "tag_detected":
            self.get_logger().info(f"survey stop: tag detected {rec}")
            self._finish_survey(tag_detected=True, detection=rec)
        elif (self.phase == "coverage" and self._helix_meta
              and self._helix_meta.get("until") == "tag_detected"):
            self.get_logger().info(f"coverage stop: tag detected {rec}")
            self._finish_coverage(tag_detected=True, detection=rec)

    # ---- command dispatch ----------------------------------------------
    def _on_command(self, m: String):
        try:
            cmd = json.loads(m.data)
        except ValueError:
            self._emit({"type": "rejected", "id": None, "reason": "command is not JSON"})
            return
        if not isinstance(cmd, dict) or "type" not in cmd:
            self._emit({"type": "rejected", "id": cmd.get("id") if isinstance(cmd, dict) else None,
                        "reason": "missing type"})
            return
        kind = str(cmd["type"]).lower()
        cid = cmd.get("id") or _new_id()
        cmd["id"] = cid

        if kind == "abort":
            self._abort(cid, reason=cmd.get("reason") or "abort")
            return
        if self.busy and kind != "hold":
            self._emit({"type": "rejected", "id": cid, "reason": f"busy ({self.phase})",
                        "command": kind})
            return

        if kind == "dock":
            self._start_dock(cmd)
        elif kind == "goto":
            self._start_goto(cmd)
        elif kind == "survey":
            self._start_survey(cmd)
        elif kind == "helix":
            self._start_helix(cmd)
        elif kind == "coverage":
            self._start_coverage(cmd)
        elif kind in ("approach_tag", "approach"):
            self._start_approach_tag(cmd)
        elif kind == "hold":
            self._hold(cid)
        else:
            self._emit({"type": "rejected", "id": cid, "reason": f"unknown type {kind}"})

    def _abort(self, cid, reason="abort"):
        if self._helix_proc is not None and self._helix_proc.poll() is None:
            self._helix_proc.terminate()
            try:
                self._helix_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._helix_proc.kill()
        if self.phase == "goto" and self._boat_armed():
            self._boat_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                               param1=0.0, param2=FORCE_ARM)
        was = self.phase
        self._clear_job()
        self._emit({"type": "aborted", "id": cid, "was": was, "reason": reason})

    def _hold(self, cid):
        if self.phase == "survey":
            self._boat_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                               param1=0.0, param2=FORCE_ARM)
            self._finish_survey(tag_detected=False)
            return
        if self.phase == "goto":
            self._boat_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                               param1=0.0, param2=FORCE_ARM)
            self._emit({"type": "arrived", "id": self.active_id or cid, "held": True,
                        "east_m": None, "north_m": None})
            self._clear_job()
        else:
            self._emit({"type": "accepted", "id": cid, "command": "hold", "note": "nothing to hold"})

    def _clear_job(self):
        self.phase = "idle"
        self.busy = False
        self.active_id = None
        self._goto = None
        self._survey = None
        self._dock = None
        self._helix_proc = None
        self._helix_meta = None
        self._auv_kind = None

    # ---- dock -----------------------------------------------------------
    def _start_dock(self, cmd):
        action = str(cmd.get("action", "")).lower()
        if action not in ("attach", "detach"):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "dock.action must be attach|detach"})
            return
        want = "attached" if action == "attach" else "detached"
        self.busy = True
        self.phase = "dock"
        self.active_id = cmd["id"]
        self._dock = {"want": want, "action": action, "deadline": time.time() + 8.0,
                      "sent": False}
        self._emit({"type": "accepted", "id": cmd["id"], "command": "dock", "action": action})

    def _tick_dock(self):
        d = self._dock
        if not d["sent"]:
            rc = subprocess.run(
                ["gz", "topic", "-t", DOCK_CMD, "-m", "gz.msgs.StringMsg",
                 "-p", f'data: "{d["action"].upper()}"'],
                capture_output=True, text=True, timeout=10, env=GZ_ENV,
            ).returncode
            d["sent"] = True
            if rc != 0:
                self._emit({"type": "failed", "id": self.active_id,
                            "reason": f"could not publish dock {d['action']}"})
                self._clear_job()
                return
        if self.latch.state == d["want"]:
            self._emit({"type": "dock_done", "id": self.active_id, "state": self.latch.state,
                        "action": d["action"]})
            self._clear_job()
            return
        if time.time() > d["deadline"]:
            self._emit({"type": "failed", "id": self.active_id,
                        "reason": f"dock {d['action']} timed out (latch={self.latch.state})"})
            self._clear_job()

    # ---- boat goto ------------------------------------------------------
    def _start_goto(self, cmd):
        try:
            east = float(cmd.get("east_m", 0.0))
            north = float(cmd.get("north_m", 0.0))
        except (TypeError, ValueError):
            self._emit({"type": "rejected", "id": cmd["id"], "reason": "east_m/north_m must be numbers"})
            return
        rel = str(cmd.get("relative_to", REL_CURRENT)).lower()
        if rel not in (REL_CURRENT, WORLD_ENU, "absolute", "heading"):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "relative_to must be current|world|heading"})
            return
        if rel == "absolute":
            rel = WORLD_ENU
        # Convenience: along_heading_m → heading-relative forward leg.
        if cmd.get("along_heading_m") is not None:
            rel = "heading"
            north = float(cmd["along_heading_m"])
            east = float(cmd.get("east_m", 0.0))
        arrive = float(cmd.get("arrive_m", os.environ.get("BOAT_ARRIVE_M", "2.0")))
        timeout = float(cmd.get("timeout_s", os.environ.get("BOAT_TIMEOUT_S", "120")))
        self.busy = True
        self.phase = "goto"
        self.active_id = cmd["id"]
        self._goto = {
            "east": east, "north": north, "rel": rel,
            "arrive_m": arrive, "timeout_s": timeout,
            "sub": "wait", "t0": None, "target_xy": None, "target_ll": None,
            "next_send": 0.0, "arm_tries": 0, "forced_arm": False, "closest": float("inf"),
        }
        self._emit({"type": "accepted", "id": cmd["id"], "command": "goto",
                    "east_m": east, "north_m": north, "relative_to": rel})

    def _boat_command(self, command, **params):
        m = VehicleCommand()
        m.timestamp = 0
        m.command = command
        for k, v in params.items():
            setattr(m, k, float(v))
        m.target_system = int(self.boat_status.system_id) if self.boat_status else 1
        m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.boat_cmd.publish(m)

    def _boat_armed(self):
        return bool(self.boat_status and
                    self.boat_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _find_boat_status_topic(self):
        names = [
            n for n, _ in self.get_topic_names_and_types()
            if n.startswith(f"{NS_BOAT}/out/vehicle_status") and self.count_publishers(n) > 0
        ]
        if not names:
            return
        topic = sorted(names)[-1]
        self.boat_status_sub = self.create_subscription(
            VehicleStatus, topic, lambda m: setattr(self, "boat_status", m),
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"boat status topic {topic}")

    def _tick_goto(self):
        g = self._goto
        if self.boat_status_sub is None:
            self._find_boat_status_topic()
        fresh, sim_t, (boat, _) = self.gz.read(self.boat_model, self.auv_model)
        if not (fresh and boat is not None and self.boat_status and self.boat_gpos):
            return
        (bx, by, bz), bq = boat

        if g["sub"] == "wait":
            if g["rel"] == WORLD_ENU:
                tx, ty = g["east"], g["north"]
            elif g["rel"] == "heading":
                yaw = rpy_from_q(bq)[2]
                # north_m = forward along yaw, east_m = starboard (right).
                fwd, stbd = g["north"], g["east"]
                tx = bx + fwd * math.cos(yaw) - stbd * math.sin(yaw)
                ty = by + fwd * math.sin(yaw) + stbd * math.cos(yaw)
            else:
                # current: ENU offset from present pose.
                tx, ty = bx + g["east"], by + g["north"]
            g["target_xy"] = (tx, ty)
            d_e, d_n = tx - bx, ty - by
            lat, lon = float(self.boat_gpos.lat), float(self.boat_gpos.lon)
            g["target_ll"] = (
                lat + d_n / 111320.0,
                lon + d_e / (111320.0 * math.cos(math.radians(lat))),
            )
            g["t0"] = sim_t
            g["sub"] = "arm"
            self.get_logger().info(
                f"goto target ENU=({tx:.2f},{ty:.2f}) from ({bx:.2f},{by:.2f})"
            )

        t = sim_t - g["t0"]
        if g["sub"] == "arm":
            if self._boat_armed():
                g["sub"] = "go"
                g["next_send"] = 0.0
            elif t >= g["next_send"]:
                g["next_send"] = t + 1.5
                g["arm_tries"] += 1
                if g["arm_tries"] > 8:
                    self._emit({"type": "failed", "id": self.active_id,
                                "reason": "boat would not arm"})
                    self._clear_job()
                    return
                g["forced_arm"] = g["arm_tries"] > 2
                self._boat_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0, param2=FORCE_ARM if g["forced_arm"] else 0.0,
                )
            return

        if g["sub"] == "go":
            d = math.dist((bx, by), g["target_xy"])
            g["closest"] = min(g["closest"], d)
            if not self._boat_armed():
                self._emit({"type": "failed", "id": self.active_id,
                            "reason": f"boat disarmed {d:.1f} m from target"})
                self._clear_job()
                return
            if d <= g["arrive_m"]:
                if self._survey is not None:
                    self._survey_advance_leg(bx, by, d, t)
                    return
                self._emit({
                    "type": "arrived", "id": self.active_id,
                    "east_m": bx, "north_m": by, "dist_m": d,
                    "elapsed_s": t,
                })
                # Leave the boat armed in Auto Loiter (surface_executor style);
                # an explicit hold/abort disarms.
                self._clear_job()
                return
            if t > g["timeout_s"]:
                self._emit({"type": "failed", "id": self.active_id,
                            "reason": f"goto timeout (closest {g['closest']:.1f} m)"})
                self._clear_job()
                return
            if t >= g["next_send"]:
                g["next_send"] = t + 1.0
                self._boat_command(
                    VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                    param1=-1.0, param2=1.0, param4=math.nan,
                    param5=g["target_ll"][0], param6=g["target_ll"][1], param7=math.nan,
                )

    # ---- survey (lawnmower of world gotos) ------------------------------
    def _start_survey(self, cmd):
        try:
            width = float(cmd.get("width_m"))
            length = float(cmd.get("length_m"))
        except (TypeError, ValueError):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "survey needs width_m and length_m"})
            return
        swath = float(cmd.get("swath_m") or max(4.0, min(width, length) / 4.0))
        heading = float(cmd.get("heading_deg") or 0.0)
        until = cmd.get("until")
        if until not in (None, "", "tag_detected"):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "survey.until must be null or tag_detected"})
            return
        rel = str(cmd.get("relative_to") or REL_CURRENT).lower()
        if rel in ("absolute",):
            rel = WORLD_ENU
        if rel not in (REL_CURRENT, WORLD_ENU):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "survey.relative_to must be current|world"})
            return
        fresh, _, (boat, _) = self.gz.read(self.boat_model, self.auv_model)
        if not (fresh and boat is not None):
            self._emit({"type": "rejected", "id": cmd["id"], "reason": "no boat pose yet"})
            return
        (bx, by, _), _ = boat
        ce = cmd.get("center_east_m", cmd.get("de", cmd.get("east_m")))
        cn = cmd.get("center_north_m", cmd.get("dn", cmd.get("north_m")))
        if ce is None and cn is None:
            ce, cn = bx, by
        else:
            ce = float(ce or 0.0)
            cn = float(cn or 0.0)
            if rel == REL_CURRENT:
                ce, cn = bx + ce, by + cn
        try:
            pts = lawnmower(cn, ce, width, length, swath, heading, start=(by, bx))
        except ValueError as ex:
            self._emit({"type": "rejected", "id": cmd["id"], "reason": str(ex)})
            return
        # pts are (north, east); coral goto wants east_m/north_m world.
        wps = [(e, n) for n, e in pts]
        arrive = float(cmd.get("arrive_m", os.environ.get("BOAT_ARRIVE_M", "2.0")))
        timeout = float(cmd.get("timeout_s", os.environ.get("BOAT_TIMEOUT_S", "180")))
        self.busy = True
        self.phase = "survey"
        self.active_id = cmd["id"]
        self._survey = {
            "wps": wps, "i": 0, "until": until or None,
            "arrive_m": arrive, "timeout_s": timeout,
            "width_m": width, "length_m": length, "swath_m": swath,
            "center": {"east_m": ce, "north_m": cn},
            "tag_detected": False, "detection": None,
        }
        self._emit({
            "type": "accepted", "id": cmd["id"], "command": "survey",
            "width_m": width, "length_m": length, "swath_m": swath,
            "waypoints": len(wps), "until": until,
            "center": self._survey["center"],
        })
        self._survey_begin_leg()

    def _survey_begin_leg(self):
        s = self._survey
        if s is None or s["i"] >= len(s["wps"]):
            self._finish_survey(tag_detected=False)
            return
        east, north = s["wps"][s["i"]]
        self.phase = "survey"
        self._goto = {
            "east": east, "north": north, "rel": WORLD_ENU,
            "arrive_m": s["arrive_m"], "timeout_s": s["timeout_s"],
            "sub": "wait", "t0": None, "target_xy": None, "target_ll": None,
            "next_send": 0.0, "arm_tries": 0, "forced_arm": False, "closest": float("inf"),
        }

    def _survey_advance_leg(self, bx, by, d, t):
        s = self._survey
        if s is None:
            return
        s["i"] += 1
        self.get_logger().info(
            f"survey leg {s['i']}/{len(s['wps'])} done at ({bx:.1f},{by:.1f}) d={d:.1f}"
        )
        if s["i"] >= len(s["wps"]):
            self._finish_survey(tag_detected=False)
            return
        # Keep armed; queue next world waypoint.
        self._survey_begin_leg()

    def _finish_survey(self, tag_detected: bool, detection=None):
        s = self._survey or {}
        cid = self.active_id
        # Soft-stop the boat if we cut a leg short for a tag.
        if tag_detected and self._boat_armed():
            self._boat_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                               param1=0.0, param2=FORCE_ARM)
        ev = {
            "type": "survey_done", "id": cid,
            "tag_detected": bool(tag_detected),
            "detection": detection or s.get("detection"),
            "legs_done": int(s.get("i") or 0),
            "legs_total": len(s.get("wps") or []),
            "width_m": s.get("width_m"), "length_m": s.get("length_m"),
            "swath_m": s.get("swath_m"),
        }
        self._clear_job()
        self._emit(ev)

    # ---- helix / coverage (AUV subprocess) ------------------------------
    def _start_helix(self, cmd):
        duration = float(cmd.get("duration_s", os.environ.get("UDRONE_HELIX_DUR", "45")))
        radius = float(cmd.get("radius_m", os.environ.get("UDRONE_HELIX_R", "0.5")))
        omega = float(cmd.get("omega", os.environ.get("UDRONE_HELIX_OMEGA", "0.40")))
        vx = float(cmd.get("vx", os.environ.get("UDRONE_HELIX_VX", "0.20")))
        z0 = float(cmd.get("z0", os.environ.get("UDRONE_HELIX_Z0", "-1.2")))
        runner = os.environ.get(
            "UDRONE_HELIX_RUNNER",
            "/root/udrone_boat/behaviours/run_helix.py",
        )
        if not os.path.isfile(runner):
            here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_helix.py")
            runner = here if os.path.isfile(here) else "/root/udrone_boat/src/udrone_sinusoidal_controller.py"

        env = {
            **os.environ,
            "GZ_PARTITION": os.environ.get("GZ_PARTITION", "aquatic"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "84"),
            "UDRONE_HELIX_DUR": str(duration),
            "UDRONE_HELIX_R": str(radius),
            "UDRONE_HELIX_OMEGA": str(omega),
            "UDRONE_HELIX_VX": str(vx),
            "UDRONE_HELIX_Z0": str(z0),
            "UDRONE_GZ_MODEL": self.auv_model,
            "UDRONE_PX4_INSTANCE": str(self.auv_px4),
            "UDRONE_HELIX_BEHAVIOUR": "1",
        }
        meta = {
            "duration_s": duration, "radius_m": radius, "omega": omega, "vx": vx, "z0": z0,
        }
        self._spawn_auv("helix", cmd["id"], runner, env, meta, "/tmp/coral_helix.log")

    def _start_coverage(self, cmd):
        width = float(cmd.get("width_m", os.environ.get("UDRONE_COV_WIDTH", "12")))
        length = float(cmd.get("length_m", cmd.get("width_m",
                       os.environ.get("UDRONE_COV_LENGTH",
                                      os.environ.get("UDRONE_COV_WIDTH", "12")))))
        swath = float(cmd.get("swath_m", os.environ.get("UDRONE_COV_SWATH", "3")))
        layers = int(float(cmd.get("layers", os.environ.get("UDRONE_COV_LAYERS", "3"))))
        z_top = float(cmd.get("z_top", os.environ.get("UDRONE_COV_Z_TOP", "-0.9")))
        z_bottom = float(cmd.get("z_bottom", os.environ.get("UDRONE_COV_Z_BOTTOM", "-1.9")))
        speed = float(cmd.get("speed_mps", os.environ.get("UDRONE_COV_SPEED", "0.25")))
        heading = float(cmd.get("heading_deg", os.environ.get("UDRONE_COV_HEADING", "0")))
        until = cmd.get("until")
        if until not in (None, "", "tag_detected"):
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "coverage.until must be null or tag_detected"})
            return
        if width <= 0 or length <= 0 or swath <= 0 or layers < 1:
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "coverage needs positive width/length/swath and layers>=1"})
            return
        runner = os.environ.get(
            "UDRONE_COV_RUNNER",
            "/root/udrone_boat/behaviours/run_coverage.py",
        )
        if not os.path.isfile(runner):
            here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_coverage.py")
            if not os.path.isfile(here):
                self._emit({"type": "rejected", "id": cmd["id"],
                            "reason": f"coverage runner not found ({runner})"})
                return
            runner = here

        env = {
            **os.environ,
            "GZ_PARTITION": os.environ.get("GZ_PARTITION", "aquatic"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "84"),
            "UDRONE_COV_WIDTH": str(width),
            "UDRONE_COV_LENGTH": str(length),
            "UDRONE_COV_SWATH": str(swath),
            "UDRONE_COV_LAYERS": str(layers),
            "UDRONE_COV_Z_TOP": str(z_top),
            "UDRONE_COV_Z_BOTTOM": str(z_bottom),
            "UDRONE_COV_SPEED": str(speed),
            "UDRONE_COV_HEADING": str(heading),
            "UDRONE_GZ_MODEL": self.auv_model,
            "UDRONE_PX4_INSTANCE": str(self.auv_px4),
        }
        ce = cmd.get("center_east_m", cmd.get("de", cmd.get("east_m")))
        cn = cmd.get("center_north_m", cmd.get("dn", cmd.get("north_m")))
        if ce is not None and cn is not None:
            env["UDRONE_COV_CENTER_E"] = str(float(ce))
            env["UDRONE_COV_CENTER_N"] = str(float(cn))
        meta = {
            "width_m": width, "length_m": length, "swath_m": swath,
            "layers": layers, "z_top": z_top, "z_bottom": z_bottom,
            "speed_mps": speed, "heading_deg": heading,
            "until": until or None,
        }
        if ce is not None and cn is not None:
            meta["center_east_m"] = float(ce)
            meta["center_north_m"] = float(cn)
        self._spawn_auv("coverage", cmd["id"], runner, env, meta, "/tmp/coral_coverage.log")

    def _start_approach_tag(self, cmd):
        tag_id = int(float(cmd.get("tag_id", 0)))
        standoff = float(cmd.get("standoff_m", os.environ.get("UDRONE_APPROACH_STANDOFF", "0")))
        depth = float(cmd.get("depth_m", os.environ.get("UDRONE_APPROACH_DEPTH", "-1.2")))
        arrive = float(cmd.get("arrive_m", os.environ.get("UDRONE_APPROACH_ARRIVE", "0.5")))
        depth_tol = float(cmd.get("depth_tol_m", os.environ.get("UDRONE_APPROACH_DEPTH_TOL", "0.25")))
        speed = float(cmd.get("speed_mps", os.environ.get("UDRONE_APPROACH_SPEED", "0.25")))
        settle = float(cmd.get("settle_s", os.environ.get("UDRONE_APPROACH_SETTLE", "3")))
        timeout = float(cmd.get("timeout_s", os.environ.get("UDRONE_APPROACH_TIMEOUT", "90")))
        acquire = float(cmd.get("acquire_s", os.environ.get("UDRONE_APPROACH_ACQUIRE", "25")))
        if arrive <= 0 or speed <= 0 or timeout <= 0:
            self._emit({"type": "rejected", "id": cmd["id"],
                        "reason": "approach_tag needs positive arrive_m/speed_mps/timeout_s"})
            return
        runner = os.environ.get(
            "UDRONE_APPROACH_RUNNER",
            "/root/udrone_boat/behaviours/run_approach_tag.py",
        )
        if not os.path.isfile(runner):
            here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_approach_tag.py")
            if not os.path.isfile(here):
                self._emit({"type": "rejected", "id": cmd["id"],
                            "reason": f"approach_tag runner not found ({runner})"})
                return
            runner = here

        ce = cmd.get("center_east_m", cmd.get("de", cmd.get("east_m")))
        cn = cmd.get("center_north_m", cmd.get("dn", cmd.get("north_m")))
        # Seed from last AprilTag ENU when the command omits a centre.
        if (ce is None or cn is None) and isinstance(self.last_detection, dict):
            det = self.last_detection
            if det.get("e") is not None and det.get("n") is not None:
                if det.get("source") != "apriltag_raw":
                    ce = det.get("e") if ce is None else ce
                    cn = det.get("n") if cn is None else cn
                elif ce is None and cn is None:
                    # Raw sighting only has vehicle pose — still a coarse seed.
                    ce, cn = det.get("e"), det.get("n")

        env = {
            **os.environ,
            "GZ_PARTITION": os.environ.get("GZ_PARTITION", "aquatic"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "84"),
            "UDRONE_APPROACH_TAG_ID": str(tag_id),
            "UDRONE_APPROACH_STANDOFF": str(standoff),
            "UDRONE_APPROACH_DEPTH": str(depth),
            "UDRONE_APPROACH_ARRIVE": str(arrive),
            "UDRONE_APPROACH_DEPTH_TOL": str(depth_tol),
            "UDRONE_APPROACH_SPEED": str(speed),
            "UDRONE_APPROACH_SETTLE": str(settle),
            "UDRONE_APPROACH_TIMEOUT": str(timeout),
            "UDRONE_APPROACH_ACQUIRE": str(acquire),
            "UDRONE_GZ_MODEL": self.auv_model,
            "UDRONE_PX4_INSTANCE": str(self.auv_px4),
        }
        if ce is not None and cn is not None:
            env["UDRONE_APPROACH_CENTER_E"] = str(float(ce))
            env["UDRONE_APPROACH_CENTER_N"] = str(float(cn))
        meta = {
            "tag_id": tag_id, "standoff_m": standoff, "depth_m": depth,
            "arrive_m": arrive, "depth_tol_m": depth_tol, "speed_mps": speed,
            "settle_s": settle, "timeout_s": timeout,
        }
        if ce is not None and cn is not None:
            meta["center_east_m"] = float(ce)
            meta["center_north_m"] = float(cn)
        self._spawn_auv("approach", cmd["id"], runner, env, meta, "/tmp/coral_approach.log")

    def _kill_auv_proc(self):
        if self._helix_proc is None or self._helix_proc.poll() is not None:
            return
        self._helix_proc.terminate()
        try:
            self._helix_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._helix_proc.kill()

    def _finish_coverage(self, tag_detected: bool, detection=None):
        meta = dict(self._helix_meta or {})
        cid = self.active_id
        self._kill_auv_proc()
        ev = {
            "type": "coverage_done", "id": cid,
            "failed": None,
            "tag_detected": bool(tag_detected),
            "detection": detection or self.last_detection,
            **{k: v for k, v in meta.items() if k != "until"},
            "until": meta.get("until"),
        }
        self._clear_job()
        self._emit(ev)

    def _spawn_auv(self, kind: str, cid: str, runner: str, env: dict, meta: dict, log_path: str):
        self.busy = True
        self.phase = kind
        self.active_id = cid
        self._auv_kind = kind
        self._helix_meta = meta
        # Truncate so each swim has a clean log (avoid orphan append noise).
        log = open(log_path, "wb", buffering=0)
        self._helix_proc = subprocess.Popen(
            [sys.executable, "-u", runner],
            env=env, stdout=log, stderr=subprocess.STDOUT,
        )
        self._emit({
            "type": "accepted", "id": cid, "command": kind,
            **meta, "pid": self._helix_proc.pid,
        })

    def _tick_auv(self):
        if self._helix_proc is None:
            return
        rc = self._helix_proc.poll()
        if rc is None:
            return
        kind = self._auv_kind or self.phase or "helix"
        done = f"{kind}_done"
        meta = self._helix_meta or {}
        if kind == "approach":
            ev = {
                "type": "approach_done", "id": self.active_id,
                "arrived": rc == 0, "failed": None if rc == 0 else f"approach exited {rc}",
                "detection": self.last_detection, **meta,
            }
            self._clear_job()
            self._emit(ev)
            if rc != 0:
                self._emit({"type": "failed", "id": ev["id"],
                            "reason": ev["failed"]})
            return
        if rc == 0:
            self._emit({"type": done, "id": self.active_id, "failed": None,
                        "tag_detected": False, **meta})
        else:
            self._emit({"type": done, "id": self.active_id,
                        "failed": f"{kind} exited {rc}", "tag_detected": False, **meta})
            self._emit({"type": "failed", "id": self.active_id,
                        "reason": f"{kind} exited {rc}"})
        self._clear_job()

    # ---- main tick ------------------------------------------------------
    def _tick(self):
        if self.phase == "dock":
            self._tick_dock()
        elif self.phase in ("goto", "survey"):
            self._tick_goto()
        elif self.phase in ("helix", "coverage", "approach"):
            self._tick_auv()

    def destroy_node(self):
        if self._helix_proc is not None and self._helix_proc.poll() is None:
            self._helix_proc.terminate()
        self.gz.stop()
        self.latch.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = CoralBehaviours()
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
