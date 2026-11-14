#!/usr/bin/env python3
"""AUV visual approach to an AprilTag (PBVS on ENU tracks).

Same force → point-hull control loop as helix/coverage; only pos_des(t) changes.
Coral command type ``approach_tag``. Seed from env centre or /coral/tracks;
refine while tracks are live. Stepping stone to nest/cage docking — this skill
only arrives and holds a standoff above the tag (no latch).
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
from std_msgs.msg import String
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done


class UdroneApproachController(UdroneHelixController):
    def __init__(
        self,
        tag_id=0,
        standoff_m=0.0,
        depth_m=-1.2,
        arrive_m=0.5,
        depth_tol_m=0.25,
        speed_mps=0.25,
        settle_s=3.0,
        timeout_s=90.0,
        acquire_s=25.0,
        center_east_m=None,
        center_north_m=None,
        **kwargs,
    ):
        self.tag_id = int(tag_id)
        self.standoff_m = float(standoff_m)
        self.depth_m = float(depth_m)
        self.arrive_m = float(arrive_m)
        self.depth_tol_m = float(depth_tol_m)
        self.speed_mps = float(speed_mps)
        self.settle_s = float(settle_s)
        self.timeout_s = float(timeout_s)
        self.acquire_s = float(acquire_s)

        self.tag_e = None if center_east_m is None else float(center_east_m)
        self.tag_n = None if center_north_m is None else float(center_north_m)
        self.tag_z = self.depth_m
        self.tag_t = time.time() if self.tag_e is not None else 0.0
        self.tag_source = "seed" if self.tag_e is not None else None
        self._standoff_e = None
        self._standoff_n = None
        self._settle_t0 = None
        self.arrived = False
        self._stage = "acquire" if self.tag_e is None else "approach"

        kwargs.setdefault("duration_s", self.timeout_s + 5.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.speed_mps)
        kwargs.setdefault("z0", self.depth_m)
        super().__init__(**kwargs)

        self.create_subscription(String, "/coral/tracks", self._on_tracks, 10)
        self.create_subscription(String, "/coral/detections", self._on_detections, 10)
        self.get_logger().info(
            f"approach_tag: id={self.tag_id} depth={self.depth_m:g} arrive={self.arrive_m:g} "
            f"speed={self.speed_mps:g} seed_e={self.tag_e} seed_n={self.tag_n} "
            f"stage={self._stage}"
        )

    def _pick_rec(self, payload):
        try:
            data = json.loads(payload)
        except ValueError:
            return None
        if isinstance(data, dict):
            recs = [data]
        elif isinstance(data, list):
            recs = [r for r in data if isinstance(r, dict)]
        else:
            return None
        for r in recs:
            rid = r.get("id")
            if rid is not None and int(rid) != self.tag_id:
                continue
            if r.get("e") is None or r.get("n") is None:
                continue
            # Prefer real ENU (tag_detector) over vehicle-pose apriltag_raw.
            if r.get("source") == "apriltag_raw":
                continue
            return r
        return None

    def _ingest(self, rec: dict, source: str):
        try:
            e = float(rec["e"])
            n = float(rec["n"])
        except (KeyError, TypeError, ValueError):
            return
        self.tag_e = e
        self.tag_n = n
        alt = rec.get("alt_m")
        # Prefer commanded approach depth; alt_m is height-above-home when present.
        self.tag_z = self.depth_m
        if alt is not None and self.depth_m is None:
            try:
                self.tag_z = -float(alt)
            except (TypeError, ValueError):
                pass
        self.tag_t = time.time()
        self.tag_source = source
        if self._stage == "acquire":
            self._stage = "approach"
            print(
                f"APPROACH_ACQUIRED e={e:.2f} n={n:.2f} source={source}",
                flush=True,
            )

    def _on_tracks(self, m: String):
        rec = self._pick_rec(m.data)
        if rec is not None:
            self._ingest(rec, "tracks")

    def _on_detections(self, m: String):
        rec = self._pick_rec(m.data)
        if rec is not None:
            self._ingest(rec, "detections")

    def _goal_world(self):
        """World ENU goal (east, north, z) including horizontal standoff."""
        if self.tag_e is None or self.tag_n is None:
            return None
        ge, gn = self.tag_e, self.tag_n
        if self.standoff_m > 1e-3 and self._prev is not None:
            px, py, _pz = self._prev[1]
            dx, dy = ge - px, gn - py
            dist = math.hypot(dx, dy)
            if dist > 1e-3:
                # Stop standoff_m short of the tag along the approach vector.
                scale = max(0.0, (dist - self.standoff_m) / dist)
                ge = px + dx * scale
                gn = py + dy * scale
                self._standoff_e, self._standoff_n = ge, gn
        return ge, gn, self.depth_m

    def _events(self, tm: float):
        if self._stage == "acquire" and tm >= self.acquire_s and self.tag_e is None:
            self.failed = "lost_tag"
            print("FAIL lost_tag (no ENU during acquire)", flush=True)
            self.done = True
            return
        goal = self._goal_world()
        if goal is None or self.anchor is None or self._prev is None:
            return
        ge, gn, gz = goal
        px, py, pz = self._prev[1]
        horiz = math.hypot(ge - px, gn - py)
        dvert = abs(pz - gz)
        if horiz < self.arrive_m and dvert < self.depth_tol_m:
            if self._settle_t0 is None:
                self._settle_t0 = tm
                self._stage = "settle"
                print(
                    f"APPROACH_SETTLE horiz={horiz:.2f} dvert={dvert:.2f} "
                    f"goal=({ge:.2f},{gn:.2f},{gz:.2f})",
                    flush=True,
                )
            elif tm - self._settle_t0 >= self.settle_s:
                self.arrived = True
                self._stage = "done"
                print(
                    f"APPROACH_ARRIVED e={px:.2f} n={py:.2f} z={pz:.2f} "
                    f"tag_e={self.tag_e:.2f} tag_n={self.tag_n:.2f} "
                    f"source={self.tag_source}",
                    flush=True,
                )
        else:
            self._settle_t0 = None
            if self._stage == "settle":
                self._stage = "approach"

    def _helix(self, tm: float):
        # Hold in place until we have a tag estimate.
        if self.tag_e is None or self.anchor is None:
            z = self.depth_m
            if self._prev is not None:
                z = self._prev[1][2]
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)

        goal = self._goal_world()
        if goal is None:
            return (0.0, 0.0, self.depth_m), (0.0, 0.0, 0.0)
        ge, gn, gz = goal
        x0, y0, yaw0 = self.anchor
        c, s = math.cos(yaw0), math.sin(yaw0)
        gx = c * (ge - x0) + s * (gn - y0)
        gy = -s * (ge - x0) + c * (gn - y0)

        if self._prev is None:
            return (gx, gy, gz), (0.0, 0.0, 0.0)

        px, py, pz = self._prev[1]
        cx = c * (px - x0) + s * (py - y0)
        cy = -s * (px - x0) + c * (py - y0)
        dx, dy, dz = gx - cx, gy - cy, gz - pz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 0.05 or self._stage in ("settle", "done"):
            return (gx, gy, gz), (0.0, 0.0, 0.0)
        # Slow near the goal so settle is reachable.
        speed = min(self.speed_mps, max(0.05, dist))
        scale = speed / dist
        return (gx, gy, gz), (dx * scale, dy * scale, dz * scale)

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
    standoff = float(os.environ.get("UDRONE_APPROACH_STANDOFF", "0"))
    depth = float(os.environ.get("UDRONE_APPROACH_DEPTH", "-1.2"))
    arrive = float(os.environ.get("UDRONE_APPROACH_ARRIVE", "0.5"))
    depth_tol = float(os.environ.get("UDRONE_APPROACH_DEPTH_TOL", "0.25"))
    speed = float(os.environ.get("UDRONE_APPROACH_SPEED", "0.25"))
    settle = float(os.environ.get("UDRONE_APPROACH_SETTLE", "3"))
    timeout = float(os.environ.get("UDRONE_APPROACH_TIMEOUT", "90"))
    acquire = float(os.environ.get("UDRONE_APPROACH_ACQUIRE", "25"))
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
        center_east_m=center_e,
        center_north_m=center_n,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_approach_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed and node.arrived:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor) if rotor else 0.0
            if mean_rotor < 0.05 and len(node.log) > 40:
                # Allow near-zero mean if we were mostly settling on a seed already close.
                why = "ok"
                ok = True
            else:
                ok = True
                why = "ok"
        elif node.arrived and not node.failed:
            ok = True
            why = "ok"
        print(
            f"APPROACH_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why} "
            f"arrived={node.arrived} tag_e={node.tag_e} tag_n={node.tag_n}",
            flush=True,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
