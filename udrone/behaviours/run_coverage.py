#!/usr/bin/env python3
"""AUV volume coverage: 3D boustrophedon at several depths.

Same force → point-hull control loop as the helix swim; only pos_des(t) changes,
and its clock waits for the hull when the hull falls behind.
Coral command type ``coverage``. Path is built in the thrust-on path frame
(centred on the AUV, or shifted to a world ENU centre if given — e.g. last tag).

With plan="map" (UDRONE_COV_PLAN=map) the path comes from xudrone_mapper
instead: one depth (z_top), lanes only where the map still needs bed depth
(target "bed") or camera footprint ("camera"), transits routed around the
costmap. The heading is then a compass heading. If the mapper does not answer
within PLAN_WAIT_S the plain lawnmower is flown.
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
from patterns import depth_layers, eased_duration, eased_polyline, path_length_3d, volume_lawnmower
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done

PLAN_WAIT_S = 10.0


class UdroneCoverageController(UdroneHelixController):
    def __init__(
        self,
        width_m=12.0,
        length_m=12.0,
        swath_m=3.0,
        z_top=-0.9,
        z_bottom=-1.9,
        layers=3,
        speed_mps=0.25,
        center_east_m=None,
        center_north_m=None,
        heading_deg=0.0,
        plan=None,
        target="bed",
        **kwargs,
    ):
        self.cov_plan = (plan or "").strip().lower() or None
        self.cov_target = target or "bed"
        self._plan_id = None
        self._plan_t = 0.0
        self._plan_info = None
        self.cov_width = float(width_m)
        self.cov_length = float(length_m)
        self.cov_swath = float(swath_m)
        self.cov_z_top = float(z_top)
        self.cov_z_bottom = float(z_bottom)
        self.cov_layers = max(1, int(layers))
        self.cov_speed = float(speed_mps)
        self.cov_heading = float(heading_deg)
        self.cov_center_e = None if center_east_m is None else float(center_east_m)
        self.cov_center_n = None if center_north_m is None else float(center_north_m)
        self._wps = []
        self._path_time = 0.0
        self._built = False
        # duration is a hard cap; _finished uses path length.
        depths = depth_layers(self.cov_z_top, self.cov_z_bottom, self.cov_layers)
        rough = volume_lawnmower(self.cov_width, self.cov_length, self.cov_swath, depths,
                                 heading_deg=self.cov_heading)
        rough_t = 2.0 * path_length_3d(rough) / max(self.cov_speed, 0.05) + 30.0
        kwargs.setdefault("duration_s", rough_t)
        # Dummy helix params (unused once _helix is overridden).
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.cov_speed)
        kwargs.setdefault("z0", 0.5 * (self.cov_z_top + self.cov_z_bottom))
        super().__init__(**kwargs)
        self.progress_gate = True
        if self.cov_plan == "map":
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._plan_pub = self.create_publisher(String, "/xudrone/map/plan_coverage", 5)
            self.create_subscription(String, "/xudrone/map/coverage_plan_info", self._on_plan, latched)
        self.get_logger().info(
            f"coverage: {self.cov_width:g}x{self.cov_length:g} swath={self.cov_swath:g} "
            f"layers={self.cov_layers} z=[{self.cov_z_top:g},{self.cov_z_bottom:g}] "
            f"speed={self.cov_speed:g} m/s plan={self.cov_plan or 'lawnmower'}"
        )

    def _on_plan(self, m):
        try:
            info = json.loads(m.data)
        except ValueError:
            return
        if self._plan_id is not None and info.get("id") == self._plan_id:
            self._plan_info = info

    def _map_plan(self):
        """Path-frame waypoints from the mapper, [] while waiting, None to fall back."""
        now = time.time()
        if self._plan_id is None:
            self._plan_id = f"cov-{os.getpid()}-{int(now)}"
            self._plan_t = now
            self._req_t = 0.0
        info = self._plan_info
        if info is None:
            if now - self._plan_t > PLAN_WAIT_S:
                print("COVERAGE_PLAN no answer from xudrone_mapper; flying the lawnmower", flush=True)
                return None
            if now - self._req_t >= 3.0:
                # Resent until answered: the mapper may still be discovering this node.
                self._req_t = now
                req = json.dumps({"id": self._plan_id, **self._plan_request()})
                self._plan_pub.publish(String(data=req))
                print(f"COVERAGE_PLAN_REQUEST {req}", flush=True)
            return []
        if not info.get("ok"):
            print(f"COVERAGE_PLAN empty ({info.get('reason')}); flying the lawnmower", flush=True)
            return None
        wps = [(*self._path_xy(x, y), z) for x, y, z in info["waypoints"]]
        return [(0.0, 0.0, wps[0][2])] + wps

    def _plan_request(self):
        x0, y0, _yaw = self.anchor
        return {"width_m": self.cov_width, "length_m": self.cov_length,
                "heading_deg": self.cov_heading, "swath_m": self.cov_swath, "z": self.cov_z_top,
                "target": self.cov_target, "start": [x0, y0],
                "center_east_m": self.cov_center_e if self.cov_center_e is not None else x0,
                "center_north_m": self.cov_center_n if self.cov_center_n is not None else y0}

    def _events(self, tm: float):
        if self._built or self.anchor is None:
            return
        if self.cov_plan == "map":
            wps = self._map_plan()
            if wps == []:
                return
            if wps is not None:
                self._wps = wps
                self._path_time = eased_duration(wps, self.cov_speed)
                self.duration_s = max(self.duration_s, 2.0 * path_length_3d(wps) / max(self.cov_speed, 0.05) + 30.0)
                self._built = True
                print(
                    f"COVERAGE_PATH map plan {self._plan_id} n={len(wps)} "
                    f"length={path_length_3d(wps):.1f} m time≈{self._path_time:.1f} s "
                    f"lanes={self._plan_info.get('lanes')} target={self.cov_target}",
                    flush=True,
                )
                return
            self.cov_plan = None
        depths = depth_layers(self.cov_z_top, self.cov_z_bottom, self.cov_layers)
        wps = volume_lawnmower(
            self.cov_width, self.cov_length, self.cov_swath, depths,
            heading_deg=self.cov_heading,
        )
        # Optional world ENU centre (e.g. last AprilTag): shift path-frame origin.
        if self.cov_center_e is not None and self.cov_center_n is not None:
            x0, y0, yaw0 = self.anchor
            c, s = math.cos(yaw0), math.sin(yaw0)
            de, dn = self.cov_center_e - x0, self.cov_center_n - y0
            # path-frame: x forward along yaw0, y left
            cx, cy = c * de + s * dn, -s * de + c * dn
            wps = [(x + cx, y + cy, z) for x, y, z in wps]
        self._wps = wps
        self._path_time = eased_duration(wps, self.cov_speed)
        self._built = True
        print(
            f"COVERAGE_PATH n={len(wps)} length={path_length_3d(wps):.1f} m "
            f"time≈{self._path_time:.1f} s depths={depths}",
            flush=True,
        )

    def _helix(self, tm: float):
        if not self._wps:
            # Before path is built: hold current depth.
            return (0.0, 0.0, self.cov_z_top), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        return eased_polyline(self._wps, self.cov_speed, tm)

    def _finished(self, tm: float) -> bool:
        if not self._built:
            return False
        # Finish a few seconds after the end of the polyline (hold last point).
        # The reference clock waits for the hull, so the cap is on thrust time.
        return self.path_t >= self._path_time + 3.0 or tm >= self.duration_s


def main():
    width = float(os.environ.get("UDRONE_COV_WIDTH", "12"))
    length = float(os.environ.get("UDRONE_COV_LENGTH", os.environ.get("UDRONE_COV_WIDTH", "12")))
    swath = float(os.environ.get("UDRONE_COV_SWATH", "3"))
    z_top = float(os.environ.get("UDRONE_COV_Z_TOP", "-0.9"))
    z_bottom = float(os.environ.get("UDRONE_COV_Z_BOTTOM", "-1.9"))
    layers = int(float(os.environ.get("UDRONE_COV_LAYERS", "3")))
    speed = float(os.environ.get("UDRONE_COV_SPEED", "0.25"))
    heading = float(os.environ.get("UDRONE_COV_HEADING", "0"))
    ce = os.environ.get("UDRONE_COV_CENTER_E")
    cn = os.environ.get("UDRONE_COV_CENTER_N")
    center_e = float(ce) if ce not in (None, "") else None
    center_n = float(cn) if cn not in (None, "") else None

    rclpy.init()
    node = UdroneCoverageController(
        width_m=width, length_m=length, swath_m=swath,
        z_top=z_top, z_bottom=z_bottom, layers=layers, speed_mps=speed,
        heading_deg=heading, center_east_m=center_e, center_north_m=center_n,
        plan=os.environ.get("UDRONE_COV_PLAN"), target=os.environ.get("UDRONE_COV_TARGET", "bed"),
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_coverage_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            else:
                ok = True
                why = "ok"
        print(f"COVERAGE_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
