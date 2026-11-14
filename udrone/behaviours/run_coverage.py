#!/usr/bin/env python3
"""AUV volume coverage: 3D boustrophedon at several depths.

Same force → point-hull control loop as the helix swim; only pos_des(t) changes.
Coral command type ``coverage``. Path is built in the thrust-on path frame
(centred on the AUV, or shifted to a world ENU centre if given — e.g. last tag).
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from patterns import depth_layers, path_length_3d, polyline_pos_vel, volume_lawnmower
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done


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
        **kwargs,
    ):
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
        rough_t = path_length_3d(rough) / max(self.cov_speed, 0.05) + 30.0
        kwargs.setdefault("duration_s", rough_t)
        # Dummy helix params (unused once _helix is overridden).
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.cov_speed)
        kwargs.setdefault("z0", 0.5 * (self.cov_z_top + self.cov_z_bottom))
        super().__init__(**kwargs)
        self.get_logger().info(
            f"coverage: {self.cov_width:g}x{self.cov_length:g} swath={self.cov_swath:g} "
            f"layers={self.cov_layers} z=[{self.cov_z_top:g},{self.cov_z_bottom:g}] "
            f"speed={self.cov_speed:g} m/s"
        )

    def _events(self, tm: float):
        if self._built or self.anchor is None:
            return
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
        self._path_time = path_length_3d(wps) / max(self.cov_speed, 0.05)
        self._built = True
        print(
            f"COVERAGE_PATH n={len(wps)} length={path_length_3d(wps):.1f} m "
            f"time≈{self._path_time:.1f} s depths={depths}",
            flush=True,
        )

    def _helix(self, tm: float):
        if not self._wps:
            # Before path is built: hold current depth, creep forward slowly.
            return (0.0, 0.0, self.cov_z_top), (0.0, 0.0, 0.0)
        return polyline_pos_vel(self._wps, self.cov_speed, tm)

    def _finished(self, tm: float) -> bool:
        if not self._built:
            return False
        # Finish a few seconds after the end of the polyline (hold last point).
        return tm >= self._path_time + 3.0


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
