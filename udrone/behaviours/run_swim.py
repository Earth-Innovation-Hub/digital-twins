#!/usr/bin/env python3
"""AUV straight leg: one segment on the helix force loop.

Same force → point-hull control as the helix and coverage swims. The path is
two points in the thrust-on frame: the arm pose, then the compass or
heading offset. Depth stays at the depth when thrust starts unless depth_m
is set. Coral command type ``swim``.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from patterns import path_length_3d, polyline_pos_vel, swim_path_end
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done


def _env_float(name, default=None):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return default
    return float(raw)


class UdroneSwimController(UdroneHelixController):
    def __init__(
        self,
        north_m=0.0,
        east_m=0.0,
        along_heading_m=None,
        relative_to="current",
        depth_m=None,
        speed_mps=0.20,
        arrive_m=1.0,
        **kwargs,
    ):
        self.swim_north = float(north_m)
        self.swim_east = float(east_m)
        self.swim_along = None if along_heading_m is None else float(along_heading_m)
        self.swim_rel = str(relative_to or "current").lower()
        self.swim_depth = None if depth_m is None else float(depth_m)
        self.swim_speed = float(speed_mps)
        self.swim_arrive = float(arrive_m)
        self._wps = []
        self._path_time = 0.0
        self._built = False
        self._target_world = None
        self._end_path = (0.0, 0.0)
        self._miss = float("inf")
        # Hard cap. _finished uses path length and the arrival radius.
        kwargs.setdefault("duration_s", 60.0 / max(self.swim_speed, 0.05) + 40.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.swim_speed)
        kwargs.setdefault("z0", self.swim_depth if self.swim_depth is not None else -1.2)
        super().__init__(**kwargs)
        self.get_logger().info(
            f"swim: rel={self.swim_rel} north={self.swim_north:g} east={self.swim_east:g} "
            f"along={self.swim_along} depth={self.swim_depth} speed={self.swim_speed:g} "
            f"arrive={self.swim_arrive:g}"
        )

    def _events(self, tm: float):
        if self._built or self.anchor is None:
            return
        x0, y0, yaw0 = self.anchor
        z_now = self.swim_depth if self.swim_depth is not None else -1.2
        fresh, _, pair = self.gz.read(self.gz_model, self.boat_model)
        auv = pair[0] if pair else None
        if fresh and auv is not None:
            z_now = float(auv[0][2])
        z_des = self.swim_depth if self.swim_depth is not None else z_now
        fwd, left = swim_path_end(
            yaw0, self.swim_rel,
            north_m=self.swim_north, east_m=self.swim_east,
            along_heading_m=self.swim_along, x0=x0, y0=y0,
        )
        self._end_path = (fwd, left)
        c, s = math.cos(yaw0), math.sin(yaw0)
        self._target_world = (x0 + c * fwd - s * left, y0 + s * fwd + c * left, z_des)
        self._wps = [(0.0, 0.0, z_now), (fwd, left, z_des)]
        length = path_length_3d(self._wps)
        self._path_time = length / max(self.swim_speed, 0.05)
        self.duration_s = self._path_time + 40.0
        self._built = True
        print(
            f"SWIM_PATH len={length:.1f} m time≈{self._path_time:.1f} s "
            f"end_path=({fwd:.2f},{left:.2f}) "
            f"end_enu=({self._target_world[0]:.2f},{self._target_world[1]:.2f},{z_des:.2f}) "
            f"yaw0={yaw0:.2f}",
            flush=True,
        )

    def _helix(self, tm: float):
        if not self._wps:
            z = self.swim_depth if self.swim_depth is not None else -1.2
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        return polyline_pos_vel(self._wps, self.swim_speed, tm)

    def _close(self) -> bool:
        if self._target_world is None or self._prev is None:
            return False
        pos = self._prev[1]
        self._miss = math.dist(pos, self._target_world)
        return self._miss <= self.swim_arrive

    def _finished(self, tm: float) -> bool:
        if not self._built:
            return False
        if tm > 1.0 and self._close():
            print(f"SWIM_ARRIVE miss={self._miss:.2f} m t={tm:.1f}", flush=True)
            return True
        if tm >= self._path_time + 15.0:
            if not self._close():
                self._fail(f"swim missed the end by {self._miss:.1f} m")
            return True
        return False


def main():
    north = _env_float("UDRONE_SWIM_NORTH", 0.0)
    east = _env_float("UDRONE_SWIM_EAST", 0.0)
    along = _env_float("UDRONE_SWIM_ALONG", None)
    depth = _env_float("UDRONE_SWIM_DEPTH", None)
    speed = _env_float("UDRONE_SWIM_SPEED", 0.20)
    arrive = _env_float("UDRONE_SWIM_ARRIVE", 1.0)
    rel = os.environ.get("UDRONE_SWIM_REL", "current")

    rclpy.init()
    node = UdroneSwimController(
        north_m=north, east_m=east, along_heading_m=along,
        relative_to=rel, depth_m=depth, speed_mps=speed, arrive_m=arrive,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_swim_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            elif node._miss > max(node.swim_arrive, 1.5):
                why = f"ended {node._miss:.1f} m from the target"
            else:
                ok = True
                why = f"ok miss={node._miss:.2f} m"
        print(f"SWIM_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
