#!/usr/bin/env python3
"""AUV straight leg from the arm pose to a compass or heading offset.

Guidance is line of sight by default (los_guidance.LosGuide): the hull turns
onto the line, holds a surge speed that ramps up and brakes onto the end
point, and steers heading and pitch by the cross-track and depth error. There
is no time-indexed reference to fall behind. UDRONE_SWIM_GUIDANCE=track flies
the older path: a trapezoid-timed point on the helix force loop with the
progress gate. Depth stays at the depth when thrust starts unless depth_m is
set. Coral command type ``swim``.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from los_guidance import LosGuide
from patterns import path_length_3d, swim_path_end, trapezoid, trapezoid_time
from shape_geom import cruise_speed
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done

ACCEL_MPS2 = 1.5
# LOS stops on the line, where pitch no longer moves the hull up or down. If
# it stops outside the arrival radius, the force loop holds the end point for
# up to SETTLE_S to close the rest.
SETTLE_REM_M = 0.3
SETTLE_SPEED_MPS = 0.1
SETTLE_AFTER_S = 1.5
SETTLE_S = 10.0


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
        arrive_m=0.35,
        guidance="los",
        **kwargs,
    ):
        self.swim_north = float(north_m)
        self.swim_east = float(east_m)
        self.swim_along = None if along_heading_m is None else float(along_heading_m)
        self.swim_rel = str(relative_to or "current").lower()
        self.swim_depth = None if depth_m is None else float(depth_m)
        self.swim_speed = float(speed_mps)
        self.swim_arrive = float(arrive_m)
        self.guidance = "track" if str(guidance).lower() == "track" else "los"
        self._wps = []
        self._path_time = 0.0
        self._built = False
        self._target_world = None
        self._end_path = (0.0, 0.0)
        self._miss = float("inf")
        self._stopped_t = None
        self._settle_t = None
        # Hard cap. _finished uses path length and the arrival radius.
        kwargs.setdefault("duration_s", 60.0 / max(self.swim_speed, 0.05) + 40.0)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", self.swim_speed)
        kwargs.setdefault("z0", self.swim_depth if self.swim_depth is not None else -1.2)
        super().__init__(**kwargs)
        self.progress_gate = self.guidance == "track"
        self.hud_preview = self.guidance == "track"
        self._length = 0.0
        self.trace_extra_cols = self.LOS_TRACE_COLUMNS
        self.trace_meta = {
            "guidance": self.guidance, "rel": self.swim_rel, "north": self.swim_north,
            "east": self.swim_east, "along": self.swim_along, "depth": self.swim_depth,
            "speed": self.swim_speed, "arrive": self.swim_arrive,
        }
        self.get_logger().info(
            f"swim: rel={self.swim_rel} north={self.swim_north:g} east={self.swim_east:g} "
            f"along={self.swim_along} depth={self.swim_depth} speed={self.swim_speed:g} "
            f"arrive={self.swim_arrive:g} guidance={self.guidance}"
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
        self._length = length
        self._path_time = trapezoid_time(length, max(self.swim_speed, 0.05), ACCEL_MPS2)
        if self.guidance == "los":
            try:
                self.los = LosGuide(
                    [(x0, y0, z_now), self._target_world],
                    cruise_mps=min(self.swim_speed, cruise_speed()),
                    pitch_max_rad=self.pitch_limit or math.radians(45.0),
                )
                self._path_time = self.los.eta()
            except ValueError as exc:
                print(f"SWIM_LOS off ({exc}); flying the timed path", flush=True)
                self.guidance = "track"
                self.progress_gate = True
                self.hud_preview = True
        self.duration_s = 2.0 * self._path_time + 30.0
        self._built = True
        print(
            f"SWIM_PATH len={length:.1f} m time≈{self._path_time:.1f} s guidance={self.guidance} "
            f"end_path=({fwd:.2f},{left:.2f}) "
            f"end_enu=({self._target_world[0]:.2f},{self._target_world[1]:.2f},{z_des:.2f}) "
            f"yaw0={yaw0:.2f}",
            flush=True,
        )
        self.trace_event(
            "path", guidance=self.guidance, length=length, eta=self._path_time,
            start=(x0, y0, z_now), end=self._target_world, yaw0=yaw0,
            cruise=self.los.cruise if self.los is not None else self.swim_speed,
        )

    def _control(self, tm, auv, boat):
        if self.los is not None:
            return self._control_los(tm, auv, boat)
        return super()._control(tm, auv, boat)

    def _hud_layers(self):
        out = super()._hud_layers()
        if self.los is not None and self._hold_point is None:
            out["paths"].append({"pts": list(self.los.pts), "color": (255, 0, 255), "width": 2, "label": "LOS"})
            if self._p_des is not None:
                out["markers"].append({"pos": self._p_des, "label": "aim", "color": (255, 0, 255)})
        return out

    def _check_settle(self, tm: float):
        g = self._los_g
        if self.los is None or g is None:
            return
        stopped = abs(g["rem"]) < SETTLE_REM_M and abs(g["u_ref"]) < SETTLE_SPEED_MPS
        if not stopped:
            self._stopped_t = None
            return
        if self._stopped_t is None:
            self._stopped_t = tm
        if tm - self._stopped_t < SETTLE_AFTER_S:
            return
        print(f"SWIM_SETTLE t={tm:.1f} miss={self._miss:.2f} m xte={g['xte']:+.2f} vte={g['vte']:+.2f}; "
              f"holding the end point on the force loop", flush=True)
        self.trace_event("settle", miss=self._miss, xte=g["xte"], vte=g["vte"])
        self.los = None
        self._los_g = None
        self._settle_t = tm
        self.trace_mode = "settle"
        self.progress_gate = False

    def _helix(self, tm: float):
        if self._settle_t is not None:
            return (self._end_path[0], self._end_path[1], self._wps[1][2]), (0.0, 0.0, 0.0)
        if not self._wps:
            z = self.swim_depth if self.swim_depth is not None else -1.2
            return (0.0, 0.0, z), (0.0, 0.0, 0.0)
        a, b = self._wps
        if self._length < 1e-6:
            return tuple(b), (0.0, 0.0, 0.0)
        u = tuple((b[i] - a[i]) / self._length for i in range(3))
        s, v, acc = trapezoid(self._length, max(self.swim_speed, 0.05), ACCEL_MPS2, tm)
        return (
            tuple(a[i] + u[i] * s for i in range(3)),
            tuple(u[i] * v for i in range(3)),
            tuple(u[i] * acc for i in range(3)),
        )

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
            print(f"SWIM_ARRIVE miss={self._miss:.2f} m t={tm:.1f} path_t={self.path_t:.1f}", flush=True)
            self.trace_event("arrive", miss=self._miss, settled=self._settle_t is not None)
            return True
        self._check_settle(tm)
        timed_out = tm >= self.duration_s
        if self._settle_t is not None:
            timed_out = timed_out or tm - self._settle_t >= SETTLE_S
        elif self.guidance == "track":
            timed_out = timed_out or self.path_t >= self._path_time + 15.0
        if timed_out:
            if not self._close():
                self._fail(f"swim missed the end by {self._miss:.1f} m")
            return True
        return False


def main():
    north = _env_float("UDRONE_SWIM_NORTH", 0.0)
    east = _env_float("UDRONE_SWIM_EAST", 0.0)
    along = _env_float("UDRONE_SWIM_ALONG", None)
    depth = _env_float("UDRONE_SWIM_DEPTH", None)
    speed = _env_float("UDRONE_SWIM_SPEED", cruise_speed())
    arrive = _env_float("UDRONE_SWIM_ARRIVE", 0.35)
    rel = os.environ.get("UDRONE_SWIM_REL", "current")
    guidance = os.environ.get("UDRONE_SWIM_GUIDANCE", "") or "los"

    rclpy.init()
    node = UdroneSwimController(
        north_m=north, east_m=east, along_heading_m=along,
        relative_to=rel, depth_m=depth, speed_mps=speed, arrive_m=arrive,
        guidance=guidance,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            elif node._miss > node.swim_arrive:
                why = f"ended {node._miss:.1f} m from the target"
            else:
                ok = True
                why = f"ok miss={node._miss:.2f} m"
        node.finish_trace("pass" if ok else "fail", why=why, miss=node._miss,
                          guidance=node.guidance, length=node._length, eta=node._path_time)
        node.write_log("/tmp/coral_swim_ctrl_log.txt")
        print(f"SWIM_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
