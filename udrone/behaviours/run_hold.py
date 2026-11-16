#!/usr/bin/env python3
"""Hold xudrone where the last behaviour left it, armed, until the next one.

A finished behaviour starts this runner and keeps flying until it takes the
baton. The next behaviour takes the baton from here, so a plan of several
steps arms once. After UDRONE_HOLD_MAX_S of holding with nothing to follow it
disarms. SIGTERM (abort, hold, reset, dock) also disarms.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done

HOLD_MAX_S = 900.0


def _env_float(name, default):
    raw = os.environ.get(name, "")
    return default if raw == "" else float(raw)


class UdroneHold(UdroneHelixController):
    def __init__(self, **kwargs):
        self.hold_max = _env_float("UDRONE_HOLD_MAX_S", HOLD_MAX_S)
        kwargs.setdefault("duration_s", self.hold_max)
        kwargs.setdefault("radius", 0.1)
        kwargs.setdefault("omega", 0.1)
        kwargs.setdefault("x_rate", 0.0)
        kwargs.setdefault("z0", -1.0)
        super().__init__(**kwargs)
        self.keep_armed = False
        self.offers_baton = True
        self.baton_only = True
        point = tuple(_env_float(k, float("nan")) for k in ("UDRONE_HOLD_X", "UDRONE_HOLD_Y", "UDRONE_HOLD_Z"))
        self._target = None if any(v != v for v in point) else point
        yaw = _env_float("UDRONE_HOLD_YAW", float("nan"))
        self._head = None if yaw != yaw else yaw
        self.get_logger().info(f"hold: point={self._target} max={self.hold_max:g}s")

    def _helix(self, tm: float):
        if self._target is None and self._prev is not None:
            self._target = tuple(self._prev[1])
        if self._target is None or self.anchor is None:
            return (0.0, 0.0, self.z0), (0.0, 0.0, 0.0)
        self._hold_point = self._target
        return self._hold_ref()

    def _finished(self, tm: float) -> bool:
        return tm >= self.hold_max


def main():
    rclpy.init()
    node = UdroneHold()
    try:
        spin_until_done(node)
    finally:
        print(f"HOLD_END {node.failed or 'handed on or timed out'}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
