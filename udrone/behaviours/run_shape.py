#!/usr/bin/env python3
"""Track one geometric shape on the xudrone force loop.

The reference is an orbit, a vertical loop, or a figure-eight, with the
curve's acceleration fed forward. Same hull-axis tracker as the helix. A
glide takes the hull from its arm depth down to the shape first, and the
reference clock waits for the hull when it falls behind. The verdict is the
tracking error, not just whether the thrusters ran.
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))

import rclpy
import shape_geom
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done, tracking_error


def shape_limits(radius_m: float):
    """(mean, max) metres of 3D error a flown shape may show."""
    return max(0.35, 0.12 * radius_m), max(1.5, 0.5 * radius_m)


class ShapeController(UdroneHelixController):
    def __init__(self, spec: dict):
        super().__init__(
            duration_s=spec["timeout_s"],
            radius=spec["radius_m"],
            omega=spec["omega"],
            x_rate=0.0,
            z0=spec["z0"],
        )
        self.spec = spec
        self.pitch_limit = None
        self.progress_gate = True
        self.z_arm = None
        self.lead_s = 0.0

    def _helix(self, tm: float):
        if self.z_arm is None:
            self.z_arm = self._prev[1][2] if self._prev else self.spec["z0"]
            _run, self.lead_s = shape_geom.lead_in(self.z_arm, self.spec["z0"])
            # The gate can slow the reference, so the wall-clock cap is generous.
            self.duration_s = 2.0 * (self.lead_s + self.spec["timeout_s"]) + 20.0
            print(f"SHAPE_LEAD from z={self.z_arm:.2f} glide={self.lead_s:.1f}s", flush=True)
        return shape_geom.sample_lead(self.spec, tm, self.z_arm)

    def _finished(self, tm: float) -> bool:
        if self.z_arm is not None:
            tau, _, _ = shape_geom._warp(self.path_t - self.lead_s)
            if self.path_t > self.lead_s and tau >= self.spec["tau_end"]:
                return True
        if tm >= self.duration_s:
            self._fail(f"shape not finished in {self.duration_s:.0f}s (reference at {self.path_t:.0f}s)")
            return True
        return False


def main():
    spec = shape_geom.plan(
        os.environ.get("UDRONE_SHAPE", "infinity"),
        float(os.environ.get("UDRONE_SHAPE_SIZE", "3")),
        float(os.environ.get("UDRONE_SHAPE_LOOPS", "2")),
        float(os.environ.get("UDRONE_SHAPE_Z", str(shape_geom.Z0))),
        None if os.environ.get("UDRONE_SHAPE_SPEED") in (None, "")
        else float(os.environ["UDRONE_SHAPE_SPEED"]),
    )
    capped = " (held to the shape's limit)" if spec["pace_capped"] else ""
    print(
        f"SHAPE {spec['shape']} size={spec['size_m']:.2f} m "
        f"flown_radius={spec['radius_m']:.2f} m loops={spec['loops']:.2f} "
        f"speed={spec['speed_mps']:.2f} m/s{capped} limit={spec['pace_limit']:.2f} m/s "
        f"timeout={spec['timeout_s']:.0f} s",
        flush=True,
    )
    rclpy.init()
    node = ShapeController(spec)
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_shape_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            mean_lim, max_lim = shape_limits(spec["radius_m"])
            e_mean, e_max = tracking_error(node.log, node.lead_s + shape_geom.RAMP_S)
            span = node.log[-1][0] if node.log else 0.0
            pace = node.path_t / span if span > 0 else 0.0
            print(
                f"SHAPE_TRACK mean={e_mean:.2f} m (limit {mean_lim:.2f}) "
                f"max={e_max:.2f} m (limit {max_lim:.2f}) reference ran at {pace:.0%} of real time",
                flush=True,
            )
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            elif not (e_mean <= mean_lim and e_max <= max_lim):
                why = f"shape not tracked (mean {e_mean:.2f} m, max {e_max:.2f} m)"
            else:
                ok = True
                why = f"ok mean {e_mean:.2f} m"
        print(f"SHAPE_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
