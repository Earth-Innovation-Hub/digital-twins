#!/usr/bin/env python3
"""Runtime helix swim for the CoRAL behaviour library.

Same controller as the helix test, but exit status is only about completing a
commanded swim (armed, thrusters ran, finished without disarm/fault) — not the
test's mean tracking MAE gate. Used by coral_behaviours when type=helix.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))

import rclpy
from udrone_sinusoidal_controller import UdroneHelixController, spin_until_done


def main():
    duration = float(os.environ.get("UDRONE_HELIX_DUR", "45"))
    radius = float(os.environ.get("UDRONE_HELIX_R", "0.5"))
    omega = float(os.environ.get("UDRONE_HELIX_OMEGA", "0.40"))
    x_rate = float(os.environ.get("UDRONE_HELIX_VX", "0.20"))
    z0 = float(os.environ.get("UDRONE_HELIX_Z0", "-1.2"))
    rclpy.init()
    node = UdroneHelixController(
        duration_s=duration, radius=radius, omega=omega, x_rate=x_rate, z0=z0,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_helix_ctrl_log.txt")
        why = node.failed or "no samples"
        if node.armed_ok and node.log and not node.failed:
            rotor = [r[18] for r in node.log]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            if mean_rotor < 0.2:
                why = "thrusters were not driven"
            else:
                ok = True
                why = "ok"
        print(f"HELIX_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
