#!/usr/bin/env python3
"""Put xudrone back under otter_k and latch it, without restarting the simulator.

A simulator shortcut, not a docking manoeuvre: the AUV is teleported to just
below the nest with Gazebo's set_pose service, then ATTACH pulls it in. Use it
between test runs (the undock test leaves the AUV free). Disarm the AUV first.

Run: run_test.sh redock      Exit status: 0 docked, 1 not.
"""
import sys
import time

from latch_test import Rig


def main():
    rig = Rig()
    try:
        rig.poses()
        rig.wait_state("attached", 2.5)
        if rig.latch.state != "attached":
            rig.place((0.0, 0.0, -0.2))
            rc, out = rig.dock("ATTACH")
            print(f"ATTACH -> {out}", flush=True)
            time.sleep(1.0)
        d, ang, _rel = rig.offset()
        ok = rig.latch.state == "attached" and d < 0.01
        print(f"latch {rig.latch.state}, {d * 100:.1f} cm / {ang:.1f} deg from the nest", flush=True)
        return 0 if ok else 1
    finally:
        rig.stop()


if __name__ == "__main__":
    sys.exit(main())
