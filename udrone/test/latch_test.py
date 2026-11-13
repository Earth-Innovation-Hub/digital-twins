#!/usr/bin/env python3
"""CoRAL dock latch test (no thrust, both vehicles stay disarmed).

Walks the latch through every transition and gate, moving the AUV with Gazebo's
set_pose service:
  1. docked at boot, seated at the nest pose
  2. DETACH releases; the AUV is free (it can be moved away from the boat)
  3. ATTACH refused when the AUV is farther than the distance gate
  4. ATTACH refused when it is close but rotated past the angle gate
  5. ATTACH accepted close and aligned, and the AUV is pulled into the nest
  6. the gz.msgs.Empty detach / attach topics work too
Leaves the pair docked.

Run: run_test.sh latch      Exit status: 0 pass, 1 fail.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from udrone_sinusoidal_controller import GZ_ENV, GzPoses, LatchState, rotate_inverse

WORLD = "kiwanis_lake"
BOAT = "heron_0"
AUV = os.environ.get("UDRONE_GZ_MODEL", "xudrone_1")
DOCK = "/usr/local/bin/dock_command.py"
NEST_TOL_M = 0.01
NEST_TOL_DEG = 1.0

results = []


def check(name, ok, detail):
    results.append((name, bool(ok), detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate(q, v):
    """v (boat frame) into the world frame."""
    w, x, y, z = q
    r = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


class Rig:
    def __init__(self):
        self.gz = GzPoses(WORLD)
        self.latch = LatchState()

    def poses(self, timeout=15.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            fresh, _t, (b, a) = self.gz.read(BOAT, AUV)
            if fresh and b is not None and a is not None:
                return b, a
            time.sleep(0.1)
        raise RuntimeError(f"no Gazebo pose for {BOAT} / {AUV}; is the CoRAL pair running?")

    def offset(self):
        """(distance m, angle deg, offset xyz) of the AUV from the nest, in the boat frame."""
        (bp, bq), (ap, aq) = self.poses()
        rel = rotate_inverse(bq, tuple(a - b for a, b in zip(ap, bp)))
        dot = min(1.0, abs(sum(p * q for p, q in zip(aq, bq))))
        return math.dist(rel, (0, 0, 0)), math.degrees(2 * math.acos(dot)), rel

    def wait_state(self, want, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.latch.state == want:
                return True
            time.sleep(0.1)
        return False

    def place(self, rel_xyz, yaw_deg=0.0):
        """Teleport the AUV to an offset from the nest, in the boat frame."""
        (bp, bq), _ = self.poses()
        off = rotate(bq, rel_xyz)
        h = math.radians(yaw_deg) / 2.0
        q = qmul(bq, (math.cos(h), 0.0, 0.0, math.sin(h)))
        req = (
            f'name: "{AUV}", position: {{x: {bp[0] + off[0]:.4f}, y: {bp[1] + off[1]:.4f}, z: {bp[2] + off[2]:.4f}}}, '
            f"orientation: {{w: {q[0]:.6f}, x: {q[1]:.6f}, y: {q[2]:.6f}, z: {q[3]:.6f}}}"
        )
        r = subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
            capture_output=True, text=True, timeout=15, env=GZ_ENV,
        )
        time.sleep(0.4)
        return "true" in r.stdout

    def dock(self, cmd):
        r = subprocess.run([sys.executable, DOCK, cmd], capture_output=True, text=True, timeout=30, env=GZ_ENV)
        return r.returncode, r.stdout.strip()

    def empty(self, which):
        subprocess.run(
            ["gz", "topic", "-t", f"/model/{BOAT}/udrone_dock/{which}", "-m", "gz.msgs.Empty", "-p", " "],
            capture_output=True, text=True, timeout=10, env=GZ_ENV,
        )

    def stop(self):
        self.gz.stop()
        self.latch.stop()


def seated(rig, label):
    time.sleep(1.0)
    d, ang, rel = rig.offset()
    check(label, d < NEST_TOL_M and ang < NEST_TOL_DEG,
          f"{d * 100:.1f} cm, {ang:.1f} deg from the nest (offset {rel[0]:.3f},{rel[1]:.3f},{rel[2]:.3f})")


def main():
    rig = Rig()
    try:
        rig.poses()
        check("latch state published", rig.wait_state("attached", 5.0) or rig.latch.state is not None,
              f"state {rig.latch.state}")
        if rig.latch.state != "attached":
            rc, out = rig.dock("ATTACH")
            print(f"  (was not docked; ATTACH -> {out})", flush=True)
        check("docked", rig.latch.state == "attached", f"state {rig.latch.state}")
        seated(rig, "seated at the nest while docked")

        rc, out = rig.dock("DETACH")
        check("DETACH", rc == 0 and out == "detached", f"dock_command -> {out} (rc {rc})")
        moved = rig.place((2.0, 0.0, -0.5))
        time.sleep(1.0)
        d, _a, _r = rig.offset()
        check("free after DETACH", moved and d > 1.5, f"AUV now {d:.2f} m from the nest")

        rc, out = rig.dock("ATTACH")
        d, _a, _r = rig.offset()
        check("ATTACH refused out of range", rc != 0 and out == "detached" and d > 1.5,
              f"dock_command -> {out} (rc {rc}), AUV still {d:.2f} m away")

        rig.place((0.0, 0.0, -0.3), yaw_deg=45.0)
        d0, a0, _r = rig.offset()
        rc, out = rig.dock("ATTACH")
        check("ATTACH refused when misaligned", rc != 0 and out == "detached",
              f"at {d0:.2f} m, {a0:.0f} deg: dock_command -> {out} (rc {rc})")

        rig.place((0.0, 0.0, -0.3), yaw_deg=10.0)
        d0, a0, _r = rig.offset()
        rc, out = rig.dock("ATTACH")
        check("ATTACH accepted in range", rc == 0 and out == "attached",
              f"from {d0:.2f} m, {a0:.0f} deg: dock_command -> {out} (rc {rc})")
        seated(rig, "pulled into the nest on ATTACH")

        rig.empty("detach")
        check("detach topic (Empty)", rig.wait_state("detached"), f"state {rig.latch.state}")
        rig.place((0.0, 0.0, -0.25))
        rig.empty("attach")
        check("attach topic (Empty)", rig.wait_state("attached"), f"state {rig.latch.state}")
        seated(rig, "seated after re-attach")
    except Exception as e:  # noqa: BLE001
        check("test ran", False, str(e))
    finally:
        if rig.latch.state != "attached":
            rig.place((0.0, 0.0, -0.2))
            rig.dock("ATTACH")
            print(f"  (left the pair {rig.latch.state})", flush=True)
        rig.stop()
    ok = all(p for _, p, _ in results)
    print("VERDICT " + ("PASS" if ok else "FAIL"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
