#!/usr/bin/env python3
"""CoRAL undock test, built on the helix (sinusoid) controller.

Timeline, in simulation seconds after thrust starts:
  0 .. UNDOCK_AT        helix controller runs while xudrone is latched under otter_k
  UNDOCK_AT             DETACH on /udrone_dock/command
  .. + POST_UNDOCK      the same controller keeps running, now free-swimming

The run passes only if every one of these held:
  - the AUV armed in offboard and started docked
  - Gazebo thrusters received non-zero commands (PX4 -> adapter -> Gazebo)
  - while latched, the AUV did not move in the boat's frame
  - the latch reported "detached" right after the command
  - after release the AUV left the boat, dived below the depth it floats at
    without thrust, and was back on the helix by the end of the run

Env:
  UDRONE_UNDOCK_AT     seconds of docked thrust before DETACH (default 10)
  UDRONE_POST_UNDOCK   seconds of free swimming after it (default 10)
  UDRONE_HELIX_R/OMEGA/VX/Z0   helix shape (defaults 0.4, 0.40, 0.20, -1.0)
  UDRONE_GZ_MODEL / UDRONE_PX4_INSTANCE   override the detected xudrone

Run on the aquatic twin:  run_test.sh undock
Log: /tmp/udrone_undock_helix_log.txt   Exit status: 0 pass, 1 fail.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from udrone_sinusoidal_controller import GZ_ENV, UdroneHelixController, spin_until_done, tracking_error

# Depth of the hull centre when the AUV floats with no thrust (awash).
PASSIVE_FLOAT_Z = -0.11
END_WINDOW_S = 2.0       # the AUV must be back on the helix over the last seconds
END_ERROR_M = 0.5
LOG_PATH = "/tmp/udrone_undock_helix_log.txt"


def dock_command(cmd: str) -> None:
    r = subprocess.run(
        ["gz", "topic", "-t", "/udrone_dock/command", "-m", "gz.msgs.StringMsg",
         "-p", f'data: "{cmd.upper()}"'],
        capture_output=True, text=True, timeout=10, env=GZ_ENV,
    )
    print(f"DOCK_{cmd.upper()} sent rc={r.returncode} {(r.stderr or '').strip()}", flush=True)


class UndockHelixController(UdroneHelixController):
    def __init__(self, undock_at_s=10.0, post_undock_s=10.0, **kw):
        super().__init__(duration_s=float(undock_at_s) + float(post_undock_s), **kw)
        self.undock_at_s = float(undock_at_s)
        self.post_undock_s = float(post_undock_s)
        self.latch_at_start = None
        self.t_undock = None         # sim s of thrust when DETACH was sent
        self.t_detached = None       # sim s of thrust when the latch reported it
        self.get_logger().info(
            f"undock test: {self.undock_at_s} s docked, DETACH, {self.post_undock_s} s free"
        )

    def _events(self, tm: float):
        if self.latch_at_start is None:
            self.latch_at_start = self.latch.state or "unknown"
            print(f"LATCH_AT_THRUST_ON {self.latch_at_start}", flush=True)
        if self.t_undock is None and tm >= self.undock_at_s:
            self.t_undock = tm
            print(f"UNDOCK_NOW t_thrust={tm:.2f}s; controller continues {self.post_undock_s}s free", flush=True)
            # gz's command-line publisher takes about a second; keep the
            # control loop running meanwhile.
            threading.Thread(target=dock_command, args=("DETACH",), daemon=True).start()
        if self.t_undock is not None and self.t_detached is None and self.latch.state == "detached":
            self.t_detached = tm
            print(f"LATCH_DETACHED t_thrust={tm:.2f}s", flush=True)


def analyze(node: UndockHelixController):
    """[(check, passed, detail)] from the logged run."""
    out = []

    def add(name, ok, detail):
        out.append((name, bool(ok), detail))

    add("armed in offboard", node.armed_ok, "thrust phase reached" if node.armed_ok else (node.failed or "never armed"))
    add("docked at thrust-on", node.latch_at_start == "attached", f"latch {node.latch_at_start}")
    rows = node.log
    if not rows or node.t_undock is None:
        add("run reached the undock", False, f"{len(rows)} control samples, undock {'not ' if node.t_undock is None else ''}sent")
        return out

    T, X, Y, Z, RX, RY, RZ, ROTOR = 0, 1, 2, 3, 15, 16, 17, 18
    t_free = node.t_detached if node.t_detached is not None else node.t_undock
    docked = [r for r in rows if r[T] < node.t_undock]
    free = [r for r in rows if r[T] >= t_free]
    have_boat = docked and not math.isnan(docked[0][RX])

    def mean_abs(rs, col):
        return sum(abs(r[col]) for r in rs) / max(1, len(rs))

    # Thrust chain. Skip the first second: the adapter low-passes the command.
    d_thr = mean_abs([r for r in docked if r[T] > 1.0], ROTOR)
    f_thr = mean_abs(free, ROTOR)
    add("thrusters driven while docked", d_thr > 0.2, f"rotor 0 mean |{d_thr:.1f}| N")
    add("thrusters driven while free", f_thr > 0.2, f"rotor 0 mean |{f_thr:.1f}| N")

    if have_boat:
        r0 = docked[0]
        dev = max(math.dist((r[RX], r[RY], r[RZ]), (r0[RX], r0[RY], r0[RZ])) for r in docked)
        add("rigid in the boat frame while latched", dev < 0.02,
            f"max shift {dev * 100:.1f} cm from nest offset ({r0[RX]:.3f},{r0[RY]:.3f},{r0[RZ]:.3f})")
        tow = math.hypot(docked[-1][12] - docked[0][12], docked[-1][13] - docked[0][13])
        add("(info) boat towed by the docked AUV", True, f"{tow:.2f} m in {node.undock_at_s:.0f} s")
    else:
        add("rigid in the boat frame while latched", False, "no boat pose in the log")

    lag = None if node.t_detached is None else node.t_detached - node.t_undock
    add("latch reported detached", lag is not None and lag < 2.0,
        "never" if lag is None else f"{lag:.2f} s after the command")
    add("DETACH on time", abs(node.t_undock - node.undock_at_s) < 0.3, f"sent at {node.t_undock:.2f} s of thrust")
    total = rows[-1][T]
    want = node.undock_at_s + node.post_undock_s
    add("controller ran through the release", total >= want - 0.3 and not node.failed,
        f"{total:.1f} of {want:.1f} s" + (f"; {node.failed}" if node.failed else ""))

    if free:
        a, b = free[0], free[-1]
        if have_boat:
            sep0 = math.dist((a[RX], a[RY], a[RZ]), (0, 0, 0))
            sep1 = math.dist((b[RX], b[RY], b[RZ]), (0, 0, 0))
            add("left the boat after release", sep1 - sep0 > 0.5,
                f"distance from the nest frame {sep0:.2f} -> {sep1:.2f} m")
        travel = math.dist((a[X], a[Y], a[Z]), (b[X], b[Y], b[Z]))
        add("covered ground after release", travel > 0.5, f"{travel:.2f} m in {b[T] - a[T]:.1f} s")
        zmin = min(r[Z] for r in free)
        add("dived under thrust", zmin < PASSIVE_FLOAT_Z - 0.25,
            f"deepest {zmin:.2f} m (floats at {PASSIVE_FLOAT_Z:.2f} without thrust)")
        e_rel, _ = tracking_error(rows, t_free, t_free + 0.5)
        e_end, _ = tracking_error(rows, b[T] - END_WINDOW_S)
        add("on the helix by the end", e_end < END_ERROR_M,
            f"{e_rel:.2f} m off at release, {e_end:.2f} m over the last {END_WINDOW_S:.0f} s (limit {END_ERROR_M} m)")
    else:
        add("free-swim phase logged", False, "no samples after the release")

    hz = node.pose_rate_hz()
    add("pose feedback rate", hz >= 20.0, f"{hz:.0f} Hz (control loop 20 Hz)")
    return out


def main():
    undock_at = float(os.environ.get("UDRONE_UNDOCK_AT", "10"))
    post = float(os.environ.get("UDRONE_POST_UNDOCK", "10"))
    radius = float(os.environ.get("UDRONE_HELIX_R", os.environ.get("UDRONE_SINE_AMP", "0.4")))
    omega = float(os.environ.get("UDRONE_HELIX_OMEGA", os.environ.get("UDRONE_SINE_OMEGA", "0.40")))
    x_rate = float(os.environ.get("UDRONE_HELIX_VX", "0.20"))
    z0 = float(os.environ.get("UDRONE_HELIX_Z0", "-1.0"))

    rclpy.init()
    node = UndockHelixController(
        undock_at_s=undock_at, post_undock_s=post,
        radius=radius, omega=omega, x_rate=x_rate, z0=z0,
    )
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log(LOG_PATH)
        checks = analyze(node)
        print("\nCHECKS", flush=True)
        for name, passed, detail in checks:
            print(f"  {'PASS' if passed else 'FAIL'}  {name}: {detail}", flush=True)
        ok = all(p for _, p, _ in checks)
        print("VERDICT " + ("PASS" if ok else "FAIL"), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
