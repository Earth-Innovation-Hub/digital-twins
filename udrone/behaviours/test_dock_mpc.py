"""Dock-phase receding MPC: centers laterally, climbs under the cone, latches path."""
import math
import random

from rendezvous_geom import DOCK_STANDOFF_M, cone_state, nest_axis_point
from dock_mpc import (
    FINAL_ALONG_M, FINAL_CLIMB_CAP, FINAL_LAT_M,
    _cone_ok, enabled, nest_state, plan,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def main():
    assert enabled({"UDRONE_RDV_DOCK_MPC": "1"})
    assert not enabled({"UDRONE_RDV_DOCK_MPC": "0"})

    boat = (10.0, 5.0, 0.0)
    # Off-axis inside the (relaxed 42°) cone but outside the inner climb tube:
    # prefer lateral close; climb hard-gated off. yf=0.70 > inner≈0.54 @ 1.4 m.
    pos = (10.70, 5.0, -1.4)
    cone = cone_state(pos, LEVEL, boat, LEVEL)
    cone = {**cone, "pos": pos}
    assert cone["inside"], cone
    along0, yf0, yl0 = nest_state(cone)
    assert along0 > 1.0 and abs(yf0 - 0.70) < 1e-6

    out = plan(cone, climb_mps=0.13)
    assert "point" in out and "vel" in out and "v_lag" in out and len(out["warm"]) > 0
    va, vf, vl = out["u0"]
    # yf=+0.70 → vf negative; climb hard-gated off while laterally wide.
    assert vf < 0.0, out["u0"]
    assert abs(va) < 1e-9, out["u0"]
    # Lagged plant: |v_lag| ≤ |u| on the first step from rest.
    assert abs(out["v_lag"][1]) <= abs(vf) + 1e-9
    # Soft axis capture still leaves residual offset (no hard snap to axis).
    assert abs(out["point"][0] - 10.0) > 0.2, out["point"]

    # On-axis below standoff target: climb (negative along-rate).
    pos2 = nest_axis_point(boat, LEVEL, 1.2)
    cone2 = {**cone_state(pos2, LEVEL, boat, LEVEL), "pos": pos2}
    out2 = plan(cone2, climb_mps=0.13)
    assert out2["u0"][0] < 0.0, out2["u0"]
    assert abs(out2["u0"][1]) < 0.15 and abs(out2["u0"][2]) < 0.15

    # Final metre + small offset: crawl climb, soft-capture lateral.
    near = (10.0 + 0.12, 5.0, -(FINAL_ALONG_M * 0.7))
    cone_n = {**cone_state(near, LEVEL, boat, LEVEL), "pos": near}
    assert cone_n["inside"], cone_n
    along_n, yf_n, _yl_n = nest_state(cone_n)
    assert along_n <= FINAL_ALONG_M and abs(yf_n) > FINAL_LAT_M
    fin = plan(cone_n, climb_mps=0.13)
    # Hold climb while off-axis in the final metre (do not shrink the cone).
    assert abs(fin["u0"][0]) < 1e-9, fin["u0"]
    # A leftover climb in the lag state must not be published during that hold.
    held = plan(cone_n, climb_mps=0.13, v_state=(-0.13, 0.05, -0.04), rng=random.Random(1))
    assert abs(held["u0"][0]) < 1e-9, held["u0"]
    assert abs(held["v_lag"][0]) < 1e-9, held["v_lag"]
    assert abs(held["along"] - along_n) < 1e-6, (held["along"], along_n)
    # The executed lag uses the control tick. 50 ms moves much less than a
    # 200 ms horizon step, from the same command and the same noise.
    slow = plan(cone2, climb_mps=0.13, dt=0.05, rng=random.Random(2))
    fast = plan(cone2, climb_mps=0.13, dt=0.20, rng=random.Random(2))
    assert abs(slow["v_lag"][0]) < abs(fast["v_lag"][0]) - 1e-4, (slow["v_lag"], fast["v_lag"])
    # Successive calls do not restart a fixed seed: the same state with a
    # continuing generator is not required to match Random(0) twice.
    same = random.Random(3)
    first = plan(cone, climb_mps=0.13, rng=same)
    second = plan(cone, climb_mps=0.13, warm=first["warm"], v_state=first["v_lag"], rng=same)
    assert len(second["warm"]) == len(first["warm"]) == 10
    # Soft capture: setpoint lateral closer to axis than live offset.
    assert abs(fin["point"][0] - 10.0) < abs(near[0] - 10.0), fin["point"]

    # Multi-step receding: stay inside cone and reduce along toward standoff.
    # Integrate the lagged nest velocity (matches CEM plant).
    warm = None
    v_state = (0.0, 0.0, 0.0)
    p = pos2
    for _ in range(40):
        c = {**cone_state(p, LEVEL, boat, LEVEL), "pos": p}
        if not c["inside"]:
            break
        step = plan(c, climb_mps=0.13, warm=warm, v_state=v_state)
        warm = step["warm"]
        v_state = step["v_lag"]
        va, vf, vl = v_state
        # boat level: down=(0,0,-1), fwd=(1,0,0), left=(0,1,0)
        p = (p[0] + vf * 0.20, p[1] + vl * 0.20, p[2] - va * 0.20)
    along_f = -p[2]
    assert along_f <= 1.2 + 1e-6
    assert along_f >= DOCK_STANDOFF_M - 0.05
    assert math.hypot(p[0] - 10.0, p[1] - 5.0) < 0.25

    # Small-offset last-metre latch: should center then reach standoff.
    warm = None
    v_state = (0.0, 0.0, 0.0)
    p = (10.12, 5.08, -0.90)
    for _ in range(80):
        c = {**cone_state(p, LEVEL, boat, LEVEL), "pos": p}
        if not c["inside"]:
            break
        step = plan(c, climb_mps=0.13, warm=warm, v_state=v_state)
        warm = step["warm"]
        v_state = step["v_lag"]
        va, vf, vl = v_state
        p = (p[0] + vf * 0.20, p[1] + vl * 0.20, p[2] - va * 0.20)
        if -p[2] <= DOCK_STANDOFF_M + 0.12 and math.hypot(p[0] - 10.0, p[1] - 5.0) < 0.10:
            break
    assert math.hypot(p[0] - 10.0, p[1] - 5.0) < 0.12, p
    assert -p[2] <= 0.55, p

    assert _cone_ok(1.0, 0.0, 0.0)
    assert not _cone_ok(0.1, 0.5, 0.0)
    print("dock mpc ok")


if __name__ == "__main__":
    main()
