"""The shape references close, start at rest, and stay in the water."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shape_geom as g


def dist(a, b):
    return math.sqrt(sum((u - v) ** 2 for u, v in zip(a, b)))


def check_derivatives(spec):
    t = 20.0
    dt = 1e-3
    p0, v0, a0 = g.sample(spec, t)
    p1, _, _ = g.sample(spec, t + dt)
    numerical = tuple((b - a) / dt for a, b in zip(p0, p1))
    assert dist(numerical, v0) < 1e-3, (spec["shape"], numerical, v0)
    _, v1, _ = g.sample(spec, t + dt)
    numerical_a = tuple((b - a) / dt for a, b in zip(v0, v1))
    assert dist(numerical_a, a0) < 2e-3, (spec["shape"], numerical_a, a0)


def closes(spec):
    p0, _, _ = g.sample(spec, 0.0)
    end = g._until(spec["tau_end"] / spec["loops"])
    p1, _, _ = g.sample(spec, end)
    assert dist(p0, p1) < 0.05, (spec["shape"], p0, p1, dist(p0, p1))


for kind in ("infinity", "orbit", "loop"):
    spec = g.plan(kind, 3.0, loops=2.0)
    p, v, _ = g.sample(spec, 0.0)
    assert dist(v, (0.0, 0.0, 0.0)) < 1e-9, (kind, v)
    assert abs(p[0]) < 1e-9 and abs(p[1]) < 1e-9
    # Derivatives are checked on a modest pace. The full-thrust plan is the
    # same curve with a larger omega, and a 1 ms difference is then too coarse.
    check_derivatives(g.plan(kind, 3.0, loops=2.0, speed_mps=1.0))
    closes(spec)
    # Settled speed stays at the vehicle's pace.
    _, v_fast, _ = g.sample(spec, 30.0)
    assert dist(v_fast, (0.0, 0.0, 0.0)) <= spec["speed_mps"] + 0.02, (kind, v_fast)

orbit = g.plan("orbit", 3.0, loops=1.0)
_, _, _ = g.sample(orbit, 0.0)
# One settled point sits on the circle of the requested radius.
tau = 10.0
p, v, a = g.sample(orbit, tau)
centre = (0.0, orbit["radius_m"], orbit["z0"])
assert abs(dist(p, centre) - orbit["radius_m"]) < 0.02, dist(p, centre)

eight = g.plan("infinity", 3.0, loops=1.0)
assert eight["radius_m"] == 3.0
# A 3 m vertical loop from -1.2 stays above the 10 m bowl, so the radius holds.
loop = g.plan("loop", 3.0, loops=1.0, z0=-1.2)
assert not loop["clamped"] and abs(loop["radius_m"] - 3.0) < 1e-9
bottom = min(g.sample(loop, t)[0][2] for t in [i * 0.5 for i in range(int(loop["timeout_s"] * 2))])
assert g.Z_DEEP - 0.05 <= bottom <= g.Z_SHALLOW, bottom
# A loop bigger than the water column shrinks.
deep = g.plan("loop", 8.0, loops=1.0, z0=-1.2)
assert deep["clamped"] and deep["radius_m"] < 8.0
# Silence means the fastest pace the curve allows: the turn plus the drag fit
# in the thrust left for control, which is below the straight-line top speed.
assert g.top_speed() > 6.0
assert g.cruise_speed() < g.top_speed()
plain = g.plan("orbit", 3.0, loops=1.0)
assert abs(plain["speed_mps"] - plain["pace_limit"]) < 1e-9
assert plain["speed_mps"] < g.top_speed()
assert g._peak_force("orbit", 3.0, plain["speed_mps"]) <= g.FORCE_MARGIN * g.SURGE_N + 1e-6
assert g._peak_force("orbit", 3.0, plain["speed_mps"] + 0.1) > g.FORCE_MARGIN * g.SURGE_N
# A tighter curve allows less pace.
assert g.feasible_pace("orbit", 1.0) < g.feasible_pace("orbit", 3.0)
fast = g.plan("loop", 3.0, loops=1.0, speed_mps=3.0)
assert abs(fast["speed_mps"] - 3.0) < 1e-9 and not fast["pace_capped"]
assert abs(fast["omega"] - 3.0 / fast["radius_m"]) < 1e-9
quicker = g.plan("loop", 3.0, loops=1.0, speed_mps=10.0)
assert quicker["pace_capped"] and abs(quicker["speed_mps"] - quicker["pace_limit"]) < 1e-9

# The glide from the arm depth starts at rest, reaches the shape's start at
# rest, and the shape then runs shifted forward by the glide's run.
spec = g.plan("orbit", 3.0, loops=1.0, z0=-1.2)
run, dur = g.lead_in(-0.1, spec["z0"])
assert run > 0.0 and dur >= 2.0
p, v, _ = g.sample_lead(spec, 0.0, -0.1)
assert dist(p, (0.0, 0.0, -0.1)) < 1e-9 and dist(v, (0.0, 0.0, 0.0)) < 1e-9
p, v, _ = g.sample_lead(spec, dur - 1e-6, -0.1)
assert dist(p, (run, 0.0, spec["z0"])) < 1e-4 and dist(v, (0.0, 0.0, 0.0)) < 1e-3
p, _, _ = g.sample_lead(spec, dur + 5.0, -0.1)
q, _, _ = g.sample(spec, 5.0)
assert dist(p, (q[0] + run, q[1], q[2])) < 1e-9
assert g.lead_in(-1.1, -1.2) == (0.0, 0.0)

# Swim profile: at rest at both ends, never above the pace, and the time adds up.
import patterns as pt
for length, vmax in ((20.0, 6.0), (1.0, 6.0), (10.0, 0.5)):
    total = pt.trapezoid_time(length, vmax, 1.5)
    s0, v0, _ = pt.trapezoid(length, vmax, 1.5, 0.0)
    s1, v1, _ = pt.trapezoid(length, vmax, 1.5, total)
    assert s0 == 0.0 and v0 == 0.0 and abs(s1 - length) < 1e-9 and v1 == 0.0
    peak = max(pt.trapezoid(length, vmax, 1.5, total * i / 200)[1] for i in range(201))
    assert peak <= vmax + 1e-9
    mid_s, _, _ = pt.trapezoid(length, vmax, 1.5, total * 0.5)
    assert abs(mid_s - 0.5 * length) < 1e-6, (length, vmax, mid_s)
# A gentle 1 m/s on a 3 m orbit is inside the curve budget and is kept.
kept = g.plan("orbit", 4.0, loops=1.0, speed_mps=1.0)
assert abs(kept["speed_mps"] - 1.0) < 1e-9
asked = g.plan("orbit", 3.0, loops=1.0, speed_mps=1.0)
assert abs(asked["speed_mps"] - 1.0) < 1e-9

assert g.normalize("figure eight") == "infinity"
assert g.normalize("circle") == "orbit"
print("shape geometry ok")
