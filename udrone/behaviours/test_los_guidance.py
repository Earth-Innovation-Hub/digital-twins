"""LOS guidance flown on a point hull that only moves along its nose.

The hull model: surge from the same speed law the controller uses (drag at the
reference speed, speed error, reference acceleration) with the xudrone drag
and a 144 N thrust limit; heading and pitch follow the guidance direction at a
limited rate. No sideslip, so a heading error is the only way to move sideways.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import los_guidance as lg

MASS = 13.666
DRAG_LIN, DRAG_QUAD = 1.2, 3.0
SURGE_N = 144.0
KV = 1.5
RATE = 1.5          # rad/s heading and pitch rate limit
DT = 0.05


def fly(wps, cruise, pos=None, yaw=None, t_max=120.0, arrive=0.35, nose_bias=0.0):
    """nose_bias: radians the nose settles under the command per (m/s)^2,
    like the hydrodynamic moment seen on the twin."""
    g = lg.LosGuide(wps, cruise)
    p = list(pos if pos is not None else wps[0])
    a, b = wps[0], wps[1]
    psi = yaw if yaw is not None else math.atan2(b[1] - a[1], b[0] - a[0])
    th = 0.0          # nose elevation, + up
    u = 0.0
    rows = []
    t = 0.0
    while t < t_max:
        out = g.step(tuple(p), psi, DT, elev=th)
        u_ref = out["u_ref"]
        f = MASS * (KV * (u_ref - u) + out["a_ref"]) + DRAG_LIN * u_ref + DRAG_QUAD * u_ref * abs(u_ref)
        f = max(-SURGE_N, min(SURGE_N, f))
        u += (f - DRAG_LIN * u - DRAG_QUAD * u * abs(u)) / MASS * DT
        dx, dy, dz = out["dir"]
        want_psi = math.atan2(dy, dx)
        want_th = math.atan2(dz, math.hypot(dx, dy)) - nose_bias * u * u
        psi = lg.wrap_pi(psi + max(-RATE * DT, min(RATE * DT, lg.wrap_pi(want_psi - psi))))
        th += max(-RATE * DT, min(RATE * DT, want_th - th))
        ce = math.cos(th)
        p[0] += u * ce * math.cos(psi) * DT
        p[1] += u * ce * math.sin(psi) * DT
        p[2] += u * math.sin(th) * DT
        rows.append((t, tuple(p), u, out))
        t += DT
        if math.dist(p, wps[-1]) <= arrive and t > 1.0:
            return True, t, rows
    return False, t, rows


def max_abs(rows, key, after=0.0):
    return max(abs(r[3][key]) for r in rows if r[0] >= after)


# Aligned straight leg at the drag-balance cruise: arrives, never at full
# speed on 8 m, slow at the end.
ok, t, rows = fly([(0, 0, -1.2), (8, 0, -1.2)], 6.2)
assert ok, ("aligned 8 m", t)
assert rows[-1][2] < 0.6, rows[-1][2]
assert max_abs(rows, "xte") < 0.05
assert max(r[2] for r in rows) < 3.5

# The 8 m west leg from a hull facing south-east (yaw -1.28 rad): the turn
# happens before the surge, so the hull never wanders off the line.
ok, t, rows = fly([(2.25, 5.72, -0.13), (-5.75, 5.72, -0.13)], 6.2, yaw=-1.28)
assert ok, ("turn first", t)
early = [r for r in rows if abs(r[3]["herr"]) > math.radians(60.0)]
assert early and all(abs(r[2]) < 0.15 for r in early), "surged before it turned"
assert max_abs(rows, "xte") < 0.4, max_abs(rows, "xte")

# Facing straight away from the leg: yaws round level, does not pitch over.
ok, t, rows = fly([(0, 0, -1.2), (0, -3, -1.2)], 6.2, yaw=math.pi / 2)
assert ok, ("about turn", t)
assert all(abs(r[3]["dir"][2]) < 1e-9 for r in rows if abs(r[3]["herr"]) > math.radians(60.0))

# Start 1 m off the line: the cross-track error decays.
ok, t, rows = fly([(0, 0, -1.2), (20, 0, -1.2)], 2.0, pos=(0, 1.0, -1.2))
assert ok
assert max(abs(r[3]["xte"]) for r in rows if r[3]["s"] > 10.0) < 0.1

# Depth change along the leg: settles onto the line, pitch stays limited.
ok, t, rows = fly([(0, 0, -0.3), (12, 0, -2.0)], 2.0)
assert ok
assert all(abs(r[3]["gamma"]) <= math.radians(45.0) + 1e-9 for r in rows)
assert abs(rows[-1][1][2] + 2.0) < 0.35

# The nose rides 6 deg low at 2 m/s: the elevation integrator keeps the hull
# near the line instead of diving half a metre (the 2026-11-19 live run).
bias = math.radians(6.0) / 4.0
ok, t, rows = fly([(0, 0, -0.3), (8, 0, -0.3)], 6.2, nose_bias=bias)
assert ok
with_i = max_abs(rows, "vte")
ki = lg.KI_ELEV
lg.KI_ELEV = 0.0
_, _, rows_no = fly([(0, 0, -0.3), (8, 0, -0.3)], 6.2, nose_bias=bias)
lg.KI_ELEV = ki
without_i = max_abs(rows_no, "vte")
assert with_i < 0.7 * without_i, (with_i, without_i)

# Past the end: backs up along the line.
ok, t, rows = fly([(0, 0, -1.2), (5, 0, -1.2)], 2.0, pos=(6.0, 0.0, -1.2))
assert ok
assert min(r[3]["u_ref"] for r in rows) < -0.1
assert rows[0][3]["reverse"] or rows[5][3]["reverse"]

# Polyline: switches legs and slows for the right-angle corner.
wps = [(0, 0, -1.2), (10, 0, -1.2), (10, 10, -1.2)]
ok, t, rows = fly(wps, 3.0)
assert ok
legs = [r[3]["leg"] for r in rows]
assert legs[0] == 0 and legs[-1] == 1 and legs == sorted(legs)
corner = min(rows, key=lambda r: math.dist(r[1], (10, 0, -1.2)))
assert corner[2] < 2.5, corner[2]

# A leg with no horizontal length cannot be steered along.
try:
    lg.LosGuide([(0, 0, -1.0), (0.1, 0, -3.0)], 1.0)
    raise AssertionError("short leg accepted")
except ValueError:
    pass

g = lg.LosGuide([(0, 0, -1.2), (8, 0, -1.2)], 6.2)
assert 5.0 < g.eta() < 20.0, g.eta()
print("los guidance ok")
