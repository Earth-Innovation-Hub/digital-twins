"""Geometric shape references for the xudrone force loop.

An orbit, a vertical loop, and a figure-eight (lemniscate of Gerono). Each
sample is position, velocity, and acceleration in the path frame: x forward
along the heading at thrust-on, y left, z world-up. The force loop points
the hull along the resulting force. That is the geometric construction for
this vehicle, which thrusts along its nose and has no roll authority.

The speed eases in, so the first sample is at rest on the curve. One loop is
one revolution of an orbit or a vertical circle, or one full figure-eight.
"""
from __future__ import annotations

import math

SHAPES = ("infinity", "orbit", "loop")
MASS_KG = 13.666
# Full forward command: four thrusters at ROTOR_MAX_N. Body-x drag is
# 1.2 v + 3 v^2. Plans leave FORCE_MARGIN of that thrust for the position
# loop, so the unstated pace is where drag (plus the turn) uses the rest.
SURGE_N = 4.0 * 36.0
DRAG_LIN_X = 1.2
DRAG_QUAD_X = 3.0
FORCE_MARGIN = 0.85
RAMP_S = 4.0
LEAD_PITCH = math.radians(30.0)   # glide from the arm depth down to the shape
LEAD_V = 1.0                      # peak speed on that glide
Z0 = -1.2
Z_SHALLOW = -0.45     # stay under the surface
Z_DEEP = -9.0         # stay above the 10 m bowl on a vertical loop
SIZE_MIN = 0.8
SIZE_MAX = 8.0
LOOPS_MIN = 0.5
LOOPS_MAX = 8.0


def _warp(t: float, ramp: float = RAMP_S):
    """Clock that starts at rest and catches real time. (tau, tau', tau'')."""
    t = max(0.0, float(t))
    e = math.exp(-t / ramp)
    return t - ramp * (1.0 - e), 1.0 - e, e / ramp


def _until(tau_end: float, ramp: float = RAMP_S) -> float:
    t = tau_end + ramp
    for _ in range(12):
        got, _, _ = _warp(t, ramp)
        t += tau_end - got
        if abs(tau_end - got) < 1e-6:
            break
    return t


def _rot45(x: float, y: float):
    """Rotate so the figure-eight's initial diagonal tangent lies on +x."""
    k = math.sqrt(0.5)
    return k * x + k * y, -k * x + k * y


def normalize(shape: str) -> str:
    name = str(shape or "").strip().lower().replace("_", " ").replace("-", " ")
    if name in ("infinity", "figure 8", "figure eight", "lemniscate", "eight"):
        return "infinity"
    if name in ("orbit", "circle", "circles"):
        return "orbit"
    if name in ("loop", "vertical", "vertical loop", "somersault"):
        return "loop"
    raise ValueError(f"shape must be infinity, orbit, or loop (got {shape!r})")


def _speed_at(force_n: float) -> float:
    disc = DRAG_LIN_X * DRAG_LIN_X + 4.0 * DRAG_QUAD_X * force_n
    return (-DRAG_LIN_X + math.sqrt(disc)) / (2.0 * DRAG_QUAD_X)


def top_speed() -> float:
    """Straight-line speed at which body-x drag equals full forward thrust."""
    return _speed_at(SURGE_N)


def cruise_speed() -> float:
    """Straight-line speed that leaves FORCE_MARGIN of the thrust for control."""
    return _speed_at(FORCE_MARGIN * SURGE_N)


def _drag(v: float) -> float:
    return DRAG_LIN_X * v + DRAG_QUAD_X * v * v


def _peak_force(kind: str, radius: float, pace: float) -> float:
    """Largest force the shape asks for at this pace (turn plus drag, no ramp)."""
    omega = pace / (radius * math.sqrt(2.0)) if kind == "infinity" else pace / radius
    spec = {"shape": kind, "radius_m": radius, "omega": omega, "z0": 0.0}
    worst = 0.0
    n = 72
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        _p, v, a = _curve(spec, theta)
        speed = math.sqrt(sum(c * c for c in v))
        unit = tuple(c / speed for c in v) if speed > 1e-9 else (0.0, 0.0, 0.0)
        need = tuple(MASS_KG * a[k] + _drag(speed) * unit[k] for k in range(3))
        worst = max(worst, math.sqrt(sum(c * c for c in need)))
    return worst


def feasible_pace(kind: str, radius: float) -> float:
    """Fastest pace whose turn and drag fit in FORCE_MARGIN of full thrust."""
    budget = FORCE_MARGIN * SURGE_N
    lo, hi = 0.05, top_speed()
    if _peak_force(kind, radius, hi) <= budget:
        return hi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _peak_force(kind, radius, mid) <= budget:
            lo = mid
        else:
            hi = mid
    return lo


def lead_in(depth_now: float, z0: float):
    """(forward metres, seconds) of the glide from the arm depth to the shape."""
    dz = abs(float(z0) - float(depth_now))
    if dz < 0.3:
        return 0.0, 0.0
    run = dz / math.tan(LEAD_PITCH)
    length = math.hypot(run, dz)
    return run, max(2.0, math.pi * length / (2.0 * LEAD_V))


def plan(shape: str, size_m: float, loops: float = 2.0, z0: float = Z0,
         speed_mps: float = None) -> dict:
    """Shape at the spoken pace, or at the fastest pace the shape allows.

    A spoken pace is also held to that limit: the curve's own acceleration
    and the drag at that pace must fit in the thrust left for control.
    """
    kind = normalize(shape)
    size = min(SIZE_MAX, max(SIZE_MIN, float(size_m)))
    laps = min(LOOPS_MAX, max(LOOPS_MIN, float(loops)))
    depth = min(Z_SHALLOW, float(z0))
    radius = size
    if kind == "loop":
        room = max(0.4, (depth - Z_DEEP) / 2.0)
        radius = min(size, room)
    limit = feasible_pace(kind, radius)
    asked = limit if speed_mps is None else max(0.05, float(speed_mps))
    pace = min(asked, limit)
    omega = pace / (radius * math.sqrt(2.0)) if kind == "infinity" else pace / radius
    tau_end = laps * (2.0 * math.pi / omega)
    return {
        "shape": kind,
        "size_m": size,
        "radius_m": radius,
        "loops": laps,
        "omega": omega,
        "z0": depth,
        "speed_mps": pace,
        "pace_limit": limit,
        "pace_capped": asked > limit + 1e-6,
        "tau_end": tau_end,
        "timeout_s": _until(tau_end) + 2.0,
        "clamped": radius + 1e-6 < size,
    }


def _curve(spec: dict, theta: float):
    """Path-frame (position, d/dtau, d2/dtau2) at curve angle theta."""
    radius = spec["radius_m"]
    omega = spec["omega"]
    s, c = math.sin(theta), math.cos(theta)
    kind = spec["shape"]
    if kind == "orbit":
        # Centre is (0, radius). theta=0 is the origin, tangent +x.
        p = (radius * s, radius * (1.0 - c), spec["z0"])
        v = (radius * omega * c, radius * omega * s, 0.0)
        a = (-radius * omega * omega * s, radius * omega * omega * c, 0.0)
    elif kind == "loop":
        # Same circle in the vertical plane, diving first.
        p = (radius * s, 0.0, spec["z0"] - radius * (1.0 - c))
        v = (radius * omega * c, 0.0, -radius * omega * s)
        a = (-radius * omega * omega * s, 0.0, -radius * omega * omega * c)
    else:
        x, y = radius * s, radius * s * c
        vx, vy = radius * omega * c, radius * omega * (c * c - s * s)
        ax, ay = -radius * omega * omega * s, -4.0 * radius * omega * omega * s * c
        px, py = _rot45(x, y)
        vx, vy = _rot45(vx, vy)
        ax, ay = _rot45(ax, ay)
        p, v, a = (px, py, spec["z0"]), (vx, vy, 0.0), (ax, ay, 0.0)
    return p, v, a


def sample_lead(spec: dict, t: float, depth_now: float):
    """Glide from the arm point to the shape start; then the shape, shifted forward.

    The glide eases in and out, so the shape still starts at rest.
    """
    run, dur = lead_in(depth_now, spec["z0"])
    if dur <= 0.0:
        return sample(spec, t)
    if t < dur:
        u = 0.5 * (1.0 - math.cos(math.pi * t / dur))
        du = 0.5 * math.pi / dur * math.sin(math.pi * t / dur)
        ddu = 0.5 * (math.pi / dur) ** 2 * math.cos(math.pi * t / dur)
        dz = spec["z0"] - depth_now
        p = (run * u, 0.0, depth_now + dz * u)
        v = (run * du, 0.0, dz * du)
        a = (run * ddu, 0.0, dz * ddu)
        return p, v, a
    p, v, a = sample(spec, t - dur)
    return (p[0] + run, p[1], p[2]), v, a


def sample(spec: dict, t: float):
    """(position, velocity, acceleration) at sim seconds `t`. z is world-up."""
    tau, tau_d, tau_dd = _warp(t)
    p, v, a = _curve(spec, spec["omega"] * tau)
    vel = tuple(tau_d * u for u in v)
    acc = tuple(tau_d * tau_d * u + tau_dd * w for u, w in zip(a, v))
    return p, vel, acc
