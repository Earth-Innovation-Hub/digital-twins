"""Receding-horizon through-hoop planner in the hoop frame (ROS-free).

State is ``(s, y, z)``:
  - ``s`` along the ring axis (0 at the opening; approach s < 0, exit s > 0)
  - ``y`` horizontal lateral
  - ``z`` vertical offset from the ring centre

Keeps the hull inside a cylindrical tube while driving ``s`` through the
opening to ``exit_m``. Same CEM + first-order velocity lag pattern as
``dock_mpc``, different frame and costs.

Disable with ``UDRONE_HOOP_MPC=0``.
"""
from __future__ import annotations

import math
import os
import random
from typing import Optional

# Horizon ≈ 2.4 s; fits the 50 ms force-loop tick.
HORIZON = 10
DT = 0.20
SAMPLES = 18
ELITES = 5
CEM_ITERS = 2
VEL_TAU_S = 0.65
V_S_MAX = 0.55
V_LAT_MAX = 0.40
TUBE_R = 0.28               # tighter than INNER_PASS 0.40 — clear of the tube
TUBE_MARGIN = 0.92
W_S = 3.5                   # progress toward exit
W_LAT = 28.0                # prefer centering the tube before the plane
W_V = 0.45
W_TERMINAL = 12.0
W_TUBE = 120.0
EXIT_DEFAULT_M = 2.0
# Soft axis capture in the position ref (fraction of residual lateral kept).
# Under yaw_hold the hull only surges; the force needs a centering component.
AXIS_KEEP = 0.45            # far approach: bleed toward the axis
AXIS_KEEP_EXIT = 0.70       # past the plane: gentler
# Final metre: hard-lock the opening centreline — do not drift onto the rim.
FINAL_S_M = 1.20            # |s| within this of the plane → final gate
FINAL_LAT_M = 0.10          # must be this close before punching through
FINAL_KEEP = 0.05           # almost snap lateral ref to the axis
FINAL_VS_CAP = 0.10         # crawl while still off-axis in the final metre


def enabled(env: Optional[dict] = None) -> bool:
    src = env if env is not None else os.environ
    raw = str(src.get("UDRONE_HOOP_MPC", "1")).strip().lower()
    return raw not in ("0", "false", "off", "no")


def _norm2(a):
    n = math.sqrt(a[0] * a[0] + a[1] * a[1])
    if n < 1e-9:
        return (1.0, 0.0), 0.0
    return (a[0] / n, a[1] / n), n


def hoop_axes(axis_xy):
    """Unit forward (horizontal), left (horizontal), up (world +Z)."""
    (fx, fy), _ = _norm2((float(axis_xy[0]), float(axis_xy[1])))
    # left = +Z × fwd → (−fy, fx) in ENU horizontal
    lx, ly = -fy, fx
    return (fx, fy, 0.0), (lx, ly, 0.0), (0.0, 0.0, 1.0)


def hoop_state(pos, center, axis_xy) -> tuple[float, float, float, tuple, tuple, tuple]:
    """Return (s, y, z, fwd, left, up) for ``pos`` relative to the ring."""
    fwd, left, up = hoop_axes(axis_xy)
    v = (
        float(pos[0]) - float(center[0]),
        float(pos[1]) - float(center[1]),
        float(pos[2]) - float(center[2]),
    )
    s = v[0] * fwd[0] + v[1] * fwd[1] + v[2] * fwd[2]
    y = v[0] * left[0] + v[1] * left[1] + v[2] * left[2]
    z = v[0] * up[0] + v[1] * up[1] + v[2] * up[2]
    return s, y, z, fwd, left, up


def tube_ok(y: float, z: float, radius: float = TUBE_R) -> bool:
    return math.hypot(float(y), float(z)) <= float(radius) * TUBE_MARGIN + 1e-9


def _final_gate(s: float, y: float, z: float) -> bool:
    """True in the last metre before the plane while still off the opening axis."""
    return (-FINAL_S_M <= float(s) < 0.15) and math.hypot(float(y), float(z)) > FINAL_LAT_M


def _clip_u(
    vs, vy, vz, speed_cap: float,
    s: Optional[float] = None, exit_m: Optional[float] = None,
    y: Optional[float] = None, z: Optional[float] = None,
):
    vs = max(-0.15, min(float(speed_cap), float(vs)))
    # Never reverse while still short of the exit — a one-step setpoint from
    # rest sits on the hull; under axis yaw_hold that becomes thrust reverse
    # and s runs away from the opening (2026-11-17 hoop log).
    if s is not None and exit_m is not None and float(s) < float(exit_m) - 0.25:
        vs = max(0.05, vs)
    # Align then punch: crawl along +s until centered in the final metre.
    if s is not None and y is not None and z is not None and _final_gate(s, y, z):
        vs = min(vs, FINAL_VS_CAP)
        vs = max(0.04, vs)
    vy = max(-V_LAT_MAX, min(V_LAT_MAX, float(vy)))
    vz = max(-V_LAT_MAX, min(V_LAT_MAX, float(vz)))
    return vs, vy, vz


def _lag_step(v_state, v_cmd, tau_s: float = VEL_TAU_S):
    tau = max(0.08, float(tau_s))
    a = min(1.0, DT / tau)
    return tuple(float(v_state[i]) + a * (float(v_cmd[i]) - float(v_state[i])) for i in range(3))


def _rollout(s0, y0, z0, U, speed_cap: float, exit_m: float, v0=(0.0, 0.0, 0.0)):
    s, y, z = s0, y0, z0
    v_s = tuple(float(c) for c in v0)
    cost = 0.0
    for vs, vy, vz in U:
        vs, vy, vz = _clip_u(vs, vy, vz, speed_cap, s=s, exit_m=exit_m, y=y, z=z)
        v_s = _lag_step(v_s, (vs, vy, vz))
        s = s + v_s[0] * DT
        y = y + v_s[1] * DT
        z = z + v_s[2] * DT
        lat = math.hypot(y, z)
        # Near the plane, lateral error dominates — do not score a rim graze
        # as progress toward exit.
        w_lat = W_LAT * (2.8 if -FINAL_S_M <= s < 0.3 else 1.0)
        stage = (
            W_S * (exit_m - s) ** 2
            + w_lat * lat * lat
            + W_V * (vs * vs + vy * vy + vz * vz)
        )
        if not tube_ok(y, z):
            stage += W_TUBE * (1.0 + lat)
        if -FINAL_S_M <= s < 0.2 and lat > FINAL_LAT_M:
            stage += 60.0 * (lat - FINAL_LAT_M) ** 2
        # Hard preference: never score reverse while short of the exit.
        if s < exit_m - 0.25 and vs < 0.05:
            stage += 40.0 * (0.05 - vs) ** 2
        cost += stage
    lat_t = math.hypot(y, z)
    cost += W_TERMINAL * ((exit_m - s) ** 2 + 2.5 * lat_t * lat_t)
    return cost, (s, y, z)


def _seed_sequence(s, y, z, speed_cap: float, exit_m: float):
    """Warm start: surge toward exit, first-order pull into the tube."""
    U = []
    ss, yy, zz = s, y, z
    lat_tau = 0.45
    for _ in range(HORIZON):
        vs = float(speed_cap)
        # Slow a bit if far off-axis so lateral can catch up before the plane.
        lat = math.hypot(yy, zz)
        if lat > TUBE_R * 0.45 and ss < 0.0:
            vs *= max(0.40, 1.0 - (lat - TUBE_R * 0.45) / max(TUBE_R, 1e-3))
        vy = max(-V_LAT_MAX, min(V_LAT_MAX, -yy / lat_tau))
        vz = max(-V_LAT_MAX, min(V_LAT_MAX, -zz / lat_tau))
        vs, vy, vz = _clip_u(vs, vy, vz, speed_cap, s=ss, exit_m=exit_m, y=yy, z=zz)
        U.append((vs, vy, vz))
        ss = ss + vs * DT
        yy = yy + vy * DT
        zz = zz + vz * DT
        if ss > exit_m:
            break
    while len(U) < HORIZON:
        U.append(U[-1] if U else (speed_cap, 0.0, 0.0))
    return U


# Force-loop lead along +s so the tracked point sits ahead of the hull.
# One lag-step from rest is ~cm and under yaw_hold reads as "behind" → reverse.
LEAD_S_M = 0.55
LEAD_S_T = 1.25   # seconds of speed_cap added on top of lagged advance
LEAD_S_FINAL = 0.35  # shorter lead while aligning in the final metre


def plan(
    pos,
    center,
    axis_xy,
    speed_mps: float = 0.32,
    exit_m: float = EXIT_DEFAULT_M,
    warm: Optional[list] = None,
    rng: Optional[random.Random] = None,
    v_state: Optional[tuple] = None,
) -> dict:
    """Receding plan; returns world point + velocity for the force loop."""
    rng = rng or random.Random(0)
    speed_cap = max(0.12, float(speed_mps))
    exit_m = max(0.6, float(exit_m))
    s, y, z, fwd, left, up = hoop_state(pos, center, axis_xy)
    v0 = tuple(float(c) for c in (v_state or (0.0, 0.0, 0.0)))
    # Drop a reverse-poisoned warm start (s running away from the opening).
    if warm and s < exit_m - 0.25 and warm[0][0] < 0.0:
        warm = None
    if v0[0] < 0.0 and s < exit_m - 0.25:
        v0 = (0.0, v0[1], v0[2])

    mean = warm
    if not mean or len(mean) != HORIZON:
        mean = _seed_sequence(s, y, z, speed_cap, exit_m)
    else:
        mean = list(mean[1:]) + [mean[-1]]

    sigma = (0.08, 0.12, 0.12)
    best_U = mean
    best_cost = float("inf")

    for _ in range(CEM_ITERS):
        scored = []
        for sample_i in range(SAMPLES):
            if sample_i == 0:
                U = [tuple(u) for u in mean]
            else:
                U = []
                for u in mean:
                    U.append((
                        u[0] + rng.gauss(0.0, sigma[0]),
                        u[1] + rng.gauss(0.0, sigma[1]),
                        u[2] + rng.gauss(0.0, sigma[2]),
                    ))
            cost, _ = _rollout(s, y, z, U, speed_cap, exit_m, v0=v0)
            scored.append((cost, U))
            if cost < best_cost:
                best_cost = cost
                best_U = U
        scored.sort(key=lambda t: t[0])
        elite = scored[:ELITES]
        mean = []
        for k in range(HORIZON):
            mean.append(tuple(
                sum(e[1][k][j] for e in elite) / float(len(elite))
                for j in range(3)
            ))
        sigma = tuple(max(0.03, 0.65 * sigma[j]) for j in range(3))

    vs, vy, vz = _clip_u(*best_U[0], speed_cap, s=s, exit_m=exit_m, y=y, z=z)
    v_lag = _lag_step(v0, (vs, vy, vz))
    # Lead along +s so the force loop always has forward bite toward the exit.
    # In the final metre while off-axis, shorten the lead — punch only once centered.
    if _final_gate(s, y, z):
        lead = LEAD_S_FINAL
    else:
        lead = max(LEAD_S_M, float(speed_cap) * LEAD_S_T)
    s_ref = min(exit_m, s + max(v_lag[0], 0.0) * DT + lead)
    # Soft axis capture far out; hard-lock the opening centreline in the final
    # metre so residual lateral does not walk the hull onto the rim.
    if -FINAL_S_M <= s < 0.4:
        keep = FINAL_KEEP
    elif s < 0.0:
        keep = AXIS_KEEP
    else:
        keep = AXIS_KEEP_EXIT
    y_ref = y * keep + v_lag[1] * DT
    z_ref = z * keep + v_lag[2] * DT
    point = tuple(
        float(center[i]) + s_ref * fwd[i] + y_ref * left[i] + z_ref * up[i]
        for i in range(3)
    )
    vel = tuple(v_lag[0] * fwd[i] + v_lag[1] * left[i] + v_lag[2] * up[i] for i in range(3))
    return {
        "point": point,
        "vel": vel,
        "s": s_ref,
        "y": y_ref,
        "z": z_ref,
        "warm": best_U,
        "cost": best_cost,
        "u0": (vs, vy, vz),
        "v_lag": v_lag,
        "state0": (s, y, z),
        "fwd": fwd,
        "left": left,
        "up": up,
        "in_tube": tube_ok(y, z),
        "final_gate": _final_gate(s, y, z),
    }
