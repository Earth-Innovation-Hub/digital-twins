"""Receding-horizon dock planner in the nest frame (ROS-free).

Replaces the open-loop climb setpoint while the hull is inside the acceptance
cone. Plans nest-frame velocities over a short horizon with cone / standoff
constraints, then returns only the first step for the force loop to track.

No QP solver: cross-entropy sampling around a warm-started control sequence.
Disable with ``UDRONE_RDV_DOCK_MPC=0``.
"""
from __future__ import annotations

import math
import os
import random
from typing import Optional

from rendezvous_geom import (
    CONE_HALF,
    DOCK_CLIMB_MPS,
    DOCK_STANDOFF_M,
)

# Horizon ≈ 2.4 s; sized to finish well inside the 50 ms force-loop tick.
HORIZON = 10
DT = 0.20
SAMPLES = 18
ELITES = 5
CEM_ITERS = 2
CONE_MARGIN = 0.90          # stay inside this fraction of CONE_HALF
# Last-metre RL elite (2026-11-17): slower climb, softer lateral, earlier gate.
V_LAT_MAX = 0.33            # m/s toward the axis (was 0.45; last-metre overshoot)
LAT_TAU = 0.81              # seed lateral first-order pull (s)
# Match force-loop velocity bandwidth (KV≈1.5 → τ≈0.67 s). CEM rolls a
# first-order lag so planned nest velocities are achievable under mass+drag.
VEL_TAU_S = 0.65
W_ALONG = 4.0
W_LAT = 12.0
W_V = 0.70                   # stronger velocity penalty in last metre
W_TERMINAL = 6.0
W_CONE = 80.0
W_FLOOR = 40.0
# Last metre under the nest: hold climb until centered + soft axis capture.
FINAL_ALONG_M = 1.12        # along ≤ this → final gate (engage earlier)
FINAL_LAT_M = 0.05          # must be this close before climb resumes
FINAL_CLIMB_CAP = 0.08      # |va| crawl once centered in the final metre
FINAL_VLAT_CAP = 0.13       # lateral m/s cap in the final metre (anti-overshoot)
AXIS_KEEP = 0.45            # soft lateral ref blend toward nest axis
FINAL_KEEP = 0.06           # almost snap lateral ref in the final metre


def enabled(env: Optional[dict] = None) -> bool:
    src = env if env is not None else os.environ
    raw = str(src.get("UDRONE_RDV_DOCK_MPC", "1")).strip().lower()
    return raw not in ("0", "false", "off", "no")


def nest_state(cone) -> tuple[float, float, float]:
    """along (m below nest), boat-forward, boat-left offsets from the nest."""
    nest = cone["nest"]
    down = cone["down"]
    fwd = cone["fwd"]
    left = cone["left"]
    # Reconstruct AUV position from cone fields when caller passes live cone
    # that already measured along/lateral; prefer explicit pos if present.
    pos = cone.get("pos")
    if pos is None:
        # Fall back: along on axis + lateral unknown direction — use along only.
        along = max(DOCK_STANDOFF_M, float(cone.get("along", cone["range"])))
        return along, 0.0, 0.0
    v = tuple(float(pos[i]) - float(nest[i]) for i in range(3))
    along = sum(v[i] * down[i] for i in range(3))
    yf = sum(v[i] * fwd[i] for i in range(3))
    yl = sum(v[i] * left[i] for i in range(3))
    return max(0.05, along), yf, yl


def _cone_ok(along: float, yf: float, yl: float) -> bool:
    if along < DOCK_STANDOFF_M - 1e-6:
        return False
    lat = math.hypot(yf, yl)
    half = CONE_HALF * CONE_MARGIN
    return lat <= along * math.tan(half) + 1e-9


def _inner_half_w(along: float) -> float:
    """Lateral tube where climb is allowed (half the margin cone width)."""
    return max(0.05, float(along) * math.tan(CONE_HALF * CONE_MARGIN) * 0.5)


def _final_gate(along: float, yf: float, yl: float) -> bool:
    """True in the last metre under the nest while still off the nest axis."""
    return (
        float(along) <= FINAL_ALONG_M + 1e-9
        and math.hypot(float(yf), float(yl)) > FINAL_LAT_M
    )


def _clip_u(along, yf, yl, va, vf, vl, climb_cap: float):
    """Saturate nest-frame velocity; hard-block climb while off the inner tube."""
    va = max(-climb_cap, min(climb_cap, float(va)))
    # Soften lateral in the final metre so axis hunting does not overshoot
    # out of the shrinking cone.
    v_lat_cap = V_LAT_MAX
    if float(along) <= FINAL_ALONG_M:
        v_lat_cap = min(V_LAT_MAX, FINAL_VLAT_CAP)
    vf = max(-v_lat_cap, min(v_lat_cap, float(vf)))
    vl = max(-v_lat_cap, min(v_lat_cap, float(vl)))
    lat = math.hypot(yf, yl)
    # On-axis deadband: kill lateral chatter that re-excites a centered dock.
    if lat <= FINAL_LAT_M:
        vf = 0.0
        vl = 0.0
    if lat > _inner_half_w(along):
        # Climb is negative along-rate; hold while off the inner tube.
        va = 0.0
    # Last metre: hold depth until on-axis. Crawling while still offset shrinks
    # the acceptance cone and dumps the hull outside (live last-metre exits).
    if _final_gate(along, yf, yl):
        va = 0.0
    elif float(along) <= FINAL_ALONG_M and va < 0.0:
        # Centered enough to climb — crawl so residual offset cannot race the cone.
        va = max(va, -max(FINAL_CLIMB_CAP, 0.5 * climb_cap))
    # Never command through the deck.
    if along + va * DT < DOCK_STANDOFF_M:
        va = min(va, (DOCK_STANDOFF_M - along) / DT)
    return va, vf, vl


def _lag_step(v_state, v_cmd, tau_s: float = VEL_TAU_S):
    """One first-order lag step matching the force-loop velocity bandwidth."""
    tau = max(0.08, float(tau_s))
    a = min(1.0, DT / tau)
    return tuple(float(v_state[i]) + a * (float(v_cmd[i]) - float(v_state[i])) for i in range(3))


def _rollout(along0, yf0, yl0, U, climb_cap: float, v0=(0.0, 0.0, 0.0)):
    """Simulate nest-frame commands through a lagged velocity plant.

    ``U[k]=(va,vf,vl)`` are commanded nest rates; the plant is a first-order
    lag (``VEL_TAU_S``) so CEM does not plan step changes the force loop cannot
    track. Lower cost is better.
    """
    along, yf, yl = along0, yf0, yl0
    v_s = tuple(float(c) for c in v0)
    cost = 0.0
    for va, vf, vl in U:
        va, vf, vl = _clip_u(along, yf, yl, va, vf, vl, climb_cap)
        v_s = _lag_step(v_s, (va, vf, vl))
        along = along + v_s[0] * DT
        yf = yf + v_s[1] * DT
        yl = yl + v_s[2] * DT
        lat = math.hypot(yf, yl)
        # Near the nest, lateral error dominates — do not score a climb past
        # a few centimetres of offset as progress toward standoff.
        w_lat = W_LAT * (3.5 if along <= FINAL_ALONG_M else 1.0)
        w_v = W_V * (2.2 if along <= FINAL_ALONG_M else 1.0)
        stage = (
            W_ALONG * (along - DOCK_STANDOFF_M) ** 2
            + w_lat * lat * lat
            + w_v * (va * va + vf * vf + vl * vl)
        )
        if along < DOCK_STANDOFF_M:
            stage += W_FLOOR * (DOCK_STANDOFF_M - along) ** 2
        if not _cone_ok(max(along, 1e-3), yf, yl):
            stage += W_CONE * (1.0 + lat)
        if along <= FINAL_ALONG_M and lat > FINAL_LAT_M:
            stage += 60.0 * (lat - FINAL_LAT_M) ** 2
        cost += stage
    lat_t = math.hypot(yf, yl)
    cost += W_TERMINAL * (
        (along - DOCK_STANDOFF_M) ** 2 + 2.5 * lat_t * lat_t
    )
    return cost, (along, yf, yl)


def _seed_sequence(along, yf, yl, climb_cap: float):
    """Heuristic warm start: damp lateral (τ≈0.8 s), climb only inside the tube.

    Avoid bang-bang ``−f/DT`` lateral seeds — those saturate V_LAT_MAX and
    overshoot the axis into a spiral under the nest.
    """
    U = []
    a, f, l = along, yf, yl
    lat_tau0 = max(0.25, float(LAT_TAU))
    for _ in range(HORIZON):
        lat = math.hypot(f, l)
        # Slow the lateral pull near the nest so seed does not bang V_LAT_MAX.
        lat_tau = lat_tau0 * (1.7 if a <= FINAL_ALONG_M else 1.0)
        climb_scale = max(0.0, 1.0 - lat / _inner_half_w(a))
        va = -climb_cap * climb_scale
        # First-order pull to axis: v = −offset / τ (not −offset / DT).
        if lat <= FINAL_LAT_M:
            vf = 0.0
            vl = 0.0
        else:
            vf = max(-V_LAT_MAX, min(V_LAT_MAX, -f / lat_tau))
            vl = max(-V_LAT_MAX, min(V_LAT_MAX, -l / lat_tau))
        va, vf, vl = _clip_u(a, f, l, va, vf, vl, climb_cap)
        U.append((va, vf, vl))
        a = max(DOCK_STANDOFF_M, a + va * DT)
        f = f + vf * DT
        l = l + vl * DT
    return U


def plan(
    cone,
    climb_mps: float = DOCK_CLIMB_MPS,
    warm: Optional[list] = None,
    rng: Optional[random.Random] = None,
    v_state: Optional[tuple] = None,
) -> dict:
    """Receding plan from a live ``cone_state`` dict (must include ``pos``).

    Returns world setpoint, nest-relative world velocity, warm sequence, and
    diagnostics. Velocity is relative to the nest (caller adds boat velocity).
    ``v_state`` is the lagged nest-frame velocity from the previous tick
    (warm-starts the first-order plant).
    """
    rng = rng or random.Random(0)
    climb_cap = max(0.05, float(climb_mps))
    along, yf, yl = nest_state(cone)
    nest = cone["nest"]
    down = cone["down"]
    fwd = cone["fwd"]
    left = cone["left"]
    v0 = tuple(float(c) for c in (v_state or (0.0, 0.0, 0.0)))

    lat0 = math.hypot(yf, yl)
    mean = warm
    # Drop poisoned warm starts once centered in the final metre — a saturated
    # ±V_LAT sequence re-excites the axis under the lag plant.
    if (
        not mean
        or len(mean) != HORIZON
        or (along <= FINAL_ALONG_M and lat0 <= FINAL_LAT_M)
    ):
        mean = _seed_sequence(along, yf, yl, climb_cap)
    else:
        # Shift warm start one step (receding).
        mean = list(mean[1:]) + [mean[-1]]

    # Narrower lateral exploration in the final metre — CEM was saturating
    # ±V_LAT and oscillating under the nest.
    if along <= FINAL_ALONG_M and lat0 <= FINAL_LAT_M:
        sigma = (0.04, 0.01, 0.01)
    elif along <= FINAL_ALONG_M:
        sigma = (0.05, 0.04, 0.04)
    else:
        sigma = (0.10, 0.12, 0.12)
    best_U = mean
    best_cost = float("inf")

    for _ in range(CEM_ITERS):
        scored = []
        for s in range(SAMPLES):
            if s == 0:
                U = [tuple(u) for u in mean]
            else:
                U = []
                for u in mean:
                    U.append((
                        u[0] + rng.gauss(0.0, sigma[0]),
                        u[1] + rng.gauss(0.0, sigma[1]),
                        u[2] + rng.gauss(0.0, sigma[2]),
                    ))
            cost, _ = _rollout(along, yf, yl, U, climb_cap, v0=v0)
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
        # Shrink exploration each CEM iteration.
        sigma = tuple(max(0.03, 0.65 * sigma[j]) for j in range(3))

    va, vf, vl = _clip_u(along, yf, yl, *best_U[0], climb_cap)
    # Achievable nest velocity after one lag step (what the force loop can track).
    v_lag = _lag_step(v0, (va, vf, vl))

    # Consistent (p, v): advance one DT from the *current* nest-frame state.
    # Soft-capture lateral toward the nest axis (AXIS_KEEP / FINAL_KEEP) so the
    # last metre does not chase small offsets with full climb velocity. Do not
    # hard-snap to yf=yl=0 — that fought v_des and looped under the boat.
    keep = AXIS_KEEP
    if along <= FINAL_ALONG_M:
        keep = min(keep, FINAL_KEEP)
    along_ref = max(DOCK_STANDOFF_M, along + v_lag[0] * DT)
    # When off-axis in the final metre, hold along (do not lead climb).
    if _final_gate(along, yf, yl):
        along_ref = max(DOCK_STANDOFF_M, along)
    yf_ref = yf * keep + v_lag[1] * DT
    yl_ref = yl * keep + v_lag[2] * DT
    point = tuple(
        float(nest[i])
        + along_ref * down[i]
        + yf_ref * fwd[i]
        + yl_ref * left[i]
        for i in range(3)
    )
    vel = tuple(v_lag[0] * down[i] + v_lag[1] * fwd[i] + v_lag[2] * left[i] for i in range(3))
    return {
        "point": point,
        "vel": vel,
        "along": along_ref,
        "warm": best_U,
        "cost": best_cost,
        "u0": (va, vf, vl),
        "v_lag": v_lag,
        "state0": (along, yf, yl),
    }
