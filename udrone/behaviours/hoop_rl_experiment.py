#!/usr/bin/env python3
"""Ten adaptive trials of the Kiwanis hoop search and fly-through.

This is a kinematic rollout of the shipped planner in hoop_geom, not a live
swim. Two agents sample a shared Gaussian policy. After every pair of trials
the mean takes one clipped step, the same trust region PPO uses, on four
continuous knobs: search speed, cell size, look-leg stand-off, and the
centerline lead-in. A network cannot learn in ten episodes, so the policy
is this four-dimensional mean.

A trial is one parameter sample scored on three arming poses. The hoop is
"seen" when its centre stays in the front-camera wedge for the planner's
look dwell. The pass test is hoop_geom.in_opening.
"""
from __future__ import annotations

import json
import math
import random

from hoop_geom import (
    DEPTH_HI,
    DEPTH_LO,
    HOOP_CENTER,
    LOOK_S,
    SEARCH_HITS,
    SEARCH_Z,
    VIEW_NEAR_M,
    WEDGE_FAR_M,
    WEDGE_HALF_RAD,
    WEDGE_NEAR_M,
    CoverageMap,
    fly_trajectory,
    in_opening,
    look_leg,
)

DT = 0.25
MAX_SEARCH_S = 90.0
MAX_THROUGH_S = 35.0
DESCEND_MPS = 0.20

# (east, north, z, yaw). The ring is at (6, 0, -4.626), axis east-west.
STARTS = (
    (0.0, 0.0, -0.30, 0.0),
    (0.0, 4.0, -0.30, math.pi / 2),
    (-2.0, -3.0, -0.30, math.pi),
)

# Shipped run_hoop.py / hoop_geom.py values.
SHIPPED = {
    "speed": 0.32,
    "cell_m": 2.07,
    "stand_m": 3.31,
    "lead_m": 1.62,
}

BOUNDS = {
    "speed": (0.16, 0.42),
    "cell_m": (1.5, 3.5),
    "stand_m": (2.4, 5.0),
    "lead_m": (1.2, 3.6),
}
KEYS = ("speed", "cell_m", "stand_m", "lead_m")


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _decode(z):
    params = {}
    for key, value in zip(KEYS, z):
        lo, hi = BOUNDS[key]
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        params[key] = mid + half * math.tanh(float(value))
    return params


def _encode_shipped():
    encoded = []
    for key in KEYS:
        lo, hi = BOUNDS[key]
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        unit = max(-0.999, min(0.999, (SHIPPED[key] - mid) / half))
        encoded.append(0.5 * math.log((1 + unit) / (1 - unit)))
    return encoded


def _hoop_visible(east, north, z, yaw) -> bool:
    if not (DEPTH_LO <= z <= DEPTH_HI):
        return False
    de = HOOP_CENTER[0] - east
    dn = HOOP_CENTER[1] - north
    dist = math.hypot(de, dn)
    if dist < WEDGE_NEAR_M or dist > WEDGE_FAR_M:
        return False
    bearing = math.atan2(dn, de)
    return abs(_wrap(bearing - yaw)) <= WEDGE_HALF_RAD


def _crossed(prev, pos) -> bool:
    """True when the segment passes through the opening, not only a sample."""
    if in_opening(pos, HOOP_CENTER):
        return True
    if prev is None:
        return False
    along0 = prev[0] - HOOP_CENTER[0]
    along1 = pos[0] - HOOP_CENTER[0]
    if along0 * along1 > 0.0:
        return False
    span = along1 - along0
    if abs(span) < 1e-6:
        return False
    frac = -along0 / span
    mid = tuple(prev[i] + frac * (pos[i] - prev[i]) for i in range(3))
    return in_opening(mid, HOOP_CENTER, plane_tol=0.05)


def _follow(pos, dest, speed, dt):
    dx = dest[0] - pos[0]
    dy = dest[1] - pos[1]
    dz = dest[2] - pos[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-6:
        return pos, 0.0, math.atan2(dy, dx)
    step = min(speed * dt, dist)
    scale = step / dist
    nxt = (pos[0] + dx * scale, pos[1] + dy * scale, pos[2] + dz * scale)
    return nxt, step, math.atan2(dy, dx)


def rollout(params, start):
    speed = params["speed"]
    stand = params["stand_m"]
    lead = params["lead_m"]
    east, north, z, yaw = start
    grid = CoverageMap(east, north, res=params["cell_m"])
    t = 0.0
    path = 0.0
    dwell = 0.0
    seen = False
    wps = None
    leg_i = 0
    goal = None
    pos = (east, north, z)

    while t < MAX_SEARCH_S:
        z = max(SEARCH_Z, z - DESCEND_MPS * DT)
        if wps is None or leg_i >= len(wps):
            nxt = grid.nearest_open(pos[0], pos[1], yaw, min_dist=WEDGE_NEAR_M)
            if nxt is None:
                nxt = grid.nearest_open(pos[0], pos[1], yaw)
            if nxt is None:
                break
            ce, cn, goal, _dist = nxt
            wps = look_leg(pos[0], pos[1], ce, cn, stand=stand, near=VIEW_NEAR_M)
            leg_i = 0
        dest = (wps[leg_i][0], wps[leg_i][1], SEARCH_Z)
        nxt_pos, step, yaw = _follow((pos[0], pos[1], z), dest, speed, DT)
        pos = nxt_pos
        z = pos[2]
        path += step
        t += DT
        if math.hypot(dest[0] - pos[0], dest[1] - pos[1]) < 0.40:
            leg_i += 1
        grid.mark_wedge(pos[0], pos[1], yaw, z, t)
        if goal is not None and grid.hits.get(goal, 0) >= SEARCH_HITS and leg_i >= len(wps):
            wps = None
        if _hoop_visible(pos[0], pos[1], z, yaw):
            dwell += DT
            seen = True
        else:
            dwell = 0.0
        if dwell >= LOOK_S:
            break

    search_s = t
    passed = False
    if dwell >= LOOK_S:
        heading = (math.cos(yaw), math.sin(yaw))
        pts, _axis = fly_trajectory(
            pos, heading, HOOP_CENTER, lead_m=lead, exit_m=2.0, through=True,
        )
        prev = pos
        cursor = pos
        for dest in pts[1:]:
            guard = 0
            while t < search_s + MAX_THROUGH_S and guard < 400:
                guard += 1
                cursor, step, yaw = _follow(cursor, dest, speed, DT)
                path += step
                t += DT
                if _crossed(prev, cursor):
                    passed = True
                    break
                prev = cursor
                if math.dist(cursor, dest) < 0.30:
                    break
            if passed:
                break

    if passed:
        reward = 120.0 - t
    elif seen:
        reward = 20.0 - search_s
    else:
        reward = -search_s
    return {
        "passed": passed,
        "seen": seen,
        "search_s": round(search_s, 2),
        "total_s": round(t, 2),
        "path_m": round(path, 2),
        "reward": round(reward, 2),
    }


def evaluate(params):
    rows = [rollout(params, start) for start in STARTS]
    reward = sum(row["reward"] for row in rows) / len(rows)
    return {
        "reward": round(reward, 2),
        "passes": sum(1 for row in rows if row["passed"]),
        "mean_s": round(sum(row["total_s"] for row in rows) / len(rows), 2),
        "mean_path_m": round(sum(row["path_m"] for row in rows) / len(rows), 2),
        "starts": rows,
    }


def _logp(z, mu, sigma):
    var = sigma * sigma
    acc = 0.0
    for value, mean in zip(z, mu):
        acc += -0.5 * ((value - mean) ** 2) / var
    return acc


def adapt(seed=7):
    rng = random.Random(seed)
    mu = _encode_shipped()
    sigma = 0.55
    baseline = evaluate(SHIPPED)
    trials = []
    agents = ("A", "B")
    for rnd in range(5):
        batch = []
        mu_old = list(mu)
        for agent in agents:
            z = [mean + sigma * rng.gauss(0.0, 1.0) for mean in mu]
            params = _decode(z)
            score = evaluate(params)
            batch.append((z, params, score, agent))
        mean_reward = sum(item[2]["reward"] for item in batch) / len(batch)
        # One clipped step. With two samples the advantages sum to zero, so
        # the mean moves toward the better agent and away from the worse one.
        grad = [0.0] * len(KEYS)
        for z, _params, score, _agent in batch:
            adv = score["reward"] - mean_reward
            log_new = _logp(z, mu, sigma)
            log_old = _logp(z, mu_old, sigma)
            ratio = math.exp(max(-20.0, min(20.0, log_new - log_old)))
            if ratio > 1.2 and adv > 0.0:
                continue
            if ratio < 0.8 and adv < 0.0:
                continue
            weight = max(0.8, min(1.2, ratio)) * adv
            for i, (value, mean) in enumerate(zip(z, mu)):
                grad[i] += weight * (value - mean) / (sigma * sigma)
        for i in range(len(mu)):
            step = 0.15 * grad[i] / len(batch)
            step = max(-0.35, min(0.35, step))
            mu[i] = mu_old[i] + step
        sigma = max(0.28, sigma * 0.92)
        for z, params, score, agent in batch:
            trials.append({
                "trial": len(trials) + 1,
                "round": rnd + 1,
                "agent": agent,
                "params": {key: round(params[key], 3) for key in KEYS},
                "reward": score["reward"],
                "passes": score["passes"],
                "mean_s": score["mean_s"],
                "mean_path_m": score["mean_path_m"],
                "starts": score["starts"],
            })
    learned = _decode(mu)
    learned = {key: round(learned[key], 3) for key in KEYS}
    final = evaluate(learned)
    return {
        "shipped": SHIPPED,
        "baseline": baseline,
        "trials": trials,
        "learned": learned,
        "learned_eval": final,
        "sigma_final": round(sigma, 3),
    }


def main():
    report = adapt()
    print(json.dumps(report))


if __name__ == "__main__":
    main()
