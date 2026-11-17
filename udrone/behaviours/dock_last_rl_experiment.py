#!/usr/bin/env python3
"""RL-style trials for the rendezvous *last metre* (dock MPC).

Kinematic nest-frame plant matching ``dock_mpc``'s lagged CEM model. Focuses on
small lateral offsets near the nest where live docks overshoot: climb too fast
while still a few centimetres off-axis.

Knobs (continuous): climb_mps, v_lat_max, lat_tau, final_along_m, final_lat_m,
final_climb_cap, axis_keep. Self-supervised warm-start at shipped means, then
clipped PPO-style updates over a few rounds.
"""
from __future__ import annotations

import json
import math
import random

from rendezvous_geom import (
    DOCK_STANDOFF_M,
    WELD_DIST_M,
    cone_state,
)

import dock_mpc

DT = dock_mpc.DT
MAX_S = 25.0
LEVEL = (1.0, 0.0, 0.0, 0.0)
BOAT = (10.0, 5.0, 0.0)

# Last-metre starts: (fwd_offset_m, left_offset_m, along_m)
# along is metres below the nest on boat −Z.
SCENARIOS = (
    ("on_axis_1.0", (0.00, 0.00, 1.00)),
    ("tiny_fwd_0.8", (0.08, 0.00, 0.80)),
    ("tiny_left_0.7", (0.00, 0.10, 0.70)),
    ("small_diag_0.9", (0.12, 0.10, 0.90)),
    ("small_diag_1.2", (0.18, -0.12, 1.20)),
    ("offset_1.5", (0.25, 0.15, 1.50)),
    ("offset_0.55", (0.06, 0.08, 0.55)),
    ("wide_1.1", (0.30, 0.00, 1.10)),
)

# Bias search toward slower climb / tighter last-metre (user: small offsets +
# too much velocity). Upper climb bound kept below the prior 0.21 live default.
SHIPPED = {
    "climb_mps": 0.13,
    "v_lat_max": 0.33,
    "lat_tau": 0.81,
    "final_along_m": 1.12,
    "final_lat_m": 0.05,
    "final_climb_cap": 0.08,
    "axis_keep": 0.45,
}
# FINAL_VLAT_CAP tracks climb crawl (kept on dock_mpc; not a free knob).

BOUNDS = {
    "climb_mps": (0.10, 0.20),
    "v_lat_max": (0.24, 0.40),
    "lat_tau": (0.45, 0.95),
    "final_along_m": (0.70, 1.15),
    "final_lat_m": (0.045, 0.09),
    "final_climb_cap": (0.05, 0.12),
    "axis_keep": (0.20, 0.55),
}
KEYS = tuple(SHIPPED.keys())


def _decode(z):
    params = {}
    for key, value in zip(KEYS, z):
        lo, hi = BOUNDS[key]
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        params[key] = mid + half * math.tanh(float(value))
    return params


def _encode(params):
    encoded = []
    for key in KEYS:
        lo, hi = BOUNDS[key]
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        unit = max(-0.999, min(0.999, (params[key] - mid) / half))
        encoded.append(0.5 * math.log((1 + unit) / (1 - unit)))
    return encoded


def _pos_from_nest(fwd_m, left_m, along_m):
    # Level boat at BOAT: down=(0,0,-1), fwd=(1,0,0), left=(0,1,0)
    return (
        BOAT[0] + float(fwd_m),
        BOAT[1] + float(left_m),
        BOAT[2] - float(along_m),
    )


def _apply_knobs(params):
    """Monkey-patch dock_mpc module knobs for one rollout."""
    dock_mpc.V_LAT_MAX = float(params["v_lat_max"])
    dock_mpc.FINAL_ALONG_M = float(params["final_along_m"])
    dock_mpc.FINAL_LAT_M = float(params["final_lat_m"])
    dock_mpc.FINAL_CLIMB_CAP = float(params["final_climb_cap"])
    dock_mpc.FINAL_VLAT_CAP = min(
        float(params["v_lat_max"]),
        max(0.10, 1.6 * float(params["final_climb_cap"])),
    )
    dock_mpc.AXIS_KEEP = float(params["axis_keep"])
    dock_mpc.LAT_TAU = float(params["lat_tau"])


def rollout(params, scenario):
    name, (fwd0, left0, along0) = scenario
    _apply_knobs(params)
    p = _pos_from_nest(fwd0, left0, along0)
    warm = None
    v_state = (0.0, 0.0, 0.0)
    t = 0.0
    path = 0.0
    peak_speed = 0.0
    peak_climb = 0.0
    min_lat = 1e9
    weld = False
    exited = False
    samples = 0

    while t < MAX_S and not weld:
        cone = {**cone_state(p, LEVEL, BOAT, LEVEL), "pos": p}
        if not cone["inside"] and cone["range"] > 1.2:
            exited = True
            break
        step = dock_mpc.plan(
            cone, climb_mps=params["climb_mps"], warm=warm, v_state=v_state,
        )
        warm = step["warm"]
        v_state = step["v_lag"]
        va, vf, vl = v_state
        speed = math.sqrt(va * va + vf * vf + vl * vl)
        peak_speed = max(peak_speed, speed)
        peak_climb = max(peak_climb, abs(va))
        # Integrate nest velocity in world (level boat).
        p = (p[0] + vf * DT, p[1] + vl * DT, p[2] - va * DT)
        path += speed * DT
        t += DT
        samples += 1
        lat = math.hypot(p[0] - BOAT[0], p[1] - BOAT[1])
        along = -(p[2] - BOAT[2])
        min_lat = min(min_lat, lat)
        # Tight weld proxy: last-metre accuracy, not a fast near-miss.
        if (
            abs(along - DOCK_STANDOFF_M) < 0.10
            and lat <= 0.06
            and along <= max(DOCK_STANDOFF_M + 0.10, WELD_DIST_M)
        ):
            weld = True

    cone = {**cone_state(p, LEVEL, BOAT, LEVEL), "pos": p}
    lat_f = math.hypot(p[0] - BOAT[0], p[1] - BOAT[1])
    along_f = -(p[2] - BOAT[2])
    # Reward: accuracy + low last-metre velocity first; time is secondary.
    if weld:
        reward = (
            140.0
            - 1.2 * t
            - 220.0 * lat_f
            - 60.0 * abs(along_f - DOCK_STANDOFF_M)
            - 90.0 * peak_climb
            - 45.0 * peak_speed
        )
    else:
        reward = (
            -0.5 * t
            + 55.0 * max(0.0, 0.20 - min_lat)
            + 25.0 * max(0.0, 0.45 - abs(along_f - DOCK_STANDOFF_M))
            - 90.0 * lat_f
            - 70.0 * peak_climb
            - 35.0 * peak_speed
            - 40.0 * (1.0 if exited else 0.0)
        )
        if cone["inside"]:
            reward += 12.0
    teacher = sum(
        ((params[k] - SHIPPED[k]) / max(1e-6, BOUNDS[k][1] - BOUNDS[k][0])) ** 2
        for k in KEYS
    )
    reward -= 1.5 * teacher
    return {
        "scenario": name,
        "weld": weld,
        "exited": exited,
        "t": round(t, 2),
        "path_m": round(path, 2),
        "final_lat": round(lat_f, 4),
        "final_along": round(along_f, 4),
        "min_lat": round(min_lat, 4),
        "peak_speed": round(peak_speed, 3),
        "peak_climb": round(peak_climb, 3),
        "reward": round(reward, 2),
    }


def evaluate(params):
    rows = [rollout(params, sc) for sc in SCENARIOS]
    return {
        "reward": round(sum(r["reward"] for r in rows) / len(rows), 2),
        "welds": sum(1 for r in rows if r["weld"]),
        "exits": sum(1 for r in rows if r["exited"]),
        "mean_t": round(sum(r["t"] for r in rows) / len(rows), 2),
        "mean_final_lat": round(sum(r["final_lat"] for r in rows) / len(rows), 4),
        "mean_peak_climb": round(sum(r["peak_climb"] for r in rows) / len(rows), 3),
        "mean_peak_speed": round(sum(r["peak_speed"] for r in rows) / len(rows), 3),
        "rows": rows,
    }


def _logp(z, mu, sigma):
    var = sigma * sigma
    return sum(-0.5 * ((v - m) ** 2) / var for v, m in zip(z, mu))


def adapt(seed: int = 17, rounds: int = 7, agents: int = 4):
    rng = random.Random(seed)
    # Ensure dock_mpc exposes the knobs (defaults if not yet patched).
    for k, v in (
        ("FINAL_ALONG_M", SHIPPED["final_along_m"]),
        ("FINAL_LAT_M", SHIPPED["final_lat_m"]),
        ("FINAL_CLIMB_CAP", SHIPPED["final_climb_cap"]),
        ("AXIS_KEEP", SHIPPED["axis_keep"]),
        ("LAT_TAU", SHIPPED["lat_tau"]),
        ("V_LAT_MAX", SHIPPED["v_lat_max"]),
    ):
        setattr(dock_mpc, k, v)

    mu = _encode(SHIPPED)
    sigma = 0.50
    baseline = evaluate(SHIPPED)
    history = []
    trials = []
    elite_params = dict(SHIPPED)
    elite_score = baseline

    for rnd in range(rounds):
        batch = []
        mu_old = list(mu)
        # Always re-evaluate current elite + shipped-biased samples.
        batch.append((_encode(elite_params), dict(elite_params), evaluate(elite_params)))
        for a in range(agents):
            z = [m + sigma * rng.gauss(0.0, 1.0) for m in mu]
            params = _decode(z)
            score = evaluate(params)
            batch.append((z, params, score))
        mean_r = sum(item[2]["reward"] for item in batch) / len(batch)
        grad = [0.0] * len(KEYS)
        for z, _params, score in batch:
            adv = score["reward"] - mean_r
            ratio = math.exp(max(-20.0, min(20.0, _logp(z, mu, sigma) - _logp(z, mu_old, sigma))))
            if ratio > 1.2 and adv > 0.0:
                continue
            if ratio < 0.8 and adv < 0.0:
                continue
            weight = max(0.8, min(1.2, ratio)) * adv
            for i, (value, mean) in enumerate(zip(z, mu)):
                grad[i] += weight * (value - mean) / (sigma * sigma)
        for i in range(len(mu)):
            step = 0.12 * grad[i] / max(1, len(batch))
            step = max(-0.40, min(0.40, step))
            mu[i] = mu_old[i] + step
        sigma = max(0.18, sigma * 0.90)

        best = max(
            batch,
            key=lambda item: (
                item[2]["welds"],
                item[2]["reward"],
                -item[2]["mean_final_lat"],
                -item[2]["mean_peak_climb"],
            ),
        )
        if (
            best[2]["welds"] > elite_score["welds"]
            or (
                best[2]["welds"] == elite_score["welds"]
                and best[2]["reward"] >= elite_score["reward"]
            )
        ):
            elite_params = dict(best[1])
            elite_score = best[2]
        # Snap mu halfway toward the running elite (CEM-style).
        elite_z = _encode(elite_params)
        mu = [0.55 * m + 0.45 * e for m, e in zip(mu, elite_z)]

        history.append({
            "round": rnd + 1,
            "mean_reward": round(mean_r, 2),
            "best_reward": best[2]["reward"],
            "best_welds": best[2]["welds"],
            "best_final_lat": best[2]["mean_final_lat"],
            "best_peak_climb": best[2]["mean_peak_climb"],
            "sigma": round(sigma, 3),
        })
        for z, params, score in batch:
            trials.append({
                "trial": len(trials) + 1,
                "round": rnd + 1,
                "params": {k: round(params[k], 3) for k in KEYS},
                "reward": score["reward"],
                "welds": score["welds"],
                "mean_final_lat": score["mean_final_lat"],
                "mean_peak_climb": score["mean_peak_climb"],
                "mean_peak_speed": score["mean_peak_speed"],
                "rows": score["rows"],
            })

    learned = {k: round(float(elite_params[k]), 3) for k in KEYS}
    final = evaluate(learned)
    return {
        "method": "self_supervised_warmstart + clipped_ppo",
        "focus": "rendezvous last-metre dock_mpc (small offsets, velocity)",
        "knobs": list(KEYS),
        "shipped": SHIPPED,
        "baseline": {
            "reward": baseline["reward"],
            "welds": baseline["welds"],
            "mean_final_lat": baseline["mean_final_lat"],
            "mean_peak_climb": baseline["mean_peak_climb"],
            "mean_peak_speed": baseline["mean_peak_speed"],
            "rows": baseline["rows"],
        },
        "history": history,
        "trials": trials,
        "learned": learned,
        "learned_eval": {
            "reward": final["reward"],
            "welds": final["welds"],
            "exits": final["exits"],
            "mean_t": final["mean_t"],
            "mean_final_lat": final["mean_final_lat"],
            "mean_peak_climb": final["mean_peak_climb"],
            "mean_peak_speed": final["mean_peak_speed"],
            "rows": final["rows"],
        },
        "sigma_final": round(sigma, 3),
        "n_scenarios": len(SCENARIOS),
        "rounds": rounds,
        "agents_per_round": agents,
    }


def main():
    report = adapt()
    print(json.dumps(report))


if __name__ == "__main__":
    main()
