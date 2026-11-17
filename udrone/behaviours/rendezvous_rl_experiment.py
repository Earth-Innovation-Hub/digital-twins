#!/usr/bin/env python3
"""Self-supervised + PPO residual trials for nest-axis rendezvous.

Kinematic plant (not a live Gazebo swim): the classical rendezvous_geom
setpoints are the teacher; a low-dimensional Gaussian policy adapts six
continuous knobs with one clipped PPO-style step per round. Self-supervised
warm-start is the shipped mean; an imitation bonus pulls early samples toward
that teacher when rewards are close.

Scenarios cover varied AUV starts and a boat that is still or moving. Success
is the weld gate (range + yaw + tilt), scored with nest-axis shaping so
partial progress is usable.
"""
from __future__ import annotations

import json
import math
import random

from rendezvous_geom import (
    CONE_RANGE_M,
    DOCK_CLIMB_MPS,
    DOCK_STANDOFF_M,
    LAND_RANGE_M,
    LEARNED_BLEND,
    LEARNED_CHASE_MPS,
    LEARNED_CLOSE_RATE,
    SEARCH_DEPTH_M,
    SERVO_LOCK_CAP_M,
    axes,
    closing_range,
    cone_state,
    dock_goal,
    nest_world,
    pull_to_nest_axis,
    under_loiter_point,
    under_nest,
)

DT = 0.20
MAX_S = 160.0
TAG_BODY = (0.15, 0.0, -0.25)  # deck tag rough offset from nest in boat frame
LEVEL = (1.0, 0.0, 0.0, 0.0)

# Live defaults after wiring the 2026-11-16 PPO residual (full power).
SHIPPED = {
    "power": 1.0,
    "close_rate": LEARNED_CLOSE_RATE,
    "climb_mps": DOCK_CLIMB_MPS,
    "blend": LEARNED_BLEND,
    "lead_m": 0.0,
    "chase_mps": LEARNED_CHASE_MPS,
}

BOUNDS = {
    "power": (0.35, 1.0),
    "close_rate": (0.06, 0.28),
    "climb_mps": (0.06, 0.28),
    "blend": (0.35, 1.0),
    "lead_m": (0.0, 1.2),
    "chase_mps": (0.18, 0.55),
}
KEYS = ("power", "close_rate", "climb_mps", "blend", "lead_m", "chase_mps")

# (auv_e, auv_n, auv_z, auv_yaw, boat_ve, boat_vn, boat_yaw_rate)
# Boat starts at (10, 5, 0). Negative z is down.
SCENARIOS = (
    ("near_static", (10.0, 5.0, -1.4, 0.0, 0.0, 0.0, 0.0)),
    ("east_static", (14.0, 5.0, -1.2, 0.0, 0.0, 0.0, 0.0)),
    ("south_deep", (10.0, 1.5, -1.8, math.pi / 2, 0.0, 0.0, 0.0)),
    ("far_static", (4.0, 9.0, -1.0, -0.4, 0.0, 0.0, 0.0)),
    ("boat_east", (10.0, 5.5, -1.3, 0.0, 0.45, 0.0, 0.0)),
    ("boat_north", (11.0, 4.0, -1.5, 0.2, 0.0, 0.35, 0.0)),
    ("boat_turn", (9.5, 5.0, -1.4, 0.0, 0.25, 0.15, 0.08)),
    ("boat_fast", (12.0, 6.0, -1.2, -0.3, 0.70, 0.10, 0.0)),
)


def yaw_q(yaw: float):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def _axes(q):
    w, x, y, z = (float(v) for v in q[:4])
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )


def _tag_world(boat_pos, boat_q):
    fwd, left, up = _axes(boat_q)
    return tuple(
        float(boat_pos[i]) + TAG_BODY[0] * fwd[i] + TAG_BODY[1] * left[i] + TAG_BODY[2] * up[i]
        for i in range(3)
    )


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


def _follow(pos, dest, speed, dt):
    dx = dest[0] - pos[0]
    dy = dest[1] - pos[1]
    dz = dest[2] - pos[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-9:
        return pos, 0.0
    step = min(speed * dt, dist)
    s = step / dist
    return (pos[0] + dx * s, pos[1] + dy * s, pos[2] + dz * s), step


def _lead_point(nest, boat_q, lead_m):
    if lead_m <= 1e-9:
        return nest
    fwd = _axes(boat_q)[0]
    return tuple(nest[i] + float(lead_m) * fwd[i] for i in range(3))


def rollout(params, scenario):
    name, (ae, an, az, ayaw, bve, bvn, byaw_rate) = scenario
    boat = [10.0, 5.0, 0.0]
    boat_yaw = 0.0
    auv = [ae, an, az]
    auv_yaw = ayaw
    power = params["power"]
    speed = params["chase_mps"] * power
    close_rate = params["close_rate"] * power
    climb = params["climb_mps"] * power
    blend = params["blend"]
    lead_m = params["lead_m"]

    t = 0.0
    path = 0.0
    stage = "search"
    search_t0 = 0.0
    lock_range = None
    lock_tm = None
    dock_along = None
    dock_tm = None
    latched = False
    min_lateral = 1e9
    min_range = 1e9
    in_cone_s = 0.0
    samples = 0

    while t < MAX_S and not latched:
        boat[0] += bve * DT
        boat[1] += bvn * DT
        boat_yaw += byaw_rate * DT
        bq = yaw_q(boat_yaw)
        bp = tuple(boat)
        aq = yaw_q(auv_yaw)
        nest = nest_world(bp, bq)
        tag = _tag_world(bp, bq)
        cone = cone_state(auv, aq, bp, bq)
        min_lateral = min(min_lateral, cone["lateral"])
        min_range = min(min_range, cone["range"])
        samples += 1

        # Approximate "seeing" the deck tag: under the boat, looking up, within 4 m.
        see = cone["range"] < 4.0 and cone["along"] > 0.2 and abs(cone["lateral"]) < 2.5
        station = under_nest(bp, bq, SEARCH_DEPTH_M)
        goal = station
        if stage == "search":
            if see:
                lock_range = math.hypot(auv[0] - tag[0], auv[1] - tag[1]) + abs(auv[2] - tag[2])
                lock_range = max(LAND_RANGE_M, min(SERVO_LOCK_CAP_M, lock_range))
                lock_tm = t
                stage = "servo"
            else:
                far = math.hypot(auv[0] - nest[0], auv[1] - nest[1]) > 5.0
                if far:
                    goal = station
                else:
                    fwd, left, _up = axes(bq)
                    goal = under_loiter_point(
                        nest, station[2], fwd, left, t - search_t0,
                        r_rate=0.12 * power, omega=0.40 * power,
                    )
                goal = _lead_point(goal, bq, lead_m if (bve or bvn) else 0.0)
        if stage == "servo":
            optical = closing_range(lock_range, t - lock_tm, LAND_RANGE_M, rate=close_rate)
            # Tag-centred station then nest-axis pull (same idea as servo_station).
            raw = (tag[0], tag[1], min(tag[2] - 0.18, max(-1.80, tag[2] - optical)))
            goal = pull_to_nest_axis(raw, bp, bq, blend=blend)
            if bve or bvn or abs(byaw_rate) > 1e-6:
                goal = _lead_point(goal, bq, lead_m)
            if cone["inside"]:
                stage = "dock"
                dock_along = max(DOCK_STANDOFF_M, cone["along"])
                dock_tm = t
        if stage == "dock":
            if not cone["inside"] and cone["range"] > CONE_RANGE_M * 1.15:
                stage = "servo" if see else "search"
                if stage == "search":
                    search_t0 = t
                    lock_range = None
                goal = station
            else:
                if dock_along is None:
                    dock_along = max(DOCK_STANDOFF_M, cone["along"])
                    dock_tm = t
                goal, dock_along = dock_goal(cone, dock_along, t - dock_tm, climb_mps=climb)
                dock_tm = t
                if bve or bvn:
                    goal = _lead_point(goal, bq, 0.5 * lead_m)
                in_cone_s += DT
                if cone["weld"]:
                    latched = True

        # Align yaw toward boat when close (plates parallel proxy).
        if cone["range"] < 3.0:
            auv_yaw += 0.6 * power * ((boat_yaw - auv_yaw + math.pi) % (2 * math.pi) - math.pi) * DT

        auv, step = _follow(auv, goal, speed, DT)
        path += step
        t += DT

    cone = cone_state(auv, yaw_q(auv_yaw), tuple(boat), yaw_q(boat_yaw))
    # Shaped reward: latch >> cone time >> lateral/range progress >> time.
    if latched:
        reward = 140.0 - t + 10.0 * power  # slight preference not to crawl forever
    else:
        reward = (
            -0.35 * t
            + 25.0 * max(0.0, 2.0 - min_lateral)
            + 15.0 * max(0.0, 2.5 - min_range)
            + 0.8 * in_cone_s
            - 8.0 * abs(cone["yaw"])
            - 4.0 * cone["tilt"]
        )
        if cone["inside"]:
            reward += 20.0
    # Self-supervised imitation: stay near the teacher when not clearly better.
    teacher_dist = sum(
        ((params[k] - SHIPPED[k]) / max(1e-6, BOUNDS[k][1] - BOUNDS[k][0])) ** 2 for k in KEYS
    )
    reward -= 4.0 * teacher_dist

    return {
        "scenario": name,
        "latched": latched,
        "stage": stage,
        "t": round(t, 2),
        "path_m": round(path, 2),
        "min_lateral": round(min_lateral, 3),
        "min_range": round(min_range, 3),
        "in_cone_s": round(in_cone_s, 2),
        "final_lateral": round(cone["lateral"], 3),
        "final_range": round(cone["range"], 3),
        "reward": round(reward, 2),
        "boat_motion": abs(bve) + abs(bvn) + abs(byaw_rate) > 1e-9,
    }


def evaluate(params):
    rows = [rollout(params, sc) for sc in SCENARIOS]
    static = [r for r in rows if not r["boat_motion"]]
    moving = [r for r in rows if r["boat_motion"]]
    return {
        "reward": round(sum(r["reward"] for r in rows) / len(rows), 2),
        "latches": sum(1 for r in rows if r["latched"]),
        "static_latches": sum(1 for r in static if r["latched"]),
        "moving_latches": sum(1 for r in moving if r["latched"]),
        "n_static": len(static),
        "n_moving": len(moving),
        "mean_t": round(sum(r["t"] for r in rows) / len(rows), 2),
        "mean_min_lateral": round(sum(r["min_lateral"] for r in rows) / len(rows), 3),
        "rows": rows,
    }


def _logp(z, mu, sigma):
    var = sigma * sigma
    return sum(-0.5 * ((v - m) ** 2) / var for v, m in zip(z, mu))


def adapt(seed: int = 11, rounds: int = 8, agents: int = 2):
    """Self-supervised warm-start at shipped mean, then clipped PPO updates."""
    rng = random.Random(seed)
    mu = _encode(SHIPPED)
    sigma = 0.50
    baseline = evaluate(SHIPPED)
    trials = []
    history = []

    for rnd in range(rounds):
        batch = []
        mu_old = list(mu)
        for a in range(agents):
            z = [m + sigma * rng.gauss(0.0, 1.0) for m in mu]
            params = _decode(z)
            score = evaluate(params)
            batch.append((z, params, score, f"{chr(ord('A') + a)}"))
        mean_r = sum(item[2]["reward"] for item in batch) / len(batch)
        grad = [0.0] * len(KEYS)
        for z, _params, score, _agent in batch:
            adv = score["reward"] - mean_r
            ratio = math.exp(max(-20.0, min(20.0, _logp(z, mu, sigma) - _logp(z, mu_old, sigma))))
            # PPO clip: skip updates that would stretch the ratio the wrong way.
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
        sigma = max(0.22, sigma * 0.93)

        best = max(batch, key=lambda item: item[2]["reward"])
        history.append({
            "round": rnd + 1,
            "mean_reward": round(mean_r, 2),
            "best_reward": best[2]["reward"],
            "best_latches": best[2]["latches"],
            "sigma": round(sigma, 3),
        })
        for z, params, score, agent in batch:
            trials.append({
                "trial": len(trials) + 1,
                "round": rnd + 1,
                "agent": agent,
                "params": {k: round(params[k], 3) for k in KEYS},
                "reward": score["reward"],
                "latches": score["latches"],
                "static_latches": score["static_latches"],
                "moving_latches": score["moving_latches"],
                "mean_t": score["mean_t"],
                "mean_min_lateral": score["mean_min_lateral"],
                "rows": score["rows"],
            })

    learned = {k: round(v, 3) for k, v in _decode(mu).items()}
    final = evaluate(learned)
    return {
        "method": "self_supervised_warmstart + clipped_ppo",
        "knobs": list(KEYS),
        "shipped": SHIPPED,
        "baseline": {
            "reward": baseline["reward"],
            "latches": baseline["latches"],
            "static_latches": baseline["static_latches"],
            "moving_latches": baseline["moving_latches"],
            "mean_t": baseline["mean_t"],
            "mean_min_lateral": baseline["mean_min_lateral"],
            "rows": baseline["rows"],
        },
        "history": history,
        "trials": trials,
        "learned": learned,
        "learned_eval": {
            "reward": final["reward"],
            "latches": final["latches"],
            "static_latches": final["static_latches"],
            "moving_latches": final["moving_latches"],
            "mean_t": final["mean_t"],
            "mean_min_lateral": final["mean_min_lateral"],
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
