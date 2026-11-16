"""Clipped-mean tune of the shared relock step. ROS-free.

Two samples draw a step length. The better one pulls the mean by a clipped
step, the same trust region the hoop search tuner uses. The score is how
fast a kinematic relock puts the feature on the camera axis without
climbing, diving, or flying into it. A cone that mis-classifies an obvious
overhead or ahead feature is penalised, so the tune cannot trade the
camera choice away.
"""
from __future__ import annotations

import math
import random

from servo_geom import NADIR_CONE, RELOCK_STEP_M, axis_miss, pick_camera, relock_leg

STEP_BOUNDS = (0.40, 2.40)
CASES = (
    ((0.0, 0.0, -1.2), (3.0, 1.2, -0.15), "up"),
    ((5.0, 4.0, -3.0), (6.5, 5.5, -9.2), "down"),
    ((1.0, 2.0, -4.6), (6.0, 0.0, -4.626), "front"),
    ((0.0, 0.0, -1.0), (0.4, -0.3, -0.20), "up"),
    ((2.0, -1.0, -2.0), (2.2, -0.7, -8.5), "down"),
)
# (vehicle, feature, expected camera). These pin the nadir cone.
EYES = (
    ((0.0, 0.0, -1.2), (0.2, 0.1, -0.2), "up"),
    ((5.0, 4.0, -6.0), (5.3, 4.2, -9.2), "down"),
    ((0.0, 0.0, -4.6), (4.0, 0.5, -4.7), "front"),
    ((5.0, 4.0, -4.0), (7.5, 6.0, -9.2), "front"),
)


def _clip(step):
    lo, hi = STEP_BOUNDS
    return min(hi, max(lo, float(step)))


def score(step_m: float, cone: float = NADIR_CONE) -> float:
    """Higher is a faster relock that still respects the camera axes."""
    total = 0.0
    saved = None
    try:
        import servo_geom as geom
        saved = geom.NADIR_CONE
        geom.NADIR_CONE = float(cone)
        for vehicle, feature, camera in CASES:
            pos = tuple(vehicle)
            start = axis_miss(pos, feature, camera)
            for _ in range(6):
                pos = relock_leg(pos, feature, camera, step_m=step_m)
            end = axis_miss(pos, feature, camera)
            total += max(0.0, start - end)
            if end <= 0.35:
                total += 2.0
            else:
                total -= end
            if camera in ("up", "down") and abs(pos[2] - vehicle[2]) > 1e-6:
                total -= 8.0
            if camera == "front" and math.dist(pos, feature) < 0.80:
                total -= 8.0
        for vehicle, feature, expect in EYES:
            if pick_camera(vehicle, feature) != expect:
                total -= 6.0
    finally:
        if saved is not None:
            import servo_geom as geom
            geom.NADIR_CONE = saved
    return total


def tune(seed: float = RELOCK_STEP_M, rounds: int = 8, rng: random.Random | None = None):
    """Eight clipped updates. Returns (step, score, shipped_score)."""
    draw = rng or random.Random(7)
    mean = float(seed)
    sigma = 0.35
    best_step, best_score = mean, score(mean)
    shipped = best_score
    for _ in range(int(rounds)):
        samples = []
        for _agent in range(2):
            step = _clip(draw.gauss(mean, sigma))
            samples.append((step, score(step)))
        samples.sort(key=lambda item: item[1], reverse=True)
        winner, winner_score = samples[0]
        # Trust region: move at most 0.25 m toward the better sample.
        delta = max(-0.25, min(0.25, winner - mean))
        mean = _clip(mean + delta)
        sigma = max(0.08, sigma * 0.85)
        if winner_score > best_score:
            best_step, best_score = winner, winner_score
        if score(mean) > best_score:
            best_step, best_score = mean, score(mean)
    return best_step, best_score, shipped


def main():
    step, tuned, shipped = tune()
    print(
        f"relock tune step={step:.2f} score={tuned:.2f} "
        f"shipped={RELOCK_STEP_M:.2f} shipped_score={shipped:.2f}"
    )
    if tuned > shipped + 0.5 and abs(step - RELOCK_STEP_M) > 0.15:
        raise SystemExit(
            f"RELOCK_STEP_M {RELOCK_STEP_M:.2f} is behind the tuned step {step:.2f}"
        )
    print("relock tune kept the shipped step")


if __name__ == "__main__":
    main()
