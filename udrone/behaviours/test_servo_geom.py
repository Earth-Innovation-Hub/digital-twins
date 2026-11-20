"""The shared lock servos, relocks, and searches without a camera driver."""
import math

from servo_geom import (
    DISAGREE_M,
    FRONT_CLEAR_M,
    RELOCK_STEP_M,
    abort_pass,
    aim_point,
    axis_miss,
    choose_eye,
    nadir_body,
    phase,
    pick_camera,
    prediction_disagrees,
    relock_leg,
    slide_aim,
)
from servo_tune import score, tune
from track_geom import CAM_Z, STANDOFF_M, station_under


def _close(a, b, tol=1e-9):
    assert abs(float(a) - float(b)) < tol, (a, b)


def main():
    assert phase(0.1) == "servo"
    assert phase(0.5) == "servo"
    assert phase(1.5) == "relock"
    assert phase(4.0) == "relock"
    assert phase(4.1) == "search"

    assert pick_camera((0, 0, -1.2), (0.2, 0.1, -0.2)) == "up"
    assert pick_camera((5, 4, -6.0), (5.3, 4.2, -9.2)) == "down"
    assert pick_camera((0, 0, -4.6), (4.0, 0.5, -4.7)) == "front"
    assert pick_camera((5, 4, -4.0), (7.5, 6.0, -9.2)) == "front"
    # The hoop stays on the front camera even when the ring is below the hull.
    assert pick_camera((6, 0, -1.0), (6, 0, -4.6), prefer="front") == "front"

    tag = (6.0, 1.0, 0.4)
    hold = station_under(tag, STANDOFF_M, CAM_Z)
    aim = aim_point(tag, "up", STANDOFF_M, cam_offset=CAM_Z)
    for i in range(3):
        _close(hold[i], aim[i])
    above = aim_point((5, 4, -9.2), "down", 1.5, cam_offset=0.06)
    _close(above[0], 5.0)
    _close(above[1], 4.0)
    _close(above[2], -9.2 + 1.56)
    short = aim_point((6, 0, -4.6), "front", 1.62, approach=(-1.0, 0.0, 0.0))
    _close(short[0], 6.0 - 1.62)
    _close(short[1], 0.0)
    _close(short[2], -4.6)

    slid = slide_aim((0, 0, -1), (0, 1, 0), (0, 0, 1), 0.2, 0.0)
    assert slid[1] < 0.0 and abs(slid[0]) < 1e-9

    vehicle = (0.0, 0.0, -1.2)
    feature = (3.0, 1.2, -0.15)
    leg = relock_leg(vehicle, feature, "up")
    _close(leg[2], vehicle[2])
    assert axis_miss(leg, feature, "up") < axis_miss(vehicle, feature, "up")
    pos = vehicle
    for _ in range(4):
        pos = relock_leg(pos, feature, "up")
    assert axis_miss(pos, feature, "up") < 0.35
    _close(pos[2], vehicle[2])

    bed = (5.0, 4.0, -4.0)
    cube = (6.4, 5.1, -9.2)
    pos = bed
    for _ in range(6):
        pos = relock_leg(pos, cube, "down")
    assert axis_miss(pos, cube, "down") <= RELOCK_STEP_M
    _close(pos[2], bed[2])

    hoop = (6.0, 0.0, -4.626)
    pos = (1.0, 2.0, -4.6)
    for _ in range(8):
        pos = relock_leg(pos, hoop, "front")
        assert math.dist(pos, hoop) >= FRONT_CLEAR_M - 1e-6
    assert axis_miss(pos, hoop, "front") < 0.05

    assert abort_pass(2.0, inside=False)            # default PASS_LOST_S=1.5
    assert not abort_pass(0.4, inside=False)
    assert not abort_pass(3.0, inside=True)
    assert not abort_pass(2.0, inside=False, lost_s=4.0)
    assert abort_pass(4.5, inside=False, lost_s=4.0)

    assert prediction_disagrees((0, 0, -1), (1.0, 0, -1))
    assert not prediction_disagrees((0, 0, -1), (DISAGREE_M * 0.5, 0, -1))

    assert choose_eye(False, True, (5, 4, -6), (5.2, 4.1, -9.2)) == "down"
    assert choose_eye(True, True, (0, 0, -4), (4, 0, -4.2)) == "front"
    assert choose_eye(True, False, None, None) == "front"
    assert choose_eye(False, False, None, None) is None
    # A dark front frame while the downward camera still has the tag is not a search.
    assert choose_eye(False, True, None, None) == "down"

    body = nadir_body(0.0, 0.0, 2.0)
    _close(body[0], 0.0)
    _close(body[1], 0.0)
    _close(body[2], -2.0)
    right = nadir_body(0.25, 0.0, 2.0)
    _close(right[1], -0.5)
    # Image up is ahead of the hull. Image down is aft.
    ahead = nadir_body(0.0, -0.25, 2.0)
    _close(ahead[0], 0.5)
    aft = nadir_body(0.0, 0.25, 2.0)
    _close(aft[0], -0.5)

    shipped = score(RELOCK_STEP_M)
    assert shipped > score(0.15)
    step, tuned, shipped_again = tune()
    assert abs(shipped - shipped_again) < 1e-9
    assert tuned + 1e-6 >= shipped
    if tuned > shipped + 0.5 and abs(step - RELOCK_STEP_M) > 0.15:
        raise SystemExit(
            f"retune RELOCK_STEP_M {RELOCK_STEP_M:.2f} -> {step:.2f} "
            f"(score {shipped:.2f} -> {tuned:.2f})"
        )
    print(
        f"servo geometry ok step={RELOCK_STEP_M:.2f} "
        f"score={shipped:.2f} tuned={step:.2f}/{tuned:.2f}"
    )


if __name__ == "__main__":
    main()
