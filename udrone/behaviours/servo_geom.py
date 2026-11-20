"""Shared visual-servo lock for the hoop, the bed tag, and the deck tag.

ROS-free. A lock is one sighting. Its age picks the next move:

  servo   the fix is fresh, so fly the aim point
  relock  the fix is stale but recent, so take a short leg that puts it
          back on a camera axis
  search  there is no usable fix, so the caller runs its own pattern

The camera comes from where the feature sits relative to the hull. The
upward camera looks along hull +Z, the downward camera along hull -Z, and
the front camera along hull +X. Image +x is hull -Y on all three. On the
upward camera image +y is hull -X. On the downward camera image +y is
hull -X.
"""
from __future__ import annotations

import math

FRESH_S = 0.50
RELOCK_S = 4.0
RELOCK_STEP_M = 1.20
NADIR_CONE = 0.55       # horizontal / |vertical| at which up or down wins
DISAGREE_M = 0.50       # boat prediction this far from the last visual fix
PASS_LOST_S = 1.5       # a through-pass this stale, before the opening, aborts
FRONT_CLEAR_M = 0.80    # a front relock stops this short of the feature


def phase(age_s, fresh: float = FRESH_S, relock: float = RELOCK_S) -> str:
    """servo, relock, or search from how long the sighting has been dark."""
    age = max(0.0, float(age_s))
    if age <= float(fresh):
        return "servo"
    if age <= float(relock):
        return "relock"
    return "search"


def pick_camera(vehicle, feature, prefer: str | None = None) -> str:
    """front, down, or up. prefer 'front' keeps the hoop on the forward camera."""
    if prefer == "front":
        return "front"
    dx = float(feature[0]) - float(vehicle[0])
    dy = float(feature[1]) - float(vehicle[1])
    dz = float(feature[2]) - float(vehicle[2])
    horiz = math.hypot(dx, dy)
    if abs(dz) < 0.20:
        return "front"
    if horiz <= NADIR_CONE * abs(dz):
        return "up" if dz > 0.0 else "down"
    return "front"


def aim_point(feature, camera: str, standoff: float, cam_offset: float = 0.0, approach=None):
    """Hull origin that puts `camera` `standoff` metres from the feature.

    `approach` is the world direction from the feature toward the side the
    front camera arrives on. Up and down ignore it and hold the feature's
    east and north.
    """
    stand = float(standoff) + float(cam_offset)
    if camera == "up":
        return (float(feature[0]), float(feature[1]), float(feature[2]) - stand)
    if camera == "down":
        return (float(feature[0]), float(feature[1]), float(feature[2]) + stand)
    if approach is None:
        approach = (1.0, 0.0, 0.0)
    norm = math.sqrt(sum(float(v) * float(v) for v in approach)) or 1.0
    unit = tuple(float(v) / norm for v in approach)
    return tuple(float(feature[i]) + stand * unit[i] for i in range(3))


def slide_aim(aim, left, up, err_x: float, err_y: float, gain: float = 0.35, cap: float = 0.28):
    """Move the aim so a feature right or below centre slides toward the axis.

    Positive err_x is image-right, which is hull -Y, so the aim shifts along
    -left. Positive err_y shifts along up.
    """
    shift_x = max(-cap, min(cap, float(gain) * float(err_x)))
    shift_y = max(-cap, min(cap, float(gain) * float(err_y)))
    return tuple(
        float(aim[i]) - shift_x * float(left[i]) + shift_y * float(up[i])
        for i in range(3)
    )


def relock_leg(vehicle, feature, camera: str, step_m: float = RELOCK_STEP_M):
    """One waypoint that puts `feature` on `camera`'s axis.

    Up and down step horizontally and hold depth, so a relock cannot climb
    into the boat or dive into the bed. Front steps toward the feature in
    three dimensions and stops FRONT_CLEAR_M short of it.
    """
    step = max(0.05, float(step_m))
    if camera in ("up", "down"):
        dx = float(feature[0]) - float(vehicle[0])
        dy = float(feature[1]) - float(vehicle[1])
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return (float(vehicle[0]), float(vehicle[1]), float(vehicle[2]))
        scale = min(1.0, step / dist)
        return (
            float(vehicle[0]) + scale * dx,
            float(vehicle[1]) + scale * dy,
            float(vehicle[2]),
        )
    delta = tuple(float(feature[i]) - float(vehicle[i]) for i in range(3))
    dist = math.sqrt(sum(v * v for v in delta))
    if dist < 1e-6:
        return tuple(float(v) for v in vehicle)
    travel = min(step, max(0.0, dist - FRONT_CLEAR_M))
    scale = travel / dist
    return tuple(float(vehicle[i]) + scale * delta[i] for i in range(3))


def axis_miss(vehicle, feature, camera: str) -> float:
    """Metres by which the feature still sits off the camera axis."""
    if camera in ("up", "down"):
        return math.hypot(
            float(feature[0]) - float(vehicle[0]),
            float(feature[1]) - float(vehicle[1]),
        )
    dist = math.dist(
        (float(vehicle[0]), float(vehicle[1]), float(vehicle[2])),
        (float(feature[0]), float(feature[1]), float(feature[2])),
    )
    return max(0.0, dist - FRONT_CLEAR_M)


def abort_pass(age_s, inside: bool, lost_s: float = PASS_LOST_S) -> bool:
    """A through-pass whose ring went stale before the hull entered the opening."""
    return (not inside) and float(age_s) > float(lost_s)


def prediction_disagrees(visual, predicted, tol: float = DISAGREE_M) -> bool:
    """The boat pose and the last visual fix no longer name the same place."""
    return math.hypot(
        float(visual[0]) - float(predicted[0]),
        float(visual[1]) - float(predicted[1]),
    ) > float(tol)


def choose_eye(front_fresh: bool, down_fresh: bool, vehicle, feature) -> str | None:
    """Which camera servos the bed tag. None means both eyes are dark."""
    if front_fresh and down_fresh and vehicle is not None and feature is not None:
        eye = pick_camera(vehicle, feature)
        return eye if eye in ("front", "down") else "front"
    if down_fresh and not front_fresh:
        return "down"
    if front_fresh:
        return "front"
    if down_fresh:
        return "down"
    return None


def nadir_body(x_n, y_n, z_opt):
    """Tag offset in the hull frame from the downward camera.

    Image +x is hull -Y and image +y is hull -X. The range is along hull -Z.
    A tag in the upper half of the image is ahead of the hull.
    """
    z = float(z_opt)
    return (-float(y_n) * z, -float(x_n) * z, -z)
