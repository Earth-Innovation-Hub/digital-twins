"""Front-camera boat fix: tag sightings -> bounded offset on the boat prior."""
import math

from boat_fix import (
    BOAT_TAGS_BODY,
    FRONT_MOUNT,
    OFFSET_MAX_M,
    STEP_MAX_M,
    BoatFix,
    body_to_world,
    fix_from_sightings,
    front_optical_to_body,
)
from rendezvous_geom import axes


def yaw_q(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def sighting(tag_id, auv_pos, auv_q, boat_pos, boat_q):
    """Optical-frame position the front camera would report for one boat tag."""
    tag_w = body_to_world(boat_pos, boat_q, BOAT_TAGS_BODY[tag_id])
    fwd, left, up = axes(auv_q)
    v = tuple(tag_w[i] - float(auv_pos[i]) for i in range(3))
    bx = sum(v[i] * fwd[i] for i in range(3)) - FRONT_MOUNT[0]
    by = sum(v[i] * left[i] for i in range(3)) - FRONT_MOUNT[1]
    bz = sum(v[i] * up[i] for i in range(3)) - FRONT_MOUNT[2]
    return (tag_id, (-by, -bz, bx))


def close(a, b, tol=1e-6):
    return all(abs(float(a[i]) - float(b[i])) <= tol for i in range(len(a)))


def main():
    assert close(front_optical_to_body((0.1, 0.2, 3.0)), (FRONT_MOUNT[0] + 3.0, -0.1, -0.2))

    boat = (20.0, -4.0, 0.0)
    bq = yaw_q(math.radians(35.0))
    auv = (16.0, -6.5, -1.6)
    aq = yaw_q(math.radians(30.0))
    frame = [sighting(t, auv, aq, boat, bq) for t in (10, 11, 12)]
    fix = fix_from_sightings(frame, auv, aq, bq)
    assert fix is not None
    origin, n, rng = fix
    assert n == 3 and rng > 1.0
    assert close(origin, boat), origin

    # Out-of-range and behind-camera sightings are ignored.
    assert fix_from_sightings([(11, (0.0, 0.0, 0.2))], auv, aq, bq) is None
    assert fix_from_sightings([(11, (0.0, 0.0, -3.0))], auv, aq, bq) is None
    assert fix_from_sightings([(7, (0.0, 0.0, 3.0))], auv, aq, bq) is None

    # A biased prior converges on the camera's boat, horizontally only.
    bias = (0.0, 0.6, 0.0)
    prior = tuple(boat[i] + bias[i] for i in range(3))
    bf = BoatFix()
    last = None
    for k in range(40):
        before = bf.offset
        bf.update(float(k), frame, auv, aq, prior, bq)
        step = math.sqrt(sum((bf.offset[i] - before[i]) ** 2 for i in range(3)))
        assert step <= STEP_MAX_M + 1e-9
        last = bf.apply(prior)
    assert close(last, boat, 0.02), last
    assert bf.offset[2] == 0.0
    assert bf.frames == 40 and bf.last_tags == 3

    # No tags in a frame: the offset is held.
    held = bf.offset
    assert bf.update(41.0, [], auv, aq, prior, bq) is None
    assert bf.offset == held

    # Vision is never trusted beyond OFFSET_MAX_M from the prior.
    far_prior = (boat[0] + 5.0, boat[1], boat[2])
    bf2 = BoatFix()
    for k in range(200):
        bf2.update(float(k), frame, auv, aq, far_prior, bq)
    on = math.sqrt(sum(c * c for c in bf2.offset))
    assert abs(on - OFFSET_MAX_M) < 1e-6, on
    print("boat fix ok")


if __name__ == "__main__":
    main()
