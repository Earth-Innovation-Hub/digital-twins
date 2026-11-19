"""otter_k position from the xudrone front camera's view of the boat's tags.

The Heron carries three AprilTags (36h11): 10 under the deck, 11 and 12 on the
port and starboard pontoon sterns. apriltag_ros publishes each tag's position in
the front camera's optical frame (x right, y down, z ahead) on /xudrone/tf_front.
With the hull pose and the boat heading, each sighting gives the boat origin.

``BoatFix`` keeps a bounded offset between that vision estimate and the boat
prior (Gazebo pose, optionally biased to stand in for boat GNSS). Guidance flies
on prior + offset. The offset is held when the tags leave the view, because the
last 1.4 m of the approach has none.
"""
from __future__ import annotations

import math

from rendezvous_geom import axes

# Tag centres in the Heron body frame (x forward, y left, z up), from
# model/heron/model.sdf.
BOAT_TAGS_BODY = {
    10: (0.0, 0.0, 0.254),
    11: (-0.606, 0.38, 0.14),
    12: (-0.606, -0.38, 0.14),
}
# xudrone base_link -> front camera link (model/xudrone/model.sdf, no rotation).
FRONT_MOUNT = (0.185, 0.0, 0.0)

MIN_RANGE_M = 0.4
MAX_RANGE_M = 8.0
GAIN = 0.30              # per-frame EMA weight on a fresh fix
STEP_MAX_M = 0.25        # largest offset change one frame may make
OFFSET_MAX_M = 1.5       # never trust vision further than this from the prior
FRAME_GAP_S = 0.05       # sightings closer than this are one camera frame


def front_optical_to_body(t_opt, mount=FRONT_MOUNT):
    """Optical (x right, y down, z ahead) -> hull body (x ahead, y left, z up)."""
    x, y, z = (float(v) for v in t_opt)
    return (mount[0] + z, mount[1] - x, mount[2] - y)


def body_to_world(pos, q, v):
    fwd, left, up = axes(q)
    return tuple(float(pos[i]) + v[0] * fwd[i] + v[1] * left[i] + v[2] * up[i] for i in range(3))


def boat_origin_from_tag(tag_id, tag_world, boat_q):
    """Heron origin implied by a tag seen at ``tag_world`` with the boat heading."""
    b = BOAT_TAGS_BODY[int(tag_id)]
    return body_to_world(tag_world, boat_q, tuple(-c for c in b))


def fix_from_sightings(sightings, auv_pos, auv_q, boat_q):
    """Weighted boat origin from one frame of (tag_id, optical xyz) sightings.

    Returns (origin, n_tags, mean_range) or None. Near tags weigh more
    (pose error grows with the square of range).
    """
    acc = [0.0, 0.0, 0.0]
    wsum = 0.0
    rsum = 0.0
    n = 0
    for tag_id, t_opt in sightings:
        if int(tag_id) not in BOAT_TAGS_BODY:
            continue
        rng = math.sqrt(sum(float(c) * float(c) for c in t_opt))
        if not (MIN_RANGE_M <= rng <= MAX_RANGE_M) or float(t_opt[2]) <= 0.0:
            continue
        tag_w = body_to_world(auv_pos, auv_q, front_optical_to_body(t_opt))
        origin = boat_origin_from_tag(tag_id, tag_w, boat_q)
        w = 1.0 / max(0.25, rng * rng)
        for i in range(3):
            acc[i] += w * origin[i]
        wsum += w
        rsum += rng
        n += 1
    if n == 0:
        return None
    return tuple(a / wsum for a in acc), n, rsum / n


class BoatFix:
    """Bounded, held offset from the boat prior to the front-camera fix."""

    def __init__(self, gain=GAIN, step_max=STEP_MAX_M, offset_max=OFFSET_MAX_M):
        self.gain = float(gain)
        self.step_max = float(step_max)
        self.offset_max = float(offset_max)
        self.offset = (0.0, 0.0, 0.0)
        self.frames = 0
        self.last_t = None
        self.last_tags = 0
        self.last_range = None
        self.last_innovation = None

    def update(self, t, sightings, auv_pos, auv_q, boat_prior_pos, boat_q):
        """Fold one camera frame in. Returns the innovation (m) or None."""
        fix = fix_from_sightings(sightings, auv_pos, auv_q, boat_q)
        if fix is None:
            return None
        origin, n, rng = fix
        # Horizontal only: the boat floats at a known height, and vision height
        # error would move the nest up and down under a climbing hull.
        meas = (origin[0] - float(boat_prior_pos[0]), origin[1] - float(boat_prior_pos[1]), 0.0)
        innov = tuple(meas[i] - self.offset[i] for i in range(3))
        # One far, foreshortened tag should not swing the reference.
        g = self.gain * min(1.0, 0.5 + 0.25 * n) * min(1.0, 4.0 / max(1.0, rng))
        step = tuple(g * c for c in innov)
        sn = math.sqrt(sum(c * c for c in step))
        if sn > self.step_max:
            step = tuple(c * self.step_max / sn for c in step)
        off = tuple(self.offset[i] + step[i] for i in range(3))
        on = math.sqrt(sum(c * c for c in off))
        if on > self.offset_max:
            off = tuple(c * self.offset_max / on for c in off)
        self.offset = off
        self.frames += 1
        self.last_t = float(t)
        self.last_tags = n
        self.last_range = rng
        self.last_innovation = math.sqrt(sum(c * c for c in innov))
        return self.last_innovation

    def apply(self, boat_prior_pos):
        return tuple(float(boat_prior_pos[i]) + self.offset[i] for i in range(3))

    def age(self, t):
        return None if self.last_t is None else float(t) - self.last_t
