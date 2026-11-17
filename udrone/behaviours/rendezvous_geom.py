"""Search under otter_k, close on the deck tag, and dock on the nest axis. ROS-free.

Geometry the weld needs (LinkAttacher on heron_0, nest_pose 0 0 0):
  - Nest at the Heron origin: midline between the pontoons.
  - Approach along the boat's -Z (normal to the deck / body plate, from below).
  - Hull attitude parallel to the boat (same heading, small tilt) before ATTACH.

The upward camera (tag 10) finds the deck. Horizontal setpoints are then
pulled onto the nest axis so the hull stays between the pontoons. The
acceptance cone opens down the nest axis; inside it the hull climbs that
axis and ATTACH is sent in the weld gate.
"""
from __future__ import annotations

import math

LAND_RANGE_M = 0.35     # camera-to-tag range the servo closes to
YAW_GATE = math.radians(25.0)

# LinkAttacher on heron_0 (world/kiwanis_lake.sdf): nest_pose 0 0 0,
# max_attach_distance 0.6 m, max_attach_angle 0.5236 rad.
NEST_BODY = (0.0, 0.0, 0.0)
WELD_DIST_M = 0.6
WELD_ANGLE = 0.5236
ATTACH_DIST_M = 0.45            # send ATTACH inside this (margin under the weld)
ATTACH_ANGLE = math.radians(22.0)
CONE_HALF = math.radians(30.0)  # half-angle about the boat's down axis
CONE_RANGE_M = 2.5              # cone length from the nest
CONE_YAW = YAW_GATE
CONE_TILT = math.radians(35.0)  # body plates must be this parallel to enter the cone
CONE_EXIT = 1.3                 # leave the cone only past this factor (hysteresis)
DOCK_STANDOFF_M = 0.20          # axis point the climb aims at, below the nest
# Residual PPO defaults (rendezvous_rl_experiment.py, kinematic, 2026-11-16):
# chase/climb up, nest-axis blend ~0.72. lead_m stayed 0 (not used live yet).
DOCK_CLIMB_MPS = 0.21
LEARNED_CHASE_MPS = 0.51
LEARNED_CLOSE_RATE = 0.148
LEARNED_BLEND = 0.718
# Deck tag sits a little above the nest on the Heron underside; keep a floor
# under the hull while still lining up on the nest axis.
SEARCH_DEPTH_M = 1.15
SERVO_FLOOR_M = 0.18


def search_offset(t, r0=2.0, r_rate=0.12, omega=0.40, r_max=12.0):
    """East/north offset of an expanding spiral. t is seconds since search began."""
    r = min(float(r_max), float(r0) + float(r_rate) * max(0.0, float(t)))
    a = float(omega) * max(0.0, float(t))
    return (r * math.cos(a), r * math.sin(a), r)


def closing_range(seen_range, elapsed, land=LAND_RANGE_M, rate=0.15):
    """Optical range to command after the tag is found. Shrinks toward `land`."""
    return max(float(land), float(seen_range) - float(rate) * max(0.0, float(elapsed)))


def heading_error(boat_fwd, auv_fwd):
    """Signed yaw of the AUV nose relative to the boat nose, radians."""
    bx, by = float(boat_fwd[0]), float(boat_fwd[1])
    ax, ay = float(auv_fwd[0]), float(auv_fwd[1])
    return math.atan2(bx * ay - by * ax, bx * ax + by * ay)


def yaw_lead(east, north, boat_yaw, lead_m):
    """Shift a horizontal setpoint forward along the boat so the nose follows it."""
    return (
        float(east) + float(lead_m) * math.cos(float(boat_yaw)),
        float(north) + float(lead_m) * math.sin(float(boat_yaw)),
    )


def axes(q):
    """World unit vectors of body x (forward), y (left), z (up) for q = (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q[:4])
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )


def relative_angle(qa, qb) -> float:
    """Angle of the rotation between two attitudes, as the weld gate measures it."""
    dot = sum(float(a) * float(b) for a, b in zip(qa[:4], qb[:4]))
    return 2.0 * math.acos(min(1.0, abs(dot)))


def nest_world(boat_pos, boat_q, nest_body=NEST_BODY):
    """Weld nest in the world: Heron origin, midline between the pontoons."""
    fwd, left, up = axes(boat_q)
    return tuple(
        float(boat_pos[i]) + nest_body[0] * fwd[i] + nest_body[1] * left[i] + nest_body[2] * up[i]
        for i in range(3)
    )


def nest_down(boat_q):
    """Unit vector from the nest into the water: boat -Z, normal to the deck."""
    return tuple(-c for c in axes(boat_q)[2])


def nest_axis_point(boat_pos, boat_q, along_m):
    """World point `along_m` metres below the nest on the boat -Z axis."""
    nest = nest_world(boat_pos, boat_q)
    down = nest_down(boat_q)
    return tuple(nest[i] + float(along_m) * down[i] for i in range(3))


def nest_body_offset(pos, boat_pos, boat_q):
    """Offset of `pos` from the nest in the boat frame (forward, left, up)."""
    nest = nest_world(boat_pos, boat_q)
    fwd, left, up = axes(boat_q)
    v = tuple(float(pos[i]) - nest[i] for i in range(3))
    return (
        sum(v[i] * fwd[i] for i in range(3)),
        sum(v[i] * left[i] for i in range(3)),
        sum(v[i] * up[i] for i in range(3)),
    )


def pull_to_nest_axis(pos, boat_pos, boat_q, blend: float = 1.0):
    """Move `pos` onto the nest's -Z axis (between the pontoons), keep depth along that axis.

    `blend` in [0, 1]: 0 leaves `pos`, 1 snaps fully onto the axis. The along-axis
    distance of `pos` below the nest is preserved so a visual standoff still holds.
    """
    blend = max(0.0, min(1.0, float(blend)))
    nest = nest_world(boat_pos, boat_q)
    down = nest_down(boat_q)
    v = tuple(float(pos[i]) - nest[i] for i in range(3))
    along = sum(v[i] * down[i] for i in range(3))
    # Stay below the nest; a point above would be inside the boat.
    along = max(DOCK_STANDOFF_M, along)
    target = tuple(nest[i] + along * down[i] for i in range(3))
    return tuple(float(pos[i]) + blend * (target[i] - float(pos[i])) for i in range(3))


def under_nest(boat_pos, boat_q, depth_m=SEARCH_DEPTH_M):
    """Search / transit station: on the nest axis, `depth_m` below the nest."""
    return nest_axis_point(boat_pos, boat_q, depth_m)


def plates_parallel(auv_q, boat_q, yaw_lim=CONE_YAW, tilt_lim=CONE_TILT):
    """True when the AUV deck is parallel enough to the boat deck for docking."""
    yaw = heading_error(axes(boat_q)[0], axes(auv_q)[0])
    tilt = relative_angle(auv_q, boat_q)
    return abs(yaw) <= yaw_lim and tilt <= tilt_lim, yaw, tilt


def cone_state(auv_pos, auv_q, boat_pos, boat_q, scale: float = 1.0):
    """Where the hull sits relative to otter_k's nest-axis acceptance cone.

    `inside` needs range, off-axis angle, heading, and plate tilt. `weld` is the
    tighter ATTACH gate (range + full attitude). `lateral` is metres off the
    nest axis in the boat's deck plane (between-pontoons error).
    """
    nest = nest_world(boat_pos, boat_q)
    b_fwd, b_left, b_up = axes(boat_q)
    a_fwd = axes(auv_q)[0]
    down = tuple(-c for c in b_up)
    v = tuple(float(auv_pos[i]) - nest[i] for i in range(3))
    rng = math.sqrt(sum(c * c for c in v))
    along = sum(v[i] * down[i] for i in range(3))
    lateral = math.sqrt(max(0.0, rng * rng - along * along))
    if rng < 0.15:
        off = 0.0
    else:
        cos_off = sum(v[i] * down[i] for i in range(3)) / rng
        off = math.acos(max(-1.0, min(1.0, cos_off)))
    yaw = heading_error(b_fwd, a_fwd)
    tilt = relative_angle(auv_q, boat_q)
    inside = (
        rng <= CONE_RANGE_M * scale
        and off <= CONE_HALF * scale
        and abs(yaw) <= CONE_YAW * scale
        and tilt <= CONE_TILT * scale
    )
    weld = (
        rng <= ATTACH_DIST_M
        and tilt <= ATTACH_ANGLE
        and abs(yaw) <= ATTACH_ANGLE
    )
    return {
        "nest": nest, "down": down, "range": rng, "along": along, "lateral": lateral,
        "off_axis": off, "yaw": yaw, "tilt": tilt, "inside": inside, "weld": weld,
        "fwd": b_fwd, "left": b_left,
    }


def dock_goal(cone, along_now: float, dt: float, climb_mps: float = DOCK_CLIMB_MPS):
    """Next point on the nest axis, climbing toward DOCK_STANDOFF_M under the nest.

    `along_now` is the axis distance the previous goal used. Returns
    (point, along) so the caller can keep climbing from there. `climb_mps`
    is scaled by maneuver power for slower, smoother docks.

    Climb is gated by lateral error: shrinking `along` while still off-axis
    grows the off-axis angle and kicks the hull out of the cone (live smoke
    with PPO climb 0.21 exited at ~39°). Hold depth until centred, then climb.
    """
    climb = max(0.02, float(climb_mps))
    along0 = max(DOCK_STANDOFF_M, float(along_now))
    lateral = abs(float(cone.get("lateral", 0.0)))
    # Half the cone width at current along; full climb only inside ~50% of that.
    half_w = max(0.05, along0 * math.tan(CONE_HALF) * 0.5)
    climb *= max(0.0, 1.0 - lateral / half_w)
    along = max(DOCK_STANDOFF_M, along0 - climb * max(0.0, float(dt)))
    nest, down = cone["nest"], cone["down"]
    return tuple(nest[i] + along * down[i] for i in range(3)), along


def servo_station(tag_enu, optical_m, boat_pos, boat_q, cam_z, blend: float = 1.0):
    """Visual station under the deck tag, snapped onto the nest axis.

    Reuses the track geometry idea (hull under the tag at optical range) but
    pulls the setpoint onto boat -Z so the approach is between the pontoons
    and normal to the deck. `blend` softens the snap while still locking.
    """
    # Import locally so this module stays free of track_geom at import time
    # for the pure geometry tests.
    from track_geom import station_under

    raw = station_under(tag_enu, optical_m, cam_z)
    # Keep a little water under the tag while lining up on the nest.
    z = min(float(tag_enu[2]) - SERVO_FLOOR_M, max(-1.80, float(raw[2])))
    candidate = (float(raw[0]), float(raw[1]), z)
    return pull_to_nest_axis(candidate, boat_pos, boat_q, blend=blend)


def cone_rim(nest, down, half: float = CONE_HALF, length: float = CONE_RANGE_M, n: int = 24):
    """World points on the cone's open end, for drawing."""
    d = tuple(float(c) for c in down)
    ref = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 1.0, 0.0)
    e1 = (d[1] * ref[2] - d[2] * ref[1], d[2] * ref[0] - d[0] * ref[2], d[0] * ref[1] - d[1] * ref[0])
    n1 = math.sqrt(sum(c * c for c in e1)) or 1.0
    e1 = tuple(c / n1 for c in e1)
    e2 = (d[1] * e1[2] - d[2] * e1[1], d[2] * e1[0] - d[0] * e1[2], d[0] * e1[1] - d[1] * e1[0])
    axial = length * math.cos(half)
    radial = length * math.sin(half)
    pts = []
    for k in range(n + 1):
        a = 2.0 * math.pi * k / n
        pts.append(tuple(
            float(nest[i]) + axial * d[i] + radial * (math.cos(a) * e1[i] + math.sin(a) * e2[i])
            for i in range(3)
        ))
    return pts
