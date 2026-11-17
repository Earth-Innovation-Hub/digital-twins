"""Approach otter_k from aft, close on the deck tag, dock on the nest axis. ROS-free.

Geometry the weld needs (LinkAttacher on heron_0, nest_pose 0 0 0):
  - Nest at the Heron origin: midline between the pontoons.
  - Approach along the boat's -Z (normal to the deck / body plate, from below).
  - Hull attitude parallel to the boat (same heading, small tilt) before ATTACH.

Continuous 3D glide (ILS-style): deep aft tip → slope up the centreline into
the weld standoff. Up-cam AprilTag 10 locks early on the slope (nose-down under
surge aims the top camera forward-up). Flare near the tip hands residual to
nest-frame dock MPC; ATTACH in the weld gate.
"""
from __future__ import annotations

import math

LAND_RANGE_M = 0.35     # camera-to-tag range the servo closes to
YAW_GATE = math.radians(40.0)

# LinkAttacher on heron_0 (world/kiwanis_lake.sdf): nest_pose 0 0 0,
# max_attach_distance 0.6 m, max_attach_angle 0.5236 rad.
NEST_BODY = (0.0, 0.0, 0.0)
WELD_DIST_M = 0.6
WELD_ANGLE = 0.5236
# Behaviour ATTACH gate (must stay inside the physical weld above).
ATTACH_DIST_M = 0.55
ATTACH_ANGLE = math.radians(28.0)
CONE_HALF = math.radians(42.0)  # half-angle about the boat's down axis
CONE_RANGE_M = 3.5              # cone length from the nest
CONE_YAW = YAW_GATE
CONE_TILT = math.radians(48.0)  # body plates must be this parallel to enter the cone
CONE_EXIT = 1.4                 # leave the cone only past this factor (hysteresis)
DOCK_STANDOFF_M = 0.20          # axis point the climb aims at, below the nest
# Residual PPO defaults (rendezvous_rl_experiment.py, kinematic, 2026-11-16):
# chase/climb up, nest-axis blend ~0.72. lead_m stayed 0 (not used live yet).
# Last-metre RL elite (2026-11-17): 0.13 — 0.21 raced small lateral offsets.
DOCK_CLIMB_MPS = 0.13
LEARNED_CHASE_MPS = 0.51
LEARNED_CLOSE_RATE = 0.148
LEARNED_BLEND = 0.718
# Deck tag sits a little above the nest on the Heron underside; keep a floor
# under the hull while still lining up on the nest axis.
SEARCH_DEPTH_M = 1.15
SERVO_FLOOR_M = 0.18
# First tag lock often reports ~4 m when the hull is deep; starting the close
# schedule from that range commands a violent climb and loses the tag. Cap the
# servo standoff near search / cone depth so centering happens first.
SERVO_LOCK_CAP_M = 1.6
# Continuous glide: deep aft tip → weld standoff (inverted ceiling landing).
AFT_STANDOFF_M = 6.0
GLIDE_AFT_DEPTH_M = 1.80          # tip depth so the slope has real climb
AFT_DEPTH_M = GLIDE_AFT_DEPTH_M   # aft_station / search tip depth
AFT_ARRIVE_M = 0.75
COMMIT_ARRIVE_M = 0.55            # legacy approach_corridor under ball
GLIDE_FLARE_S = 0.72              # progress s where flare / crawl begins
GLIDE_LATCH_S = 0.92              # near threshold → latch / dock MPC
GLIDE_HALF_TIP_M = 1.50           # lateral half-width at the deep tip
GLIDE_HALF_THR_M = 0.28           # lateral half-width at weld threshold
GLIDE_SPEED_MPS = 0.42            # along-track cruise on the slope
GLIDE_FLARE_MPS = 0.12            # crawl in flare
# Local loiter under the nest only if the up-cam has not locked yet.
UNDER_SPIRAL_R0_M = 0.6
UNDER_SPIRAL_R_MAX_M = 2.0
# search_otter coverage: expanding lateral sinusoid from the aft station
# toward (and past) the nest until the up-cam locks deck tag 10.
SEARCH_ALONG_MPS = 0.28
SEARCH_AMP0_M = 0.8
SEARCH_AMP_RATE = 0.10       # m/s amplitude growth
SEARCH_AMP_MAX_M = 5.0
SEARCH_OMEGA = 0.55          # rad/s of the sine
SEARCH_ALONG_PAD_M = 1.5     # metres past the nest before the along-track bounce


def search_offset(t, r0=2.0, r_rate=0.12, omega=0.40, r_max=12.0):
    """Planar spiral offsets (forward, left, radius). t is seconds since search began.

    Apply with boat ``fwd``/``left`` via ``under_loiter_point`` so a yawing otter
    keeps the loiter between the pontoons (not a world E/N scribble).
    """
    r = min(float(r_max), float(r0) + float(r_rate) * max(0.0, float(t)))
    a = float(omega) * max(0.0, float(t))
    return (r * math.cos(a), r * math.sin(a), r)


def _horiz(v):
    x, y = float(v[0]), float(v[1])
    n = math.hypot(x, y) or 1.0
    return (x / n, y / n)


def under_loiter_point(nest, depth_z, fwd, left, t,
                       r0=UNDER_SPIRAL_R0_M, r_rate=0.06, omega=0.40,
                       r_max=UNDER_SPIRAL_R_MAX_M):
    """World point on a boat-frame under-nest spiral at depth ``depth_z``."""
    df, dl, _r = search_offset(t, r0=r0, r_rate=r_rate, omega=omega, r_max=r_max)
    fx, fy = _horiz(fwd)
    lx, ly = _horiz(left)
    return (
        float(nest[0]) + df * fx + dl * lx,
        float(nest[1]) + df * fy + dl * ly,
        float(depth_z),
    )


def aft_sine_coverage(aft, fwd, left, t,
                      along_span_m=AFT_STANDOFF_M + SEARCH_ALONG_PAD_M,
                      along_mps=SEARCH_ALONG_MPS,
                      amp0_m=SEARCH_AMP0_M,
                      amp_rate=SEARCH_AMP_RATE,
                      amp_max_m=SEARCH_AMP_MAX_M,
                      omega=SEARCH_OMEGA,
                      depth_z=None):
    """Expanding lateral sinusoid starting at the aft station (boat frame).

    Along-track progresses from aft toward the nest (+fwd) and bounces over
    ``along_span_m`` (triangle wave). Lateral amplitude grows with time so the
    sweep covers a widening corridor — coverage built into search_otter.
    Returns ``(point, along_m, amp_m, lateral_m)``.
    """
    t = max(0.0, float(t))
    span = max(1.0, float(along_span_m))
    # Triangle bounce in [0, span]: 0 at aft, span under/past nest, back again.
    period = 2.0 * span / max(0.05, float(along_mps))
    phase = (t % period) * max(0.05, float(along_mps))
    if phase <= span:
        along = phase
    else:
        along = 2.0 * span - phase
    amp = min(float(amp_max_m), float(amp0_m) + float(amp_rate) * t)
    lat = amp * math.sin(float(omega) * t)
    fx, fy = _horiz(fwd)
    lx, ly = _horiz(left)
    z = float(aft[2] if depth_z is None else depth_z)
    pt = (
        float(aft[0]) + along * fx + lat * lx,
        float(aft[1]) + along * fy + lat * ly,
        z,
    )
    return pt, along, amp, lat


def aft_sine_polyline(aft, fwd, left, duration_s=40.0, dt=0.5, **kwargs):
    """Sampled polyline of ``aft_sine_coverage`` for HUD / tests."""
    pts = []
    t = 0.0
    dur = max(dt, float(duration_s))
    step = max(0.1, float(dt))
    while t <= dur + 1e-9:
        pt, _a, _amp, _lat = aft_sine_coverage(aft, fwd, left, t, **kwargs)
        pts.append(pt)
        t += step
    return pts


def closing_range(seen_range, elapsed, land=LAND_RANGE_M, rate=0.15):
    """Optical range to command after the tag is found. Shrinks toward `land`."""
    return max(float(land), float(seen_range) - float(rate) * max(0.0, float(elapsed)))


def closing_range_rate(seen_range, elapsed, land=LAND_RANGE_M, rate=0.15):
    """d(optical)/dt of ``closing_range`` (0 once landed on the schedule)."""
    if closing_range(seen_range, elapsed, land=land, rate=rate) <= float(land) + 1e-9:
        return 0.0
    return -float(rate)


def servo_close_velocity(boat_q, seen_range, elapsed, land=LAND_RANGE_M, rate=0.15):
    """World velocity of the servo station from the optical close schedule.

    Shrinking optical range climbs the hull along boat −Z (toward the deck).
    """
    d_opt = closing_range_rate(seen_range, elapsed, land=land, rate=rate)
    down = nest_down(boat_q)
    return tuple(float(d_opt) * float(down[i]) for i in range(3))


def gate_optical_range(z_opt, along_m, slack_m: float = 0.6):
    """Reject foreshortened optical range using nest-axis along when available.

    Returns a capped range for the servo lock schedule: never much longer than
    the geometric along (plus slack), and never below the weld standoff floor.
    """
    z = float(z_opt)
    if along_m is None:
        return z
    along = max(DOCK_STANDOFF_M, float(along_m))
    return max(DOCK_STANDOFF_M, min(z, along + float(slack_m)))


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


def aft_station(boat_pos, boat_q, aft_m=AFT_STANDOFF_M, depth_m=AFT_DEPTH_M):
    """Deep aft tip of the glide: centreline `aft_m` behind the nest."""
    under = under_nest(boat_pos, boat_q, depth_m)
    fwd = axes(boat_q)[0]
    return (
        float(under[0]) - float(aft_m) * fwd[0],
        float(under[1]) - float(aft_m) * fwd[1],
        float(under[2]),
    )


def _lerp3(a, b, t: float):
    t = float(t)
    return tuple(float(a[i]) + t * (float(b[i]) - float(a[i])) for i in range(3))


def _shift(p, left, side: float, half_m: float):
    w = float(side) * float(half_m)
    return tuple(float(p[i]) + w * float(left[i]) for i in range(3))


def glide_tip(boat_pos, boat_q, aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M):
    """Deep aft tip of the continuous 3D glide centreline."""
    return aft_station(boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m)


def glide_threshold(boat_pos, boat_q, standoff_m=DOCK_STANDOFF_M):
    """Weld aim: nest axis at dock standoff (touchdown)."""
    return nest_axis_point(boat_pos, boat_q, standoff_m)


def _unit3(v):
    n = math.sqrt(sum(float(c) * float(c) for c in v[:3])) or 1.0
    return tuple(float(c) / n for c in v[:3])


def glide_segment(boat_pos, boat_q,
                  aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M,
                  standoff_m=DOCK_STANDOFF_M):
    """Tip, threshold, unit tangent tip→threshold, and segment length."""
    tip = glide_tip(boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m)
    thr = glide_threshold(boat_pos, boat_q, standoff_m=standoff_m)
    delta = tuple(float(thr[i]) - float(tip[i]) for i in range(3))
    length = math.sqrt(sum(c * c for c in delta)) or 1.0
    tangent = tuple(c / length for c in delta)
    return tip, thr, tangent, length


def glide_station(s, boat_pos, boat_q,
                  aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M,
                  standoff_m=DOCK_STANDOFF_M):
    """World point on the glide centreline at progress ``s`` in [0, 1]."""
    tip, thr, tangent, length = glide_segment(
        boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=standoff_m,
    )
    u = max(0.0, min(1.0, float(s)))
    point = _lerp3(tip, thr, u)
    return {
        "point": point,
        "s": u,
        "tip": tip,
        "threshold": thr,
        "tangent": tangent,
        "length_m": length,
    }


def glide_half_m(s, half_tip=GLIDE_HALF_TIP_M, half_thr=GLIDE_HALF_THR_M):
    """Tapering tube half-width at progress ``s``."""
    u = max(0.0, min(1.0, float(s)))
    return float(half_tip) + u * (float(half_thr) - float(half_tip))


def glide_speed_mps(s, cruise=GLIDE_SPEED_MPS, flare=GLIDE_FLARE_MPS,
                    flare_s=GLIDE_FLARE_S):
    """Along-track speed schedule: cruise early, crawl in flare."""
    u = max(0.0, min(1.0, float(s)))
    fs = max(0.05, float(flare_s))
    if u < fs:
        # Ease from cruise toward flare speed as s→flare_s.
        t = u / fs
        return float(cruise) + t * (float(flare) - float(cruise)) * 0.55
    return float(flare)


def project_onto_glide(auv_pos, boat_pos, boat_q,
                       aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M,
                       standoff_m=DOCK_STANDOFF_M):
    """Project the hull onto the tip→threshold segment (s, lateral, along).

    ``s`` is clamped to [0, 1]. ``on_segment`` is False when the raw projection
    falls outside the tip→threshold interval — a far off-axis hull can clamp to
    s≈1 without being near the weld (warm-start must not treat that as latch).
    """
    tip, thr, tangent, length = glide_segment(
        boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=standoff_m,
    )
    v = tuple(float(auv_pos[i]) - float(tip[i]) for i in range(3))
    along = sum(v[i] * tangent[i] for i in range(3))
    s_raw = along / length
    s = max(0.0, min(1.0, s_raw))
    closest = _lerp3(tip, thr, s)
    lat_vec = tuple(float(auv_pos[i]) - closest[i] for i in range(3))
    lat = math.sqrt(sum(c * c for c in lat_vec))
    return {
        "s": s,
        "s_raw": s_raw,
        "on_segment": 0.0 <= s_raw <= 1.0,
        "lat": lat,
        "along_m": along,
        "length_m": length,
        "closest": closest,
        "tip": tip,
        "threshold": thr,
        "tangent": tangent,
    }


def glide_corridor(auv_pos, boat_pos, boat_q,
                   aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M,
                   standoff_m=DOCK_STANDOFF_M,
                   aft_arrive_m=AFT_ARRIVE_M,
                   flare_s=GLIDE_FLARE_S, latch_s=GLIDE_LATCH_S,
                   half_tip=GLIDE_HALF_TIP_M, half_thr=GLIDE_HALF_THR_M,
                   prev_phase: str | None = None):
    """Progress along the continuous 3D glide into the weld.

    ``phase``:
      - ``approach``: swim to the deep aft tip
      - ``glide``: climb the slope toward the nest
      - ``flare``: crawl near threshold (plates level, dock MPC ready)
      - ``latch``: at/near weld standoff

    Sticky: once on the slope, do not bounce back to approach every tick.
    """
    nest = nest_world(boat_pos, boat_q)
    tip, thr, tangent, length = glide_segment(
        boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=standoff_m,
    )
    fwd, left, _up = axes(boat_q)
    proj = project_onto_glide(
        auv_pos, boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=standoff_m,
    )
    s = proj["s"]
    lat = proj["lat"]
    half = glide_half_m(s, half_tip=half_tip, half_thr=half_thr)
    in_funnel = lat <= half * 1.20 + 1e-9
    on_segment = bool(proj["on_segment"])
    # Near the tube: refuse flare/latch from a clamped s when the hull is far.
    near_tube = in_funnel and on_segment and lat <= max(half * 1.5, 1.25)
    dist_tip = math.sqrt(sum((float(auv_pos[i]) - tip[i]) ** 2 for i in range(3)))
    dist_thr = math.sqrt(sum((float(auv_pos[i]) - thr[i]) ** 2 for i in range(3)))
    prev = (prev_phase or "").lower()
    # Legacy + new stage names that mean "already committed on the slope".
    slope_sticky = prev in (
        "glide", "flare", "latch", "commit", "under", "servo", "relock", "dock",
    )
    tip_ok = dist_tip <= float(aft_arrive_m) or (s > 0.02 and in_funnel and on_segment)
    if (
        near_tube
        and dist_thr <= max(0.45, float(standoff_m) + 0.35)
        and s >= float(latch_s) - 0.05
    ):
        phase = "latch"
    elif (
        prev == "latch"
        and near_tube
        and s >= float(flare_s)
        and dist_thr <= float(aft_m) * 0.5
    ):
        phase = "latch"
    elif s >= float(flare_s) and near_tube:
        phase = "flare"
    elif tip_ok or (slope_sticky and near_tube and s > 0.0):
        phase = "glide"
    elif prev == "glide" and near_tube and dist_tip <= float(aft_m) + 1.5:
        phase = "glide"
    else:
        phase = "approach"
    return {
        "phase": phase,
        "s": s,
        "s_raw": proj["s_raw"],
        "on_segment": on_segment,
        "near_tube": near_tube,
        "lat": lat,
        "half_m": half,
        "in_funnel": in_funnel,
        "nest": nest,
        "tip": tip,
        "aft": tip,              # alias for HUD / search
        "threshold": thr,
        "under": thr,            # touchdown ≈ under nest at standoff
        "tangent": tangent,
        "length_m": length,
        "fwd": fwd,
        "left": left,
        "dist_tip": dist_tip,
        "dist_aft": dist_tip,    # alias
        "dist_threshold": dist_thr,
        "dist_under": dist_thr,  # alias for warm-start
        "speed_mps": glide_speed_mps(s),
        "station": glide_station(
            s, boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=standoff_m,
        ),
    }


def approach_corridor(auv_pos, boat_pos, boat_q,
                      aft_m=AFT_STANDOFF_M, depth_m=AFT_DEPTH_M,
                      aft_arrive_m=AFT_ARRIVE_M, commit_arrive_m=COMMIT_ARRIVE_M,
                      prev_phase: str | None = None):
    """Legacy aft→under corridor (search_otter). Prefer ``glide_corridor`` for dock.

    Maps glide sticky names onto under/commit so search warm-starts still work.
    """
    nest = nest_world(boat_pos, boat_q)
    aft = aft_station(boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m)
    # Horizontal under station at the same depth as the tip (search coverage).
    under = under_nest(boat_pos, boat_q, depth_m)
    fwd, left, _up = axes(boat_q)
    bf, bl, bu = nest_body_offset(auv_pos, boat_pos, boat_q)
    dist_aft = math.hypot(
        float(auv_pos[0]) - aft[0], float(auv_pos[1]) - aft[1],
    )
    dist_under = math.hypot(
        float(auv_pos[0]) - under[0], float(auv_pos[1]) - under[1],
    )
    prev = (prev_phase or "").lower()
    under_sticky = prev in (
        "under", "servo", "relock", "dock", "glide", "flare", "latch",
    )
    if dist_under <= commit_arrive_m:
        phase = "under"
    elif under_sticky and dist_under <= float(aft_m):
        phase = "under"
    elif prev == "commit" and dist_under > commit_arrive_m:
        phase = "commit"
    elif bf > -1.0 and abs(bl) < 1.5 and float(auv_pos[2]) < -0.4:
        phase = "commit" if dist_under > commit_arrive_m else "under"
    elif dist_aft <= aft_arrive_m:
        phase = "commit"
    else:
        phase = "aft"
    return {
        "phase": phase,
        "nest": nest,
        "aft": aft,
        "under": under,
        "fwd": fwd,
        "left": left,
        "body": (bf, bl, bu),
        "dist_aft": dist_aft,
        "dist_under": dist_under,
    }


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
    Gate width uses the live cone ``along`` (not the open-loop ``along_now``)
    so a lagging hull cannot inflate the tube and climb out of the cone.
    """
    climb = max(0.02, float(climb_mps))
    along_live = max(DOCK_STANDOFF_M, float(cone.get("along", along_now)))
    # Do not lead the open-loop climb far past where the hull actually is.
    along0 = max(DOCK_STANDOFF_M, min(float(along_now), along_live + 0.15))
    lateral = abs(float(cone.get("lateral", 0.0)))
    # Half the cone width at the live along; full climb only inside ~50% of that.
    half_w = max(0.05, along_live * math.tan(CONE_HALF) * 0.5)
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


# HUD range-gate fractions along the continuous tip→threshold glide.
GLIDE_AFT_HALF_M = GLIDE_HALF_TIP_M  # alias for older tests
GLIDE_GATE_FRACS = (0.0, 0.25, 0.50, 0.75, 1.0)


def glide_path(boat_pos, boat_q,
               aft_m=AFT_STANDOFF_M, depth_m=GLIDE_AFT_DEPTH_M,
               aft_half_m=GLIDE_HALF_TIP_M,
               thr_half_m=GLIDE_HALF_THR_M,
               cone_half=CONE_HALF, cone_len=CONE_RANGE_M,
               dock_standoff=DOCK_STANDOFF_M,
               inside: bool = False, weld: bool = False):
    """HUD polylines for the continuous tip→weld glide (ILS funnel).

    One tapering centreline from the deep aft tip into the nest standoff —
    no separate horizontal approach + vertical climb.
    """
    nest = nest_world(boat_pos, boat_q)
    tip, thr, _tangent, _length = glide_segment(
        boat_pos, boat_q, aft_m=aft_m, depth_m=depth_m, standoff_m=dock_standoff,
    )
    fwd, left, _up = axes(boat_q)
    aft_half = max(float(thr_half_m), float(aft_half_m))
    thr_half = float(thr_half_m)

    tip_l = _shift(tip, left, 1.0, aft_half)
    tip_r = _shift(tip, left, -1.0, aft_half)
    thr_l = _shift(thr, left, 1.0, thr_half)
    thr_r = _shift(thr, left, -1.0, thr_half)

    color = (255, 255, 255) if weld else ((0, 220, 0) if inside else (0, 180, 80))
    paths = [
        {
            "pts": [tip_l, thr_l, thr_r, tip_r],
            "color": color,
            "width": 1,
            "closed": True,
            "fill": True,
            "fill_alpha": 0.22,
            "label": "",
        },
        {"pts": [tip_l, thr_l], "color": color, "width": 2, "label": ""},
        {"pts": [tip_r, thr_r], "color": color, "width": 2, "label": ""},
        {"pts": [tip, thr], "color": color, "width": 2, "label": "glide"},
    ]
    for t in GLIDE_GATE_FRACS:
        c = _lerp3(tip, thr, t)
        w = aft_half + t * (thr_half - aft_half)
        label = ""
        if t <= 1e-9:
            label = f"{aft_m:.0f}m"
        elif t >= 1.0 - 1e-9:
            label = "threshold"
        paths.append({
            "pts": [_shift(c, left, 1.0, w), _shift(c, left, -1.0, w)],
            "color": color,
            "width": 1,
            "label": label,
        })
    # Soft cone mouth at the nest for up-cam context.
    paths.append({
        "pts": cone_rim(nest, nest_down(boat_q), half=cone_half, length=cone_len),
        "color": color,
        "width": 1,
        "label": "cone",
    })
    markers = [
        {"pos": tip, "label": "aft", "color": color, "shape": "diamond"},
        {"pos": thr, "label": "threshold", "color": color, "shape": "circle"},
        {"pos": nest, "label": "nest", "color": color, "shape": "circle"},
        {"pos": thr, "label": "touchdown", "color": color, "shape": "diamond"},
    ]
    return {
        "paths": paths,
        "markers": markers,
        "cone": {
            "center": nest,
            "radius": float(cone_len) * math.sin(float(cone_half)),
            "inside": bool(inside),
        },
        "aft": tip,
        "under": thr,
        "tip": tip,
        "threshold": thr,
        "nest": nest,
        "fwd": fwd,
        "left": left,
        "aft_half_m": aft_half,
        "under_half_m": thr_half,
    }
