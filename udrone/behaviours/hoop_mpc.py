"""Dynamics-matched through-hoop reference (ROS-free).

State is ``(s, y, z)``:
  - ``s`` along the ring axis (0 at the opening; approach s < 0, exit s > 0)
  - ``y`` horizontal lateral
  - ``z`` vertical offset from the ring centre

The force loop adds at most 0.4 m/s of position correction on top of the
commanded velocity, and the hull has no sideslip: the nose follows the
force. A distant carrot or a lateral velocity the hull cannot produce
pitches the nose and runs through the ring. This planner commands a short
pursuit point on one continuous curve: close lateral and depth error before
the plane, cruise at ≤ 0.32 m/s, then brake at the exit.

Disable with ``UDRONE_HOOP_MPC=0``.
"""
from __future__ import annotations

import math
import os
import random
from typing import Optional

# Horizon ≈ 2.4 s; fits the 50 ms force-loop tick.
HORIZON = 10
DT = 0.20
SAMPLES = 18
ELITES = 5
CEM_ITERS = 2
VEL_TAU_S = 0.65
V_S_MAX = 0.55
V_LAT_MAX = 0.40
TUBE_R = 0.28               # tighter than INNER_PASS 0.40 — clear of the tube
TUBE_MARGIN = 0.92
W_S = 3.5                   # progress toward exit
W_LAT = 28.0                # prefer centering the tube before the plane
W_V = 0.45
W_TERMINAL = 12.0
W_TUBE = 120.0
EXIT_DEFAULT_M = 2.0
# Soft axis capture in the position ref (fraction of residual lateral kept).
# Under yaw_hold the hull only surges; the force needs a centering component.
AXIS_KEEP = 0.45            # far approach: bleed toward the axis
AXIS_KEEP_EXIT = 0.70       # past the plane: gentler
# Final metre: hard-lock the opening centreline — do not drift onto the rim.
FINAL_S_M = 1.20            # |s| within this of the plane → final gate
FINAL_LAT_M = 0.10          # must be this close before punching through
FINAL_KEEP = 0.05           # almost snap lateral ref to the axis
FINAL_VS_CAP = 0.10         # crawl while still off-axis in the final metre


def enabled(env: Optional[dict] = None) -> bool:
    src = env if env is not None else os.environ
    raw = str(src.get("UDRONE_HOOP_MPC", "1")).strip().lower()
    return raw not in ("0", "false", "off", "no")


def _norm2(a):
    n = math.sqrt(a[0] * a[0] + a[1] * a[1])
    if n < 1e-9:
        return (1.0, 0.0), 0.0
    return (a[0] / n, a[1] / n), n


def hoop_axes(axis_xy):
    """Unit forward (horizontal), left (horizontal), up (world +Z)."""
    (fx, fy), _ = _norm2((float(axis_xy[0]), float(axis_xy[1])))
    # left = +Z × fwd → (−fy, fx) in ENU horizontal
    lx, ly = -fy, fx
    return (fx, fy, 0.0), (lx, ly, 0.0), (0.0, 0.0, 1.0)


def hoop_state(pos, center, axis_xy) -> tuple[float, float, float, tuple, tuple, tuple]:
    """Return (s, y, z, fwd, left, up) for ``pos`` relative to the ring."""
    fwd, left, up = hoop_axes(axis_xy)
    v = (
        float(pos[0]) - float(center[0]),
        float(pos[1]) - float(center[1]),
        float(pos[2]) - float(center[2]),
    )
    s = v[0] * fwd[0] + v[1] * fwd[1] + v[2] * fwd[2]
    y = v[0] * left[0] + v[1] * left[1] + v[2] * left[2]
    z = v[0] * up[0] + v[1] * up[1] + v[2] * up[2]
    return s, y, z, fwd, left, up


def tube_ok(y: float, z: float, radius: float = TUBE_R) -> bool:
    return math.hypot(float(y), float(z)) <= float(radius) * TUBE_MARGIN + 1e-9


def _final_gate(s: float, y: float, z: float) -> bool:
    """True in the last metre before the plane while still off the opening axis."""
    return (-FINAL_S_M <= float(s) < 0.15) and math.hypot(float(y), float(z)) > FINAL_LAT_M


# Force-loop limits (udrone_sinusoidal_controller). Position correction is
# capped at 0.4 m/s, so cruise stays under that and the pursuit lead is short.
# The velocity loop lags about 0.65 s (KV≈1.5), so the brake distance includes
# that coast. Pitch matches run_hoop: 40° in cruise, 70° while still off depth.
A_BRAKE = 0.28
VEL_TAU_S = 0.65
LEAD_T = 0.30
LEAD_MAX = 0.12
CRUISE_PITCH = math.radians(40.0)
DIVE_PITCH = math.radians(70.0)
PITCH_SPAN_M = 0.80          # depth error over which pitch opens 40° → 70°
SLEW_MPS2 = 0.30             # how fast the runner may change the commanded velocity
DEPTH_HOLD_M = 0.20
# Descent the hull actually makes on the approach (0.28 commanded; the live
# 2026-11-19 run made about 0.2 m/s) and the depth error allowed at the plane.
VZ_EFF_MPS = 0.20
PLANE_DEPTH_TOL_M = 0.08
# Lateral closure gets the same continuous feasibility treatment as depth.
# This replaces the old hard OFF_TUBE_M branch, which made the live runner
# alternate join/through whenever y crossed ±0.40 m.
VY_EFF_MPS = 0.13
PLANE_LAT_TOL_M = 0.10
# Past the plane with no in-opening sample: the pass is lost. Go around.
MISS_S_M = 0.45
DIVE_ERR_M = 0.55          # only a clearly shallow hull dives in place
DIVE_FAR_S = -2.5           # and only while still well short of the ring
DIVE_MPS = 0.28
ON_LINE_M = 0.22
CENTER_M = 0.15
OFF_TUBE_M = 0.40           # beyond this, slide to the axis instead of advancing
CRUISE_CAP = 0.32
GATE_S = -1.20


def _brake_speed(dist, cruise, accel: float = A_BRAKE, tau: float = VEL_TAU_S) -> float:
    """Fastest speed that can stop in ``dist`` metres, including velocity lag.

    The kinematic stop is ``v²/(2a)``. The force loop keeps the old speed for
    about ``tau`` seconds, so that coast is part of the distance.
    """
    dist = max(0.0, float(dist))
    if dist <= 1e-6:
        return 0.0
    a = max(1e-3, float(accel))
    lag = max(0.0, float(tau))
    v = a * (-lag + math.sqrt(lag * lag + 2.0 * dist / a))
    return min(float(cruise), max(0.0, v))


def pitch_limit_rad(depth_m: float) -> float:
    """Nose allowance: 40° on depth, opening to 70° as the depth error grows."""
    t = min(1.0, abs(float(depth_m)) / PITCH_SPAN_M)
    return CRUISE_PITCH + t * (DIVE_PITCH - CRUISE_PITCH)


def slew_velocity(prev, target, dt, accel: float = SLEW_MPS2):
    """Step ``prev`` toward ``target`` by at most ``accel * dt``.

    Returns ``(velocity, acceleration)``. The acceleration is that step, so
    the force loop is not handed a square-wave brake.
    """
    step = min(0.25, max(1e-3, float(dt)))
    cap = max(0.0, float(accel)) * step
    delta = tuple(float(target[i]) - float(prev[i]) for i in range(3))
    n = math.sqrt(sum(d * d for d in delta))
    if n > cap:
        delta = tuple(d * cap / n for d in delta)
    vel = tuple(float(prev[i]) + delta[i] for i in range(3))
    acc = tuple(delta[i] / step for i in range(3))
    return vel, acc


def filter_center(prev, observed, alpha: float = 0.25, step_m: float = 0.40):
    """Bounded EMA for a static world-frame ring centre."""
    obs = tuple(float(v) for v in observed)
    if prev is None:
        return obs
    old = tuple(float(v) for v in prev)
    delta = tuple(
        max(-float(step_m), min(float(step_m), obs[i] - old[i]))
        for i in range(3)
    )
    return tuple(old[i] + float(alpha) * delta[i] for i in range(3))


def search_dive_reference(z_now, target_z, max_speed: float = 0.24, lead_m: float = 0.12):
    """Short-carrot search dive; returns ``(z_ref, vz)``.

    The full-depth point made the position loop add its 0.4 m/s correction to
    the descent velocity. A short point keeps that correction proportional and
    fades both point and velocity at the target.
    """
    z = float(z_now)
    target = float(target_z)
    # Positive error is above the ring (dive). Negative is below it (climb),
    # which is how a bed-tag approach hands the hull to the hoop.
    if z < target:
        err = target - z
        vz = min(float(max_speed), 0.7 * err)
        z_ref = min(target, z + min(float(lead_m), abs(vz) * 0.6))
        return z_ref, vz
    err = z - target
    vz = -min(float(max_speed), 0.7 * err)
    z_ref = max(target, z - min(float(lead_m), abs(vz) * 0.6))
    return z_ref, vz


def search_glide_reference(
    current,
    end,
    target_z,
    max_vz: float = 0.24,
    min_vh: float = 0.12,
    max_vh: float = 0.32,
    ramp_depth_m: float = 4.0,
):
    """Forward nose-down search glide with a short, continuous carrot."""
    x, y, z = (float(v) for v in current)
    ex, ey = float(end[0]), float(end[1])
    dx, dy = ex - x, ey - y
    dist = math.hypot(dx, dy)
    if dist > 1e-6:
        ux, uy = dx / dist, dy / dist
    else:
        ux = uy = 0.0
    depth_err = abs(z - float(target_z))
    progress = 1.0 - min(1.0, depth_err / max(0.1, float(ramp_depth_m)))
    vh = float(min_vh) + (float(max_vh) - float(min_vh)) * progress
    vh = min(vh, dist / 0.6) if dist > 0.0 else 0.0
    z_ref, vz = search_dive_reference(z, target_z, max_speed=max_vz)
    lead_h = min(0.18, vh * 0.6, dist)
    point = (x + lead_h * ux, y + lead_h * uy, z_ref)
    vel = (vh * ux, vh * uy, vz)
    return point, vel


def _along_lead(vel, fwd) -> float:
    direction, speed = _unit(vel)
    lead = min(LEAD_MAX, speed * LEAD_T)
    return lead * (
        direction[0] * fwd[0] + direction[1] * fwd[1] + direction[2] * fwd[2]
    )


def _lead_point(pos, vel):
    """A short point along ``vel``. The carrot stays near the hull."""
    direction, speed = _unit(vel)
    lead = min(LEAD_MAX, speed * LEAD_T)
    if speed < 1e-3:
        return tuple(float(p) for p in pos)
    return tuple(float(pos[i]) + lead * direction[i] for i in range(3))


def _axis_velocity(
    y, z, dist_along, along_sign, cruise, fwd, left, up,
    dist_plane=None, vy_meas=0.0,
):
    """Along-track speed plus a dive that finishes before the plane.

    Vertical speed fades with the depth error. Forward speed is capped so
    that, at the descent the hull really makes (``VZ_EFF_MPS``), the depth
    left over at the plane is inside ``PLANE_DEPTH_TOL_M``. The cap is
    recomputed from the measured depth every tick, so a slow descent slows
    the approach. ``dist_plane`` is metres of approach left; ``None`` (past
    the plane, or backing away from it) leaves the brake curve alone.
    """
    depth = float(z)
    vz_mag = min(DIVE_MPS, abs(depth) * 0.7)
    vh = _brake_speed(dist_along, cruise)
    excess = abs(depth) - PLANE_DEPTH_TOL_M
    if dist_plane is not None and excess > 1e-3:
        vh = min(vh, VZ_EFF_MPS * max(0.0, float(dist_plane)) / excess)
    # Lateral PD in the hoop frame. Velocity damping removes the residual
    # ±0.15 m crossings left by position-only centering.
    vy = max(-0.16, min(0.16, -0.75 * float(y) - 0.55 * float(vy_meas)))
    lateral_excess = abs(float(y)) - PLANE_LAT_TOL_M
    if dist_plane is not None and lateral_excess > 1e-3:
        vh = min(
            vh,
            VY_EFF_MPS * max(0.0, float(dist_plane)) / lateral_excess,
        )
    vz = -vz_mag if depth > 0.0 else vz_mag
    return tuple(
        float(along_sign) * vh * float(fwd[i])
        + vy * float(left[i])
        + vz * float(up[i])
        for i in range(3)
    )


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        return (0.0, 0.0, 0.0), 0.0
    return tuple(c / n for c in v), n


def _world(center, fwd, left, up, s, y, z):
    return tuple(
        float(center[i]) + float(s) * float(fwd[i]) + float(y) * float(left[i]) + float(z) * float(up[i])
        for i in range(3)
    )


def _pursuit(pos, target, cruise):
    """Point a fraction of a second toward ``target``, with a stoppable speed."""
    delta = tuple(float(target[i]) - float(pos[i]) for i in range(3))
    direction, dist = _unit(delta)
    speed = _brake_speed(dist, cruise)
    lead = min(LEAD_MAX, speed * LEAD_T, dist)
    point = tuple(float(pos[i]) + lead * direction[i] for i in range(3))
    vel = tuple(speed * direction[i] for i in range(3))
    braking = speed < float(cruise) - 0.02 and dist > 1e-3
    acc = tuple((-A_BRAKE * direction[i]) if braking else 0.0 for i in range(3))
    return point, vel, acc, speed


def plan(
    pos,
    center,
    axis_xy,
    speed_mps: float = 0.32,
    exit_m: float = EXIT_DEFAULT_M,
    warm: Optional[list] = None,
    rng: Optional[random.Random] = None,
    v_state: Optional[tuple] = None,
) -> dict:
    """One pursuit reference the force loop can track.

    Close depth and lateral error continuously while making only feasible
    forward progress, then brake to a stop at the exit. Past the plane the
    lateral correction continues forward; the runner starts a go-around when
    no opening sample was earned. ``warm`` and ``rng`` are accepted so older
    callers keep working.
    """
    del warm, rng
    cruise = min(CRUISE_CAP, max(0.12, float(speed_mps)))
    exit_m = max(0.6, float(exit_m))
    s, y, z, fwd, left, up = hoop_state(pos, center, axis_xy)
    vy_meas = 0.0
    if v_state is not None and len(v_state) >= 2:
        vy_meas = (
            float(v_state[0]) * float(left[0])
            + float(v_state[1]) * float(left[1])
        )
    # Depth is not a sideways miss. It scales forward speed in _axis_velocity.
    off_axis = abs(float(y))
    exit_pt = _world(center, fwd, left, up, exit_m, 0.0, 0.0)
    stopped = (0.0, 0.0, 0.0)

    def pack(point, vel, acc, mode, s_ref, final):
        vs = vel[0] * fwd[0] + vel[1] * fwd[1]
        vy = vel[0] * left[0] + vel[1] * left[1]
        vz = vel[2]
        return {
            "point": point,
            "vel": vel,
            "acc": acc,
            "s": float(s_ref),
            "y": 0.0,
            "z": 0.0,
            "warm": None,
            "cost": 0.0,
            "u0": (vs, vy, vz),
            "v_lag": (vs, vy, vz),
            "state0": (s, y, z),
            "fwd": fwd,
            "left": left,
            "up": up,
            "in_tube": tube_ok(y, z),
            "final_gate": bool(final),
            "yaw_hold": None,
            "mode": mode,
            "exit": exit_pt,
        }

    # One law for the whole approach. Forward speed falls continuously when
    # the depth cannot be closed before the plane; there is no dive-in-place
    # switch to chatter on.
    dist_exit = exit_m - float(s)
    if dist_exit <= 0.0 and abs(float(z)) <= DEPTH_HOLD_M:
        return pack(exit_pt, stopped, stopped, "through", exit_m, False)
    # One continuous run-in. Lateral and depth rates are always present and
    # forward speed falls only as much as needed to close both errors before
    # the plane. There is no branch at a tube radius and no reverse command.
    scale = max(0.45, 1.0 - off_axis / 0.50)
    dist_plane = max(0.0, -float(s)) if float(s) < 0.0 else None
    vel = _axis_velocity(
        y, z, max(0.0, dist_exit), 1.0, cruise * scale,
        fwd, left, up, dist_plane, vy_meas,
    )
    point = exit_pt if dist_exit <= 0.0 else _lead_point(pos, vel)
    mode = "approach" if float(s) < 0.0 else "through"
    return pack(
        point, vel, stopped, mode,
        float(s) + _along_lead(vel, fwd), False,
    )
