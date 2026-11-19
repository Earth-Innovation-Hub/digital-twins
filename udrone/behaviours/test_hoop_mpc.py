"""Dynamics-matched hoop reference: dive, join the line, brake at the exit."""
import math

from hoop_mpc import (
    CENTER_M,
    CRUISE_CAP,
    GATE_S,
    LEAD_MAX,
    enabled,
    filter_center,
    hoop_state,
    pitch_limit_rad,
    plan,
    search_dive_reference,
    search_glide_reference,
    slew_velocity,
    tube_ok,
)


def _speed(vel):
    return math.sqrt(sum(v * v for v in vel))


def _assert_pitch(u0, depth):
    """The command pitch stays under the nose limit, with margin."""
    limit = pitch_limit_rad(depth) * 0.80
    pitched = abs(math.atan2(u0[2], max(abs(u0[0]), 1e-3)))
    assert pitched <= limit + 0.05, (pitched, limit, u0)


def main():
    assert enabled({"UDRONE_HOOP_MPC": "1"})
    assert not enabled({"UDRONE_HOOP_MPC": "0"})

    center = (6.0, 0.0, -4.626)
    axis = (1.0, 0.0)

    # Far and shallow: descend while cruising. The pitch of this command stays
    # under the nose limit, so the stop is not a switch.
    shallow = (0.4, 0.1, -0.2)
    dive = plan(shallow, center, axis, speed_mps=0.32, exit_m=2.0)
    assert dive["mode"] == "approach", dive["mode"]
    assert dive["u0"][0] > 0.15, dive["u0"]
    assert dive["u0"][2] < -0.15, dive["u0"]
    _s_far, _y_far, z_far, *_ = hoop_state(shallow, center, axis)
    _assert_pitch(dive["u0"], z_far)

    # Off the line: close laterally while making only feasible forward progress.
    # There is no join/through switch at a tube radius.
    wide = (2.5, 1.2, -4.626)
    s_w, y_w, z_w, *_ = hoop_state(wide, center, axis)
    assert s_w < GATE_S and abs(y_w) > CENTER_M
    join = plan(wide, center, axis)
    assert join["mode"] == "approach"
    assert 0.05 < join["u0"][0] < CRUISE_CAP, join["u0"]
    assert join["u0"][1] < 0.0  # pull +y back to the axis
    assert _speed(join["vel"]) <= CRUISE_CAP + 1e-9

    # A little off the line past the gate: keep going. Reversing here
    # chattered for the whole 2026-11-17 run.
    mild = (5.2, 0.18, -4.50)
    s_m, y_m, z_m, *_ = hoop_state(mild, center, axis)
    assert s_m > GATE_S and math.hypot(y_m, z_m) > CENTER_M
    keep = plan(mild, center, axis)
    assert keep["mode"] == "approach", keep["mode"]
    assert keep["u0"][0] > 0.10, keep["u0"]
    # Depth bobbing near the ring must not restart the dive.
    bob = (5.0, 0.05, center[2] + 0.40)
    assert plan(bob, center, axis)["mode"] != "dive"
    # Clearly off the tube past the gate: slow and center, never reverse.
    graze = (5.5, 0.90, -4.626)
    s_g, *_ = hoop_state(graze, center, axis)
    assert s_g > GATE_S
    back = plan(graze, center, axis)
    assert back["mode"] == "approach", back["mode"]
    assert 0.0 <= back["u0"][0] < 0.12, back["u0"]
    assert back["u0"][1] < -0.10, back["u0"]
    # Commands are continuous across the old ±0.40 m branch.
    edge_a = plan((4.0, 0.399, -4.626), center, axis)
    edge_b = plan((4.0, 0.401, -4.626), center, axis)
    assert edge_a["mode"] == edge_b["mode"] == "approach"
    assert max(abs(edge_a["u0"][i] - edge_b["u0"][i]) for i in range(3)) < 0.01
    # Past the plane and outside the tube: slide sideways. Do not reverse
    # back through the opening — that undoes the crossing.
    crossed = (6.20, 0.45, -4.626)
    s_c, y_c, z_c, *_ = hoop_state(crossed, center, axis)
    assert s_c > 0.0 and math.hypot(y_c, z_c) > 0.40
    side = plan(crossed, center, axis)
    assert side["mode"] == "through", side["mode"]
    assert side["u0"][0] >= -0.02, side["u0"]
    s_side, *_ = hoop_state(side["point"], center, axis)
    assert s_side >= s_c - 1e-6

    # On the centreline: short lead, cruise under the brake cap, stop at the exit.
    on = (2.5, 0.0, -4.626)
    s0, y0, z0, *_ = hoop_state(on, center, axis)
    cruise = plan(on, center, axis, speed_mps=0.32, exit_m=2.0)
    assert cruise["mode"] == "approach"
    assert 0.2 < cruise["u0"][0] <= CRUISE_CAP + 1e-9
    assert abs(cruise["u0"][1]) < 1e-6
    s_pt, y_pt, z_pt, *_ = hoop_state(cruise["point"], center, axis)
    assert 0.0 < s_pt - s0 <= LEAD_MAX + 1e-9
    assert abs(y_pt) < 1e-6 and abs(z_pt) < 1e-6
    assert abs(y0) < 1e-9 and abs(z0) < 1e-9

    done = plan((8.0, 0.0, -4.626), center, axis, exit_m=2.0)
    assert _speed(done["vel"]) < 1e-9
    near_exit = plan((7.95, 0.0, -4.626), center, axis, exit_m=2.0)
    assert near_exit["u0"][0] < 0.25
    # 0.30 m from the exit is already inside a 0.32 m/s stop once lag is counted.
    braking = plan((7.70, 0.0, -4.626), center, axis, exit_m=2.0)
    assert 0.05 < braking["u0"][0] < 0.30, braking["u0"]
    # A few tenths shallow, still in the tube: keep descending while cruising.
    trim = plan((5.0, 0.0, -4.326), center, axis, speed_mps=0.32, exit_m=2.0)
    assert trim["mode"] == "approach", trim["mode"]
    assert trim["u0"][2] < -0.10, trim["u0"]
    assert trim["u0"][0] > 0.20, trim["u0"]
    # Metres shallow and 1 m from the plane: forward speed nearly stops, dive stays.
    shallow_near = plan((5.0, 0.0, -1.5), center, axis, speed_mps=0.32, exit_m=2.0)
    assert shallow_near["mode"] == "approach", shallow_near["mode"]
    assert shallow_near["u0"][2] < -0.15, shallow_near["u0"]
    assert shallow_near["u0"][0] < 0.10, shallow_near["u0"]
    # Very shallow at the plane: hold and dive.
    hold = plan((5.8, 0.0, 1.574), center, axis, exit_m=2.0)
    assert hold["mode"] == "approach", hold["mode"]
    assert abs(hold["u0"][0]) < 0.03, hold["u0"]
    assert hold["u0"][2] < -0.15, hold["u0"]
    # The old far-dive gate must not change the command as the hull crosses it.
    far_side = plan((3.45, 0.0, -1.5), center, axis, exit_m=2.0)
    near_side = plan((3.55, 0.0, -1.5), center, axis, exit_m=2.0)
    assert abs(far_side["u0"][0] - near_side["u0"][0]) < 0.05, (far_side["u0"], near_side["u0"])
    assert abs(far_side["u0"][2] - near_side["u0"][2]) < 0.05, (far_side["u0"], near_side["u0"])
    # 2026-11-19 tour: locked at the surface 6 m short. A hull that only makes
    # 0.2 m/s down must still reach the plane inside the opening.
    p = (0.0, 0.0, -0.13)
    plane_r = None
    for _ in range(2400):
        st = plan(p, center, axis, speed_mps=0.32, exit_m=2.0)
        v = st["vel"]
        vz = max(-0.20, min(0.20, v[2]))
        p_next = (p[0] + 0.05 * v[0], p[1] + 0.05 * v[1], p[2] + 0.05 * vz)
        s_a = hoop_state(p, center, axis)[0]
        s_b = hoop_state(p_next, center, axis)[0]
        p = p_next
        if s_a < 0.0 <= s_b:
            _s, y_x, z_x, *_ = hoop_state(p, center, axis)
            plane_r = math.hypot(y_x, z_x)
            break
    assert plane_r is not None and plane_r < 0.35, plane_r
    # Half a metre shallow on the centreline is not a sideways miss, so no reverse.
    above = plan((5.0, 0.0, -4.10), center, axis, exit_m=2.0)
    assert above["u0"][0] > 0.10, above["u0"]
    # Neighbouring depths move the command continuously.
    near_a = plan((5.0, 0.0, -1.50), center, axis, exit_m=2.0)
    near_b = plan((5.0, 0.0, -1.55), center, axis, exit_m=2.0)
    assert abs(near_a["u0"][0] - near_b["u0"][0]) < 0.05, (near_a["u0"], near_b["u0"])
    # The slew cannot follow a 0.32 m/s step in one tick.
    stepped, step_acc = slew_velocity((0.0, 0.0, 0.0), (0.32, 0.0, 0.0), 0.05)
    assert abs(stepped[0] - 0.015) < 1e-9, stepped
    assert abs(step_acc[0] - 0.30) < 1e-9, step_acc
    v = (0.0, 0.0, -0.28)
    for _ in range(30):
        v, _ = slew_velocity(v, (0.32, 0.0, 0.0), 0.05)
    assert v[0] > 0.25 and v[2] > -0.05, v
    assert abs(pitch_limit_rad(0.0) - math.radians(40.0)) < 1e-9
    assert abs(pitch_limit_rad(0.80) - math.radians(70.0)) < 1e-6
    mid_pitch = pitch_limit_rad(0.40)
    assert math.radians(50.0) < mid_pitch < math.radians(60.0), mid_pitch
    # Perception filtering cannot accept an arbitrarily large one-frame jump.
    fc = filter_center((6.0, 0.0, -4.626), (8.0, 2.0, -2.0))
    assert max(abs(fc[i] - (6.1, 0.1, -4.526)[i]) for i in range(3)) < 1e-9, fc
    # Search dive uses a short point, not the full 4.5 m depth error.
    z_ref, vz = search_dive_reference(-0.13, center[2])
    assert -0.26 < z_ref < -0.20, (z_ref, vz)
    assert abs(vz + 0.24) < 1e-9, vz
    z_ref, vz = search_dive_reference(center[2] + 0.05, center[2])
    assert center[2] <= z_ref < center[2] + 0.05
    assert -0.04 < vz < 0.0
    # Search depth closes on a forward glide, with no full-depth point.
    gp, gv = search_glide_reference((0.0, 0.0, -0.13), (3.0, 0.0, center[2]), center[2])
    assert 0.05 < gp[0] < 0.20 and gp[1] == 0.0
    assert -0.26 < gp[2] < -0.20
    assert 0.10 < gv[0] < 0.20 and abs(gv[2] + 0.24) < 1e-9
    # Measured motion across the axis is damped, not allowed to build a cycle.
    undamped = plan((4.0, 0.15, -4.626), center, axis)
    damped = plan((4.0, 0.15, -4.626), center, axis, v_state=(0.0, 0.20, 0.0))
    assert damped["u0"][1] < undamped["u0"][1] < 0.0

    # Hull plus the force loop's position correction (capped at 0.4 m/s).
    p = (2.5, 1.2, -4.626)
    modes = []
    crossed_y = None
    for _ in range(800):
        step = plan(p, center, axis, speed_mps=0.32, exit_m=2.0)
        if not modes or modes[-1] != step["mode"]:
            modes.append(step["mode"])
        err = tuple(step["point"][i] - p[i] for i in range(3))
        en = math.sqrt(sum(e * e for e in err)) or 1.0
        corr = min(0.4, 0.7 * en)
        vel = tuple(step["vel"][i] + corr * err[i] / en for i in range(3))
        p = tuple(p[i] + 0.05 * vel[i] for i in range(3))
        s, y, z, *_ = hoop_state(p, center, axis)
        if s > 0.0 and crossed_y is None:
            crossed_y = math.hypot(y, z)
        if s > 1.7 and _speed(step["vel"]) < 0.08:
            break
    assert len(modes) <= 2, modes
    assert crossed_y is not None and crossed_y < 0.15, crossed_y
    s_f, y_f, z_f, *_ = hoop_state(p, center, axis)
    assert s_f > 1.5, s_f
    assert tube_ok(y_f, z_f) or math.hypot(y_f, z_f) < 0.25, (y_f, z_f)
    print("hoop mpc ok")


if __name__ == "__main__":
    main()
