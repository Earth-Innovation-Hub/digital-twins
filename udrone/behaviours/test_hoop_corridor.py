"""Hoop approach corridor + glide path (rendezvous analogue)."""
import math

from hoop_geom import (
    APPROACH_ARRIVE_M,
    APPROACH_STANDOFF_M,
    GATE_ARRIVE_M,
    GATE_STANDOFF_M,
    HOOP_AXIS,
    HOOP_CENTER,
    approach_point,
    approach_station,
    authored_ring_lock,
    leg_clock,
    exit_station,
    gate_station,
    hoop_corridor,
    hoop_glide_path,
    hoop_pass,
    plane_sample,
    punch_centered,
    ready_to_punch,
    surge_speed,
    through_dir,
)


def main():
    c = HOOP_CENTER
    # Vehicle west of the ring: approach from the west, through +east.
    west = (0.5, 0.2, -4.6)
    th = through_dir(west, c, HOOP_AXIS)
    assert th[0] > 0.9, th
    ap = approach_station(c, HOOP_AXIS, west)
    assert ap[0] < c[0], ap
    assert abs(ap[0] - (c[0] - APPROACH_STANDOFF_M)) < 1e-6
    gt = gate_station(c, th)
    assert abs(gt[0] - (c[0] - GATE_STANDOFF_M)) < 1e-6
    ex = exit_station(c, th)
    assert ex[0] > c[0]
    # Frozen open-loop exit = centre + EXIT_STANDOFF_M * through (unit xy).
    from hoop_geom import EXIT_STANDOFF_M
    frozen = (
        c[0] + EXIT_STANDOFF_M * th[0],
        c[1] + EXIT_STANDOFF_M * th[1],
        ex[2],
    )
    assert math.dist(ex, frozen) < 1e-6, (ex, frozen)

    # Last-metre early-punch geometry: ~1.2 m short of gate, on-axis → still
    # commit in hoop_corridor; run_hoop latches through only when lat/depth ready.
    early = (gt[0] - 1.2 * th[0], gt[1] - 1.2 * th[1], gt[2])
    early_c = hoop_corridor(early, c, HOOP_AXIS, prev_phase="commit")
    assert early_c["phase"] == "commit", early_c
    assert early_c["dist_gate"] <= 1.25, early_c
    assert abs(early_c["lat"]) <= 0.28, early_c
    # Deep / off-axis at the same range must stay outside the punch lat gate.
    deep_off = (early[0], early[1] + 0.55, early[2] - 0.80)
    deep_c = hoop_corridor(deep_off, c, HOOP_AXIS, prev_phase="commit")
    assert deep_c["phase"] == "commit", deep_c
    assert abs(deep_c["lat"]) > 0.28, deep_c

    far = hoop_corridor(west, c, HOOP_AXIS)
    assert far["phase"] == "approach", far
    assert far["dist_approach"] > APPROACH_ARRIVE_M

    at_ap = hoop_corridor(ap, c, HOOP_AXIS, prev_phase="approach")
    assert at_ap["phase"] == "commit", at_ap
    assert at_ap["in_funnel"]

    # Mid-funnel sticky commit.
    mid = ((ap[0] + gt[0]) / 2.0, 0.0, ap[2])
    mid_c = hoop_corridor(mid, c, HOOP_AXIS, prev_phase="commit")
    assert mid_c["phase"] == "commit", mid_c

    at_gate = hoop_corridor(gt, c, HOOP_AXIS, prev_phase="commit")
    assert at_gate["phase"] == "through", at_gate
    assert at_gate["dist_gate"] <= GATE_ARRIVE_M + 1e-9

    # Off-axis at the gate: stay on commit (do not punch toward the rim).
    off = (gt[0], 0.55, gt[2])
    off_c = hoop_corridor(off, c, HOOP_AXIS, prev_phase="commit")
    assert off_c["phase"] == "commit", off_c
    assert off_c["lat"] > 0.4

    gp = hoop_glide_path(c, HOOP_AXIS, vehicle=west)
    assert len(gp["paths"]) >= 5
    assert len(gp["markers"]) >= 4
    assert gp["approach"][0] < gp["gate"][0] < gp["opening"][0] < gp["exit"][0]

    # Vehicle east of the ring: approach from the east, through −east.
    east = (10.0, 0.0, -4.6)
    th_e = through_dir(east, c, HOOP_AXIS)
    assert th_e[0] < -0.9, th_e
    ap_e = approach_station(c, HOOP_AXIS, east)
    assert ap_e[0] > c[0]

    # Punch gates are per axis. 0.20 m left used to pass a combined-lat check.
    assert punch_centered(0.10, -0.10)
    assert not punch_centered(0.20, 0.0)
    assert not punch_centered(0.0, 0.20)
    on_plane = plane_sample((6.0, 0.05, -4.60), c, HOOP_AXIS)
    assert on_plane["radial"] < 0.40
    assert hoop_pass(on_plane, 0.40)
    assert not hoop_pass(on_plane, 1.2)
    assert not hoop_pass(None, 0.2)
    graze = dict(on_plane)
    graze["radial"] = 0.55
    assert not hoop_pass(graze, 0.2)

    from hoop_geom import corridor_lead_point
    # Beside the opening, a clamped projection is not "at the gate".
    beside = hoop_corridor((c[0], 4.0, c[2]), c, HOOP_AXIS, through=th)
    assert not beside["on_segment"], beside
    assert not beside["near_tube"], beside
    assert beside["through"][0] > 0.9
    assert beside["approach"][0] < c[0]
    # On the segment: capture, and the lead stays short of the gate.
    mid_pt = ((ap[0] + gt[0]) / 2.0, 0.0, ap[2])
    on_c = hoop_corridor(mid_pt, c, HOOP_AXIS, through=th)
    assert on_c["on_segment"] and on_c["near_tube"], on_c
    lead_pt, _s_cmd, lead = corridor_lead_point(
        on_c["approach"], on_c["gate"], on_c["s_prog"], on_c["length_m"],
    )
    assert 0.0 < lead < 0.15
    assert math.dist(lead_pt, on_c["gate"]) > 0.8
    # Before the station: not on the tube, so the runner goes to the station.
    before = hoop_corridor(west, c, HOOP_AXIS, through=th)
    assert not before["on_segment"] and not before["near_tube"]
    # A graze past the plane must not flip a frozen through-direction.
    past = hoop_corridor((c[0] + 1.5, 0.0, c[2]), c, HOOP_AXIS, through=th)
    assert past["through"][0] > 0.9, past["through"]
    assert past["approach"][0] < c[0]
    assert not past["on_segment"]
    # A centre on the authored ring locks even when it sits low in the image.
    assert authored_ring_lock((6.05, 0.1, -4.70))
    assert not authored_ring_lock((5.03, 1.65, -2.98))  # shallow rim blob
    assert not authored_ring_lock((6.1, 0.0, -5.90))    # bed
    # Moving east through the plane: the sample one second back stays west.
    graze_pos = (c[0] + 0.05, 0.10, c[2] - 0.37)
    backed = approach_point(graze_pos, (0.30, 0.0, 0.0))
    assert backed[0] < c[0]
    assert through_dir(backed, c, HOOP_AXIS)[0] > 0.9
    # Still: no invented velocity, so the geometric side stands.
    assert approach_point(graze_pos, (0.0, 0.0, 0.0))[0] == graze_pos[0]
    # Surge is full until the last metre, then zero past the exit.
    assert surge_speed(2.0) == 0.60
    assert surge_speed(0.5) == 0.30
    assert surge_speed(-0.4) == 0.0
    # Centered 1.1 m short of the gate is still the hold, not the surge.
    assert not ready_to_punch(1.1, 0.0, 0.0)
    assert ready_to_punch(0.40, 0.10, -0.10)
    assert not ready_to_punch(0.40, 0.20, 0.0)
    # Search clock: dive, start once, and pause through a shallow bob.
    z_ring = -4.626
    hold, held, t0, shallow_t = leg_clock(0.0, z_ring, 0.40, False, None, 10.0, None)
    assert hold and not held and t0 is None
    hold, held, t0, shallow_t = leg_clock(z_ring, z_ring, 0.40, False, None, 12.0, None)
    assert not hold and held and t0 == 12.0 and shallow_t is None
    hold, held, t0, shallow_t = leg_clock(z_ring + 0.50, z_ring, 0.40, True, 12.0, 15.0, None)
    assert hold and held and t0 == 12.0 and shallow_t == 15.0
    hold, held, t0, shallow_t = leg_clock(z_ring + 0.50, z_ring, 0.40, True, 12.0, 16.0, 15.0)
    assert hold and shallow_t == 15.0
    hold, held, t0, shallow_t = leg_clock(z_ring, z_ring, 0.40, True, 12.0, 18.0, 15.0)
    assert not hold and held and abs(t0 - 15.0) < 1e-9 and shallow_t is None
    print("hoop corridor ok")


if __name__ == "__main__":
    main()
