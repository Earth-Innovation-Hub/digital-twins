"""Hoop approach corridor + glide path (rendezvous analogue)."""
import math

from hoop_geom import (
    APPROACH_ARRIVE_M,
    APPROACH_STANDOFF_M,
    GATE_ARRIVE_M,
    GATE_STANDOFF_M,
    HOOP_AXIS,
    HOOP_CENTER,
    approach_station,
    exit_station,
    gate_station,
    hoop_corridor,
    hoop_glide_path,
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
    print("hoop corridor ok")


if __name__ == "__main__":
    main()
