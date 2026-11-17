"""Nest-axis rendezvous: spiral under the nest, climb boat -Z, plates parallel."""
import math

from rendezvous_geom import (
    AFT_ARRIVE_M,
    AFT_DEPTH_M,
    AFT_STANDOFF_M,
    ATTACH_ANGLE,
    ATTACH_DIST_M,
    CONE_EXIT,
    CONE_HALF,
    CONE_RANGE_M,
    CONE_TILT,
    DOCK_STANDOFF_M,
    GLIDE_AFT_DEPTH_M,
    GLIDE_FLARE_S,
    GLIDE_LATCH_S,
    LAND_RANGE_M,
    NEST_BODY,
    SEARCH_DEPTH_M,
    SERVO_LOCK_CAP_M,
    WELD_ANGLE,
    WELD_DIST_M,
    aft_sine_coverage,
    aft_sine_polyline,
    aft_station,
    approach_corridor,
    axes,
    closing_range,
    closing_range_rate,
    cone_rim,
    cone_state,
    dock_goal,
    gate_optical_range,
    glide_corridor,
    glide_path,
    glide_station,
    glide_threshold,
    glide_tip,
    heading_error,
    nest_axis_point,
    nest_body_offset,
    nest_down,
    nest_world,
    plates_parallel,
    project_onto_glide,
    pull_to_nest_axis,
    search_offset,
    servo_close_velocity,
    under_loiter_point,
    under_nest,
    yaw_lead,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def yaw_q(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def pitch_q(pitch):
    return (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)


def main():
    e, n, r = search_offset(0.0)
    assert abs(math.hypot(e, n) - 2.0) < 1e-6
    _e2, _n2, r2 = search_offset(1000.0)
    assert abs(r2 - 12.0) < 1e-6
    # Boat-frame loiter: t=0 is +forward of the nest (not world-east after yaw).
    boat = (10.0, 5.0, 0.0)
    north = yaw_q(math.pi / 2.0)
    nest_n = nest_world(boat, north)
    loiter = under_loiter_point(nest_n, -1.15, *axes(north)[:2], 0.0, r0=0.6, r_max=2.0)
    assert abs(loiter[0] - nest_n[0]) < 1e-6
    assert abs(loiter[1] - (nest_n[1] + 0.6)) < 1e-6
    assert abs(loiter[2] + 1.15) < 1e-9
    # Expanding sine from aft: t=0 at aft; amplitude grows; polyline has length.
    aft0 = aft_station(boat, LEVEL)
    fwd0, left0, _u0 = axes(LEVEL)
    p0, along0, amp0, lat0 = aft_sine_coverage(aft0, fwd0, left0, 0.0)
    assert along0 == 0.0 and abs(lat0) < 1e-9
    assert abs(p0[0] - aft0[0]) < 1e-6 and abs(p0[1] - aft0[1]) < 1e-6
    _p1, _a1, amp1, _l1 = aft_sine_coverage(aft0, fwd0, left0, 10.0)
    assert amp1 > amp0
    poly = aft_sine_polyline(aft0, fwd0, left0, duration_s=8.0, dt=0.5)
    assert len(poly) >= 10
    assert abs(closing_range(2.0, 0.0) - 2.0) < 1e-9
    assert abs(closing_range(2.0, 20.0) - LAND_RANGE_M) < 1e-9
    assert abs(closing_range_rate(2.0, 0.0, rate=0.15) + 0.15) < 1e-9
    assert abs(closing_range_rate(2.0, 20.0, rate=0.15)) < 1e-9
    v_close = servo_close_velocity(LEVEL, 2.0, 0.0, rate=0.15)
    assert abs(v_close[0]) < 1e-9 and abs(v_close[1]) < 1e-9
    assert abs(v_close[2] - 0.15) < 1e-9   # climb = −d(opt)·down, down=(0,0,−1)
    # Nest-relative contract: world station FD minus boat_v must not keep boat_v.
    boat_v = (0.4, -0.1, 0.0)
    world_fd = (0.4 + 0.0, -0.1 + 0.05, 0.15)  # boat ride + nest-relative
    nest_rel = tuple(world_fd[i] - boat_v[i] for i in range(3))
    assert abs(nest_rel[0]) < 1e-9 and abs(nest_rel[1] - 0.05) < 1e-9
    assert abs(nest_rel[2] - 0.15) < 1e-9
    composed = tuple(boat_v[i] + nest_rel[i] for i in range(3))
    assert abs(composed[0] - world_fd[0]) < 1e-9
    assert abs(gate_optical_range(4.0, 1.2, slack_m=0.6) - 1.8) < 1e-9
    assert abs(gate_optical_range(1.0, 1.2) - 1.0) < 1e-9
    assert abs(heading_error((1.0, 0.0, 0.0), (1.0, 0.0, 0.0))) < 1e-9
    assert abs(heading_error((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) - math.pi / 2) < 1e-6
    lead = yaw_lead(0.0, 0.0, 0.0, 0.4)
    assert abs(lead[0] - 0.4) < 1e-9 and abs(lead[1]) < 1e-9

    # The attach margins sit inside the LinkAttacher gate.
    assert ATTACH_DIST_M < WELD_DIST_M and ATTACH_ANGLE < WELD_ANGLE
    assert NEST_BODY == (0.0, 0.0, 0.0)

    boat = (10.0, 5.0, 0.0)
    nest = nest_world(boat, LEVEL)
    assert nest == boat
    assert nest_down(LEVEL) == (0.0, 0.0, -1.0)
    under = under_nest(boat, LEVEL)
    assert abs(under[2] + SEARCH_DEPTH_M) < 1e-9 and under[:2] == (10.0, 5.0)
    assert nest_axis_point(boat, LEVEL, 0.8) == (10.0, 5.0, -0.8)

    # Off-axis point snaps onto boat -Z, keeping depth along that axis.
    pulled = pull_to_nest_axis((11.0, 6.0, -1.2), boat, LEVEL)
    assert abs(pulled[0] - 10.0) < 1e-9 and abs(pulled[1] - 5.0) < 1e-9
    assert abs(pulled[2] + 1.2) < 1e-9
    half = pull_to_nest_axis((11.0, 5.0, -1.0), boat, LEVEL, blend=0.5)
    assert abs(half[0] - 10.5) < 1e-9
    # Above the nest is clamped to the standoff so we never aim into the hull.
    above = pull_to_nest_axis((10.0, 5.0, 0.5), boat, LEVEL)
    assert abs(above[2] + DOCK_STANDOFF_M) < 1e-9

    # Body offset: +forward / +left / +up from the nest.
    fwd, left, up = nest_body_offset((10.5, 5.2, 0.1), boat, LEVEL)
    assert abs(fwd - 0.5) < 1e-9 and abs(left - 0.2) < 1e-9 and abs(up - 0.1) < 1e-9

    ok, yaw, tilt = plates_parallel(LEVEL, LEVEL)
    assert ok and abs(yaw) < 1e-9 and abs(tilt) < 1e-9
    ok, _y, _t = plates_parallel(pitch_q(CONE_TILT + 0.05), LEVEL)
    assert not ok

    # Straight under the nest, lined up: inside, not yet weldable.
    s = cone_state((10.0, 5.0, -1.5), LEVEL, boat, LEVEL)
    assert s["inside"] and not s["weld"] and abs(s["off_axis"]) < 1e-9, s
    assert abs(s["range"] - 1.5) < 1e-9 and abs(s["along"] - 1.5) < 1e-9
    assert abs(s["lateral"]) < 1e-9 and s["down"] == (0.0, 0.0, -1.0)
    # Off to the side at the same depth: outside the cone, lateral > 0.
    s = cone_state((11.5, 5.0, -1.5), LEVEL, boat, LEVEL)
    assert not s["inside"] and s["off_axis"] > CONE_HALF and s["lateral"] > 1.0
    # The exit scale keeps a hull that drifted a little.
    edge = math.tan(CONE_HALF * 1.1) * 1.5
    assert not cone_state((10.0 + edge, 5.0, -1.5), LEVEL, boat, LEVEL)["inside"]
    assert cone_state((10.0 + edge, 5.0, -1.5), LEVEL, boat, LEVEL, CONE_EXIT)["inside"]
    # Too deep, or turned across the boat: outside.
    assert not cone_state((10.0, 5.0, -(CONE_RANGE_M + 0.2)), LEVEL, boat, LEVEL)["inside"]
    assert not cone_state((10.0, 5.0, -1.5), yaw_q(math.radians(60)), boat, LEVEL)["inside"]
    # Plate tilt beyond CONE_TILT keeps the hull out even when under the nest.
    assert not cone_state((10.0, 5.0, -1.5), pitch_q(CONE_TILT + 0.05), boat, LEVEL)["inside"]
    # The cone follows the boat's heading.
    s = cone_state((10.0, 5.0, -1.0), yaw_q(1.0), boat, yaw_q(1.0))
    assert s["inside"] and abs(s["yaw"]) < 1e-9
    # Pitched boat: nest axis follows boat -Z (not world down).
    p = math.radians(20.0)
    bq = pitch_q(p)
    axis = nest_axis_point(boat, bq, 1.0)
    s = cone_state(axis, bq, boat, bq)
    assert s["inside"] and abs(s["lateral"]) < 1e-6 and abs(s["along"] - 1.0) < 1e-6
    # Close and level: weldable. Close but pitched 30 degrees: not.
    assert cone_state((10.0, 5.0, -0.3), LEVEL, boat, LEVEL)["weld"]
    assert not cone_state((10.0, 5.0, -0.3), pitch_q(math.radians(30)), boat, LEVEL)["weld"]

    # The climb moves up the axis at a bounded rate and stops short of the nest.
    s = cone_state((10.0, 5.0, -1.5), LEVEL, boat, LEVEL)
    pt, along = dock_goal(s, 1.5, 1.0)
    # Climb default is last-metre RL 0.13 m/s (was 0.21); on-axis → full climb.
    assert abs(along - 1.37) < 1e-9 and abs(pt[2] + 1.37) < 1e-9 and pt[:2] == (10.0, 5.0)
    _pt, along = dock_goal(s, 0.25, 10.0)
    assert along == DOCK_STANDOFF_M
    # Off-axis past ~half cone width: climb holds (avoids cone-angle exit).
    s_off = cone_state((10.9, 5.0, -1.5), LEVEL, boat, LEVEL)
    assert s_off["lateral"] > 0.8
    _pt, along_hold = dock_goal(s_off, 1.5, 1.0)
    assert abs(along_hold - 1.5) < 1e-9
    # Stale open-loop along must not inflate the gate: live along is short.
    s_lag = cone_state((10.55, 5.0, -0.8), LEVEL, boat, LEVEL)
    assert s_lag["along"] < 1.0 and s_lag["lateral"] > 0.4
    _pt, along_lag = dock_goal(s_lag, 1.5, 1.0)
    assert along_lag <= s_lag["along"] + 0.15 + 1e-9
    assert abs(along_lag - max(DOCK_STANDOFF_M, min(1.5, s_lag["along"] + 0.15))) < 1e-9

    # Servo lock cap: far sightings start closing from here, not 4 m.
    assert LAND_RANGE_M < SERVO_LOCK_CAP_M <= SEARCH_DEPTH_M + 0.5
    assert abs(max(LAND_RANGE_M, min(SERVO_LOCK_CAP_M, 4.16)) - SERVO_LOCK_CAP_M) < 1e-9
    assert abs(max(LAND_RANGE_M, min(SERVO_LOCK_CAP_M, 1.0)) - 1.0) < 1e-9

    # Continuous 3D glide: deep tip → weld threshold.
    tip = glide_tip(boat, LEVEL)
    thr = glide_threshold(boat, LEVEL)
    assert abs(tip[0] - (10.0 - AFT_STANDOFF_M)) < 1e-9
    assert abs(tip[2] + GLIDE_AFT_DEPTH_M) < 1e-9
    assert abs(thr[2] + DOCK_STANDOFF_M) < 1e-9
    mid_st = glide_station(0.5, boat, LEVEL)
    assert abs(mid_st["point"][2] - 0.5 * (tip[2] + thr[2])) < 1e-6
    # Far aft → approach; at tip → glide; near threshold → flare/latch.
    far_g = glide_corridor((10.0 - AFT_STANDOFF_M - 3.0, 5.0, -GLIDE_AFT_DEPTH_M), boat, LEVEL)
    assert far_g["phase"] == "approach", far_g
    at_tip = glide_corridor(tip, boat, LEVEL)
    assert at_tip["phase"] == "glide" and at_tip["s"] < 0.05, at_tip
    near = glide_station(GLIDE_FLARE_S + 0.05, boat, LEVEL)["point"]
    flare = glide_corridor(near, boat, LEVEL, prev_phase="glide")
    assert flare["phase"] in ("flare", "latch"), flare
    latch_pt = glide_station(GLIDE_LATCH_S + 0.05, boat, LEVEL)["point"]
    latch = glide_corridor(latch_pt, boat, LEVEL, prev_phase="flare")
    assert latch["phase"] == "latch", latch
    # Sticky: mid-slope with prev=glide stays on the slope.
    mid_pos = glide_station(0.4, boat, LEVEL)["point"]
    mid_g = glide_corridor(mid_pos, boat, LEVEL, prev_phase="glide")
    assert mid_g["phase"] == "glide" and mid_g["in_funnel"], mid_g
    proj = project_onto_glide(mid_pos, boat, LEVEL)
    assert abs(proj["s"] - 0.4) < 0.05 and proj["lat"] < 0.05
    # Monotonic s along the centreline.
    s_prev = -0.01
    for u in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        p = glide_station(u, boat, LEVEL)["point"]
        pr = project_onto_glide(p, boat, LEVEL)
        assert pr["s"] + 1e-6 >= s_prev
        s_prev = pr["s"]
    # Far off-axis: clamped s must NOT look like latch / near_tube.
    far_side = (10.0 + 20.0, 5.0, -1.0)
    bad = glide_corridor(far_side, boat, LEVEL)
    assert bad["phase"] == "approach", bad
    assert not bad["near_tube"], bad
    assert bad["dist_threshold"] > 5.0
    # Projection may clamp s high; on_segment / near_tube reject it.
    bad_p = project_onto_glide(far_side, boat, LEVEL)
    assert bad_p["lat"] > 1.0
    assert not (bad_p["on_segment"] and bad_p["lat"] < 0.5)

    # Legacy approach_corridor still works for search_otter.
    aft = aft_station(boat, LEVEL)
    assert abs(aft[2] + AFT_DEPTH_M) < 1e-9
    far = approach_corridor((10.0 - AFT_STANDOFF_M - 3.0, 5.0, -AFT_DEPTH_M), boat, LEVEL)
    assert far["phase"] == "aft"
    at_aft = approach_corridor(aft, boat, LEVEL)
    assert at_aft["phase"] == "commit" and at_aft["dist_aft"] <= AFT_ARRIVE_M
    mid = approach_corridor((10.0 - 2.0, 5.0, -AFT_DEPTH_M), boat, LEVEL, prev_phase="commit")
    assert mid["phase"] == "commit"
    under = under_nest(boat, LEVEL, AFT_DEPTH_M)
    at_under = approach_corridor(under, boat, LEVEL)
    assert at_under["phase"] == "under"
    drifted = approach_corridor((10.0 - 2.5, 5.0, -AFT_DEPTH_M), boat, LEVEL, prev_phase="under")
    assert drifted["phase"] == "under" and drifted["dist_under"] > 1.5
    for prev in ("servo", "relock", "dock", "glide", "flare", "latch"):
        stuck = approach_corridor(
            (10.0 - 2.5, 5.0, -AFT_DEPTH_M), boat, LEVEL, prev_phase=prev,
        )
        assert stuck["phase"] == "under", (prev, stuck["phase"])

    # Glide-path HUD: one continuous tip→threshold funnel.
    gp = glide_path(boat, LEVEL, inside=False)
    assert gp["aft_half_m"] >= gp["under_half_m"] > 0.0
    assert abs(gp["aft"][0] - (10.0 - AFT_STANDOFF_M)) < 1e-9
    assert abs(gp["aft"][2] + GLIDE_AFT_DEPTH_M) < 1e-9
    assert any(p.get("label") == "glide" for p in gp["paths"])
    assert not any(p.get("label") == "approach" for p in gp["paths"])
    assert any(p.get("fill") and p.get("closed") for p in gp["paths"])
    assert any(m.get("label") == "touchdown" for m in gp["markers"])
    assert gp["cone"]["center"] == gp["nest"]

    rim = cone_rim(s["nest"], s["down"])
    assert len(rim) == 25
    for p in rim:
        v = tuple(p[i] - s["nest"][i] for i in range(3))
        assert abs(math.sqrt(sum(c * c for c in v)) - CONE_RANGE_M) < 1e-9
        assert abs(math.acos(-v[2] / CONE_RANGE_M) - CONE_HALF) < 1e-9
    print("rendezvous geometry ok")


if __name__ == "__main__":
    main()
