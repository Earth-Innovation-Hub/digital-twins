"""Nest-axis rendezvous: spiral under the nest, climb boat -Z, plates parallel."""
import math

from rendezvous_geom import (
    ATTACH_ANGLE,
    ATTACH_DIST_M,
    CONE_EXIT,
    CONE_HALF,
    CONE_RANGE_M,
    CONE_TILT,
    DOCK_STANDOFF_M,
    LAND_RANGE_M,
    NEST_BODY,
    SEARCH_DEPTH_M,
    WELD_ANGLE,
    WELD_DIST_M,
    closing_range,
    cone_rim,
    cone_state,
    dock_goal,
    heading_error,
    nest_axis_point,
    nest_body_offset,
    nest_down,
    nest_world,
    plates_parallel,
    pull_to_nest_axis,
    search_offset,
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
    assert abs(closing_range(2.0, 0.0) - 2.0) < 1e-9
    assert abs(closing_range(2.0, 20.0) - LAND_RANGE_M) < 1e-9
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
    # Off to the side at the same depth: outside the 30 degree cone, lateral > 0.
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
    # Climb default is LEARNED 0.21 m/s (was 0.15); on-axis → full climb.
    assert abs(along - 1.29) < 1e-9 and abs(pt[2] + 1.29) < 1e-9 and pt[:2] == (10.0, 5.0)
    _pt, along = dock_goal(s, 0.25, 10.0)
    assert along == DOCK_STANDOFF_M
    # Off-axis: climb holds so along does not shrink (avoids cone-angle exit).
    s_off = cone_state((10.6, 5.0, -1.5), LEVEL, boat, LEVEL)
    _pt, along_hold = dock_goal(s_off, 1.5, 1.0)
    assert abs(along_hold - 1.5) < 1e-9

    rim = cone_rim(s["nest"], s["down"])
    assert len(rim) == 25
    for p in rim:
        v = tuple(p[i] - s["nest"][i] for i in range(3))
        assert abs(math.sqrt(sum(c * c for c in v)) - CONE_RANGE_M) < 1e-9
        assert abs(math.acos(-v[2] / CONE_RANGE_M) - CONE_HALF) < 1e-9
    print("rendezvous geometry ok")


if __name__ == "__main__":
    main()
