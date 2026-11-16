"""NED/FRD to ENU/FLU, and a filleted coverage polyline."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pose_frame as pf
from patterns import eased_duration, eased_polyline


def body_x(q):
    w, x, y, z = q
    return (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))


def test_ned():
    assert pf.ned_to_enu(3.0, 4.0, 5.0) == (4.0, 3.0, -5.0)
    # Level, nose north in FRD/NED is the identity quaternion.
    # In ENU that nose points north, which is +Y.
    fwd = body_x(pf.px4_q_to_enu((1.0, 0.0, 0.0, 0.0)))
    assert abs(fwd[0]) < 1e-6 and abs(fwd[1] - 1.0) < 1e-6 and abs(fwd[2]) < 1e-6, fwd
    print("ok ned/enu")


def test_fillet():
    # A square corner. The arc's acceleration points at the centre, and
    # numerical derivatives match the analytic ones.
    wps = [(0.0, 0.0, -1.0), (10.0, 0.0, -1.0), (10.0, 10.0, -1.0)]
    dur = eased_duration(wps, 0.25, fillet_m=0.6)
    assert dur > 0.0
    # Sample on the arc: far enough that the straight is done (trim 0.6 of 10 m).
    t_arc = 9.5 / 0.25
    p, v, a = eased_polyline(wps, 0.25, t_arc, fillet_m=0.6)
    dt = 1e-3
    p2, v2, _ = eased_polyline(wps, 0.25, t_arc + dt, fillet_m=0.6)
    num_v = tuple((b - a_) / dt for a_, b in zip(p, p2))
    num_a = tuple((b - a_) / dt for a_, b in zip(v, v2))
    assert math.dist(num_v, v) < 2e-2, (num_v, v)
    assert math.dist(num_a, a) < 5e-2, (num_a, a)
    # The corner itself is not a waypoint of the smoothed path.
    for t in (i * 0.05 for i in range(int(dur / 0.05))):
        pos, _, _ = eased_polyline(wps, 0.25, t, fillet_m=0.6)
        assert math.dist(pos, (10.0, 0.0, -1.0)) > 0.15, math.dist(pos, (10.0, 0.0, -1.0))
    # After the end, the path holds still.
    held, vel, acc = eased_polyline(wps, 0.25, dur + 5.0, fillet_m=0.6)
    assert math.dist(vel, (0.0, 0.0, 0.0)) < 1e-9
    assert math.dist(acc, (0.0, 0.0, 0.0)) < 1e-9
    assert math.dist(held[:2], (10.0, 10.0)) < 0.05
    print("ok fillet")


def test_fuser():
    f = pf.PoseFuser()
    q = (1.0, 0.0, 0.0, 0.0)
    gz = (10.0, 20.0, -2.0)
    # Stale odometry: fly Gazebo.
    pos, _ = f.fuse(gz, q, (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0), q, now=1.0)
    assert f.source == "gazebo" and pos == gz and f.vel is None
    # Fresh odometry: the first sample sets the offset, so the estimate lands on Gazebo.
    vpos = (5.0, 4.0, 1.0, 0.1, 0.2, 0.3, 10.0)
    pos, _ = f.fuse(gz, q, vpos, q, now=10.1)
    assert f.source == "odom" and math.dist(pos, gz) < 1e-9, pos
    assert f.vel == (0.2, 0.1, -0.3)
    # A 2 m jump in the estimate is refused and Gazebo is flown.
    jump = (7.0, 4.0, 1.0, 0.0, 0.0, 0.0, 10.2)
    pos, _ = f.fuse(gz, q, jump, q, now=10.2)
    assert f.rejected and f.source == "gazebo" and pos == gz and f.rejects == 1
    assert abs(f.gz_miss - 2.0) < 1e-6
    print("ok pose fuser")


if __name__ == "__main__":
    test_ned()
    test_fillet()
    test_fuser()
    print("pose frame ok")
