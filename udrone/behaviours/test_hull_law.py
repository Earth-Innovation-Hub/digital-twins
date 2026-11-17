"""Hull direction law and baton helpers of the xudrone force loop.

Needs the controller module (rclpy and px4_msgs), so run it in the aquatic
container: python3 test_hull_law.py
"""
import math
import os
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import udrone_sinusoidal_controller as c

hull_dir = c.UdroneHelixController._hull_dir


def state(limit=math.radians(45.0), yaw_hold=None, head=None):
    return SimpleNamespace(pitch_limit=limit, yaw_hold=yaw_hold, _head=head,
                           _elev_sign=None, _dir=None)


def elev(d):
    return math.degrees(math.asin(max(-1.0, min(1.0, d[2]))))


def heading(d):
    return math.atan2(d[1], d[0])


LEVEL_EAST = (1.0, 0.0, 0.0)

# No limit and no heading: the hull points along the force (shapes, helix).
s = state(limit=None)
d = hull_dir(s, (3.0, 4.0, 0.0), LEVEL_EAST, 5.0)
assert abs(d[0] - 0.6) < 1e-9 and abs(d[1] - 0.8) < 1e-9

# A straight-down force pitches only to the limit and keeps the heading.
s = state()
d = hull_dir(s, (0.0, 0.0, -5.0), LEVEL_EAST, 5.0)
assert abs(elev(d) + 45.0) < 1e-6 and abs(heading(d)) < 1e-9, d

# A small sideways force under HEAD_MIN_N does not swing the nose round.
s = state(head=0.0)
d = hull_dir(s, (0.0, 1.0, -0.5), LEVEL_EAST, 1.2)
assert abs(heading(d)) < 1e-9, d
# Above HEAD_MIN_N the nose steers onto the force (zippy yaw).
s = state(head=0.0)
d = hull_dir(s, (0.0, 2.5, -0.5), LEVEL_EAST, 2.6)
assert abs(abs(heading(d)) - math.pi / 2) < 1e-6, d

# Controller exposes last thrust / horizontal error for stuck-steer.
assert hasattr(c.UdroneHelixController, "_control")

# A modest force behind the nose backs up: the heading stays, the nose pitches
# so that reverse thrust also pushes the right way vertically.
s = state(head=0.0)
d = hull_dir(s, (-4.0, 0.0, -1.0), LEVEL_EAST, 4.1)
assert abs(heading(d)) < 1e-9 and elev(d) > 0.0, d
thrust = sum(f * u for f, u in zip((-4.0, 0.0, -1.0), d))
assert thrust < 0.0
# A large one turns the hull round.
s = state(head=0.0)
d = hull_dir(s, (-12.0, 0.0, 0.0), LEVEL_EAST, 12.0)
assert abs(abs(heading(d)) - math.pi) < 1e-9, d

# Near vertical, the pitch side is kept while the forward part changes sign,
# and it changes once the forward part is a real share of the force.
s = state(head=0.0)
d1 = hull_dir(s, (0.5, 0.0, -5.0), LEVEL_EAST, 5.0)
d2 = hull_dir(s, (-0.5, 0.0, -5.0), d1, 5.0)
assert elev(d1) < 0.0 and elev(d2) < 0.0, (d1, d2)
d3 = hull_dir(s, (-3.5, 0.0, -5.0), d2, 6.1)
assert elev(d3) > 0.0 and abs(heading(d3)) < 1e-9, d3

# A held heading (station keeping) is kept for any force.
s = state(limit=math.radians(20.0), yaw_hold=1.0)
d = hull_dir(s, (-3.0, 0.5, -2.0), LEVEL_EAST, 3.6)
assert abs(heading(d) - 1.0) < 1e-9 and abs(elev(d)) <= 20.0 + 1e-9

# Tiny forces leave the last direction alone.
s = state()
s._dir = (0.0, 1.0, 0.0)
assert hull_dir(s, (0.1, 0.0, -0.1), LEVEL_EAST, 0.14) == (0.0, 1.0, 0.0)

# Baton: a live other pid owns it, this pid or a dead pid does not.
with tempfile.TemporaryDirectory() as tmp:
    c.BATON_DIR = tmp
    path, req = c.baton_paths("xudrone_t")
    assert path.startswith(tmp) and req.endswith(".req")
    assert c.baton_owner("xudrone_t") is None
    with open(path, "w") as f:
        f.write('{"pid": %d}' % os.getpid())
    assert c.baton_owner("xudrone_t", os.getpid()) is None
    assert c.baton_owner("xudrone_t", 0) == os.getpid()
    with open(path, "w") as f:
        f.write('{"pid": 999999}')
    assert c.baton_owner("xudrone_t") is None

# Trim: a little positive, well under the old 2 N.
assert 0.1 < c.NET_BUOYANCY_N < 0.6, c.NET_BUOYANCY_N
print("hull law ok")
