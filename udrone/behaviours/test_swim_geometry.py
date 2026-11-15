"""Path-frame geometry for a straight xudrone swim. No ROS."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from patterns import swim_path_end


def check(name, got, exp, tol=1e-9):
    if math.dist(got, exp) > tol:
        raise SystemExit(f"{name}: {got} != {exp}")
    print(f"ok {name}")


def main():
    # Facing east (yaw 0): north is to the left, east is forward.
    check("north while facing east", swim_path_end(0.0, "current", north_m=10.0), (0.0, 10.0))
    check("east while facing east", swim_path_end(0.0, "current", east_m=10.0), (10.0, 0.0))
    # Facing north (yaw +pi/2): a north offset is straight ahead.
    check("north while facing north",
          swim_path_end(math.pi / 2, "current", north_m=10.0), (10.0, 0.0))
    # 40 m north, 0 east, facing east → 40 m to the left.
    check("40 m north", swim_path_end(0.0, "current", north_m=40.0), (0.0, 40.0))
    # Heading leg ignores yaw: forward 8, starboard 2 → path (8, -2).
    check("heading",
          swim_path_end(1.2, "heading", east_m=2.0, along_heading_m=8.0), (8.0, -2.0))
    # World target 4 m east of an arm pose at east=1, facing east → forward 4.
    check("world",
          swim_path_end(0.0, "world", north_m=2.0, east_m=5.0, x0=1.0, y0=2.0), (4.0, 0.0))
    print("swim geometry ok")


if __name__ == "__main__":
    main()
