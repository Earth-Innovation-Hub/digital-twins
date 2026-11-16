"""Pose-graph keyframes, odometry chain, and a tag landmark (needs gtsam)."""
import math
import os
import tempfile

import numpy as np

import factor_graph as fg

LEVEL = (1.0, 0.0, 0.0, 0.0)


def main():
    body = fg.optical_to_body((0.0, 0.0, 2.0), (0.185, 0.0, 0.0), LEVEL)
    assert abs(body[0] - 2.185) < 1e-9 and abs(body[1]) < 1e-9 and abs(body[2]) < 1e-9
    if not fg.available():
        print("factor graph skipped (gtsam missing)")
        return
    g = fg.PoseGraph()
    p0 = fg.pose3((0.0, 0.0, -1.0), LEVEL)
    o0 = fg.pose3((10.0, 0.0, -1.0), LEVEL)   # odometry frame offset from map
    assert g.need_keyframe(o0, 0.0)
    g.add_keyframe(o0, p0, 0.0, depth_z=-1.05, gnss=(0.1, -0.1, -0.2))
    g.update()
    assert g.k == 0
    # Depth factor pulls z toward the measurement; prior still keeps it near −1.
    assert abs(g.pose(0).translation()[2] + 1.0) < 0.1
    # Drive 2 m east in odometry; that is a new keyframe.
    o1 = fg.pose3((12.0, 0.0, -1.0), LEVEL)
    assert g.need_keyframe(o1, 1.0)
    g.add_keyframe(o1, fg.pose3((2.0, 0.0, -1.0), LEVEL), 1.0, depth_z=-1.0)
    # Tag 0 two metres ahead in the body frame (same tick as the keyframe).
    assert g.add_tag(0, (2.0, 0.0, 0.0))
    assert not g.add_tag(0, (2.0, 0.0, 0.0))          # same keyframe
    assert not g.add_tag(10, (1.0, 0.0, 1.0))         # dynamic deck tag
    g.update()
    g.refresh_sigma()
    now = g.now_pose(o1)
    assert now is not None and abs(now.translation()[0] - 2.0) < 0.3
    lms = g.landmark_list()
    assert len(lms) == 1 and lms[0]["id"] == 0
    assert abs(lms[0]["x"] - 4.0) < 0.5
    assert g.sigma_m is not None and g.sigma_m > 0
    # A later sighting must wait for a new keyframe (stale bearing would bias L).
    assert not g.add_tag(0, (2.1, 0.0, 0.0))
    o2 = fg.pose3((13.0, 0.0, -1.0), LEVEL)
    assert g.need_keyframe(o2, 2.0, tag=True)
    g.add_keyframe(o2, fg.pose3((3.0, 0.0, -1.0), LEVEL), 2.0, depth_z=-1.0)
    assert g.add_tag(0, (2.0, 0.0, 0.0))
    g.update()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "g.json")
        g.dump(path)
        assert os.path.getsize(path) > 50
    print("factor graph ok")


if __name__ == "__main__":
    main()
