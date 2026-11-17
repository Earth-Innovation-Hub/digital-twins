"""Hoop-axis CEM MPC: stays in the tube and advances through the opening."""
import math

from hoop_mpc import (
    DT, FINAL_LAT_M, FINAL_S_M, FINAL_VS_CAP, TUBE_R,
    enabled, hoop_state, plan, tube_ok,
)


def main():
    assert enabled({"UDRONE_HOOP_MPC": "1"})
    assert not enabled({"UDRONE_HOOP_MPC": "0"})

    center = (6.0, 0.0, -4.626)
    axis = (1.0, 0.0)  # +east through
    # Approach from west, slightly high and left.
    pos = (4.2, 0.25, -4.40)
    s0, y0, z0, *_ = hoop_state(pos, center, axis)
    assert s0 < 0.0 and abs(y0 - 0.25) < 1e-6

    out = plan(pos, center, axis, speed_mps=0.32, exit_m=2.0)
    assert "point" in out and "vel" in out and "v_lag" in out
    vs, vy, vz = out["u0"]
    assert vs > 0.0, out["u0"]           # surge toward exit
    assert vy < 0.0, out["u0"]           # pull toward axis (y was +0.25)
    # Soft capture far out: residual lateral remains (no full snap).
    assert abs(out["point"][1] - center[1]) > 0.02, out["point"]
    # Lead must put the tracked point ahead of the hull toward the exit.
    s_pt, y_pt, z_pt, *_ = hoop_state(out["point"], center, axis)
    assert s_pt > s0 + 0.35, (s0, s_pt, out["point"])
    assert out["point"][0] > pos[0] + 0.25, out["point"]
    assert abs(y_pt) < abs(y0), (y0, y_pt)

    # Final metre, off-axis: crawl + hard-lock the opening — do not punch the rim.
    near = (5.2, 0.22, -4.50)
    s_n, y_n, z_n, *_ = hoop_state(near, center, axis)
    assert -FINAL_S_M <= s_n < 0.0 and abs(y_n) > FINAL_LAT_M
    fin = plan(near, center, axis, speed_mps=0.32, exit_m=2.0)
    assert fin["final_gate"], fin
    assert fin["u0"][0] <= FINAL_VS_CAP + 1e-9, fin["u0"]
    _s_f, y_f, z_f, *_ = hoop_state(fin["point"], center, axis)
    assert abs(y_f) < abs(y_n) * 0.25, (y_n, y_f)
    assert math.hypot(y_f, z_f) < FINAL_LAT_M + 0.08, (y_f, z_f)

    # Receding: integrate lagged world velocity; cross the plane in the tube.
    warm = None
    v_state = (0.0, 0.0, 0.0)
    p = pos
    crossed = False
    for _ in range(80):
        step = plan(p, center, axis, speed_mps=0.32, exit_m=2.0, warm=warm, v_state=v_state)
        warm = step["warm"]
        v_state = step["v_lag"]
        fwd, left, up = step["fwd"], step["left"], step["up"]
        p = tuple(
            p[i]
            + v_state[0] * DT * fwd[i]
            + v_state[1] * DT * left[i]
            + v_state[2] * DT * up[i]
            for i in range(3)
        )
        s, y, z, *_ = hoop_state(p, center, axis)
        if s > 0.0:
            crossed = True
        if s > 1.5:
            break
        # Final metre must stay near the opening, not walk onto the rim.
        if -FINAL_S_M <= s < 0.2:
            assert math.hypot(y, z) < TUBE_R * 1.6, (s, y, z)
        else:
            assert math.hypot(y, z) < TUBE_R * 2.5, (s, y, z)

    assert crossed, "did not cross the opening plane"
    s_f, y_f, z_f, *_ = hoop_state(p, center, axis)
    assert s_f > 0.5
    assert tube_ok(y_f, z_f) or math.hypot(y_f, z_f) < TUBE_R * 1.2

    assert tube_ok(0.0, 0.0)
    assert not tube_ok(0.5, 0.0)
    print("hoop mpc ok")


if __name__ == "__main__":
    main()
