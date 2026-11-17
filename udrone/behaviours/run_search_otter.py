#!/usr/bin/env python3
"""Search for otter_k using the deck underside AprilTag (id 10) only.

Flow:
  1. Aft station on the boat centreline
  2. Expanding lateral sinusoid from aft (coverage) until under the nest
  3. Hold under the nest; up-cam locks deck tag 10 → PASS (no latch)

Chain with ``rendezvous`` to climb and ATTACH. Warm-start skips aft when
already under.
Phrase: ``xudrone, search for otter_k`` / ``find otter_k then rendezvous``.
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy

import maneuver_power as mp
from rendezvous_geom import (
    AFT_ARRIVE_M,
    AFT_DEPTH_M,
    AFT_STANDOFF_M,
    COMMIT_ARRIVE_M,
    SEARCH_ALONG_MPS,
    SEARCH_AMP0_M,
    SEARCH_AMP_MAX_M,
    SEARCH_AMP_RATE,
    SEARCH_OMEGA,
    aft_sine_coverage,
    aft_sine_polyline,
    aft_station,
    approach_corridor,
    under_nest,
)
from run_rendezvous import AFT_PITCH_RAD, FOUND_HITS, UdroneRendezvous
from udrone_sinusoidal_controller import (
    PITCH_MAX_RAD,
    UdroneHelixController,
    spin_until_done,
)

HOLD_S = 1.5  # fresh deck tag under the nest this long → PASS


class UdroneSearchOtter(UdroneRendezvous):
    """Aft → expanding sine → under nest → deck tag 10 lock (no latch)."""

    def __init__(self, **kwargs):
        kwargs.setdefault(
            "duration_s",
            float(os.environ.get("UDRONE_SEARCH_OTTER_BUDGET", "120")) + 30.0,
        )
        super().__init__(**kwargs)
        self.budget_s = float(os.environ.get(
            "UDRONE_SEARCH_OTTER_BUDGET", min(120.0, self.budget_s),
        ))
        self._fail_budget_s = self.budget_s / max(0.35, self._power)
        self._dock_mpc = False
        self._acquired = False
        self._acquired_tm = None
        self._cover_t0 = None
        self._sine_path = None
        self._sine_meta = (0.0, 0.0, 0.0)  # along, amp, lat
        self.hold_s = float(os.environ.get("UDRONE_SEARCH_OTTER_HOLD", HOLD_S))
        self._along_mps = max(0.08, mp.scale(SEARCH_ALONG_MPS, self._power))
        self._amp_rate = max(0.02, mp.scale(SEARCH_AMP_RATE, self._power))
        self._amp0 = SEARCH_AMP0_M
        self._amp_max = SEARCH_AMP_MAX_M
        self._omega = SEARCH_OMEGA
        self._stage = "aft"
        self.get_logger().info(
            f"search otter_k: aft → sine → under → deck tag {self.tag_id} "
            f"(up-cam, no latch); along={self._along_mps:.2f} m/s "
            f"amp0={self._amp0:g}→{self._amp_max:g} hold={self.hold_s:g}s "
            f"budget={self._fail_budget_s:g}s"
        )

    def _ready_to_dock(self):
        return False

    def _request_attach(self, cone):
        return

    def _under_ok(self, boat):
        """True when close enough under the nest for a search PASS."""
        if self._auv_pos is None or boat is None:
            return False
        corr = approach_corridor(
            self._auv_pos, boat[0], boat[1], depth_m=AFT_DEPTH_M,
            prev_phase="under" if self._stage in ("under", "servo") else None,
        )
        return (
            corr["phase"] == "under"
            or corr["dist_under"] <= max(COMMIT_ARRIVE_M, 1.0)
        )

    def _lock_quality(self):
        """Deck tag at FOUND_HITS with a fresh frame and depth in band."""
        return (
            self._hits >= FOUND_HITS
            and self._fresh()
            and self._depth_ok()
        )

    def _cover_goal(self, tm, boat):
        """Expanding sinusoid from the live aft station (boat frame)."""
        aft = aft_station(boat[0], boat[1], aft_m=AFT_STANDOFF_M, depth_m=AFT_DEPTH_M)
        corr = approach_corridor(
            self._auv_pos or aft, boat[0], boat[1], depth_m=AFT_DEPTH_M,
            prev_phase="aft" if self._stage == "aft" else "commit",
        )
        self._corridor = corr
        # Sine carried us under → stop covering and hold the nest axis.
        if corr["phase"] == "under" or corr["dist_under"] <= COMMIT_ARRIVE_M:
            if self._stage != "under":
                self._stage = "under"
                print("SEARCH_OTTER_UNDER nest axis (sine reached under)", flush=True)
            return corr["under"], (0.0, 0.0, 0.0)
        if self._cover_t0 is None:
            self._cover_t0 = tm
            self._stage = "cover"
            print(
                f"SEARCH_OTTER_COVER aft=({aft[0]:.1f},{aft[1]:.1f},{aft[2]:.2f}) "
                f"sine amp {self._amp0:g}→{self._amp_max:g} m "
                f"ω={self._omega:.2f} along={self._along_mps:.2f} m/s",
                flush=True,
            )
        pt, along, amp, lat = aft_sine_coverage(
            aft, corr["fwd"], corr["left"], tm - self._cover_t0,
            along_mps=self._along_mps,
            amp0_m=self._amp0,
            amp_rate=self._amp_rate,
            amp_max_m=self._amp_max,
            omega=self._omega,
            depth_z=aft[2],
        )
        self._sine_meta = (along, amp, lat)
        self._sine_path = aft_sine_polyline(
            aft, corr["fwd"], corr["left"], duration_s=min(36.0, tm - self._cover_t0 + 8.0),
            dt=0.6,
            along_mps=self._along_mps,
            amp0_m=self._amp0,
            amp_rate=self._amp_rate,
            amp_max_m=self._amp_max,
            omega=self._omega,
            depth_z=aft[2],
        )
        if self._stage != "cover":
            self._stage = "cover"
        return pt, self._dive_rel_vel(self._auv_pos, z_des=aft[2])

    def _approach_goal(self, tm, auv, boat):
        """Aft → sine until under; then hold for deck tag 10."""
        # Once under (or already locking), park / servo on the nest axis.
        if self._stage in ("under", "servo") or self._under_ok(boat):
            under = under_nest(boat[0], boat[1], AFT_DEPTH_M)
            if self._stage not in ("under", "servo"):
                self._stage = "under"
                print("SEARCH_OTTER_UNDER nest axis, waiting for deck tag", flush=True)
            # Dive and hold depth before counting a tag PASS / servo.
            if self._dive_needed(auv[0], z_des=under[2]) or not self._depth_ok(
                auv[0], z_des=under[2],
            ):
                return under, self._dive_rel_vel(auv[0], z_des=under[2])
            if self._lock_quality():
                goal = self._servo_goal(tm, auv, boat)
                if goal is not None:
                    return goal
            return under, (0.0, 0.0, 0.0)

        # Reach aft first, then expand sinusoidally.
        if self._stage in ("aft", None, ""):
            corr = approach_corridor(
                auv[0], boat[0], boat[1], depth_m=AFT_DEPTH_M, prev_phase="aft",
            )
            self._corridor = corr
            if corr["dist_aft"] > AFT_ARRIVE_M and corr["phase"] == "aft":
                if self._stage != "aft":
                    self._stage = "aft"
                return corr["aft"], (0.0, 0.0, 0.0)
            return self._cover_goal(tm, boat)

        if self._stage == "cover":
            return self._cover_goal(tm, boat)

        return self._cover_goal(tm, boat)

    def _drop_to_search(self, tm, boat):
        """Lost deck tag under nest: resume sine from aft coverage clock."""
        self._lock_range = None
        self._lock_tm = None
        self._hits = 0
        self._servo_prev = None
        if self._cover_t0 is None:
            self._cover_t0 = tm
        self._stage = "cover"
        print("SEARCH_OTTER_LOST resume sine coverage", flush=True)
        return self._cover_goal(tm, boat)

    def _update(self, tm, auv, boat):
        if boat is None:
            return
        self._auv_pos = auv[0]
        self._boat = boat
        self._warm_started = True
        from rendezvous_geom import cone_state
        cone = cone_state(auv[0], auv[1], boat[0], boat[1], 1.0)
        cone["pos"] = auv[0]
        self._cone = cone
        goal = self._approach_goal(tm, auv, boat)
        if goal is None:
            return
        station, rel_vel = goal
        self._des = station
        boat_v = self._boat_vel.update(tm, boat[0][0], boat[0][1])
        self._vel = tuple(boat_v[i] + float(rel_vel[i]) for i in range(3))
        if time.time() >= self._log_t:
            self._log_t = time.time() + 2.0
            along, amp, lat = self._sine_meta
            print(
                f"SEARCH_OTTER {self._stage} des=({self._des[0]:.1f},{self._des[1]:.1f},"
                f"{self._des[2]:.2f}) deck_hits={self._hits} "
                f"z={auv[0][2]:.2f} depth_ok={self._depth_ok()} "
                f"along={along:.1f} amp={amp:.1f} lat={lat:+.1f}",
                flush=True,
            )
            if (
                self._stage in ("under", "servo")
                and self._under_ok(boat)
                and self._depth_ok()
                and self._hits < FOUND_HITS
            ):
                print(
                    f"SEARCH_OTTER_BLIND depth_ok hits={self._hits} "
                    f"fresh={self._fresh()} waiting for deck tag {self.tag_id}",
                    flush=True,
                )
        if self._acquired:
            return
        # PASS only when under + depth band + fresh deck lock (not mid-sine).
        if (
            self._stage in ("under", "servo")
            and self._under_ok(boat)
            and self._lock_quality()
        ):
            if self._acquired_tm is None:
                self._acquired_tm = tm
                print(
                    f"SEARCH_OTTER_LOCK deck={self._hits} stage={self._stage} "
                    f"z={auv[0][2]:.2f} holding {self.hold_s:g}s",
                    flush=True,
                )
            elif tm - self._acquired_tm >= self.hold_s:
                self._acquired = True
                print(
                    f"SEARCH_OTTER_FOUND deck tag {self.tag_id} under otter_k "
                    f"z={auv[0][2]:.2f}",
                    flush=True,
                )
        else:
            self._acquired_tm = None

    def _control(self, tm, auv, boat):
        self._update(tm, auv, boat)
        pos = auv[0]
        # Cover/aft: force-directed yaw, dive pitch when shallow. Under/servo:
        # boat yaw only after depth band (up-cam), else dive authority.
        if self._stage in ("under", "servo"):
            self._nest_attitude(
                tm, pos, boat,
                prefer_boat_yaw=self._depth_ok(pos),
                cruise_pitch=AFT_PITCH_RAD,
            )
        elif self._stage in ("aft", "cover"):
            self._nest_attitude(
                tm, pos, boat, prefer_boat_yaw=False, cruise_pitch=AFT_PITCH_RAD,
            )
        else:
            self.yaw_hold = None
            self.pitch_limit = PITCH_MAX_RAD
        return UdroneHelixController._control(self, tm, auv, boat)

    def _finished(self, tm: float) -> bool:
        if self._acquired or self.failed:
            return True
        if tm >= self._fail_budget_s:
            self._fail(f"did not see deck tag {self.tag_id} under otter_k")
            return True
        return False

    def _hud_layers(self):
        out = UdroneRendezvous._hud_layers(self)
        out["camera"] = "up" if self._stage in ("under", "servo") else "front"
        out["stage"] = f"search:{self._stage}"
        along, amp, lat = self._sine_meta
        out["lines"] = list(out.get("lines") or [])
        out["lines"].insert(
            0,
            f"search otter  {self._stage}  "
            f"deck {self.tag_id}:{self._hits}/{FOUND_HITS}  "
            f"amp {amp:.1f} m  lat {lat:+.1f}  "
            f"{'ACQUIRED' if self._acquired else 'seeking'}",
        )
        if self._sine_path and len(self._sine_path) >= 2 and self._stage == "cover":
            out["paths"] = list(out.get("paths") or [])
            out["paths"].insert(0, {
                "pts": self._sine_path,
                "color": (0, 180, 255),
                "width": 2,
                "label": "sine cover",
            })
        return out


def main():
    rclpy.init()
    node = UdroneSearchOtter()
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/coral_search_otter_ctrl_log.txt")
        why = node.failed or "no samples"
        if node._acquired and not node.failed:
            ok = True
            why = f"found deck tag {node.tag_id} under otter_k"
        elif not node.failed:
            why = f"did not find otter_k deck tag {node.tag_id}"
        print(f"SEARCH_OTTER_BEHAVIOUR {'PASS' if ok else 'FAIL'} {why}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
