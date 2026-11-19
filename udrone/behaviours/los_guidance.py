"""Line-of-sight waypoint guidance for a hull that only thrusts along its nose.

There is no time parameter. The hull steers onto the line between the current
pair of waypoints and holds a surge speed, so nothing runs ahead of it and a
slow hull is never pulled toward a point it has fallen behind.

  heading  psi   = chi_leg   - atan(e_left / delta)
  pitch    gamma = gamma_leg - atan(e_up / delta), limited to pitch_max
  speed    ramps up at accel, and is capped so it can slow for a corner or
           stop on the last waypoint at decel. It falls to zero while the
           heading is more than SURGE_CONE off, so the hull turns in place
           before it surges.

Past the last waypoint the speed goes negative and the hull backs up along the
line; the heading and pitch corrections flip so backing up still closes the
error. World ENU, z up. ROS-free so it can be tested on its own.
"""
from __future__ import annotations

import math

ACCEL_MPS2 = 1.0
DECEL_MPS2 = 0.6
SLEW_DOWN_MPS2 = 1.5      # how fast the speed reference may fall (heading gate)
END_GAIN = 1.0            # 1/s: inside the last metre speed = END_GAIN * distance
LOOKAHEAD_MIN_M = 1.5
LOOKAHEAD_MAX_M = 4.0
LOOKAHEAD_S = 0.5         # extra look-ahead per m/s of speed
VLOOK_MIN_M = 1.0         # depth look-ahead: shorter, depth drifts while the hull is fast
VLOOK_MAX_M = 2.5
VLOOK_S = 0.3
# The nose rides several degrees under the command at speed. The integrator
# learns that bias from the measured elevation, only while the hull moves.
KI_ELEV = 1.0             # 1/s
ELEV_I_MAX_RAD = math.radians(12.0)
ELEV_I_MIN_MPS = 0.3
SURGE_CONE_RAD = math.radians(30.0)
STEER_STEP_RAD = math.radians(75.0)
CORNER_MIN_MPS = 0.4
SWITCH_M = 1.0            # an interior waypoint is passed this far before it
LEG_MIN_M = 0.5           # shorter horizontal legs cannot be steered along
REVERSE_DEADBAND_MPS = 0.05


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LosGuide:
    def __init__(self, waypoints, cruise_mps, pitch_max_rad=math.radians(45.0),
                 accel=ACCEL_MPS2, decel=DECEL_MPS2):
        pts = []
        for p in waypoints:
            p = (float(p[0]), float(p[1]), float(p[2]))
            if not pts or math.dist(p, pts[-1]) > 1e-6:
                pts.append(p)
        if len(pts) < 2:
            raise ValueError("LOS needs two distinct waypoints")
        for a, b in zip(pts, pts[1:]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) < LEG_MIN_M:
                raise ValueError(f"leg {a}->{b} is shorter than {LEG_MIN_M} m horizontally")
        self.pts = pts
        self.cruise = max(0.05, float(cruise_mps))
        self.pitch_max = pitch_max_rad
        self.accel = float(accel)
        self.decel = float(decel)
        self.leg = 0
        self.u_ref = 0.0
        self.elev_i = 0.0
        self._reverse = False

    @property
    def end(self):
        return self.pts[-1]

    def length(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.pts, self.pts[1:]))

    def eta(self) -> float:
        """Rough flight time: accelerate, cruise, brake, then the last metre."""
        total = 0.0
        a_eff = min(self.accel, self.decel)
        for a, b in zip(self.pts, self.pts[1:]):
            d = math.dist(a, b)
            v = self.cruise
            ramp = v * v / a_eff
            total += 2.0 * v / a_eff + (d - ramp) / v if d > ramp else 2.0 * math.sqrt(d / a_eff)
        return total + 3.0 / END_GAIN

    def _leg(self, i):
        a, b = self.pts[i], self.pts[i + 1]
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        lh = math.hypot(dx, dy)
        return a, b, lh, math.atan2(dy, dx), math.atan2(dz, lh)

    def _along(self, i, pos):
        a, _, lh, chi, _ = self._leg(i)
        dx, dy = pos[0] - a[0], pos[1] - a[1]
        c, s = math.cos(chi), math.sin(chi)
        return c * dx + s * dy, -s * dx + c * dy

    def _corner_speed(self, i) -> float:
        """Speed to arrive at the end of leg i with (0 at the last waypoint)."""
        if i >= len(self.pts) - 2:
            return 0.0
        chi0 = self._leg(i)[3]
        chi1 = self._leg(i + 1)[3]
        turn = abs(wrap_pi(chi1 - chi0))
        return max(CORNER_MIN_MPS, self.cruise * 0.5 * (1.0 + math.cos(turn)))

    def step(self, pos, yaw, dt, elev=None):
        """Guidance for one tick. elev is the measured nose elevation (rad, up
        positive) for the bias integrator. Returns a dict; ``dir`` is the hull
        direction."""
        last = len(self.pts) - 2
        while self.leg < last:
            s, _ = self._along(self.leg, pos)
            lh = self._leg(self.leg)[2]
            if s >= lh - min(SWITCH_M, 0.5 * lh):
                self.leg += 1
            else:
                break
        a, b, lh, chi, gamma_leg = self._leg(self.leg)
        s, e_left = self._along(self.leg, pos)
        frac = _clamp(s / lh, 0.0, 1.0)
        z_line = a[2] + (b[2] - a[2]) * frac
        e_up = pos[2] - z_line
        if not 0.0 < s < lh:
            gamma_leg = 0.0
        rem = lh - s

        if self.leg == last:
            mag = min(self.cruise, math.sqrt(2.0 * self.decel * abs(rem)), END_GAIN * abs(rem))
            u_t = math.copysign(mag, rem)
        else:
            v_next = self._corner_speed(self.leg)
            u_t = min(self.cruise, math.sqrt(v_next * v_next + 2.0 * self.decel * max(rem, 0.0)))

        if abs(self.u_ref) > REVERSE_DEADBAND_MPS:
            self._reverse = self.u_ref < 0.0
        elif abs(u_t) > REVERSE_DEADBAND_MPS:
            self._reverse = u_t < 0.0
        back = -1.0 if self._reverse else 1.0

        delta = _clamp(LOOKAHEAD_MIN_M + LOOKAHEAD_S * abs(self.u_ref), LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M)
        psi = wrap_pi(chi - back * math.atan2(e_left, delta))
        vlook = _clamp(VLOOK_MIN_M + VLOOK_S * abs(self.u_ref), VLOOK_MIN_M, VLOOK_MAX_M)
        flight = gamma_leg - math.atan2(e_up, vlook)
        gamma = _clamp(back * flight, -self.pitch_max, self.pitch_max)

        herr = wrap_pi(psi - yaw)
        cos_cone = math.cos(SURGE_CONE_RAD)
        gate = _clamp((math.cos(herr) - cos_cone) / (1.0 - cos_cone), 0.0, 1.0)
        u_t *= gate

        prev = self.u_ref
        if abs(u_t) > abs(prev) and u_t * prev >= 0.0:
            step = self.accel * dt
        else:
            step = SLEW_DOWN_MPS2 * dt
        self.u_ref = prev + _clamp(u_t - prev, -step, step)
        a_ref = (self.u_ref - prev) / dt if dt > 1e-6 else 0.0

        # Aim at most STEER_STEP off the present heading, level while turning
        # in place. An aim behind the nose has no unique rotation axis, and the
        # attitude loop could pitch over instead of yawing round.
        psi_aim = wrap_pi(yaw + _clamp(herr, -STEER_STEP_RAD, STEER_STEP_RAD))
        gamma_aim = gamma if abs(herr) < SURGE_CONE_RAD else 0.0
        if elev is not None and abs(self.u_ref) >= ELEV_I_MIN_MPS and abs(herr) < SURGE_CONE_RAD:
            self.elev_i = _clamp(self.elev_i + KI_ELEV * (gamma_aim - elev) * dt,
                                 -ELEV_I_MAX_RAD, ELEV_I_MAX_RAD)
        gamma_aim = _clamp(gamma_aim + self.elev_i, -self.pitch_max, self.pitch_max)
        ce = math.cos(gamma_aim)
        direction = (ce * math.cos(psi_aim), ce * math.sin(psi_aim), math.sin(gamma_aim))
        dist_end = math.dist(pos, self.end)
        # Point the HUD marks: on the line, one look-ahead ahead of the hull.
        ahead = _clamp(s + back * delta, 0.0, lh)
        c, sn = math.cos(chi), math.sin(chi)
        carrot = (a[0] + c * ahead, a[1] + sn * ahead, a[2] + (b[2] - a[2]) * ahead / lh)
        return {
            "leg": self.leg, "s": s, "rem": rem, "xte": e_left, "vte": e_up,
            "psi": psi, "gamma": gamma, "herr": herr, "gate": gate,
            "u_t": u_t, "u_ref": self.u_ref, "a_ref": a_ref, "delta": delta, "elev_i": self.elev_i,
            "dir": direction, "dist_end": dist_end, "carrot": carrot,
            "reverse": self._reverse,
        }
