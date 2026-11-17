#!/usr/bin/env python3
"""Aquatic ROS 2 helical offboard controller for xudrone.

State for the force loop is PX4 local odometry, turned into the Gazebo ENU
frame. Gazebo ground truth is the sim check: when the estimate is stale or
more than 1.5 m off the truth, that tick flies the truth and says so.
Commands go to PX4 as offboard body torque / thrust
(airframe 60003 has UUV_SKIP_CTRL 1, so vehicle_rates_setpoint is fed through).

Trajectory: forward helix, anchored where the vehicle is when thrust starts and
pointing along its heading at that moment (path frame x forward, y left):
  x = v * t
  y = R * sin(w t)
  z = z0 + R * cos(w t)          (z is world depth, negative down)

Control: xudrone has four thrusters that all push along the hull axis, so it
is steered like a quadrotor lying on its side. The position loop asks for a
force (velocity error + drag + the 0.3 N of net buoyancy); the hull axis is
turned to point along that force with pitch / yaw torque, and the thrust is the
part of the force along the hull. There is no roll authority (ballast keeps it
upright). Cruising behaviours limit the pitch and hold the heading when the
horizontal force is small, backing up rather than turning round for a small
force behind the nose. While the dock latch is attached only thrust is
commanded (the hull cannot turn inside the boat).

Under coral_behaviours (UDRONE_KEEP_ARMED=1) a finished behaviour stays armed:
it holds where it ended and hands the vehicle to run_hold.py, and the next
behaviour takes it from there without a disarm (see the baton helpers).

Works in both modes of the aquatic twin without editing anything:
  solo   xudrone_0, PX4 instance 0
  CoRAL  xudrone_1, PX4 instance 1 (instance 0 is the boat otter_k)
The mode is detected from the Gazebo model list; UDRONE_GZ_MODEL and
UDRONE_PX4_INSTANCE override it. In CoRAL mode release the AUV first
(dock_command.py DETACH), or use udrone_undock_helix_test.py.

All times are simulation seconds, so a run means the same thing whether the
simulator keeps up with the wall clock or not.

Env: UDRONE_HELIX_DUR (45), UDRONE_HELIX_R (0.5), UDRONE_HELIX_OMEGA (0.40),
     UDRONE_HELIX_VX (0.20), UDRONE_HELIX_Z0 (-1.2).
Run: run_test.sh helix   (sources ROS and sets the domain / partition)
Exit status 0 = armed, thrusters driven, and the helix was tracked.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleRatesSetpoint,
    VehicleAttitudeSetpoint,
    VehicleControlMode,
    VehicleOdometry,
    VehicleAttitude,
    VehicleLocalPosition,
)

GZ_ENV = {**os.environ, "GZ_PARTITION": os.environ.get("GZ_PARTITION", "aquatic")}
PX4_COMMANDER = "/root/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"

LOG_COLUMNS = (
    "t x y z x_des y_des z_des thrust yaw_u pitch_u yaw roll "
    "boat_x boat_y boat_z rel_x rel_y rel_z rotor0_N latch"
)


# xudrone model (model/xudrone/model.sdf, airframe 60003, thrust adapter).
# Hull 13.60 kg plus 0.066 kg of sensor and rotor links. Only the four hull
# boxes displace water (13.70 kg), so the vehicle floats with about 0.3 N.
MASS_KG = 13.666
DISPLACED_KG = 13.70
NET_BUOYANCY_N = (DISPLACED_KG - MASS_KG) * 9.81
ROTOR_MAX_N = 36.0                          # per thruster at command 1.0
SURGE_N_PER_UNIT = 4 * ROTOR_MAX_N          # thrust_body[0] = 1
# Measured on the twin: pitch 1.0 drives the motors at +-0.707, yaw 1.0 at +-1.0,
# on a 0.14 m arm. Positive pitch = nose down, positive yaw = nose left
# (Gazebo FLU axes). Roll commands produce no motor output.
PITCH_NM_PER_UNIT = 4 * ROTOR_MAX_N * 0.14 * 0.7071
YAW_NM_PER_UNIT = 4 * ROTOR_MAX_N * 0.14
CG_BELOW_M = 0.02                           # ballast: centre of mass under the centre of buoyancy
DRAG_LIN = (1.2, 4.0, 4.0)                  # N per m/s, body x y z
DRAG_QUAD = (3.0, 35.0, 35.0)               # N per (m/s)^2
INERTIA_YZ = 0.179                          # kg m^2 about the pitch / yaw axes

# Gains. Attitude loop about 4 rad/s, position loop well below it.
ATT_WN = 4.0
KP_PITCH = INERTIA_YZ * ATT_WN ** 2 / PITCH_NM_PER_UNIT
KD_PITCH = 2.0 * INERTIA_YZ * ATT_WN / PITCH_NM_PER_UNIT
KP_YAW = INERTIA_YZ * ATT_WN ** 2 / YAW_NM_PER_UNIT
KD_YAW = 2.0 * INERTIA_YZ * ATT_WN / YAW_NM_PER_UNIT
GRAVITY_FF = MASS_KG * 9.81 * CG_BELOW_M / PITCH_NM_PER_UNIT
TORQUE_MAX = 0.4                            # normalized pitch / yaw command limit
KP_POS = 0.7                                # 1/s: position error -> velocity correction
V_CORR_MAX = 0.4                            # m/s
KV = 1.5                                    # 1/s: velocity error -> acceleration
KI_DEPTH = 1.0                              # N per m s
I_DEPTH_MAX = 3.0                           # N
I_DEPTH_BAND_M = 0.5                        # integrate only this close to the depth
# Hull direction while cruising. Pitch is limited and the heading is held when
# the horizontal force is too small to give one. Force behind the nose backs
# the hull up unless it is large, then the hull turns round.
PITCH_MAX_RAD = math.radians(float(os.environ.get("UDRONE_PITCH_MAX_DEG", "") or 45.0))
HEAD_MIN_N = 2.0
HEAD_TURN_N = 8.0
HEAD_BEHIND_RAD = math.radians(120.0)
ELEV_KEEP = math.tan(math.radians(25.0))   # force this close to vertical keeps the pitch side
# Progress gate: the reference slows as the hull falls behind and stops at GATE_E1.
GATE_E0 = 0.6
GATE_E1 = 2.0
# Baton: the process flying the armed vehicle hands it to the next one without
# a disarm. The owner's pid is in the baton file and a successor writes its pid
# to the request file.
BATON_DIR = os.environ.get("UDRONE_BATON_DIR", "") or "/tmp"
HANDOFF_WAIT_S = 20.0
BATON_WAIT_S = 5.0
HOLD_RUNNER = os.environ.get("UDRONE_HOLD_RUNNER", "") or "/root/udrone_boat/behaviours/run_hold.py"


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def pid_alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def baton_paths(gz_model: str):
    """(baton file, request file) for one vehicle."""
    stem = os.path.join(BATON_DIR, f"udrone_baton_{gz_model}")
    return stem + ".json", stem + ".req"


def baton_owner(gz_model: str, me: int = 0):
    """Pid of a live process holding this vehicle's baton, other than `me`, or None."""
    rec = read_json(baton_paths(gz_model)[0])
    pid = rec.get("pid") if isinstance(rec, dict) else None
    if pid is None or int(pid) == int(me) or not pid_alive(pid):
        return None
    return int(pid)


def _pose_frame():
    """pose_frame.py lives with the behaviours. The helix script lives in src or test/."""
    here = os.path.dirname(os.path.abspath(__file__))
    for folder in (here, os.path.abspath(os.path.join(here, "..", "behaviours"))):
        if os.path.isfile(os.path.join(folder, "pose_frame.py")) and folder not in sys.path:
            sys.path.insert(0, folder)
    import pose_frame
    return pose_frame


def _hud_module():
    """hud_overlay.py lives with the behaviours, which may not be on the path yet."""
    here = os.path.dirname(os.path.abspath(__file__))
    for folder in (here, os.path.abspath(os.path.join(here, "..", "behaviours"))):
        if os.path.isfile(os.path.join(folder, "hud_overlay.py")) and folder not in sys.path:
            sys.path.insert(0, folder)
    import hud_overlay
    return hud_overlay


def _maneuver_power():
    here = os.path.dirname(os.path.abspath(__file__))
    for folder in (here, os.path.abspath(os.path.join(here, "..", "behaviours"))):
        if os.path.isfile(os.path.join(folder, "maneuver_power.py")) and folder not in sys.path:
            sys.path.insert(0, folder)
    import maneuver_power
    return maneuver_power


_pf = _pose_frame()
_mp = _maneuver_power()
ned_to_enu = _pf.ned_to_enu
px4_q_to_enu = _pf.px4_q_to_enu


def body_axes(q):
    """World-frame unit vectors of the body x (forward), y (left), z (up) axes."""
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )


def body_rates(q0, q1, dt):
    """Body angular velocity (rad/s) that turns attitude q0 into q1 in dt."""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    # dq = conj(q0) * q1
    dw = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    dx = w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1
    dy = w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1
    dz = w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1
    s = 2.0 / dt if dw >= 0 else -2.0 / dt
    return (s * dx, s * dy, s * dz)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rpy_from_q(q):
    """Roll, pitch, yaw (ZYX) from a quaternion given as w, x, y, z."""
    w, x, y, z = [float(v) for v in q[:4]]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rotate_inverse(q, v):
    """v expressed in the frame whose orientation is q (w, x, y, z)."""
    w, x, y, z = q
    # Rows of R(q) transposed applied to v.
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z); r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y); r21 = 2 * (y * z + w * x); r22 = 1 - 2 * (x * x + y * y)
    return (
        r00 * v[0] + r10 * v[1] + r20 * v[2],
        r01 * v[0] + r11 * v[1] + r21 * v[2],
        r02 * v[0] + r12 * v[1] + r22 * v[2],
    )


class GzStream:
    """Feeds every line of `gz topic -e -t <topic>` to a callback, from a thread."""

    def __init__(self, topic, on_line):
        self.topic = topic
        self.on_line = on_line
        self.proc = None
        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self._stop:
            try:
                self.proc = subprocess.Popen(
                    ["gz", "topic", "-e", "-t", self.topic],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, bufsize=1, env=GZ_ENV,
                )
                for line in self.proc.stdout:
                    if self._stop:
                        break
                    self.on_line(line)
            except Exception:
                pass
            finally:
                self._kill()
            if not self._stop:
                time.sleep(0.5)

    def _kill(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass

    def stop(self):
        self._stop = True
        self._kill()


class GzPoses:
    """World poses of every moving model, with the simulation time they belong to.

    Reads /world/<world>/dynamic_pose/info (about 56 Hz) as one stream. Each
    snapshot is one Gazebo message, so two models read together are from the
    same instant. Protobuf text omits fields that are exactly zero; they are
    filled in as zero here.
    """

    def __init__(self, world="kiwanis_lake"):
        self.lock = threading.Lock()
        self.snap = {}
        self.sim_t = None
        self.rx_wall = 0.0
        self.n_msgs = 0
        self._stack = []
        self._msg = {}
        self._cur = None
        self._sec = 0
        self._nsec = 0
        self.stream = GzStream(f"/world/{world}/dynamic_pose/info", self._line)

    def _publish(self):
        if self._msg:
            with self.lock:
                self.snap = self._msg
                self.sim_t = self._sec + self._nsec * 1e-9
                self.rx_wall = time.time()
                self.n_msgs += 1
        self._msg = {}
        self._sec = self._nsec = 0

    def _line(self, line):
        s = line.strip()
        if not s:
            return
        if s.endswith("{"):
            name = s[:-1].strip()
            if not self._stack:
                if name == "header":
                    self._publish()        # the previous message is complete
                elif name == "pose":
                    self._cur = {"name": "", "p": [0.0, 0.0, 0.0], "q": [1.0, 0.0, 0.0, 0.0]}
            elif name == "orientation" and self._cur is not None and self._stack == ["pose"]:
                self._cur["q"] = [0.0, 0.0, 0.0, 0.0]
            self._stack.append(name)
            return
        if s == "}":
            if not self._stack:
                return
            top = self._stack.pop()
            if top == "pose" and not self._stack and self._cur is not None:
                self._msg[self._cur["name"]] = (tuple(self._cur["p"]), tuple(self._cur["q"]))
                self._cur = None
            return
        key, _, val = s.partition(":")
        val = val.strip()
        try:
            if self._stack == ["header", "stamp"]:
                if key == "sec":
                    self._sec = int(val)
                elif key == "nsec":
                    self._nsec = int(val)
            elif self._cur is not None:
                if self._stack == ["pose"] and key == "name":
                    self._cur["name"] = val.strip('"')
                elif self._stack == ["pose", "position"] and key in ("x", "y", "z"):
                    self._cur["p"]["xyz".index(key)] = float(val)
                elif self._stack == ["pose", "orientation"] and key in ("w", "x", "y", "z"):
                    self._cur["q"]["wxyz".index(key)] = float(val)
        except ValueError:
            pass

    def read(self, *names):
        """(fresh, sim time, [pose or None per name]); pose = (xyz, quaternion wxyz)."""
        with self.lock:
            fresh = self.sim_t is not None and (time.time() - self.rx_wall) < 1.0
            return fresh, self.sim_t, [self.snap.get(n) for n in names]

    def stop(self):
        self.stream.stop()


class LatchState:
    """attached | detached from the dock latch (published every second and on change)."""

    def __init__(self, topic="/model/heron_0/udrone_dock/state"):
        self.state = None
        self.changes = []            # (wall time, state)
        self.stream = GzStream(topic, self._line)

    def _line(self, line):
        s = line.strip()
        if s.startswith("data:"):
            new = s.partition(":")[2].strip().strip('"')
            if new != self.state:
                self.changes.append((time.time(), new))
            self.state = new

    def code(self):
        return {"attached": 1.0, "detached": 0.0}.get(self.state, -1.0)

    def stop(self):
        self.stream.stop()


class ThrustProbe:
    """Newtons the adapter is commanding on one Gazebo thruster (0 when idle).

    This is the last hop of PX4 -> adapter -> Gazebo, so a non-zero reading
    means the whole chain is alive. A zero command prints as an empty message.
    """

    def __init__(self, model, rotor=0):
        self.value = 0.0
        self.rx_wall = 0.0
        self.stream = GzStream(f"/model/{model}/joint/rotor_{rotor}_joint/cmd_thrust", self._line)

    def _line(self, line):
        s = line.strip()
        if s.startswith("data:"):
            try:
                self.value = float(s.partition(":")[2])
                self.rx_wall = time.time()
            except ValueError:
                pass

    def newtons(self):
        return self.value if (time.time() - self.rx_wall) < 0.3 else 0.0

    def stop(self):
        self.stream.stop()


def detect_target():
    """(gz model, PX4 instance) for the xudrone in the running simulator."""
    model = os.environ.get("UDRONE_GZ_MODEL")
    inst = os.environ.get("UDRONE_PX4_INSTANCE")
    listed = ""
    if not (model and inst):
        try:
            listed = subprocess.run(
                ["gz", "model", "--list"], capture_output=True, text=True, timeout=15, env=GZ_ENV,
            ).stdout
        except Exception:
            listed = ""
    dual = "xudrone_1" in listed
    return model or ("xudrone_1" if dual else "xudrone_0"), int(inst) if inst else (1 if dual else 0)


class UdroneHelixController(Node):
    def __init__(
        self,
        duration_s=45.0,
        radius=0.7,
        omega=0.40,
        x_rate=0.20,
        z0=-1.5,
        gz_model=None,
        px4_instance=None,
        boat_model="heron_0",
    ):
        super().__init__("udrone_helix_controller")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if gz_model is None or px4_instance is None:
            found_model, found_inst = detect_target()
            gz_model = gz_model or found_model
            px4_instance = found_inst if px4_instance is None else px4_instance
        self.gz_model = gz_model
        self.px4_instance = int(px4_instance)
        self.boat_model = boat_model

        ns = "/xudrone/fmu"
        self.pub_off = self.create_publisher(OffboardControlMode, f"{ns}/in/offboard_control_mode", qos)
        self.pub_rates = self.create_publisher(VehicleRatesSetpoint, f"{ns}/in/vehicle_rates_setpoint", qos)
        self.pub_att = self.create_publisher(VehicleAttitudeSetpoint, f"{ns}/in/vehicle_attitude_setpoint", qos)

        self.vmode = None
        self.odom = None
        self.att = None
        self.create_subscription(VehicleControlMode, f"{ns}/out/vehicle_control_mode", lambda m: setattr(self, "vmode", m), qos)
        self.create_subscription(VehicleOdometry, f"{ns}/out/vehicle_odometry", self._on_odom, qos)
        self.create_subscription(VehicleAttitude, f"{ns}/out/vehicle_attitude", lambda m: setattr(self, "att", m), qos)
        self.create_subscription(VehicleLocalPosition, f"{ns}/out/vehicle_local_position", self._on_local, qos)

        self.radius = float(radius)
        self.omega = float(omega)
        self.x_rate = float(x_rate)
        self.z0 = float(z0)
        self.duration_s = float(duration_s)

        self.gz = GzPoses()
        self.latch = LatchState()
        self.probe = ThrustProbe(self.gz_model)

        self.phase = "wait_pose"
        self.t_wall0 = time.time()
        self.last_ok_wall = self.t_wall0
        self.t0 = None               # sim time of the first pose
        self.t_thrust0 = None        # sim time thrust started
        self.anchor = None           # (x0, y0, yaw0) of the path frame
        self.next_arm = 0.0
        self.next_print = 0.0
        self.armed_ok = False
        self.done = False
        self.failed = ""
        self.log = []
        self._prev = None            # (sim t, position, quaternion) of the last control step
        self.vel = (0.0, 0.0, 0.0)   # world velocity estimate
        self.w_body = (0.0, 0.0, 0.0)
        self.depth_i = 0.0           # N, learned correction to the buoyancy estimate
        self._dir = None             # last commanded hull direction
        self.yaw_hold = None         # world ENU yaw; None points the nose along the force
        self._stop_requested = False
        self._sim_anchor = None      # (sim time, wall time) of the last fresh Gazebo clock
        self._vel_meas = None        # ENU m/s from PX4, or None to difference the pose
        self._vpos = None            # (n, e, d, vn, ve, vd, wall)
        self._odom_q = None
        self._fuser = _pf.PoseFuser()
        self.pose_source = "gazebo"
        self._gz_miss = 0.0
        self.pitch_limit = PITCH_MAX_RAD   # None lets the hull point anywhere (shapes, helix)
        self._head = None            # held hull heading while cruising
        self._elev_sign = None       # +1 forward thrust, -1 reverse, kept near vertical
        self.progress_gate = False   # reference waits for the hull (swim, shape, coverage)
        self.path_t = 0.0            # reference clock; equals thrust time without the gate
        self._gate_rate = 1.0
        # Fraction of full thrust / gains (UDRONE_POWER_PCT or set_power). Lower is
        # slower and smoother; 1.0 is the historical full-authority loop.
        self.power = _mp.from_env()
        self._err_n = 0.0
        self.keep_armed = os.environ.get("UDRONE_KEEP_ARMED", "") == "1"
        self.offers_baton = False    # the hold runner owns the baton while it flies
        self.baton_only = False      # the hold runner never arms; it only takes over
        self._baton, self._baton_req = baton_paths(self.gz_model)
        self._baton_t = None         # wall time this process asked for the baton
        self._baton_write_t = 0.0
        self._hold_point = None      # world (x, y, z) flown while handing off
        self._handoff_t = None
        self._p_des = None           # world reference of the last control step
        self.hud_preview = True      # draw the next seconds of the reference on the HUD
        self.hud = None
        if os.environ.get("UDRONE_HUD", "1") != "0":
            try:
                self.hud = _hud_module().TaskHud(self)
            except Exception as exc:
                print(f"HUD off ({exc!r})", flush=True)
        signal.signal(signal.SIGTERM, self._on_stop_signal)
        signal.signal(signal.SIGINT, self._on_stop_signal)
        self.create_timer(0.05, self._tick)
        pitch = 2.0 * math.pi * self.x_rate / max(self.omega, 1e-6)
        self.get_logger().info(
            f"helix: model={self.gz_model} px4 instance={self.px4_instance} "
            f"R={self.radius} w={self.omega} vx={self.x_rate} z0={self.z0} "
            f"pitch≈{pitch:.2f} m/rev dur={self.duration_s} s (sim time)"
        )

    def _on_stop_signal(self, _signum, _frame):
        # Handled on the next tick so disarm does not run inside the signal.
        self._stop_requested = True

    def _on_odom(self, m: VehicleOdometry):
        self.odom = m
        if len(m.q) >= 4:
            self._odom_q = px4_q_to_enu(m.q)

    def _on_local(self, m: VehicleLocalPosition):
        xy = bool(getattr(m, "xy_valid", False))
        zz = bool(getattr(m, "z_valid", False))
        vxy = bool(getattr(m, "v_xy_valid", False))
        vz = bool(getattr(m, "v_z_valid", False))
        if not (xy and zz and vxy and vz):
            return
        self._vpos = (float(m.x), float(m.y), float(m.z),
                      float(m.vx), float(m.vy), float(m.vz), time.time())

    def _fuse_pose(self, gz_pos, gz_q):
        """Control pose in ENU (pose_frame.PoseFuser, shared with the mapper)."""
        f = self._fuser
        pos, q = f.fuse(gz_pos, gz_q, self._vpos, self._odom_q)
        self.pose_source, self._vel_meas, self._gz_miss = f.source, f.vel, f.gz_miss
        if f.rejected and (f.rejects == 1 or f.rejects % 40 == 0):
            print(f"ODOM_CHECK reject dist={f.gz_miss:.2f} m, flying gazebo", flush=True)
        return pos, q

    # ---- hooks for subclasses -------------------------------------------
    def _events(self, tm: float):
        """Called every thrust tick with the sim seconds since thrust started."""

    def _finished(self, tm: float) -> bool:
        return tm >= self.duration_s

    # ---------------------------------------------------------------------
    def _helix(self, tm: float):
        """Desired position and velocity in the path frame."""
        wt = self.omega * tm
        r, w = self.radius, self.omega
        pos = (self.x_rate * tm, r * math.sin(wt), self.z0 + r * math.cos(wt))
        vel = (self.x_rate, r * w * math.cos(wt), -r * w * math.sin(wt))
        acc = (0.0, -r * w * w * math.sin(wt), -r * w * w * math.cos(wt))
        return pos, vel, acc

    def _shell(self, *args):
        try:
            subprocess.call(
                [PX4_COMMANDER, "--instance", str(self.px4_instance), *args],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10,
            )
        except Exception:
            pass

    def _path_xy(self, x, y):
        x0, y0, yaw0 = self.anchor
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        return c * dx + s * dy, -s * dx + c * dy

    def _fail(self, why: str):
        print(f"FAIL {why}", flush=True)
        self.failed = why
        self.done = True

    def _stop_vehicle(self):
        self._shell("disarm", "-f")
        self._shell("mode", "manual")

    # ---- baton ----------------------------------------------------------
    def _armed_offboard(self) -> bool:
        return bool(
            self.vmode and self.vmode.flag_armed and self.vmode.flag_control_offboard_enabled
        )

    def _write_baton(self, pid=None):
        rec = {"pid": pid or os.getpid(), "depth_i": self.depth_i, "kind": type(self).__name__}
        tmp = self._baton + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(rec, f)
            os.replace(tmp, self._baton)
        except OSError:
            pass
        self._baton_write_t = time.time()

    def _drop_baton(self):
        rec = read_json(self._baton)
        if isinstance(rec, dict) and rec.get("pid") == os.getpid():
            for path in (self._baton, self._baton_req):
                try:
                    os.remove(path)
                except OSError:
                    pass

    def _successor(self):
        """Pid of a live process asking for this process's baton, or None."""
        try:
            with open(self._baton_req) as f:
                pid = int(f.read().strip() or 0)
        except (OSError, ValueError):
            return None
        if pid == os.getpid() or not pid_alive(pid):
            return None
        return pid

    def _request_baton(self, owner: int):
        rec = read_json(self._baton)
        if isinstance(rec, dict):
            try:
                self.depth_i = clamp(float(rec.get("depth_i", 0.0)), I_DEPTH_MAX)
            except (TypeError, ValueError):
                pass
        try:
            with open(self._baton_req, "w") as f:
                f.write(str(os.getpid()))
        except OSError:
            return False
        self._baton_t = time.time()
        print(f"BATON_REQUEST from pid {owner}", flush=True)
        return True

    def _start_thrust(self, t, x, y, yaw, sim_t, how):
        self.phase = "thrust"
        self.armed_ok = True
        self.t_thrust0 = sim_t
        self.anchor = (x, y, yaw)
        self.path_t = 0.0
        if self.offers_baton:
            self._write_baton()
        print(
            f"THRUST_ON t={t:.1f} {how} anchor=({x:.2f},{y:.2f}) yaw={yaw:.2f} "
            f"helix R={self.radius} w={self.omega}", flush=True,
        )

    def _can_hand_off(self) -> bool:
        return (
            self.keep_armed
            and self.latch.state != "attached"
            and self._armed_offboard()
            and os.path.isfile(HOLD_RUNNER)
        )

    def _begin_handoff(self, pos, yaw):
        """Keep flying where the behaviour ended and start the hold runner."""
        self.phase = "handoff"
        self.done = False
        self._hold_point = (float(pos[0]), float(pos[1]), float(pos[2]))
        self._handoff_t = time.time()
        self.yaw_hold = None
        self.progress_gate = False
        self.pitch_limit = PITCH_MAX_RAD
        self._head = yaw
        self._write_baton()
        env = {
            **os.environ,
            "UDRONE_GZ_MODEL": self.gz_model,
            "UDRONE_PX4_INSTANCE": str(self.px4_instance),
            "UDRONE_HOLD_X": f"{pos[0]:.3f}",
            "UDRONE_HOLD_Y": f"{pos[1]:.3f}",
            "UDRONE_HOLD_Z": f"{pos[2]:.3f}",
            "UDRONE_HOLD_YAW": f"{yaw:.4f}",
        }
        try:
            log = open("/tmp/coral_hold.log", "wb", buffering=0)
            subprocess.Popen(
                [sys.executable, "-u", HOLD_RUNNER],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
        except OSError as ex:
            print(f"HANDOFF could not start the hold runner ({ex}); disarming", flush=True)
            self._handoff_t = time.time() - HANDOFF_WAIT_S
        print(
            f"HANDOFF holding ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) armed "
            f"until the hold runner takes the vehicle",
            flush=True,
        )

    def _hold_ref(self):
        x0, y0, yaw0 = self.anchor
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = self._hold_point[0] - x0, self._hold_point[1] - y0
        return (c * dx + s * dy, -s * dx + c * dy, self._hold_point[2]), (0.0, 0.0, 0.0)

    def _tick(self):
        if self.done:
            return
        if self._stop_requested:
            print("STOP disarming and leaving offboard", flush=True)
            self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
            self._stop_vehicle()
            self._drop_baton()
            self.failed = self.failed or "stopped"
            self.done = True
            return
        # Another process is flying this vehicle: stay silent until it lets go.
        owner = None
        if self.phase in ("wait_pose", "stream", "baton"):
            owner = baton_owner(self.gz_model, os.getpid())
            if owner is None and self.baton_only and self.phase != "baton":
                self._fail("no armed vehicle to take over")
                return
        if owner is None:
            # timestamp 0: PX4 stamps the message on arrival, so this does not
            # depend on clock sync between ROS (wall time) and PX4 (sim time).
            off = OffboardControlMode()
            off.timestamp = 0
            off.attitude = True
            self.pub_off.publish(off)

        fresh, sim_t, (auv, boat) = self.gz.read(self.gz_model, self.boat_model)
        ok = fresh and auv is not None
        if ok and sim_t is not None:
            self._sim_anchor = (sim_t, time.time())
        elif self._sim_anchor is not None:
            # The Gazebo clock froze with the stream. Keep the path moving.
            sim_t = self._sim_anchor[0] + (time.time() - self._sim_anchor[1])

        if self.phase == "wait_pose":
            if ok:
                self.t0 = sim_t
                self.last_ok_wall = time.time()
                self.phase = "stream"
                print(f"POSE_OK {self.gz_model} xyz=({auv[0][0]:.2f},{auv[0][1]:.2f},{auv[0][2]:.2f})", flush=True)
            elif time.time() - self.t_wall0 > 30.0:
                self._fail(f"no Gazebo pose for {self.gz_model} in 30 s (is the simulator running?)")
            if owner is None:
                self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
            return

        if ok:
            (gz_x, gz_y, gz_z), gz_q = auv
            pos, q = self._fuse_pose((gz_x, gz_y, gz_z), gz_q)
        else:
            # Gazebo dropped out. Keep flying the estimator if it is still fresh.
            pos, q = self._fuse_pose(None, self._odom_q)
        if pos is None or q is None or sim_t is None:
            if owner is None:
                self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
            if time.time() - self.last_ok_wall > 10.0:
                if owner is None:
                    self._stop_vehicle()
                self._drop_baton()
                self._fail(f"no pose for {self.gz_model} for 10 s")
            return
        self.last_ok_wall = time.time()
        auv = (pos, q)
        x, y, z = pos
        yaw = rpy_from_q(q)[2]
        t = sim_t - self.t0
        thrust = roll_u = pitch_u = yaw_u = 0.0

        if self.phase == "stream":
            if owner is not None:
                # Already armed under another process: take it over in place.
                if self._armed_offboard() and self._request_baton(owner):
                    self.phase = "baton"
                elif t > 15.0:
                    self._fail(f"pid {owner} holds {self.gz_model} and did not report armed")
                return
            if t > 2.0:
                self._shell("mode", "offboard")
                self._shell("arm", "-f")
                print("SHELL_ARM sent", flush=True)
                self.next_arm = t + 1.0
                self.phase = "arming"
        elif self.phase == "baton":
            if owner is None:
                self._start_thrust(t, x, y, yaw, sim_t, "BATON_TAKEN")
            elif time.time() - self._baton_t > BATON_WAIT_S:
                print(f"BATON_TIMEOUT pid {owner} kept the vehicle; stopping it and arming", flush=True)
                try:
                    os.kill(owner, signal.SIGTERM)
                except OSError:
                    pass
                try:
                    os.remove(self._baton_req)
                except OSError:
                    pass
                self.t0 = sim_t
                self.phase = "stream"
                return
            else:
                return
        elif self.phase == "arming":
            ob = bool(self.vmode and self.vmode.flag_control_offboard_enabled)
            ar = bool(self.vmode and self.vmode.flag_armed)
            if ob and ar:
                self._start_thrust(t, x, y, yaw, sim_t, "ARMED")
            elif t > 16.0:
                self._stop_vehicle()
                self._fail(f"could not arm in offboard (offboard={ob} armed={ar})")
            elif t >= self.next_arm:
                print(f"ARM_CHECK t={t:.1f} offboard={ob} armed={ar}; retrying", flush=True)
                self.next_arm = t + 1.0
                if not ob:
                    self._shell("mode", "offboard")
                if not ar:
                    self._shell("arm", "-f")

        if self.phase in ("thrust", "handoff"):
            tm = sim_t - self.t_thrust0
            if not bool(self.vmode and self.vmode.flag_armed):
                self._drop_baton()
                self._fail(f"vehicle disarmed at t={tm:.1f} s of thrust")
                return
            if self.phase == "handoff" or self.offers_baton:
                nxt = self._successor()
                if nxt is not None:
                    # Stop publishing; the successor starts on its next tick.
                    # The file names it at once, so a third process stays quiet.
                    self._write_baton(nxt)
                    try:
                        os.remove(self._baton_req)
                    except OSError:
                        pass
                    print(f"BATON_RELEASED to pid {nxt} at t={tm:.1f}s, still armed", flush=True)
                    self.done = True
                    return
                if time.time() - self._baton_write_t > 1.0:
                    self._write_baton()

        if self.phase == "handoff":
            if time.time() - self._handoff_t > HANDOFF_WAIT_S:
                print("HANDOFF the hold runner did not take the vehicle; disarming", flush=True)
                self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
                self._stop_vehicle()
                self._drop_baton()
                self.done = True
                return
            thrust, roll_u, pitch_u, yaw_u = UdroneHelixController._control(self, tm, auv, boat)
        elif self.phase == "thrust":
            self._events(tm)
            thrust, roll_u, pitch_u, yaw_u = self._control(tm, auv, boat)

            if self._finished(tm):
                print(f"DONE thrust_elapsed={tm:.1f}s (sim)", flush=True)
                if self._can_hand_off():
                    self._begin_handoff(pos, yaw)
                else:
                    self._publish_setpoints(0.0, 0.0, 0.0, 0.0)
                    self._stop_vehicle()
                    self._drop_baton()
                    self.done = True
                    return

        self._publish_setpoints(thrust, roll_u, pitch_u, yaw_u)
        if self.hud is not None and self.phase in ("thrust", "handoff"):
            self.hud.render(pos, q, boat, (thrust, roll_u, pitch_u, yaw_u), tm)

    def _hud_layers(self):
        """What the HUD draws for this task. Subclasses extend the dict."""
        out = {"paths": [], "markers": [], "lines": []}
        if self._hold_point is not None:
            out["markers"].append({"pos": self._hold_point, "label": "hold", "color": (0, 255, 255)})
        elif self.hud_preview and self.anchor is not None:
            x0, y0, yaw0 = self.anchor
            c, s = math.cos(yaw0), math.sin(yaw0)
            t0 = self.path_t
            pts = []
            for k in range(25):
                (xd, yd, zd) = self._helix(t0 + 0.5 * k)[0]
                pts.append((x0 + c * xd - s * yd, y0 + s * xd + c * yd, zd))
            out["paths"].append({"pts": pts, "color": (255, 0, 255), "width": 2, "label": "+12s"})
        return out

    def _control(self, tm, auv, boat):
        """One control step: (thrust, roll_u, pitch_u, yaw_u), all normalized."""
        pos, q = auv
        x, y, z = pos
        fwd, left, up = body_axes(q)
        sim_t = self.t_thrust0 + tm

        # Body rates from the attitude. Velocity is the estimator's when the
        # sim check accepted it, otherwise a difference of the pose.
        dt = 0.05
        if self._prev is None:
            self._prev = (sim_t, pos, q)
        elif sim_t > self._prev[0] + 1e-3:
            dt = sim_t - self._prev[0]
            v_raw = tuple((a - b) / dt for a, b in zip(pos, self._prev[1]))
            w_raw = body_rates(self._prev[2], q, dt)
            if self._vel_meas is None:
                self.vel = tuple(0.5 * a + 0.5 * b for a, b in zip(self.vel, v_raw))
            self.w_body = tuple(0.5 * a + 0.5 * b for a, b in zip(self.w_body, w_raw))
            self._prev = (sim_t, pos, q)
        if self._vel_meas is not None:
            self.vel = self._vel_meas

        # Desired point on the path, path frame -> world. A third tuple is the
        # path acceleration. Helix, swim, and coverage omit it. With the
        # progress gate the reference clock slows while the hull is behind, so
        # the lag cannot grow past GATE_E1.
        rate = 1.0
        if self._hold_point is not None:
            ref = self._hold_ref()
        elif self.progress_gate:
            e = self._err_n
            if e > GATE_E0:
                rate = max(0.0, 1.0 - (e - GATE_E0) / (GATE_E1 - GATE_E0))
            ref = self._helix(self.path_t)
        else:
            ref = self._helix(tm)
        self._gate_rate = rate
        if len(ref) == 3:
            (x_des, y_des, z_des), (vx_p, vy_p, vz_p), (ax_p, ay_p, az_p) = ref
        else:
            (x_des, y_des, z_des), (vx_p, vy_p, vz_p) = ref
            ax_p = ay_p = az_p = 0.0
        vx_p, vy_p, vz_p = vx_p * rate, vy_p * rate, vz_p * rate
        ax_p, ay_p, az_p = ax_p * rate * rate, ay_p * rate * rate, az_p * rate * rate
        x0, y0, yaw0 = self.anchor
        c, s = math.cos(yaw0), math.sin(yaw0)
        p_des = (x0 + c * x_des - s * y_des, y0 + s * x_des + c * y_des, z_des)
        v_des = (c * vx_p - s * vy_p, s * vx_p + c * vy_p, vz_p)
        a_w = (c * ax_p - s * ay_p, s * ax_p + c * ay_p, az_p)
        self._p_des = p_des
        err = tuple(a - b for a, b in zip(p_des, pos))
        self._err_n = norm(err)
        if self._hold_point is None:
            self.path_t = self.path_t + rate * dt if self.progress_gate else tm
        gs = _mp.gain_scale(self.power)
        corr = tuple(KP_POS * gs * e for e in err)
        n = norm(corr)
        v_corr_max = V_CORR_MAX * max(self.power, _mp.GAIN_FLOOR)
        if n > v_corr_max:
            corr = tuple(v * v_corr_max / n for v in corr)
        v_cmd = tuple(a + b for a, b in zip(v_des, corr))

        docked = self.latch.state == "attached"
        if not docked and abs(z - z_des) < I_DEPTH_BAND_M:
            # Too shallow -> push down harder. Not while latched: the boat holds
            # the depth. Not during a dive or climb, which would wind it up.
            self.depth_i = clamp(self.depth_i + KI_DEPTH * (z - z_des) * dt, I_DEPTH_MAX)

        # Force wanted on the hull. The mass times path acceleration is the
        # geometric feedforward (the curve's own acceleration, as in a
        # differentially flat tracker). The velocity term closes the error.
        # Drag is cancelled at the commanded velocity, then buoyancy.
        # power scales feedforward and thrust so low % is slower and smoother.
        vb = (dot(v_cmd, fwd), dot(v_cmd, left), dot(v_cmd, up))
        drag_b = tuple(DRAG_LIN[i] * vb[i] + DRAG_QUAD[i] * vb[i] * abs(vb[i]) for i in range(3))
        drag_w = tuple(fwd[i] * drag_b[0] + left[i] * drag_b[1] + up[i] * drag_b[2] for i in range(3))
        force = [
            MASS_KG * (KV * gs * (v_cmd[i] - self.vel[i]) + self.power * a_w[i]) + drag_w[i]
            for i in range(3)
        ]
        force[2] -= NET_BUOYANCY_N + self.depth_i
        fn = norm(force)
        # Throttle ceiling is the maneuver power (1.0 = full SURGE_N_PER_UNIT).
        thrust = clamp(dot(force, fwd) / SURGE_N_PER_UNIT, self.power)
        roll_u = pitch_u = yaw_u = 0.0
        if not docked:
            self._dir = self._hull_dir(force, fwd, fn)
            # Turn the hull axis onto the force direction. The error is the
            # rotation vector from one to the other, resolved on the body axes.
            rot = cross(fwd, self._dir)
            if dot(fwd, self._dir) < 0.0:       # more than 90 deg off: full error
                rn = norm(rot)
                rot = tuple(r / rn for r in rot) if rn > 1e-6 else left
            tq = TORQUE_MAX * gs
            pitch_u = clamp(
                KP_PITCH * gs * dot(rot, left) - KD_PITCH * self.w_body[1] - GRAVITY_FF * fwd[2], tq)
            yaw_u = clamp(KP_YAW * gs * dot(rot, up) - KD_YAW * self.w_body[2], tq)

        # Log in the path frame.
        roll, pitch, yaw = rpy_from_q(q)
        xp, yp = self._path_xy(x, y)
        yaw_p = wrap_pi(yaw - yaw0)
        e3 = norm(err)
        nan = float("nan")
        bx = by = bz = rx = ry = rz = nan
        if boat is not None:
            (wbx, wby, bz), bq = boat
            bx, by = self._path_xy(wbx, wby)
            rx, ry, rz = rotate_inverse(bq, (x - wbx, y - wby, z - bz))
        if self._hold_point is None:
            self.log.append((
                tm, xp, yp, z, x_des, y_des, z_des, thrust, yaw_u, pitch_u, yaw_p, roll,
                bx, by, bz, rx, ry, rz, self.probe.newtons(), self.latch.code(),
            ))
        if tm >= self.next_print:
            self.next_print = tm + 2.0
            if self.progress_gate:
                print(f"t={tm:5.1f} PROGRESS path_t={self.path_t:.1f} rate={rate:.2f}", flush=True)
            print(
                f"t={tm:5.1f} pos=({xp:.2f},{yp:.2f},{z:.2f}) des=({x_des:.2f},{y_des:.2f},{z_des:.2f}) "
                f"err={e3:.2f} m nose-down={math.degrees(math.asin(clamp(-fwd[2], 1.0))):.0f} deg "
                f"force={fn:.1f} N thrust={thrust:+.3f} rotor0={self.probe.newtons():.1f} N "
                f"latch={self.latch.state} pose={self.pose_source} gz_miss={self._gz_miss:.2f}",
                flush=True,
            )
        return thrust, roll_u, pitch_u, yaw_u

    def _hull_dir(self, force, fwd, fn):
        """Unit vector the hull axis should point along for this force.

        Without a pitch limit or a heading to hold, that is the force itself
        (shapes and the helix). Otherwise the heading comes from the
        horizontal force, is held when that force is small, and reverses
        (backing up) when a modest force lies behind the nose. The elevation
        is the force's, limited to the cruise pitch.
        """
        limit = self.pitch_limit
        if limit is None and self.yaw_hold is None:
            if fn > 0.3 or self._dir is None:
                return tuple(f / fn for f in force) if fn > 1e-6 else fwd
            return self._dir
        if fn < 0.3 and self._dir is not None:
            return self._dir
        if self.yaw_hold is not None:
            head = self.yaw_hold
        else:
            head = self._head if self._head is not None else math.atan2(fwd[1], fwd[0])
            fh = math.hypot(force[0], force[1])
            if fh >= HEAD_MIN_N:
                want = math.atan2(force[1], force[0])
                if abs(wrap_pi(want - head)) <= HEAD_BEHIND_RAD or fh >= HEAD_TURN_N:
                    head = want
                else:
                    head = wrap_pi(want + math.pi)
        self._head = head
        hx, hy = math.cos(head), math.sin(head)
        along = force[0] * hx + force[1] * hy
        # Forward thrust nose-down and reverse thrust nose-up both make a
        # downward force. Near vertical, keep the one already in use rather
        # than pitching across as the small forward part changes sign.
        sign = 1.0 if along >= 0.0 else -1.0
        if self._elev_sign is not None and sign != self._elev_sign \
                and abs(along) < ELEV_KEEP * abs(force[2]):
            sign = self._elev_sign
        self._elev_sign = sign
        elev = math.atan2(sign * force[2], max(abs(along), 1e-3))
        if limit is not None:
            elev = max(-limit, min(limit, elev))
        ce = math.cos(elev)
        return (hx * ce, hy * ce, math.sin(elev))

    def _publish_setpoints(self, thrust, roll_u, pitch_u, yaw_u):
        r = VehicleRatesSetpoint()
        r.timestamp = 0
        r.roll = float(roll_u)
        r.pitch = float(pitch_u)
        r.yaw = float(yaw_u)
        r.thrust_body = [float(thrust), 0.0, 0.0]
        self.pub_rates.publish(r)

        asp = VehicleAttitudeSetpoint()
        asp.timestamp = 0
        if self.att is not None and len(self.att.q) >= 4:
            asp.q_d = [float(self.att.q[i]) for i in range(4)]
        elif self.odom is not None and len(self.odom.q) >= 4:
            asp.q_d = [float(self.odom.q[i]) for i in range(4)]
        else:
            asp.q_d = [1.0, 0.0, 0.0, 0.0]
        asp.thrust_body = [float(thrust), 0.0, 0.0]
        self.pub_att.publish(asp)

    def write_log(self, path: str):
        with open(path, "w") as f:
            f.write(LOG_COLUMNS + "\n")
            for row in self.log:
                f.write(" ".join(f"{v:.6f}" for v in row) + "\n")
        print(f"wrote {path} n={len(self.log)}", flush=True)

    def pose_rate_hz(self):
        """Pose messages per wall second since start."""
        return self.gz.n_msgs / max(1e-3, time.time() - self.t_wall0)

    def destroy_node(self):
        self.gz.stop()
        self.latch.stop()
        self.probe.stop()
        super().destroy_node()


def spin_until_done(node):
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("interrupted; disarming", flush=True)
        node._stop_vehicle()
        node.failed = node.failed or "interrupted"


def tracking_error(rows, t_from=0.0, t_to=float("inf")):
    """(mean, max) 3D distance from the helix over log rows with t_from <= t < t_to."""
    d = [math.dist(r[1:4], r[4:7]) for r in rows if t_from <= r[0] < t_to]
    return (sum(d) / len(d), max(d)) if d else (float("nan"), float("nan"))


SETTLE_S = 10.0          # time allowed to dive from the surface onto the helix
TRACK_MEAN_M = 0.25      # mean 3D error allowed after that


def main():
    duration = float(os.environ.get("UDRONE_SINE_DUR", os.environ.get("UDRONE_HELIX_DUR", "45")))
    radius = float(os.environ.get("UDRONE_HELIX_R", os.environ.get("UDRONE_SINE_AMP", "0.5")))
    omega = float(os.environ.get("UDRONE_HELIX_OMEGA", os.environ.get("UDRONE_SINE_OMEGA", "0.40")))
    x_rate = float(os.environ.get("UDRONE_HELIX_VX", "0.20"))
    z0 = float(os.environ.get("UDRONE_HELIX_Z0", "-1.2"))
    rclpy.init()
    node = UdroneHelixController(
        duration_s=duration, radius=radius, omega=omega, x_rate=x_rate, z0=z0,
    )
    node.pitch_limit = None
    ok = False
    try:
        spin_until_done(node)
    finally:
        node.write_log("/tmp/udrone_helix_ctrl_log.txt")
        why = node.failed or "no control samples"
        if node.log:
            cols = list(zip(*node.log))
            xs, ys, zs, yds, zds, rotor = cols[1], cols[2], cols[3], cols[5], cols[6], cols[18]
            mean_rotor = sum(abs(v) for v in rotor) / len(rotor)
            e_mean, e_max = tracking_error(node.log, SETTLE_S)
            print(
                f"x advance {xs[-1] - xs[0]:.2f} m | y span {max(ys) - min(ys):.2f} (helix {max(yds) - min(yds):.2f}) | "
                f"z [{min(zs):.2f},{max(zs):.2f}] (helix [{min(zds):.2f},{max(zds):.2f}]) | "
                f"3D error after {SETTLE_S:.0f} s: mean {e_mean:.2f} m, max {e_max:.2f} m | "
                f"rotor0 mean |{mean_rotor:.1f}| N | pose feed {node.pose_rate_hz():.0f} Hz",
                flush=True,
            )
            if node.latch.state == "attached":
                print("NOTE the dock latch reports attached: the AUV was pushing the boat, not flying the helix", flush=True)
            if not node.armed_ok or node.failed:
                why = node.failed or "never armed"
            elif mean_rotor < 0.2:
                why = "thrusters were not driven"
            elif not e_mean <= TRACK_MEAN_M:
                why = f"helix not tracked (mean error {e_mean:.2f} m, limit {TRACK_MEAN_M} m)"
            else:
                ok = True
        print("VERDICT " + ("PASS" if ok else f"FAIL {why}"), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
