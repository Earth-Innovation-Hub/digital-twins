#!/usr/bin/env python3
"""Aquatic ROS2 port of udrone_boat sinusoidal attitude controller.

Uses Gazebo pose as state (PX4 local odom on this twin is unreliable) and
px4_msgs offboard + UUV_SKIP_CTRL rates/thrust feedthrough.
Trajectory: advance in x while tracking y=A*sin(wt), hold depth z_des (UUV weave).
"""
from __future__ import annotations

import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from PID import PID  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleRatesSetpoint,
    VehicleAttitudeSetpoint,
    VehicleControlMode,
    VehicleOdometry,
    ActuatorMotors,
)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_q(q) -> float:
    w, x, y, z = [float(v) for v in q[:4]]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_from_euler_line(line: str):
    # gz model -p second [...] is often rpy-like; fallback None
    nums = line.strip().strip("[]").split()
    if len(nums) >= 3:
        try:
            return float(nums[2])  # yaw if rpy
        except ValueError:
            return None
    return None


class GzPose:
    def __init__(self, model="xudrone_0"):
        self.model = model
        self.lock = threading.Lock()
        self.xyz = (0.0, 0.0, -1.0)
        self.yaw = 0.0
        self.ok = False
        self._stop = False
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def _loop(self):
        while not self._stop:
            try:
                out = subprocess.check_output(
                    ["gz", "model", "-m", self.model, "-p"],
                    text=True,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                )
                lines = [l.strip() for l in out.splitlines() if l.strip().startswith("[")]
                if lines:
                    nums = lines[0].strip("[]").split()
                    x, y, z = float(nums[0]), float(nums[1]), float(nums[2])
                    yaw = 0.0
                    if len(lines) > 1:
                        # orientation line may be euler; treat third as yaw when plausible
                        n2 = lines[1].strip("[]").split()
                        if len(n2) >= 3:
                            yaw = float(n2[2])
                    with self.lock:
                        self.xyz = (x, y, z)
                        self.yaw = yaw
                        self.ok = True
            except Exception:
                pass
            time.sleep(0.05)

    def get(self):
        with self.lock:
            return self.ok, self.xyz, self.yaw

    def stop(self):
        self._stop = True


class UdroneSinusoidalController(Node):
    def __init__(self, duration_s=30.0, amplitude=1.5, omega=0.30):
        super().__init__("udrone_sinusoidal_controller")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ns = "/udrone/fmu"
        self.pub_off = self.create_publisher(OffboardControlMode, f"{ns}/in/offboard_control_mode", qos)
        self.pub_rates = self.create_publisher(VehicleRatesSetpoint, f"{ns}/in/vehicle_rates_setpoint", qos)
        self.pub_att = self.create_publisher(VehicleAttitudeSetpoint, f"{ns}/in/vehicle_attitude_setpoint", qos)

        self.vmode = None
        self.odom = None
        self.mots = deque(maxlen=20)
        self.create_subscription(VehicleControlMode, f"{ns}/out/vehicle_control_mode", lambda m: setattr(self, "vmode", m), qos)
        self.create_subscription(VehicleOdometry, f"{ns}/out/vehicle_odometry", lambda m: setattr(self, "odom", m), qos)
        self.create_subscription(
            ActuatorMotors, f"{ns}/out/actuator_motors",
            lambda m: self.mots.append([float(v) for v in m.control[:4]]), qos,
        )

        self.xPID = PID(kp=0.12, kd=0.04, ki=0.01)
        self.yPID = PID(kp=0.10, kd=0.03, ki=0.01)
        self.zPID = PID(kp=0.08, kd=0.03, ki=0.01)

        self.amplitude = float(amplitude)
        self.omega = float(omega)
        self.z_des = -1.0
        self.x_des = 0.0
        self.x_des_rate = 0.12  # m/s slow advance for weave
        self.duration_s = float(duration_s)
        self.gz = GzPose("xudrone_0")

        self.t0 = time.time()
        self.phase = "stream"
        self.did_shell_arm = False
        self.done = False
        self.log = []
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"sinusoidal(GZ): A={self.amplitude} w={self.omega} z_des={self.z_des} dur={self.duration_s}s"
        )

    def _us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _shell(self, *args):
        b = "/root/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"
        try:
            subprocess.call([b, *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _tick(self):
        t = time.time() - self.t0
        now = self._us()

        off = OffboardControlMode(); off.timestamp = now; off.attitude = True
        self.pub_off.publish(off)

        ok, (x, y, z), yaw = self.gz.get()
        # Body rates from odom if available; yaw MUST come from Gazebo (EKF yaw is wrong here)
        wr = wp = wy = 0.0
        if self.odom is not None and len(self.odom.angular_velocity) >= 3:
            wr, wp, wy = [float(v) for v in self.odom.angular_velocity[:3]]

        y_des = self.amplitude * math.sin(self.omega * t)
        x_des = self.x_des_rate * max(0.0, t - 2.0)  # weave forward after arm
        thrust = roll_u = pitch_u = yaw_u = 0.0

        if self.phase == "thrust" and ok:
            x_u = self.xPID.run(x, x_des)
            y_u = self.yPID.run(y, y_des)
            z_u = self.zPID.run(z, self.z_des)

            pitch_u = max(-0.45, min(0.45, -z_u - 0.3 * wp))
            roll_u = max(-0.35, min(0.35, -0.6 * wr))

            bearing = math.atan2(y_des - y, x_des - x)
            yaw_err = wrap_pi(bearing - yaw)
            yaw_u = max(-0.55, min(0.55, 0.4 * y_u + 1.1 * yaw_err - 0.6 * wy))

            dist_xy = math.hypot(x_des - x, y_des - y)
            # Hold surge low until roughly pointed at the sine target
            align = max(0.0, 1.0 - abs(yaw_err) / 1.2)
            thrust = max(0.0, min(0.45, (0.08 + 0.28 * min(dist_xy, 2.0)) * (0.25 + 0.75 * align)))

            self.log.append((t, x, y, z, y_des, x_des, thrust, yaw_u, pitch_u, yaw))
            if int(t * 10) % 20 == 0:
                print(
                    f"t={t:.1f} gz=({x:.2f},{y:.2f},{z:.2f}) y_des={y_des:.2f} "
                    f"thr={thrust:.2f} yaw_u={yaw_u:.2f} pitch_u={pitch_u:.2f} "
                    f"mot={self.mots[-1] if self.mots else None}",
                    flush=True,
                )

        r = VehicleRatesSetpoint()
        r.timestamp = now
        r.roll = float(roll_u); r.pitch = float(pitch_u); r.yaw = float(yaw_u)
        r.thrust_body = [float(thrust), 0.0, 0.0]
        self.pub_rates.publish(r)

        asp = VehicleAttitudeSetpoint(); asp.timestamp = now
        if self.odom is not None and len(self.odom.q) >= 4:
            asp.q_d = [float(self.odom.q[i]) for i in range(4)]
        else:
            asp.q_d = [1.0, 0.0, 0.0, 0.0]
        asp.thrust_body = [float(thrust), 0.0, 0.0]
        self.pub_att.publish(asp)

        if self.phase == "stream" and t > 2.0 and not self.did_shell_arm:
            self.did_shell_arm = True
            self._shell("mode", "offboard"); self._shell("arm", "-f")
            print("SHELL_ARM sent", flush=True)
            self.phase = "arming"
        elif self.phase == "arming":
            ob = bool(self.vmode and self.vmode.flag_control_offboard_enabled)
            ar = bool(self.vmode and self.vmode.flag_armed)
            if int(t * 10) % 10 == 0:
                print(f"ARM_CHECK t={t:.1f} offboard={ob} armed={ar}", flush=True)
            if not ar and t > 3.0:
                self._shell("arm", "-f")
            if ob and ar:
                self.phase = "thrust"
                print(f"THRUST_ON t={t:.1f}", flush=True)
            elif t > 12.0:
                print("FAIL arm", flush=True); self.done = True
        elif self.phase == "thrust" and t > self.duration_s:
            print("DONE", flush=True)
            self._shell("disarm", "-f")
            self.done = True

    def write_log(self, path: str):
        with open(path, "w") as f:
            f.write("t x y z y_des x_des thrust yaw_u pitch_u yaw\n")
            for row in self.log:
                f.write(" ".join(f"{v:.6f}" for v in row) + "\n")
        print(f"wrote {path} n={len(self.log)}", flush=True)

    def destroy_node(self):
        self.gz.stop()
        super().destroy_node()


def main():
    duration = float(os.environ.get("UDRONE_SINE_DUR", "30"))
    amp = float(os.environ.get("UDRONE_SINE_AMP", "1.5"))
    omega = float(os.environ.get("UDRONE_SINE_OMEGA", "0.30"))
    rclpy.init()
    node = UdroneSinusoidalController(duration_s=duration, amplitude=amp, omega=omega)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.write_log("/tmp/udrone_sine_ctrl_log.txt")
        # metrics
        if node.log:
            ys = [r[2] for r in node.log]
            yds = [r[4] for r in node.log]
            errs = [abs(y - yd) for y, yd in zip(ys, yds)]
            print(
                f"GZ y range [{min(ys):.3f},{max(ys):.3f}] span {max(ys)-min(ys):.3f} "
                f"MAE {sum(errs)/len(errs):.3f} max_err {max(errs):.3f}",
                flush=True,
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
