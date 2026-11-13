#!/usr/bin/env python3
"""CoRAL carry test: otter_k drives a short leg with xudrone latched underneath.

Uses the boat exactly as the production otter is used: arm, then
MAV_CMD_DO_REPOSITION (Auto Loiter go-to) re-sent until arrival, over ROS 2 on
/otter_k/fmu/... . Ground truth comes from Gazebo.

Passes only if:
  - the boat armed and Gazebo jets received non-zero thrust
  - the boat arrived within BOAT_ARRIVE_M of the target
  - the latch stayed attached and the AUV did not move in the boat's frame

Env: BOAT_LEG_M (8) leg length along the current heading, BOAT_ARRIVE_M (2.0),
     BOAT_TIMEOUT_S (120, simulation seconds).
Run: run_test.sh transit      Exit status: 0 pass, 1 fail.
"""
from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition, VehicleStatus

from udrone_sinusoidal_controller import GzPoses, GzStream, LatchState, rotate_inverse, rpy_from_q

NS = "/otter_k/fmu"
FORCE = 21196.0


class JetProbe:
    """Newtons commanded on one Heron jet in Gazebo."""

    def __init__(self, joint="port_jet_joint"):
        self.value = 0.0
        self.peak = 0.0
        self.stream = GzStream(f"/model/heron_0/joint/{joint}/cmd_thrust", self._line)

    def _line(self, line):
        s = line.strip()
        if s.startswith("data:"):
            try:
                self.value = float(s.partition(":")[2])
                self.peak = max(self.peak, abs(self.value))
            except ValueError:
                pass

    def stop(self):
        self.stream.stop()


class BoatTransit(Node):
    def __init__(self, leg_m, arrive_m, timeout_s):
        super().__init__("otter_k_transit_test")
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.cmd_pub = self.create_publisher(VehicleCommand, f"{NS}/in/vehicle_command", pub_qos)
        self.status = None
        self.gpos = None
        self.status_sub = None       # created once discovery shows which status topic PX4 publishes
        self.create_subscription(VehicleGlobalPosition, f"{NS}/out/vehicle_global_position", lambda m: setattr(self, "gpos", m), qos_profile_sensor_data)

        self.leg_m, self.arrive_m, self.timeout_s = leg_m, arrive_m, timeout_s
        self.gz = GzPoses()
        self.latch = LatchState()
        self.jet = JetProbe()
        self.phase = "wait"
        self.t_wall0 = time.time()
        self.t0 = None
        self.start = None            # boat world xy at the start
        self.target = None           # world xy
        self.target_ll = None
        self.rel0 = None
        self.max_rel_dev = 0.0
        self.latch_lost = False
        self.forced_arm = False
        self.arm_tries = 0
        self.next_send = 0.0
        self.closest = float("inf")
        self.arrived_t = None
        self.done = False
        self.failed = ""
        self.create_timer(0.1, self._tick)

    def _command(self, command, **params):
        m = VehicleCommand()
        m.timestamp = 0              # PX4 stamps it on arrival
        m.command = command
        for k, v in params.items():
            setattr(m, k, float(v))
        m.target_system = int(self.status.system_id) if self.status else 1
        m.target_component = 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def _armed(self):
        return bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _fail(self, why):
        print(f"FAIL {why}", flush=True)
        self.failed = why
        self.done = True

    def _find_status_topic(self):
        # PX4 names the topic vehicle_status or vehicle_status_v<N> depending
        # on the message version; subscribe to the one that exists.
        names = [n for n, _ in self.get_topic_names_and_types()
                 if n.startswith(f"{NS}/out/vehicle_status") and self.count_publishers(n) > 0]
        if not names:
            return
        topic = os.environ.get("BOAT_STATUS_TOPIC") or sorted(names)[-1]
        self.status_sub = self.create_subscription(
            VehicleStatus, topic, lambda m: setattr(self, "status", m), qos_profile_sensor_data)
        print(f"STATUS_TOPIC {topic}", flush=True)

    def _tick(self):
        if self.done:
            return
        if self.status_sub is None:
            self._find_status_topic()
        fresh, sim_t, (boat, auv) = self.gz.read("heron_0", "xudrone_1")
        # The latch state arrives once a second; wait for the first one.
        ready = self.phase != "wait" or self.latch.state is not None
        if not (fresh and boat is not None and auv is not None and self.status and self.gpos and ready):
            if time.time() - self.t_wall0 > 40.0 and self.phase == "wait":
                missing = [n for n, v in (("boat pose", boat), ("AUV pose", auv),
                                          ("/otter_k vehicle_status", self.status),
                                          ("/otter_k global position", self.gpos),
                                          ("latch state", self.latch.state)) if v is None]
                self._fail("no data after 40 s: " + ", ".join(missing))
            return
        (bx, by, bz), bq = boat
        (ax, ay, az), _aq = auv
        rel = rotate_inverse(bq, (ax - bx, ay - by, az - bz))

        if self.phase == "wait":
            yaw = rpy_from_q(bq)[2]
            self.t0 = sim_t
            self.start = (bx, by)
            self.target = (bx + self.leg_m * math.cos(yaw), by + self.leg_m * math.sin(yaw))
            d_e, d_n = self.target[0] - bx, self.target[1] - by      # world is ENU
            lat, lon = float(self.gpos.lat), float(self.gpos.lon)
            self.target_ll = (lat + d_n / 111320.0, lon + d_e / (111320.0 * math.cos(math.radians(lat))))
            self.rel0 = rel
            print(
                f"START boat=({bx:.2f},{by:.2f}) yaw={yaw:.2f} target=({self.target[0]:.2f},{self.target[1]:.2f}) "
                f"latch={self.latch.state} nest offset=({rel[0]:.3f},{rel[1]:.3f},{rel[2]:.3f})", flush=True,
            )
            if self.latch.state != "attached":
                self._fail(f"latch is {self.latch.state}, not attached; dock first (dock_command.py ATTACH or simctl restart)")
                return
            self.phase = "arm"

        t = sim_t - self.t0
        self.max_rel_dev = max(self.max_rel_dev, math.dist(rel, self.rel0))
        if self.latch.state != "attached":
            self.latch_lost = True

        if self.phase == "arm":
            if self._armed():
                print(f"ARMED t={t:.1f}" + (" (forced: preflight checks did not pass)" if self.forced_arm else ""), flush=True)
                self.phase = "go"
            elif t >= self.next_send:
                self.next_send = t + 1.5
                self.arm_tries += 1
                if self.arm_tries > 8:
                    self._fail("boat would not arm")
                    return
                # Two plain attempts, then force (the sandbox boat can fail preflight).
                self.forced_arm = self.arm_tries > 2
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0,
                              param2=FORCE if self.forced_arm else 0.0)
        elif self.phase == "go":
            d = math.dist((bx, by), self.target)
            self.closest = min(self.closest, d)
            if not self._armed():
                self._fail(f"boat disarmed {d:.1f} m from the target")
                return
            if d <= self.arrive_m:
                self.arrived_t = t
                print(f"ARRIVED t={t:.1f} {d:.2f} m from the target", flush=True)
                self.phase = "stop"
                self.next_send = 0.0
            elif t > self.timeout_s:
                self._fail(f"timeout: closest approach {self.closest:.1f} m")
                self.phase = "stop"
                self.done = False
                self.next_send = 0.0
            elif t >= self.next_send:
                self.next_send = t + 1.0
                self._command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1=-1.0, param2=1.0,
                              param4=math.nan, param5=self.target_ll[0], param6=self.target_ll[1], param7=math.nan)
                print(f"t={t:5.1f} boat=({bx:.2f},{by:.2f}) to go {d:.1f} m jet={self.jet.value:.1f} N "
                      f"nest shift {self.max_rel_dev * 100:.1f} cm", flush=True)
        if self.phase == "stop":
            if not self._armed():
                print("DISARMED", flush=True)
                self.done = True
            elif t >= self.next_send:
                self.next_send = t + 1.0
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=FORCE)

    def destroy_node(self):
        self.gz.stop()
        self.latch.stop()
        self.jet.stop()
        super().destroy_node()


def main():
    leg = float(os.environ.get("BOAT_LEG_M", "8"))
    arrive = float(os.environ.get("BOAT_ARRIVE_M", "2.0"))
    timeout = float(os.environ.get("BOAT_TIMEOUT_S", "120"))
    rclpy.init()
    node = BoatTransit(leg, arrive, timeout)
    ok = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.failed = "interrupted"
    finally:
        checks = [
            ("ROS 2 can reach /otter_k", node.status is not None and node.gpos is not None, "status and position received"),
            ("boat armed", node.phase in ("go", "stop"), "forced arm" if node.forced_arm else "normal arm"),
            ("jets driven", node.jet.peak > 1.0, f"peak |{node.jet.peak:.1f}| N on the port jet"),
            ("arrived", node.arrived_t is not None,
             f"in {node.arrived_t:.0f} s" if node.arrived_t is not None else f"closest {node.closest:.1f} m"),
            ("latch stayed attached", not node.latch_lost and node.latch.state == "attached", f"state {node.latch.state}"),
            ("AUV rigid in the boat frame", node.rel0 is not None and node.max_rel_dev < 0.02,
             f"max shift {node.max_rel_dev * 100:.1f} cm"),
        ]
        print("\nCHECKS", flush=True)
        for name, passed, detail in checks:
            print(f"  {'PASS' if passed else 'FAIL'}  {name}: {detail}", flush=True)
        ok = all(p for _, p, _ in checks) and not node.failed
        print("VERDICT " + ("PASS" if ok else f"FAIL {node.failed}".rstrip()), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
