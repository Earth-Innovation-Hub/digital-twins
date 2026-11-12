#!/usr/bin/env python3
"""Closed-loop leg test for otter: command the surface executor, log truth and thrust.

usage: closed_test.py NAME goto:<bearing_deg_rel>:<dist_m>[:timeout]  |  orbit:<radius>:<sec>[:ccw]
       ... (several, run in order)   [land]
Bearings are relative to the boat's heading at the start of the leg (0 = straight ahead,
90 = to starboard). Writes /tmp/closed_NAME.csv and prints one summary block per leg.
"""
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
lock = threading.Lock()
truth = []                 # (t, x_enu, y_enu, z, roll, pitch, yaw_enu, port_N, stbd_N)
jets = [0.0, 0.0]
px4 = {"pos": None, "armed": False, "nav": None}
events = []


def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    for p in msg.pose:
        if p.name == "heron_0":
            q = p.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            with lock:
                truth.append((t, p.position.x, p.position.y, p.position.z, roll, pitch, yaw, jets[0], jets[1]))
            return


gz = GzNode()
gz.subscribe(Pose_V, "/world/tempe_lake/dynamic_pose/info", on_pose)
gz.subscribe(Double, "/model/heron_0/joint/port_jet_joint/cmd_thrust", lambda m: jets.__setitem__(0, m.data))
gz.subscribe(Double, "/model/heron_0/joint/starboard_jet_joint/cmd_thrust", lambda m: jets.__setitem__(1, m.data))


class Driver(RosNode):
    def __init__(self):
        super().__init__("otter_leg_test")
        self.cmd = self.create_publisher(String, "/otter/mission_supervisor/command", 10)
        self.create_subscription(String, "/otter/mission_supervisor/event", lambda m: events.append((time.monotonic(), m.data)), 10)
        self.create_subscription(VehicleLocalPosition, "/otter/fmu/out/vehicle_local_position", self.on_pos, QOS)
        self.create_subscription(VehicleStatus, "/otter/fmu/out/vehicle_status_v1", self.on_status, QOS)

    def on_pos(self, m):
        px4["pos"] = (m.x, m.y, m.vx, m.vy, m.heading)

    def on_status(self, m):
        px4["armed"] = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        px4["nav"] = m.nav_state

    def send(self, d):
        # A command published before the executor's subscription is matched is lost.
        t0 = time.monotonic()
        while self.cmd.get_subscription_count() == 0 and time.monotonic() - t0 < 8.0:
            time.sleep(0.1)
        self.cmd.publish(String(data=json.dumps(d)))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def body_rates(seg, win=0.3):
    out, j0, n = [], 0, len(seg)
    for i in range(n):
        t = seg[i][0]
        while seg[j0][0] < t - win / 2:
            j0 += 1
        j1 = i
        while j1 + 1 < n and seg[j1 + 1][0] <= t + win / 2:
            j1 += 1
        a, b = seg[j0], seg[j1]
        dt = b[0] - a[0]
        if dt < 0.05:
            continue
        vx, vy, yaw = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt, seg[i][6]
        out.append((t, vx * math.cos(yaw) + vy * math.sin(yaw), -vx * math.sin(yaw) + vy * math.cos(yaw),
                    wrap(b[6] - a[6]) / dt, i))
    return out


def summarize(label, seg, goal_enu=None, centre_enu=None, radius=None):
    if len(seg) < 30:
        print(f"{label}: no data")
        return
    rs = body_rates(seg)
    dur = seg[-1][0] - seg[0][0]
    speed = [math.hypot(r[1], r[2]) for r in rs]
    print(f"--- {label}: {dur:.1f} s sim time")
    print(f"  speed max {max(speed):.2f}  mean {mean(speed):.2f} m/s;  |sway| max {max(abs(r[2]) for r in rs):.2f} m/s;"
          f"  yaw rate max {math.degrees(max(abs(r[3]) for r in rs)):.0f} deg/s")
    print(f"  pitch max {math.degrees(max(abs(s[5]) for s in seg)):.1f}  roll max {math.degrees(max(abs(s[4]) for s in seg)):.1f} deg")
    # yaw-rate reversals: how often the boat changes turning direction (weaving)
    sign, flips = 0, 0
    for r in rs:
        s = 1 if r[3] > math.radians(4) else -1 if r[3] < -math.radians(4) else 0
        if s and sign and s != sign:
            flips += 1
        sign = s or sign
    print(f"  yaw-rate reversals {flips} ({flips / dur * 60:.0f} per minute)")
    # thrust activity: mean absolute change per second and time at a limit
    dthr = [abs(seg[i][7] - seg[i - 1][7]) + abs(seg[i][8] - seg[i - 1][8]) for i in range(1, len(seg))]
    sat = sum(1 for s in seg if s[7] > 33 or s[8] > 33 or s[7] < -19.5 or s[8] < -19.5) / len(seg)
    rev = sum(1 for s in seg if s[7] < -1 or s[8] < -1) / len(seg)
    print(f"  thrust travel {sum(dthr) / dur:.0f} N/s;  at a limit {100 * sat:.0f} % of the time;  a jet in reverse {100 * rev:.0f} %")
    x0, y0 = seg[0][1], seg[0][2]
    if goal_enu:
        gx, gy = goal_enu
        L = math.hypot(gx - x0, gy - y0)
        ux, uy = (gx - x0) / L, (gy - y0) / L
        xt = [-(s[1] - x0) * uy + (s[2] - y0) * ux for s in seg]
        along = [(s[1] - x0) * ux + (s[2] - y0) * uy for s in seg]
        path = sum(math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2]) for i in range(1, len(seg)))
        end = math.hypot(seg[-1][1] - gx, seg[-1][2] - gy)
        closest = min(math.hypot(s[1] - gx, s[2] - gy) for s in seg)
        print(f"  leg {L:.1f} m: path {path:.1f} m ({path / L:.2f}x);  cross-track max {max(abs(v) for v in xt):.1f}  rms {rms(xt):.1f} m;"
              f"  overshoot past goal {max(0.0, max(along) - L):.1f} m")
        print(f"  ended {end:.1f} m from the goal (closest {closest:.1f} m), "
              f"{'past it' if along[-1] > L else 'short of it'}; final speed {speed[-1]:.2f} m/s")
        # approach profile: speed and total thrust when first inside each distance
        prof = []
        for d in (8, 5, 3, 2, 1, 0.5):
            for r in rs:
                s = seg[r[4]]
                if math.hypot(s[1] - gx, s[2] - gy) <= d:
                    prof.append(f"{d:g} m: {math.hypot(r[1], r[2]):.2f} m/s, {s[7] + s[8]:+.0f} N")
                    break
        print("  approach: " + ";  ".join(prof))
        # speed overshoot above the cruise plateau
        t_peak = max(rs, key=lambda r: math.hypot(r[1], r[2]))[0] - seg[0][0]
        print(f"  peak speed at {t_peak:.1f} s")
        # heading error while under way (after the first 3 m, until 6 m short)
        he = []
        for r in rs:
            s = seg[r[4]]
            a = (s[1] - x0) * ux + (s[2] - y0) * uy
            if 3 < a < L - 6:
                he.append(math.degrees(wrap(math.atan2(gy - s[2], gx - s[1]) - s[6])))
        if he:
            print(f"  heading error to goal while under way: rms {rms(he):.0f}  max {max(abs(h) for h in he):.0f} deg")
    if centre_enu:
        cx, cy = centre_enu
        on = [s for s in seg if abs(math.hypot(s[1] - cx, s[2] - cy) - radius) < 4]
        t_on = on[0][0] if on else None
        if t_on is None:
            print("  never reached the circle")
            return
        late = [s for s in seg if s[0] >= t_on + 10]
        err = [math.hypot(s[1] - cx, s[2] - cy) - radius for s in late]
        ang = 0.0
        for i in range(1, len(late)):
            ang += wrap(math.atan2(late[i][2] - cy, late[i][1] - cx) - math.atan2(late[i - 1][2] - cy, late[i - 1][1] - cx))
        lsp = [math.hypot(r[1], r[2]) for r in rs if seg[r[4]][0] >= t_on + 10]
        slip = [math.degrees(math.atan2(r[2], r[1])) for r in rs if seg[r[4]][0] >= t_on + 10 and abs(r[1]) > 0.2]
        print(f"  on the circle after {t_on - seg[0][0]:.0f} s; radius error mean {mean(err):+.1f}  rms {rms(err):.1f}  max {max(abs(e) for e in err):.1f} m")
        print(f"  swept {abs(math.degrees(ang)):.0f} deg at {mean(lsp):.2f} m/s; sideslip mean {mean(slip):+.0f} deg")


def main():
    name = sys.argv[1]
    rclpy.init()
    node = Driver()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while (px4["pos"] is None or not truth) and time.monotonic() - t0 < 15:
        time.sleep(0.2)
    if px4["pos"] is None or not truth:
        print("no PX4 position or no Gazebo pose")
        return
    # NED <-> ENU offset: PX4 local origin in the Gazebo world
    with lock:
        s = truth[-1]
    n, e = px4["pos"][0], px4["pos"][1]
    off_x, off_y = s[1] - e, s[2] - n          # enu_x = e + off_x, enu_y = n + off_y
    print(f"PX4 local ({n:.1f} N, {e:.1f} E) = world ({s[1]:.1f}, {s[2]:.1f}); armed {px4['armed']} nav {px4['nav']}")
    # PX4 takes home where it arms, so for an rtl leg start the script with the boat disarmed.
    home_n, home_e = n, e
    rows = []
    for k, arg in enumerate(sys.argv[2:]):
        parts = arg.split(":")
        tid = f"test_{name}_{k}_{int(time.time()) % 100000}"
        n, e, _, _, hdg = px4["pos"]
        with lock:
            i0 = len(truth)
        ev0 = len(events)
        if parts[0] == "land":
            node.send({"type": "land", "reason": "test end"})
            t0 = time.monotonic()
            while px4["armed"] is not False and time.monotonic() - t0 < 10.0:
                time.sleep(0.2)
            print(f"land: armed {px4['armed']}")
            continue
        if parts[0] == "rtl":
            node.send({"type": "rtl", "reason": "test"})
            t_start = time.monotonic()
            timeout = float(parts[1]) if len(parts) > 1 else 240.0
            while time.monotonic() - t_start < timeout:
                time.sleep(0.5)
                x, y, vx, vy, _ = px4["pos"]
                if time.monotonic() - t_start > 5 and not px4["armed"]:
                    break
            time.sleep(6.0)
            with lock:
                seg = truth[i0:]
            x, y = px4["pos"][0], px4["pos"][1]
            print(f"result: rtl ended {math.hypot(x - home_n, y - home_e):.1f} m from home, armed {px4['armed']}, nav {px4['nav']}  (wall {time.monotonic() - t_start:.0f} s)")
            summarize("return to launch", seg, goal_enu=(home_e + off_x, home_n + off_y))
            rows += [(k,) + r for r in seg]
            continue
        if parts[0] == "wait":
            time.sleep(float(parts[1]))
            with lock:
                seg = truth[i0:]
            summarize(f"wait {parts[1]} s", seg)
            rows += [(k,) + r for r in seg]
            continue
        if parts[0] == "goto":
            brg, dist = hdg + math.radians(float(parts[1])), float(parts[2])
            timeout = float(parts[3]) if len(parts) > 3 else 120.0
            gn, ge = n + dist * math.cos(brg), e + dist * math.sin(brg)
            cmd = {"type": "goto", "target_id": tid, "n": gn, "e": ge, "new_mission": True}
            label = f"goto {parts[1]} deg, {dist:g} m"
        else:
            radius, secs = float(parts[1]), float(parts[2])
            timeout = secs + 90.0
            gn, ge = n + radius * math.cos(hdg + math.pi / 2), e + radius * math.sin(hdg + math.pi / 2)
            cmd = {"type": "orbit", "target_id": tid, "n": gn, "e": ge, "radius_m": radius, "seconds": secs,
                   "clockwise": len(parts) < 4, "new_mission": True}
            label = f"orbit r {radius:g} m for {secs:g} s"
        node.send(cmd)
        t_start, result = time.monotonic(), "timeout"
        while time.monotonic() - t_start < timeout:
            time.sleep(0.1)
            done = [d for _, d in events[ev0:] if tid in d or '"failed"' in d]
            if done:
                result = done[-1]
                break
        time.sleep(8.0)           # watch it settle
        with lock:
            seg = truth[i0:]
        print(f"result: {result[:160]}  (wall {time.monotonic() - t_start:.0f} s)")
        if parts[0] == "goto":
            summarize(label, seg, goal_enu=(ge + off_x, gn + off_y))
        else:
            summarize(label, seg, centre_enu=(ge + off_x, gn + off_y), radius=radius)
        rows += [(k,) + r for r in seg]
    with open(f"/tmp/closed_{name}.csv", "w") as f:
        f.write("leg,t,x,y,z,roll,pitch,yaw,port,stbd\n")
        for r in rows:
            f.write(",".join(f"{v:.4f}" for v in r) + "\n")
    node.destroy_node()
    rclpy.shutdown()


main()
