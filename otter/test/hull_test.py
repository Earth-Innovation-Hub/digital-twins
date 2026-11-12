#!/usr/bin/env python3
"""Open-loop hull test for the Heron twin: command jet thrust directly, log Gazebo ground truth.

usage: hull_test.py NAME port,stbd,seconds [port,stbd,seconds ...]
Prints, per segment, the mean over its last 30 %: surge u, sway v (body, m/s), yaw rate r
(deg/s), pitch, roll (deg), z (m), and for the whole segment the peak pitch/roll and the time
to reach 63 % of the final surge speed or yaw rate. Writes /tmp/hull_NAME.csv.
"""
import math
import sys
import threading
import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

MODEL = "heron_0"
node = Node()
lock = threading.Lock()
samples = []          # (t, x, y, z, roll, pitch, yaw)


def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    for p in msg.pose:
        if p.name == MODEL:
            q = p.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            with lock:
                samples.append((t, p.position.x, p.position.y, p.position.z, roll, pitch, yaw))
            return


node.subscribe(Pose_V, "/world/tempe_lake/dynamic_pose/info", on_pose)
pubs = [node.advertise(f"/model/{MODEL}/joint/{j}_jet_joint/cmd_thrust", Double) for j in ("port", "starboard")]


def send(port, stbd):
    for pub, val in zip(pubs, (port, stbd)):
        m = Double()
        m.data = float(val)
        pub.publish(m)


def rates(seg, win=0.25):
    """Body-frame u, v and yaw rate by central differences over about `win` seconds."""
    out = []
    n = len(seg)
    j0 = 0
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
        vx, vy = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
        yaw = seg[i][6]
        u = vx * math.cos(yaw) + vy * math.sin(yaw)
        v = -vx * math.sin(yaw) + vy * math.cos(yaw)
        r = math.atan2(math.sin(b[6] - a[6]), math.cos(b[6] - a[6])) / dt
        out.append((t, u, v, r))
    return out


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def main():
    name = sys.argv[1]
    segs = [tuple(float(v) for v in a.split(",")) for a in sys.argv[2:]]
    time.sleep(1.0)
    print(f"{'port':>6} {'stbd':>6} {'sec':>5} | {'u':>6} {'v':>6} {'r dps':>7} {'pitch':>6} {'roll':>6} {'z':>7} |"
          f" {'pk pit':>6} {'pk rol':>6} {'tau':>5} {'dist':>6}")
    everything = []
    for port, stbd, dur in segs:
        with lock:
            i0 = len(samples)
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            send(port, stbd)
            time.sleep(0.02)
        with lock:
            seg = samples[i0:]
        everything += [(port, stbd) + s for s in seg]
        if len(seg) < 20:
            print("no pose samples")
            continue
        rs = rates(seg)
        t0, t1 = seg[0][0], seg[-1][0]
        tail = [x for x in rs if x[0] >= t1 - 0.3 * (t1 - t0)]
        tail_s = [s for s in seg if s[0] >= t1 - 0.3 * (t1 - t0)]
        u, v, r = mean([x[1] for x in tail]), mean([x[2] for x in tail]), mean([x[3] for x in tail])
        # time to 63 % of the change in the dominant signal
        key = 1 if abs(port + stbd) >= abs(port - stbd) else 3
        start, final = rs[0][key], (u if key == 1 else r)
        tau = float("nan")
        if abs(final - start) > 1e-3:
            for x in rs:
                if (x[key] - start) / (final - start) >= 0.632:
                    tau = x[0] - t0
                    break
        dist = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2])
        print(f"{port:6.1f} {stbd:6.1f} {dur:5.0f} | {u:6.2f} {v:6.2f} {math.degrees(r):7.1f} "
              f"{math.degrees(mean([s[5] for s in tail_s])):6.2f} {math.degrees(mean([s[4] for s in tail_s])):6.2f} "
              f"{mean([s[3] for s in tail_s]):7.3f} | "
              f"{math.degrees(max(abs(s[5]) for s in seg)):6.2f} {math.degrees(max(abs(s[4]) for s in seg)):6.2f} "
              f"{tau:5.2f} {dist:6.1f}")
    for _ in range(25):
        send(0, 0)
        time.sleep(0.02)
    with open(f"/tmp/hull_{name}.csv", "w") as f:
        f.write("port,stbd,t,x,y,z,roll,pitch,yaw\n")
        for row in everything:
            f.write(",".join(f"{v:.5f}" for v in row) + "\n")


main()
