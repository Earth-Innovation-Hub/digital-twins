#!/usr/bin/env python3
"""Publish one /coral/command and optionally wait for its terminal event.

Examples:
  coral_cmd.py dock detach
  coral_cmd.py dock attach
  coral_cmd.py goto --along 8
  coral_cmd.py goto --east 8 --north 0
  coral_cmd.py goto --east 1.5 --north 2.0 --world
  coral_cmd.py helix --duration 20 --radius 0.4
  coral_cmd.py abort
  coral_cmd.py status
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CMD = "/coral/command"
EVT = "/coral/event"
STS = "/coral/status"
TERMINAL = {"arrived", "dock_done", "helix_done", "failed", "rejected", "aborted"}


class Once(Node):
    def __init__(self):
        super().__init__("coral_cmd")
        self.pub = self.create_publisher(String, CMD, 10)
        self.events = []
        self.status = None
        self.create_subscription(String, EVT, self._on_evt, 10)
        self.create_subscription(String, STS, self._on_sts, 10)

    def _on_evt(self, m):
        try:
            self.events.append(json.loads(m.data))
        except ValueError:
            pass

    def _on_sts(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(argv=None):
    p = argparse.ArgumentParser(description="Send a CoRAL behaviour command")
    p.add_argument("type", choices=("dock", "goto", "helix", "abort", "hold", "status"))
    p.add_argument("action", nargs="?", help="attach|detach for dock")
    p.add_argument("--east", type=float, default=0.0)
    p.add_argument("--north", type=float, default=0.0)
    p.add_argument("--along", type=float, default=None,
                   help="metres forward along current heading (sets relative_to=heading)")
    p.add_argument("--heading", action="store_true",
                   help="treat --north as forward, --east as starboard")
    p.add_argument("--world", action="store_true",
                   help="goto absolute ENU (default: ENU offset from current pose)")
    p.add_argument("--arrive", type=float, default=None)
    p.add_argument("--timeout", type=float, default=None)
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--radius", type=float, default=None)
    p.add_argument("--omega", type=float, default=None)
    p.add_argument("--vx", type=float, default=None)
    p.add_argument("--z0", type=float, default=None)
    p.add_argument("--id", default=None)
    p.add_argument("--wait", type=float, default=180.0,
                   help="seconds to wait for a terminal event")
    p.add_argument("--no-wait", action="store_true")
    args = p.parse_args(argv)

    rclpy.init()
    node = Once()
    spin_for(node, 0.5)

    if args.type == "status":
        spin_for(node, 2.0)
        print(json.dumps(node.status or {"error": "no /coral/status yet"}, indent=2))
        rc = 0 if node.status else 1
        node.destroy_node()
        rclpy.shutdown()
        return rc

    cid = args.id or uuid.uuid4().hex[:10]
    cmd = {"type": args.type, "id": cid}
    if args.type == "dock":
        action = (args.action or "").lower()
        if action not in ("attach", "detach"):
            print("dock requires attach|detach", file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 2
        cmd["action"] = action
    elif args.type == "goto":
        if args.along is not None:
            cmd["along_heading_m"] = args.along
            cmd["relative_to"] = "heading"
        else:
            cmd["east_m"] = args.east
            cmd["north_m"] = args.north
            if args.world:
                cmd["relative_to"] = "world"
            elif args.heading:
                cmd["relative_to"] = "heading"
            else:
                cmd["relative_to"] = "current"
        if args.arrive is not None:
            cmd["arrive_m"] = args.arrive
        if args.timeout is not None:
            cmd["timeout_s"] = args.timeout
    elif args.type == "helix":
        mapping = (
            (args.duration, "duration_s"),
            (args.radius, "radius_m"),
            (args.omega, "omega"),
            (args.vx, "vx"),
            (args.z0, "z0"),
        )
        for val, key in mapping:
            if val is not None:
                cmd[key] = val

    print(f"SEND {json.dumps(cmd)}", flush=True)
    spin_for(node, 0.3)
    node.pub.publish(String(data=json.dumps(cmd)))

    if args.no_wait or args.wait <= 0:
        spin_for(node, 0.3)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    terminal = None
    deadline = time.time() + args.wait
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        for e in node.events:
            if e.get("id") == cid and e.get("type") in TERMINAL:
                terminal = e
                break
        if terminal:
            break

    if terminal is None:
        print(f"TIMEOUT waiting for id={cid}", flush=True)
        print("events so far:", json.dumps(node.events, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"EVENT {json.dumps(terminal)}", flush=True)
    ok = True
    if terminal.get("type") in ("failed", "rejected"):
        ok = False
    if terminal.get("type") == "helix_done" and terminal.get("failed"):
        ok = False
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
