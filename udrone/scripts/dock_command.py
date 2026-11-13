#!/usr/bin/env python3
"""Open-UAV-style ATTACH/DETACH for the CoRAL latch (gz-sim LinkAttacher).

Baseline: https://github.com/Open-UAV/gazebo_ros_link_attacher scripts/attach.py

  dock_command.py ATTACH    weld xudrone into the nest under otter_k
                            (refused beyond 0.6 m / 30 deg from the nest)
  dock_command.py DETACH    release it
  dock_command.py STATE     print attached | detached

ATTACH and DETACH print the latch state afterwards and exit 1 if the latch
did not end up in the requested state (for example ATTACH out of range).
"""
import os
import re
import subprocess
import sys
import time

os.environ.setdefault("GZ_PARTITION", "aquatic")

COMMAND_TOPIC = "/udrone_dock/command"
STATE_TOPIC = "/model/heron_0/udrone_dock/state"


def state(timeout_s=4.0):
    """Latch state from the plugin's periodic message, or None."""
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", STATE_TOPIC, "-n", "1"],
            capture_output=True, text=True, timeout=timeout_s,
        ).stdout
    except subprocess.TimeoutExpired:
        return None
    m = re.search(r'data: "(\w+)"', out)
    return m.group(1) if m else None


def send(cmd):
    return subprocess.run(
        ["gz", "topic", "-t", COMMAND_TOPIC, "-m", "gz.msgs.StringMsg",
         "-p", f'data: "{cmd}"'],
        capture_output=True, text=True, timeout=10,
    ).returncode


def main():
    cmd = sys.argv[1].upper() if len(sys.argv) > 1 else ""
    if cmd not in ("ATTACH", "DETACH", "STATE"):
        print("usage: dock_command.py ATTACH|DETACH|STATE", file=sys.stderr)
        return 2
    if cmd == "STATE":
        s = state()
        print(s or "unknown (no latch plugin answering)")
        return 0 if s else 1
    want = "attached" if cmd == "ATTACH" else "detached"
    rc = send(cmd)
    if rc != 0:
        print(f"could not publish {cmd}", file=sys.stderr)
        return rc
    s = None
    deadline = time.time() + 6.0
    while time.time() < deadline:
        s = state(2.5)
        if s == want:
            break
        time.sleep(0.2)
    print(s or "unknown")
    return 0 if s == want else 1


if __name__ == "__main__":
    raise SystemExit(main())
