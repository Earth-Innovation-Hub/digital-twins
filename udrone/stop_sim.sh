#!/bin/bash
# Stop the aquatic simulator: PX4 (every instance), Gazebo, the uXRCE agent,
# both thrust adapters, the xudrone camera bridge and the launch scripts.
# The noVNC desktop, px4_relay and the DeepGIS agents are left running.
#   stop_sim.sh --dry-run   list what would be stopped
set +e
dry=0
if [ "${1:-}" = "--dry-run" ]; then
  dry=1
fi
python3 - "$dry" << 'PY'
import os
import signal
import sys
import time

dry = sys.argv[1] == "1"
me = os.getpid()
parent = os.getppid()

def cmdline(pid):
    try:
        raw = open(f"/proc/{pid}/cmdline", "rb").read()
    except OSError:
        return ""
    return raw.replace(b"\0", b" ").decode(errors="replace").strip()

LAUNCH_SCRIPTS = ("start_boat.sh", "start_udrone.sh", "start_xroboato.sh")

def kind(cmd):
    if cmd.startswith("gz sim ") or cmd == "gz sim":
        return "gz"
    if cmd.startswith("MicroXRCEAgent ") or cmd == "MicroXRCEAgent":
        return "agent"
    if cmd.startswith(("/usr/local/bin/heron_thrust_adapter",
                       "/usr/local/bin/udrone_thrust_adapter",
                       "/usr/local/bin/xroboato_thrust_adapter")):
        return "adapter"
    if "xudrone_camera_bridge" in cmd or "front_camera_link/sensor/front_camera" in cmd:
        return "bridge"
    if "PX4_SIM_MODEL=" in cmd or (cmd.startswith("bash") and "bin/px4" in cmd):
        return "launcher"
    words = cmd.split()
    if words and os.path.basename(words[0]) in ("bash", "sh") and any(
            os.path.basename(w) in LAUNCH_SCRIPTS for w in words[1:3]):
        return "launcher"
    if "/bin/px4" in cmd and "px4_relay" not in cmd and "px4_executor" not in cmd:
        return "px4"
    if cmd.startswith("tail -f /dev/null"):
        return "tail"
    return ""

rows = []
for name in os.listdir("/proc"):
    if not name.isdigit():
        continue
    pid = int(name)
    if pid in (me, parent, 1):
        continue
    cmd = cmdline(pid)
    tag = kind(cmd)
    if tag:
        rows.append((pid, tag, cmd))

launcher_pids = {pid for pid, tag, _ in rows if tag == "launcher"}
kept = []
for pid, tag, cmd in rows:
    if tag == "tail":
        try:
            ppid = int(open(f"/proc/{pid}/stat", "rb").read().split()[3])
        except (OSError, IndexError, ValueError):
            ppid = 0
        if ppid not in launcher_pids:
            continue
    kept.append((pid, tag, cmd))

# Launchers first: a launcher still waiting for the world would otherwise
# spawn a vehicle after the simulator is gone.
order = {"launcher": 0, "px4": 1, "gz": 2, "adapter": 3, "bridge": 3, "agent": 4, "tail": 5}
kept.sort(key=lambda row: order[row[1]])

if not kept:
    print("no sim processes")
    sys.exit(0)

for pid, tag, cmd in kept:
    print(f"{'would stop' if dry else 'stop'} {pid} {tag} {cmd[:180]}")
    if dry:
        continue
    try:
        os.kill(pid, signal.SIGTERM)
    except OSError as exc:
        print(exc)

if dry:
    sys.exit(0)

time.sleep(2)
for pid, tag, cmd in kept:
    if os.path.exists(f"/proc/{pid}"):
        print(f"kill {pid} {tag}")
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError as exc:
            print(exc)
print("sim stopped")
PY
# Tells a host watchdog (if any) not to restart the simulator.
if [ "$dry" = 0 ]; then touch /root/.sim_hold; echo "auto-restart paused until simctl start"; fi
