#!/bin/bash
# Make the aquatic simctl describe the CoRAL pair (otter_k + xudrone).
# Runs inside the container; safe to run again (it rewrites the same lines).
set -e
S=/usr/local/bin/simctl
[ -x "$S" ] || exit 0
python3 - "$S" <<'PY'
import pathlib, re, shutil, sys, time
p = pathlib.Path(sys.argv[1])
old = p.read_text()
t = old

# Both PX4 instances, with their callsigns and logs.
t, n = re.subn(
    r'TWIN=boat; INSTANCES="[^"]*"; NAMES=\([^)]*\); LOGS="[^"]*"',
    lambda m: 'TWIN=boat; INSTANCES="0 1"; NAMES=(otter_k xudrone); '
              'LOGS="/root/log/boat.log /root/log/xudrone.log /root/log/start_boat.out"',
    t, count=1)
if n != 1:
    sys.exit("simctl: boat branch not found; not patched")

heron = "show \"thrust adapter\" \"$(pgrep -f '^/usr/local/bin/heron_thrust_adapter' | head -1)\""
udrone = "    show \"udrone thrust adapter\" \"$(pgrep -f '^/usr/local/bin/udrone_thrust_adapter' | head -1)\""
camera = "    show \"xudrone camera\" \"$(pgrep -f 'lib/ros_gz_bridge/parameter_bridge .*front_camera' | head -1)\""
latch = ("    printf \"  %-22s %s\\n\" \"dock latch\" \"$(GZ_PARTITION=aquatic timeout 4 gz topic -e "
         "-t /model/heron_0/udrone_dock/state -n 1 2>/dev/null | sed -n 's/.*data: \"\\(.*\\)\".*/\\1/p' "
         "| grep . || echo unknown)\"")

lines = []
for line in t.split("\n"):
    s = line.strip()
    if s.startswith('show "udrone thrust adapter"') or s.startswith('show "udrone dock lock"') \
            or s.startswith('show "xudrone camera"') or '"dock latch"' in s:
        continue                      # re-added below in one place
    if s.startswith('show "camera bridge"'):
        continue                      # heron bridge: not part of this twin
    lines.append(line)
    if s == heron:
        lines += [udrone, camera, latch]
t = "\n".join(lines)
if udrone not in t:
    sys.exit("simctl: heron thrust adapter line not found; not patched")

if t != old:
    shutil.copy2(p, f"{p}.bak-coral-{time.strftime('%Y%m%d%H%M%S')}")
    p.write_text(t)
    print("simctl patched for the CoRAL pair")
else:
    print("simctl already patched")
PY
