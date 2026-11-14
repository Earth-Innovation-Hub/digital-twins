#!/bin/bash
# uDrone AUV — xudrone. Solo (default) or Locutus-style secondary under otter_k.
#
#   /root/start_udrone.sh              # alone: instance 0, free swim at 0,0,-1
#   /root/start_udrone.sh --secondary  # CoRAL dock: instance 1 under heron_0
set -e
export ROS_DOMAIN_ID=84
export GZ_PARTITION=aquatic
export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}

SECONDARY=0
if [ "${1:-}" = "--secondary" ] || [ "${1:-}" = "-i1" ]; then
  SECONDARY=1
fi

mkdir -p /root/log

# start_adapter <gz model>: one adapter per model. The pattern is anchored to
# the executable path; `pgrep -x` cannot match a name longer than 15 characters.
#
# Helpers are started through `( ... & )` so they belong to init, not to this
# script: the script later becomes PX4 (exec), which never reaps children, and
# a helper that exits would stay behind as a zombie.
start_adapter() {
  if ! pgrep -f '^/usr/local/bin/udrone_thrust_adapter' >/dev/null; then
    (
      setsid nohup env GZ_PARTITION=aquatic UDRONE_GZ_MODEL="$1" \
        /usr/local/bin/udrone_thrust_adapter \
        >> /root/log/udrone_thrust_adapter.log 2>&1 < /dev/null &
    )
  fi
}

# start_camera_bridge <gz model>: waits for front + down camera topics, then bridges.
# xudrone_camera_bridge.sh holds a lock, so a second front copy exits by itself
# (the host watchdog may have started one first).
start_camera_bridge() {
  (
    setsid bash -c '
      for _ in $(seq 1 90); do
        if gz topic -l 2>/dev/null | grep -q "$0.*front_camera/image"; then
          break
        fi
        sleep 1
      done
      GZ_MODEL_NAME="$0" exec /usr/local/bin/xudrone_camera_bridge.sh
    ' "$1" >> /root/log/xudrone_camera_bridge.log 2>&1 < /dev/null &
  )
  (
    setsid bash -c '
      for _ in $(seq 1 90); do
        if gz topic -l 2>/dev/null | grep -q "$0.*down_camera/image"; then
          break
        fi
        sleep 1
      done
      GZ_MODEL_NAME="$0" exec /usr/local/bin/xudrone_down_camera_bridge.sh
    ' "$1" >> /root/log/xudrone_down_camera.log 2>&1 < /dev/null &
  )
}

boat_pose() {
  # Spawn at the boat so the latch plugin only has to seat the AUV in its nest
  # (boat-frame 0,0,0: between the hulls, under the deck). Waits until the
  # boat has dropped from its spawn height onto the water.
  python3 - <<'PY'
import os, subprocess, re, time
os.environ["GZ_PARTITION"] = "aquatic"
def read():
    return subprocess.check_output(
        "timeout 5 gz topic -e -t /world/kiwanis_lake/dynamic_pose/info -n 1",
        shell=True, text=True,
    )
def xyz(name, t):
    m = re.search(r'pose \{\s*name: "%s"(.*?)\n\}' % name, t, re.S)
    if not m:
        return None
    pos = re.search(r'position \{(.*?)\}', m.group(1), re.S)
    out = {"x": 0.0, "y": 0.0, "z": 0.0}   # zero fields are omitted
    for k, v in re.findall(r'([xyz]): ([-\d.eE+]+)', pos.group(1) if pos else ""):
        out[k] = float(v)
    return out["x"], out["y"], out["z"]
pose = None
for _ in range(30):
    try:
        pose = xyz("heron_0", read())
    except Exception:
        pose = None
    if pose and pose[2] < 0.05:
        break
    time.sleep(1)
if not pose:
    print("0,0,-0.15")
else:
    print("%.4f,%.4f,%.4f" % pose)
PY
}

if [ "$SECONDARY" = 1 ]; then
  ready=0
  for _ in $(seq 1 120); do
    if gz topic -l 2>/dev/null | grep -q '^/world/kiwanis_lake/clock$'; then
      ready=1
      break
    fi
    sleep 1
  done
  if [ "$ready" != 1 ]; then
    echo "ERROR xudrone secondary: /world/kiwanis_lake/clock never appeared" >&2
    exit 1
  fi

  # A copy left by an earlier secondary would collide with the new spawn.
  if gz model --list 2>/dev/null | grep -q 'xudrone_1'; then
    gz service -s /world/kiwanis_lake/remove \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req 'name: "xudrone_1" type: MODEL' >/dev/null 2>&1 || true
    sleep 1
  fi

  start_adapter xudrone_1
  start_camera_bridge xudrone_1

  if [ -n "${PX4_GZ_MODEL_POSE:-}" ]; then
    MOUNT_POSE=$PX4_GZ_MODEL_POSE
  else
    MOUNT_POSE=$(boat_pose)
  fi
  echo "$(date -u +%FT%TZ) xudrone secondary spawn pose: $MOUNT_POSE"

  cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
  [ -f ./gz_env.sh ] && . ./gz_env.sh || true
  export ROS_DOMAIN_ID=84
  export GZ_PARTITION=aquatic
  export PX4_UXRCE_DDS_NS=xudrone
  export PX4_SYS_AUTOSTART=60003
  export PX4_SIM_MODEL=xudrone
  export PX4_GZ_WORLD=kiwanis_lake
  export PX4_GZ_MODEL_POSE="$MOUNT_POSE"
  exec ../bin/px4 -i 1 -d >> /root/log/xudrone.log 2>&1
fi

# ---- solo mode (instance 0) ----
if pgrep -x px4 >/dev/null || pgrep -f '^gz sim' >/dev/null; then
  echo "a simulator is already running; stop it first: simctl stop" >&2
  exit 1
fi

rm -f /root/.sim_hold

pkill -f '^/usr/local/bin/udrone_thrust_adapter' 2>/dev/null || true
pkill -f 'xudrone_camera_bridge|front_camera_link/sensor/front_camera' 2>/dev/null || true
pkill -f 'xudrone_down_camera_bridge|xudrone.*/down_camera' 2>/dev/null || true
pkill -f 'apriltag_node|tag_detector|run_tags' 2>/dev/null || true
sleep 1

if ! pgrep -x MicroXRCEAgent >/dev/null; then
  MicroXRCEAgent udp4 -p 8888 >> /root/log/uxrce84.log 2>&1 &
fi

start_adapter xudrone_0
start_camera_bridge xudrone_0
(
  sleep 35
  exec /usr/local/bin/coral_tags_xudrone.sh
) >> /root/log/coral_tags_xudrone.log 2>&1 &

cd /root/PX4-Autopilot/build/px4_sitl_default
tail -f /dev/null | env \
  PX4_UXRCE_DDS_NS=xudrone \
  PX4_SYS_AUTOSTART=60003 \
  PX4_SIM_MODEL=xudrone \
  PX4_GZ_WORLD=kiwanis_lake \
  PX4_GZ_MODEL_POSE=0,0,-1.0 \
  ./bin/px4 >> /root/log/xudrone.log 2>&1
