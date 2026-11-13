#!/bin/bash
# Install the CoRAL aquatic twin (otter_k + xudrone on Kiwanis Lake) into
# digital-twin-aquatic. Run on the twin host from this folder:
#     C=digital-twin-aquatic bash install.sh
# Safe to run again. Builds first, then stops the simulator (refuses if a
# vehicle is armed), installs, and leaves the simulator stopped:
#     docker exec digital-twin-aquatic simctl start
# Does not touch the production otter twin (:40099).
set -e
C=${C:-digital-twin-aquatic}
HERE=$(cd "$(dirname "$0")" && pwd)
N=/tmp/udrone_new

docker exec "$C" rm -rf $N
docker exec "$C" mkdir -p $N/test
copy_in() {   # host file -> container path, without Windows line endings
  local tmp; tmp=$(mktemp)
  tr -d '\r' < "$1" > "$tmp"
  chmod 644 "$tmp"
  docker cp "$tmp" "$C:$2" >/dev/null
  rm -f "$tmp"
}

copy_in "$HERE/install_inner.sh"                  $N/install_inner.sh
copy_in "$HERE/patch_simctl_coral.sh"             $N/patch_simctl_coral.sh
copy_in "$HERE/adapter/udrone_thrust_adapter.cpp" $N/udrone_thrust_adapter.cpp
copy_in "$HERE/plugin/LinkAttacher.cc"            $N/LinkAttacher.cc
copy_in "$HERE/plugin/LinkAttacher.hh"            $N/LinkAttacher.hh
copy_in "$HERE/start_udrone.sh"                   $N/start_udrone.sh
copy_in "$HERE/start_boat.sh"                     $N/start_boat.sh
copy_in "$HERE/stop_sim.sh"                       $N/stop_sim.sh
copy_in "$HERE/xudrone_camera_bridge.sh"          $N/xudrone_camera_bridge.sh
copy_in "$HERE/scripts/dock_command.py"           $N/dock_command.py
copy_in "$HERE/airframe/60003_gz_xudrone"         $N/60003_gz_xudrone
copy_in "$HERE/model/xudrone/model.sdf"           $N/xudrone_model.sdf
copy_in "$HERE/model/xudrone/model.config"        $N/xudrone_model.config
copy_in "$HERE/model/heron/model.sdf"             $N/heron_model.sdf
copy_in "$HERE/model/heron/model.config"          $N/heron_model.config
copy_in "$HERE/world/kiwanis_lake.sdf"            $N/kiwanis_lake.sdf
for f in "$HERE"/test/*.py "$HERE"/test/*.sh; do
  if [ -f "$f" ]; then copy_in "$f" "$N/test/$(basename "$f")"; fi
done
if [ -f "$HERE/docs/KIWANIS.md" ]; then copy_in "$HERE/docs/KIWANIS.md" $N/KIWANIS.md; fi
tar -C "$HERE/world" -cf - kiwanis_lake | docker exec -i "$C" tar -C $N -xf -

echo "=== build ==="
docker exec "$C" bash $N/install_inner.sh build

if docker exec "$C" pgrep -x px4 >/dev/null; then
  echo "=== simulator running: every vehicle must be disarmed ==="
  if ! docker exec "$C" bash $N/install_inner.sh armed; then
    echo "a vehicle is armed; disarm it first" >&2
    exit 1
  fi
  echo "=== stop simulator ==="
  docker exec "$C" simctl stop | tail -3
fi

echo "=== install ==="
docker exec "$C" bash $N/install_inner.sh install

echo "=== done. Start the pair:  docker exec $C simctl start ==="
echo "=== solo AUV instead:      docker exec -d -e DISPLAY=:1 -e VGL_DISPLAY=:0.0 $C bash -c 'cd /root && exec vglrun ./start_udrone.sh >> /root/log/start_udrone.out 2>&1' ==="
