#!/bin/bash
# Install badger's r1_rover_cam model and start_rover.sh into digital-twin-locutus.
# Meshes are copied from the stock r1_rover model already in the PX4 tree (not vendored).
# Does not restart the twin; restart the rover process afterwards.
set -e
C=${C:-digital-twin-locutus}
P=/root/PX4-Autopilot
GZ=$P/Tools/simulation/gz/models
SFX=bak-$(date +%Y%m%d)
HERE=$(cd "$(dirname "$0")" && pwd)

docker exec $C test -d $GZ/r1_rover/meshes || {
  echo "stock r1_rover meshes not found in $C:$GZ/r1_rover/meshes" >&2
  exit 1
}

docker exec $C mkdir -p /tmp/badger_new/r1_rover_cam
tr -d '\r' < "$HERE/models/r1_rover_cam/model.sdf" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/r1_rover_cam/model.sdf
tr -d '\r' < "$HERE/models/r1_rover_cam/model.config" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/r1_rover_cam/model.config
tr -d '\r' < "$HERE/start_rover.sh" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/start_rover.sh
rm -f /tmp/badger_install.$$

docker exec $C bash -c "
  set -e
  bk() { [ ! -e \"\$1\" ] || [ -e \"\$1.$SFX\" ] || cp -p \"\$1\" \"\$1.$SFX\"; }
  D=$GZ/r1_rover_cam
  mkdir -p \$D/meshes
  bk \$D/model.sdf; bk \$D/model.config; bk /root/start_rover.sh
  cp /tmp/badger_new/r1_rover_cam/model.sdf \$D/model.sdf
  cp /tmp/badger_new/r1_rover_cam/model.config \$D/model.config
  # meshes: prefer hard links to stock r1_rover (same content, no duplicate bytes)
  for f in $GZ/r1_rover/meshes/*; do
    b=\$(basename \"\$f\")
    ln -f \"\$f\" \"\$D/meshes/\$b\" 2>/dev/null || cp -p \"\$f\" \"\$D/meshes/\$b\"
  done
  cp /tmp/badger_new/start_rover.sh /root/start_rover.sh
  chmod +x /root/start_rover.sh
  echo \"installed \$D (\$(ls \$D/meshes | wc -l) meshes) and /root/start_rover.sh\"
"
echo "done — restart the rover (or the locutus container) to pick this up"
