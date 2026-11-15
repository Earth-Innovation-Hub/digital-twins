#!/bin/bash
# Install the Earth Rover model and start_rover.sh into digital-twin-locutus.
# Does not restart the twin; restart the rover process afterwards.
set -e
C=${C:-digital-twin-locutus}
P=/root/PX4-Autopilot
GZ=$P/Tools/simulation/gz/models
SFX=bak-$(date +%Y%m%d)
HERE=$(cd "$(dirname "$0")" && pwd)

docker exec $C mkdir -p /tmp/badger_new/earth_rover/materials/textures
tr -d '\r' < "$HERE/models/earth_rover/model.sdf" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/earth_rover/model.sdf
if [ -f "$HERE/models/earth_rover/materials/textures/apriltag36h11_1.png" ]; then
  docker cp "$HERE/models/earth_rover/materials/textures/apriltag36h11_1.png" \
    $C:/tmp/badger_new/earth_rover/materials/textures/apriltag36h11_1.png
fi
tr -d '\r' < "$HERE/models/earth_rover/model.config" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/earth_rover/model.config
tr -d '\r' < "$HERE/start_rover.sh" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/start_rover.sh
tr -d '\r' < "$HERE/airframe/4024_gz_earth_rover" > /tmp/badger_install.$$
docker cp /tmp/badger_install.$$ $C:/tmp/badger_new/4024_gz_earth_rover
rm -f /tmp/badger_install.$$

docker exec $C bash -c "
  set -e
  bk() { [ ! -e \"\$1\" ] || [ -e \"\$1.$SFX\" ] || cp -p \"\$1\" \"\$1.$SFX\"; }
  D=$GZ/earth_rover
  B=$P/build/px4_sitl_default
  A1=$P/ROMFS/px4fmu_common/init.d-posix/airframes/4024_gz_earth_rover
  A2=\$B/etc/init.d-posix/airframes/4024_gz_earth_rover
  mkdir -p \$D/materials/textures \$B/etc/init.d-posix/airframes
  bk \$D/model.sdf; bk \$D/model.config; bk /root/start_rover.sh
  bk \$A1; bk \$A2
  cp /tmp/badger_new/earth_rover/model.sdf \$D/model.sdf
  cp /tmp/badger_new/earth_rover/model.config \$D/model.config
  if [ -f /tmp/badger_new/earth_rover/materials/textures/apriltag36h11_1.png ]; then
    cp /tmp/badger_new/earth_rover/materials/textures/apriltag36h11_1.png \$D/materials/textures/apriltag36h11_1.png
  fi
  cp /tmp/badger_new/start_rover.sh /root/start_rover.sh
  chmod +x /root/start_rover.sh
  cp /tmp/badger_new/4024_gz_earth_rover \$A1
  cp /tmp/badger_new/4024_gz_earth_rover \$A2
  chmod +x \$A1 \$A2
  L=$P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
  grep -q '4024_gz_earth_rover' \$L || sed -i 's/^\t4012_gz_rover_ackermann\$/\t4012_gz_rover_ackermann\n\t4024_gz_earth_rover/' \$L
  echo \"installed \$D, /root/start_rover.sh, and airframe 4024\"
"
echo "done — restart the rover (or the locutus container) to pick this up"
