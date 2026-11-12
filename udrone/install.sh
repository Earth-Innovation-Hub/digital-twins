#!/bin/bash
# Install udrone (uDrone AUV + Kiwanis Lake) into digital-twin-aquatic.
# Run on the twin host from this folder:  C=digital-twin-aquatic bash install.sh
# Idempotent. Stops the sim if running (must be disarmed).
set -e
C=${C:-digital-twin-aquatic}
P=/root/PX4-Autopilot
B=$P/build/px4_sitl_default
SFX=bak-$(date +%Y%m%d)
HERE=$(cd "$(dirname "$0")" && pwd)

docker exec "$C" mkdir -p /tmp/udrone_new
copy_in() {
  local src=$1 dest=$2
  tr -d '\r' < "$src" > /tmp/udrone_install.$$
  docker cp /tmp/udrone_install.$$ "$C:$dest"
  rm -f /tmp/udrone_install.$$
}

copy_in "$HERE/adapter/udrone_thrust_adapter.cpp" /tmp/udrone_new/udrone_thrust_adapter.cpp
copy_in "$HERE/start_udrone.sh" /tmp/udrone_new/start_udrone.sh
copy_in "$HERE/airframe/60003_gz_xudrone" /tmp/udrone_new/60003_gz_xudrone
copy_in "$HERE/model/xudrone/model.sdf" /tmp/udrone_new/xudrone_model.sdf
copy_in "$HERE/model/xudrone/model.config" /tmp/udrone_new/xudrone_model.config
copy_in "$HERE/world/kiwanis_lake.sdf" /tmp/udrone_new/kiwanis_lake.sdf
# model dir (meshes) via tar
tar -C "$HERE/world" -cf - kiwanis_lake | docker exec -i "$C" tar -C /tmp/udrone_new -xf -
# tests
docker exec "$C" mkdir -p /root/udrone_boat/src
copy_in "$HERE/test/udrone_sinusoidal_controller.py" /tmp/udrone_new/udrone_sinusoidal_controller.py
copy_in "$HERE/test/PID.py" /tmp/udrone_new/PID.py
if [ -f "$HERE/docs/KIWANIS.md" ]; then
  copy_in "$HERE/docs/KIWANIS.md" /tmp/udrone_new/KIWANIS.md
fi

echo "=== adapter builds ==="
docker exec "$C" bash -c "
  set -e
  cd /tmp/udrone_new
  g++ -O2 -std=c++17 udrone_thrust_adapter.cpp -o udrone_thrust_adapter \$(pkg-config --cflags --libs gz-transport13 gz-msgs10)
  echo ok
"

if docker exec "$C" pgrep -x px4 >/dev/null; then
  echo "=== simulator running: require disarmed ==="
  if docker exec "$C" bash -c '$B/bin/px4-commander status 2>/dev/null' | grep -qi 'Armed'; then
    echo "vehicle is armed; disarm first"; exit 1
  fi
  echo "=== stop simulator ==="
  docker exec "$C" bash -c 'simctl stop 2>/dev/null || /root/stop_sim.sh 2>/dev/null || true' | tail -5
fi

echo "=== install ==="
docker exec "$C" bash -c "
  set -e
  bk() { [ ! -e \"\$1\" ] || [ -e \"\$1.$SFX\" ] || cp -a \"\$1\" \"\$1.$SFX\"; }
  Mdir=$P/Tools/simulation/gz/models/xudrone
  Kdir=$P/Tools/simulation/gz/models/kiwanis_lake
  W=$P/Tools/simulation/gz/worlds/kiwanis_lake.sdf
  A1=$P/ROMFS/px4fmu_common/init.d-posix/airframes/60003_gz_xudrone
  A2=$B/etc/init.d-posix/airframes/60003_gz_xudrone
  mkdir -p \$Mdir \$Kdir
  bk \$Mdir/model.sdf; bk \$Mdir/model.config
  bk \$W; bk \$A1; bk /root/udrone_thrust_adapter.cpp
  bk /usr/local/bin/udrone_thrust_adapter; bk /root/start_udrone.sh
  cp -a /tmp/udrone_new/xudrone_model.sdf \$Mdir/model.sdf
  cp -a /tmp/udrone_new/xudrone_model.config \$Mdir/model.config
  cp -a /tmp/udrone_new/kiwanis_lake.sdf \$W
  rm -rf \$Kdir
  cp -a /tmp/udrone_new/kiwanis_lake \$Kdir
  cp -a /tmp/udrone_new/60003_gz_xudrone \$A1
  mkdir -p \$(dirname \$A2)
  cp -a /tmp/udrone_new/60003_gz_xudrone \$A2
  cp -a /tmp/udrone_new/udrone_thrust_adapter.cpp /root/udrone_thrust_adapter.cpp
  cp -a /tmp/udrone_new/udrone_thrust_adapter /usr/local/bin/udrone_thrust_adapter
  chmod +x /usr/local/bin/udrone_thrust_adapter /tmp/udrone_new/start_udrone.sh
  cp -a /tmp/udrone_new/start_udrone.sh /root/start_udrone.sh
  mkdir -p /root/udrone_boat/src
  cp -a /tmp/udrone_new/udrone_sinusoidal_controller.py /root/udrone_boat/src/
  cp -a /tmp/udrone_new/PID.py /root/udrone_boat/src/
  [ -f /tmp/udrone_new/KIWANIS.md ] && cp -a /tmp/udrone_new/KIWANIS.md /root/KIWANIS.md
  # ensure airframe is listed for posix
  grep -q '60003_gz_xudrone' $P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
    || echo 'px4_add_romfs_files(60003_gz_xudrone)' >> $P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
  echo installed
"

echo "=== done. start with:  vglrun /root/start_udrone.sh ==="
