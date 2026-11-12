#!/bin/bash
# Install otter (Heron ASV) into digital-twin-water-tempe-lake.
# Run on the twin host from this folder:  bash install.sh
# Idempotent. Refuses while the boat is armed. Stops the sim, restarts at the end (~90 s).
#
#   model/           -> PX4 Tools/simulation/gz/models/heron/
#   world/           -> PX4 Tools/simulation/gz/worlds/tempe_lake.sdf
#   airframe/        -> PX4 ROMFS + build-tree airframes
#   adapter/         -> /root + /usr/local/bin/heron_thrust_adapter
#   px4/*.py         -> patch DifferentialPosVelControl.cpp, ninja px4
#   start_boat.sh    -> /root/start_boat.sh
# Backups get suffix .bak-YYYYMMDD the first time a file is replaced that day.
set -e
C=${C:-digital-twin-water-tempe-lake}
P=/root/PX4-Autopilot
B=$P/build/px4_sitl_default
SFX=bak-$(date +%Y%m%d)
HERE=$(cd "$(dirname "$0")" && pwd)

docker exec $C mkdir -p /tmp/heron_new
copy_in() {
  local src=$1 dest=$2
  tr -d '\r' < "$src" > /tmp/heron_install.$$
  docker cp /tmp/heron_install.$$ "$C:$dest"
}
copy_in "$HERE/model/model.sdf" /tmp/heron_new/model.sdf
copy_in "$HERE/model/model.config" /tmp/heron_new/model.config
copy_in "$HERE/world/tempe_lake.sdf" /tmp/heron_new/tempe_lake.sdf
copy_in "$HERE/airframe/4023_gz_heron" /tmp/heron_new/4023_gz_heron
copy_in "$HERE/airframe/4023_gz_heron.post" /tmp/heron_new/4023_gz_heron.post
copy_in "$HERE/adapter/heron_thrust_adapter.cpp" /tmp/heron_new/heron_thrust_adapter.cpp
copy_in "$HERE/px4/px4_rover_differential_patch.py" /tmp/heron_new/px4_rover_differential_patch.py
copy_in "$HERE/start_boat.sh" /tmp/heron_new/start_boat.sh
rm -f /tmp/heron_install.$$

echo "=== the model parses, the adapter builds ==="
docker exec $C bash -c "
  set -e
  cd /tmp/heron_new
  SDF_PATH=$P/Tools/simulation/gz/models gz sdf -k model.sdf 2>&1 | grep -q '^Valid' || { echo 'model.sdf does not validate'; exit 1; }
  g++ -O2 -std=c++17 heron_thrust_adapter.cpp -o heron_thrust_adapter \$(pkg-config --cflags --libs gz-transport13 gz-msgs10)
  echo ok
"

if docker exec $C pgrep -x px4 >/dev/null; then
  echo "=== simulator is running: must be disarmed ==="
  docker exec $C bash -c "simctl status | tail -2 | head -1" | tee /tmp/heron_state.$$
  grep -q ' disarmed' /tmp/heron_state.$$ || { rm -f /tmp/heron_state.$$; echo 'boat is armed; stopping here'; exit 1; }
  rm -f /tmp/heron_state.$$
  echo "=== clear saved calibration and saved overrides of the airframe's values ==="
  docker exec $C bash -c "
    cd $B
    for p in CAL_ACC0_XOFF CAL_ACC0_YOFF CAL_ACC0_ZOFF CAL_GYRO0_XOFF CAL_GYRO0_YOFF CAL_GYRO0_ZOFF \
             CAL_MAG0_XOFF CAL_MAG0_YOFF CAL_MAG0_ZOFF CAL_MAG1_XOFF CAL_MAG1_YOFF CAL_MAG1_ZOFF \
             NAV_ACC_RAD PP_LOOKAHD_GAIN PP_LOOKAHD_MIN PP_LOOKAHD_MAX RD_TRANS_DRV_TRN RD_TRANS_TRN_DRV \
             RO_MAX_THR_SPEED RO_SPEED_LIM RO_SPEED_P RO_SPEED_I RO_ACCEL_LIM RO_DECEL_LIM RO_JERK_LIM \
             RD_WHEEL_TRACK RD_MAX_THR_YAW_R RO_YAW_RATE_LIM RO_YAW_RATE_P RO_YAW_RATE_I RO_YAW_RATE_TH RO_YAW_P \
             EKF2_GPS_DELAY EKF2_GPS_CHECK SENS_MAG_AUTOCAL SENS_IMU_AUTOCAL; do
      ./bin/px4-param reset \$p >/dev/null 2>&1 || echo \"could not reset \$p\"
    done
    ./bin/px4-param save >/dev/null 2>&1
  "
  echo "=== stop the simulator ==="
  docker exec $C simctl stop | tail -2
else
  echo "=== simulator is not running (saved PX4 parameters, if any, are left as they are) ==="
fi

echo "=== back up and install ==="
docker exec $C bash -c "
  set -e
  bk() { [ ! -e \"\$1\" ] || [ -e \"\$1.$SFX\" ] || cp -p \"\$1\" \"\$1.$SFX\"; }
  Mdir=$P/Tools/simulation/gz/models/heron
  mkdir -p \$Mdir
  A1=$P/ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_heron
  A2=$B/etc/init.d-posix/airframes/4023_gz_heron
  W=$P/Tools/simulation/gz/worlds/tempe_lake.sdf
  bk \$Mdir/model.sdf; bk \$Mdir/model.config; bk \$W
  bk \$A1; bk \$A1.post; bk /root/heron_thrust_adapter.cpp; bk /usr/local/bin/heron_thrust_adapter
  bk /root/start_boat.sh; bk $B/bin/px4
  cp /tmp/heron_new/model.sdf \$Mdir/model.sdf
  cp /tmp/heron_new/model.config \$Mdir/model.config
  cp /tmp/heron_new/tempe_lake.sdf \$W
  cp /tmp/heron_new/4023_gz_heron \$A1;           chmod +x \$A1
  cp /tmp/heron_new/4023_gz_heron.post \$A1.post; chmod +x \$A1.post
  mkdir -p $B/etc/init.d-posix/airframes
  cp \$A1 \$A2; cp \$A1.post \$A2.post; chmod +x \$A2 \$A2.post
  cp /tmp/heron_new/heron_thrust_adapter.cpp /root/heron_thrust_adapter.cpp
  cp /tmp/heron_new/heron_thrust_adapter /usr/local/bin/heron_thrust_adapter
  cp /tmp/heron_new/start_boat.sh /root/start_boat.sh; chmod +x /root/start_boat.sh
  L=$P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
  grep -q '^\s*4023_gz_heron\$' \$L || sed -i 's/^\t4021_gz_x500_flow\$/\t4021_gz_x500_flow\n\t4023_gz_heron/' \$L
  grep -q '4023_gz_heron.post' \$L || sed -i 's/^\t4023_gz_heron\$/\t4023_gz_heron\n\t4023_gz_heron.post/' \$L
  grep -c '4023_gz_heron' \$L | sed 's/^/airframe list entries: /'
  echo '--- PX4 rover_differential'
  python3 /tmp/heron_new/px4_rover_differential_patch.py
  cd $B && ninja px4 2>&1 | tail -n 2
  cp /tmp/heron_new/4023_gz_heron \$A2;           chmod +x \$A2
  cp /tmp/heron_new/4023_gz_heron.post \$A2.post; chmod +x \$A2.post
"

echo "=== start ==="
docker exec $C simctl start | tail -3
sleep 25
docker exec $C simctl status
echo "=== checks ==="
docker exec $C bash -c "
  cd $B
  ./bin/px4-gyro_calibration status 2>&1 | head -1 | sed 's/^/online gyro calibration (want: not running): /'
  saved=\$(./bin/px4-param show -c 2>/dev/null | grep -E '^[x ] [+*] (RO_|RD_|PP_|NAV_ACC|CAL_[A-Z]+[0-9]_[XYZ]OFF|SENS_(MAG|IMU)_AUTOCAL|EKF2_GPS)' | sed 's/ \[[0-9,]*\]//')
  echo \"saved parameters this airframe owns (want: none): \${saved:-none}\"
  echo 'in use:'
  for p in NAV_ACC_RAD RO_JERK_LIM PP_LOOKAHD_MIN EKF2_GPS_DELAY EKF2_GPS_CHECK SIM_GZ_WH_MAX1; do ./bin/px4-param show \$p 2>/dev/null | grep -E '^[x +*] ' | sed 's/ \[[0-9,]*\]//'; done
  grep -c 'heron-twin' $P/src/modules/rover_differential/DifferentialPosVelControl/DifferentialPosVelControl.cpp | sed 's/^/PX4 source changes present (want 2): /'
"
