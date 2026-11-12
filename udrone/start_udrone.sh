#!/bin/bash
# uDrone AUV — auv_udrone model with otter-style thrusters. Aquatic sandbox.
set -e
export ROS_DOMAIN_ID=84
export GZ_PARTITION=aquatic
export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}

if pgrep -x px4 >/dev/null || pgrep -f "kiwanis_lake.sdf" >/dev/null; then
  echo "a simulator is already running; stop it first: simctl stop" >&2
  exit 1
fi

rm -f /root/.sim_hold
mkdir -p /root/log

if ! pgrep -x MicroXRCEAgent >/dev/null; then
  MicroXRCEAgent udp4 -p 8888 >> /root/log/uxrce84.log 2>&1 &
fi

if ! pgrep -f "^GZ_PARTITION=aquatic /usr/local/bin/udrone_thrust_adapter$" >/dev/null; then
  GZ_PARTITION=aquatic /usr/local/bin/udrone_thrust_adapter >> /root/log/udrone_thrust_adapter.log 2>&1 &
fi

# Spawn ~2 m under the Kiwanis Lake waterline (world Z=0).
cd /root/PX4-Autopilot/build/px4_sitl_default
tail -f /dev/null | env \
  PX4_UXRCE_DDS_NS=udrone \
  PX4_SYS_AUTOSTART=60003 \
  PX4_SIM_MODEL=xudrone \
  PX4_GZ_WORLD=kiwanis_lake \
  PX4_GZ_MODEL_POSE=0,0,-1.0 \
  ./bin/px4 >> /root/log/xudrone.log 2>&1
