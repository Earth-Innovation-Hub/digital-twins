#!/bin/bash
# Tempe Town Lake Heron: buoyancy, Fossen hydrodynamics, and two 20 N jets.
set -e
export ROS_DOMAIN_ID=27
export GZ_PARTITION=tempe_lake

if pgrep -x px4 >/dev/null || pgrep -f "tempe_lake.sdf" >/dev/null; then
  echo "boat simulator is already running" >&2
  exit 1
fi

rm -f /root/.sim_hold   # resume dreamslab boat-twin-watchdog.sh auto-restart

mkdir -p /root/log

if ! pgrep -x MicroXRCEAgent >/dev/null; then
  MicroXRCEAgent udp4 -p 8888 >> /root/log/uxrce27.log 2>&1 &
fi

if ! pgrep -f "^/usr/local/bin/heron_thrust_adapter$" >/dev/null; then
  /usr/local/bin/heron_thrust_adapter >> /root/log/thrust_adapter.log 2>&1 &
fi

# stdin is held open so the pxh shell does not spin and flood the log.
cd /root/PX4-Autopilot/build/px4_sitl_default
tail -f /dev/null | env PX4_UXRCE_DDS_NS=otter PX4_SIM_MODEL=gz_heron PX4_GZ_WORLD=tempe_lake PX4_GZ_MODEL_POSE=0,0,0.25 \
  /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 >> /root/log/boat.log 2>&1
