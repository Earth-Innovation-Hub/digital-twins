#!/bin/bash
# Aquatic CoRAL twin: otter_k (Heron ASV, PX4 instance 0) + xudrone docked under
# the catamaran. Same two-robot pattern as Locutus (kestrel 0 + badger 1).
# Callsign otter_k (underscore: ROS 2 names cannot contain a hyphen).
# The production Tempe otter (:40099) is a different container and untouched.
set -e
export ROS_DOMAIN_ID=84
export GZ_PARTITION=aquatic
export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}

if pgrep -x px4 >/dev/null || pgrep -f '^gz sim' >/dev/null; then
  echo "a simulator is already running; stop it first: simctl stop" >&2
  exit 1
fi

rm -f /root/.sim_hold
mkdir -p /root/log

# Helpers a previous boot may have left behind (stop_sim.sh reaps them too).
pkill -f '^/usr/local/bin/udrone_thrust_adapter' 2>/dev/null || true
pkill -f 'xudrone_camera_bridge|front_camera_link/sensor/front_camera' 2>/dev/null || true
pkill -f 'xudrone_down_camera_bridge|xudrone.*/down_camera' 2>/dev/null || true
pkill -f 'otter_k_down_camera_bridge|heron_0.*/down_camera' 2>/dev/null || true
pkill -f 'apriltag_node|tag_detector|run_tags' 2>/dev/null || true
pkill -f 'coral_behaviours|udrone_boat/behaviours/coral_behaviours' 2>/dev/null || true
sleep 1

# LinkAttacher system plugin (world SDF) needs to be on the plugin path.
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/gz-sim-8/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}

if ! pgrep -x MicroXRCEAgent >/dev/null; then
  MicroXRCEAgent udp4 -p 8888 >> /root/log/uxrce84.log 2>&1 &
fi

if ! pgrep -f '^/usr/local/bin/heron_thrust_adapter$' >/dev/null; then
  setsid nohup env GZ_PARTITION=aquatic /usr/local/bin/heron_thrust_adapter \
    >> /root/log/thrust_adapter.log 2>&1 < /dev/null &
fi

# The only place the docked AUV's adapter is started (it also refuses to run
# twice for one model).
setsid nohup env GZ_PARTITION=aquatic UDRONE_GZ_MODEL=xudrone_1 \
  /usr/local/bin/udrone_thrust_adapter \
  >> /root/log/udrone_thrust_adapter.log 2>&1 < /dev/null &

# Locutus-style secondary: waits for the world, spawns xudrone at the boat;
# the world's LinkAttacher plugin seats it in the nest and welds it.
(
  exec /root/start_udrone.sh --secondary
) >> /root/log/xudrone_secondary.log 2>&1 &

# Behaviour library: /coral/command|event|status (dock, goto, survey, helix, coverage).
# Starts after a short delay so DDS / Gazebo are up; safe to retry if early.
(
  sleep 25
  exec /usr/local/bin/coral_behaviours.sh
) >> /root/log/coral_behaviours.log 2>&1 &

# Down-camera bridges + AprilTag stacks (boat + AUV; Tempe/Locutus workflow).
(
  sleep 20
  exec /usr/local/bin/otter_k_down_camera_bridge.sh
) >> /root/log/otter_k_down_camera.log 2>&1 &
(
  sleep 28
  # Secondary AUV model is xudrone_1 when launched from the boat twin.
  GZ_MODEL_NAME=xudrone_1 exec /usr/local/bin/xudrone_down_camera_bridge.sh
) >> /root/log/xudrone_down_camera.log 2>&1 &
(
  sleep 35
  exec /usr/local/bin/coral_tags.sh otter_k
) >> /root/log/coral_tags.log 2>&1 &
(
  sleep 40
  exec /usr/local/bin/coral_tags_xudrone.sh
) >> /root/log/coral_tags_xudrone.log 2>&1 &

cd /root/PX4-Autopilot/build/px4_sitl_default
tail -f /dev/null | env \
  PX4_UXRCE_DDS_NS=otter_k \
  PX4_SIM_MODEL=gz_heron \
  PX4_GZ_WORLD=kiwanis_lake \
  PX4_GZ_MODEL_POSE=0,0,0.25 \
  ./bin/px4 >> /root/log/boat.log 2>&1
